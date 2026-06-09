"""
FrustumNode — ROS 2 node for Frustum-based 3D object detection.

Pipeline per LiDAR scan
-----------------------
  1. Receive /painting/scored_cloud  — LiDAR points with per-class PP scores
                                       [x, y, z, intensity, ring, s_ped, s_car, s_cyc]
  2. Receive /blackfly_s/cam0/image_rectified — camera frame (latest-cache)
  3. Run YOLO segmentation on the camera frame (if not already cached for this stamp)
  4. Run FrustumDetector.detect() with the scored cloud → raw detections
  5. Apply NMS
  6. Update AB3DMOT tracker
  7. Publish:
       /frustum/markers      (visualization_msgs/MarkerArray) — 3D bounding boxes
       /frustum/bev          (sensor_msgs/Image)              — BEV + camera panel
       /frustum/debug        (std_msgs/String)                — detection counts

Subscribed topics
-----------------
  /painting/scored_cloud          (sensor_msgs/PointCloud2)
  /blackfly_s/cam0/image_rectified (sensor_msgs/Image)

Parameters
----------
  calib_file        (str)   — path to KITTI-format calib.txt
  checkpoint_path   (str)   — optional YOLO model file path
  conf_thr          (float) — YOLO confidence threshold (default: 0.40)
  min_pts           (int)   — min LiDAR points inside frustum (default: 4)
  max_age           (int)   — tracker max_age (default: 3)
  min_hits          (int)   — tracker min_hits to confirm (default: 3)
  nms_dist          (float) — NMS BEV distance threshold (default: 1.0)
"""

import sys
import os

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2 as pc2

# Shared frustum_detection library (lives at repo root, added to sys.path by entrypoint)
_WS = os.environ.get('COLCON_PREFIX_PATH', '').split(':')[0]
_REPO_ROOT = os.path.join(_WS, '..', 'src', 'AI-Based-Data-Fusion')
for _p in [
    '/workspace/AI-Based-Data-Fusion',
    '/workspace',
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..'),
]:
    if os.path.isdir(os.path.join(_p, 'frustum_detection')):
        sys.path.insert(0, _p)
        break

from frustum_detection import (
    FrustumDetector, Detection3D, nms_3d,
    AB3DMOT, TrackedObject, draw_combined, CLASS_COLORS_BGR,
)
from perception_framework.lidar_to_image_projection import KittiLidarToImageProjector


class FrustumNode(Node):
    """
    ROS 2 node — Frustum-based 3D detection using PointPainting-scored clouds.

    Subscribes to the /painting/scored_cloud topic published by PaintingNode and
    the rectified camera image. For each incoming LiDAR scan it:
      1. Re-runs YOLO on the latest camera frame (cached per unique stamp)
      2. Passes scored cloud + YOLO results to FrustumDetector
      3. Publishes 3D bounding boxes as MarkerArray + BEV image
    """

    def __init__(self):
        super().__init__('frustum_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('calib_file',      '')
        self.declare_parameter('checkpoint_path', '')
        self.declare_parameter('conf_thr',        0.40)
        self.declare_parameter('min_pts',         4)
        self.declare_parameter('max_age',         3)
        self.declare_parameter('min_hits',        3)
        self.declare_parameter('nms_dist',        1.0)

        calib_file    = self.get_parameter('calib_file').value
        checkpoint    = self.get_parameter('checkpoint_path').value
        conf_thr      = self.get_parameter('conf_thr').value
        self._conf_thr = conf_thr
        min_pts       = self.get_parameter('min_pts').value
        max_age       = self.get_parameter('max_age').value
        min_hits      = self.get_parameter('min_hits').value
        self._nms_dist = self.get_parameter('nms_dist').value

        # ── Projector ─────────────────────────────────────────────────────────
        if calib_file:
            self._projector = KittiLidarToImageProjector(calib_file)
            self.get_logger().info(f'Calibration loaded: {calib_file}')
        else:
            self._projector = None
            self.get_logger().warn(
                'No calib_file parameter — frustum detection will not run. '
                'Pass: --ros-args -p calib_file:=/path/to/calib.txt'
            )

        # ── YOLO model ────────────────────────────────────────────────────────
        self._yolo_model = None
        try:
            from point_painting.segmentation.yolo_segmentation import load_model
            self._yolo_model = load_model(checkpoint if checkpoint else None)
            self.get_logger().info(
                f'YOLO model loaded: {self._yolo_model.ckpt_path}'
            )
        except Exception as e:
            self.get_logger().error(f'Failed to load YOLO model: {e}')

        # ── FrustumDetector + Tracker ─────────────────────────────────────────
        self._detector = FrustumDetector(
            conf_thr=conf_thr, min_pts=min_pts, use_dbscan=True
        )
        self._tracker = AB3DMOT(
            iou_threshold=0.25, max_age=max_age, min_hits=min_hits
        )
        self.get_logger().info(
            f'FrustumDetector: conf={conf_thr}, min_pts={min_pts}'
        )
        self.get_logger().info(
            f'AB3DMOT: max_age={max_age}, min_hits={min_hits}'
        )

        # ── State ─────────────────────────────────────────────────────────────
        self._bridge           = CvBridge()
        self._img_queue        = []
        self._cached_stamp     = None
        self._cached_yolo      = None       # YOLO results for cached stamp
        self._cached_cv_bgr    = None       # OpenCV BGR image for visualisation
        self._frame_count      = 0

        # ── Publishers ────────────────────────────────────────────────────────
        self._marker_pub = self.create_publisher(
            MarkerArray, '/frustum/markers', 10)
        self._bev_pub    = self.create_publisher(
            Image, '/frustum/bev', 10)
        self._debug_pub  = self.create_publisher(
            String, '/frustum/debug', 10)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Image, '/blackfly_s/cam0/image_rectified', self._img_cb, 10)
        self.create_subscription(
            PointCloud2, '/painting/scored_cloud', self._cloud_cb, 10)

        self.get_logger().info('FrustumNode started — waiting for scored cloud...')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _img_cb(self, msg: Image) -> None:
        """Cache incoming camera frames in a queue to allow timestamp synchronization."""
        self._img_queue.append(msg)
        if len(self._img_queue) > 50:
            self._img_queue.pop(0)

    def _cloud_cb(self, msg: PointCloud2) -> None:
        """
        Main callback: receive scored cloud, run FrustumDetector, publish results.
        """
        if self._projector is None or self._yolo_model is None:
            return
        if not self._img_queue:
            return

        # Find the image frame with the closest timestamp to the cloud message's stamp.
        # Use the closest match unconditionally — the sensors may run on different
        # hardware clocks with a large offset, so a hard cutoff would drop everything.
        t_cloud = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        best_img = None
        best_diff = float('inf')
        for img in self._img_queue:
            t_img = img.header.stamp.sec + img.header.stamp.nanosec * 1e-9
            diff = abs(t_cloud - t_img)
            if diff < best_diff:
                best_diff = diff
                best_img = img

        if best_img is None:
            return

        img_msg = best_img
        img_stamp = (img_msg.header.stamp.sec, img_msg.header.stamp.nanosec)

        # ── Run YOLO only when image is new ───────────────────────────────────
        if self._cached_stamp != img_stamp:
            cv_image = self._bridge.imgmsg_to_cv2(
                img_msg, desired_encoding='passthrough')
            # Convert to BGR for visualisation
            if cv_image.ndim == 2:
                cv_bgr = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
            else:
                cv_bgr = cv_image[..., ::-1].copy()   # RGB → BGR
            img_rgb = cv_image[..., ::-1] if cv_image.ndim == 3 else \
                      cv2.cvtColor(cv_image, cv2.COLOR_GRAY2RGB)

            h, w = img_rgb.shape[:2]
            yolo_results = self._yolo_model(
                img_rgb, verbose=False, conf=self._conf_thr, imgsz=(h, w))

            self._cached_stamp  = img_stamp
            self._cached_yolo   = yolo_results
            self._cached_cv_bgr = cv_bgr

        yolo_results = self._cached_yolo
        cv_bgr       = self._cached_cv_bgr
        img_shape    = cv_bgr.shape[:2]   # (H, W)

        # ── Read scored cloud ─────────────────────────────────────────────────
        field_names = ('x', 'y', 'z', 'intensity', 'ring', 's_ped', 's_car', 's_cyc')
        raw = list(pc2.read_points(msg, field_names=field_names, skip_nans=True))
        if not raw:
            return

        import numpy.lib.recfunctions as rfn
        scored = rfn.structured_to_unstructured(np.array(raw), dtype=np.float32)   # (N, 8)
        lidar_filtered = scored                     # FrustumDetector uses cols 0-2

        # ── Detect ────────────────────────────────────────────────────────────
        dets = self._detector.detect(
            lidar_filtered=lidar_filtered,
            yolo_results=yolo_results,
            projector=self._projector,
            img_shape=img_shape,
            scored_cloud=scored,
        )
        dets = nms_3d(dets, dist_thr=self._nms_dist)

        # ── Track ─────────────────────────────────────────────────────────────
        tracked = self._tracker.update(dets)

        # ── Publish ───────────────────────────────────────────────────────────
        self._publish_markers(dets, tracked, msg.header)
        self._frame_count += 1
        if self._frame_count % 5 == 0:
            self._publish_bev(cv_bgr, dets, tracked, img_msg.header)

        debug_msg = String()
        debug_msg.data = (
            f'frame={self._frame_count} '
            f'dets={len(dets)} tracked={len(tracked)}'
        )
        self._debug_pub.publish(debug_msg)

        if self._frame_count % 50 == 0:
            self.get_logger().info(debug_msg.data)

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_markers(
        self,
        dets: list,
        tracked: list,
        header,
    ) -> None:
        """
        Publish 3D bounding boxes as visualization_msgs/MarkerArray.

        Each detection gets a LINE_LIST marker (wireframe box) coloured by class.
        A DELETE_ALL marker is prepended to clear stale markers from the previous frame.
        """
        marker_array = MarkerArray()

        # Clear previous markers
        clear = Marker()
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        _EDGES = [
            (0,1),(1,2),(2,3),(3,0),
            (4,5),(5,6),(6,7),(7,4),
            (0,4),(1,5),(2,6),(3,7),
        ]

        def _corners(det: Detection3D):
            c, s = np.cos(det.heading), np.sin(det.heading)
            hl, hw, hh = det.dx/2, det.dy/2, det.dz/2
            loc = np.array([
                [ hl, hw,-hh],[ hl,-hw,-hh],[-hl,-hw,-hh],[-hl, hw,-hh],
                [ hl, hw, hh],[ hl,-hw, hh],[-hl,-hw, hh],[-hl, hw, hh],
            ], dtype=np.float32)
            R = np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]], dtype=np.float32)
            return (R @ loc.T).T + np.array([det.x, det.y, det.z])

        for idx, det in enumerate(dets):
            bgr = CLASS_COLORS_BGR.get(det.cls_id, (200, 200, 200))
            r, g, b = bgr[2]/255., bgr[1]/255., bgr[0]/255.

            m = Marker()
            m.header = header
            m.ns     = 'frustum_dets'
            m.id     = idx
            m.type   = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = 0.05
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.9

            corners = _corners(det)
            for i, j in _EDGES:
                from geometry_msgs.msg import Point
                p0 = Point(x=float(corners[i,0]),
                           y=float(corners[i,1]),
                           z=float(corners[i,2]))
                p1 = Point(x=float(corners[j,0]),
                           y=float(corners[j,1]),
                           z=float(corners[j,2]))
                m.points.extend([p0, p1])

            marker_array.markers.append(m)

        # Publish tracked boxes and labels
        for t in tracked:
            cx, cy, cz, dx, dy, dz, heading = t.box7
            bgr = CLASS_COLORS_BGR.get(t.cls_id, (200, 200, 200))
            r, g, b = bgr[2]/255., bgr[1]/255., bgr[0]/255.

            # 3D Box marker
            m = Marker()
            m.header = header
            m.ns     = 'frustum_tracks'
            m.id     = t.track_id
            m.type   = Marker.LINE_LIST
            m.action = Marker.ADD
            m.scale.x = 0.08  # Thicker than raw detections
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.95

            # Calculate corners of the tracked box
            c_yaw, s_yaw = np.cos(heading), np.sin(heading)
            hl, hw, hh = dx/2, dy/2, dz/2
            loc = np.array([
                [ hl, hw,-hh],[ hl,-hw,-hh],[-hl,-hw,-hh],[-hl, hw,-hh],
                [ hl, hw, hh],[ hl,-hw, hh],[-hl,-hw, hh],[-hl, hw, hh],
            ], dtype=np.float32)
            R = np.array([[c_yaw,-s_yaw,0.],[s_yaw,c_yaw,0.],[0.,0.,1.]], dtype=np.float32)
            corners = (R @ loc.T).T + np.array([cx, cy, cz])

            for i, j in _EDGES:
                from geometry_msgs.msg import Point
                p0 = Point(x=float(corners[i,0]),
                           y=float(corners[i,1]),
                           z=float(corners[i,2]))
                p1 = Point(x=float(corners[j,0]),
                           y=float(corners[j,1]),
                           z=float(corners[j,2]))
                m.points.extend([p0, p1])

            marker_array.markers.append(m)

            # Text label marker above the box
            ml = Marker()
            ml.header = header
            ml.ns     = 'frustum_track_labels'
            ml.id     = t.track_id
            ml.type   = Marker.TEXT_VIEW_FACING
            ml.action = Marker.ADD
            ml.pose.position.x = float(cx)
            ml.pose.position.y = float(cy)
            ml.pose.position.z = float(cz + dz/2 + 0.5)
            ml.scale.z = 0.6  # Text size
            ml.color.r = 1.0  # Bright yellow/white for labels
            ml.color.g = 1.0
            ml.color.b = 0.0
            ml.color.a = 1.0
            ml.text    = f"{t.cls_name} ID:{t.track_id}"

            marker_array.markers.append(ml)

        self._marker_pub.publish(marker_array)

    def _publish_bev(
        self,
        cv_bgr: np.ndarray,
        dets: list,
        tracked: list,
        header,
    ) -> None:
        """Publish combined camera + BEV panel as an image."""
        panel = draw_combined(
            cv_bgr, dets, self._projector, tracked,
            range_m=60.0, bev_size=700,
        )
        bev_msg = self._bridge.cv2_to_imgmsg(panel, encoding='bgr8')
        bev_msg.header = header
        self._bev_pub.publish(bev_msg)


def main(args=None):
    """Entry point — initialise ROS 2 and spin the frustum node."""
    rclpy.init(args=args)
    node = FrustumNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
