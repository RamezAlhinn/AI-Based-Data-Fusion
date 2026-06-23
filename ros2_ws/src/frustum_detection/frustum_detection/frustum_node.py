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
        /frustum/diagnostics  (diagnostic_msgs/DiagnosticArray)— heartbeat / pipeline health (1 Hz)
        /frustum/yolo_detection (sensor_msgs/Image)            — YOLO boxes drawn on camera image

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
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2 as pc2
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from perception_msgs.msg import TrackedObject as TrackedObjectMsg, TrackedObjectArray

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


# ── Mock YOLO wrappers to feed deserialized JSON results to FrustumDetector ────
class MockTensor:
    def __init__(self, data):
        self._data = data
    def cpu(self):
        return self
    def numpy(self):
        return self._data

class MockBoxes:
    def __init__(self, cls_list, conf_list):
        self.cls = MockTensor(np.array(cls_list))
        self.conf = MockTensor(np.array(conf_list))

class MockMasks:
    def __init__(self, masks_list):
        self.data = MockTensor(np.array(masks_list))

class MockYoloResult:
    def __init__(self, masks_list, cls_list, conf_list):
        self.masks = MockMasks(masks_list) if masks_list else None
        self.boxes = MockBoxes(cls_list, conf_list)


class FrustumNode(Node):
    """
    ROS 2 node — Frustum-based 3D detection using PointPainting-scored clouds.

    Subscribes to the /painting/scored_cloud and /painting/yolo_results topics
    published by PaintingNode, and the rectified camera image. For each incoming
    LiDAR scan it:
      1. Matches the camera image and the YOLO results using the exact timestamp
      2. Reconstructs YOLO detections and passes them to FrustumDetector
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

        # ── YOLO model (Skipped in Optimized pipeline since YOLO runs in PaintingNode only) ──
        self._yolo_model = None
        self.get_logger().info('Optimized pipeline: YOLO model loading skipped in FrustumNode.')

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
        self._latest_img_msg   = None
        self._yolo_buffer      = {}         # stamp -> list of instance dicts
        self._img_buffer       = {}         # stamp -> Image msg
        self._frame_count      = 0
        self._trajectories     = {}         # track_id -> list of (x, y, z)

        # ── Runtime stats for heartbeat ───────────────────────────────────────
        self._start_time      = time.monotonic()
        self._last_frame_time = None
        self._last_det_count  = 0
        self._last_trk_count  = 0
        self._fps_estimate    = 0.0
        self._last_tracked    = []          # list[TrackedObject] from latest frame

        # ── Publishers ────────────────────────────────────────────────────────
        self._marker_pub = self.create_publisher(
            MarkerArray, '/frustum/markers', 10)
        self._bev_pub    = self.create_publisher(
            Image, '/frustum/bev', 10)
        self._debug_pub  = self.create_publisher(
            String, '/frustum/debug', 10)
        self._diag_pub   = self.create_publisher(
            DiagnosticArray, '/frustum/diagnostics', 10)
        self._objects_pub = self.create_publisher(
            TrackedObjectArray, '/frustum/objects', 10)
        self._traj_pub = self.create_publisher(
            MarkerArray, '/frustum/trajectories', 10)
        self._arrow_pub = self.create_publisher(
            MarkerArray, '/frustum/velocity_arrows', 10)

        # 1 Hz diagnostics heartbeat timer
        self.create_timer(1.0, self._publish_diagnostics)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Image, '/blackfly_s/cam0/image_rectified', self._img_cb, 10)
        self.create_subscription(
            PointCloud2, '/painting/scored_cloud', self._cloud_cb, 10)
        self.create_subscription(
            String, '/painting/yolo_results', self._yolo_results_cb, 10)

        self.get_logger().info('FrustumNode started — waiting for scored cloud...')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _img_cb(self, msg: Image) -> None:
        """Cache the latest camera frame and add it to the buffer."""
        self._latest_img_msg = msg
        stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self._img_buffer[stamp] = msg
        
        # Prune buffer to keep it small (e.g. last 50 images)
        if len(self._img_buffer) > 50:
            oldest_stamp = min(self._img_buffer.keys(), key=lambda s: s[0] + s[1]*1e-9)
            self._img_buffer.pop(oldest_stamp, None)

    def _yolo_results_cb(self, msg: String) -> None:
        """Receive serialized YOLO results from PaintingNode."""
        import json
        try:
            payload = json.loads(msg.data)
            sec = payload['stamp']['sec']
            nanosec = payload['stamp']['nanosec']
            stamp = (sec, nanosec)
            self._yolo_buffer[stamp] = payload['instances']
            
            # Prune buffer to keep it small (e.g. last 50 entries)
            if len(self._yolo_buffer) > 50:
                oldest_stamp = min(self._yolo_buffer.keys(), key=lambda s: s[0] + s[1]*1e-9)
                self._yolo_buffer.pop(oldest_stamp, None)
        except Exception as e:
            self.get_logger().error(f'Failed to parse YOLO JSON results: {e}')

    def _cloud_cb(self, msg: PointCloud2) -> None:
        """
        Main callback: receive scored cloud, retrieve matched YOLO results and image,
        run FrustumDetector, and publish results.
        """
        if self._projector is None:
            return

        cloud_stamp = (msg.header.stamp.sec, msg.header.stamp.nanosec)

        # ── Retrieve matched image from buffer ────────────────────────────────
        if cloud_stamp in self._img_buffer:
            img_msg = self._img_buffer[cloud_stamp]
        else:
            # Fallback to closest image in buffer
            t_cloud = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            best_stamp = None
            best_diff = float('inf')
            for stamp, img in self._img_buffer.items():
                t_img = stamp[0] + stamp[1] * 1e-9
                diff = abs(t_cloud - t_img)
                if diff < best_diff:
                    best_diff = diff
                    best_stamp = stamp
            if best_stamp is not None and best_diff < 0.2:
                img_msg = self._img_buffer[best_stamp]
            else:
                if self._latest_img_msg is None:
                    return
                img_msg = self._latest_img_msg

        # ── Retrieve matched YOLO results from buffer ─────────────────────────
        instances = []
        if cloud_stamp in self._yolo_buffer:
            instances = self._yolo_buffer[cloud_stamp]
        else:
            # Fallback to closest YOLO results in buffer
            t_cloud = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            best_stamp = None
            best_diff = float('inf')
            for stamp, insts in self._yolo_buffer.items():
                t_yolo = stamp[0] + stamp[1] * 1e-9
                diff = abs(t_cloud - t_yolo)
                if diff < best_diff:
                    best_diff = diff
                    best_stamp = stamp
            if best_stamp is not None and best_diff < 0.2:
                instances = self._yolo_buffer[best_stamp]

        # Convert img_msg to OpenCV
        cv_image = self._bridge.imgmsg_to_cv2(
            img_msg, desired_encoding='passthrough')
        # Convert to BGR for visualisation
        if cv_image.ndim == 2:
            cv_bgr = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        else:
            cv_bgr = cv_image[..., ::-1].copy()   # RGB → BGR
        h, w = cv_image.shape[:2]
        img_shape = (h, w)

        # ── Reconstruct YOLO results using Mock wrapper ───────────────────────
        masks_list = []
        cls_list = []
        conf_list = []
        for inst in instances:
            cls_id = inst['class']
            conf = inst['conf']
            poly = inst['polygon']
            
            mask = np.zeros((h, w), dtype=np.float32)
            if len(poly) > 0:
                poly_np = np.array(poly, dtype=np.int32)
                cv2.fillPoly(mask, [poly_np], 1.0)
            
            masks_list.append(mask)
            cls_list.append(cls_id)
            conf_list.append(conf)

        if masks_list:
            yolo_results = [MockYoloResult(masks_list, cls_list, conf_list)]
        else:
            yolo_results = []

        # ── Read scored cloud ─────────────────────────────────────────────────
        field_names = ('x', 'y', 'z', 'intensity', 'ring', 's_ped', 's_car', 's_cyc')
        raw = list(pc2.read_points(msg, field_names=field_names, skip_nans=True))
        if not raw:
            return

        raw_arr = np.array(raw)
        scored = raw_arr.view((np.float32, len(field_names)))   # (N, 8)
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

        # ── Update runtime stats for diagnostics ──────────────────────────────
        now = time.monotonic()
        if self._last_frame_time is not None:
            dt = now - self._last_frame_time
            self._fps_estimate = 0.8 * self._fps_estimate + 0.2 * (1.0 / max(dt, 1e-3))
        self._last_frame_time = now
        self._last_det_count  = len(dets)
        self._last_trk_count  = len(tracked)
        self._last_tracked    = tracked     # snapshot for heartbeat

        # ── Update trajectory history ─────────────────────────────────────────
        for t in tracked:
            tid = t.track_id
            cx, cy, cz = float(t.box7[0]), float(t.box7[1]), float(t.box7[2])
            if tid not in self._trajectories:
                self._trajectories[tid] = []
            self._trajectories[tid].append((cx, cy, cz))

        # ── Publish ───────────────────────────────────────────────────────────
        self._publish_markers(dets, tracked, msg.header)
        self._publish_objects(tracked, msg.header)
        self._publish_trajectories(tracked, msg.header)
        self._publish_velocity_arrows(tracked, msg.header)
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
        Publish 3D bounding boxes + track IDs as visualization_msgs/MarkerArray.

        Namespaces:
          'frustum_dets'    — thin wireframe boxes for raw detections (blue-ish)
          'frustum_tracks'  — thick wireframe boxes for confirmed tracks
          'frustum_labels'  — TEXT_VIEW_FACING markers: class name + track ID

        A DELETE_ALL marker with the correct header is prepended each frame to
        clear stale markers from the previous frame.
        """
        # Ensure markers are published in the LiDAR frame so Foxglove can place
        # them correctly in 3D space.  The scored-cloud header carries the
        # LiDAR frame_id (typically 'velodyne').
        marker_array = MarkerArray()

        # DELETE_ALL must carry a valid header; ns='' means "all namespaces".
        clear = Marker()
        clear.header    = header
        clear.ns        = ''
        clear.id        = 0
        clear.action    = Marker.DELETEALL
        marker_array.markers.append(clear)

        _EDGES = [
            (0,1),(1,2),(2,3),(3,0),
            (4,5),(5,6),(6,7),(7,4),
            (0,4),(1,5),(2,6),(3,7),
        ]

        def _corners_det(det: Detection3D) -> np.ndarray:
            c, s = np.cos(det.heading), np.sin(det.heading)
            hl, hw, hh = det.dx/2, det.dy/2, det.dz/2
            loc = np.array([
                [ hl,  hw, -hh], [ hl, -hw, -hh], [-hl, -hw, -hh], [-hl,  hw, -hh],
                [ hl,  hw,  hh], [ hl, -hw,  hh], [-hl, -hw,  hh], [-hl,  hw,  hh],
            ], dtype=np.float32)
            R = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]], dtype=np.float32)
            return (R @ loc.T).T + np.array([det.x, det.y, det.z])

        def _corners_track(t) -> np.ndarray:
            cx, cy, cz, dx, dy, dz, heading = t.box7
            c, s = np.cos(heading), np.sin(heading)
            hl, hw, hh = dx/2, dy/2, dz/2
            loc = np.array([
                [ hl,  hw, -hh], [ hl, -hw, -hh], [-hl, -hw, -hh], [-hl,  hw, -hh],
                [ hl,  hw,  hh], [ hl, -hw,  hh], [-hl, -hw,  hh], [-hl,  hw,  hh],
            ], dtype=np.float32)
            R = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]], dtype=np.float32)
            return (R @ loc.T).T + np.array([cx, cy, cz])

        def _wire_marker(ns: str, mid: int, corners: np.ndarray,
                         r: float, g: float, b: float,
                         thickness: float = 0.05) -> Marker:
            m = Marker()
            m.header  = header
            m.ns      = ns
            m.id      = mid
            m.type    = Marker.LINE_LIST
            m.action  = Marker.ADD
            m.scale.x = thickness
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 1.0
            for i, j in _EDGES:
                m.points.append(Point(x=float(corners[i, 0]),
                                      y=float(corners[i, 1]),
                                      z=float(corners[i, 2])))
                m.points.append(Point(x=float(corners[j, 0]),
                                      y=float(corners[j, 1]),
                                      z=float(corners[j, 2])))
            return m

        # ── Raw detection boxes (thin, dimmer) ────────────────────────────────
        for idx, det in enumerate(dets):
            bgr = CLASS_COLORS_BGR.get(det.cls_id, (200, 200, 200))
            r, g, b = bgr[2]/255. * 0.5, bgr[1]/255. * 0.5, bgr[0]/255. * 0.5
            corners = _corners_det(det)
            marker_array.markers.append(
                _wire_marker('frustum_dets', idx, corners, r, g, b, thickness=0.04)
            )

        # ── Confirmed track boxes (green=high, red=low) + text labels ───────────
        for t in tracked:
            conf = float(t.score) if t.score > 0 else 0.0
            conf = max(0.0, min(1.0, conf))
            if conf >= 0.6:
                r, g, b = 0.0, 1.0, 0.0   # green — high confidence
                conf_label = 'High'
            else:
                r, g, b = 1.0, 0.0, 0.0   # red — low confidence
                conf_label = 'Low'
            corners = _corners_track(t)

            # Thick wireframe box
            marker_array.markers.append(
                _wire_marker('frustum_tracks', t.track_id, corners,
                             r, g, b, thickness=0.08)
            )

            # Text label above the box: "Car ID:3  0.89 High"
            cx, cy, cz = float(t.box7[0]), float(t.box7[1]), float(t.box7[2])
            dz         = float(t.box7[5])
            txt = Marker()
            txt.header   = header
            txt.ns       = 'frustum_labels'
            txt.id       = t.track_id
            txt.type     = Marker.TEXT_VIEW_FACING
            txt.action   = Marker.ADD
            txt.pose.position.x = cx
            txt.pose.position.y = cy
            txt.pose.position.z = cz + dz / 2.0 + 0.4
            txt.pose.orientation.w = 1.0
            txt.scale.z  = 0.5
            txt.color.r  = r
            txt.color.g  = g
            txt.color.b  = b
            txt.color.a  = 1.0
            score_str = f' {t.score:.2f} {conf_label}' if t.score > 0 else ''
            txt.text  = f'{t.cls_name} ID:{t.track_id}{score_str}'
            marker_array.markers.append(txt)

        self._marker_pub.publish(marker_array)

    def _publish_objects(self, tracked: list, header) -> None:
        """Publish confirmed tracks as a perception_msgs/TrackedObjectArray."""
        array_msg = TrackedObjectArray()
        array_msg.header = header
        
        for t in tracked:
            obj_msg = TrackedObjectMsg()
            obj_msg.track_id = int(t.track_id)
            obj_msg.class_name = str(t.cls_name)
            obj_msg.class_id = int(t.cls_id)
            
            # Position
            obj_msg.position.x = float(t.box7[0])
            obj_msg.position.y = float(t.box7[1])
            obj_msg.position.z = float(t.box7[2])
            
            # Dimensions
            obj_msg.size.x = float(t.box7[3])
            obj_msg.size.y = float(t.box7[4])
            obj_msg.size.z = float(t.box7[5])
            
            # Yaw angle
            obj_msg.heading = float(t.box7[6])
            
            # Confidence
            obj_msg.confidence = float(t.score)
            
            # Additional tracking parameters
            obj_msg.is_predicted = bool(t.is_predicted)
            
            # Velocity: vx, vy, vz
            obj_msg.velocity.x = float(t.velocity[0])
            obj_msg.velocity.y = float(t.velocity[1])
            obj_msg.velocity.z = float(t.velocity[2])
            
            array_msg.objects.append(obj_msg)
            
        self._objects_pub.publish(array_msg)

    def _publish_trajectories(self, tracked: list, header) -> None:
        """Publish LINE_STRIP markers showing the path each track has taken."""
        ma = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.ns     = 'frustum_trajectories'
        clear.id     = 0
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for tid, pts in self._trajectories.items():
            if len(pts) < 2:
                continue
            # Each track gets a distinct fixed colour (white/cyan/magenta/orange/purple)
            _TRAJ_COLOURS = [
                (1.0, 1.0, 1.0),   # white
                (0.0, 1.0, 1.0),   # cyan
                (1.0, 0.0, 1.0),   # magenta
                (1.0, 0.6, 0.0),   # orange
                (0.5, 0.0, 1.0),   # purple
            ]
            r, g, b = _TRAJ_COLOURS[tid % len(_TRAJ_COLOURS)]

            m = Marker()
            m.header  = header
            m.ns      = 'frustum_trajectories'
            m.id      = tid
            m.type    = Marker.LINE_STRIP
            m.action  = Marker.ADD
            m.scale.x = 0.06
            m.color.r = r
            m.color.g = g
            m.color.b = b
            m.color.a = 0.8
            for px, py, pz in pts:
                m.points.append(Point(x=px, y=py, z=pz))
            ma.markers.append(m)

        self._traj_pub.publish(ma)

    def _publish_velocity_arrows(self, tracked: list, header) -> None:
        """Publish ARROW markers showing Kalman filter velocity per track."""
        ma = MarkerArray()
        clear = Marker()
        clear.header = header
        clear.ns     = 'frustum_velocity'
        clear.id     = 0
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        for t in tracked:
            vx, vy, vz = float(t.velocity[0]), float(t.velocity[1]), float(t.velocity[2])
            speed = (vx**2 + vy**2 + vz**2) ** 0.5
            if speed < 0.05:
                continue

            cx, cy, cz = float(t.box7[0]), float(t.box7[1]), float(t.box7[2])
            scale = min(speed * 1.5, 4.0)   # cap arrow length at 4 m

            m = Marker()
            m.header  = header
            m.ns      = 'frustum_velocity'
            m.id      = t.track_id
            m.type    = Marker.ARROW
            m.action  = Marker.ADD
            m.scale.x = scale           # arrow length
            m.scale.y = 0.15            # shaft diameter
            m.scale.z = 0.25            # head diameter
            m.color.r = 0.2
            m.color.g = 0.8
            m.color.b = 1.0
            m.color.a = 0.9
            # Arrow defined by two points: tail → tip
            tail = Point(x=cx, y=cy, z=cz)
            tip  = Point(x=cx + vx / max(speed, 1e-3) * scale,
                         y=cy + vy / max(speed, 1e-3) * scale,
                         z=cz + vz / max(speed, 1e-3) * scale)
            m.points.append(tail)
            m.points.append(tip)
            ma.markers.append(m)

        self._arrow_pub.publish(ma)

    # ── Diagnostics heartbeat (1 Hz) ──────────────────────────────────────────

    def _publish_diagnostics(self) -> None:
        """
        Publish a DiagnosticArray heartbeat every second.
        Visible in Foxglove Studio → Diagnostics panel.
        Allows the professor / team to distinguish visualization issues
        from actual pipeline failures.
        """
        uptime_s = time.monotonic() - self._start_time

        # Overall health level
        if self._fps_estimate > 0.1 or self._frame_count == 0:
            level   = DiagnosticStatus.OK
            message = 'Pipeline running'
        else:
            level   = DiagnosticStatus.WARN
            message = 'No frames received — check bag or painting_node'

        status = DiagnosticStatus()
        status.level   = level
        status.name    = 'FrustumNode / Tracker'
        status.message = message
        status.hardware_id = 'frustum_node'
        status.values  = [
            KeyValue(key='uptime_s',         value=f'{uptime_s:.1f}'),
            KeyValue(key='frames_total',     value=str(self._frame_count)),
            KeyValue(key='fps_estimate',     value=f'{self._fps_estimate:.2f}'),
            KeyValue(key='last_dets',        value=str(self._last_det_count)),
            KeyValue(key='last_tracks',      value=str(self._last_trk_count)),
            KeyValue(key='tracker_max_age',  value=str(self._tracker.max_age)),
            KeyValue(key='tracker_min_hits', value=str(self._tracker.min_hits)),
            KeyValue(key='tracker_iou_thr',  value=str(self._tracker.iou_thr)),
            KeyValue(key='active_tracks',    value=str(len(self._tracker.tracks))),
        ]
        # One KeyValue per confirmed track (visible in Foxglove Diagnostics panel)
        for t in self._last_tracked:
            cx, cy, cz = float(t.box7[0]), float(t.box7[1]), float(t.box7[2])
            status.values.append(KeyValue(
                key=f'track_{t.track_id}',
                value=(f'{t.cls_name} | '
                       f'pos=({cx:+.1f},{cy:+.1f},{cz:+.1f})m | '
                       f'conf={t.score:.2f}')
            ))

        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status.append(status)
        self._diag_pub.publish(arr)

        # Terminal heartbeat: summary line then one indented line per tracked object
        status_sym = '\u2713 OK  ' if level == DiagnosticStatus.OK else '\u26a0 WARN'
        self.get_logger().info(
            f'[HEARTBEAT] {status_sym} | '
            f'uptime={uptime_s:.0f}s | '
            f'frames={self._frame_count} | '
            f'fps={self._fps_estimate:.2f} | '
            f'dets={self._last_det_count} | '
            f'tracks={self._last_trk_count}'
        )
        for t in self._last_tracked:
            cx, cy, cz = float(t.box7[0]), float(t.box7[1]), float(t.box7[2])
            conf_str = f'{t.score:.2f}' if t.score > 0 else 'n/a'
            self.get_logger().info(
                f'  \u2514 ID:{t.track_id:>3}  {t.cls_name:<11} '
                f'pos=({cx:+.1f},{cy:+.1f},{cz:+.1f})m  '
                f'conf={conf_str}'
            )

    def _draw_confidence_legend(self, img: np.ndarray) -> np.ndarray:
        """Overlay a confidence colour legend in the bottom-left corner."""
        h, w = img.shape[:2]
        legend_items = [
            ((0, 200, 0),   'High confidence  (>= 0.60)'),
            ((0, 0, 255),   'Low confidence   (< 0.60)'),
            ((255, 200, 0), 'Cyan arrow = velocity direction'),
        ]
        box_w, box_h = 220, len(legend_items) * 22 + 12
        x0, y0 = 8, h - box_h - 8

        # Semi-transparent dark background
        overlay = img.copy()
        cv2.rectangle(overlay, (x0, y0), (x0 + box_w, y0 + box_h),
                      (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.6, img, 0.4, 0, img)

        for i, (bgr, label) in enumerate(legend_items):
            cy = y0 + 10 + i * 22
            cv2.rectangle(img, (x0 + 6, cy), (x0 + 20, cy + 14), bgr, -1)
            cv2.putText(img, label, (x0 + 26, cy + 11),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.38, (220, 220, 220), 1,
                        cv2.LINE_AA)
        return img

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
        panel = self._draw_confidence_legend(panel)
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
