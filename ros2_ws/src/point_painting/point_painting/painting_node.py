"""
PointPainting ROS 2 Node.

Subscribes to a camera image topic and a LiDAR point cloud topic, runs
YOLO instance segmentation on each camera frame, projects every LiDAR
point onto the segmentation mask, and attaches the COCO class ID at that
pixel to the point.

Published topics:
    /painting/debug               (std_msgs/String)      — painted/skipped counts per frame
    /painting/painted_cloud       (sensor_msgs/PointCloud2) — full point cloud coloured by class
    /painting/segmentation_overlay (sensor_msgs/Image)   — YOLO masks blended on camera image
    /painting/points_overlay      (sensor_msgs/Image)    — projected LiDAR dots on camera image

Parameters:
    calib_file      (str) — path to KITTI-format calibration file (calib.txt)
    checkpoint_path (str) — optional path to a custom YOLO model file
                            defaults to yolo11m-seg.pt (auto-downloaded)
"""

import sys
import struct
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String, Header
from cv_bridge import CvBridge
from PIL import Image as PilImage
from sensor_msgs_py import point_cloud2 as pc2

from point_painting.painting_logic import init_projector, paint_points

# COCO class IDs mapped to RGB display colours for Foxglove.
# -1 = background / no detection — uses UNPAINTED_COLOR instead.
CLASS_COLORS = {
    0:  (255, 0,   0),    # person     — red
    1:  (0,   0,   255),  # bicycle    — blue
    2:  (0,   255, 0),    # car        — green
    3:  (255, 128, 0),    # motorcycle — orange
    5:  (255, 255, 0),    # bus        — yellow
    7:  (0,   200, 0),    # truck      — dark green
}
UNPAINTED_COLOR = (30, 30, 30)


def _color_to_float(r, g, b):
    """Pack an RGB triplet into a single float32 for PointCloud2 RGB field."""
    packed = struct.pack('BBBB', b, g, r, 0)
    return struct.unpack('f', packed)[0]


class PaintingNode(Node):
    """
    ROS 2 node that implements the PointPainting fusion algorithm.

    On every incoming LiDAR scan or camera frame, pairs the latest message
    from each topic (latest-message cache instead of time synchronisation —
    the two sensors were recorded with different clocks in the bag so
    timestamp-based sync never fires), then:

      1. Runs YOLO26n-seg on the camera frame to get a per-pixel class mask.
      2. Projects all LiDAR points onto the mask using the KITTI calibration.
      3. Attaches the class ID at each projected pixel to the original 3D point.
      4. Publishes the enriched point cloud and two verification image topics.
    """

    def __init__(self):
        super().__init__('painting_node')
        self._bridge = CvBridge()
        self._frame_count = 0
        self._seg_model = None
        self._latest_img_msg = None
        self._latest_seg_image = None
        self._latest_seg_image_stamp = None
        self._latest_cv_image = None

        # --- Calibration ---
        self.declare_parameter('calib_file', '')
        calib_file = self.get_parameter('calib_file').get_parameter_value().string_value
        if calib_file:
            init_projector(calib_file)
            self.get_logger().info(f'Loaded calibration from: {calib_file}')
        else:
            self.get_logger().warn(
                'No calib_file parameter set — projection will skip all points. '
                'Pass: --ros-args -p calib_file:=/path/to/calib.txt'
            )

        # --- Segmentation model ---
        self.declare_parameter('checkpoint_path', '')
        checkpoint = self.get_parameter('checkpoint_path').get_parameter_value().string_value

        try:
            from point_painting.segmentation.yolo_segmentation import load_model
            self._seg_model = load_model(checkpoint if checkpoint else None)
            model_name = checkpoint if checkpoint else 'yolo11n-seg.pt (default)'
            self.get_logger().info(f'Segmentation model loaded: {model_name}')
        except Exception as e:
            self.get_logger().error(f'Failed to load segmentation model: {e}')
            self.get_logger().warn('Node will use raw image channel as label map.')

        # --- Publishers / Subscribers ---
        self._debug_pub = self.create_publisher(String, '/painting/debug', 10)
        self._painted_pub = self.create_publisher(PointCloud2, '/painting/painted_cloud', 10)
        self._overlay_pub = self.create_publisher(Image, '/painting/segmentation_overlay', 10)
        self._points_overlay_pub = self.create_publisher(Image, '/painting/points_overlay', 10)
        self.create_subscription(Image, '/blackfly_s/cam0/image_rectified', self._img_cb, 10)
        self.create_subscription(PointCloud2, '/velodyne/points_raw', self._cloud_cb, 10)

        self.get_logger().info('PaintingNode started, waiting for messages...')

    def _img_cb(self, msg: Image):
        """Cache the latest camera frame without triggering painting."""
        self._latest_img_msg = msg

    def _cloud_cb(self, msg: PointCloud2):
        """
        Process the latest LiDAR scan.
        Only runs YOLO segmentation on the cached camera frame if it is new.
        """
        if self._latest_img_msg is None:
            return

        img_msg = self._latest_img_msg
        img_stamp = (img_msg.header.stamp.sec, img_msg.header.stamp.nanosec)

        if self._latest_seg_image is None or self._latest_seg_image_stamp != img_stamp:
            cv_image = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
            if self._seg_model is not None:
                from point_painting.segmentation.yolo_segmentation import segment_image
                pil_image = PilImage.fromarray(cv_image[..., ::-1])
                seg_image = segment_image(self._seg_model, pil_image)
            else:
                seg_image = cv_image[:, :, 0] if cv_image.ndim == 3 else cv_image

            self._latest_seg_image = seg_image
            self._latest_seg_image_stamp = img_stamp
            self._latest_cv_image = cv_image

        cv_image = self._latest_cv_image
        seg_image = self._latest_seg_image

        points = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if len(points) == 0:
            return

        xyz = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)
        painted, skipped, class_ids = paint_points(xyz, seg_image)

        self._publish_painted_cloud(xyz, class_ids, msg.header)

        self._frame_count += 1
        # Throttle verification overlays to every 5th LiDAR frame
        if self._frame_count % 5 == 0:
            self._publish_segmentation_overlay(cv_image, seg_image, img_msg.header)
            self._publish_points_overlay(cv_image, xyz, class_ids, msg.header)
        if self._frame_count % 50 == 0:
            self.get_logger().info(
                f'Frame {self._frame_count}: painted={painted}, skipped={skipped}')

        msg_str = String()
        msg_str.data = f'frame={self._frame_count} painted={painted} skipped={skipped}'
        self._debug_pub.publish(msg_str)

    def _publish_segmentation_overlay(self, cv_image: np.ndarray,
                                      seg_image: np.ndarray, header):
        """
        Blend YOLO class masks onto the camera image and publish.

        Useful for verifying that the segmentation model labels the correct
        objects before trusting the painted point cloud. Published on
        /painting/segmentation_overlay at 1/5 of the node rate.
        """
        overlay = cv_image.copy() if cv_image.ndim == 3 else cv2.cvtColor(
            cv_image, cv2.COLOR_GRAY2BGR)
        colour_mask = np.zeros_like(overlay)

        for class_id, (r, g, b) in CLASS_COLORS.items():
            if class_id == -1:
                continue
            mask = seg_image == class_id
            if not mask.any():
                continue
            if seg_image.shape != overlay.shape[:2]:
                import cv2 as _cv2
                mask_u8 = mask.astype(np.uint8) * 255
                mask_u8 = _cv2.resize(mask_u8, (overlay.shape[1], overlay.shape[0]),
                                      interpolation=_cv2.INTER_NEAREST)
                mask = mask_u8 > 0
            colour_mask[mask] = (b, g, r)  # BGR order for OpenCV

        blended = cv2.addWeighted(overlay, 0.6, colour_mask, 0.4, 0)
        overlay_msg = self._bridge.cv2_to_imgmsg(blended, encoding='bgr8')
        overlay_msg.header = header
        self._overlay_pub.publish(overlay_msg)

    def _publish_points_overlay(self, cv_image: np.ndarray, xyz: np.ndarray,
                                class_ids: list, header):
        """
        Draw projected LiDAR points as coloured dots on the camera image and publish.

        Each dot shows exactly which pixel a LiDAR point projects onto and what
        class it received. This is the clearest end-to-end verification of the
        full projection + segmentation pipeline. Published on
        /painting/points_overlay at 1/5 of the node rate.
        """
        from point_painting.painting_logic import _projector

        if _projector is None:
            return

        h, w = cv_image.shape[:2]
        canvas = cv_image.copy()

        camera_pts = _projector.lidar_to_camera(xyz)
        depth_ok = camera_pts[:, 2] > 0
        cam_depth = camera_pts[depth_ok]
        depth_indices = np.where(depth_ok)[0]

        import cv2 as _cv2
        proj, _ = _cv2.projectPoints(
            cam_depth.astype(np.float64),
            np.zeros((3, 1)), np.zeros((3, 1)),
            _projector.camera_matrix.astype(np.float64),
            _projector.dist_coeffs,
        )
        proj = proj.reshape(-1, 2)
        u_all, v_all = proj[:, 0], proj[:, 1]
        inside = (u_all >= 0) & (u_all < w) & (v_all >= 0) & (v_all < h)

        # Depth-sorted rendering with scaled radius (far points drawn first)
        depths = cam_depth[inside, 2]
        order = np.argsort(depths)[::-1]

        class_arr = np.array(class_ids)
        inside_indices = np.where(inside)[0]

        for idx in order:
            orig_idx = depth_indices[inside_indices[idx]]
            u = int(np.clip(u_all[inside_indices[idx]], 0, w - 1))
            v = int(np.clip(v_all[inside_indices[idx]], 0, h - 1))
            cls_id = class_arr[orig_idx]
            r, g, b = CLASS_COLORS.get(cls_id, UNPAINTED_COLOR)
            depth = depths[idx]
            radius = max(3, int(30.0 / max(depth, 1.0)))
            _cv2.circle(canvas, (u, v), radius=radius, color=(b, g, r), thickness=-1)

        msg = self._bridge.cv2_to_imgmsg(canvas, encoding='bgr8')
        msg.header = header
        self._points_overlay_pub.publish(msg)

    def _publish_painted_cloud(self, xyz: np.ndarray, class_ids: list, header):
        """
        Publish the semantically enriched point cloud.

        Each point keeps its original LiDAR x, y, z coordinates. The rgb
        field encodes the COCO class colour so Foxglove can display it with
        Color mode: BGR (packed), Color field: rgb.
        """
        fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = []
        for pt, cid in zip(xyz, class_ids):
            r, g, b = CLASS_COLORS.get(cid, UNPAINTED_COLOR)
            cloud_data.append([float(pt[0]), float(pt[1]), float(pt[2]),
                                _color_to_float(r, g, b)])

        cloud_msg = pc2.create_cloud(header, fields, cloud_data)
        self._painted_pub.publish(cloud_msg)


def main(args=None):
    """Entry point — initialise ROS 2 and spin the painting node."""
    rclpy.init(args=args)
    node = PaintingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
