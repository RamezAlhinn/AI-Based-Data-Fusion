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

# RGB colors per class ID for the painted point cloud (COCO classes)
# painted points show their semantic class as a color in Foxglove
CLASS_COLORS = {
    0:  (50,  50,  50),   # background — dark grey
    2:  (0,   0,   255),  # bicycle    — blue
    3:  (0,   255, 0),    # car        — green
    4:  (255, 128, 0),    # motorcycle — orange
    6:  (255, 255, 0),    # bus        — yellow
    7:  (0,   255, 255),  # train      — cyan
    15: (255, 0,   0),    # person     — red
}
UNPAINTED_COLOR = (30, 30, 30)  # near-black for unpainted points


def _color_to_float(r, g, b):
    packed = struct.pack('BBBB', b, g, r, 0)
    return struct.unpack('f', packed)[0]


class PaintingNode(Node):
    def __init__(self):
        super().__init__('painting_node')
        self._bridge = CvBridge()
        self._frame_count = 0
        self._seg_model = None
        self._latest_img = None
        self._latest_cloud = None

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
        self.declare_parameter('deeplab_repo_path', '')
        self.declare_parameter('checkpoint_path', '')
        checkpoint = self.get_parameter('checkpoint_path').get_parameter_value().string_value

        try:
            from point_painting.segmentation.deeplab_segmentation import load_model
            self._seg_model = load_model(checkpoint if checkpoint else None)
            self.get_logger().info('Segmentation model loaded (torchvision DeepLabV3-ResNet101)')
        except Exception as e:
            self.get_logger().error(f'Failed to load segmentation model: {e}')
            self.get_logger().warn('Node will use raw image channel as label map.')

        # --- Publishers / Subscribers ---
        # The LiDAR and camera were recorded with different clocks so we use a
        # latest-message cache instead of ApproximateTimeSynchronizer.
        self._debug_pub = self.create_publisher(String, '/painting/debug', 10)
        self._painted_pub = self.create_publisher(PointCloud2, '/painting/painted_cloud', 10)
        # Segmentation overlay: original image with coloured class masks blended on top.
        # View this in Foxglove Image panel to verify the model labels the right objects.
        self._overlay_pub = self.create_publisher(Image, '/painting/segmentation_overlay', 10)
        self.create_subscription(Image, '/blackfly_s/cam0/image_rectified', self._img_cb, 10)
        self.create_subscription(PointCloud2, '/velodyne/points_raw', self._cloud_cb, 10)

        self.get_logger().info('PaintingNode started, waiting for synced messages...')

    def _img_cb(self, msg: Image):
        self._latest_img = msg
        if self._latest_cloud is not None:
            self._callback(self._latest_img, self._latest_cloud)

    def _cloud_cb(self, msg: PointCloud2):
        self._latest_cloud = msg
        if self._latest_img is not None:
            self._callback(self._latest_img, self._latest_cloud)

    def _callback(self, img_msg: Image, cloud_msg: PointCloud2):
        cv_image = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')

        if self._seg_model is not None:
            from point_painting.segmentation.deeplab_segmentation import segment_image
            pil_image = PilImage.fromarray(cv_image[..., ::-1])
            seg_image = segment_image(self._seg_model, pil_image)
        else:
            seg_image = cv_image[:, :, 0] if cv_image.ndim == 3 else cv_image

        points = list(pc2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if len(points) == 0:
            return

        xyz = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)
        painted, skipped, class_ids = paint_points(xyz, seg_image)

        self._publish_painted_cloud(xyz, class_ids, cloud_msg.header)
        self._publish_segmentation_overlay(cv_image, seg_image, img_msg.header)

        self._frame_count += 1
        if self._frame_count % 50 == 0:
            self.get_logger().info(
                f'Frame {self._frame_count}: painted={painted}, skipped={skipped}')

        msg = String()
        msg.data = f'frame={self._frame_count} painted={painted} skipped={skipped}'
        self._debug_pub.publish(msg)

    def _publish_segmentation_overlay(self, cv_image: np.ndarray, seg_image: np.ndarray, header):
        """Blend coloured class masks onto the original image and publish for verification."""
        # Work in BGR (cv_image from cv_bridge is BGR)
        overlay = cv_image.copy() if cv_image.ndim == 3 else cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        colour_mask = np.zeros_like(overlay)

        for class_id, (r, g, b) in CLASS_COLORS.items():
            if class_id == 0:
                continue  # skip background
            mask = seg_image == class_id
            if not mask.any():
                continue
            # seg_image is at model output resolution — resize mask to match image
            if seg_image.shape != overlay.shape[:2]:
                import cv2 as _cv2
                mask_u8 = mask.astype(np.uint8) * 255
                mask_u8 = _cv2.resize(mask_u8, (overlay.shape[1], overlay.shape[0]),
                                      interpolation=_cv2.INTER_NEAREST)
                mask = mask_u8 > 0
            colour_mask[mask] = (b, g, r)  # BGR order

        blended = cv2.addWeighted(overlay, 0.6, colour_mask, 0.4, 0)
        overlay_msg = self._bridge.cv2_to_imgmsg(blended, encoding='bgr8')
        overlay_msg.header = header
        self._overlay_pub.publish(overlay_msg)

    def _publish_painted_cloud(self, xyz: np.ndarray, class_ids: list, header):
        fields = [
            PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]

        cloud_data = []
        for i, (pt, cid) in enumerate(zip(xyz, class_ids)):
            r, g, b = CLASS_COLORS.get(cid, UNPAINTED_COLOR)
            rgb_float = _color_to_float(r, g, b)
            cloud_data.append([float(pt[0]), float(pt[1]), float(pt[2]), rgb_float])

        cloud_msg = pc2.create_cloud(header, fields, cloud_data)
        self._painted_pub.publish(cloud_msg)


def main(args=None):
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
