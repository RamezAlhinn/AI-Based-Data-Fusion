import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from cv_bridge import CvBridge
from PIL import Image as PilImage
from sensor_msgs_py import point_cloud2 as pc2

from point_painting.painting_logic import init_projector, paint_points


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
        # deeplab_repo_path: folder that contains the 'modeling/' package
        # (i.e. the root of the pytorch-deeplab-xception clone)
        # checkpoint_path: path to deeplab-resnet.pth.tar
        self.declare_parameter('deeplab_repo_path', '')
        self.declare_parameter('checkpoint_path', '')
        deeplab_repo = self.get_parameter('deeplab_repo_path').get_parameter_value().string_value
        checkpoint = self.get_parameter('checkpoint_path').get_parameter_value().string_value

        if deeplab_repo and checkpoint:
            if deeplab_repo not in sys.path:
                sys.path.insert(0, deeplab_repo)
            try:
                from point_painting.segmentation.deeplab_segmentation import load_model
                self._seg_model = load_model(checkpoint)
                self.get_logger().info(f'Segmentation model loaded from: {checkpoint}')
            except Exception as e:
                self.get_logger().error(f'Failed to load segmentation model: {e}')
        else:
            self.get_logger().warn(
                'No segmentation model loaded — node will use raw image channel as label map. '
                'Pass: --ros-args -p deeplab_repo_path:=/path/to/pytorch-deeplab-xception '
                '-p checkpoint_path:=/path/to/deeplab-resnet.pth.tar'
            )

        # --- Publishers / Subscribers ---
        # The LiDAR and camera were recorded with different clocks so we use a
        # latest-message cache instead of ApproximateTimeSynchronizer.
        self._debug_pub = self.create_publisher(String, '/painting/debug', 10)
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
            # Run DeepLab: BGR numpy → PIL RGB → (H, W) class-index array
            from point_painting.segmentation.deeplab_segmentation import segment_image
            pil_image = PilImage.fromarray(cv_image[..., ::-1])
            seg_image = segment_image(self._seg_model, pil_image)
        else:
            # Degraded mode: treat first channel of raw image as label map
            seg_image = cv_image[:, :, 0] if cv_image.ndim == 3 else cv_image

        points = list(pc2.read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if len(points) == 0:
            return

        xyz = np.array([(p[0], p[1], p[2]) for p in points], dtype=np.float32)
        painted, skipped, _ = paint_points(xyz, seg_image)

        self._frame_count += 1
        if self._frame_count % 50 == 0:
            self.get_logger().info(
                f'Frame {self._frame_count}: painted={painted}, skipped={skipped}')

        msg = String()
        msg.data = f'frame={self._frame_count} painted={painted} skipped={skipped}'
        self._debug_pub.publish(msg)


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
