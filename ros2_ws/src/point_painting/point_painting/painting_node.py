import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from cv_bridge import CvBridge
import message_filters
from sensor_msgs_py import point_cloud2 as pc2

from point_painting.painting_logic import init_projector, paint_points


class PaintingNode(Node):
    def __init__(self):
        super().__init__('painting_node')
        self._bridge = CvBridge()
        self._frame_count = 0

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

        self._debug_pub = self.create_publisher(String, '/painting/debug', 10)

        img_sub = message_filters.Subscriber(
            self, Image, '/blackfly_s/cam0/image_rectified')
        lidar_sub = message_filters.Subscriber(
            self, PointCloud2, '/velodyne/points_raw')

        self._sync = message_filters.ApproximateTimeSynchronizer(
            [img_sub, lidar_sub], queue_size=10, slop=0.1)
        self._sync.registerCallback(self._callback)

        self.get_logger().info('PaintingNode started, waiting for synced messages...')

    def _callback(self, img_msg: Image, cloud_msg: PointCloud2):
        seg_image = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')

        # Treat multi-channel image as a 2-D segmentation label map using first channel.
        if seg_image.ndim == 3:
            seg_image = seg_image[:, :, 0]

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
