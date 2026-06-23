"""
BEVRecorderNode — subscribes to /frustum/bev and records frames into an MP4
video file saved to /workspace/video_output/tracking_output.mp4 on shutdown.

Run:
    ros2 run frustum_detection bev_recorder_node \
        --ros-args -p output_dir:=/workspace/video_output
"""

import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np


class BEVRecorderNode(Node):

    def __init__(self):
        super().__init__('bev_recorder_node')
        self.declare_parameter('output_dir', '/workspace/video_output')
        self.declare_parameter('fps', 10.0)

        self._out_dir = self.get_parameter('output_dir').value
        self._fps     = self.get_parameter('fps').value
        os.makedirs(self._out_dir, exist_ok=True)

        self._bridge  = CvBridge()
        self._frames  = []
        self._writer  = None
        self._out_path = os.path.join(self._out_dir, 'tracking_output.mp4')

        self.create_subscription(Image, '/frustum/bev', self._cb, 10)
        self.get_logger().info(
            f'BEVRecorderNode ready — recording /frustum/bev to {self._out_path}')

    def _cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Frame decode error: {e}')
            return

        if self._writer is None:
            import cv2
            h, w = frame.shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
            self._writer = cv2.VideoWriter(
                self._out_path, fourcc, self._fps, (w, h))
            self.get_logger().info(f'Video writer opened: {w}x{h} @ {self._fps} fps')

        self._writer.write(frame)

    def destroy_node(self):
        if self._writer is not None:
            self._writer.release()
            self.get_logger().info(f'Video saved to: {self._out_path}')
        else:
            self.get_logger().warn('No frames recorded — /frustum/bev had no data.')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = BEVRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
