import rclpy
from rclpy.node import Node

import rosbag2_py
import cv2
import numpy as np
import os

from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

from sensor_msgs.msg import Image
from ultralytics import YOLO


class YOLOMaskPublisher(Node):

    def __init__(self):
        super().__init__("yolo_mask_publisher")

        self.pub = self.create_publisher(Image, "/yolo/mask", 10)

        self.bridge = CvBridge()
        self.model = YOLO("yolo26n-seg.pt")

        self.bag_path = "./studentProject/"
        self.image_topic = "/blackfly_s/cam0/image_rectified"

        self.reader = rosbag2_py.SequentialReader()

        storage_options = rosbag2_py.StorageOptions(
            uri=self.bag_path, storage_id="sqlite3"
        )

        converter_options = rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        )

        self.reader.open(storage_options, converter_options)

        self.topic_types = {
            t.name: t.type for t in self.reader.get_all_topics_and_types()
        }

        # Save only the first one
        self.saved_first = False

        # Output Directory
        self.output_dir = "output"
        os.makedirs(self.output_dir, exist_ok=True)

        self.get_logger().info("Processing entire RosBag (saving only the first mask)")

        self.timer = self.create_timer(0.01, self.process)

    def process(self):

        if not self.reader.has_next():
            self.get_logger().info("End of the RosBag")
            rclpy.shutdown()
            return

        topic, data, timestamp = self.reader.read_next()

        if topic != self.image_topic:
            return

        msg_type = get_message(self.topic_types[topic])
        msg = deserialize_message(data, msg_type)

        img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # =========================
        # YOLO SEGMENTATION
        # =========================
        results = self.model(img)

        mask_final = np.zeros((img.shape[0], img.shape[1]), dtype=np.uint8)

        for r in results:
            if r.masks is None:
                continue

            masks = r.masks.data.cpu().numpy()

            for mask in masks:
                mask = (mask * 255).astype(np.uint8)
                mask = cv2.resize(mask, (img.shape[1], img.shape[0]))
                mask_final = np.maximum(mask_final, mask)

        # =========================
        # ONLY FIRST: PRINT & SAVE
        # =========================
        if not self.saved_first:
            # print("\n--- MASK (shape + unique values) ---")
            # print("Shape:", mask_final.shape)
            # print("Unique Values:", np.unique(mask_final))

            # print("\n--- ROS Message (original) ---")
            # print(msg)

            output_path = os.path.join(self.output_dir, "mask_first.png")
            print(">>> Trying to save image...")
            cv2.imwrite(output_path, mask_final)

            print(f"\n[OK] Mask saved in: {output_path}")

            self.saved_first = True

        # =========================
        # PUBLISH always
        # =========================
        mask_msg = self.bridge.cv2_to_imgmsg(mask_final, encoding="mono8")

        # For synchronization
        mask_msg.header = msg.header

        self.pub.publish(mask_msg)


def main(args=None):
    rclpy.init(args=args)

    node = YOLOMaskPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
