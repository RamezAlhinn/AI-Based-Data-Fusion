import os
import rosbag2_py
import cv2
from cv_bridge import CvBridge
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from sensor_msgs_py import point_cloud2
import numpy as np


def extract_bag_data(bag_path, image_topic, lidar_topic, output_dir="output_data"):
    # Create output directory
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)
        print(f"Created directory: {output_dir}")

    # Setup the reader
    reader = rosbag2_py.SequentialReader()

    storage_options = rosbag2_py.StorageOptions(
        uri=bag_path, storage_id="sqlite3"
    )  # or 'sqlite3'
    serialization_format = "cdr"
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format=serialization_format,
        output_serialization_format=serialization_format,
    )

    reader.open(storage_options, converter_options)

    # Map topic names to their message types
    topic_types = {
        topic.name: topic.type for topic in reader.get_all_topics_and_types()
    }
    bridge = CvBridge()

    img_count = 0
    lidar_logged = False

    while reader.has_next() and (img_count < 10 or not lidar_logged):
        (topic, data, timestamp) = reader.read_next()

        # Handle Images
        if topic == image_topic and img_count < 10:
            msg_type = get_message(topic_types[topic])
            msg = deserialize_message(data, msg_type)

            # Convert ROS Image to OpenCV and save
            cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            file_path = os.path.join(output_dir, f"frame_{img_count:04d}.jpg")
            cv2.imwrite(file_path, cv_img)

            print(f"Saved: {file_path}")
            img_count += 1

        # Handle LiDAR Timestamp
        elif topic == lidar_topic and not lidar_logged:
            msg_type = get_message(topic_types[topic])
            msg = deserialize_message(data, msg_type)

            # 1. Read points with specific fields
            # Note: Ensure these field names match what is actually in your bag
            points_generator = point_cloud2.read_points(
                msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
            )

            # 2. Create the structured array first
            pc_structured = np.array(list(points_generator))

            # 3. Collapse the structure into a pure float32 matrix
            # We extract the fields we want and cast them to a record-free array
            pc_array = (
                pc_structured[["x", "y", "z", "intensity"]]
                .view(np.float32)
                .reshape(-1, 4)
            )

            # 4. Save
            pc_file_path = os.path.join(output_dir, "lidar_points.npy")
            np.save(pc_file_path, pc_array)
            print(
                f"Successfully saved matrix of shape {pc_array.shape} to {pc_file_path}"
            )

            # 4. Log metadata for confirmation
            log_path = os.path.join(output_dir, "lidar_metadata.txt")
            with open(log_path, "w") as f:
                f.write(
                    f"Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec}\n"
                )
                f.write(f"Frame ID: {msg.header.frame_id}\n")
                f.write(
                    f"Points Shape: {pc_array.shape}\n"
                )  # Shows (Number of points, 3)

            lidar_logged = True


if __name__ == "__main__":
    # Path to the folder containing metadata.yaml
    BAG_FILE = "/workspace/studentProject/"
    IMG_TOPIC = "/blackfly_s/cam0/image_rectified"
    LIDAR_TOPIC = "/velodyne/points_raw"

    extract_bag_data(BAG_FILE, IMG_TOPIC, LIDAR_TOPIC)
