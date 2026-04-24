import numpy as np
import cv2


class KittiLidarToImageProjector:
    """
    Projects LiDAR points into the camera image plane using KITTI-style calibration.

    Required KITTI calibration values:
    - P2: camera projection matrix
    - R0_rect: rectification matrix
    - Tr_velo_to_cam: LiDAR-to-camera transformation
    """

    def __init__(self, calib_file_path: str):
        self.P2, self.R0_rect, self.Tr_velo_to_cam = self.load_kitti_calibration(
            calib_file_path
        )

        # Extract camera intrinsics from P2
        self.camera_matrix = self.P2[:, :3]

        # For OpenCV projection after transformation into camera coordinates
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float64)

    def load_kitti_calibration(self, calib_file_path: str):
        calib = {}

        with open(calib_file_path, "r") as file:
            for line in file:
                if ":" not in line:
                    continue

                key, value = line.split(":", 1)
                values = np.array([float(x) for x in value.strip().split()])

                calib[key] = values

        P2 = calib["P2"].reshape(3, 4)
        R0_rect = np.eye(4)
        R0_rect[:3, :3] = calib["R0_rect"].reshape(3, 3)

        Tr_velo_to_cam = np.eye(4)
        Tr_velo_to_cam[:3, :] = calib["Tr_velo_to_cam"].reshape(3, 4)

        return P2, R0_rect, Tr_velo_to_cam

    def lidar_to_camera(self, lidar_points: np.ndarray) -> np.ndarray:
        """
        Converts LiDAR points into rectified camera coordinates.

        Input:
            lidar_points: Nx3 array [x, y, z]

        Output:
            camera_points: Nx3 array in camera coordinate frame
        """

        if lidar_points.shape[1] != 3:
            raise ValueError("lidar_points must have shape Nx3")

        num_points = lidar_points.shape[0]

        # Convert to homogeneous coordinates: [x, y, z, 1]
        lidar_homogeneous = np.hstack((lidar_points, np.ones((num_points, 1))))

        # KITTI chain:
        # LiDAR → camera → rectified camera
        camera_homogeneous = (
            self.R0_rect @ self.Tr_velo_to_cam @ lidar_homogeneous.T
        ).T

        return camera_homogeneous[:, :3]

    def project_lidar_to_image(self, lidar_points: np.ndarray, image_shape: tuple):
        """
        Projects LiDAR points onto the image plane.

        Input:
            lidar_points: Nx3 LiDAR points
            image_shape: image shape as (height, width, channels) or (height, width)

        Output:
            image_points: Mx2 pixel coordinates
            valid_lidar_points: Mx3 LiDAR points that project inside the image
        """

        height, width = image_shape[:2]

        camera_points = self.lidar_to_camera(lidar_points)

        # Keep only points in front of the camera
        valid_depth_mask = camera_points[:, 2] > 0
        camera_points = camera_points[valid_depth_mask]
        valid_lidar_points = lidar_points[valid_depth_mask]

        # OpenCV projection
        rvec = np.zeros((3, 1), dtype=np.float64)
        tvec = np.zeros((3, 1), dtype=np.float64)

        image_points, _ = cv2.projectPoints(
            camera_points.astype(np.float64),
            rvec,
            tvec,
            self.camera_matrix.astype(np.float64),
            self.dist_coeffs,
        )

        image_points = image_points.reshape(-1, 2)

        # Keep only points inside image boundaries
        u = image_points[:, 0]
        v = image_points[:, 1]

        inside_image_mask = (u >= 0) & (u < width) & (v >= 0) & (v < height)

        image_points = image_points[inside_image_mask]
        valid_lidar_points = valid_lidar_points[inside_image_mask]

        return image_points, valid_lidar_points

    def draw_projected_points(self, image: np.ndarray, lidar_points: np.ndarray):
        """
        Draws projected LiDAR points on the image.
        """

        image_points, valid_lidar_points = self.project_lidar_to_image(
            lidar_points, image.shape
        )

        output_image = image.copy()

        for point in image_points:
            u, v = int(point[0]), int(point[1])
            cv2.circle(output_image, (u, v), radius=2, color=(0, 255, 0), thickness=-1)

        return output_image


if __name__ == "__main__":
    calib_path = "calib.txt"

    projector = KittiLidarToImageProjector(calib_path)

    # Example dummy LiDAR points: x, y, z
    lidar_points = np.array([[5.0, 0.0, 1.0], [10.0, 1.0, 1.5], [15.0, -2.0, 1.2]])

    # Dummy image
    image = np.zeros((375, 1242, 3), dtype=np.uint8)

    result = projector.draw_projected_points(image, lidar_points)

    cv2.imshow("Projected LiDAR Points", result)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
