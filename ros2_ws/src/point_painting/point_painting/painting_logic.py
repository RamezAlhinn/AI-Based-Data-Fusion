"""
PointPainting core logic — projection and class label lookup.

This module is intentionally kept free of ROS dependencies so it can be
imported and unit-tested on any machine without a ROS runtime.

The two public functions are:
    init_projector(calib_file_path) — load calibration matrices once at startup
    paint_points(points_xyz, seg_image) — project LiDAR points and return class IDs

The projection uses KittiLidarToImageProjector from the perception_framework
package (implemented by arpitashil676). We call its lidar_to_camera() method
and camera_matrix directly — one projection pass, no duplication.
"""

import numpy as np
from perception_framework.lidar_to_image_projection import KittiLidarToImageProjector

_projector: KittiLidarToImageProjector | None = None


def init_projector(calib_file_path: str) -> None:
    global _projector
    _projector = KittiLidarToImageProjector(calib_file_path)


def paint_points(points_xyz: np.ndarray, seg_image: np.ndarray):
    """
    Project each LiDAR point onto the segmentation mask and return its class ID.

    Returns (painted_count, skipped_count, class_ids).
    class_ids[i] == -1 means point i did not land inside the image.

    Uses KittiLidarToImageProjector (perception_framework) as the single
    source of projection — one projection pass, no duplicate math.
    """
    n = len(points_xyz)

    if _projector is None or n == 0:
        return 0, n, [-1] * n

    h, w = seg_image.shape[:2]

    # Step 1: filter ground points in Velodyne frame (z-up)
    GROUND_Z_THRESH = -1.5
    not_ground = points_xyz[:, 2] > GROUND_Z_THRESH
    not_ground_indices = np.where(not_ground)[0]
    points_filtered = points_xyz[not_ground]

    if len(points_filtered) == 0:
        return 0, n, [-1] * n

    # Step 2: transform points to camera space
    camera_pts = _projector.lidar_to_camera(points_filtered)

    # Step 3: keep only points in front of the camera
    # (points behind the camera would project to negative depth and cause issues)
    depth_ok = camera_pts[:, 2] > 0
    depth_indices = np.where(depth_ok)[0]
    cam_front = camera_pts[depth_ok]

    if len(cam_front) == 0:
        return 0, n, [-1] * n

    # Step 4: project to pixels using camera matrix
    import cv2
    proj, _ = cv2.projectPoints(
        cam_front.astype(np.float64),
        np.zeros((3, 1)), np.zeros((3, 1)),
        _projector.camera_matrix.astype(np.float64),
        _projector.dist_coeffs,
    )
    proj = proj.reshape(-1, 2)
    u_all, v_all = proj[:, 0], proj[:, 1]

    # Step 5: keep only points inside the image frame
    inside = (u_all >= 0) & (u_all < w) & (v_all >= 0) & (v_all < h)
    orig_indices = not_ground_indices[depth_indices[inside]]
    u_in = np.clip(u_all[inside].astype(int), 0, w - 1)
    v_in = np.clip(v_all[inside].astype(int), 0, h - 1)

    # Step 6: majority-vote class lookup in a 3px neighborhood
    def majority_class(lm, cy, cx, r=3):
        y0, y1 = max(0, cy - r), min(lm.shape[0], cy + r + 1)
        x0, x1 = max(0, cx - r), min(lm.shape[1], cx + r + 1)
        patch = lm[y0:y1, x0:x1]
        valid = patch[patch >= 0]
        if len(valid) == 0:
            return -1
        counts = np.bincount(valid)
        return int(np.argmax(counts))

    class_ids = np.full(n, -1, dtype=int)
    class_ids_in = np.array([majority_class(seg_image, v, u, r=3) for u, v in zip(u_in, v_in)])

    # Step 7: depth filtering using a 2D local minimum depth map
    depths_in = cam_front[inside, 2]
    depth_map = np.full((h, w), 1000.0, dtype=np.float32)
    # Sort depths in descending order so that the minimum depth is written last and overwrites larger depths
    sorted_idx = np.argsort(depths_in)[::-1]
    depth_map[v_in[sorted_idx], u_in[sorted_idx]] = depths_in[sorted_idx]

    # Perform morphological erosion to propagate local minimum depth
    kernel = np.ones((7, 7), np.uint8)
    min_depth_map = cv2.erode(depth_map, kernel)

    # Check depth consistency
    local_min_depths = min_depth_map[v_in, u_in]
    depth_valid = depths_in <= local_min_depths + 3.0

    # Invalidate class IDs for points that fail depth verification
    class_ids_in[~depth_valid] = -1

    class_ids[orig_indices] = class_ids_in
    painted = int((class_ids >= 0).sum())

    return painted, n - painted, class_ids.tolist()
