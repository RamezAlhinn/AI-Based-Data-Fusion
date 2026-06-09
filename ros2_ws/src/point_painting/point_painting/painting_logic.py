"""
PointPainting core logic — projection and class label lookup.

This module is intentionally kept free of ROS dependencies so it can be
imported and unit-tested on any machine without a ROS runtime.

Public functions:
    init_projector(calib_file_path)     — load calibration matrices once at startup
    paint_points(points_xyz, seg_image) — project LiDAR points and return class IDs
    paint_points_scored(points_xyz, score_maps)
                                        — return full scored cloud [x,y,z,int,ring,s_ped,s_car,s_cyc]
                                          for consumption by the frustum_detection node

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


# COCO class IDs that carry paint scores
# Column layout of scored cloud: [x, y, z, intensity, ring, s_ped, s_car, s_cyc]
# These COCO IDs map to score columns as follows:
#   person (0)                      → col 5  (s_ped)
#   car (2), bus (5), truck (7)     → col 6  (s_car)
#   bicycle (1), motorcycle (3)     → col 7  (s_cyc)
_COCO_TO_SCORE_COL = {0: 5, 2: 6, 5: 6, 7: 6, 1: 7, 3: 7}


def paint_points_scored(
    points_xyz: np.ndarray,
    score_maps: dict,
) -> np.ndarray:
    """
    Project LiDAR points and return a scored cloud for the frustum_detection node.

    Parameters
    ----------
    points_xyz : (N, 3+) LiDAR points. Columns 0-2 must be x, y, z.
    score_maps : dict mapping COCO class ID → (H, W) float32 score map [0, 1].
                 Keys must include at least the COCO IDs present in
                 _COCO_TO_SCORE_COL (0, 1, 2, 3, 5, 7).

    Returns
    -------
    scored_cloud : np.ndarray, shape (N, 8), dtype float32.
        Columns:
          0-2  x, y, z   (LiDAR frame)
          3    intensity  (set to 0.0 — not available from this path)
          4    ring       (set to 0.0 — not available from this path)
          5    s_ped      pedestrian score [0, 1]
          6    s_car      car / bus / truck score [0, 1]
          7    s_cyc      cyclist / motorcycle score [0, 1]

    Points that do not project into the image get all scores = 0.
    """
    import cv2

    n = len(points_xyz)
    scored = np.zeros((n, 8), dtype=np.float32)
    scored[:, :3] = points_xyz[:, :3]

    if _projector is None or n == 0 or not score_maps:
        return scored

    # Retrieve the first score map to get image dimensions
    sample_map = next(iter(score_maps.values()))
    h, w = sample_map.shape[:2]

    # Step 1: filter ground points
    GROUND_Z_THRESH = -1.5
    not_ground = points_xyz[:, 2] > GROUND_Z_THRESH
    not_ground_indices = np.where(not_ground)[0]
    points_filtered = points_xyz[not_ground]

    if len(points_filtered) == 0:
        return scored

    # Step 2: transform to camera space
    camera_pts = _projector.lidar_to_camera(points_filtered)

    # Step 3: keep points in front of camera
    depth_ok = camera_pts[:, 2] > 0
    depth_indices = np.where(depth_ok)[0]
    cam_front = camera_pts[depth_ok]

    if len(cam_front) == 0:
        return scored

    # Step 4: project to pixel coordinates
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

    # Step 6: look up per-class scores for each projected point
    # Group COCO classes into the 3 score columns
    _score_groups = {
        5: [0],        # s_ped  ← person
        6: [2, 5, 7],  # s_car  ← car, bus, truck
        7: [1, 3],     # s_cyc  ← bicycle, motorcycle
    }

    for col_idx, coco_ids in _score_groups.items():
        # Combine scores from all COCO classes that map to this column
        combined = np.zeros(len(u_in), dtype=np.float32)
        for coco_id in coco_ids:
            smap = score_maps.get(coco_id)
            if smap is None:
                continue
            if smap.shape != (h, w):
                smap = cv2.resize(smap, (w, h), interpolation=cv2.INTER_LINEAR)
            combined = np.maximum(combined, smap[v_in, u_in])
        scored[orig_indices, col_idx] = combined

    return scored
