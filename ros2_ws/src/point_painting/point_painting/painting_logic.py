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
and the full P2 projection matrix directly — one projection pass, no duplication.
Using the full P2 (rather than just the 3×3 camera_matrix) preserves the KITTI
baseline translation encoded in P2[:,3] and matches the reference implementation.
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
    camera_pts = _projector.lidar_to_camera(points_filtered[:, :3])

    # Step 3: keep only points in front of the camera
    # (points behind the camera would project to negative depth and cause issues)
    depth_ok = camera_pts[:, 2] > 0
    depth_indices = np.where(depth_ok)[0]
    cam_front = camera_pts[depth_ok]

    if len(cam_front) == 0:
        return 0, n, [-1] * n

    # Step 4: project to pixels using the full P2 matrix.
    # P2 is (3,4); cam_front is (M,3). Homogenise cam_front to (M,4) with w=1
    # so we use the full KITTI projection: uv_h = P2 @ [X,Y,Z,1]^T.
    # Using only camera_matrix (P2[:,:3]) would drop the stereo baseline
    # translation in P2[:,3] and produce slightly shifted pixel coordinates.
    import cv2
    cam_front_h = np.hstack([cam_front, np.ones((len(cam_front), 1))])
    uv_h = ((_projector.P2.astype(np.float64)) @ cam_front_h.astype(np.float64).T).T
    u_all = uv_h[:, 0] / uv_h[:, 2]
    v_all = uv_h[:, 1] / uv_h[:, 2]

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
    yolo_results: list = None,
    depth_window: float = 3.0,
) -> np.ndarray:
    """
    Project LiDAR points and return a scored cloud for the frustum_detection node.

    Parameters
    ----------
    points_xyz : (N, 3+) LiDAR points. Columns 0-2 must be x, y, z.
    score_maps : dict mapping COCO class ID → (H, W) float32 score map [0, 1].
                 Keys must include at least the COCO IDs present in
                 _COCO_TO_SCORE_COL (0, 1, 2, 3, 5, 7).
    yolo_results : list of Ultralytics YOLO Results (optional).
                   If provided, instance-level depth filtering is applied to resolve
                   background bleeding.
    depth_window : float (default: 3.0 meters)
                   The window size around the median depth of an instance to consider points valid.

    Returns
    -------
    scored_cloud : np.ndarray, shape (N, 8), dtype float32.
        Columns:
          0-2  x, y, z   (LiDAR frame)
          3    intensity  (col 3 of input if available, else 0.0)
          4    ring       (col 4 of input if available, else 0.0)
          5    s_ped      pedestrian score [0, 1]
          6    s_car      car / bus / truck score [0, 1]
          7    s_cyc      cyclist / motorcycle score [0, 1]

    Points that do not project into the image get all scores = 0.
    """
    import cv2

    n = len(points_xyz)
    scored = np.zeros((n, 8), dtype=np.float32)
    scored[:, :3] = points_xyz[:, :3]
    # Carry intensity and ring through from the input cloud when available
    if points_xyz.shape[1] >= 4:
        scored[:, 3] = points_xyz[:, 3]
    if points_xyz.shape[1] >= 5:
        scored[:, 4] = points_xyz[:, 4]

    if _projector is None or n == 0 or (not score_maps and not yolo_results):
        return scored

    # Retrieve image dimensions from the first score map, or default to YOLO result dimensions if available
    h, w = 1200, 1920
    if score_maps:
        sample_map = next(iter(score_maps.values()))
        h, w = sample_map.shape[:2]
    elif yolo_results and len(yolo_results) > 0 and yolo_results[0].orig_shape is not None:
        h, w = yolo_results[0].orig_shape[:2]

    # Step 1: filter ground points
    GROUND_Z_THRESH = -1.5
    not_ground = points_xyz[:, 2] > GROUND_Z_THRESH
    not_ground_indices = np.where(not_ground)[0]
    points_filtered = points_xyz[not_ground]

    if len(points_filtered) == 0:
        return scored

    # Step 2: transform to camera space
    camera_pts = _projector.lidar_to_camera(points_filtered[:, :3])

    # Step 3: keep points in front of camera
    depth_ok = camera_pts[:, 2] > 0
    depth_indices = np.where(depth_ok)[0]
    cam_front = camera_pts[depth_ok]

    if len(cam_front) == 0:
        return scored

    # Step 4: project to pixel coordinates using the full P2 matrix.
    # Homogenise cam_front → (M, 4) so we use P2 @ [X,Y,Z,1]^T, which
    # correctly accounts for the KITTI stereo baseline translation in P2[:,3].
    cam_front_h = np.hstack([cam_front, np.ones((len(cam_front), 1))])
    uv_h = ((_projector.P2.astype(np.float64)) @ cam_front_h.astype(np.float64).T).T
    u_all = uv_h[:, 0] / uv_h[:, 2]
    v_all = uv_h[:, 1] / uv_h[:, 2]

    # Step 5: keep only points inside the image frame
    inside = (u_all >= 0) & (u_all < w) & (v_all >= 0) & (v_all < h)
    orig_indices = not_ground_indices[depth_indices[inside]]
    u_in = np.clip(u_all[inside].astype(int), 0, w - 1)
    v_in = np.clip(v_all[inside].astype(int), 0, h - 1)
    depths_in = cam_front[inside, 2]

    # Step 6: look up per-class scores for each projected point
    # Case A: Instance-level depth-consistent painting (resolves background bleeding)
    if yolo_results is not None and len(yolo_results) > 0:
        for result in yolo_results:
            if result.masks is None:
                continue
            masks   = result.masks.data.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            confs   = result.boxes.conf.cpu().numpy()

            for mask, cls_id, conf in zip(masks, classes, confs):
                cls_id = int(cls_id)
                if cls_id not in _COCO_TO_SCORE_COL:
                    continue
                # Resize mask to original image size
                mask_resized = cv2.resize(mask, (w, h), interpolation=cv2.INTER_LINEAR)
                inside_mask = mask_resized[v_in, u_in] > 0.5
                if not inside_mask.any():
                    continue

                # Compute median depth of points projecting inside this mask
                med_depth = np.median(depths_in[inside_mask])

                # Keep only points within depth window of the median depth
                depth_mask = (depths_in >= med_depth - depth_window) & (depths_in <= med_depth + depth_window)
                valid_paint = inside_mask & depth_mask

                if not valid_paint.any():
                    continue

                # Paint the valid points
                scores = mask_resized[v_in[valid_paint], u_in[valid_paint]] * float(conf)
                col_idx = _COCO_TO_SCORE_COL[cls_id]
                scored[orig_indices[valid_paint], col_idx] = np.maximum(
                    scored[orig_indices[valid_paint], col_idx], scores
                )

    # Case B: Fallback to standard projection-only painting (when yolo_results is None)
    else:
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
