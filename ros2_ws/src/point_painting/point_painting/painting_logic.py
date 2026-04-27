import numpy as np
from perception_framework.lidar_to_image_projection import KittiLidarToImageProjector

_projector: KittiLidarToImageProjector | None = None


def init_projector(calib_file_path: str) -> None:
    """Call once at node startup with the path to the KITTI calib file."""
    global _projector
    _projector = KittiLidarToImageProjector(calib_file_path)


def paint_points(points_xyz: np.ndarray, seg_image: np.ndarray):
    """
    For each (x, y, z) point, project to a pixel and look up its class ID.

    Returns (painted_count, skipped_count, class_ids).

    Requires init_projector() to have been called first.
    Falls back to skipping all points if no projector is set (useful in tests).
    """
    n = len(points_xyz)

    if _projector is None or n == 0:
        return 0, n, [-1] * n

    h, w = seg_image.shape[:2]

    # project_lidar_to_image returns only the subset of points that land
    # inside the image, so we need to track which original indices survive.
    image_points, valid_lidar = _projector.project_lidar_to_image(points_xyz, (h, w))

    # Map valid lidar points back to original indices via a boolean mask.
    # We rerun the depth filter here to build the same mask cheaply.
    camera_pts = _projector.lidar_to_camera(points_xyz)
    depth_ok = camera_pts[:, 2] > 0

    # Among depth-valid points, find those that land inside the image frame.
    cam_depth_pts = camera_pts[depth_ok]
    depth_indices = np.where(depth_ok)[0]

    class_ids = np.full(n, -1, dtype=int)
    painted = 0

    if len(cam_depth_pts) > 0 and len(image_points) > 0:
        us = image_points[:, 0].astype(int)
        vs = image_points[:, 1].astype(int)

        # image_points are already clipped to [0,w) x [0,h) by project_lidar_to_image.
        # Recover the original indices: valid_lidar rows correspond 1-to-1 with
        # image_points rows, both derived from the same in-bounds mask.
        import cv2 as _cv2
        import numpy as _np

        rvec = _np.zeros((3, 1), dtype=_np.float64)
        tvec = _np.zeros((3, 1), dtype=_np.float64)
        all_img_pts, _ = _cv2.projectPoints(
            cam_depth_pts.astype(_np.float64),
            rvec, tvec,
            _projector.camera_matrix.astype(_np.float64),
            _projector.dist_coeffs,
        )
        all_img_pts = all_img_pts.reshape(-1, 2)
        u_all = all_img_pts[:, 0]
        v_all = all_img_pts[:, 1]
        inside = (u_all >= 0) & (u_all < w) & (v_all >= 0) & (v_all < h)
        inside_orig_indices = depth_indices[inside]
        u_in = u_all[inside].astype(int)
        v_in = v_all[inside].astype(int)

        class_ids[inside_orig_indices] = seg_image[v_in, u_in]
        painted = int(inside.sum())

    skipped = n - painted
    return painted, skipped, class_ids.tolist()
