import numpy as np


def project_point_to_pixel(x, y, z):
    """Stub: returns a hardcoded pixel coordinate until real calibration is wired in."""
    return (100, 100)


def paint_points(points_xyz, seg_image):
    """
    For each (x, y, z) point, project to a pixel and look up its class ID.

    Returns (painted_count, skipped_count, class_ids).
    """
    h, w = seg_image.shape[:2]
    class_ids = []
    painted = 0
    skipped = 0

    for x, y, z in points_xyz:
        u, v = project_point_to_pixel(x, y, z)
        if 0 <= v < h and 0 <= u < w:
            class_ids.append(int(seg_image[v, u]))
            painted += 1
        else:
            class_ids.append(-1)
            skipped += 1

    return painted, skipped, class_ids
