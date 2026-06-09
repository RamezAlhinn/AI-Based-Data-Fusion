"""
frustum_detection — ROS-independent shared library for Frustum-based 3D detection.

Public API:
    FrustumDetector   — YOLO-guided frustum 3D detector
    Detection3D       — single 3D bounding box detection result
    nms_3d            — BEV centre-distance NMS
    AB3DMOT           — lightweight 3D SORT multi-object tracker
    TrackedObject     — confirmed track output
    draw_combined     — camera + BEV visualisation for detections
    draw_tracks       — camera + BEV visualisation for tracks
"""

from .frustum_detector import (
    FrustumDetector,
    Detection3D,
    nms_3d,
    AB3DMOT,
    TrackedObject,
    draw_combined,
    draw_tracks,
    COCO_TO_INTERNAL,
    CLASS_COLORS_BGR,
)

__all__ = [
    "FrustumDetector",
    "Detection3D",
    "nms_3d",
    "AB3DMOT",
    "TrackedObject",
    "draw_combined",
    "draw_tracks",
    "COCO_TO_INTERNAL",
    "CLASS_COLORS_BGR",
]
