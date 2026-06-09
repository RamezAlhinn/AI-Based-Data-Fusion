"""
frustum_detection ROS package — re-exports from the repo-root library.

The actual implementation lives at /workspace/AI-Based-Data-Fusion/frustum_detection/
(or equivalent repo root). This __init__.py loads frustum_detector.py directly
by file path to avoid circular imports while ensuring the public API is available
regardless of which 'frustum_detection' Python resolves to.
"""

import sys
import os
import importlib.util

# Find the real frustum_detector.py in the repo root
_detector_path = None
for _p in [
    '/workspace/AI-Based-Data-Fusion/frustum_detection/frustum_detector.py',
    '/workspace/frustum_detection/frustum_detector.py',
    os.path.join(os.path.dirname(__file__), '..', '..', '..', '..', '..',
                 'frustum_detection', 'frustum_detector.py'),
]:
    if os.path.isfile(_p):
        _detector_path = os.path.abspath(_p)
        break

if _detector_path is None:
    raise ImportError(
        "Cannot find frustum_detector.py. "
        "Expected at /workspace/AI-Based-Data-Fusion/frustum_detection/frustum_detector.py"
    )

_spec = importlib.util.spec_from_file_location('_frustum_detector_impl', _detector_path)
_mod  = importlib.util.module_from_spec(_spec)
sys.modules['_frustum_detector_impl'] = _mod   # required for @dataclass cls.__module__ lookup
_spec.loader.exec_module(_mod)

FrustumDetector   = _mod.FrustumDetector
Detection3D       = _mod.Detection3D
nms_3d            = _mod.nms_3d
AB3DMOT           = _mod.AB3DMOT
TrackedObject     = _mod.TrackedObject
draw_combined     = _mod.draw_combined
draw_tracks       = _mod.draw_tracks
COCO_TO_INTERNAL  = _mod.COCO_TO_INTERNAL
CLASS_COLORS_BGR  = _mod.CLASS_COLORS_BGR

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
