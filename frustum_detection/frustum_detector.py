"""
frustum_detector.py — ROS-independent core of the Frustum + PointPainting pipeline.

This module is intentionally kept free of ROS and rosbag dependencies so it can
be imported and unit-tested anywhere, and shared between:
  - test_pipeline_isolation.py  (offline debugging)
  - ros2_ws/src/frustum_detection/frustum_detection/frustum_node.py  (live ROS2)

Public classes / functions
--------------------------
  FrustumDetector   YOLO-guided frustum 3D detector
  Detection3D       Single 3D bounding box result
  nms_3d            BEV centre-distance NMS
  AB3DMOT           Lightweight 3D SORT tracker
  TrackedObject     Confirmed tracker output
  draw_combined     Camera + BEV visualisation for detections
  draw_tracks       Camera + BEV visualisation for tracks

Class-ID conventions (used throughout)
---------------------------------------
  COCO IDs   — raw YOLO output (person=0, bicycle=1, car=2, motorcycle=3, bus=5, truck=7)
  Internal   — 0=Pedestrian, 1=Cyclist, 2=Car  (trucks/buses folded into Car)
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple, Optional

import cv2
import numpy as np
from scipy.optimize import linear_sum_assignment

# ──────────────────────────────────────────────────────────────────────────────
#  Class-ID mappings
# ──────────────────────────────────────────────────────────────────────────────

# COCO → internal (0=Ped, 1=Cyc, 2=Car)
COCO_TO_INTERNAL: dict[int, int] = {
    0: 0,   # person      → Pedestrian
    1: 1,   # bicycle     → Cyclist
    2: 2,   # car         → Car
    3: 1,   # motorcycle  → Cyclist
    5: 2,   # bus         → Car
    7: 2,   # truck       → Car
}

# Internal class ID → BGR colour for OpenCV drawing
CLASS_COLORS_BGR: dict[int, tuple] = {
    0: (0,   0,   255),   # Pedestrian — red
    1: (255, 0,   0  ),   # Cyclist    — blue
    2: (0,   255, 0  ),   # Car        — green
}
_UNPAINTED_BGR = (80, 80, 80)

# Score-column index inside scored_cloud  [x,y,z,int,ring,s_ped,s_car,s_cyc]
# Matches what paint_points_scored() produces
_SCORE_COL: dict[int, int] = {0: 5, 1: 7, 2: 6}  # internal_cls → column

GROUND_Z_THRESH = -1.5   # metres — Velodyne frame (z-up)


# ──────────────────────────────────────────────────────────────────────────────
#  Detection3D
# ──────────────────────────────────────────────────────────────────────────────

@dataclass
class Detection3D:
    """
    A single 3D bounding box in the LiDAR (Velodyne) coordinate frame.

    Coordinate convention: x=forward, y=left, z=up.
    heading is the yaw angle in radians (counter-clockwise from x-axis).
    cls_id uses **internal** IDs (0=Ped, 1=Cyc, 2=Car), NOT COCO IDs.
    """
    x: float
    y: float
    z: float
    dx: float           # length (x extent)
    dy: float           # width  (y extent)
    dz: float           # height (z extent)
    heading: float      # yaw in radians
    score: float        # YOLO confidence
    cls_id: int         # internal class ID

    # PointPainting filter bookkeeping
    paint_filtered:  bool = False
    paint_kept:      int  = 0
    paint_discarded: int  = 0

    cls_name: str = field(init=False)
    _NAMES = {0: "Pedestrian", 1: "Cyclist", 2: "Car"}

    def __post_init__(self):
        self.cls_name = self._NAMES.get(self.cls_id, str(self.cls_id))

    @property
    def corners_bev(self) -> np.ndarray:
        """4 BEV corner coordinates (x-forward, y-left)."""
        c, s   = np.cos(self.heading), np.sin(self.heading)
        hl, hw = self.dx / 2, self.dy / 2
        corners = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
        R = np.array([[c, -s], [s, c]])
        return (R @ corners.T).T + np.array([self.x, self.y])

    @property
    def box7(self) -> np.ndarray:
        """[cx, cy, cz, dx, dy, dz, heading] — compatible with AB3DMOT."""
        return np.array([self.x, self.y, self.z,
                         self.dx, self.dy, self.dz, self.heading])


# ──────────────────────────────────────────────────────────────────────────────
#  FrustumDetector
# ──────────────────────────────────────────────────────────────────────────────

class FrustumDetector:
    """
    YOLO-guided frustum 3D detector.

    Algorithm (per YOLO instance)
    ------------------------------
    1. Resize the 2-D segmentation mask to full image resolution.
    2. Project every above-ground LiDAR point onto the image plane once.
    3. Select points whose pixel (u, v) falls inside the binary mask AND
       within the class-specific depth limit.
    4. (Optional) PointPainting filter: drop points with paint score < 0.15
       for the detected class (uses scored_cloud if provided).
    5. (Optional) Intra-frustum DBSCAN: keep the dominant cluster only.
    6. Fit an axis-aligned 3-D bounding box; PCA heading for non-pedestrians.

    Guarantees
    ----------
    * 1 YOLO instance → 1 3-D detection (structurally, not by heuristic).
    * The "3 pedestrians from 1 person" and "2 cars" problems cannot occur.

    Parameters
    ----------
    conf_thr   : min YOLO confidence to process an instance
    min_pts    : min LiDAR points inside frustum to produce a detection
    use_dbscan : if True run intra-frustum DBSCAN to strip background clutter
    """

    # (dbscan_eps_m, depth_max_m, depth_window_m) per COCO class.
    # depth_window_m: max range *from the nearest point* inside the frustum.
    # Anything deeper than nearest_depth + window is background and discarded.
    _COCO_CFG: dict[int, tuple] = {
        0: (1.5, 40.0,  3.0),   # person      — max ~0.5m thick; allow 3m window
        1: (1.5, 40.0,  3.0),   # bicycle     — similar to person
        2: (2.5, 80.0,  6.0),   # car         — up to 5m long; allow 6m window
        3: (1.5, 40.0,  3.0),   # motorcycle  — thin object
        5: (3.0, 80.0, 10.0),   # bus         — can be long
        7: (3.0, 80.0,  8.0),   # truck       — longer than car
    }
    _DEFAULT_CFG = (2.0, 60.0, 6.0)

    # Max allowed box dimensions (dx, dy, dz) per internal class (Ped/Cyc/Car).
    # Clamp prevents one bad frame from producing a warehouse-sized box.
    _DIM_MAX: dict[int, tuple] = {
        0: (1.0, 1.0, 2.2),    # Pedestrian — 1m × 1m × 2.2m
        1: (2.0, 1.2, 2.0),    # Cyclist    — 2m × 1.2m × 2m
        2: (6.0, 3.0, 2.5),    # Car        — 6m × 3m × 2.5m
    }

    # Min allowed box dimensions (dx, dy, dz) per internal class.
    # Rejects tiny noise clusters (e.g. 4 points spread over 5cm).
    _DIM_MIN: dict[int, tuple] = {
        0: (0.15, 0.15, 0.4),  # Pedestrian
        1: (0.2, 0.2, 0.4),    # Cyclist
        2: (0.8, 0.6, 0.4),    # Car
    }

    def __init__(self, conf_thr: float = 0.40, min_pts: int = 4,
                 use_dbscan: bool = True):
        self.conf_thr   = conf_thr
        self.min_pts    = min_pts
        self.use_dbscan = use_dbscan

    # ── public API ────────────────────────────────────────────────────────────

    def detect(
        self,
        lidar_filtered: np.ndarray,       # (N, 3|4|8) above-ground LiDAR points
        yolo_results,                      # list returned by model(img_rgb, ...)
        projector,                         # KittiLidarToImageProjector
        img_shape: tuple,                  # (H, W)
        scored_cloud: np.ndarray = None,   # (N, ≥8) painted cloud with class scores
    ) -> List[Detection3D]:
        """
        Run frustum detection on a single frame.

        Parameters
        ----------
        lidar_filtered : above-ground LiDAR points, shape (N, 3+).
                         Columns 0..2 must be x, y, z in Velodyne frame.
        yolo_results   : raw output from YOLO model call.
        projector      : KittiLidarToImageProjector instance.
        img_shape      : (H, W) of the camera frame.
        scored_cloud   : optional painted cloud (N, ≥8).
                         Columns: [x, y, z, intensity, ring, s_ped, s_car, s_cyc].
                         If provided, points with score < 0.15 for the detected
                         class are dropped before DBSCAN / box fitting.

        Returns
        -------
        List[Detection3D] sorted by score descending, **before** NMS.
        Call nms_3d() on the result to suppress duplicates.
        """
        from sklearn.cluster import DBSCAN

        h, w = img_shape

        # ── Project ALL LiDAR points once ─────────────────────────────────────
        cam_pts = projector.lidar_to_camera(lidar_filtered[:, :3])
        front   = cam_pts[:, 2] > 0

        uv_h   = (projector.P2[:, :3] @ cam_pts[front].T).T   # (M, 3)
        depths = uv_h[:, 2]                                     # (M,)
        uv     = uv_h[:, :2] / uv_h[:, 2:3]                   # (M, 2)

        front_idx = np.where(front)[0]

        in_img    = ((uv[:, 0] >= 0) & (uv[:, 0] < w) &
                     (uv[:, 1] >= 0) & (uv[:, 1] < h))
        u_int     = np.clip(uv[in_img, 0].astype(int), 0, w - 1)
        v_int     = np.clip(uv[in_img, 1].astype(int), 0, h - 1)
        depth_in  = depths[in_img]
        lidar_idx = front_idx[in_img]   # indices into lidar_filtered

        dets: List[Detection3D] = []

        # ── One frustum per YOLO instance ─────────────────────────────────────
        for result in yolo_results:
            if result.masks is None:
                continue
            masks   = result.masks.data.cpu().numpy()
            classes = result.boxes.cls.cpu().numpy().astype(int)
            confs   = result.boxes.conf.cpu().numpy()

            for mask, coco_cls, conf_score in zip(masks, classes, confs):
                coco_cls = int(coco_cls)
                if coco_cls not in COCO_TO_INTERNAL:
                    continue
                if float(conf_score) < self.conf_thr:
                    continue

                eps_m, depth_max, depth_win = self._COCO_CFG.get(
                    coco_cls, self._DEFAULT_CFG)
                internal_cls = COCO_TO_INTERNAL[coco_cls]

                # Binary mask at full image resolution
                mask_bin = cv2.resize(
                    (mask * 255).astype(np.uint8), (w, h),
                    interpolation=cv2.INTER_LINEAR,
                ) > 127

                # LiDAR points inside this mask AND within global depth limit
                in_mask = mask_bin[v_int, u_int] & (depth_in < depth_max)
                if in_mask.sum() < self.min_pts:
                    continue

                # ── Depth-window filter (kills background splash) ─────────────
                # Find the nearest occupied depth (5th-percentile to ignore
                # stray LiDAR noise) and discard anything farther than
                # nearest_depth + depth_win.  This is the key fix for
                # points projecting onto a mask but sitting behind the object.
                in_mask_depths = depth_in[in_mask]
                near_depth = float(np.percentile(in_mask_depths, 5))
                depth_ceil = near_depth + depth_win
                depth_ok   = in_mask_depths <= depth_ceil

                # Build final index into lidar_filtered
                in_mask_idx   = np.where(in_mask)[0]          # indices into lidar_idx
                in_mask_idx   = in_mask_idx[depth_ok]
                if len(in_mask_idx) < self.min_pts:
                    continue

                pts3d = lidar_filtered[lidar_idx[in_mask_idx], :3]   # (K, 3)

                # ── PointPainting filter ───────────────────────────────────────
                paint_filtered  = False
                paint_kept      = len(pts3d)
                paint_discarded = 0

                if scored_cloud is not None and scored_cloud.shape[1] >= 8:
                    col_idx        = _SCORE_COL[internal_cls]
                    paint_scores   = scored_cloud[lidar_idx[in_mask_idx], col_idx]
                    # Tighter threshold (0.25) vs original 0.15 — reduces
                    # weakly-painted background points that slipped past the
                    # depth-window filter.
                    keep_paint     = paint_scores > 0.25
                    pts3d_filtered = pts3d[keep_paint]

                    if len(pts3d_filtered) >= self.min_pts:
                        paint_filtered  = True
                        paint_kept      = len(pts3d_filtered)
                        paint_discarded = len(pts3d) - paint_kept
                        pts3d = pts3d_filtered
                    else:
                        paint_discarded = len(pts3d) - len(pts3d_filtered)

                # ── Intra-frustum DBSCAN ───────────────────────────────────────
                if self.use_dbscan and len(pts3d) >= self.min_pts * 2:
                    labels   = DBSCAN(eps=eps_m,
                                      min_samples=self.min_pts).fit_predict(pts3d)
                    valid    = labels[labels >= 0]
                    if len(valid) == 0:
                        continue
                    main_lbl = int(np.argmax(np.bincount(valid)))
                    pts3d    = pts3d[labels == main_lbl]
                    if len(pts3d) < self.min_pts:
                        continue

                # ── Fit 3-D axis-aligned bounding box ─────────────────────────
                mins   = pts3d.min(0)
                maxs   = pts3d.max(0)
                centre = (mins + maxs) / 2.0
                dims   = maxs - mins + 1e-3

                # Clamp dimensions to realistic per-class maxima.
                # This prevents one stray point from inflating the box.
                d_max = self._DIM_MAX.get(internal_cls, (8.0, 4.0, 3.0))
                dims  = np.minimum(dims, np.array(d_max, dtype=np.float32))

                # Enforce minimum dimensions to reject tiny false detections / noise
                d_min = self._DIM_MIN.get(internal_cls, (0.1, 0.1, 0.1))
                if np.any(dims < np.array(d_min, dtype=np.float32)):
                    continue

                # PCA heading (skip for pedestrians — clusters too small)
                heading = 0.0
                if internal_cls != 0 and len(pts3d) >= 4:
                    try:
                        cov     = np.cov(pts3d[:, :2].T)
                        _, evec = np.linalg.eigh(cov)
                        heading = float(np.arctan2(evec[1, -1], evec[0, -1]))
                    except Exception:
                        heading = 0.0

                dets.append(Detection3D(
                    x=float(centre[0]), y=float(centre[1]), z=float(centre[2]),
                    dx=float(dims[0]),  dy=float(dims[1]),  dz=float(dims[2]),
                    heading=heading,
                    score=float(conf_score),
                    cls_id=internal_cls,
                    paint_filtered=paint_filtered,
                    paint_kept=paint_kept,
                    paint_discarded=paint_discarded,
                ))

        dets.sort(key=lambda d: d.score, reverse=True)
        return dets


# ──────────────────────────────────────────────────────────────────────────────
#  NMS
# ──────────────────────────────────────────────────────────────────────────────

def nms_3d(detections: List[Detection3D], dist_thr: float = 1.0) -> List[Detection3D]:
    """
    BEV centre-distance NMS.

    Per-class base radii are intentionally conservative — FrustumDetector
    already guarantees 1-instance-1-detection; NMS only suppresses the rare
    case of two YOLO masks for the same physical object.
    """
    _CLS_BASE = {0: 1.5, 1: 1.5, 2: 2.5}   # Ped / Cyc / Car

    kept = []
    suppressed: set = set()
    for i, di in enumerate(detections):
        if i in suppressed:
            continue
        kept.append(di)
        for j, dj in enumerate(detections):
            if j <= i or j in suppressed or di.cls_id != dj.cls_id:
                continue
            base      = _CLS_BASE.get(di.cls_id, dist_thr)
            radius    = max(di.dx + di.dy, dj.dx + dj.dy) / 4.0
            threshold = max(base, radius)
            if np.hypot(di.x - dj.x, di.y - dj.y) < threshold:
                suppressed.add(j)
    return kept


# ──────────────────────────────────────────────────────────────────────────────
#  AB3DMOT tracker
# ──────────────────────────────────────────────────────────────────────────────

class KalmanBox3D:
    """
    Constant-velocity Kalman filter for a single 3D box.
    State  x = [cx, cy, cz, dx, dy, dz, heading, vcx, vcy, vcz]   (10-dim)
    Measurement z = [cx, cy, cz, dx, dy, dz, heading]              (7-dim)
    """
    _Q = np.diag([1.] * 10)
    _R = np.diag([1.] * 7)

    def __init__(self, box7: np.ndarray):
        self.F = np.eye(10)
        self.F[0, 7] = self.F[1, 8] = self.F[2, 9] = 1.0
        self.H = np.zeros((7, 10))
        for i in range(7):
            self.H[i, i] = 1.0
        self.x = np.zeros((10, 1))
        self.x[:7, 0] = box7
        self.P = np.eye(10) * 10.0
        self.P[7:, 7:] *= 1000.0
        self.Q = self._Q.copy()
        self.R = self._R.copy()

    def predict(self) -> np.ndarray:
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:7, 0].copy()

    def update(self, z: np.ndarray) -> None:
        z = z.copy()
        pθ = self.x[6, 0]
        while z[6] - pθ >  np.pi / 2: z[6] -= np.pi
        while z[6] - pθ < -np.pi / 2: z[6] += np.pi
        y = z.reshape(-1, 1) - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(10) - K @ self.H) @ self.P

    @property
    def state(self) -> np.ndarray:
        return self.x[:7, 0].copy()


_NEXT_TRACK_ID = 1


@dataclass
class _Track:
    kf: KalmanBox3D
    track_id: int
    cls_id: int
    hits: int    = 1
    no_match: int = 0

    @classmethod
    def spawn(cls, box7: np.ndarray, cls_id: int) -> "_Track":
        global _NEXT_TRACK_ID
        t = cls(kf=KalmanBox3D(box7), track_id=_NEXT_TRACK_ID, cls_id=cls_id)
        _NEXT_TRACK_ID += 1
        return t

    @property
    def box7(self) -> np.ndarray:
        return self.kf.state


@dataclass
class TrackedObject:
    """A confirmed tracked object from AB3DMOT."""
    track_id: int
    cls_id: int
    cls_name: str
    box7: np.ndarray   # [cx, cy, cz, dx, dy, dz, heading]
    score: float
    _NAMES = {0: "Pedestrian", 1: "Cyclist", 2: "Car"}


def _bev_iou(a: np.ndarray, b: np.ndarray) -> float:
    ix = max(0, min(a[0]+a[3]/2, b[0]+b[3]/2) - max(a[0]-a[3]/2, b[0]-b[3]/2))
    iy = max(0, min(a[1]+a[4]/2, b[1]+b[4]/2) - max(a[1]-a[4]/2, b[1]-b[4]/2))
    inter = ix * iy
    union = a[3]*a[4] + b[3]*b[4] - inter
    return inter / union if union > 0 else 0.0


def _assoc_cost(tb: np.ndarray, db: np.ndarray, iou_thr: float) -> Tuple[float, bool]:
    iou = _bev_iou(tb, db)
    if iou >= iou_thr:
        return 1.0 - iou, True
    dist = float(np.hypot(tb[0] - db[0], tb[1] - db[1]))
    gate = (db[3] + db[4]) / 2.0
    if dist < gate:
        return 0.5 + 0.5 * (dist / gate), True
    return 1.0, False


class AB3DMOT:
    """
    Lightweight 3D SORT tracker.

    Uses BEV IoU + centre-distance gating for association, and a
    constant-velocity Kalman filter per track.
    """

    def __init__(self, iou_threshold: float = 0.25,
                 max_age: int = 3, min_hits: int = 3):
        self.iou_thr    = iou_threshold
        self.max_age    = max_age
        self.min_hits   = min_hits
        self.tracks: List[_Track] = []
        self.frame_count = 0

    def update(self, detections: List[Detection3D]) -> List[TrackedObject]:
        """
        Update tracker with new detections.

        Parameters
        ----------
        detections : list from FrustumDetector (already NMS-filtered).

        Returns
        -------
        List of confirmed TrackedObject for this frame.
        """
        self.frame_count += 1
        for t in self.tracks:
            t.kf.predict()

        det_boxes  = [d.box7 for d in detections]
        det_scores = [d.score  for d in detections]
        det_cls    = [d.cls_id for d in detections]

        matched_t: set = set()
        matched_d: set = set()
        track_to_det: dict = {}

        if self.tracks and det_boxes:
            T, D = len(self.tracks), len(det_boxes)
            cost  = np.ones((T, D))
            valid = np.zeros((T, D), dtype=bool)
            for ti, trk in enumerate(self.tracks):
                for di, db in enumerate(det_boxes):
                    c, v = _assoc_cost(trk.box7, db, self.iou_thr)
                    cost[ti, di] = c
                    valid[ti, di] = v
            cm = cost.copy()
            cm[~valid] = 1e6
            rows, cols = linear_sum_assignment(cm)
            for r, c in zip(rows, cols):
                if valid[r, c]:
                    matched_t.add(r)
                    matched_d.add(c)
                    track_to_det[r] = c
                    self.tracks[r].kf.update(det_boxes[c])
                    self.tracks[r].hits    += 1
                    self.tracks[r].no_match = 0

        for ti, trk in enumerate(self.tracks):
            if ti not in matched_t:
                trk.no_match += 1

        for di, (b7, cls) in enumerate(zip(det_boxes, det_cls)):
            if di not in matched_d:
                self.tracks.append(_Track.spawn(b7, cls))

        self.tracks = [t for t in self.tracks if t.no_match <= self.max_age]

        confirmed: List[TrackedObject] = []
        for ti, trk in enumerate(self.tracks):
            if trk.hits >= self.min_hits or self.frame_count <= self.min_hits:
                di    = track_to_det.get(ti)
                score = det_scores[di] if di is not None else 0.0
                confirmed.append(TrackedObject(
                    track_id=trk.track_id,
                    cls_id=trk.cls_id,
                    cls_name=TrackedObject._NAMES.get(trk.cls_id, str(trk.cls_id)),
                    box7=trk.box7,
                    score=score,
                ))
        return confirmed


# ──────────────────────────────────────────────────────────────────────────────
#  Visualisation helpers
# ──────────────────────────────────────────────────────────────────────────────

def _box_corners_3d(det: Detection3D) -> np.ndarray:
    """8 corners of the 3D box in LiDAR frame (x-forward, y-left, z-up)."""
    c, s = np.cos(det.heading), np.sin(det.heading)
    hl, hw, hh = det.dx / 2.0, det.dy / 2.0, det.dz / 2.0
    loc = np.array([
        [ hl,  hw, -hh], [ hl, -hw, -hh], [-hl, -hw, -hh], [-hl,  hw, -hh],
        [ hl,  hw,  hh], [ hl, -hw,  hh], [-hl, -hw,  hh], [-hl,  hw,  hh],
    ], dtype=np.float32)
    R = np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]], dtype=np.float32)
    return (R @ loc.T).T + np.array([det.x, det.y, det.z], dtype=np.float32)


_BOX_EDGES = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,6),(6,7),(7,4),
    (0,4),(1,5),(2,6),(3,7),
]


def _draw_bev_panel(
    detections: List[Detection3D],
    tracked: List[TrackedObject],
    range_m: float,
    bev_size: int,
) -> np.ndarray:
    """Shared BEV panel renderer."""
    bev   = np.zeros((bev_size, bev_size, 3), dtype=np.uint8)
    scale = bev_size / (2 * range_m)
    ox, oy = bev_size // 2, bev_size // 2

    # Range rings + grid
    for r in [10, 20, 30, 40, 50]:
        cv2.circle(bev, (ox, oy), int(r * scale), (30, 30, 30), 1)
        cv2.putText(bev, f"{r}m", (ox + 2, oy - int(r * scale) + 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.3, (60, 60, 60), 1)
    cv2.line(bev, (ox, 0), (ox, bev_size), (30, 30, 30), 1)
    cv2.line(bev, (0, oy), (bev_size, oy), (30, 30, 30), 1)
    cv2.putText(bev, "FWD", (ox + 4, 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (80, 80, 80), 1)
    # Ego vehicle
    cv2.rectangle(bev, (ox - 6, oy - 10), (ox + 6, oy + 10), (100, 100, 100), -1)

    # Detection footprints (thin)
    for det in detections:
        color = CLASS_COLORS_BGR.get(det.cls_id, _UNPAINTED_BGR)
        pts   = np.stack([
            (ox + det.corners_bev[:, 1] * scale).astype(np.int32),
            (oy - det.corners_bev[:, 0] * scale).astype(np.int32),
        ], axis=1)
        cv2.polylines(bev, [pts], isClosed=True, color=color, thickness=1)

    # Tracked object footprints (thick) + IDs
    for t in tracked:
        cx, cy, _, dx, dy, _, heading = t.box7
        c, s   = np.cos(heading), np.sin(heading)
        hl, hw = dx / 2, dy / 2
        cors   = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
        R      = np.array([[c, -s], [s, c]])
        cors_bev = (R @ cors.T).T + np.array([cx, cy])

        color = CLASS_COLORS_BGR.get(t.cls_id, _UNPAINTED_BGR)
        pts   = np.stack([
            (ox + cors_bev[:, 1] * scale).astype(np.int32),
            (oy - cors_bev[:, 0] * scale).astype(np.int32),
        ], axis=1)
        cv2.polylines(bev, [pts], isClosed=True, color=color, thickness=2)
        cu  = int(ox + cy * scale)
        cv_ = int(oy - cx * scale)
        cv2.circle(bev, (cu, cv_), 6, color, -1)
        label = f"{t.cls_name} ID:{t.track_id}"
        if t.score > 0:
            label += f" {t.score:.2f}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.38, 1)
        tx = int(np.clip(cu + 8, 2, bev_size - tw - 2))
        ty = int(np.clip(cv_ - 8, th + 2, bev_size - 2))
        cv2.putText(bev, label, (tx, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.38, (0, 255, 255), 1)
    return bev


def draw_combined(
    img_bgr: np.ndarray,
    detections: List[Detection3D],
    projector,
    tracked: Optional[List[TrackedObject]] = None,
    range_m: float = 60.0,
    bev_size: int  = 700,
) -> np.ndarray:
    """
    Combined visualisation: camera panel (top) + BEV panel (bottom).

    Parameters
    ----------
    img_bgr    : camera image in BGR.
    detections : list of Detection3D to draw wireframe boxes.
    projector  : KittiLidarToImageProjector (needs .lidar_to_camera and .P2).
    tracked    : optional tracker output; drawn as thick boxes + IDs in BEV.
    range_m    : BEV range in metres (half-width).
    bev_size   : pixel size of the square BEV panel.
    """
    h_cam, w_cam = img_bgr.shape[:2]
    cam = img_bgr.copy()

    def _proj(pt3d):
        cp = projector.lidar_to_camera(pt3d.reshape(1, 3).astype(np.float32))
        if cp[0, 2] <= 0:
            return None
        uv = (projector.P2[:, :3] @ cp.T).T
        uv = uv[:, :2] / uv[:, 2:3]
        u, v = int(uv[0, 0]), int(uv[0, 1])
        if -300 < u < w_cam + 300 and -300 < v < h_cam + 300:
            return (u, v)
        return None

    for det in detections:
        color   = CLASS_COLORS_BGR.get(det.cls_id, _UNPAINTED_BGR)
        corners = _box_corners_3d(det)
        proj_c  = [_proj(corners[i]) for i in range(8)]
        for i, j in _BOX_EDGES:
            if proj_c[i] and proj_c[j]:
                cv2.line(cam, proj_c[i], proj_c[j], color, 2, cv2.LINE_AA)
        anchor = proj_c[4] or next((p for p in proj_c if p), None)
        if anchor:
            label = f"{det.cls_name} {det.score:.2f}"
            if det.paint_filtered:
                label += f" [PP↑{det.paint_kept}]"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ax = int(np.clip(anchor[0] + 3, 0, w_cam - tw - 1))
            ay = int(np.clip(anchor[1] - 4, th + 1, h_cam - 1))
            cv2.rectangle(cam, (ax - 1, ay - th - 1), (ax + tw + 1, ay + 2), (0, 0, 0), -1)
            cv2.putText(cam, label, (ax, ay),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    # Add track IDs on camera panel
    if tracked:
        for t in tracked:
            uv = _proj(t.box7[:3])
            if uv is None:
                continue
            u, v = uv
            color = CLASS_COLORS_BGR.get(t.cls_id, _UNPAINTED_BGR)
            label = f"{t.cls_name} ID:{t.track_id}"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
            cv2.rectangle(cam, (u - 2, v - th - 6), (u + tw + 2, v + 2), (0, 0, 0), -1)
            cv2.putText(cam, label, (u, v - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
            cv2.circle(cam, (u, v), 6, color, -1)

    scale_cam = bev_size / w_cam
    cam_panel = cv2.resize(cam, (bev_size, int(h_cam * scale_cam)),
                            interpolation=cv2.INTER_AREA)

    bev = _draw_bev_panel(detections, tracked or [], range_m, bev_size)

    sep = np.full((4, bev_size, 3), 80, dtype=np.uint8)
    return np.vstack([cam_panel, sep, bev])


def draw_tracks(
    img_bgr: np.ndarray,
    detections: List[Detection3D],
    tracked: List[TrackedObject],
    projector,
    range_m: float = 60.0,
    bev_size: int  = 700,
) -> np.ndarray:
    """
    Alias for draw_combined with tracked objects always shown.
    Kept for backwards compatibility with existing scripts.
    """
    return draw_combined(img_bgr, detections, projector, tracked, range_m, bev_size)
