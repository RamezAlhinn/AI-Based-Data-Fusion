"""
test_pipeline_frustum.py  —  Full Pipeline with Frustum-based 3D Detection
===========================================================================

Key difference from test_pipeline_isolation.py
-----------------------------------------------
Stage 6 uses FrustumDetector instead of ClusterDetector:

  ClusterDetector  (old):
    DBSCAN on score-enriched point cloud per class channel
    → same body can generate 3 clusters (one per channel)
    → 3 Pedestrian detections from 1 person

  FrustumDetector  (new):
    Each YOLO 2D instance mask → select LiDAR points projecting into it
    → optional intra-frustum DBSCAN to discard background clutter
    → fit 3D box to the foreground cluster
    → guaranteed: 1 YOLO instance = 1 3D detection

Why this fixes the two reported problems
-----------------------------------------
  Problem 1 – "3 pedestrian detections for 1 person":
    YOLO detects the person ONCE → 1 mask → 1 frustum → 1 3D box.
    Fragmentation is structurally impossible.

  Problem 2 – "2 cars in tracker":
    If YOLO produces 1 mask for the car → 1 3D box → 1 tracker ID.
    If it produces 2 masks (front/back separately), NMS fuses them.

Stage 5 is lighter: score maps saved for visualisation but no full
point-cloud enrichment (FrustumDetector works from raw lidar_filtered).

Run modes
---------
  python3 test_pipeline_frustum.py --seed 7
  python3 test_pipeline_frustum.py --all-frames
"""

# ══════════════════════════════════════════════════════════════════════════════
#  Imports
# ══════════════════════════════════════════════════════════════════════════════
import sys
import os
sys.path.insert(0, "/workspace")
import random
import argparse
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Tuple, Optional

import numpy as np
import cv2
from scipy.optimize import linear_sum_assignment

# ══════════════════════════════════════════════════════════════════════════════
#  CLI
# ══════════════════════════════════════════════════════════════════════════════
parser = argparse.ArgumentParser()
parser.add_argument("--seed",       type=int,   default=None)
parser.add_argument("--checkpoint", type=str,   default=None,
                    help="YOLO weights file")
parser.add_argument("--all-frames", action="store_true")
parser.add_argument("--conf",       type=float, default=0.40,
                    help="YOLO confidence threshold")
parser.add_argument("--min-pts",    type=int,   default=4,
                    help="Min LiDAR points inside frustum to keep detection")
parser.add_argument("--iou-thr",    type=float, default=0.25)
parser.add_argument("--max-age",    type=int,   default=3)
parser.add_argument("--min-hits",   type=int,   default=3)
parser.add_argument("--nms-dist",   type=float, default=1.0)
args = parser.parse_args()

rng = random.Random(args.seed)

# ══════════════════════════════════════════════════════════════════════════════
#  Paths
# ══════════════════════════════════════════════════════════════════════════════
BAG_PATH   = "/tmp/studentProject1/"
CALIB_PATH = "/workspace/AI-Based-Data-Fusion/calib.txt"
OUTPUT_DIR = "/workspace/AI-Based-Data-Fusion/frustum_output"
_script_dir = os.path.dirname(os.path.abspath(__file__))
YOLO_MODEL  = args.checkpoint or os.path.join(_script_dir, "yolo11m-seg.pt")

os.makedirs(OUTPUT_DIR, exist_ok=True)

# ══════════════════════════════════════════════════════════════════════════════
#  Constants
# ══════════════════════════════════════════════════════════════════════════════
# COCO class indices
CLS_CAR, CLS_PED, CLS_CYC = 2, 0, 1
SCORE_CLASSES = [CLS_CAR, CLS_PED, CLS_CYC]   # for score-map visualisation

# COCO → internal class ID (0=Ped, 1=Cyc, 2=Car)
# Trucks and buses are folded into Car
COCO_TO_INTERNAL = {0: 0, 1: 1, 2: 2, 3: 1, 5: 2, 7: 2}

CLASS_COLORS_BGR = {
    0: (0, 0, 255),    # person      — red
    1: (255, 0, 0),    # bicycle     — blue
    2: (0, 255, 0),    # car         — green
    3: (0, 128, 255),  # motorcycle  — orange
    5: (0, 255, 255),  # bus         — yellow
    7: (0, 200, 0),    # truck       — dark green
}
UNPAINTED_BGR   = (80, 80, 80)
GROUND_Z_THRESH = -1.5

IMG_TOPIC   = "/blackfly_s/cam0/image_rectified"
LIDAR_TOPIC = "/velodyne/points_raw"


# ══════════════════════════════════════════════════════════════════════════════
#  Stage 2 helper — score maps (visualisation only)
# ══════════════════════════════════════════════════════════════════════════════

def build_score_maps(img_rgb: np.ndarray, model, conf: float = 0.40):
    """
    Run YOLO once; return per-class soft score maps + raw results.

    Per-pixel winner-take-all (BEFORE blur) ensures car pixels cannot
    contaminate the pedestrian score map and vice-versa.
    """
    h, w = img_rgb.shape[:2]
    results = model(img_rgb, verbose=False, conf=conf, imgsz=(h, w))

    # Collect (weighted_mask, cls_id) per instance
    instance_scores: List[Tuple[np.ndarray, int]] = []
    for result in results:
        if result.masks is None:
            continue
        masks   = result.masks.data.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        confs   = result.boxes.conf.cpu().numpy()
        for mask, cls_id, c in zip(masks, classes, confs):
            if cls_id not in SCORE_CLASSES:
                continue
            m = cv2.resize((mask * 255).astype(np.uint8), (w, h),
                           interpolation=cv2.INTER_LINEAR).astype(np.float32) / 255.0
            instance_scores.append((m * float(c), cls_id))

    score_maps = {c: np.zeros((h, w), dtype=np.float32) for c in SCORE_CLASSES}
    if instance_scores:
        stack    = np.stack([s for s, _ in instance_scores], axis=0)  # (N,H,W)
        winner   = np.argmax(stack, axis=0)                            # (H,W)
        any_det  = stack.max(axis=0) > 0
        for idx, (sm, cls_id) in enumerate(instance_scores):
            owns = (winner == idx) & any_det
            score_maps[cls_id] = np.maximum(score_maps[cls_id],
                                            np.where(owns, sm, 0.).astype(np.float32))

    for cls_id in SCORE_CLASSES:
        score_maps[cls_id] = cv2.GaussianBlur(score_maps[cls_id], (5, 5), 0)

    return score_maps, results


# ══════════════════════════════════════════════════════════════════════════════
#  Stage 6 — Detection3D dataclass
# ══════════════════════════════════════════════════════════════════════════════

@dataclass
class Detection3D:
    x: float; y: float; z: float
    dx: float; dy: float; dz: float
    heading: float
    score: float
    cls_id: int                          # internal ID: 0=Ped, 1=Cyc, 2=Car
    paint_filtered: bool = False
    paint_kept: int = 0
    paint_discarded: int = 0
    cls_name: str = field(init=False)
    # cls_id uses INTERNAL IDs, not COCO
    _NAMES = {0: "Pedestrian", 1: "Cyclist", 2: "Car"}

    def __post_init__(self):
        self.cls_name = self._NAMES.get(self.cls_id, str(self.cls_id))

    @property
    def corners_bev(self) -> np.ndarray:
        c, s   = np.cos(self.heading), np.sin(self.heading)
        hl, hw = self.dx / 2, self.dy / 2
        corners = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
        R = np.array([[c, -s], [s, c]])
        return (R @ corners.T).T + np.array([self.x, self.y])


# ══════════════════════════════════════════════════════════════════════════════
#  Stage 6 — FrustumDetector  ← KEY CHANGE
# ══════════════════════════════════════════════════════════════════════════════

class FrustumDetector:
    """
    YOLO-guided frustum 3D detector.

    Algorithm per YOLO instance
    ----------------------------
    1. Resize the 2D segmentation mask to full image resolution.
    2. Project every above-ground LiDAR point to image coordinates.
    3. Select points whose pixel (u,v) falls inside the binary mask.
    4. (Optional) Run intra-frustum DBSCAN on those 3D points to isolate the
       foreground cluster and discard background / occluded objects that
       accidentally project into the same mask area.
    5. Fit an axis-aligned 3D bounding box to the dominant cluster.

    Guarantees
    ----------
    * 1 YOLO instance  →  1 3D detection (structurally, not by heuristic).
    * Cross-class contamination is impossible: each frustum is instance-specific.
    * The two-car / three-pedestrian problem is eliminated at the source.

    Parameters
    ----------
    conf_thr    : min YOLO detection confidence to attempt 3D fitting
    min_pts     : min LiDAR points inside mask to produce a 3D box
    use_dbscan  : if True, run intra-frustum DBSCAN to discard BG clutter
    """

    # Per COCO class: (dbscan_eps_m, depth_max_m)
    # depth_max limits how far we look for each class (avoids phantom detections
    # from LiDAR points on the far end of the frustum that belong to another obj)
    _COCO_CFG = {
        0: (1.5, 40.0),   # person       — tight eps, 40 m max
        1: (1.5, 40.0),   # bicycle
        2: (2.5, 80.0),   # car
        3: (1.5, 40.0),   # motorcycle
        5: (3.0, 80.0),   # bus
        7: (3.0, 80.0),   # truck
    }
    _DEFAULT_CFG = (2.0, 60.0)

    def __init__(self, conf_thr: float = 0.40, min_pts: int = 4,
                 use_dbscan: bool = True):
        self.conf_thr  = conf_thr
        self.min_pts   = min_pts
        self.use_dbscan = use_dbscan

    # ── public API ────────────────────────────────────────────────────────────

    def detect(
        self,
        lidar_filtered: np.ndarray,   # (N, 3|4)  above-ground LiDAR points
        yolo_results,                  # list from model(img_rgb, ...)
        projector,                     # KittiLidarToImageProjector
        img_shape: tuple,              # (H, W)
        painted_cloud: np.ndarray = None,
    ) -> List[Detection3D]:
        from sklearn.cluster import DBSCAN

        h, w = img_shape

        # ── Project ALL LiDAR points once ─────────────────────────────────────
        cam_pts = projector.lidar_to_camera(lidar_filtered[:, :3])
        front   = cam_pts[:, 2] > 0

        uv_h   = (projector.P2[:, :3] @ cam_pts[front].T).T   # (M,3)
        depths = uv_h[:, 2]                                    # (M,)
        uv     = uv_h[:, :2] / uv_h[:, 2:3]                  # (M,2)

        front_idx = np.where(front)[0]

        in_img = ((uv[:, 0] >= 0) & (uv[:, 0] < w) &
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

                eps_m, depth_max = self._COCO_CFG.get(coco_cls, self._DEFAULT_CFG)
                internal_cls = COCO_TO_INTERNAL[coco_cls]

                # Binary mask at full resolution
                mask_bin = cv2.resize(
                    (mask * 255).astype(np.uint8), (w, h),
                    interpolation=cv2.INTER_LINEAR
                ) > 127

                # LiDAR points inside this mask AND within depth limit
                in_mask = mask_bin[v_int, u_int] & (depth_in < depth_max)
                if in_mask.sum() < self.min_pts:
                    continue

                pts3d = lidar_filtered[lidar_idx[in_mask], :3]   # (K,3)

                paint_filtered = False
                paint_kept = len(pts3d)
                paint_discarded = 0

                if painted_cloud is not None:
                    score_col_map = {0: 5, 1: 6, 2: 7}
                    col_idx = score_col_map[internal_cls]
                    painting_scores = painted_cloud[lidar_idx[in_mask], col_idx]
                    keep_mask_paint = painting_scores > 0.15
                    pts3d_filtered = pts3d[keep_mask_paint]
                    
                    if len(pts3d_filtered) >= self.min_pts:
                        paint_filtered = True
                        paint_kept = len(pts3d_filtered)
                        paint_discarded = len(pts3d) - len(pts3d_filtered)
                        pts3d = pts3d_filtered
                    else:
                        paint_filtered = False
                        paint_kept = len(pts3d)
                        paint_discarded = len(pts3d) - len(pts3d_filtered)

                # ── Intra-frustum DBSCAN: keep largest cluster (foreground) ──
                if self.use_dbscan and len(pts3d) >= self.min_pts * 2:
                    labels = DBSCAN(
                        eps=eps_m, min_samples=self.min_pts
                    ).fit_predict(pts3d)
                    valid = labels[labels >= 0]
                    if len(valid) == 0:
                        continue
                    main_lbl = int(np.argmax(np.bincount(valid)))
                    pts3d = pts3d[labels == main_lbl]
                    if len(pts3d) < self.min_pts:
                        continue

                # ── Fit 3D axis-aligned bounding box ──────────────────────────
                mins   = pts3d.min(0)
                maxs   = pts3d.max(0)
                centre = (mins + maxs) / 2.0
                dims   = maxs - mins + 1e-3

                # PCA-based heading calculation (skip for pedestrians, internal_cls == 0)
                heading = 0.0
                if internal_cls != 0 and len(pts3d) >= 4:
                    try:
                        cov = np.cov(pts3d[:, :2].T)
                        evals, evecs = np.linalg.eigh(cov)
                        heading = float(np.arctan2(evecs[1, -1], evecs[0, -1]))
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


# ══════════════════════════════════════════════════════════════════════════════
#  NMS  (light version — fewer duplicates expected with frustum approach)
# ══════════════════════════════════════════════════════════════════════════════

def nms_3d(detections: List[Detection3D], dist_thr: float = 1.0) -> List[Detection3D]:
    """
    BEV centre-distance NMS.  Per-class base radii (smaller than ClusterDetector
    version because FrustumDetector already guarantees 1-instance-1-detection;
    NMS here only handles the rare case of two YOLO masks for the same object).
    """
    _CLS_BASE = {0: 1.5, 1: 1.5, 2: 2.5}   # Ped / Cyc / Car

    kept = []
    suppressed = set()
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


# ══════════════════════════════════════════════════════════════════════════════
#  Stage 7 — AB3DMOT tracker  (unchanged from test_pipeline_isolation.py)
# ══════════════════════════════════════════════════════════════════════════════

class KalmanBox3D:
    _Q = np.diag([1.]*10)
    _R = np.diag([1.]*7)

    def __init__(self, box7: np.ndarray):
        self.F = np.eye(10);  self.F[0,7] = self.F[1,8] = self.F[2,9] = 1.
        self.H = np.zeros((7,10))
        for i in range(7): self.H[i,i] = 1.
        self.x = np.zeros((10,1));  self.x[:7,0] = box7
        self.P = np.eye(10)*10.;    self.P[7:,7:] *= 1000.
        self.Q, self.R = self._Q.copy(), self._R.copy()

    def predict(self):
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.x[:7,0].copy()

    def update(self, z: np.ndarray):
        z = z.copy()
        pθ = self.x[6,0]
        while z[6]-pθ >  np.pi/2: z[6] -= np.pi
        while z[6]-pθ < -np.pi/2: z[6] += np.pi
        y = z.reshape(-1,1) - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(10) - K @ self.H) @ self.P

    @property
    def state(self): return self.x[:7,0].copy()


_NEXT_TRACK_ID = 1

@dataclass
class _Track:
    kf: KalmanBox3D; track_id: int; cls_id: int
    hits: int = 1;   no_match: int = 0

    @classmethod
    def spawn(cls, box7, cls_id):
        global _NEXT_TRACK_ID
        t = cls(kf=KalmanBox3D(box7), track_id=_NEXT_TRACK_ID, cls_id=cls_id)
        _NEXT_TRACK_ID += 1
        return t

    @property
    def box7(self): return self.kf.state


@dataclass
class TrackedObject:
    track_id: int; cls_id: int; cls_name: str
    box7: np.ndarray; score: float
    _NAMES = {0: "Pedestrian", 1: "Cyclist", 2: "Car"}


def _bev_iou(a, b):
    ix = max(0, min(a[0]+a[3]/2, b[0]+b[3]/2) - max(a[0]-a[3]/2, b[0]-b[3]/2))
    iy = max(0, min(a[1]+a[4]/2, b[1]+b[4]/2) - max(a[1]-a[4]/2, b[1]-b[4]/2))
    inter = ix*iy;  union = a[3]*a[4] + b[3]*b[4] - inter
    return inter/union if union > 0 else 0.


def _assoc_cost(tb, db, iou_thr):
    iou = _bev_iou(tb, db)
    if iou >= iou_thr:
        return 1.-iou, True
    dist = np.hypot(tb[0]-db[0], tb[1]-db[1])
    gate = (db[3]+db[4])/2.
    if dist < gate:
        return 0.5 + 0.5*(dist/gate), True
    return 1., False


class AB3DMOT:
    def __init__(self, iou_threshold=0.25, max_age=3, min_hits=3):
        self.iou_thr   = iou_threshold
        self.max_age   = max_age
        self.min_hits  = min_hits
        self.tracks: List[_Track] = []
        self.frame_count = 0

    def update(self, detections: List[Detection3D]) -> List[TrackedObject]:
        self.frame_count += 1
        for t in self.tracks: t.kf.predict()

        det_boxes  = [np.array([d.x,d.y,d.z,d.dx,d.dy,d.dz,d.heading]) for d in detections]
        det_scores = [d.score   for d in detections]
        det_cls    = [d.cls_id  for d in detections]

        matched_t, matched_d, track_to_det = set(), set(), {}
        if self.tracks and det_boxes:
            T, D = len(self.tracks), len(det_boxes)
            cost  = np.ones((T,D));  valid = np.zeros((T,D), dtype=bool)
            for ti,trk in enumerate(self.tracks):
                for di,db in enumerate(det_boxes):
                    c,v = _assoc_cost(trk.box7, db, self.iou_thr)
                    cost[ti,di]=c;  valid[ti,di]=v
            cm = cost.copy();  cm[~valid] = 1e6
            rows, cols = linear_sum_assignment(cm)
            for r,c in zip(rows,cols):
                if valid[r,c]:
                    matched_t.add(r);  matched_d.add(c);  track_to_det[r]=c
                    self.tracks[r].kf.update(det_boxes[c])
                    self.tracks[r].hits += 1;  self.tracks[r].no_match = 0

        for ti,trk in enumerate(self.tracks):
            if ti not in matched_t: trk.no_match += 1

        for di,(b7,cls) in enumerate(zip(det_boxes,det_cls)):
            if di not in matched_d:
                self.tracks.append(_Track.spawn(b7,cls))

        self.tracks = [t for t in self.tracks if t.no_match <= self.max_age]

        confirmed: List[TrackedObject] = []
        for ti,trk in enumerate(self.tracks):
            if trk.hits >= self.min_hits or self.frame_count <= self.min_hits:
                di    = track_to_det.get(ti)
                score = det_scores[di] if di is not None else 0.
                confirmed.append(TrackedObject(
                    track_id=trk.track_id, cls_id=trk.cls_id,
                    cls_name=TrackedObject._NAMES.get(trk.cls_id, str(trk.cls_id)),
                    box7=trk.box7, score=score,
                ))
        return confirmed


# ══════════════════════════════════════════════════════════════════════════════
#  Visualisation helpers
# ══════════════════════════════════════════════════════════════════════════════

def _box_corners_3d(det: Detection3D) -> np.ndarray:
    c, s = np.cos(det.heading), np.sin(det.heading)
    hl, hw, hh = det.dx/2., det.dy/2., det.dz/2.
    loc = np.array([
        [ hl, hw,-hh],[ hl,-hw,-hh],[-hl,-hw,-hh],[-hl, hw,-hh],
        [ hl, hw, hh],[ hl,-hw, hh],[-hl,-hw, hh],[-hl, hw, hh],
    ], dtype=np.float32)
    R = np.array([[c,-s,0.],[s,c,0.],[0.,0.,1.]], dtype=np.float32)
    return (R @ loc.T).T + np.array([det.x, det.y, det.z], dtype=np.float32)


_BOX_EDGES = [
    (0,1),(1,2),(2,3),(3,0),
    (4,5),(5,6),(6,7),(7,4),
    (0,4),(1,5),(2,6),(3,7),
]

# Internal class ID → BGR colour
_IDET_COLORS = {0: (0,0,255), 1: (255,0,0), 2: (0,255,0)}  # Ped/Cyc/Car


def draw_combined(
    img_bgr: np.ndarray,
    detections: List[Detection3D],
    projector,
    range_m: float = 60.,
    bev_size: int  = 700,
) -> np.ndarray:
    """
    Combined visualisation:
      top    — camera image with 3D wireframe boxes
      bottom — BEV with top-down footprints + range rings
    """
    h_cam, w_cam = img_bgr.shape[:2]

    # ── Camera panel ──────────────────────────────────────────────────────────
    cam = img_bgr.copy()

    def _proj(pt3d):
        cp = projector.lidar_to_camera(pt3d.reshape(1,3).astype(np.float32))
        if cp[0,2] <= 0: return None
        uv = (projector.P2[:,:3] @ cp.T).T
        uv = uv[:,:2] / uv[:,2:3]
        u, v = int(uv[0,0]), int(uv[0,1])
        if -300 < u < w_cam+300 and -300 < v < h_cam+300: return (u,v)
        return None

    for det in detections:
        color   = _IDET_COLORS.get(det.cls_id, (200,200,200))
        corners = _box_corners_3d(det)
        proj_c  = [_proj(corners[i]) for i in range(8)]

        for i,j in _BOX_EDGES:
            if proj_c[i] and proj_c[j]:
                cv2.line(cam, proj_c[i], proj_c[j], color, 2, cv2.LINE_AA)

        anchor = proj_c[4] or next((p for p in proj_c if p), None)
        if anchor:
            label = f"{det.cls_name} {det.score:.2f}"
            (tw,th),_ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            ax = int(np.clip(anchor[0]+3, 0, w_cam-tw-1))
            ay = int(np.clip(anchor[1]-4, th+1, h_cam-1))
            cv2.rectangle(cam,(ax-1,ay-th-1),(ax+tw+1,ay+2),(0,0,0),-1)
            cv2.putText(cam, label, (ax,ay),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

    scale_cam  = bev_size / w_cam
    cam_panel  = cv2.resize(cam, (bev_size, int(h_cam*scale_cam)),
                             interpolation=cv2.INTER_AREA)

    # ── BEV panel ─────────────────────────────────────────────────────────────
    bev    = np.zeros((bev_size, bev_size, 3), dtype=np.uint8)
    scale  = bev_size / (2*range_m)
    ox, oy = bev_size//2, bev_size//2

    for r in [10,20,30,40,50]:
        cv2.circle(bev,(ox,oy),int(r*scale),(30,30,30),1)
        cv2.putText(bev,f"{r}m",(ox+2,oy-int(r*scale)+12),
                    cv2.FONT_HERSHEY_SIMPLEX,0.3,(60,60,60),1)
    cv2.line(bev,(ox,0),(ox,bev_size),(30,30,30),1)
    cv2.line(bev,(0,oy),(bev_size,oy),(30,30,30),1)
    cv2.putText(bev,"FWD",(ox+4,14),cv2.FONT_HERSHEY_SIMPLEX,0.4,(80,80,80),1)
    cv2.rectangle(bev,(ox-6,oy-10),(ox+6,oy+10),(100,100,100),-1)

    for det in detections:
        color  = _IDET_COLORS.get(det.cls_id,(200,200,200))
        pts    = np.stack([
            (ox + det.corners_bev[:,1]*scale).astype(np.int32),
            (oy - det.corners_bev[:,0]*scale).astype(np.int32),
        ], axis=1)
        cv2.polylines(bev,[pts],isClosed=True,color=color,thickness=2)
        cu  = int(ox + det.y*scale)
        cv_ = int(oy - det.x*scale)
        cv2.circle(bev,(cu,cv_),4,color,-1)
        label = f"{det.cls_name} {det.score:.2f}"
        (tw,th),_ = cv2.getTextSize(label,cv2.FONT_HERSHEY_SIMPLEX,0.38,1)
        tx = int(np.clip(cu+6,2,bev_size-tw-2))
        ty = int(np.clip(cv_-6,th+2,bev_size-2))
        cv2.putText(bev,label,(tx,ty),cv2.FONT_HERSHEY_SIMPLEX,0.38,color,1)

    sep = np.full((4,bev_size,3),80,dtype=np.uint8)
    return np.vstack([cam_panel, sep, bev])


def draw_tracks(
    img_bgr: np.ndarray,
    detections: List[Detection3D],
    tracked: List[TrackedObject],
    projector,
    range_m: float = 60.,
    bev_size: int  = 700,
) -> np.ndarray:
    """
    Combined track visualisation:
      top    — Camera image with 3D wireframe boxes (detections) + track labels
      bottom — BEV panel with 2D footprints, tracks, and IDs
    """
    out = img_bgr.copy()
    h_cam, w_cam = out.shape[:2]

    def _proj(pt3d):
        cp = projector.lidar_to_camera(pt3d.reshape(1,3).astype(np.float32))
        if cp[0,2] <= 0: return None
        uv = (projector.P2[:,:3] @ cp.T).T
        uv = uv[:,:2] / uv[:,2:3]
        u,v = int(uv[0,0]),int(uv[0,1])
        if -100<u<w_cam+100 and -100<v<h_cam+100: return (u,v)
        return None

    # Detection wireframes
    for det in detections:
        color   = _IDET_COLORS.get(det.cls_id,(200,200,200))
        corners = _box_corners_3d(det)
        proj_c  = [_proj(corners[i]) for i in range(8)]
        for i,j in _BOX_EDGES:
            if proj_c[i] and proj_c[j]:
                cv2.line(out, proj_c[i], proj_c[j], color, 2, cv2.LINE_AA)

    # Track labels at Kalman-smoothed positions
    for t in tracked:
        uv = _proj(t.box7[:3])
        if uv is None: continue
        u, v = uv
        color = _IDET_COLORS.get(t.cls_id,(200,200,200))
        label = f"{t.cls_name}  ID:{t.track_id}"
        if t.score > 0: label += f"  {t.score:.2f}"
        (tw,th),_ = cv2.getTextSize(label,cv2.FONT_HERSHEY_SIMPLEX,0.55,2)
        cv2.rectangle(out,(u-2,v-th-6),(u+tw+2,v+2),(0,0,0),-1)
        cv2.putText(out,label,(u,v-4),cv2.FONT_HERSHEY_SIMPLEX,0.55,(0,255,255),2)
        cv2.circle(out,(u,v),6,color,-1)

    scale_cam  = bev_size / w_cam
    cam_panel  = cv2.resize(out, (bev_size, int(h_cam*scale_cam)),
                             interpolation=cv2.INTER_AREA)

    # ── BEV panel ─────────────────────────────────────────────────────────────
    bev    = np.zeros((bev_size, bev_size, 3), dtype=np.uint8)
    scale  = bev_size / (2*range_m)
    ox, oy = bev_size//2, bev_size//2

    for r in [10,20,30,40,50]:
        cv2.circle(bev,(ox,oy),int(r*scale),(30,30,30),1)
        cv2.putText(bev,f"{r}m",(ox+2,oy-int(r*scale)+12),
                    cv2.FONT_HERSHEY_SIMPLEX,0.3,(60,60,60),1)
    cv2.line(bev,(ox,0),(ox,bev_size),(30,30,30),1)
    cv2.line(bev,(0,oy),(bev_size,oy),(30,30,30),1)
    cv2.putText(bev,"FWD",(ox+4,14),cv2.FONT_HERSHEY_SIMPLEX,0.4,(80,80,80),1)
    cv2.rectangle(bev,(ox-6,oy-10),(ox+6,oy+10),(100,100,100),-1)

    # Draw detections BEV footprint (thin class-colored outline)
    for det in detections:
        color  = _IDET_COLORS.get(det.cls_id,(200,200,200))
        pts    = np.stack([
            (ox + det.corners_bev[:,1]*scale).astype(np.int32),
            (oy - det.corners_bev[:,0]*scale).astype(np.int32),
        ], axis=1)
        cv2.polylines(bev,[pts],isClosed=True,color=color,thickness=1)

    # Draw tracked objects BEV footprint (thick class-colored outline + track labels with IDs)
    for t in tracked:
        cx, cy, cz, dx, dy, dz, heading = t.box7
        c, s   = np.cos(heading), np.sin(heading)
        hl, hw = dx / 2, dy / 2
        corners = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
        R = np.array([[c, -s], [s, c]])
        corners_bev = (R @ corners.T).T + np.array([cx, cy])

        color  = _IDET_COLORS.get(t.cls_id,(200,200,200))
        pts    = np.stack([
            (ox + corners_bev[:,1]*scale).astype(np.int32),
            (oy - corners_bev[:,0]*scale).astype(np.int32),
        ], axis=1)
        cv2.polylines(bev,[pts],isClosed=True,color=color,thickness=2)
        cu  = int(ox + cy*scale)
        cv_ = int(oy - cx*scale)
        cv2.circle(bev,(cu,cv_),6,color,-1)
        label = f"{t.cls_name} ID:{t.track_id}"
        if t.score > 0: label += f" {t.score:.2f}"
        (tw,th),_ = cv2.getTextSize(label,cv2.FONT_HERSHEY_SIMPLEX,0.38,1)
        tx = int(np.clip(cu+8,2,bev_size-tw-2))
        ty = int(np.clip(cv_-8,th+2,bev_size-2))
        cv2.putText(bev,label,(tx,ty),cv2.FONT_HERSHEY_SIMPLEX,0.38,(0,255,255),1)

    sep = np.full((4,bev_size,3),80,dtype=np.uint8)
    return np.vstack([cam_panel, sep, bev])


# ══════════════════════════════════════════════════════════════════════════════
#  ROS2 / bag utilities
# ══════════════════════════════════════════════════════════════════════════════

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2 as pc2

sys.path.insert(0, "/workspace/ros2_ws/src/perception_framework")
sys.path.insert(0, "/workspace/ros2_ws/install/perception_framework/lib/python3.10/site-packages")
from perception_framework.lidar_to_image_projection import KittiLidarToImageProjector


def _open_reader(bag_path):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                     output_serialization_format="cdr"),
    )
    return r


def _majority_class(label_mask, cy, cx, r=3):
    y0,y1 = max(0,cy-r), min(label_mask.shape[0],cy+r+1)
    x0,x1 = max(0,cx-r), min(label_mask.shape[1],cx+r+1)
    patch  = label_mask[y0:y1,x0:x1];  valid = patch[patch>=0]
    return int(np.argmax(np.bincount(valid))) if len(valid) else -1


# ══════════════════════════════════════════════════════════════════════════════
#  Initialise models
# ══════════════════════════════════════════════════════════════════════════════

print("\n[Init] Loading YOLO model...")
from ultralytics import YOLO
model = YOLO(YOLO_MODEL)
print(f"  {os.path.abspath(YOLO_MODEL)}  ({type(model.model).__name__})")

print("[Init] Initialising FrustumDetector...")
detector = FrustumDetector(conf_thr=args.conf, min_pts=args.min_pts, use_dbscan=True)
print(f"  conf_thr={args.conf}  min_pts={args.min_pts}  use_dbscan=True")

print("[Init] Initialising AB3DMOT tracker...")
tracker   = AB3DMOT(iou_threshold=args.iou_thr, max_age=args.max_age,
                    min_hits=args.min_hits)
projector = KittiLidarToImageProjector(CALIB_PATH)
bridge    = CvBridge()

# ══════════════════════════════════════════════════════════════════════════════
#  Stage 1 — Scan bag, choose frames
# ══════════════════════════════════════════════════════════════════════════════

print("\n[Stage 1] Scanning bag...")
reader     = _open_reader(BAG_PATH)
topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
img_count  = sum(1 for t,_,_ in (reader.read_next()
                                  for _ in iter(reader.has_next, False))
                 if t == IMG_TOPIC)

# Simpler re-scan
reader    = _open_reader(BAG_PATH)
topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
img_count = 0
while reader.has_next():
    t,_,_ = reader.read_next()
    if t == IMG_TOPIC: img_count += 1
print(f"  {img_count} image frames found")

if args.all_frames:
    target_indices = set(range(img_count))
    print("  Processing all frames")
else:
    idx = rng.randint(0, img_count-1)
    target_indices = {idx}
    seed_used = args.seed if args.seed is not None else "(random)"
    print(f"  Single frame: index {idx}  [seed: {seed_used}]")

# ══════════════════════════════════════════════════════════════════════════════
#  Main loop
# ══════════════════════════════════════════════════════════════════════════════

reader      = _open_reader(BAG_PATH)
topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
img_seen    = 0
img_frame   = None
all_tracked: List[TrackedObject] = []

while reader.has_next():
    topic, data, _ = reader.read_next()

    # ── Image ─────────────────────────────────────────────────────────────────
    if topic == IMG_TOPIC:
        if img_seen in target_indices:
            msg       = deserialize_message(data, get_message(topic_types[topic]))
            img_frame = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        img_seen += 1
        continue

    if topic != LIDAR_TOPIC or img_frame is None:
        continue

    # ── LiDAR ─────────────────────────────────────────────────────────────────
    msg      = deserialize_message(data, get_message(topic_types[topic]))
    pts_list = list(pc2.read_points(msg,
                    field_names=("x","y","z","intensity"), skip_nans=True))
    lidar_raw = np.array([(p[0],p[1],p[2],p[3]) for p in pts_list],
                         dtype=np.float32)

    frame_idx = img_seen - 1
    h, w      = img_frame.shape[:2]
    print(f"\n{'═'*60}")
    print(f"  Frame {frame_idx:4d}   image:{img_frame.shape}   LiDAR:{lidar_raw.shape[0]} pts")

    # ── Stage 1 output ────────────────────────────────────────────────────────
    cv2.imwrite(f"{OUTPUT_DIR}/01_raw_image.jpg", img_frame)

    # ── Stage 2 — YOLO (single call) ─────────────────────────────────────────
    print("  [Stage 2] YOLO segmentation...")
    img_rgb = cv2.cvtColor(img_frame, cv2.COLOR_BGR2RGB)
    score_maps, results_yolo = build_score_maps(img_rgb, model, conf=args.conf)

    # Hard label mask for Stage 3/4 visualisation only
    label_mask = np.full((h, w), -1, dtype=np.int32)
    PRIORITY   = {0: 10, 1: 9, 3: 8}
    detected_cls = []
    for result in results_yolo:
        if result.masks is None: continue
        masks   = result.masks.data.cpu().numpy()
        classes = result.boxes.cls.cpu().numpy().astype(int)
        pairs   = sorted(zip(masks, classes), key=lambda mc: PRIORITY.get(mc[1],0))
        for mask, cls_id in pairs:
            m8 = cv2.resize((mask*255).astype(np.uint8),(w,h),
                             interpolation=cv2.INTER_LINEAR)
            label_mask[m8 > 200] = cls_id
        detected_cls = list(np.unique(label_mask[label_mask >= 0]))

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(7,7))
    clean  = np.full_like(label_mask,-1)
    for cls_id in np.unique(label_mask[label_mask >= 0]):
        dilated = cv2.dilate((label_mask==cls_id).astype(np.uint8),kernel,iterations=1)
        clean[dilated==1] = cls_id
    label_mask = clean

    mask_vis = np.zeros((h,w,3),dtype=np.uint8)
    for cls_id,color in CLASS_COLORS_BGR.items():
        mask_vis[label_mask==cls_id] = color
    cv2.imwrite(f"{OUTPUT_DIR}/02_yolo_mask.jpg", mask_vis)
    print(f"    Classes: {[(c,model.names[c]) for c in detected_cls if c in model.names]}")

    # ── Stage 3 — Overlay ────────────────────────────────────────────────────
    overlay = cv2.addWeighted(img_frame,0.6,mask_vis,0.4,0)
    y_leg   = 30
    for cls_id,color in CLASS_COLORS_BGR.items():
        if cls_id in detected_cls:
            cv2.rectangle(overlay,(10,y_leg-15),(30,y_leg+5),color,-1)
            cv2.putText(overlay,model.names.get(cls_id,str(cls_id)),
                        (35,y_leg),cv2.FONT_HERSHEY_SIMPLEX,0.6,(255,255,255),2)
            y_leg += 30
    cv2.imwrite(f"{OUTPUT_DIR}/03_overlay.jpg", overlay)

    # ── Stage 4 — LiDAR → image ──────────────────────────────────────────────
    print("  [Stage 4] LiDAR projection...")
    ground_mask    = lidar_raw[:,2] > GROUND_Z_THRESH
    lidar_filtered = lidar_raw[ground_mask]
    image_points, _ = projector.project_lidar_to_image(lidar_filtered[:,:3],(h,w))

    cam_all   = projector.lidar_to_camera(lidar_filtered[:,:3])
    front     = cam_all[:,2] > 0
    uv_all    = (projector.P2[:,:3] @ cam_all[front].T).T
    uv_all    = uv_all[:,:2] / uv_all[:,2:3]
    in_bounds = ((uv_all[:,0]>=0)&(uv_all[:,0]<w)&
                 (uv_all[:,1]>=0)&(uv_all[:,1]<h))
    depths    = cam_all[front][in_bounds,2]
    order     = np.argsort(depths)[::-1]

    lidar_img = img_frame.copy()
    for pt_px,depth in zip(image_points[order],depths[order]):
        u,v   = int(pt_px[0]),int(pt_px[1])
        cls_id = _majority_class(label_mask,v,u,r=3)
        color  = CLASS_COLORS_BGR.get(cls_id,UNPAINTED_BGR)
        radius = max(3,int(30./max(depth,1.)))
        cv2.circle(lidar_img,(u,v),radius,color,-1)
    cv2.imwrite(f"{OUTPUT_DIR}/04_lidar_projected.jpg", lidar_img)
    print(f"    {len(image_points)} pts projected")

    # ── Stage 5 — Score map visualisation (no enrichment needed) ─────────────
    print("  [Stage 5] Score map visualisation...")
    for cls_id,smap in score_maps.items():
        vis = (smap*255).astype(np.uint8)
        cv2.imwrite(f"{OUTPUT_DIR}/05_score_map_cls{cls_id}.jpg",
                    cv2.applyColorMap(vis,cv2.COLORMAP_JET))
    print("    saved 05_score_map_cls*.jpg")

    # ── Stage 6 — Frustum 3D detection  ← KEY CHANGE ─────────────────────────
    print("  [Stage 6] FrustumDetector...")
    from point_pillars import paint_point_cloud
    painted_cloud = paint_point_cloud(
        lidar_filtered,
        score_maps,
        projector,
        (h, w),
        yolo_results=results_yolo
    )
    detections_raw = detector.detect(lidar_filtered, results_yolo, projector, (h,w), painted_cloud=painted_cloud)
    detections     = nms_3d(detections_raw, dist_thr=args.nms_dist)
    print(f"    {len(detections_raw)} raw  →  {len(detections)} after NMS")
    for d in detections:
        print(f"      {d.cls_name:12s}  score={d.score:.2f}  "
              f"({d.x:.1f},{d.y:.1f},{d.z:.1f})  "
              f"{d.dx:.2f}×{d.dy:.2f}×{d.dz:.2f}  "
              f"heading={d.heading:.3f}  "
              f"paint_filtered={d.paint_filtered}  "
              f"kept={d.paint_kept}  "
              f"discarded={d.paint_discarded}  "
              f"pts_in_frustum≥{args.min_pts}")

    combined = draw_combined(img_frame, detections, projector)
    cv2.imwrite(f"{OUTPUT_DIR}/06_detections.jpg", combined)
    print("    saved 06_detections.jpg")

    # ── Stage 7 — AB3DMOT tracking ───────────────────────────────────────────
    print("  [Stage 7] Tracking...")
    confirmed = tracker.update(detections)
    all_tracked.extend(confirmed)
    print(f"    active:{len(tracker.tracks)}  confirmed:{len(confirmed)}")
    for t in confirmed:
        print(f"      ID={t.track_id:3d}  {t.cls_name:12s}  "
              f"pos=({t.box7[0]:.2f},{t.box7[1]:.2f})  score={t.score:.2f}")

    final    = draw_tracks(img_frame, detections, confirmed, projector)
    out_path = f"{OUTPUT_DIR}/07_tracked_{frame_idx:04d}.jpg"
    cv2.imwrite(out_path, final)
    print(f"    saved 07_tracked_{frame_idx:04d}.jpg")

    img_frame = None
    if not args.all_frames:
        break

# ══════════════════════════════════════════════════════════════════════════════
#  Summary
# ══════════════════════════════════════════════════════════════════════════════
unique_ids = len(set(t.track_id for t in all_tracked))
print(f"\n{'═'*60}")
print(f"Done.  Frames:{len(target_indices)}  Unique tracks:{unique_ids}")
print(f"Output: {OUTPUT_DIR}/")
for f in sorted(os.listdir(OUTPUT_DIR)):
    print(f"  {f}")
if not args.all_frames:
    print(f"\nReproduce: python3 {os.path.basename(__file__)} --seed {list(target_indices)[0]}")
print(f"All frames: python3 {os.path.basename(__file__)} --all-frames")


def make_video(output_dir, fps=10):
    from pathlib import Path
    files = sorted(Path(output_dir).glob("07_tracked_*.jpg"), key=lambda p: int(p.stem.split("_")[-1]))
    if len(files) < 2:
        print("Warning: Fewer than 2 frames found, skipping video generation.")
        return
    
    first_frame = cv2.imread(str(files[0]))
    h_f, w_f = first_frame.shape[:2]
    
    out_video_path = os.path.join(output_dir, "pipeline_output.mp4")
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    writer = cv2.VideoWriter(out_video_path, fourcc, fps, (w_f, h_f))
    
    for f in files:
        frame = cv2.imread(str(f))
        writer.write(frame)
    writer.release()
    print(f"Saved video: pipeline_output.mp4 ({len(files)} frames at {fps}fps)")

make_video(OUTPUT_DIR)
