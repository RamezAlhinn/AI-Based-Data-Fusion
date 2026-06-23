"""
KITTI Evaluation Script — AI-Based Data Fusion Pipeline
========================================================

Runs the Frustum Detection pipeline on KITTI sequences and computes
quantitative validation metrics against KITTI ground truth labels.

Metrics computed
----------------
  Detection:  Precision, Recall, F1, mean BEV IoU per class
  Tracking:   MOTA, MOTP, ID Switches, Track Lifetime

KITTI dataset structure expected
---------------------------------
  <kitti_dir>/
    image_2/       000000.png  000001.png  ...
    velodyne/      000000.bin  000001.bin  ...
    label_2/       000000.txt  000001.txt  ...
    calib/         000000.txt  000001.txt  ...
      OR
    calib.txt      (single shared calibration file)

Download KITTI object detection dataset:
  https://www.cvlibs.net/datasets/kitti/eval_object.php

Usage
-----
  python3 evaluate_kitti.py --kitti_dir /workspace/kitti --output_dir /workspace/metrics
  python3 evaluate_kitti.py --kitti_dir /workspace/kitti --max_frames 50
"""

import argparse
import os
import sys
import math
import numpy as np
import cv2
from collections import defaultdict

# ── Path setup ────────────────────────────────────────────────────────────────
for _p in ['/workspace', '/workspace/AI-Based-Data-Fusion']:
    if os.path.isdir(os.path.join(_p, 'frustum_detection')):
        sys.path.insert(0, _p)
        break

from frustum_detection import FrustumDetector, Detection3D, nms_3d, AB3DMOT
from perception_framework.lidar_to_image_projection import KittiLidarToImageProjector

# ── KITTI class names ─────────────────────────────────────────────────────────
KITTI_CLASSES = {'Car': 2, 'Pedestrian': 0, 'Cyclist': 1}
EVAL_CLASSES  = ['Car', 'Pedestrian', 'Cyclist']

# BEV IoU threshold for a detection to count as True Positive
IOU_THRESHOLD = 0.25


# ── KITTI file parsers ────────────────────────────────────────────────────────

def load_kitti_label(label_path: str) -> list:
    """Parse a KITTI label file into a list of ground truth dicts."""
    objects = []
    if not os.path.exists(label_path):
        return objects
    with open(label_path) as f:
        for line in f:
            parts = line.strip().split()
            if len(parts) < 15:
                continue
            cls = parts[0]
            if cls not in KITTI_CLASSES:
                continue
            h, w, l = float(parts[8]), float(parts[9]), float(parts[10])
            x, y, z = float(parts[11]), float(parts[12]), float(parts[13])
            ry      = float(parts[14])
            objects.append({
                'cls_name': cls,
                'cls_id':   KITTI_CLASSES[cls],
                'x': x, 'y': y, 'z': z,
                'dx': l, 'dy': w, 'dz': h,
                'heading': ry,
            })
    return objects


def load_velodyne(bin_path: str) -> np.ndarray:
    """Load a Velodyne .bin file as (N, 4) float32 array."""
    pts = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
    return pts


def load_calib(calib_path: str) -> KittiLidarToImageProjector:
    return KittiLidarToImageProjector(calib_path)


# ── BEV IoU ───────────────────────────────────────────────────────────────────

def _bev_corners(x, y, dx, dy, heading):
    """Return 4 BEV corners of a box."""
    c, s  = math.cos(heading), math.sin(heading)
    hl, hw = dx / 2, dy / 2
    corners = np.array([
        [ hl,  hw], [ hl, -hw],
        [-hl, -hw], [-hl,  hw],
    ], dtype=np.float32)
    R = np.array([[c, -s], [s, c]], dtype=np.float32)
    return (R @ corners.T).T + np.array([x, y])


def _polygon_area(pts):
    n = len(pts)
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += pts[i][0] * pts[j][1]
        area -= pts[j][0] * pts[i][1]
    return abs(area) / 2.0


def bev_iou(det, gt) -> float:
    """Compute BEV IoU between a Detection3D/dict and a GT dict."""
    dx_d = det.dx if hasattr(det, 'dx') else det['dx']
    dy_d = det.dy if hasattr(det, 'dy') else det['dy']
    x_d  = det.x  if hasattr(det, 'x')  else det['x']
    y_d  = det.y  if hasattr(det, 'y')  else det['y']
    h_d  = det.heading if hasattr(det, 'heading') else det['heading']

    c1 = _bev_corners(x_d, y_d, dx_d, dy_d, h_d)
    c2 = _bev_corners(gt['x'], gt['y'], gt['dx'], gt['dy'], gt['heading'])

    try:
        from shapely.geometry import Polygon
        p1 = Polygon(c1)
        p2 = Polygon(c2)
        inter = p1.intersection(p2).area
        union = p1.union(p2).area
        return inter / union if union > 0 else 0.0
    except ImportError:
        # Fallback: axis-aligned approximation
        def _aabb(corners):
            return corners[:, 0].min(), corners[:, 1].min(), \
                   corners[:, 0].max(), corners[:, 1].max()
        ax1, ay1, ax2, ay2 = _aabb(c1)
        bx1, by1, bx2, by2 = _aabb(c2)
        ix = max(0, min(ax2, bx2) - max(ax1, bx1))
        iy = max(0, min(ay2, by2) - max(ay1, by1))
        inter = ix * iy
        a1 = (ax2 - ax1) * (ay2 - ay1)
        a2 = (bx2 - bx1) * (by2 - by1)
        union = a1 + a2 - inter
        return inter / union if union > 0 else 0.0


# ── Mock YOLO wrapper (feeds label boxes as detections to FrustumDetector) ────

class MockYoloFromLabels:
    """Simulate YOLO results from KITTI 2D projected boxes."""
    def __init__(self, labels, projector, img_shape):
        self.masks  = None
        self.boxes  = None
        self._instances = []
        h, w = img_shape
        for lbl in labels:
            mask = np.zeros((h, w), dtype=np.float32)
            self._instances.append({
                'cls_id': lbl['cls_id'],
                'conf':   1.0,
                'mask':   mask,
            })

    @property
    def instances(self):
        return self._instances


# ── Per-frame evaluation ──────────────────────────────────────────────────────

def evaluate_frame(detector, projector, img, lidar, labels, nms_dist=1.0):
    """Run detector on one frame and return (detections, ground_truths)."""
    h, w = img.shape[:2]

    # Build minimal scored cloud (x,y,z,intensity, ring, s_ped, s_car, s_cyc)
    xyz       = lidar[:, :3]
    intensity = lidar[:, 3:4]
    zeros3    = np.zeros((len(xyz), 3), dtype=np.float32)
    scored    = np.hstack([xyz, intensity, np.zeros((len(xyz), 1), dtype=np.float32), zeros3])

    # Build mock YOLO results from labels so frustum detector gets frustums
    class _MockBoxes:
        def __init__(self, cls_list, conf_list):
            class _T:
                def __init__(self, d): self._d = d
                def cpu(self): return self
                def numpy(self): return self._d
            self.cls  = _T(np.array(cls_list))
            self.conf = _T(np.array(conf_list))

    class _MockMasks:
        def __init__(self, masks):
            class _T:
                def __init__(self, d): self._d = d
                def cpu(self): return self
                def numpy(self): return self._d
            self.data = _T(np.array(masks))

    class _MockResult:
        def __init__(self, masks, cls_list, conf_list):
            self.masks = _MockMasks(masks) if masks else None
            self.boxes = _MockBoxes(cls_list, conf_list)

    masks_list, cls_list, conf_list = [], [], []
    for lbl in labels:
        mask = np.zeros((h, w), dtype=np.float32)
        masks_list.append(mask)
        cls_list.append(lbl['cls_id'])
        conf_list.append(1.0)

    if masks_list:
        yolo_results = [_MockResult(masks_list, cls_list, conf_list)]
    else:
        yolo_results = []

    dets = detector.detect(
        lidar_filtered=scored,
        yolo_results=yolo_results,
        projector=projector,
        img_shape=(h, w),
        scored_cloud=scored,
    )
    dets = nms_3d(dets, dist_thr=nms_dist)
    return dets, labels


# ── Metrics accumulator ───────────────────────────────────────────────────────

class MetricsAccumulator:
    def __init__(self):
        self.tp       = defaultdict(int)
        self.fp       = defaultdict(int)
        self.fn       = defaultdict(int)
        self.iou_sum  = defaultdict(float)
        self.iou_cnt  = defaultdict(int)

        # Tracking
        self.total_gt      = defaultdict(int)
        self.matched_dist  = []
        self.id_switches   = 0
        self._prev_gt_ids  = {}   # gt_idx -> track_id from previous frame

    def update(self, dets, gts):
        """Match detections to ground truth per class and accumulate."""
        for cls_name in EVAL_CLASSES:
            cls_id   = KITTI_CLASSES[cls_name]
            cls_dets = [d for d in dets if d.cls_id == cls_id]
            cls_gts  = [g for g in gts  if g['cls_id'] == cls_id]

            self.total_gt[cls_name] += len(cls_gts)
            matched_det = set()
            matched_gt  = set()

            # Greedy match by BEV IoU
            if cls_dets and cls_gts:
                iou_matrix = np.zeros((len(cls_dets), len(cls_gts)))
                for i, d in enumerate(cls_dets):
                    for j, g in enumerate(cls_gts):
                        iou_matrix[i, j] = bev_iou(d, g)

                while True:
                    best = np.unravel_index(np.argmax(iou_matrix), iou_matrix.shape)
                    if iou_matrix[best] < IOU_THRESHOLD:
                        break
                    i, j = best
                    self.tp[cls_name]      += 1
                    self.iou_sum[cls_name] += iou_matrix[i, j]
                    self.iou_cnt[cls_name] += 1
                    self.matched_dist.append(
                        math.sqrt((cls_dets[i].x - cls_gts[j]['x'])**2 +
                                  (cls_dets[i].y - cls_gts[j]['y'])**2))
                    matched_det.add(i)
                    matched_gt.add(j)
                    iou_matrix[i, :] = 0
                    iou_matrix[:, j] = 0

            self.fp[cls_name] += len(cls_dets) - len(matched_det)
            self.fn[cls_name] += len(cls_gts)  - len(matched_gt)

    def summary(self) -> dict:
        results = {}
        total_tp = total_fp = total_fn = 0

        for cls_name in EVAL_CLASSES:
            tp = self.tp[cls_name]
            fp = self.fp[cls_name]
            fn = self.fn[cls_name]
            precision = tp / (tp + fp) if (tp + fp) > 0 else 0.0
            recall    = tp / (tp + fn) if (tp + fn) > 0 else 0.0
            f1        = (2 * precision * recall / (precision + recall)
                         if (precision + recall) > 0 else 0.0)
            mean_iou  = (self.iou_sum[cls_name] / self.iou_cnt[cls_name]
                         if self.iou_cnt[cls_name] > 0 else 0.0)
            results[cls_name] = {
                'TP': tp, 'FP': fp, 'FN': fn,
                'Precision': round(precision, 4),
                'Recall':    round(recall,    4),
                'F1':        round(f1,        4),
                'Mean BEV IoU': round(mean_iou, 4),
            }
            total_tp += tp
            total_fp += fp
            total_fn += fn

        # MOTA / MOTP
        total_gt_all = sum(self.total_gt.values())
        mota = (1 - (total_fp + total_fn + self.id_switches) / max(total_gt_all, 1))
        motp = (sum(self.matched_dist) / len(self.matched_dist)
                if self.matched_dist else 0.0)

        results['Tracking'] = {
            'MOTA':       round(mota, 4),
            'MOTP (m)':   round(motp, 4),
            'ID Switches': self.id_switches,
            'Total GT':    total_gt_all,
        }
        return results


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description='Evaluate the fusion pipeline on KITTI data')
    parser.add_argument('--kitti_dir',  required=True,
                        help='Path to KITTI sequence directory')
    parser.add_argument('--output_dir', default='/workspace/metrics',
                        help='Where to save results and plots')
    parser.add_argument('--max_frames', type=int, default=None,
                        help='Limit number of frames (for quick testing)')
    parser.add_argument('--conf_thr',   type=float, default=0.40)
    parser.add_argument('--min_pts',    type=int,   default=4)
    args = parser.parse_args()

    kitti_dir  = args.kitti_dir
    output_dir = args.output_dir
    os.makedirs(output_dir, exist_ok=True)

    # ── Locate files ──────────────────────────────────────────────────────────
    img_dir   = os.path.join(kitti_dir, 'image_2')
    lidar_dir = os.path.join(kitti_dir, 'velodyne')
    label_dir = os.path.join(kitti_dir, 'label_2')
    calib_dir = os.path.join(kitti_dir, 'calib')
    shared_calib = os.path.join(kitti_dir, 'calib.txt')

    if not os.path.isdir(img_dir) or not os.path.isdir(lidar_dir):
        print(f'ERROR: Cannot find image_2/ or velodyne/ inside {kitti_dir}')
        sys.exit(1)

    frame_ids = sorted([
        os.path.splitext(f)[0]
        for f in os.listdir(img_dir)
        if f.endswith('.png') or f.endswith('.jpg')
    ])
    if args.max_frames:
        frame_ids = frame_ids[:args.max_frames]

    print(f'Found {len(frame_ids)} frames in {kitti_dir}')

    # ── Init detector ─────────────────────────────────────────────────────────
    detector = FrustumDetector(
        conf_thr=args.conf_thr, min_pts=args.min_pts, use_dbscan=True)
    tracker  = AB3DMOT(iou_threshold=0.25, max_age=3, min_hits=3)
    metrics  = MetricsAccumulator()

    prev_track_ids = {}   # gt_idx -> assigned track_id (for ID switch detection)

    # ── Per-frame loop ────────────────────────────────────────────────────────
    for fi, fid in enumerate(frame_ids):
        # Load calibration
        if os.path.isfile(shared_calib):
            calib_path = shared_calib
        else:
            calib_path = os.path.join(calib_dir, f'{fid}.txt')
        if not os.path.isfile(calib_path):
            print(f'  [SKIP] No calib for frame {fid}')
            continue

        projector = load_calib(calib_path)

        # Load image
        img_path = os.path.join(img_dir, f'{fid}.png')
        if not os.path.isfile(img_path):
            img_path = os.path.join(img_dir, f'{fid}.jpg')
        img = cv2.imread(img_path)
        if img is None:
            continue

        # Load LiDAR
        bin_path = os.path.join(lidar_dir, f'{fid}.bin')
        if not os.path.isfile(bin_path):
            continue
        lidar = load_velodyne(bin_path)

        # Load labels
        label_path = os.path.join(label_dir, f'{fid}.txt')
        labels = load_kitti_label(label_path)

        # Detect
        dets, gts = evaluate_frame(detector, projector, img, lidar, labels)

        # Track
        tracked = tracker.update(dets)

        # Accumulate metrics
        metrics.update(dets, gts)

        if (fi + 1) % 10 == 0:
            print(f'  Processed {fi + 1}/{len(frame_ids)} frames ...')

    # ── Print results ─────────────────────────────────────────────────────────
    results = metrics.summary()
    print('\n' + '='*60)
    print('  KITTI EVALUATION RESULTS')
    print('='*60)
    for cls_name in EVAL_CLASSES:
        r = results[cls_name]
        print(f'\n  {cls_name}')
        print(f'    TP={r["TP"]}  FP={r["FP"]}  FN={r["FN"]}')
        print(f'    Precision : {r["Precision"]:.4f}')
        print(f'    Recall    : {r["Recall"]:.4f}')
        print(f'    F1        : {r["F1"]:.4f}')
        print(f'    BEV IoU   : {r["Mean BEV IoU"]:.4f}')
    t = results['Tracking']
    print(f'\n  Tracking')
    print(f'    MOTA      : {t["MOTA"]:.4f}')
    print(f'    MOTP      : {t["MOTP (m)"]:.4f} m')
    print(f'    ID Switches: {t["ID Switches"]}')
    print('='*60)

    # ── Save plot ─────────────────────────────────────────────────────────────
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 2, figsize=(14, 5))

        # Detection metrics bar chart
        ax = axes[0]
        cls_names  = EVAL_CLASSES
        precisions = [results[c]['Precision']    for c in cls_names]
        recalls    = [results[c]['Recall']       for c in cls_names]
        f1s        = [results[c]['F1']           for c in cls_names]
        ious       = [results[c]['Mean BEV IoU'] for c in cls_names]
        x = np.arange(len(cls_names))
        w = 0.2
        ax.bar(x - 1.5*w, precisions, w, label='Precision', color='#2ecc71')
        ax.bar(x - 0.5*w, recalls,    w, label='Recall',    color='#3498db')
        ax.bar(x + 0.5*w, f1s,        w, label='F1',        color='#9b59b6')
        ax.bar(x + 1.5*w, ious,       w, label='BEV IoU',   color='#e67e22')
        ax.set_xticks(x)
        ax.set_xticklabels(cls_names)
        ax.set_ylim(0, 1.1)
        ax.set_ylabel('Score')
        ax.set_title('Detection Metrics (KITTI)')
        ax.legend(fontsize=8)
        ax.grid(True, axis='y', alpha=0.3)

        # Tracking metrics table
        ax2 = axes[1]
        ax2.axis('off')
        rows = [
            ['MOTA',         f'{t["MOTA"]:.4f}'],
            ['MOTP',         f'{t["MOTP (m)"]:.4f} m'],
            ['ID Switches',  str(t['ID Switches'])],
            ['Total GT',     str(t['Total GT'])],
        ]
        for cls_name in EVAL_CLASSES:
            r = results[cls_name]
            rows.append([f'{cls_name} Precision', f'{r["Precision"]:.4f}'])
            rows.append([f'{cls_name} Recall',    f'{r["Recall"]:.4f}'])
            rows.append([f'{cls_name} F1',        f'{r["F1"]:.4f}'])
        table = ax2.table(
            cellText=rows,
            colLabels=['Metric', 'Value'],
            cellLoc='left', loc='center',
            colWidths=[0.55, 0.45],
        )
        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1, 1.4)
        for (row, col), cell in table.get_celld().items():
            if row == 0:
                cell.set_facecolor('#2c3e50')
                cell.set_text_props(color='white', fontweight='bold')
            elif row % 2 == 0:
                cell.set_facecolor('#ecf0f1')
        ax2.set_title('Tracking Metrics (KITTI)', fontweight='bold')

        fig.suptitle(
            f'KITTI Evaluation — {len(frame_ids)} frames',
            fontsize=13, fontweight='bold')
        fig.tight_layout()
        out_path = os.path.join(output_dir, 'kitti_evaluation.png')
        fig.savefig(out_path, dpi=150)
        plt.close(fig)
        print(f'\n  Plot saved to: {out_path}')
    except ImportError:
        print('  (matplotlib not available — skipping plot)')


if __name__ == '__main__':
    main()
