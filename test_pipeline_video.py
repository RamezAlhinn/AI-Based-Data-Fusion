#!/usr/bin/env python3
"""
test_pipeline_video.py — Full PointPainting + PointPillars + 3D-SORT tracking video pipeline.

Reads every paired image/LiDAR frame from the ROS2 bag, runs the complete
detection + tracking pipeline, and writes an annotated MP4.

Usage:
    python3 test_pipeline_video.py [options]

Key options:
    --max-frames N     Only process first N frames (0 = all, default 30)
    --output FILE      Output MP4 path (default: tracking_output.mp4)
    --fps N            Output video frame rate (default: 10)
    --show-bev         Embed BEV radar inset in bottom-right corner (default: on)
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np

# ── Paths ──────────────────────────────────────────────────────────────────────
BAG_PATH   = '/workspace/studentProject1/studentProject1_0.db3'
CALIB_PATH = '/workspace/calib.txt'
YOLO_PATH  = '/workspace/models/yolo11m-seg.pt'
PP_CKPT    = '/workspace/models/pointpillars_kitti_3class.pth'
OUTPUT_DIR = '/workspace/video_output'

sys.path.insert(0, '/workspace')
sys.path.insert(0, '/workspace/ros2_ws/src/perception_framework/perception_framework')

# ── Argument parsing ───────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description='Full pipeline video export')
parser.add_argument('--bag',          default=BAG_PATH)
parser.add_argument('--output',       default=os.path.join(OUTPUT_DIR, 'tracking_output.mp4'))
parser.add_argument('--fps',          type=float, default=10.0)
parser.add_argument('--max-frames',   type=int,   default=30,
                    help='Maximum frames to process (0 = all)')
parser.add_argument('--pp-score-thr', type=float, default=0.05)
parser.add_argument('--z-offset',     type=float, default=-0.30)
parser.add_argument('--yaw-offset',   type=float, default=0.0)
parser.add_argument('--dilate-radius',type=int,   default=15)
parser.add_argument('--filter-thr',   type=float, default=0.10)
parser.add_argument('--intensity-max',type=float, default=1910.0)
parser.add_argument('--show-bev',     action='store_true', default=True)
parser.add_argument('--no-bev',       dest='show_bev', action='store_false')
parser.add_argument('--max-age',      type=int,   default=5,
                    help='SORT max track age (frames)')
parser.add_argument('--min-hits',     type=int,   default=2,
                    help='SORT min hits before track is confirmed')
parser.add_argument('--max-dist',     type=float, default=4.0,
                    help='SORT max BEV association distance (m)')
args = parser.parse_args()

os.makedirs(OUTPUT_DIR, exist_ok=True)

# ── Imports ────────────────────────────────────────────────────────────────────
print('Loading models and calibration...')
from ultralytics import YOLO
from point_pillars import (
    PointPillars, PointPillarsConfig, build_pointpillars,
    paint_point_cloud, filter_point_cloud_by_yolo, draw_boxes_on_image, draw_bev
)
from lidar_to_image_projection import KittiLidarToImageProjector
from tracking import Sort3D

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2 as pc2
import torch

# ── Model initialisation ───────────────────────────────────────────────────────
yolo_model  = YOLO(YOLO_PATH)
pp_model    = build_pointpillars(painted=False, score_thr=args.pp_score_thr)
cfg         = PointPillarsConfig()
projector   = KittiLidarToImageProjector(CALIB_PATH)
tracker     = Sort3D(max_age=args.max_age, min_hits=args.min_hits, max_dist=args.max_dist)

# Precompute calibration matrices for YOLO-snapping
R0_Tr     = (projector.R0_rect @ projector.Tr_velo_to_cam)[:3, :]
R_cam     = R0_Tr[:, :3]
t_cam     = R0_Tr[:, 3]
P2        = projector.P2
fx, fy    = float(P2[0, 0]), float(P2[1, 1])
cx_cam_k  = float(P2[0, 2])
cy_cam_k  = float(P2[1, 2])
R_cam_inv = np.linalg.inv(R_cam)
PP_TO_YOLO = {0: 0, 1: 1, 2: 2}

print(f'  YOLO : {YOLO_PATH}')
print(f'  PP   : {PP_CKPT}')
print(f'  Calib: {CALIB_PATH}')

# Suppress verbose PointPillars logging after first load
import logging
logging.getLogger('point_pillars').setLevel(logging.WARNING)

# ── Constants ──────────────────────────────────────────────────────────────────
IMG_TOPIC   = '/blackfly_s/cam0/image_rectified'
LIDAR_TOPIC = '/velodyne/points_raw'
PAINT_CLASSES = {0: 'ped', 2: 'car', 1: 'cyc'}
CLASS_COLORS_BGR = {0: (0, 100, 255), 1: (0, 255, 150), 2: (255, 200, 0)}


def _open_reader(bag_path):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                    output_serialization_format='cdr'),
    )
    return r


# ── Collect all paired frames ──────────────────────────────────────────────────
print('\nScanning bag for paired image+LiDAR frames...')
reader = _open_reader(args.bag)
topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
bridge = CvBridge()

all_frames = []          # list of (img_bgr, lidar_np)
pending_img = None

while reader.has_next():
    topic, data, _ = reader.read_next()
    if topic == IMG_TOPIC:
        msg = deserialize_message(data, get_message(topic_types[topic]))
        pending_img = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    elif topic == LIDAR_TOPIC and pending_img is not None:
        msg = deserialize_message(data, get_message(topic_types[topic]))
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity', 'ring'),
                                   skip_nans=True))
        lidar_np = np.array([(p[0], p[1], p[2], p[3], p[4]) for p in pts], dtype=np.float32)
        # Normalise intensity
        lidar_np[:, 3] = np.clip(lidar_np[:, 3] / args.intensity_max, 0.0, 1.0)
        lidar_np[:, 4] = lidar_np[:, 4] / (lidar_np[:, 4].max() + 1e-6)
        all_frames.append((pending_img.copy(), lidar_np))
        pending_img = None

n_total = len(all_frames)
n_proc  = n_total if args.max_frames == 0 else min(args.max_frames, n_total)
print(f'  Found {n_total} paired frames → processing first {n_proc}')

# ── Video writer ───────────────────────────────────────────────────────────────
h_img, w_img = all_frames[0][0].shape[:2]
fourcc  = cv2.VideoWriter_fourcc(*'mp4v')
writer  = cv2.VideoWriter(args.output, fourcc, args.fps, (w_img, h_img))
print(f'  Output: {args.output}  ({w_img}×{h_img} @ {args.fps} fps)')


# ── Helper: YOLO-snap a set of vis_boxes to YOLO mask centroids ───────────────
def yolo_snap(vis_boxes, labels, yolo_results, w, h):
    """Return boxes with their lateral (y) positions snapped to YOLO centroids."""
    # Collect per-class YOLO instance centroids
    yolo_centroids = {}
    for result in yolo_results:
        if result.masks is None:
            continue
        masks_np   = result.masks.data.cpu().numpy()
        classes_np = result.boxes.cls.cpu().numpy().astype(int)
        for mask, cls_id in zip(masks_np, classes_np):
            mask_rsz = cv2.resize((mask * 255).astype(np.uint8), (w, h),
                                   interpolation=cv2.INTER_LINEAR)
            ys_m, xs_m = np.where(mask_rsz > 127)
            if len(xs_m) == 0:
                continue
            yolo_centroids.setdefault(cls_id, []).append(
                (float(xs_m.mean()), float(ys_m.mean()))
            )

    corrected = vis_boxes.copy()
    for i, (box, label) in enumerate(zip(vis_boxes, labels)):
        cx_l, cy_l, cz_l = float(box[0]), float(box[1]), float(box[2])
        lp_h      = np.array([cx_l, cy_l, cz_l, 1.0])
        cam_pt    = R0_Tr @ lp_h
        cam_depth = cam_pt[2]
        if cam_depth <= 0:
            continue
        u_box = fx * cam_pt[0] / cam_depth + cx_cam_k
        v_box = fy * cam_pt[1] / cam_depth + cy_cam_k

        yolo_cls   = PP_TO_YOLO.get(int(label))
        candidates = yolo_centroids.get(yolo_cls, [])
        if not candidates:
            continue

        dists   = [((u_c - u_box)**2 + (v_c - v_box)**2)**0.5 for u_c, v_c in candidates]
        best_i  = int(np.argmin(dists))
        u_c, v_c = candidates[best_i]

        cam_x_new  = (u_c - cx_cam_k) / fx * cam_depth
        cam_y_new  = (v_c - cy_cam_k) / fy * cam_depth
        cam_pt_new = np.array([cam_x_new, cam_y_new, cam_depth])
        lp_new     = R_cam_inv @ (cam_pt_new - t_cam)
        corrected[i, 0] = lp_new[0]
        corrected[i, 1] = lp_new[1]
        corrected[i, 2] = lp_new[2]

    return corrected


# ── Helper: draw tracked boxes on image ───────────────────────────────────────
def _box_corners_3d(cx, cy, cz, w, l, h, heading):
    c, s = np.cos(heading), np.sin(heading)
    hl, hw, hh = l / 2, w / 2, h / 2
    corners = np.array([
        [ hl,  hw, -hh], [ hl, -hw, -hh], [-hl, -hw, -hh], [-hl,  hw, -hh],
        [ hl,  hw,  hh], [ hl, -hw,  hh], [-hl, -hw,  hh], [-hl,  hw,  hh],
    ], dtype=np.float32)
    R = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=np.float32)
    return (R @ corners.T).T + np.array([cx, cy, cz], dtype=np.float32)


def draw_tracked_boxes(img, tracks, projector):
    out = img.copy()
    h_im, w_im = img.shape[:2]
    for trk in tracks:
        box    = trk.box                           # [cx,cy,cz,w,l,h,heading]
        color  = trk.color
        cls_name = PointPillars.CLASS_NAMES.get(trk.label, str(trk.label))
        label_txt = f'{cls_name[:3]} #{trk.id}  {trk.score:.2f}'

        corners = _box_corners_3d(*box)            # (8,3) in LiDAR frame
        cam_pts = projector.lidar_to_camera(corners)
        if (cam_pts[:, 2] <= 0).any():
            continue                               # any corner behind camera → skip

        uv = (P2[:, :3] @ cam_pts.T).T
        uv = uv[:, :2] / uv[:, 2:3]
        uv = uv.astype(int)

        # Clip to image (allow some outside)
        def _line(p1, p2):
            cv2.line(out, tuple(np.clip(p1, -5000, 5000)),
                     tuple(np.clip(p2, -5000, 5000)), color, 2)

        # Bottom face (0-3), top face (4-7), pillars
        for i, j in [(0,1),(1,2),(2,3),(3,0),(4,5),(5,6),(6,7),(7,4),
                     (0,4),(1,5),(2,6),(3,7)]:
            _line(uv[i], uv[j])

        # Label above top-centre
        top_pts = uv[4:]
        tx, ty  = int(top_pts[:, 0].mean()), int(top_pts[:, 1].min()) - 6
        if 0 <= tx < w_im and 0 <= ty < h_im:
            (tw, th), _ = cv2.getTextSize(label_txt, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)
            cv2.rectangle(out, (tx - 2, ty - th - 2), (tx + tw + 2, ty + 2), color, -1)
            cv2.putText(out, label_txt, (tx, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 1, cv2.LINE_AA)

        # Track trail (history of centres)
        trail = trk.history[-10:]
        if len(trail) > 1:
            trail_cam = projector.lidar_to_camera(np.stack(trail))
            fwd = trail_cam[:, 2] > 0
            if fwd.sum() > 1:
                trail_uv = (P2[:, :3] @ trail_cam[fwd].T).T
                trail_uv = (trail_uv[:, :2] / trail_uv[:, 2:3]).astype(int)
                for k in range(len(trail_uv) - 1):
                    alpha = (k + 1) / len(trail_uv)
                    col_fade = tuple(int(c * alpha) for c in color)
                    cv2.line(out, tuple(trail_uv[k]), tuple(trail_uv[k+1]), col_fade, 1)

    return out


# ── Helper: draw BEV inset ─────────────────────────────────────────────────────
BEV_SIZE  = 300    # pixels for the inset square
BEV_RANGE = 30.0  # metres from ego

def draw_bev_inset(tracks, cfg):
    """Return a BEV radar image (BEV_SIZE × BEV_SIZE)."""
    bev = np.zeros((BEV_SIZE, BEV_SIZE, 3), dtype=np.uint8)
    cx, cy = BEV_SIZE // 2, BEV_SIZE     # ego at bottom-centre

    # Grid rings
    for r_m in [5, 10, 15, 20, 25, 30]:
        r_px = int(r_m / BEV_RANGE * BEV_SIZE)
        cv2.circle(bev, (cx, cy), r_px, (40, 40, 40), 1)
        cv2.putText(bev, f'{r_m}m', (cx + r_px + 2, cy - 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.28, (80, 80, 80), 1)

    # Axis lines
    cv2.line(bev, (cx, 0), (cx, BEV_SIZE), (40, 40, 40), 1)
    cv2.putText(bev, 'FWD', (cx + 3, 12), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (80, 80, 80), 1)

    # Ego vehicle marker
    cv2.rectangle(bev, (cx - 4, cy - 8), (cx + 4, cy), (200, 200, 200), -1)

    def lidar_to_bev(lx, ly):
        """LiDAR x=fwd,y=left → BEV pixel (left=right, fwd=up)."""
        px = cx - int(ly / BEV_RANGE * BEV_SIZE)
        py = cy - int(lx / BEV_RANGE * BEV_SIZE)
        return px, py

    for trk in tracks:
        bx, by = float(trk.box[0]), float(trk.box[1])
        px, py = lidar_to_bev(bx, by)
        if 0 <= px < BEV_SIZE and 0 <= py < BEV_SIZE:
            color = trk.color
            # Draw small box footprint in BEV (simplified circle + heading arrow)
            cv2.circle(bev, (px, py), 5, color, -1)
            # Heading arrow
            hd     = float(trk.box[6])
            arr_len = 12
            ax = px - int(np.sin(hd) * arr_len)
            ay = py - int(np.cos(hd) * arr_len)
            cv2.arrowedLine(bev, (px, py), (ax, ay), color, 1, tipLength=0.4)
            # ID label
            cls_name = PointPillars.CLASS_NAMES.get(trk.label, '?')[:3]
            cv2.putText(bev, f'{cls_name}#{trk.id}', (px + 6, py - 4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.30, color, 1)
            # Trail
            trail = trk.history[-10:]
            for k in range(len(trail) - 1):
                p1 = lidar_to_bev(trail[k][0], trail[k][1])
                p2 = lidar_to_bev(trail[k+1][0], trail[k+1][1])
                alpha = (k + 1) / len(trail)
                col_fade = tuple(int(c * alpha) for c in color)
                cv2.line(bev, p1, p2, col_fade, 1)

    return bev


# ── Main processing loop ───────────────────────────────────────────────────────
print(f'\nProcessing {n_proc} frames...\n')
t_start = time.time()

for frame_idx, (img_bgr, lidar_raw) in enumerate(all_frames[:n_proc]):
    t0 = time.time()
    h, w = img_bgr.shape[:2]
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)

    # ── YOLO segmentation ──────────────────────────────────────────────────────
    yolo_results = yolo_model(img_rgb, verbose=False, conf=0.25, imgsz=(h, w))

    score_maps = {cls_id: np.zeros((h, w), dtype=np.float32) for cls_id in PAINT_CLASSES}
    instance_scores = []
    for result in yolo_results:
        if result.masks is None:
            continue
        masks_np   = result.masks.data.cpu().numpy()
        classes_np = result.boxes.cls.cpu().numpy().astype(int)
        confs_np   = result.boxes.conf.cpu().numpy()
        for mask, cls_id, conf in zip(masks_np, classes_np, confs_np):
            if cls_id not in PAINT_CLASSES:
                continue
            m = cv2.resize((mask * 255).astype(np.uint8), (w, h),
                           interpolation=cv2.INTER_LINEAR).astype(np.float32) / 255.0
            instance_scores.append((m * float(conf), cls_id))

    if instance_scores:
        stack   = np.stack([s for s, _ in instance_scores], axis=0)
        winner  = np.argmax(stack, axis=0)
        any_det = stack.max(axis=0) > 0
        for idx, (sm, cls_id) in enumerate(instance_scores):
            owns = (winner == idx) & any_det
            score_maps[cls_id] = np.maximum(score_maps[cls_id],
                                             np.where(owns, sm, 0.0).astype(np.float32))
    for cls_id in PAINT_CLASSES:
        score_maps[cls_id] = cv2.GaussianBlur(score_maps[cls_id], (5, 5), 0)

    # ── PointPainting ──────────────────────────────────────────────────────────
    painted_cloud = paint_point_cloud(
        lidar_raw, score_maps, projector, image_shape=(h, w),
        yolo_results=yolo_results
    )

    # ── YOLO pillar filter ─────────────────────────────────────────────────────
    filtered_kitti = filter_point_cloud_by_yolo(
        painted_cloud, cfg,
        filter_thr=args.filter_thr,
        dilate_radius=args.dilate_radius
    )
    if len(filtered_kitti) == 0:
        # No painted points → no detections → update tracker with empty set
        tracks = tracker.update(np.zeros((0, 7)), [], [])
        annotated = img_bgr.copy()
        print(f'  [{frame_idx+1:3d}/{n_proc}] no painted points — skip detection')
    else:
        # ── PointPillars ───────────────────────────────────────────────────────
        points_4ch = filtered_kitti[:, :4].copy()
        if args.z_offset != 0.0:
            points_4ch[:, 2] += args.z_offset
        if args.yaw_offset != 0.0:
            cos_y = np.cos(args.yaw_offset)
            sin_y = np.sin(args.yaw_offset)
            x_r   = points_4ch[:, 0] * cos_y - points_4ch[:, 1] * sin_y
            y_r   = points_4ch[:, 0] * sin_y + points_4ch[:, 1] * cos_y
            points_4ch[:, 0] = x_r
            points_4ch[:, 1] = y_r

        boxes, scores, labels = pp_model.detect(points_4ch)
        boxes_arr = np.array(boxes, dtype=np.float32) if len(boxes) else np.zeros((0, 7))

        # Undo alignment for visualisation
        vis_boxes = boxes_arr.copy()
        if args.yaw_offset != 0.0:
            inv_yaw = -args.yaw_offset
            cos_i   = np.cos(inv_yaw); sin_i = np.sin(inv_yaw)
            cx_r = vis_boxes[:, 0] * cos_i - vis_boxes[:, 1] * sin_i
            cy_r = vis_boxes[:, 0] * sin_i + vis_boxes[:, 1] * cos_i
            vis_boxes[:, 0] = cx_r
            vis_boxes[:, 1] = cy_r
        if args.z_offset != 0.0:
            vis_boxes[:, 2] -= args.z_offset

        # YOLO-snap lateral positions
        if len(vis_boxes) > 0:
            vis_boxes = yolo_snap(vis_boxes, labels, yolo_results, w, h)

        # ── 3D SORT tracking ───────────────────────────────────────────────────
        scores_f = [float(s) for s in scores]
        labels_i = [int(l)   for l in labels]
        tracks   = tracker.update(vis_boxes, scores_f, labels_i)

        # ── Draw tracked boxes ─────────────────────────────────────────────────
        annotated = draw_tracked_boxes(img_bgr, tracks, projector)

        n_det = len(boxes)
        n_trk = len(tracks)
        elapsed = time.time() - t0
        det_str = ', '.join(
            f'{PointPillars.CLASS_NAMES.get(int(l),"?")}({s:.2f})'
            for l, s in zip(labels_i, scores_f)
        ) or 'none'
        print(f'  [{frame_idx+1:3d}/{n_proc}]  det={n_det}  tracks={n_trk}  '
              f'{elapsed:.1f}s  [{det_str}]')

    # ── BEV inset ──────────────────────────────────────────────────────────────
    if args.show_bev and 'tracks' in dir():
        bev_inset = draw_bev_inset(tracks, cfg)
        # Paste in bottom-right corner with a small border
        bev_h, bev_w = bev_inset.shape[:2]
        pad = 10
        y1 = h - bev_h - pad
        x1 = w - bev_w - pad
        # Semi-transparent background
        roi = annotated[y1:y1+bev_h, x1:x1+bev_w]
        cv2.addWeighted(bev_inset, 0.85, roi, 0.15, 0, roi)
        annotated[y1:y1+bev_h, x1:x1+bev_w] = roi
        # Border
        cv2.rectangle(annotated, (x1-1, y1-1), (x1+bev_w, y1+bev_h),
                      (150, 150, 150), 1)

    # ── Frame overlay ──────────────────────────────────────────────────────────
    cv2.putText(annotated, f'Frame {frame_idx+1}/{n_proc}',
                (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
    n_trk_disp = len(tracks) if 'tracks' in dir() else 0
    cv2.putText(annotated, f'Tracks: {n_trk_disp}',
                (12, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)

    writer.write(annotated)

writer.release()
t_total = time.time() - t_start
print(f'\nDone. Processed {n_proc} frames in {t_total/60:.1f} min')
print(f'Output video: {args.output}')
