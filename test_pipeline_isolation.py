"""
Pipeline isolation test — no ROS, no live node needed.

Stages
------
  1. Extract one image + nearest LiDAR frame from the ROS bag
  2. YOLO segmentation → per-class soft score maps + hard label mask
  3. Segmentation overlay (visualisation)
  4. LiDAR points projected onto image, coloured by class (visualisation)
  5. PointPainting — build scored cloud [x, y, z, int, ring, s_ped, s_car, s_cyc]
  6. FrustumDetector — YOLO 2D masks + scored cloud → 3D bounding boxes
  7. Visualise: camera with projected 3D boxes + BEV + tracker output

Run inside the container:
  python3 /workspace/AI-Based-Data-Fusion/test_pipeline_isolation.py
  python3 /workspace/AI-Based-Data-Fusion/test_pipeline_isolation.py --seed 7

Output (saved to /workspace/AI-Based-Data-Fusion/isolation_output/):
  01_raw_image.jpg          — original camera frame
  02_yolo_mask.jpg          — YOLO class mask (colour per class)
  03_overlay.jpg            — mask blended on top of image
  04_lidar_projected.jpg    — LiDAR points coloured by class
  05_painted_scores.jpg     — per-point painting scores (heat map)
  06_detections.jpg         — camera + BEV with FrustumDetector 3D boxes
  07_tracked.jpg            — camera + BEV with AB3DMOT track IDs
"""

import sys
import os
import random
import argparse
import numpy as np
import cv2

parser = argparse.ArgumentParser()
parser.add_argument('--seed',        type=int,   default=None,
                    help='Random seed for frame selection')
parser.add_argument('--checkpoint',  type=str,   default=None,
                    help='YOLO model file path (default: yolo11m-seg.pt)')
parser.add_argument('--conf',        type=float, default=0.40,
                    help='YOLO confidence threshold (default: 0.40)')
parser.add_argument('--min-pts',     type=int,   default=4,
                    help='Min LiDAR points per frustum (default: 4)')
parser.add_argument('--nms-dist',    type=float, default=1.0,
                    help='NMS BEV distance threshold (default: 1.0)')
parser.add_argument('--max-age',     type=int,   default=3,
                    help='Tracker max_age (default: 3)')
parser.add_argument('--min-hits',    type=int,   default=3,
                    help='Tracker min_hits to confirm (default: 3)')
parser.add_argument('--intensity-max', type=float, default=1910.0,
                    help='Sensor intensity ceiling for normalisation (default: 1910.0)')
args = parser.parse_args()

rng = random.Random(args.seed)

# ── Paths ──────────────────────────────────────────────────────────────────────
_script_dir = os.path.dirname(os.path.abspath(__file__))
BAG_PATH   = '/workspace/studentProject1'
CALIB_PATH = os.path.join(_script_dir, 'calib.txt')
OUTPUT_DIR = os.path.join(_script_dir, 'isolation_output')
YOLO_MODEL = args.checkpoint
if not YOLO_MODEL:
    for _p in [
        os.path.join(_script_dir, 'models', 'yolo11m-seg.pt'),
        os.path.join(_script_dir, 'yolo11m-seg.pt'),
        '/workspace/models/yolo11m-seg.pt',
        '/yolo11m-seg.pt',
    ]:
        if os.path.exists(_p):
            YOLO_MODEL = _p
            break
    if not YOLO_MODEL:
        YOLO_MODEL = 'yolo11m-seg.pt'

# Make the shared frustum_detection module importable
sys.path.insert(0, _script_dir)
# Make ROS2 packages importable
sys.path.insert(0, '/workspace/ros2_ws/src/perception_framework')
sys.path.insert(0, '/workspace/ros2_ws/install/perception_framework/lib/python3.10/site-packages')
sys.path.insert(0, '/workspace/ros2_ws/src/point_painting')
sys.path.insert(0, '/workspace/ros2_ws/install/point_painting/lib/python3.10/site-packages')

# COCO class colours (BGR for OpenCV)
CLASS_COLORS_BGR = {
    0:  (0,   0,   255),   # person     — red
    1:  (255, 0,   0),     # bicycle    — blue
    2:  (0,   255, 0),     # car        — green
    3:  (0,   128, 255),   # motorcycle — orange
    5:  (0,   255, 255),   # bus        — yellow
    7:  (0,   200, 0),     # truck      — dark green
}
UNPAINTED_BGR = (80, 80, 80)

os.makedirs(OUTPUT_DIR, exist_ok=True)


# ── Stage 1: Pick a random frame from the bag ──────────────────────────────────
print('\n[Stage 1] Scanning bag to count frames...')

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from cv_bridge import CvBridge
from sensor_msgs_py import point_cloud2 as pc2

IMG_TOPIC   = '/blackfly_s/cam0/image_rectified'
LIDAR_TOPIC = '/velodyne/points_raw'


def _open_reader(bag_path):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                    output_serialization_format='cdr'),
    )
    return r


reader = _open_reader(BAG_PATH)
topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
img_count = 0
while reader.has_next():
    topic, _, _ = reader.read_next()
    if topic == IMG_TOPIC:
        img_count += 1

print(f'  Found {img_count} image frames in bag.')
target_idx = rng.randint(0, img_count - 1)
seed_used  = args.seed if args.seed is not None else '(random — use --seed to reproduce)'
print(f'  Picking frame index {target_idx}  [seed: {seed_used}]')

reader     = _open_reader(BAG_PATH)
bridge     = CvBridge()
img_frame  = None
lidar_frame = None
img_seen   = 0

while reader.has_next():
    topic, data, _ = reader.read_next()

    if topic == IMG_TOPIC:
        if img_seen == target_idx:
            msg       = deserialize_message(data, get_message(topic_types[topic]))
            img_frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            print(f'  Got image #{target_idx}: {img_frame.shape}')
        img_seen += 1

    elif topic == LIDAR_TOPIC and img_frame is not None and lidar_frame is None:
        msg = deserialize_message(data, get_message(topic_types[topic]))
        pts = list(pc2.read_points(msg,
                                   field_names=('x', 'y', 'z', 'intensity', 'ring'),
                                   skip_nans=True))
        lidar_frame = np.array(
            [(p[0], p[1], p[2], p[3], p[4]) for p in pts], dtype=np.float32)
        raw_i   = lidar_frame[:, 3]
        raw_p95 = np.percentile(raw_i, 95)
        print(f'  Raw intensity — min={raw_i.min():.1f}  max={raw_i.max():.1f}  '
              f'mean={raw_i.mean():.1f}  p95={raw_p95:.1f}')
        lidar_frame[:, 3] = np.clip(lidar_frame[:, 3] / args.intensity_max, 0.0, 1.0)
        lidar_frame[:, 4] = lidar_frame[:, 4] / (lidar_frame[:, 4].max() + 1e-6)
        print(f'  Got LiDAR: {lidar_frame.shape[0]} points  '
              f'(intensity scaled 1/{args.intensity_max:.0f}, ring [0,1])')
        break

cv2.imwrite(f'{OUTPUT_DIR}/01_raw_image.jpg', img_frame)
print(f'  Saved: 01_raw_image.jpg')


# ── Stage 2: YOLO segmentation ─────────────────────────────────────────────────
print('\n[Stage 2] Running YOLO segmentation...')

from ultralytics import YOLO

model   = YOLO(YOLO_MODEL)
img_rgb = cv2.cvtColor(img_frame, cv2.COLOR_BGR2RGB)
h, w    = img_frame.shape[:2]
print(f'  Model: {os.path.abspath(YOLO_MODEL)}  ({type(model.model).__name__})')

# Single YOLO forward pass — reused for label mask, score maps, and FrustumDetector
yolo_results = model(img_rgb, verbose=False, conf=args.conf, imgsz=(h, w))

# ── Hard label mask (for viz in Stages 3 & 4) ────────────────────────────────
label_mask = np.full((h, w), -1, dtype=np.int32)
PRIORITY   = {0: 10, 1: 9, 3: 8}   # person > bicycle > motorcycle > rest

for result in yolo_results:
    if result.masks is None:
        continue
    masks   = result.masks.data.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    pairs   = sorted(zip(masks, classes), key=lambda mc: PRIORITY.get(mc[1], 0))
    for mask, cls_id in pairs:
        mask_u8     = (mask * 255).astype(np.uint8)
        mask_resized = cv2.resize(mask_u8, (w, h), interpolation=cv2.INTER_LINEAR)
        label_mask[mask_resized > 200] = cls_id

kernel     = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
clean_mask = np.full_like(label_mask, -1)
for cls_id in np.unique(label_mask[label_mask >= 0]):
    binary  = (label_mask == cls_id).astype(np.uint8)
    dilated = cv2.dilate(binary, kernel, iterations=1)
    clean_mask[dilated == 1] = cls_id
label_mask = clean_mask

# ── Soft score maps (for Stage 5 PointPainting) ──────────────────────────────
# Uses the imported build_score_maps from yolo_segmentation (same algorithm)
from point_painting.segmentation.yolo_segmentation import build_score_maps
score_maps = build_score_maps(img_rgb, yolo_results, conf=args.conf)

detected = np.unique(label_mask[label_mask >= 0])
print(f'  Detected: {[(c, model.names.get(c, str(c))) for c in detected]}')

mask_vis = np.zeros((h, w, 3), dtype=np.uint8)
for cls_id, color in CLASS_COLORS_BGR.items():
    mask_vis[label_mask == cls_id] = color
cv2.imwrite(f'{OUTPUT_DIR}/02_yolo_mask.jpg', mask_vis)
print(f'  Saved: 02_yolo_mask.jpg')


# ── Stage 3: Overlay mask on image ────────────────────────────────────────────
print('\n[Stage 3] Creating segmentation overlay...')

overlay = cv2.addWeighted(img_frame, 0.6, mask_vis, 0.4, 0)
y = 30
for cls_id, color in CLASS_COLORS_BGR.items():
    if cls_id in detected:
        name = model.names.get(cls_id, str(cls_id))
        cv2.rectangle(overlay, (10, y - 15), (30, y + 5), color, -1)
        cv2.putText(overlay, name, (35, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y += 30
cv2.imwrite(f'{OUTPUT_DIR}/03_overlay.jpg', overlay)
print(f'  Saved: 03_overlay.jpg')


# ── Stage 4: Project LiDAR points onto image ──────────────────────────────────
print('\n[Stage 4] Projecting LiDAR points onto image...')

from perception_framework.lidar_to_image_projection import KittiLidarToImageProjector

projector = KittiLidarToImageProjector(CALIB_PATH)

# Above-ground filter
GROUND_Z = -1.5
above_ground = lidar_frame[:, 2] > GROUND_Z
lidar_filtered = lidar_frame[above_ground]
print(f'  Total points: {len(lidar_frame)}'
      f'  Above ground: {len(lidar_filtered)}'
      f'  ({100*len(lidar_filtered)/max(len(lidar_frame),1):.0f}%)')

cam_all  = projector.lidar_to_camera(lidar_filtered[:, :3])
front    = cam_all[:, 2] > 0
cam_front = cam_all[front]
uv_all   = (projector.P2[:, :3] @ cam_front.T).T
uv_all   = uv_all[:, :2] / uv_all[:, 2:3]
depths   = cam_front[:, 2]
in_bounds = ((uv_all[:, 0] >= 0) & (uv_all[:, 0] < w) &
             (uv_all[:, 1] >= 0) & (uv_all[:, 1] < h))

u_in = np.clip(uv_all[in_bounds, 0].astype(int), 0, w - 1)
v_in = np.clip(uv_all[in_bounds, 1].astype(int), 0, h - 1)
d_in = depths[in_bounds]
print(f'  Points projecting into image: {in_bounds.sum()}')

def majority_class(lm, cy, cx, r=3):
    y0, y1 = max(0, cy - r), min(lm.shape[0], cy + r + 1)
    x0, x1 = max(0, cx - r), min(lm.shape[1], cx + r + 1)
    patch = lm[y0:y1, x0:x1]
    valid = patch[patch >= 0]
    if len(valid) == 0:
        return -1
    return int(np.argmax(np.bincount(valid)))

order      = np.argsort(d_in)[::-1]
lidar_img  = img_frame.copy()
painted_cnt = 0
for idx in order:
    u, v = u_in[idx], v_in[idx]
    cls_id = majority_class(label_mask, v, u, r=3)
    color  = CLASS_COLORS_BGR.get(cls_id, UNPAINTED_BGR)
    radius = max(3, int(30.0 / max(d_in[idx], 1.0)))
    cv2.circle(lidar_img, (u, v), radius=radius, color=color, thickness=-1)
    if cls_id >= 0:
        painted_cnt += 1

print(f'  Painted points (non-background): {painted_cnt}')
cv2.imwrite(f'{OUTPUT_DIR}/04_lidar_projected.jpg', lidar_img)
print(f'  Saved: 04_lidar_projected.jpg')


# ── Stage 5: PointPainting — build scored cloud ────────────────────────────────
print('\n[Stage 5] PointPainting — building scored cloud...')

from point_painting.painting_logic import init_projector, paint_points_scored

init_projector(CALIB_PATH)
scored = paint_points_scored(lidar_filtered, score_maps, yolo_results=yolo_results)
print(f'  Scored cloud: {scored.shape}  '
      f'[x, y, z, intensity, ring, s_ped, s_car, s_cyc]')

n_ped = (scored[:, 5] > 0.1).sum()
n_car = (scored[:, 6] > 0.1).sum()
n_cyc = (scored[:, 7] > 0.1).sum()
print(f'  Points with score > 0.1:  ped={n_ped}  car={n_car}  cyc={n_cyc}')

# Visualise as heatmap on camera image
score_combined = scored[:, 5:].max(axis=1)
cam_sc  = projector.lidar_to_camera(lidar_filtered[:, :3])
front_sc = cam_sc[:, 2] > 0
uv_sc   = (projector.P2[:, :3] @ cam_sc[front_sc].T).T
uv_sc   = uv_sc[:, :2] / uv_sc[:, 2:3]
in_sc   = ((uv_sc[:, 0] >= 0) & (uv_sc[:, 0] < w) &
           (uv_sc[:, 1] >= 0) & (uv_sc[:, 1] < h))
fwd_idx_sc = np.where(front_sc)[0][in_sc]

score_img = img_frame.copy()
for li, (uf, vf) in zip(fwd_idx_sc, uv_sc[in_sc]):
    sc = float(score_combined[li])
    if sc > 0.05:
        color = (0, int(255 * (1 - sc)), 255)   # yellow→red with score
        cv2.circle(score_img, (int(uf), int(vf)), 3, color, -1)

cv2.imwrite(f'{OUTPUT_DIR}/05_painted_scores.jpg', score_img)
print(f'  Saved: 05_painted_scores.jpg')


# ── Stage 6: FrustumDetector 3D detection ─────────────────────────────────────
print('\n[Stage 6] FrustumDetector 3D detection...')

from frustum_detection import FrustumDetector, nms_3d, AB3DMOT, draw_combined

detector = FrustumDetector(conf_thr=args.conf, min_pts=args.min_pts, use_dbscan=True)
print(f'  FrustumDetector: conf_thr={args.conf}  min_pts={args.min_pts}  use_dbscan=True')

dets = detector.detect(
    lidar_filtered=lidar_filtered,
    yolo_results=yolo_results,
    projector=projector,
    img_shape=(h, w),
    scored_cloud=scored,
)
dets = nms_3d(dets, dist_thr=args.nms_dist)

print(f'  Detections after NMS: {len(dets)}')
for det in dets:
    pp_info = f'  [PP↑ kept={det.paint_kept}]' if det.paint_filtered else ''
    print(f'    {det.cls_name:12s}  score={det.score:.2f}  '
          f'pos=({det.x:.1f}, {det.y:.1f}, {det.z:.1f})  '
          f'size=({det.dx:.2f}×{det.dy:.2f}×{det.dz:.2f})  '
          f'heading={det.heading:.2f}rad{pp_info}')

# Camera + BEV panel (no tracks yet — first frame)
panel = draw_combined(img_frame, dets, projector, tracked=None, range_m=60., bev_size=700)
cv2.imwrite(f'{OUTPUT_DIR}/06_detections.jpg', panel)
print(f'  Saved: 06_detections.jpg')


# ── Stage 7: Tracker (single-frame warm-up) ────────────────────────────────────
print('\n[Stage 7] Tracker update...')

tracker = AB3DMOT(iou_threshold=0.25, max_age=args.max_age, min_hits=args.min_hits)
tracked = tracker.update(dets)
print(f'  Confirmed tracks: {len(tracked)}'
      f'  (min_hits={args.min_hits} — expect 0 on first frame)')

panel_tracked = draw_combined(img_frame, dets, projector, tracked=tracked,
                               range_m=60., bev_size=700)
cv2.imwrite(f'{OUTPUT_DIR}/07_tracked.jpg', panel_tracked)
print(f'  Saved: 07_tracked.jpg')

print(f'\nDone.  Output: {OUTPUT_DIR}/')
print(f'To reproduce: python3 {os.path.basename(__file__)} --seed {target_idx}')
