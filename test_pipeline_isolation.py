"""
Pipeline isolation test — no ROS, no live node needed.

Stages
------
  1. Extract one image + nearest LiDAR frame from the ROS bag
  2. YOLO segmentation → per-class soft score maps
  3. Segmentation overlay (visualisation)
  4. LiDAR points projected onto image, coloured by class (visualisation)
  5. PointPainting — attach YOLO scores to each LiDAR point
       output: painted cloud [x, y, z, intensity, score_ped, score_car, score_cyc]
  6. PointPillars 3-D detection on the painted cloud
       output: 3-D bounding boxes + bird's-eye-view image

Run inside the container:
  python3 /workspace/AI-Based-Data-Fusion/test_pipeline_isolation.py
  python3 /workspace/AI-Based-Data-Fusion/test_pipeline_isolation.py --seed 7
  python3 /workspace/AI-Based-Data-Fusion/test_pipeline_isolation.py --pp-checkpoint /path/to/kitti.pth

Output (saved to /workspace/AI-Based-Data-Fusion/isolation_output/):
  01_raw_image.jpg          — original camera frame
  02_yolo_mask.jpg          — YOLO class mask (colour per class)
  03_overlay.jpg            — mask blended on top of image
  04_lidar_projected.jpg    — LiDAR points coloured by class
  05_painted_scores.jpg     — per-point painting scores (heat map)
  06_detections.jpg         — camera image with projected 3-D boxes
  07_bev.jpg                — bird's-eye view with 3-D box footprints
"""

import sys
import os
import random
import argparse
import numpy as np
import cv2

parser = argparse.ArgumentParser()
parser.add_argument('--seed', type=int, default=None,
                    help='Random seed for frame selection (omit for a new random frame each run)')
parser.add_argument('--checkpoint', type=str, default=None,
                    help='Path to YOLO model file (default: yolo11m-seg.pt)')
parser.add_argument('--pp-checkpoint', type=str, default=None,
                    help='Path to PointPillars .pth checkpoint (optional)')
parser.add_argument('--pp-score-thr', type=float, default=0.20,
                    help='PointPillars score threshold (default: 0.20)')
args = parser.parse_args()

rng = random.Random(args.seed)

# ── Paths ──────────────────────────────────────────────────────────────────────
_script_dir = os.path.dirname(os.path.abspath(__file__))
BAG_PATH   = '/workspace/studentProject1'
CALIB_PATH = os.path.join(_script_dir, 'calib.txt')
OUTPUT_DIR = os.path.join(_script_dir, 'isolation_output')
YOLO_MODEL = args.checkpoint if args.checkpoint else os.path.join(_script_dir, 'yolo11m-seg.pt')

# COCO class colours (BGR for OpenCV): person=0, bicycle=1, car=2, motorcycle=3, bus=5, truck=7
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


# First pass — count image frames so we can pick a random index
reader = _open_reader(BAG_PATH)
topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
img_count = 0
while reader.has_next():
    topic, _, _ = reader.read_next()
    if topic == IMG_TOPIC:
        img_count += 1

print(f'  Found {img_count} image frames in bag.')
target_idx = rng.randint(0, img_count - 1)
seed_used = args.seed if args.seed is not None else '(random — use --seed to reproduce)'
print(f'  Picking frame index {target_idx}  [seed: {seed_used}]')

# Second pass — extract the chosen image and the next LiDAR scan after it
reader = _open_reader(BAG_PATH)
bridge = CvBridge()

img_frame   = None
lidar_frame = None
img_seen    = 0

while reader.has_next():
    topic, data, _ = reader.read_next()

    if topic == IMG_TOPIC:
        if img_seen == target_idx:
            msg = deserialize_message(data, get_message(topic_types[topic]))
            img_frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            print(f'  Got image #{target_idx}: {img_frame.shape}')
        img_seen += 1

    elif topic == LIDAR_TOPIC and img_frame is not None and lidar_frame is None:
        msg = deserialize_message(data, get_message(topic_types[topic]))
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z', 'intensity', 'ring'),
                                   skip_nans=True))
        lidar_frame = np.array([(p[0], p[1], p[2], p[3], p[4]) for p in pts],
                               dtype=np.float32)
        # Normalise ring index to [0,1] so a 128-ring sensor matches the scale
        # the pretrained model expects (trained on 64-ring, ring values 0-63).
        # Normalise intensity to [0,1] (raw values are 0-65535 or 0-255 depending on firmware)
        lidar_frame[:, 3] = lidar_frame[:, 3] / (lidar_frame[:, 3].max() + 1e-6)
        # Normalise ring to [0,1] (128-ring → 0..127 → divide by 127)
        lidar_frame[:, 4] = lidar_frame[:, 4] / (lidar_frame[:, 4].max() + 1e-6)
        print(f'  Got LiDAR: {lidar_frame.shape[0]} points  '
              f'(intensity [0,1]  ring [0,1]  channels: x,y,z,intensity,ring)')
        break  # both frames acquired

cv2.imwrite(f'{OUTPUT_DIR}/01_raw_image.jpg', img_frame)
print(f'  Saved: 01_raw_image.jpg')


# ── Stage 2: Run YOLO segmentation ────────────────────────────────────────────
print('\n[Stage 2] Running YOLO segmentation...')

from ultralytics import YOLO

model = YOLO(YOLO_MODEL)
print(f'  Model file : {os.path.abspath(YOLO_MODEL)}')
print(f'  Model type : {type(model.model).__name__}')
img_rgb = cv2.cvtColor(img_frame, cv2.COLOR_BGR2RGB)
h, w = img_frame.shape[:2]

# COCO classes used for PointPainting scores
PAINT_CLASSES = {0: 'ped', 2: 'car', 1: 'cyc'}

# Single YOLO call — results reused for both hard label mask and soft score maps
results = model(img_rgb, verbose=False, conf=0.25, imgsz=(h, w))

# ── Hard label mask (for visualisation, Stages 3 & 4) ────────────────────────
label_mask = np.full((h, w), -1, dtype=np.int32)
PRIORITY   = {0: 10, 1: 9, 3: 8}  # person > bicycle > motorcycle > everything else

for result in results:
    if result.masks is None:
        continue
    masks   = result.masks.data.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    pairs   = sorted(zip(masks, classes), key=lambda mc: PRIORITY.get(mc[1], 0))
    for mask, cls_id in pairs:
        mask_u8     = (mask * 255).astype(np.uint8)
        mask_resized = cv2.resize(mask_u8, (w, h), interpolation=cv2.INTER_LINEAR)
        label_mask[mask_resized > 200] = cls_id

kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
clean_mask = np.full_like(label_mask, -1)
for cls_id in np.unique(label_mask[label_mask >= 0]):
    binary  = (label_mask == cls_id).astype(np.uint8)
    dilated = cv2.dilate(binary, kernel, iterations=1)
    clean_mask[dilated == 1] = cls_id
label_mask = clean_mask

# ── Soft score maps (for PointPainting, Stage 5) ─────────────────────────────
# Per-pixel, winner-take-all across instances → Gaussian blur for smooth scores
score_maps = {cls_id: np.zeros((h, w), dtype=np.float32)
              for cls_id in PAINT_CLASSES}

instance_scores = []
for result in results:
    if result.masks is None:
        continue
    masks   = result.masks.data.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    confs   = result.boxes.conf.cpu().numpy()
    for mask, cls_id, conf in zip(masks, classes, confs):
        if cls_id not in PAINT_CLASSES:
            continue
        m = cv2.resize((mask * 255).astype(np.uint8), (w, h),
                       interpolation=cv2.INTER_LINEAR).astype(np.float32) / 255.0
        instance_scores.append((m * float(conf), cls_id))

if instance_scores:
    stack   = np.stack([s for s, _ in instance_scores], axis=0)   # (N, H, W)
    winner  = np.argmax(stack, axis=0)
    any_det = stack.max(axis=0) > 0
    for idx, (sm, cls_id) in enumerate(instance_scores):
        owns = (winner == idx) & any_det
        score_maps[cls_id] = np.maximum(
            score_maps[cls_id],
            np.where(owns, sm, 0.0).astype(np.float32)
        )

for cls_id in PAINT_CLASSES:
    score_maps[cls_id] = cv2.GaussianBlur(score_maps[cls_id], (5, 5), 0)

detected = np.unique(label_mask[label_mask >= 0])
print(f'  Detected classes: {[(c, model.names[c]) for c in detected if c in model.names]}')

mask_vis = np.zeros((h, w, 3), dtype=np.uint8)
for cls_id, color in CLASS_COLORS_BGR.items():
    mask_vis[label_mask == cls_id] = color

cv2.imwrite(f'{OUTPUT_DIR}/02_yolo_mask.jpg', mask_vis)
print(f'  Saved: 02_yolo_mask.jpg')


# ── Stage 3: Overlay mask on image ────────────────────────────────────────────
print('\n[Stage 3] Creating segmentation overlay...')

overlay = cv2.addWeighted(img_frame, 0.6, mask_vis, 0.4, 0)

# Add class legend
y = 30
for cls_id, color in CLASS_COLORS_BGR.items():
    if cls_id in detected:
        name = model.names.get(cls_id, str(cls_id))
        cv2.rectangle(overlay, (10, y-15), (30, y+5), color, -1)
        cv2.putText(overlay, name, (35, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        y += 30

cv2.imwrite(f'{OUTPUT_DIR}/03_overlay.jpg', overlay)
print(f'  Saved: 03_overlay.jpg')


# ── Stage 4: Project LiDAR points onto image ──────────────────────────────────
print('\n[Stage 4] Projecting LiDAR points onto image...')

sys.path.insert(0, '/workspace/ros2_ws/src/perception_framework')
sys.path.insert(0, '/workspace/ros2_ws/install/perception_framework/lib/python3.10/site-packages')

from perception_framework.lidar_to_image_projection import KittiLidarToImageProjector

projector = KittiLidarToImageProjector(CALIB_PATH)

# Keep all points — PointPillars handles its own z range clipping internally.
# The KITTI z range [-3, 3] (extended for our lower-mounted sensor) will drop
# true sky/building points while retaining road + objects of interest.
lidar_filtered = lidar_frame
ground_mask = np.ones(len(lidar_frame), dtype=bool)

image_points, valid_lidar = projector.project_lidar_to_image(lidar_filtered[:, :3], (h, w))

print(f'  Total points: {len(lidar_frame)}')
print(f'  After ground filter: {len(lidar_filtered)}')
print(f'  Points in camera frame: {len(image_points)} ({100*len(image_points)/len(lidar_frame):.1f}%)')

# Re-derive depths aligned to image_points
all_cam = projector.lidar_to_camera(lidar_filtered[:, :3])
front_mask = all_cam[:, 2] > 0
all_cam_front = all_cam[front_mask]
uv_all = (projector.P2[:, :3] @ all_cam_front.T).T
uv_all = uv_all[:, :2] / uv_all[:, 2:3]
in_bounds = (uv_all[:, 0] >= 0) & (uv_all[:, 0] < w) & (uv_all[:, 1] >= 0) & (uv_all[:, 1] < h)
depths = all_cam_front[in_bounds, 2]  # aligned to image_points

# Neighbourhood majority-vote for class lookup: avoids single-pixel mask gaps
def majority_class(lm, cy, cx, r=3):
    y0, y1 = max(0, cy - r), min(lm.shape[0], cy + r + 1)
    x0, x1 = max(0, cx - r), min(lm.shape[1], cx + r + 1)
    patch = lm[y0:y1, x0:x1]
    valid = patch[patch >= 0]
    if len(valid) == 0:
        return -1
    counts = np.bincount(valid)
    return int(np.argmax(counts))

# Sort by depth descending so far points are drawn first (occluded by near points)
order = np.argsort(depths)[::-1]
image_points_sorted = image_points[order]
depths_sorted = depths[order]

lidar_img = img_frame.copy()
painted = 0
class_counts = {}

for pt_px, depth in zip(image_points_sorted, depths_sorted):
    u, v = int(pt_px[0]), int(pt_px[1])
    if 0 <= u < w and 0 <= v < h:
        cls_id = majority_class(label_mask, v, u, r=3)
        color = CLASS_COLORS_BGR.get(cls_id, UNPAINTED_BGR)
        # Depth-scaled radius: 6px at 5m, 4px at 10m, 3px at 20m+
        radius = max(3, int(30.0 / max(depth, 1.0)))
        cv2.circle(lidar_img, (u, v), radius=radius, color=color, thickness=-1)
        if cls_id >= 0:
            painted += 1
            class_counts[cls_id] = class_counts.get(cls_id, 0) + 1

print(f'  Painted points (non-background): {painted}')
for cls_id, count in sorted(class_counts.items()):
    name = model.names.get(cls_id, str(cls_id))
    print(f'    {name} (class {cls_id}): {count} points')

cv2.imwrite(f'{OUTPUT_DIR}/04_lidar_projected.jpg', lidar_img)
print(f'  Saved: 04_lidar_projected.jpg')

# ── Stage 5: PointPainting — attach YOLO scores to each LiDAR point ──────────
print('\n[Stage 5] PointPainting...')

sys.path.insert(0, _script_dir)
from point_pillars import (paint_point_cloud, PointPillars, PointPillarsConfig,
                            build_pointpillars, draw_boxes_on_image, draw_bev)

painted = paint_point_cloud(
    lidar_filtered,          # (N, 5): x,y,z,intensity,ring_normalised
    score_maps,              # {coco_id: (H,W) float32}
    projector,               # KittiLidarToImageProjector
    (h, w),
)
print(f'  Painted cloud: {painted.shape}  '
      f'[x, y, z, intensity, ring, score_ped, score_car, score_cyc]')

# Count non-zero painting scores
n_ped = (painted[:, 5] > 0.1).sum()
n_car = (painted[:, 6] > 0.1).sum()
n_cyc = (painted[:, 7] > 0.1).sum()
print(f'  Points with score > 0.1:  ped={n_ped}  car={n_car}  cyc={n_cyc}')

# Visualise painting scores as a heatmap on the image
score_combined = painted[:, 5:].max(axis=1)   # max score across ped/car/cyc (cols 5,6,7)
cam_all2  = projector.lidar_to_camera(lidar_filtered[:, :3])
front2    = cam_all2[:, 2] > 0
uv2       = (projector.P2[:, :3] @ cam_all2[front2].T).T
uv2       = uv2[:, :2] / uv2[:, 2:3]
in2       = ((uv2[:, 0] >= 0) & (uv2[:, 0] < w) &
             (uv2[:, 1] >= 0) & (uv2[:, 1] < h))
front_idx2 = np.where(front2)[0][in2]

score_img = img_frame.copy()
for idx, (u_f, v_f) in zip(front_idx2,
                             uv2[in2]):
    u_i, v_i = int(u_f), int(v_f)
    sc = float(score_combined[idx])
    if sc > 0.05:
        # colour: low score = yellow, high = red
        color = (0, int(255 * (1 - sc)), 255)
        cv2.circle(score_img, (u_i, v_i), 3, color, -1)

cv2.imwrite(f'{OUTPUT_DIR}/05_painted_scores.jpg', score_img)
print(f'  Saved: 05_painted_scores.jpg')


# ── Stage 6: PointPillars 3-D detection ───────────────────────────────────────
print('\n[Stage 6] PointPillars detection...')

# ── Coordinate frame alignment ────────────────────────────────────────────────
# KITTI PointPillars expects: x=forward, y=left, z=up, sensor at ~1.7m above ground.
# Our sensor frame (from raw bag data):
#   x: [-151, +200]  — lateral (full 360° scan)
#   y: [-46,  +76]   — another lateral axis
#   z: [-1.5, +30.5] — vertical (buildings/height, ground already filtered)
# Tr_velo_to_cam row3 ≈ [0.987, -0.022, -0.162] meaning camera depth ≈ 0.987*x_lidar
# → x_lidar IS the forward axis in the calibration.
# But raw x goes -151..+200 which is a full 360° range — the forward half is x>0.
# The KITTI detection zone x=[0,69.12] covers exactly the forward half.
# z going up to 30.5m with ground at -1.5m means z is UP (buildings are tall).
# y range [-46,+76] is the left-right lateral axis.
# So the frame IS already x=forward, y=left, z=up — we just need to feed it directly.
# No axis remapping needed; the KITTI voxel range x=[0,69.12] will select forward points.
raw_xyz = painted[:, :3].copy()
painted_kitti = painted.copy()  # no remap needed

print(f'  Point cloud (original Velodyne frame, no remap):')
print(f'    x (fwd) range: [{painted_kitti[:,0].min():.1f}, {painted_kitti[:,0].max():.1f}]')
print(f'    y (lat) range: [{painted_kitti[:,1].min():.1f}, {painted_kitti[:,1].max():.1f}]')
print(f'    z (up)  range: [{painted_kitti[:,2].min():.1f}, {painted_kitti[:,2].max():.1f}]')
in_kitti = ((painted_kitti[:,0] >= 0) & (painted_kitti[:,0] <= 69.12) &
            (painted_kitti[:,1] >= -39.68) & (painted_kitti[:,1] <= 39.68) &
            (painted_kitti[:,2] >= -3.0) & (painted_kitti[:,2] <= 1.0))
print(f'    Points inside KITTI detection range: {in_kitti.sum()} / {len(painted_kitti)}')
# Break down by axis to find which constraint kills the most points
in_x = (painted_kitti[:,0] >= 0) & (painted_kitti[:,0] <= 69.12)
in_y = (painted_kitti[:,1] >= -39.68) & (painted_kitti[:,1] <= 39.68)
in_z = (painted_kitti[:,2] >= -3.0) & (painted_kitti[:,2] <= 1.0)
print(f'    In x=[0,69]: {in_x.sum()}  |  in y=[-40,40]: {in_y.sum()}  |  in z=[-3,1]: {in_z.sum()}')
print(f'    z histogram: <-3: {(painted_kitti[:,2]<-3).sum()}  '
      f'-3..0: {((painted_kitti[:,2]>=-3)&(painted_kitti[:,2]<0)).sum()}  '
      f'0..1: {((painted_kitti[:,2]>=0)&(painted_kitti[:,2]<1)).sum()}  '
      f'1..3: {((painted_kitti[:,2]>=1)&(painted_kitti[:,2]<3)).sum()}  '
      f'>3: {(painted_kitti[:,2]>=3).sum()}')
# Density in forward zone: how many points per pillar on average?
fwd = painted_kitti[in_x & in_y]
if len(fwd):
    # Estimate pillar density
    n_pillars_fwd = len(np.unique(
        (((fwd[:,0]) / 0.16).astype(int)) * 10000 +
        (((fwd[:,1] + 39.68) / 0.16).astype(int))
    ))
    print(f'    Forward zone: {len(fwd)} pts in ~{n_pillars_fwd} pillars '
          f'({len(fwd)/max(n_pillars_fwd,1):.1f} pts/pillar avg)')

# Use painted=False with 5 channels [x,y,z,intensity,ring] — exact match to
# the pretrained checkpoint which has PFN linear weight (64,10) = 5+5 aug.
# Ring is normalised to [0,1] so our 128-ring sensor maps to the same scale
# as the 64-ring KITTI training data.
pp_model = build_pointpillars(painted=False, score_thr=args.pp_score_thr)
cfg = pp_model.cfg

pretrained = False
if args.pp_checkpoint:
    pretrained = pp_model.load_pretrained(args.pp_checkpoint)

if not pretrained:
    print('  WARNING: no pretrained checkpoint loaded — detections are random.')

# 5-channel input: [x,y,z,intensity,ring_normalised]
points_5ch = painted_kitti[:, :5]

# ── Diagnostic: raw score distribution ────────────────────────────────────────
import torch
pp_model.eval()
with torch.no_grad():
    _pnp, _cnp, _nnp = pp_model._voxelise(points_5ch, cfg.max_voxels_test)
    print(f'  Voxelised: {_pnp.shape[0]} pillars')
    _pf  = pp_model.pfn(torch.from_numpy(_pnp), torch.from_numpy(_nnp))
    _bev = pp_model.scatter(_pf, torch.from_numpy(_cnp), batch_size=1)
    _cls_raw = pp_model.head(pp_model.neck(pp_model.backbone(_bev)))[0]
    _sig = torch.sigmoid(_cls_raw)[0].cpu().numpy()
    print(f'  cls sigmoid — min={_sig.min():.4f}  max={_sig.max():.4f}  '
          f'mean={_sig.mean():.4f}  >0.3: {(_sig>0.3).sum()}  >0.1: {(_sig>0.1).sum()}')

boxes, scores, labels = pp_model.detect(points_5ch)
print(f'  Detections (thr={args.pp_score_thr}): {len(boxes)}')
for box, score, label in zip(boxes, scores, labels):
    cls_name = PointPillars.CLASS_NAMES.get(int(label), str(label))
    cx, cy, cz, bw, bl, bh, heading = box
    print(f'    {cls_name:12s}  score={score:.3f}  '
          f'pos=({cx:.1f}, {cy:.1f}, {cz:.1f})  '
          f'size=({bw:.2f}×{bl:.2f}×{bh:.2f})  heading={heading:.2f}rad')


# ── Stage 7: Visualise 3-D detections ─────────────────────────────────────────
print('\n[Stage 7] Visualising detections...')

# Boxes are in the same frame as the input point cloud (no axis remap was applied),
# so they can be projected directly using the calibrated projector.
det_img = draw_boxes_on_image(img_frame, boxes, scores, labels, projector)
cv2.imwrite(f'{OUTPUT_DIR}/06_detections.jpg', det_img)
print(f'  Saved: 06_detections.jpg')

bev_img = draw_bev(boxes, scores, labels, cfg)
cv2.imwrite(f'{OUTPUT_DIR}/07_bev.jpg', bev_img)
print(f'  Saved: 07_bev.jpg')

print(f'\nDone.  Output: {OUTPUT_DIR}/')
print(f'To reproduce: python3 {os.path.basename(__file__)} --seed {target_idx}')
if not pretrained:
    print('\nNext step: provide a pretrained PointPillars checkpoint:')
    print('  python3 ... --pp-checkpoint /workspace/pointpillars_kitti.pth')
