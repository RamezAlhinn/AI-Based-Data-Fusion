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
parser.add_argument('--pp-score-thr', type=float, default=0.05,
                    help='PointPillars score threshold (default: 0.05)')
parser.add_argument('--intensity-max', type=float, default=1910.0,
                    help='Absolute raw intensity ceiling of the sensor (default: 1910.0 — measured p100 for this Velodyne)')
parser.add_argument('--z-offset', type=float, default=-0.30,
                    help='Vertical shift applied to all points before detection (metres, default: -0.30)')
parser.add_argument('--yaw-offset', type=float, default=0.0,
                    help='Rotation around Z-axis applied before detection (radians, default: 0.0)')
parser.add_argument('--no-filter-points', dest='filter_points', action='store_false',
                    help='Disable YOLO-based point cloud filtering (default: filtering is enabled)')
parser.set_defaults(filter_points=True)
parser.add_argument('--filter-thr', type=float, default=0.1,
                    help='YOLO score threshold for point cloud filtering (default: 0.1)')
parser.add_argument('--dilate-radius', type=int, default=15,
                    help='Dilation radius (in pillars) for filtering (default: 15)')
args = parser.parse_args()

rng = random.Random(args.seed)

# ── Paths ──────────────────────────────────────────────────────────────────────
_script_dir = os.path.dirname(os.path.abspath(__file__))
BAG_PATH   = '/workspace/studentProject1'
CALIB_PATH = os.path.join(_script_dir, 'calib.txt')
OUTPUT_DIR = os.path.join(_script_dir, 'isolation_output')
YOLO_MODEL = args.checkpoint
if not YOLO_MODEL:
    if os.path.exists(os.path.join(_script_dir, 'models', 'yolo11m-seg.pt')):
        YOLO_MODEL = os.path.join(_script_dir, 'models', 'yolo11m-seg.pt')
    elif os.path.exists(os.path.join(_script_dir, 'yolo11m-seg.pt')):
        YOLO_MODEL = os.path.join(_script_dir, 'yolo11m-seg.pt')
    elif os.path.exists('/yolo11m-seg.pt'):
        YOLO_MODEL = '/yolo11m-seg.pt'
    else:
        YOLO_MODEL = 'yolo11m-seg.pt'

PP_CHECKPOINT = args.pp_checkpoint
if not PP_CHECKPOINT:
    if os.path.exists(os.path.join(_script_dir, 'models', 'pointpillars_kitti_3class.pth')):
        PP_CHECKPOINT = os.path.join(_script_dir, 'models', 'pointpillars_kitti_3class.pth')
    elif os.path.exists(os.path.join(_script_dir, 'pointpillars_kitti_3class.pth')):
        PP_CHECKPOINT = os.path.join(_script_dir, 'pointpillars_kitti_3class.pth')
    elif os.path.exists('/workspace/models/pointpillars_kitti_3class.pth'):
        PP_CHECKPOINT = '/workspace/models/pointpillars_kitti_3class.pth'
    elif os.path.exists('/workspace/pointpillars_kitti_3class.pth'):
        PP_CHECKPOINT = '/workspace/pointpillars_kitti_3class.pth'
    elif os.path.exists(os.path.join(_script_dir, 'models', 'pointpillars_kitti.pth')):
        PP_CHECKPOINT = os.path.join(_script_dir, 'models', 'pointpillars_kitti.pth')
    elif os.path.exists(os.path.join(_script_dir, 'pointpillars_kitti.pth')):
        PP_CHECKPOINT = os.path.join(_script_dir, 'pointpillars_kitti.pth')


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
        raw_i = lidar_frame[:, 3]
        raw_p95 = np.percentile(raw_i, 95)
        print(f'  Raw intensity — min={raw_i.min():.1f}  max={raw_i.max():.1f}  '
              f'mean={raw_i.mean():.1f}  p95={raw_p95:.1f}')
        # Stable absolute intensity scale — avoids outlier compression from max()
        lidar_frame[:, 3] = np.clip(lidar_frame[:, 3] / args.intensity_max, 0.0, 1.0)
        # Normalise ring to [0,1] (128-ring → 0..127 → divide by max)
        lidar_frame[:, 4] = lidar_frame[:, 4] / (lidar_frame[:, 4].max() + 1e-6)
        print(f'  Got LiDAR: {lidar_frame.shape[0]} points  '
              f'(intensity scaled by 1/{args.intensity_max:.0f}  ring [0,1]  channels: x,y,z,intensity,ring)')
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
    yolo_results=results,
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

# Use painted=False with 4 channels [x,y,z,intensity] — exact match to the
# pretrained KITTI checkpoint (PFN linear weight 64×10 = 4 base + 6 augmented).
# The PFN forward appends dx_c/dy_c/dz_c (cluster offsets), dx_p/dy_p (pillar
# centre offsets), and range depth — always 10 channels total regardless of mode.
pp_model = build_pointpillars(painted=False, score_thr=args.pp_score_thr)
cfg = pp_model.cfg

pretrained = False
if PP_CHECKPOINT:
    pretrained = pp_model.load_pretrained(PP_CHECKPOINT)

if pretrained:
    print(f'  Model file : {os.path.abspath(PP_CHECKPOINT)}')
else:
    print('  WARNING: no pretrained checkpoint loaded — detections are random.')

# ── YOLO-based point cloud filtering ──────────────────────────────────────────
if args.filter_points:
    from point_pillars import filter_point_cloud_by_yolo
    filtered_kitti = filter_point_cloud_by_yolo(
        painted_kitti,
        cfg,
        filter_thr=args.filter_thr,
        dilate_radius=args.dilate_radius
    )
    points_4ch = filtered_kitti[:, :4].copy()
    print(f'  Pillar filtering: kept {len(points_4ch)} / {len(painted_kitti)} points '
          f'in pillars containing YOLO scores > {args.filter_thr} (dilate radius = {args.dilate_radius})')
else:
    # 4-channel input: [x, y, z, intensity]
    points_4ch = painted_kitti[:, :4].copy()

# ── Domain alignment ──────────────────────────────────────────────────────────
if args.z_offset != 0.0:
    points_4ch[:, 2] += args.z_offset

if args.yaw_offset != 0.0:
    cos_y = np.cos(args.yaw_offset)
    sin_y = np.sin(args.yaw_offset)
    x_rot = points_4ch[:, 0] * cos_y - points_4ch[:, 1] * sin_y
    y_rot = points_4ch[:, 0] * sin_y + points_4ch[:, 1] * cos_y
    points_4ch[:, 0] = x_rot
    points_4ch[:, 1] = y_rot

print(f'  After alignment (z_offset={args.z_offset:+.3f}  yaw={args.yaw_offset:+.4f}rad):')
print(f'    x range: [{points_4ch[:,0].min():.2f}, {points_4ch[:,0].max():.2f}]')
print(f'    y range: [{points_4ch[:,1].min():.2f}, {points_4ch[:,1].max():.2f}]')
print(f'    z range: [{points_4ch[:,2].min():.2f}, {points_4ch[:,2].max():.2f}]')
print(f'    intensity mean: {points_4ch[:,3].mean():.4f}  max: {points_4ch[:,3].max():.4f}')

# ── Diagnostic: raw score distribution ────────────────────────────────────────
import torch
pp_model.eval()
with torch.no_grad():
    _pnp, _cnp, _nnp = pp_model._voxelise(points_4ch, cfg.max_voxels_test)
    print(f'  Voxelised: {_pnp.shape[0]} pillars')
    _pf  = pp_model.pfn(torch.from_numpy(_pnp), torch.from_numpy(_nnp),
                        torch.from_numpy(_cnp))
    _bev = pp_model.scatter(_pf, torch.from_numpy(_cnp), batch_size=1)
    _cls_raw = pp_model.head(pp_model.neck(pp_model.backbone(_bev)))[0]
    _sig = torch.sigmoid(_cls_raw)[0].cpu().numpy()
    print(f'  cls sigmoid — min={_sig.min():.4f}  max={_sig.max():.4f}  '
          f'mean={_sig.mean():.4f}  >0.3: {(_sig>0.3).sum()}  >0.1: {(_sig>0.1).sum()}')
    # ── Tuning hints ─────────────────────────────────────────────────────────
    _i_mean = points_4ch[:, 3].mean()
    if _i_mean > 0.3:
        print(f'  [hint] intensity mean {_i_mean:.3f} is high (KITTI expects ~0.10-0.20). '
              f'Try --intensity-max {args.intensity_max * _i_mean / 0.15:.0f} '
              f'(scales mean to 0.15)')

print(f'  Point cloud shape passed to detect(): {points_4ch.shape}')
boxes, scores, labels = pp_model.detect(points_4ch)
print(f'  Detections (thr={args.pp_score_thr}): {len(boxes)}')
for box, score, label in zip(boxes, scores, labels):
    cls_name = PointPillars.CLASS_NAMES.get(int(label), str(label))
    cx, cy, cz, bw, bl, bh, heading = box
    print(f'    {cls_name:12s}  score={score:.3f}  '
          f'pos=({cx:.1f}, {cy:.1f}, {cz:.1f})  '
          f'size=({bw:.2f}×{bl:.2f}×{bh:.2f})  heading={heading:.2f}rad')


# ── Stage 7: Visualise 3-D detections ─────────────────────────────────────────
print('\n[Stage 7] Visualising detections...')

# Boxes come out of detect() in the ALIGNED frame (after z_offset + yaw_offset were
# applied to points_4ch). The raw-Velodyne projector does NOT know about these shifts,
# so we must UNDO them on the box centres before projecting onto the image.
vis_boxes = boxes.copy()

# Undo yaw rotation on (cx, cy)
if args.yaw_offset != 0.0:
    inv_yaw = -args.yaw_offset
    cos_i = np.cos(inv_yaw)
    sin_i = np.sin(inv_yaw)
    cx_r = vis_boxes[:, 0] * cos_i - vis_boxes[:, 1] * sin_i
    cy_r = vis_boxes[:, 0] * sin_i + vis_boxes[:, 1] * cos_i
    vis_boxes[:, 0] = cx_r
    vis_boxes[:, 1] = cy_r

# Undo z shift
if args.z_offset != 0.0:
    vis_boxes[:, 2] -= args.z_offset

# ── YOLO-guided lateral correction ────────────────────────────────────────────
# PointPillars gives good depth (x) and heading but the lateral (y) position can
# drift when the dilation buffer merges nearby objects into one point-cloud blob.
# Fix: project each box centre to image space → find nearest YOLO instance
# centroid of the matching class → back-project at the same camera depth.
#   PP label 0=Pedestrian → YOLO class 0=person
#   PP label 1=Cyclist    → YOLO class 1=bicycle
#   PP label 2=Car        → YOLO class 2=car
PP_TO_YOLO = {0: 0, 1: 1, 2: 2}

# Build per-YOLO-class list of instance centroids (u_c, v_c) in image pixels
yolo_centroids = {}
for result in results:
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

# Calibration: combined R0_rect @ Tr_velo_to_cam maps LiDAR → rectified cam
R0_Tr     = (projector.R0_rect @ projector.Tr_velo_to_cam)[:3, :]  # (3,4)
R_cam     = R0_Tr[:, :3]                                             # (3,3)
t_cam     = R0_Tr[:, 3]                                             # (3,)
P2        = projector.P2
fx, fy    = float(P2[0, 0]), float(P2[1, 1])
cx_cam_k  = float(P2[0, 2])
cy_cam_k  = float(P2[1, 2])
R_cam_inv = np.linalg.inv(R_cam)                                    # cam → LiDAR

corrected_boxes = vis_boxes.copy()
for i, (box, label) in enumerate(zip(vis_boxes, labels)):
    cx_l, cy_l, cz_l = float(box[0]), float(box[1]), float(box[2])

    # LiDAR → rectified camera
    lp_h      = np.array([cx_l, cy_l, cz_l, 1.0])
    cam_pt    = R0_Tr @ lp_h            # (3,) [cam_x, cam_y, cam_z]
    cam_depth = cam_pt[2]
    if cam_depth <= 0:
        continue

    # Project to pixel
    u_box = fx * cam_pt[0] / cam_depth + cx_cam_k
    v_box = fy * cam_pt[1] / cam_depth + cy_cam_k

    # Find matching YOLO centroids
    yolo_cls   = PP_TO_YOLO.get(int(label))
    candidates = yolo_centroids.get(yolo_cls, [])
    if not candidates:
        cls_name = PointPillars.CLASS_NAMES.get(int(label), str(label))
        print(f'  [snap] {cls_name}: no YOLO centroid for class {yolo_cls}, keeping box as-is')
        continue

    # Nearest centroid by pixel distance
    dists   = [((u_c - u_box)**2 + (v_c - v_box)**2)**0.5 for u_c, v_c in candidates]
    best_i  = int(np.argmin(dists))
    u_c, v_c = candidates[best_i]
    px_err  = dists[best_i]

    # Back-project (u_c, v_c) at the same camera depth → corrected camera point
    cam_x_new = (u_c - cx_cam_k) / fx * cam_depth
    cam_y_new = (v_c - cy_cam_k) / fy * cam_depth
    cam_pt_new = np.array([cam_x_new, cam_y_new, cam_depth])

    # Rectified camera → LiDAR:  x_L = R_cam^-1 @ (x_cam - t_cam)
    lp_new = R_cam_inv @ (cam_pt_new - t_cam)
    corrected_boxes[i, 0] = lp_new[0]
    corrected_boxes[i, 1] = lp_new[1]
    corrected_boxes[i, 2] = lp_new[2]

    cls_name = PointPillars.CLASS_NAMES.get(int(label), str(label))
    print(f'  [snap] {cls_name}: box_px=({u_box:.0f},{v_box:.0f}) → '
          f'yolo_px=({u_c:.0f},{v_c:.0f})  Δpx={px_err:.0f}  '
          f'Δy_lidar={lp_new[1]-cy_l:+.2f}m')

det_img = draw_boxes_on_image(img_frame, corrected_boxes, scores, labels, projector)
cv2.imwrite(f'{OUTPUT_DIR}/06_detections.jpg', det_img)
print(f'  Saved: 06_detections.jpg')

# BEV is drawn in the aligned frame (that is what the model sees), keep as-is
bev_img = draw_bev(boxes, scores, labels, cfg)
cv2.imwrite(f'{OUTPUT_DIR}/07_bev.jpg', bev_img)
print(f'  Saved: 07_bev.jpg')

print(f'\nDone.  Output: {OUTPUT_DIR}/')
print(f'To reproduce: python3 {os.path.basename(__file__)} --seed {target_idx}')
if not pretrained:
    print('\nNext step: provide a pretrained PointPillars checkpoint:')
    print('  python3 ... --pp-checkpoint /workspace/pointpillars_kitti_3class.pth')
