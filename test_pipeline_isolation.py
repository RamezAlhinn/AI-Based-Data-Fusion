"""
Pipeline isolation test — no ROS, no live node needed.

What this does:
  1. Counts all image frames in the bag, picks one at random
  2. Extracts the chosen image + the nearest LiDAR frame
  3. Runs YOLO segmentation on the image
  4. Projects LiDAR points onto the image using calib.txt
  5. Colours each projected point by its class
  6. Saves 4 images so you can visually verify each stage

Run inside the container:
  python3 /workspace/test_pipeline_isolation.py           # random frame
  python3 /workspace/test_pipeline_isolation.py --seed 7  # reproducible

Output (saved to /workspace/isolation_output/):
  01_raw_image.jpg          — original camera frame
  02_yolo_mask.jpg          — YOLO class mask (colour per class)
  03_overlay.jpg            — mask blended on top of image
  04_lidar_projected.jpg    — LiDAR points drawn ON the image, coloured by class
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
args = parser.parse_args()

rng = random.Random(args.seed)

# ── Paths ──────────────────────────────────────────────────────────────────────
BAG_PATH   = '/workspace/studentProject1/'
CALIB_PATH = '/workspace/calib.txt'
OUTPUT_DIR = '/workspace/isolation_output'
_script_dir = os.path.dirname(os.path.abspath(__file__))
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
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        lidar_frame = np.array([(p[0], p[1], p[2]) for p in pts], dtype=np.float32)
        print(f'  Got LiDAR: {lidar_frame.shape[0]} points')
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

# Fix 4: pass exact image size so YOLO doesn't letterbox/pad internally
results = model(img_rgb, verbose=False, conf=0.25, imgsz=(h, w))
label_mask = np.full((h, w), -1, dtype=np.int32)  # -1 = background/no detection

PRIORITY = {0: 10, 1: 9, 3: 8}  # person > bicycle > motorcycle > everything else

for result in results:
    if result.masks is None:
        continue
    masks   = result.masks.data.cpu().numpy()
    classes = result.boxes.cls.cpu().numpy().astype(int)
    pairs = sorted(zip(masks, classes), key=lambda mc: PRIORITY.get(mc[1], 0))
    for mask, cls_id in pairs:
        mask_u8 = (mask * 255).astype(np.uint8)
        # Fix 1: bilinear resize + tight threshold for clean mask boundaries
        mask_resized = cv2.resize(mask_u8, (w, h), interpolation=cv2.INTER_LINEAR)
        label_mask[mask_resized > 200] = cls_id

# Slight dilation to close small holes inside masks before lookup
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))
clean_mask = np.full_like(label_mask, -1)
for cls_id in np.unique(label_mask[label_mask >= 0]):
    binary = (label_mask == cls_id).astype(np.uint8)
    dilated = cv2.dilate(binary, kernel, iterations=1)
    clean_mask[dilated == 1] = cls_id
label_mask = clean_mask

# Visualise mask
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

# Filter ground points: in Velodyne coords z is up, so z < -1.5 m is ground
GROUND_Z_THRESH = -1.5
ground_mask = lidar_frame[:, 2] > GROUND_Z_THRESH
lidar_filtered = lidar_frame[ground_mask]

image_points, valid_lidar = projector.project_lidar_to_image(lidar_filtered, (h, w))

print(f'  Total points: {len(lidar_frame)}')
print(f'  After ground filter: {len(lidar_filtered)}')
print(f'  Points in camera frame: {len(image_points)} ({100*len(image_points)/len(lidar_frame):.1f}%)')

# Compute depth for each projected point (camera Z) for depth-scaled radius
cam_points = projector.lidar_to_camera(lidar_filtered)
valid_depth = cam_points[ground_mask[:len(cam_points)], 2] if False else None  # placeholder
# Re-derive depths aligned to image_points via the same valid_depth_mask path
all_cam = projector.lidar_to_camera(lidar_filtered)
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

print(f'\nDone. Open /workspace/isolation_output/ to see all 4 images.')
print('To reproduce this exact frame: python3 test_pipeline_isolation.py --seed', target_idx)
