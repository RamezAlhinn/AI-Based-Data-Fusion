# Quickstart — See the Pipeline Running

All commands run **inside the Dev Container** (`><` → Reopen in Container).  
Open four terminal tabs in VS Code (`Ctrl+Shift+\`` to split).

> **Visualisation:** RViz2 has no display inside the container. We use **Foxglove Studio** on your Mac instead, connected over a WebSocket bridge on port `9090`.

---

## Step 1 — Build (once per container start)

```bash
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select perception_framework point_painting
source install/setup.bash
```

---

## Step 2 — Run the painting node (Terminal 1)

### Mode A — Projection only (no segmentation model needed)

```bash
source /workspace/ros2_ws/install/setup.bash

ros2 run point_painting painting_node \
  --ros-args -p calib_file:=/workspace/calib.txt
```

Expected output:
```
[INFO] Loaded calibration from: /workspace/calib.txt
[WARN] No segmentation model loaded — node will use raw image channel as label map.
[INFO] PaintingNode started, waiting for synced messages...
```

### Mode B — Full pipeline with YOLO segmentation

No model file needed. YOLO11s-seg downloads automatically on first run (~22 MB, cached).  
Uses native COCO class IDs: `0`=person, `1`=bicycle, `2`=car, `3`=motorcycle, `5`=bus, `7`=truck.

```bash
source /workspace/ros2_ws/install/setup.bash

ros2 run point_painting painting_node \
  --ros-args -p calib_file:=/workspace/calib.txt
```

Expected output:
```
[INFO] Loaded calibration from: /workspace/calib.txt
[INFO] Segmentation model loaded: yolo11m-seg.pt
[INFO] PaintingNode started, waiting for synced messages...
```

To use a custom model file (e.g. a fine-tuned checkpoint):
```bash
ros2 run point_painting painting_node \
  --ros-args \
  -p calib_file:=/workspace/calib.txt \
  -p checkpoint_path:=/workspace/yolo11m-seg.pt
```

**Colour map in Foxglove 3D view:**
- 🔴 Red = person
- 🟢 Green = car / truck
- 🔵 Blue = bicycle
- 🟠 Orange = motorcycle
- 🟡 Yellow = bus

---

## Step 3 — Play the bag (Terminal 2)

```bash
ros2 bag play /workspace/studentProject1/ --loop
```

---

## Step 4 — Watch the painting output (Terminal 3)

```bash
ros2 topic echo /painting/debug
```

You will see a line per frame:
```
data: 'frame=1 painted=5050 skipped=59070'
```

- **painted** = LiDAR points that projected onto a detected object
- **skipped** = points behind the camera, outside the frame, or on background

---

## Step 5 — Visualise in Foxglove Studio (Terminal 4)

Start the Foxglove WebSocket bridge:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=9090
```

Then on your **Mac**:
1. Open [Foxglove Studio](https://app.foxglove.dev) (browser or desktop app)
2. Click **Open connection** → **Foxglove WebSocket** → `ws://localhost:9090`

Inside Foxglove add these panels:
- **3D** → subscribe to `/painting/painted_cloud` (set Color mode → RGB) to see the painted point cloud
- **Image** → subscribe to `/painting/segmentation_overlay` to verify YOLO masks on the camera image
- **Image** → subscribe to `/blackfly_s/cam0/image_rectified` for the raw camera feed
- **Raw Messages** → subscribe to `/painting/debug` to watch painted/skipped counts

Port `9090` is automatically forwarded by the devcontainer — no extra configuration needed.

---

## Optional — Run the pipeline isolation test (no ROS needed)

Runs the full pipeline (YOLO → PointPainting → PointPillars) on one bag frame and saves
verification images to `/workspace/AI-Based-Data-Fusion/isolation_output/`.

```bash
# Minimal run — random frame, no 3-D detection checkpoint
python3 /workspace/test_pipeline_isolation.py

# With PointPillars checkpoint (recommended)
python3 /workspace/test_pipeline_isolation.py \
    --pp-checkpoint /workspace/pointpillars_kitti.pth

# Reproduce a specific frame (use the seed printed at the end of any run)
python3 /workspace/test_pipeline_isolation.py \
    --seed 42 \
    --pp-checkpoint /workspace/pointpillars_kitti.pth

# Lower score threshold to surface more detections
python3 /workspace/test_pipeline_isolation.py \
    --pp-checkpoint /workspace/pointpillars_kitti.pth \
    --pp-score-thr 0.10

# Tune Z-axis alignment (sensor mounted higher/lower than KITTI baseline)
python3 /workspace/test_pipeline_isolation.py \
    --pp-checkpoint /workspace/pointpillars_kitti.pth \
    --z-offset -0.25

# Fix intensity scale — this Velodyne outputs raw values up to ~1910 (not 0-255).
# Use p95 of the raw distribution (~1072) to avoid outlier compression.
python3 /workspace/test_pipeline_isolation.py \
    --pp-checkpoint /workspace/pointpillars_kitti.pth \
    --intensity-max 1072.0

# Correct a heading misalignment (radians)
python3 /workspace/test_pipeline_isolation.py \
    --pp-checkpoint /workspace/pointpillars_kitti.pth \
    --yaw-offset 0.05
```

Output images:
```
isolation_output/01_raw_image.jpg          ← original camera frame
isolation_output/02_yolo_mask.jpg          ← YOLO class mask (colour per class)
isolation_output/03_overlay.jpg            ← mask blended on image
isolation_output/04_lidar_projected.jpg    ← LiDAR points on image, coloured by class
isolation_output/05_painted_scores.jpg     ← per-point painting score heat map
isolation_output/06_detections.jpg         ← camera image with projected 3-D boxes
isolation_output/07_bev.jpg                ← bird's-eye view with box footprints
```

If `04_lidar_projected.jpg` shows green dots on cars and red dots on people, painting is correct.
If `06_detections.jpg` / `07_bev.jpg` show boxes, PointPillars is working end-to-end.

---

## Optional — Extract frames from the bag

```bash
cd /workspace
python3 -c "
import sys
sys.path.insert(0, '/workspace/ros2_ws/src/point_painting')
from point_painting.rosbag_extractor import extract_bag_data
extract_bag_data(
    '/workspace/studentProject1/',
    '/blackfly_s/cam0/image_rectified',
    '/velodyne/points_raw',
    'output_data'
)
"
```

---

## Optional — Run the standalone tests (no ROS needed)

```bash
python3 /workspace/ros2_ws/src/point_painting/test/test_painting_node.py
```

Expected:
```
Running test_no_projector_skips_all_points ... PASSED
Running test_empty_point_cloud ... PASSED
Running test_class_ids_length_matches_input ... PASSED
ALL TESTS PASSED
```
