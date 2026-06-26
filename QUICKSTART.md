# Quickstart — AI-Based Data Fusion Pipeline

**PointPainting + Frustum 3D Detection + AB3DMOT Tracking**

All commands run **inside the Dev Container**. Open it in VS Code: click `><` (bottom-left) → **Reopen in Container**.

---

## How the Pipeline Works

```
Camera ──────────────────────────────────────────────────────┐
                                                             ▼
LiDAR ──► [PaintingNode] ──► scored_cloud ──► [FrustumNode] ──► 3D Boxes + Tracks
               │                                    │
               ▼                                    ▼
          YOLO Segmentation               AB3DMOT Kalman Tracker
                                                    │
                                                    ▼
                                         [MetricsLoggerNode] ──► Validation plots
```

| Node | What it does |
|---|---|
| `painting_node` | Runs YOLO on camera, projects LiDAR onto image, labels each point with a class score |
| `frustum_node` | Uses YOLO boxes to crop frustums of LiDAR points, clusters with DBSCAN, tracks with Kalman filter |
| `metrics_logger_node` | Subscribes to `/frustum/objects` and saves validation plots automatically on shutdown |
| `bev_recorder_node` | Records the `/frustum/bev` image stream to an MP4 video file |

---

## Step 1 — Build the Workspace

Run this **once** after cloning, or after adding new files / changing `setup.py`:

```bash
cd /workspace/ros2_ws && colcon build --symlink-install
source install/setup.bash
```

> After this you do **not** need to rebuild just because you edited a Python file — `--symlink-install` picks up changes automatically.

### Refreshing after a `git pull`

If you pulled new changes from Git or something isn't working, rebuild and re-source:

```bash
cd /workspace/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

---

## Step 2 — Run the Pipeline

Single command to start everything:

```bash
source /workspace/ros2_ws/install/setup.bash && ros2 launch frustum_detection pipeline.launch.py
```

This starts `painting_node`, `frustum_node`, `metrics_logger_node`, and `bev_recorder_node`, then plays the bag automatically after 3 seconds.

### Optional arguments

```bash
ros2 launch frustum_detection pipeline.launch.py \
    bag_path:=/workspace/studentProject1 \
    bag_rate:=0.5 \
    calib_file:=/workspace/calib.txt \
    output_dir:=/workspace/metrics
```

| Argument | Default | Description |
|---|---|---|
| `bag_path` | `/workspace/studentProject1` | Path to the ROS 2 bag directory |
| `bag_rate` | `0.1` | Playback speed — `0.1` = 10× slower, `1.0` = real-time |
| `calib_file` | `/workspace/calib.txt` | KITTI calibration file |
| `output_dir` | `/workspace/metrics` | Where validation plots are saved on shutdown |

### Stop and save plots

Press **Ctrl+C** — validation plots are automatically saved to `/workspace/metrics/`.

---

## Step 3 — Inspect Live Topic Output

While the pipeline is running, verify the raw tracking data in your terminal:

```bash
source /workspace/ros2_ws/install/setup.bash
ros2 topic echo /frustum/objects
```

This prints the full `TrackedObjectArray` message per frame, including track IDs, class names, positions, bounding box sizes, heading angles, confidence scores, prediction states, and velocities.

---

## Step 4 — Visualize in RViz2

### Windows users — start XLaunch first

RViz2 is a GUI application that needs a display server on Windows. Before running RViz2:

1. Download and install **VcXsrv** (XLaunch) from [sourceforge.net/projects/vcxsrv](https://sourceforge.net/projects/vcxsrv/)
2. Open **XLaunch** on Windows
3. Select **Multiple windows** → Next
4. Select **Start no client** → Next
5. Check **Disable access control** → Finish

Then inside the devcontainer terminal:

```bash
source /workspace/ros2_ws/install/setup.bash && rviz2
```

RViz2 will open as a window on your Windows desktop.

### Mac / Linux users

```bash
source /workspace/ros2_ws/install/setup.bash && rviz2
```

### RViz2 setup

1. Set **Fixed Frame** → `velodyne`
2. Add the following displays (**Add → By topic**):

| Topic | Display type | What you see |
|---|---|---|
| `/painting/painted_cloud` | PointCloud2 | Semantically coloured point cloud |
| `/frustum/markers` | MarkerArray | 3D bounding boxes (green = high conf, red = low conf) |
| `/frustum/trajectories` | MarkerArray | Path each tracked object has taken |
| `/frustum/velocity_arrows` | MarkerArray | Cyan arrows showing movement direction and speed |
| `/frustum/bev` | Image | Bird's eye view panel with confidence legend |
| `/painting/segmentation_overlay` | Image | YOLO mask overlaid on camera image |

### Confidence colour meaning (bounding boxes)

| Colour | Meaning | Threshold |
|---|---|---|
| Green | High confidence | ≥ 0.60 |
| Red | Low confidence | < 0.60 |

---

## Step 5 — View Validation Plots

After stopping the pipeline (Ctrl+C), the plots are saved inside the container at `/workspace/metrics/`. Open them from your host machine by navigating to the folder where you cloned the repo — the `/workspace` folder is mounted directly to it.

| File | What it shows |
|---|---|
| `confidence_over_time.png` | Confidence score per track across all frames |
| `track_lifetime.png` | Bar chart — how many frames each track stayed alive |
| `bev_trajectories.png` | Top-down (BEV) path of every track |
| `validation_summary.png` | Key validation metrics in one table |

The recorded BEV video is saved to `/workspace/video_output/tracking_output.mp4`.

---

## Alternative — Run Nodes Individually

If you want separate terminals to inspect each node's output independently:

**Terminal 1 — PaintingNode**
```bash
source /workspace/ros2_ws/install/setup.bash
ros2 run point_painting painting_node --ros-args -p calib_file:=/workspace/calib.txt
```

**Terminal 2 — FrustumNode**
```bash
source /workspace/ros2_ws/install/setup.bash
ros2 run frustum_detection frustum_node --ros-args -p calib_file:=/workspace/calib.txt
```

**Terminal 3 — Bag playback**
```bash
ros2 bag play /workspace/studentProject1 --clock --loop --rate 0.1
```

**Terminal 4 (optional) — MetricsLogger**
```bash
source /workspace/ros2_ws/install/setup.bash
ros2 run frustum_detection metrics_logger_node --ros-args -p output_dir:=/workspace/metrics
```

---

## Offline Isolation Test (No ROS needed)

To verify the pipeline on a single frame without running any nodes:

```bash
python3 /workspace/test_pipeline_isolation.py --seed 7
```

Output images are saved to `/workspace/isolation_output/`:

| File | What it shows |
|---|---|
| `01_raw_image.jpg` | Raw camera frame |
| `02_yolo_mask.jpg` | YOLO segmentation mask |
| `03_overlay.jpg` | YOLO mask blended on camera |
| `04_lidar_projected.jpg` | LiDAR points projected onto image |
| `05_painted_scores.jpg` | Per-point painting score heatmap |
| `06_detections.jpg` | FrustumDetector 3D boxes (camera + BEV) |
| `07_tracked.jpg` | Tracked objects with IDs (camera + BEV) |

If `07_tracked.jpg` shows correct boxes, the pipeline is fully working.

---

## KITTI Offline Evaluation (Quantitative Metrics)

> **Why KITTI?** The student dataset (`studentProject1`) has no ground truth annotations. KITTI provides ready-made labels for Cars, Pedestrians, and Cyclists — the standard benchmark for this type of pipeline.

### Download KITTI data

1. Go to [https://www.cvlibs.net/datasets/kitti/eval_object.php](https://www.cvlibs.net/datasets/kitti/eval_object.php)
2. Download: **Left color images**, **Velodyne point clouds**, **Camera calibration matrices**, **Training labels**
3. Extract to `/workspace/kitti/` inside the container

Expected structure:
```
/workspace/kitti/
    image_2/     000000.png  000001.png  ...
    velodyne/    000000.bin  000001.bin  ...
    label_2/     000000.txt  000001.txt  ...
    calib/       000000.txt  000001.txt  ...
```

### Run the evaluator

```bash
python3 /workspace/evaluate_kitti.py --kitti_dir /workspace/kitti --output_dir /workspace/metrics
```

Quick test on just 50 frames:
```bash
python3 /workspace/evaluate_kitti.py --kitti_dir /workspace/kitti --max_frames 50
```

### Output

Results are printed to the terminal and saved as `kitti_evaluation.png` in `/workspace/metrics/`:

| Metric | Description |
|---|---|
| **Precision** | Fraction of detections that matched a ground truth box (BEV IoU > 0.25) |
| **Recall** | Fraction of ground truth objects that were detected |
| **F1** | Harmonic mean of Precision and Recall |
| **BEV IoU** | Mean Bird's Eye View overlap between predicted and ground truth boxes |
| **MOTA** | Multi-Object Tracking Accuracy — combined FP, FN, and ID switches |
| **MOTP** | Multi-Object Tracking Precision — mean localisation error in metres |
