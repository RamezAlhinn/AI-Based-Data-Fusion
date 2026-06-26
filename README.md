# AI-Based Data Fusion

A ROS 2 Humble workspace for real-time multi-sensor perception: this project fuses camera, LiDAR, and radar data to produce tracked, classified 3-D objects, forming the perception backbone of an autonomous or assisted-driving system. The pipeline algorithm is still being finalised; this repository provides a clean, buildable foundation that the team can extend incrementally.

---

## 🛠️ Prerequisites

We use **Docker** and **VS Code Dev Containers** to ensure everyone has the exact same ROS 2 environment, regardless of their operating system. You do **not** need to install ROS 2 natively on your machine!

| Tool | Notes |
|---|---|
| **Docker Desktop** | Latest version. *Windows users: ensure the WSL 2 backend is enabled in settings.* |
| **Visual Studio Code** | Latest stable release. |
| **Dev Containers Extension** | Search for `ms-vscode-remote.remote-containers` in VS Code extensions. |

---

## 🚀 Getting Started

### 1. Clone the repository
```bash
git clone https://github.com/RamezAlhinn/AI-Based-Data-Fusion.git
cd AI-Based-Data-Fusion
```

### 2. Open in VS Code
```bash
code .
```

### 3. Start the Dev Container
When you open the folder, VS Code will automatically detect the `.devcontainer` configuration.
- A popup will appear in the bottom right saying *"Folder contains a Dev Container configuration file"*. Click **Reopen in Container**.
- **Alternative:** Click the green `><` icon in the absolute bottom-left corner of VS Code and select **"Dev Containers: Reopen in Container"**.

> ⏳ **Note:** The very first time you do this, Docker will download the official ROS 2 image and build the environment. This takes roughly **5–10 minutes** depending on your internet connection. After this initial setup, opening the project will be nearly instant!

---

## 📂 Repository Structure

```
AI-Based-Data-Fusion/
├── .devcontainer/                  # Docker + VS Code Dev Container config
├── ros2_ws/src/
│   ├── point_painting/             # YOLO segmentation + PointPainting node
│   ├── frustum_detection/          # Frustum 3D detection + tracking node
│   │   └── launch/
│   │       └── pipeline.launch.py  # Single launch file for full pipeline
│   ├── perception_msgs/            # Custom ROS 2 message definitions
│   └── perception_framework/       # Shared calibration utilities
├── frustum_detection/              # Core detection library (no ROS dependency)
├── evaluate_kitti.py               # Offline KITTI benchmark evaluation script
├── test_pipeline_isolation.py      # Single-frame offline pipeline test
├── QUICKSTART.md                   # Step-by-step run guide
└── README.md
```

---

## 🤖 ROS 2 Workspace

The `ros2_ws/` workspace contains four packages. See the **QUICKSTART.md** for full run instructions.

### Building the workspace

```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 🧠 Pipeline Architecture

```
Camera ──────────────────────────────────────────────────────┐
                                                             ▼
LiDAR ──► [PaintingNode] ──► scored_cloud ──► [FrustumNode] ──► 3D Boxes + Tracks
               │                                    │
               ▼                                    ▼
          YOLO Segmentation               AB3DMOT Kalman Tracker
```

| Node | Package | Description |
|---|---|---|
| `painting_node` | `point_painting` | Runs YOLO11m-seg on camera frames, projects LiDAR onto image, labels each point with a semantic class score |
| `frustum_node` | `frustum_detection` | Extracts 3D point frustums per YOLO box, clusters with DBSCAN, fits bounding boxes, tracks with AB3DMOT Kalman filter |
| `metrics_logger_node` | `frustum_detection` | Subscribes to `/frustum/objects` and generates validation plots on shutdown |

---

## 📊 Validation Results

Validation plots are generated automatically when the pipeline is stopped (Ctrl+C) and saved to `/workspace/metrics/`.

| Plot | Description |
|---|---|
| `confidence_over_time.png` | Detection confidence score per track across frames — shows detection stability |
| `track_lifetime.png` | How many frames each track stayed alive — shows tracking consistency |
| `bev_trajectories.png` | Top-down path of every tracked object — shows spatial coverage |
| `validation_summary.png` | Key metrics table: total frames, unique tracks, ID switches, mean confidence and speed per track |

### Key metrics

| Metric | Description |
|---|---|
| **Confidence score** | YOLO detection certainty — above 0.60 = high (green box), below = low (red box) |
| **Track lifetime** | Number of frames a track was continuously maintained by the Kalman filter |
| **ID switches** | How many times a tracked object was lost and re-assigned a new ID (lower is better) |
| **Mean speed** | Average velocity estimated by the Kalman filter per track (m/s) |

---

## 📐 KITTI Benchmark Evaluation

For quantitative detection and tracking metrics (Precision, Recall, F1, BEV IoU, MOTA, MOTP), the pipeline can be evaluated against the **KITTI Object Detection** dataset which provides ground truth 3D labels.

> **Why KITTI?** The student dataset (`studentProject1`) has no ground truth annotations. KITTI is the standard autonomous driving benchmark used in research and provides ready-made labels for Cars, Pedestrians, and Cyclists.

### Download KITTI data

1. Go to [https://www.cvlibs.net/datasets/kitti/eval_object.php](https://www.cvlibs.net/datasets/kitti/eval_object.php)
2. Download: **Left color images**, **Velodyne point clouds**, **Camera calibration**, **Training labels**
3. Extract to `/workspace/kitti/` inside the container

Expected structure:
```
/workspace/kitti/
    image_2/     000000.png  000001.png  ...
    velodyne/    000000.bin  000001.bin  ...
    label_2/     000000.txt  000001.txt  ...
    calib/       000000.txt  000001.txt  ...
```

### Run evaluation

```bash
python3 /workspace/evaluate_kitti.py --kitti_dir /workspace/kitti --output_dir /workspace/metrics
```

Quick test on first 50 frames:
```bash
python3 /workspace/evaluate_kitti.py --kitti_dir /workspace/kitti --max_frames 50
```

### Output

Results are printed to terminal and saved as `kitti_evaluation.png` in `/workspace/metrics/`:

| Metric | Description |
|---|---|
| **Precision** | Fraction of detections that matched a ground truth box (BEV IoU > 0.25) |
| **Recall** | Fraction of ground truth objects that were detected |
| **F1** | Harmonic mean of Precision and Recall |
| **BEV IoU** | Mean Bird's Eye View overlap between predicted and ground truth boxes |
| **MOTA** | Multi-Object Tracking Accuracy — combined measure of FP, FN, and ID switches |
| **MOTP** | Multi-Object Tracking Precision — mean localisation error in metres |

> **Note:** KITTI uses different sensors (HDL-64E LiDAR, urban roads) vs the student dataset (VLP-16, campus). Metrics reflect benchmark performance — qualitative results on the student dataset are shown via RViz2 and the live metrics plots.

---

## 🗂️ Repository Structure

```
AI-Based-Data-Fusion/
├── .devcontainer/                  # Docker + VS Code Dev Container config
├── ros2_ws/src/
│   ├── point_painting/             # YOLO segmentation + PointPainting node
│   ├── frustum_detection/          # Frustum 3D detection + tracking node
│   │   └── launch/
│   │       └── pipeline.launch.py  # Single launch file for full pipeline
│   ├── perception_msgs/            # Custom ROS 2 message definitions
│   └── perception_framework/       # Shared calibration utilities
├── frustum_detection/              # Core detection library (no ROS dependency)
├── evaluate_kitti.py               # Offline KITTI benchmark evaluation script
├── test_pipeline_isolation.py      # Single-frame offline pipeline test
├── QUICKSTART.md                   # Step-by-step run guide
└── README.md
```