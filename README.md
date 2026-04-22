# AI-Based Data Fusion

A ROS 2 Humble workspace for real-time multi-sensor perception. The project fuses **camera + LiDAR** data using the **PointPainting** algorithm to produce tracked, classified 3-D objects — forming the perception backbone of an autonomous/assisted-driving system.

---

## Pipeline Overview

The chosen algorithm is **PointPainting** (Vora et al., 2020): a sequential, memory-safe fusion approach suited for laptop-class hardware inside Docker.

```
Camera Frame ──► 2D Segmentation CNN ──► Per-Pixel Class Score Map ──┐
                                                                       ▼
LiDAR Scan ───────────────────────────────────────────────► Project + Paint Points
                                                                       │
                                                       [x,y,z,i] → [x,y,z,i, p_car, p_ped, p_cyc]
                                                                       │
                                                                       ▼
                                                            3D Detector (PointPillars)
                                                                       │
                                                                       ▼
                                                            AB3DMOT Tracker
                                                                       │
                                                                       ▼
                                                         ROS 2 Publisher → RViz
```

See [docs/Architecture_Proposal.md](docs/Architecture_Proposal.md) for the full design rationale and comparison against AVOD.

---

## Repository Structure

```
AI-Based-Data-Fusion/
├── .devcontainer/
│   ├── Dockerfile              # ROS 2 Humble + Python deps (PyTorch, ultralytics, open3d)
│   └── devcontainer.json       # VS Code Dev Container config
├── docs/
│   ├── Architecture_Proposal.md
│   └── *.pdf                   # Reference papers
├── ros2_ws/
│   └── src/
│       └── point_painting/     # PointPainting ROS 2 package
│           ├── point_painting/
│           │   ├── painting_logic.py   # Pure fusion logic (no ROS — testable standalone)
│           │   └── painting_node.py    # ROS 2 node: subscribers, sync, publisher
│           ├── test/
│           │   └── test_painting_node.py  # Standalone tests, no ROS runtime needed
│           ├── package.xml
│           └── setup.py
└── README.md
```

---

## Prerequisites

We use **Docker** and **VS Code Dev Containers** so everyone gets the exact same ROS 2 environment — no native ROS install required.

| Tool | Notes |
|---|---|
| **Docker Desktop** | Latest version. Windows users: enable the WSL 2 backend in settings. |
| **Visual Studio Code** | Latest stable release. |
| **Dev Containers extension** | `ms-vscode-remote.remote-containers` |

The container image includes:
- ROS 2 Humble Desktop (Ubuntu 22.04)
- `ros-humble-message-filters`, `ros-humble-vision-msgs`, `ros-humble-rviz2`, `ros-humble-tf2-ros`
- `cv_bridge`, `sensor_msgs_py`
- PyTorch (CPU), `ultralytics` (YOLOv8/v11), `open3d`, `numpy`, `scipy`

---

## Getting Started

### 1. Clone

```bash
git clone https://github.com/RamezAlhinn/AI-Based-Data-Fusion.git
cd AI-Based-Data-Fusion
```

### 2. Open in VS Code

```bash
code .
```

### 3. Reopen in Dev Container

VS Code will detect `.devcontainer/` and show a popup — click **Reopen in Container**.
Alternatively: click the `><` icon in the bottom-left corner → **Dev Containers: Reopen in Container**.

> The first build downloads the ROS 2 image and installs PyTorch — allow 5–15 minutes. Subsequent opens are instant.

### 4. Build the workspace

The container auto-runs `colcon build` on start. To rebuild manually:

```bash
cd /workspace/ros2_ws
colcon build --packages-select point_painting
source install/setup.bash
```

Shortcut alias available inside the container: `cw` (build + source).

---

## Running the PointPainting Node

The node subscribes to your bag file's two key topics and syncs them:

| Topic | Message Type |
|---|---|
| `/blackfly_s/cam0/image_rectified` | `sensor_msgs/msg/Image` |
| `/velodyne/points_raw` | `sensor_msgs/msg/PointCloud2` |

```bash
# Terminal 1 — start the node
ros2 run point_painting painting_node

# Terminal 2 — play a bag file
ros2 bag play <path-to-your-bag>

# Monitor debug output
ros2 topic echo /painting/debug
```

The node publishes a `std_msgs/String` on `/painting/debug` with per-frame paint counts:

```
frame=1 painted=12543 skipped=0
```

Every 50 frames it also logs painted vs. skipped counts to the ROS logger.

---

## Testing (no ROS runtime needed)

The painting logic is isolated in `painting_logic.py` so tests run on any machine:

```bash
python3 ros2_ws/src/point_painting/test/test_painting_node.py
```

Expected output:

```
Running test_stub_projection_returns_valid_pixel ...
  project_point_to_pixel(1.0, 2.0, 5.0) -> (100, 100)
  PASSED

Running test_all_points_get_class_id ...
  painted=10, skipped=0
  class_ids=[7, 7, 7, 7, 7, 7, 7, 7, 7, 7]
  PASSED

Running test_out_of_bounds_points_are_skipped ...
  painted=0, skipped=5 (all out of 50x50 bounds as expected)
  PASSED

ALL TESTS PASSED
```

---

## What's Next

| Step | Status |
|---|---|
| Package skeleton + sync node | Done |
| Standalone test harness | Done |
| Real camera-to-LiDAR projection (calibration matrix) | TODO |
| 2D segmentation CNN (YOLOv8 semantic head) | TODO |
| Painted point cloud publisher (`sensor_msgs/PointCloud2`) | TODO |
| 3D detector integration (PointPillars) | TODO |
| AB3DMOT tracker | TODO |
| RViz visualization | TODO |

---

## Useful Aliases (inside container)

| Alias | Expands to |
|---|---|
| `cw` | `cd /workspace/ros2_ws && colcon build --symlink-install && source install/setup.bash` |
| `cs` | `cd /workspace/ros2_ws/src` |
