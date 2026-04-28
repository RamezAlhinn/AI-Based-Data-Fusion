# AI-Based Data Fusion — Project Summary

**Presentation document for team handover**  
**Author:** Ramez Alhinn  
**Date:** April 2026  

---

## What We Are Building

A real-time **Camera + LiDAR fusion pipeline** running on ROS 2 Humble inside a Docker container. The goal is to take raw sensor data from a vehicle and produce semantically-enriched 3D point clouds — each LiDAR point knows what object class it belongs to (car, pedestrian, cyclist). This feeds into a downstream 3D detector and tracker.

The algorithm is called **PointPainting** (Vora et al., 2020): project each 3D LiDAR point onto the camera image, read the semantic label at that pixel, and attach it to the point.

```
Camera Image ──► Segmentation CNN ──► Per-pixel class labels ──┐
                                                                ▼
LiDAR Point Cloud ──────────────────────────────► Project + Paint
                                                                │
                                              [x,y,z] → [x,y,z, class_id]
                                                                │
                                                                ▼
                                                    3D Detector (next step)
```

---

## Team Contributions

| Member | What they built |
|---|---|
| **Ramez Alhinn** | Overall architecture, ROS 2 node, painting logic, system integration, calibration derivation, tests |
| **arpitashil676** | LiDAR-to-image projection framework (`perception_framework` package) |
| **BelenNuñez** | 2D semantic segmentation module (DeepLab v3+) |
| **carlosaterans-cmd** | Bag data extraction utility |

---

## What Ramez Implemented

### 1. The PointPainting ROS 2 Node — `painting_node.py`

The central piece that connects everything. It:

- Subscribes to **two sensor topics** simultaneously:
  - `/blackfly_s/cam0/image_rectified` — camera frames (1920×1200, bgr8)
  - `/velodyne/points_raw` — LiDAR point cloud (~95,000 points per scan)
- Uses **`ApproximateTimeSynchronizer`** to pair camera frames with LiDAR scans that are within 100 ms of each other — critical because the camera runs at ~10 Hz and the LiDAR at ~10 Hz and their timestamps drift
- On each synced pair: decodes both messages into numpy arrays, runs segmentation, then paints
- Accepts **three ROS 2 parameters** at launch — no hardcoded paths anywhere:

| Parameter | What it points to |
|---|---|
| `calib_file` | `calib.txt` — sensor calibration matrices |
| `deeplab_repo_path` | Root of the `pytorch-deeplab-xception` clone |
| `checkpoint_path` | `deeplab-resnet.pth.tar` weights file |

- **Graceful degraded mode** — runs without any parameter: logs a warning, skips painting or segmentation, does not crash. Useful during development when only one component is ready.

**Run — projection only (no DeepLab model needed):**
```bash
ros2 run point_painting painting_node \
  --ros-args -p calib_file:=/workspace/calib.txt
```

**Run — full pipeline with real semantic labels:**
```bash
ros2 run point_painting painting_node --ros-args \
  -p calib_file:=/workspace/calib.txt \
  -p deeplab_repo_path:=/path/to/pytorch-deeplab-xception \
  -p checkpoint_path:=/path/to/deeplab-resnet.pth.tar
```

**Monitor:**
```bash
ros2 topic echo /painting/debug
# → data: 'frame=42 painted=8243 skipped=4901'
```

---

### 2. The Painting Logic — `painting_logic.py`

The pure algorithm, completely separated from ROS so it can be unit-tested on any machine without a ROS runtime.

- **`init_projector(calib_file)`** — loads the calibration matrices once at startup
- **`paint_points(points_xyz, seg_image)`** — vectorised over the full point cloud:
  1. Transforms all N LiDAR points to camera frame using `Tr_velo_to_cam`
  2. Filters out points with negative depth (behind the camera)
  3. Projects survivors to pixel coordinates using `cv2.projectPoints`
  4. Filters out points outside the image frame
  5. Samples `seg_image[v, u]` for each valid point
  6. Returns a `class_ids` list — one entry per input point, `-1` for points that missed the camera

The entire projection runs in a single NumPy/OpenCV call — no Python loop over 95k points. This is what makes it fast enough to run at sensor rate.

---

### 3. Integration of Teammates' Work

**From arpitashil676 — `KittiLidarToImageProjector`:**  
`painting_logic.py` imports and uses this class directly. It handles the full KITTI calibration chain:

```
LiDAR point [x, y, z]
       │
       ▼  Tr_velo_to_cam (4×4)  — rigid transform: LiDAR frame → camera frame
       │
       ▼  R0_rect (4×4)         — rectification (identity for monocular)
       │
       ▼  cv2.projectPoints     — focal length + principal point → pixel [u, v]
       │
       ▼  bounds check          — discard if outside 1920×1200
```

Before integration, the projection was a stub returning `(100, 100)` for every point. After integration, each point gets its real pixel coordinate.

**From BelenNuñez — `deeplab_segmentation.py`:**  
Refactored from a standalone script with hardcoded Windows paths into a proper importable module with three clean functions: `load_model()`, `segment_image()`, `decode_segmap()`. Moved from `.vscode/` into the ROS 2 package at `point_painting/segmentation/`. Now wired directly into the painting node — `load_model()` is called once at startup, and `segment_image()` is called on every synced camera frame before painting. Both the DeepLab repo path and checkpoint path are passed as ROS 2 parameters so no path is hardcoded.

**From carlosaterans-cmd — `rosbag_extractor.py`:**  
Moved from the repository root into the package. Used to pull raw camera frames and LiDAR scans out of the bag for offline testing.

---

### 4. The Calibration File — `calib.txt`

No external calibration file was provided with the dataset. It was **derived entirely from data already inside the bag**.

#### Where the data came from

**Extrinsics (rotation + translation between sensors):**  
The bag's `/tf_static` topic contains the static transform `velodyne → cam0`:

```
translation : x = +0.692 m (camera is 69 cm in front of LiDAR)
              y =  0.000 m
              z = -0.180 m (camera is 18 cm below LiDAR)
rotation    : quaternion [x=-0.534, y=0.543, z=-0.464, w=0.452]
```

**One non-obvious step:** ROS tf stores the *passive* rotation (how the child frame is oriented relative to the parent). KITTI needs the *active* rotation (how to rotate points from LiDAR frame into camera frame). These are transposes of each other. Using the quaternion directly put all points behind the camera. The fix: transpose the rotation matrix and re-express the translation in the camera frame (`t_cam = -R.T @ t_velo`).

**Intrinsics (focal length, principal point):**  
No `/camera_info` topic exists in the bag. Values estimated from hardware:

| Parameter | Value | How derived |
|---|---|---|
| Camera | Blackfly S BFS-U3-51S5C | From topic name / known platform |
| Sensor | Sony IMX250, 2/3 inch | Datasheet |
| Pixel pitch | 3.45 µm | Datasheet |
| Lens | 8 mm (estimated) | Typical for automotive setup |
| `fx = fy` | 8.0 mm ÷ 0.00345 mm/px = **2318.8 px** | Calculated |
| `cx` | 959.5 px | Centre of rectified 1920-wide image |
| `cy` | 599.5 px | Centre of rectified 1200-tall image |
| Implied HFOV | 45° | Consistent with 8 mm lens on 2/3" sensor |

**Validation:**  
A LiDAR point placed 10 m straight ahead of the vehicle projects to pixel **(910, 174)** on the 1920×1200 image — slightly left of centre (the camera is not perfectly boresighted with the LiDAR) and in the upper half of the frame (the camera sits below the LiDAR and angles upward). This is physically consistent with the sensor geometry.

> **Note:** `fx` is the parameter most sensitive to the actual lens fitted. If points appear consistently shifted horizontally, the lens may be 6 mm (fx ≈ 1739, HFOV ≈ 58°) or 12 mm (fx ≈ 3478, HFOV ≈ 31°). Proper validation requires overlaying projected LiDAR points on a saved camera frame.

---

## The Dataset — `studentProject1/` Bag

| Property | Value |
|---|---|
| Duration | ~16 seconds |
| Camera frames | 162 |
| LiDAR scans | 164 |
| Points per scan | ~95,000 |
| Image size | 1920 × 1200 px |
| Camera topic | `/blackfly_s/cam0/image_rectified` |
| LiDAR topic | `/velodyne/points_raw` |
| Calibration source | `/tf_static` (embedded in bag) |

---

## Current Pipeline Status

```
  DONE ✅                              TODO 🔲
  ─────────────────────────────        ──────────────────────────────
  ROS 2 node (painting_node.py)        Painted cloud publisher
  Time synchronisation                   (/painted/points PointCloud2)
  KITTI projection (real calib)        PointPillars 3D detector
  Painting logic (vectorised)          AB3DMOT tracker
  Calibration derived from bag         RViz2 MarkerArray visualisation
  DeepLab wired into live node         Performance profiling
    via ROS 2 parameters
  Degraded mode (no model → still runs)
  DeepLab segmentation module
  Rosbag extractor
  Unit tests (3 passing)
  Dev Container (Docker)
```

The pipeline runs end-to-end today. Play the bag, watch `painted=~8000` per frame on `/painting/debug`, and visualise the raw point cloud and camera feed in RViz2. With the DeepLab checkpoint provided, each painted point carries a real semantic class index (0=background, 7=car, 15=person). The remaining work is packaging those class-enriched points into a published `PointCloud2` and feeding them into a 3D detector.

---

## Repository Structure

```
AI-Based-Data-Fusion/
├── calib.txt                          ← derived sensor calibration
├── QUICKSTART.md                      ← 4-terminal run instructions
├── studentProject1/                   ← ROS 2 bag (sensor data)
├── docs/
│   ├── Project_Summary.md             ← this document
│   ├── PointPainting_Learning_Guide.md
│   └── Architecture_Proposal.md
└── ros2_ws/src/
    ├── perception_framework/          ← arpitashil676
    │   └── lidar_to_image_projection.py
    └── point_painting/                ← Ramez (main package)
        ├── painting_node.py
        ├── painting_logic.py
        ├── rosbag_extractor.py
        ├── segmentation/
        │   └── deeplab_segmentation.py   ← BelenNuñez
        └── test/
            └── test_painting_node.py
```
