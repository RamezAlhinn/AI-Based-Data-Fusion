# AI-Based Data Fusion — Project Summary

**Author:** Ramez Alhinn  
**Date:** May 2026  

---

## What We Are Building

A real-time **Camera + LiDAR fusion pipeline** running on ROS 2 Humble inside a Docker container. The goal is to take raw sensor data from a vehicle and produce semantically-enriched 3D point clouds — each LiDAR point knows what object class it belongs to (car, pedestrian, cyclist). This feeds into a downstream 3D detector and tracker.

The algorithm is called **PointPainting** (Vora et al., 2020): project each 3D LiDAR point onto the camera image, read the semantic label at that pixel, and attach it to the point.

```
Camera Image ──► YOLO11n-seg ──► Per-pixel class labels ──┐
                                                           ▼
LiDAR Point Cloud ─────────────────────────► Project + Paint
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
| **Ramez Alhinn** | Overall architecture, ROS 2 node, painting logic, YOLO integration, segmentation overlay, isolation testing, calibration derivation, system integration |
| **arpitashil676** | LiDAR-to-image projection framework (`perception_framework` package) |
| **BelenNuñez** | Initial 2D segmentation module (DeepLab v3+, replaced by YOLO) |
| **carlosaterans-cmd** | Bag data extraction utility, initial YOLO segmentation script |

---

## What Ramez Implemented

### 1. The PointPainting ROS 2 Node — `painting_node.py`

The central piece that connects everything. It:

- Subscribes to **two sensor topics** simultaneously:
  - `/blackfly_s/cam0/image_rectified` — camera frames (1920×1200, bgr8)
  - `/velodyne/points_raw` — LiDAR point cloud (~64,000 points per scan)
- Uses a **latest-message cache** instead of `ApproximateTimeSynchronizer` — the LiDAR and camera were recorded with different clocks in the bag, so timestamp-based sync never fires
- On each incoming message: runs YOLO segmentation on the latest camera frame, projects LiDAR points onto the mask, and paints each point with its class ID
- Publishes **three output topics**:

| Topic | Type | Description |
|---|---|---|
| `/painting/debug` | String | Per-frame painted/skipped counts |
| `/painting/painted_cloud` | PointCloud2 | Full point cloud coloured by semantic class |
| `/painting/segmentation_overlay` | Image | Camera image with YOLO masks blended on top |
| `/painting/points_overlay` | Image | Camera image with projected LiDAR dots coloured by class |

**Run:**
```bash
ros2 run point_painting painting_node \
  --ros-args -p calib_file:=/workspace/calib.txt
```

Optional — custom YOLO model:
```bash
ros2 run point_painting painting_node \
  --ros-args \
  -p calib_file:=/workspace/calib.txt \
  -p checkpoint_path:=/workspace/yolo11s-seg.pt
```

---

### 2. YOLO Segmentation — `yolo_segmentation.py`

Built on top of colleagues' initial YOLO work (Code branch) but with class labels properly preserved.

- Their version: published a binary black/white mask — class information discarded
- This version: returns a full `(H, W)` array with native COCO class IDs per pixel
- Uses **YOLO11s-seg** — the small model from Ultralytics (~22 MB), chosen over nano because the dataset has dark/underexposed vehicles that nano misses
- Confidence threshold set to `0.15` to catch small/partially occluded objects
- Background initialised to `-1` (not `0`) — COCO class 0 is `person`, so zero-init caused everything to be labelled as person

**COCO class IDs used:**

| Class ID | Object | Colour in Foxglove |
|---|---|---|
| 0 | person | 🔴 Red |
| 1 | bicycle | 🔵 Blue |
| 2 | car | 🟢 Green |
| 3 | motorcycle | 🟠 Orange |
| 5 | bus | 🟡 Yellow |
| 7 | truck | 🟢 Dark green |

---

### 3. The Painting Logic — `painting_logic.py`

Pure algorithm, completely separated from ROS for unit testing.

- **`init_projector(calib_file)`** — loads calibration matrices once at startup
- **`paint_points(points_xyz, seg_image)`** — single projection pass (fixed double-projection bug):
  1. Transforms all N LiDAR points to camera frame using `Tr_velo_to_cam`
  2. Filters out points with negative depth (behind the camera)
  3. Projects to pixel coordinates using `cv2.projectPoints`
  4. Filters out points outside the image frame
  5. Samples `seg_image[v, u]` for each valid point
  6. Returns `class_ids` — one per input point, `-1` for misses

---

### 4. Segmentation Overlay & Points Overlay

Two image topics for visual verification of the pipeline:

- **`/painting/segmentation_overlay`** — YOLO masks blended on the camera image (60/40). Shows what the model detected. Throttled to every 5th frame to avoid flooding Foxglove's buffer.
- **`/painting/points_overlay`** — projected LiDAR dots drawn directly on the camera image, coloured by semantic class. Shows exactly which pixels each LiDAR point lands on — the clearest end-to-end verification.

---

### 5. Pipeline Isolation Test — `test_pipeline_isolation.py`

Standalone Python script (no ROS needed) that runs the full pipeline on one bag frame and saves 4 verification images:

```
isolation_output/01_raw_image.jpg          ← original camera frame
isolation_output/02_yolo_mask.jpg          ← YOLO class mask
isolation_output/03_overlay.jpg            ← mask blended on image
isolation_output/04_lidar_projected.jpg    ← LiDAR points on image coloured by class
```

Used to confirm the projection is correct before running the live node.

---

### 6. The Calibration File — `calib.txt`

Derived entirely from data inside the bag — no external calibration file was provided.

**Extrinsics** — from `/tf_static` topic in the bag:
```
translation : x = +0.692 m (camera is 69 cm in front of LiDAR)
              y =  0.000 m
              z = -0.180 m (camera is 18 cm below LiDAR)
rotation    : quaternion [x=-0.534, y=0.543, z=-0.464, w=0.452]
```

The KITTI format needs the inverse transform (camera→LiDAR inverted), confirmed by verifying a point 10m ahead projects correctly to pixel (910, 174).

**Intrinsics** — estimated from Blackfly S hardware specs (no `/camera_info` topic in bag):

| Parameter | Value |
|---|---|
| `fx = fy` | 2318.8 px |
| `cx` | 959.5 px (image centre) |
| `cy` | 599.5 px (image centre) |
| Implied HFOV | ~45° |

---

## Current Pipeline Status

```
DONE ✅                                  TODO ⬜
──────────────────────────────────       ──────────────────────────
ROS 2 devcontainer (Docker)              Merge feature branch → main
Bag playback + clock handling            PointPillars 3D detector
Camera + LiDAR latest-msg sync          AB3DMOT tracker
YOLO11n-seg segmentation w/ classes     Performance profiling
LiDAR → image projection (verified)     Camera intrinsics verification
PointPainting (single projection pass)    (checkerboard calibration)
Painted point cloud (/painted_cloud)
Segmentation overlay (/seg_overlay)
Points overlay (/points_overlay)
Pipeline isolation test
Foxglove visualisation (port 9090)
Unit tests (3 passing)
```

---

## Repository Structure

```
AI-Based-Data-Fusion/
├── calib.txt                              ← derived sensor calibration
├── QUICKSTART.md                          ← run instructions
├── test_pipeline_isolation.py             ← offline pipeline verification
├── studentProject1/                       ← ROS 2 bag (sensor data)
├── .devcontainer/
│   ├── Dockerfile                         ← ROS 2 Humble + YOLO + foxglove-bridge
│   └── devcontainer.json                  ← port 9090 forwarded
├── docs/
│   ├── Project_Summary.md                 ← this document
│   ├── PointPainting_Learning_Guide.md
│   └── Architecture_Proposal.md
└── ros2_ws/src/
    ├── perception_framework/              ← arpitashil676
    │   └── lidar_to_image_projection.py
    └── point_painting/                    ← Ramez (main package)
        ├── painting_node.py
        ├── painting_logic.py
        ├── rosbag_extractor.py
        ├── segmentation/
        │   └── yolo_segmentation.py       ← YOLO11n-seg with class labels
        └── test/
            └── test_painting_node.py
```
