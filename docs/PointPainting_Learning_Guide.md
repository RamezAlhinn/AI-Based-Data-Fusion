# PointPainting: System Engineering Learning Guide

**Project:** AI-Based Data Fusion — Camera + LiDAR Perception  
**Platform:** ROS 2 Humble · Docker Dev Container · Velodyne LiDAR · Blackfly S Camera  
**Updated:** April 2026

### Team

| Name | Role / Contribution |
|------|---------------------|
| **Ramez Alhinn** |  ROS 2 node architecture, painting logic, system integration, tests |
| **arpitashil676** | LiDAR-to-image projection framework (`perception_framework` package) |
| **BelenNuñez** | 2D semantic segmentation (DeepLab v3+ module) |
| **carlosaterans-cmd** | Rosbag data extraction utility |

---

## Table of Contents

1. [The Big Picture — Why Fuse Camera and LiDAR?](#1-the-big-picture)
2. [Sensor Primer — What Each Sensor Gives You](#2-sensor-primer)
3. [PointPainting Algorithm — The Core Idea](#3-pointpainting-algorithm)
4. [System Architecture — End-to-End Data Flow](#4-system-architecture)
5. [ROS 2 Communication Model](#5-ros-2-communication-model)
6. [Your Modules — How They Work and How They Connect](#6-your-modules)
7. [The Calibration Math — How Projection Works](#7-the-calibration-math)
8. [Dev Container — Your Execution Environment](#8-dev-container)
9. [Visualization with RViz2](#9-visualization-with-rviz2)
10. [What's Left — Remaining Work](#10-whats-left)
11. [Quick Reference](#11-quick-reference)

---

## 1. The Big Picture

### Why Not Just Use Camera? Why Not Just Use LiDAR?

```
CAMERA ALONE:
  ┌─────────────────────────────────────────────────┐
  │  RGB image: rich color, texture, semantics      │
  │  ✅ Knows WHAT things are (car, pedestrian)     │
  │  ❌ Cannot measure depth precisely              │
  │  ❌ Fails in low light, glare, fog              │
  └─────────────────────────────────────────────────┘

LIDAR ALONE:
  ┌─────────────────────────────────────────────────┐
  │  Point cloud: precise 3D geometry               │
  │  ✅ Knows WHERE things are (x, y, z, range)     │
  │  ✅ Works at night and in some adverse weather  │
  │  ❌ Cannot tell a person from a mailbox         │
  │  ❌ Sparse — far objects have very few points   │
  └─────────────────────────────────────────────────┘

FUSED (PointPainting):
  ┌─────────────────────────────────────────────────┐
  │  ✅ Precise 3D location from LiDAR              │
  │  ✅ Semantic labels (class) from camera         │
  │  ✅ Each 3D point gets a class probability      │
  │  → Better 3D object detection than either alone │
  └─────────────────────────────────────────────────┘
```

### The Fundamental Insight

A 3D LiDAR point is just a location in space: `(x, y, z)`. Alone it has no semantic meaning — you cannot tell if it belongs to a car, tree, or road. But if you **project** that point onto the camera image, you can look up what the neural network thinks that pixel represents. You then **paint** the 3D point with that semantic label. Now your point cloud entries are enriched: `(x, y, z, class_car, class_person, class_cyclist)`.

This is PointPainting in one sentence:
> **Project LiDAR points onto the camera image, sample semantic scores, attach them to the 3D points, then feed the enriched cloud to a 3D detector.**

---

## 2. Sensor Primer

### 2.1 Camera — Blackfly S

Your camera produces **rectified images** on topic `/blackfly_s/cam0/image_rectified`.

```
Physical World
     │
     │  photons enter lens
     ▼
┌─────────────────────────────────────────┐
│              LENS                       │
│  Bends light to focus on sensor         │
│  Introduces distortion (barrel/pincush) │
└─────────────────────────────────────────┘
     │
     ▼
┌─────────────────────────────────────────┐
│           IMAGE SENSOR (CMOS)           │
│  Converts photons → digital values      │
│  Output: raw image with distortion      │
└─────────────────────────────────────────┘
     │
     │  rectification (pre-applied to your topic)
     ▼
┌─────────────────────────────────────────┐
│         RECTIFIED IMAGE                 │
│  Distortion removed by calibration      │
│  Straight lines in world = straight px  │
│  Safe to apply projection equations     │
└─────────────────────────────────────────┘
     │
     │  ROS 2 message
     ▼
   sensor_msgs/msg/Image
   Topic: /blackfly_s/cam0/image_rectified
```

**Key camera model parameters:**

| Symbol | Name | What it means |
|--------|------|---------------|
| `fx`   | Focal length X | Pixels per meter at 1m distance (horizontal) |
| `fy`   | Focal length Y | Same but vertical |
| `cx`   | Principal point X | Pixel coordinate of the optical axis center |
| `cy`   | Principal point Y | Same but vertical |

These form the **intrinsic matrix K** (embedded as the top-left 3×3 of P2 in KITTI calibration files):
```
K = | fx   0   cx |
    |  0  fy   cy |
    |  0   0    1 |
```

### 2.2 LiDAR — Velodyne

Your LiDAR produces **raw point clouds** on topic `/velodyne/points_raw`.

```
Rotating laser head (360° horizontal sweep)
     │
     │  pulses laser beams outward
     ▼
┌─────────────────────────────────────────┐
│         LASER EMITTER/RECEIVER          │
│  Sends pulse, measures return time      │
│  distance = (speed_of_light × time) / 2│
│  Velodyne: 16/32/64 vertical beams      │
└─────────────────────────────────────────┘
     │
     │  azimuth angle + elevation angle + range
     │  → converted to Cartesian (x, y, z, intensity)
     │
     │  ROS 2 message
     ▼
   sensor_msgs/msg/PointCloud2
   Topic: /velodyne/points_raw
   Fields: x, y, z, intensity, ring, time
   Typical: ~25,000–130,000 points per scan @ 10 Hz
```

**LiDAR coordinate frame:**
```
        Z (up)
        │
        │
        └──────── X (forward, toward front of vehicle)
       /
      /
     Y (left)
```

---

## 3. PointPainting Algorithm

### 3.1 The Original Paper (Vora et al., 2020)

PointPainting is a **sequential fusion** method — camera runs first, then its output enriches the LiDAR data before the 3D detector sees it. This is in contrast to **parallel fusion** (like AVOD) which runs both streams simultaneously and merges feature maps.

### 3.2 Step-by-Step Algorithm

```
┌────────────────────────────────────────────────────────────────┐
│                    POINTPAINTING PIPELINE                      │
├────────────────────────────────────────────────────────────────┤
│                                                                │
│  STEP 1: 2D Semantic Segmentation          [BelenNuñez]        │
│  ┌──────────────────────────────────────────────────┐         │
│  │  Camera Image (H × W × 3)                        │         │
│  │         │                                        │         │
│  │         ▼                                        │         │
│  │  DeepLab v3+  (deeplab_segmentation.py)          │         │
│  │         │                                        │         │
│  │         ▼                                        │         │
│  │  Segmentation Map (H × W)                        │         │
│  │  Each pixel holds one integer class index:       │         │
│  │    0=background, 1=aeroplane, 2=bicycle,         │         │
│  │    7=car, 15=person, ...  (PASCAL VOC 21 classes)│         │
│  └──────────────────────────────────────────────────┘         │
│                           │                                    │
│                           │ class label per pixel              │
│                           ▼                                    │
│  STEP 2: Project LiDAR Points onto Image  [arpitashil676]      │
│  ┌──────────────────────────────────────────────────┐         │
│  │  LiDAR Point Cloud (N × 3) [x, y, z]            │         │
│  │         │                                        │         │
│  │         │  KittiLidarToImageProjector            │         │
│  │         │  (lidar_to_image_projection.py)        │         │
│  │         │                                        │         │
│  │         │  Transform to camera frame:            │         │
│  │         │  p_cam = R0_rect · Tr_velo_to_cam · p  │         │
│  │         │                                        │         │
│  │         │  Project to pixel using P2:            │         │
│  │         │  [u, v] = P2 · p_cam / depth           │         │
│  │         │                                        │         │
│  │         │  Discard if: depth ≤ 0 (behind cam)   │         │
│  │         │              or pixel outside image    │         │
│  └──────────────────────────────────────────────────┘         │
│                           │                                    │
│                           │ (u, v) for each valid point        │
│                           ▼                                    │
│  STEP 3: Paint Points with Semantic Labels  [Ramez Alhinn]     │
│  ┌──────────────────────────────────────────────────┐         │
│  │  paint_points()  in  painting_logic.py           │         │
│  │                                                  │         │
│  │  For each in-bounds point:                       │         │
│  │    class_id = seg_map[v, u]                      │         │
│  │  For out-of-bounds points:                       │         │
│  │    class_id = -1  (not visible in camera)        │         │
│  └──────────────────────────────────────────────────┘         │
│                           │                                    │
│                           │ class_ids per point                │
│                           ▼                                    │
│  STEP 4: 3D Object Detection  [TODO]                           │
│  ┌──────────────────────────────────────────────────┐         │
│  │  PointPillars                                    │         │
│  │  Input: enriched point cloud [x,y,z, class_id]  │         │
│  │  Output: 3D bounding boxes + class labels        │         │
│  └──────────────────────────────────────────────────┘         │
│                           │                                    │
│                           ▼                                    │
│  STEP 5: Multi-Object Tracking  [TODO]                         │
│  ┌──────────────────────────────────────────────────┐         │
│  │  AB3DMOT (Kalman Filter + Hungarian Algorithm)   │         │
│  │  Input: detections per frame                     │         │
│  │  Output: tracked objects with IDs across frames  │         │
│  └──────────────────────────────────────────────────┘         │
│                           │                                    │
│                           ▼                                    │
│  STEP 6: Visualization (RViz2)  [TODO]                         │
│  ┌──────────────────────────────────────────────────┐         │
│  │  PointCloud2: painted colored cloud              │         │
│  │  MarkerArray: 3D boxes with track IDs            │         │
│  └──────────────────────────────────────────────────┘         │
└────────────────────────────────────────────────────────────────┘
```

### 3.3 Why PointPainting Over AVOD (Your Architecture Decision)

Your `docs/Architecture_Proposal.md` evaluated two approaches and chose PointPainting:

```
┌─────────────────────────────────────────────────────────────────┐
│                    PIPELINE COMPARISON                          │
├──────────────────┬──────────────────┬──────────────────────────┤
│ Attribute        │ PointPainting    │ AVOD                     │
├──────────────────┼──────────────────┼──────────────────────────┤
│ Fusion type      │ Sequential       │ Parallel                 │
│ Camera use       │ Offline seg pass │ ResNet18 feature map     │
│ LiDAR use        │ Enriched cloud   │ BEV feature map          │
│ Feature maps     │ 1 (LiDAR only)   │ 2 simultaneously         │
│ OOM Risk         │ Very Low ✅       │ Moderate-High ❌         │
│ Complexity       │ Low ✅            │ High ❌                  │
│ Target hardware  │ Laptop ✅         │ GPU workstation ❌       │
└──────────────────┴──────────────────┴──────────────────────────┘

Decision: PointPainting chosen for laptop-class, CPU-only execution.
```

---

## 4. System Architecture — End-to-End Data Flow

### 4.1 Full System Block Diagram (Current + Planned)

```
┌─────────────────────────────────────────────────────────────────┐
│                         DATA SOURCES                            │
│                                                                 │
│   ┌──────────────────┐          ┌─────────────────────────┐     │
│   │  Blackfly S Cam  │          │    Velodyne LiDAR        │     │
│   │  (or bag file)   │          │    (or bag file)         │     │
│   └────────┬─────────┘          └──────────┬──────────────┘     │
│            │ sensor_msgs/Image              │ sensor_msgs/        │
│            │ /blackfly_s/cam0/              │ PointCloud2         │
│            │ image_rectified                │ /velodyne/          │
│            │                               │ points_raw          │
└────────────┼───────────────────────────────┼─────────────────────┘
             │                               │
             ▼                               ▼
┌────────────────────────────────────────────────────────────────┐
│    PAINTING NODE  (painting_node.py)  [Ramez Alhinn]           │
│                                                                │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  RAMEZ — ROS plumbing & orchestration                   │   │
│  │                                                         │   │
│  │  1. ApproximateTimeSynchronizer (slop=0.1s)             │   │
│  │     waits for image + cloud within 100ms of each other  │   │
│  │                                                         │   │
│  │  2. CvBridge → raw BGR numpy image                      │   │
│  │     read_points → (N×3) float32 xyz array               │   │
│  └───────────────────────┬─────────────────────────────────┘   │
│                          │                                     │
│                          ▼                                     │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  BELEN — segment_image(model, image)                    │   │
│  │  Camera frame → (H×W) grid of class indices             │   │
│  │  0=background  7=car  15=person  ...                    │   │
│  └───────────────────────┬─────────────────────────────────┘   │
│                          │ seg_image (H×W)                     │
│                          ▼                                     │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  ARPITA — project_lidar_to_image(xyz, image_shape)      │   │
│  │  Each 3D point → pixel [u, v] via KITTI calib chain     │   │
│  │  Drops points behind camera or outside frame            │   │
│  └───────────────────────┬─────────────────────────────────┘   │
│                          │ pixel coords per valid point        │
│                          ▼                                     │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │  RAMEZ — the painting step  (painting_logic.py)         │   │
│  │                                                         │   │
│  │  class_ids[i] = seg_image[v, u]                         │   │
│  │                                                         │   │
│  │  ← this is the fusion: a 3D LiDAR point gets its        │   │
│  │    semantic class for the first time here               │   │
│  └───────────────────────┬─────────────────────────────────┘   │
│                          │                                     │
│  5. Publish /painting/debug  "frame=42 painted=8500 skipped=1200"│
└────────────────────────────────────────────────────────────────┘
             │
             │  [Not yet implemented]
             ▼
┌────────────────────────────────────────────────────────────────┐
│                    DOWNSTREAM PIPELINE                         │
│                                                                │
│  /painted/points  (PointCloud2 with semantic class_ids)        │
│         │                                                      │
│         ▼                                                      │
│  ┌──────────────────┐    ┌──────────────────────────────┐      │
│  │   PointPillars   │───▶│        AB3DMOT Tracker        │      │
│  │  3D Detector     │    │  Kalman + Hungarian matching  │      │
│  └──────────────────┘    └───────────────┬──────────────┘      │
│                                          │                     │
│                                          ▼                     │
│                          /tracked_objects (MarkerArray)        │
│                                          │                     │
│                                          ▼                     │
│                                     RViz2                      │
└────────────────────────────────────────────────────────────────┘
```

### 4.2 Time Synchronization — Why It Matters

Camera and LiDAR produce data at different rates. If you naively pair the "most recent" message from each, you get temporal misalignment — the LiDAR cloud might be from 100ms ago while the image is fresh. Moving objects would project to the wrong pixel.

```
Camera frames (30 Hz, every 33ms):
  t=0ms  ────●────────●────────●──────────▶
            f1       f2       f3

LiDAR scans (10 Hz, every 100ms):
  t=0ms  ────────────────────●─────────────▶
                             L1

ApproximateTimeSynchronizer with slop=0.1s:
  Pairs: (f3, L1)  ← both within 100ms → callback fires
  Skips: f1, f2    ← no LiDAR scan nearby
```

---

## 5. ROS 2 Communication Model

### 5.1 Topics and the Publish-Subscribe Pattern

ROS 2 uses a **publish-subscribe** model over DDS. Nodes don't call each other directly — they broadcast to named topics.

```
PUBLISHER              TOPIC                SUBSCRIBER
   │                     │                      │
   │   publish(msg)       │                      │
   └────────────────────▶│                      │
                          │  DDS delivers        │
                          │─────────────────────▶│  callback fires
```

Your painting node uses two subscribers and one publisher:

```
/blackfly_s/cam0/image_rectified  ──┐
  (sensor_msgs/Image)               ├─▶ ApproximateTimeSynchronizer ─▶ _callback()
/velodyne/points_raw             ───┘
  (sensor_msgs/PointCloud2)

/painting/debug  ◀── PaintingNode publishes here
  (std_msgs/String)
```

### 5.2 Node Lifecycle

```
rclpy.init()
     │
     ▼
PaintingNode.__init__()
     ├── Read calib_file → init_projector()       ← loads calibration matrices
     ├── Read deeplab_repo_path + checkpoint_path
     │     └── load_model()                       ← loads DeepLab weights once
     ├── Create publisher (/painting/debug)
     ├── Create message_filter subscribers
     ├── Register ApproximateTimeSynchronizer
     └── Log "waiting for synced messages..."
     │
     ▼
rclpy.spin(node)   ← blocks here, event loop running
     │
     │  on every synced (image, cloud) pair:
     │    _callback() → decode image → segment_image() → paint_points() → publish
     │
     │  on Ctrl+C:
     │    break out of spin
     ▼
node.destroy_node()
rclpy.shutdown()
```

---

## 6. Your Modules — How They Work and How They Connect

### 6.1 Directory Layout

```
ros2_ws/src/
├── perception_framework/                ← arpitashil676's package
│   └── perception_framework/
│       └── lidar_to_image_projection.py ← KITTI projection math
│
└── point_painting/                      ← your main package (Ramez)
    ├── point_painting/
    │   ├── __init__.py
    │   ├── painting_node.py             ← ROS 2 node (I/O, sync, publish)
    │   ├── painting_logic.py            ← core algorithm (no ROS, testable)
    │   ├── rosbag_extractor.py          ← offline data extraction tool
    │   └── segmentation/
    │       ├── __init__.py
    │       └── deeplab_segmentation.py  ← DeepLab v3+ offline segmentation
    ├── test/
    │   └── test_painting_node.py        ← standalone unit tests
    ├── package.xml
    └── setup.py
```

**Key design principle:** `painting_logic.py` contains no ROS imports — it is pure Python. This means the core algorithm can be unit-tested on any machine without spinning up a ROS runtime.

---

### 6.2 `perception_framework` — KITTI Projection Math

**File:** [lidar_to_image_projection.py](../ros2_ws/src/perception_framework/perception_framework/lidar_to_image_projection.py)  
**Author:** arpitashil676  
**Purpose:** Converts LiDAR 3D points into 2D pixel coordinates using KITTI calibration matrices.

**How it works:**

```
calib.txt (KITTI format)
     │
     ▼
KittiLidarToImageProjector.__init__()
     ├── load P2         (3×4 camera projection matrix)
     ├── load R0_rect    (3×3 rectification → padded to 4×4)
     └── load Tr_velo_to_cam  (3×4 LiDAR→camera → padded to 4×4)
```

**Two public methods you use from painting_logic.py:**

```python
# Method 1: transforms N points to camera coordinate frame
camera_pts = projector.lidar_to_camera(lidar_points)
# Input:  (N, 3) numpy array [x, y, z] in LiDAR frame
# Output: (N, 3) numpy array [Xc, Yc, Zc] in rectified camera frame
# Math:   R0_rect @ Tr_velo_to_cam @ [x, y, z, 1]ᵀ  (for all N at once)

# Method 2: full pipeline — projects to pixel, filters in-bounds
image_pts, valid_lidar = projector.project_lidar_to_image(lidar_points, (h, w))
# Input:  (N, 3) lidar points, image (height, width)
# Output: (M, 2) pixel coords  +  (M, 3) the lidar points that survived
# M ≤ N  because points behind camera or outside frame are dropped
```

**The KITTI calibration chain in detail:**

```
LiDAR point [x, y, z]
     │
     │  homogeneous:  [x, y, z, 1]
     │
     ▼
Tr_velo_to_cam  (4×4)  —  rigid transform: LiDAR frame → camera frame
     │
     ▼
R0_rect         (4×4)  —  rectification: aligns camera to stereo baseline
     │
     ▼
camera point [Xc, Yc, Zc]
     │
     │  depth check: Zc > 0 (discard points behind camera)
     │
     ▼
cv2.projectPoints with P2's 3×3 camera_matrix
     │
     ▼
pixel [u, v]
     │
     │  bounds check: 0 ≤ u < W  and  0 ≤ v < H
     │
     ▼
valid projected point
```

---

### 6.3 `painting_logic.py` — Core Painting Algorithm

**File:** [painting_logic.py](../ros2_ws/src/point_painting/point_painting/painting_logic.py)  
**Author:** Ramez Alhinn  
**Purpose:** Ties the projector and segmentation map together. No ROS — pure numpy.

**Module state:**

```python
_projector: KittiLidarToImageProjector | None = None
```

This is a module-level singleton. The node calls `init_projector(calib_file)` once at startup, which loads the calibration matrices. Every subsequent call to `paint_points()` reuses the same projector.

**`init_projector(calib_file_path)`**
- Called once by `painting_node.py` at startup
- Instantiates `KittiLidarToImageProjector` and stores it in `_projector`
- If never called, `paint_points()` skips all points (safe fallback for testing)

**`paint_points(points_xyz, seg_image)` — full vectorized flow:**

```
points_xyz  (N, 3) float32              seg_image  (H, W) uint8
     │                                       │
     │  if _projector is None or N==0:        │
     │      return 0, N, [-1]*N               │
     │                                       │
     ▼                                       │
lidar_to_camera(points_xyz)                  │
     │                                       │
     ▼                                       │
depth_ok = camera_pts[:, 2] > 0  (mask)     │
depth_indices = original indices that passed │
cam_depth_pts = camera_pts[depth_ok]        │
     │                                       │
     ▼                                       │
cv2.projectPoints(cam_depth_pts, ...)        │
     │                                       │
     ▼                                       │
inside = (0 ≤ u < W) & (0 ≤ v < H)  (mask) │
inside_orig_indices = depth_indices[inside]  │
     │                                       │
     ▼                                       ▼
class_ids[inside_orig_indices] = seg_image[v_in, u_in]
     │
     ▼
return painted, skipped, class_ids.tolist()
```

**Why vectorised?** Calling `cv2.projectPoints` once on all N points is orders of magnitude faster than a Python loop over each point. A Velodyne scan has ~100,000 points — a per-point loop would add hundreds of milliseconds of latency per frame.

---

### 6.4 `painting_node.py` — The ROS 2 Node

**File:** [painting_node.py](../ros2_ws/src/point_painting/point_painting/painting_node.py)  
**Author:** Ramez Alhinn  
**Purpose:** All ROS I/O. Subscribes, synchronizes, decodes messages, calls `paint_points`, publishes results.

**Three ROS 2 parameters — all optional, all safe to omit:**

| Parameter | Purpose | Effect if missing |
|---|---|---|
| `calib_file` | Path to `calib.txt` | Projection skipped — painted=0 |
| `deeplab_repo_path` | Root of `pytorch-deeplab-xception` clone | Segmentation disabled — degraded mode |
| `checkpoint_path` | Path to `deeplab-resnet.pth.tar` | Segmentation disabled — degraded mode |

**Startup sequence:**

```python
# Calibration
init_projector(calib_file)               # loads KITTI matrices from calib.txt

# Segmentation model — repo path added to sys.path so 'modeling.deeplab' resolves
sys.path.insert(0, deeplab_repo_path)
from point_painting.segmentation.deeplab_segmentation import load_model
self._seg_model = load_model(checkpoint_path)   # loads weights once, sets eval mode
```

**Per-frame callback:**

```python
def _callback(self, img_msg, cloud_msg):
    # 1. Decode raw camera image
    cv_image = self._bridge.imgmsg_to_cv2(img_msg, 'passthrough')

    # 2. Segmentation — real labels or degraded fallback
    if self._seg_model is not None:
        pil_image = PilImage.fromarray(cv_image[..., ::-1])   # BGR → RGB
        seg_image = segment_image(self._seg_model, pil_image) # (H,W) class indices 0–20
    else:
        seg_image = cv_image[:, :, 0]   # raw channel — projection works, labels meaningless

    # 3. Point cloud → numpy (N×3)
    xyz = np.array([...], dtype=np.float32)

    # 4. Paint (calls KittiLidarToImageProjector internally)
    painted, skipped, _ = paint_points(xyz, seg_image)

    # 5. Publish
    self._debug_pub.publish(...)
```

**Run — Mode A (projection only, no model needed):**
```bash
ros2 run point_painting painting_node \
  --ros-args -p calib_file:=/workspace/calib.txt
```

**Run — Mode B (full pipeline with real segmentation):**
```bash
ros2 run point_painting painting_node --ros-args \
  -p calib_file:=/workspace/calib.txt \
  -p deeplab_repo_path:=/path/to/pytorch-deeplab-xception \
  -p checkpoint_path:=/path/to/deeplab-resnet.pth.tar
```

---

### 6.5 `segmentation/deeplab_segmentation.py` — Segmentation Module

**File:** [deeplab_segmentation.py](../ros2_ws/src/point_painting/point_painting/segmentation/deeplab_segmentation.py)  
**Author:** BelenNuñez  
**Purpose:** Runs DeepLab v3+ (ResNet backbone) on a camera image and produces a per-pixel class label map. Called live from `painting_node.py` on every synced frame when a model checkpoint is provided.

**Three importable functions:**

```python
model = load_model('/path/to/deeplab-resnet.pth.tar')
# Loads DeepLab weights, sets eval mode, moves to GPU if available

label_mask = segment_image(model, pil_image)
# Input:  PIL Image (any size)
# Output: numpy (H, W) uint8 — one integer class index per pixel
# Internally: resizes to 513×513, normalises, runs forward pass, argmax

rgb_image = decode_segmap(label_mask)
# Input:  numpy (H, W) class indices
# Output: numpy (H, W, 3) RGB — colourised for human viewing
```

**Run from command line:**
```bash
cd pytorch-deeplab-xception/   # must be here so 'from modeling.deeplab import DeepLab' resolves
python3 /path/to/deeplab_segmentation.py \
    --checkpoint /path/to/deeplab-resnet.pth.tar \
    --image /path/to/image.jpg
```

**PASCAL VOC classes used (21 total):**

| Index | Class | Colour |
|-------|-------|--------|
| 0 | background | black |
| 7 | car | dark magenta |
| 15 | person | olive |
| 2 | bicycle | dark green |
| ... | ... | ... |

**Note:** DeepLab is trained on PASCAL VOC (indoor/general objects). For automotive perception, a model trained on Cityscapes or KITTI would give better results. This is a valid starting point.

---

### 6.6 `rosbag_extractor.py` — Offline Data Extraction

**File:** [rosbag_extractor.py](../ros2_ws/src/point_painting/point_painting/rosbag_extractor.py)  
**Author:** carlosaterans-cmd  

**Purpose:** Reads a ROS 2 bag file and saves the first 10 camera frames as JPEGs and the first LiDAR scan as a `.npy` file. Used for offline experimentation — run segmentation or visualise data without playing a live bag.

**How to use:**

```python
from point_painting.rosbag_extractor import extract_bag_data

extract_bag_data(
    bag_path='/workspace/studentProject/',
    image_topic='/blackfly_s/cam0/image_rectified',
    lidar_topic='/velodyne/points_raw',
    output_dir='output_data'
)
```

**Output:**
```
output_data/
├── frame_0000.jpg   ← camera frame 0
├── frame_0001.jpg   ← camera frame 1
│   ...
├── frame_0009.jpg   ← camera frame 9
├── lidar_points.npy ← shape (N, 4): x, y, z, intensity
└── lidar_metadata.txt
```

**Use this to test DeepLab offline:**
1. Run `rosbag_extractor.py` → get `frame_0000.jpg`
2. Run `deeplab_segmentation.py --image output_data/frame_0000.jpg` → see segmentation
3. This proves both tools work before wiring them into a live ROS 2 node

---

### 6.7 Tests — `test_painting_node.py`

**File:** [test_painting_node.py](../ros2_ws/src/point_painting/test/test_painting_node.py)  
**Author:** Ramez Alhinn  
**Purpose:** Validates `paint_points()` in isolation — no ROS runtime needed.

The tests are designed around the real behaviour of `painting_logic.py`: without a calibration file loaded, the projector is `None` and all points are skipped. This is the correct safe-degraded state.

```
TEST 1: test_no_projector_skips_all_points
  Setup:  10 random points, 200×200 seg image, NO init_projector() called
  Expect: painted=0, skipped=10, all class_ids=-1
  Why:    Safety check — node must not crash without calibration

TEST 2: test_empty_point_cloud
  Setup:  0 points (empty array)
  Expect: painted=0, skipped=0, class_ids=[]
  Why:    Edge case — empty cloud from bag replay or sensor dropout

TEST 3: test_class_ids_length_matches_input
  Setup:  15 points
  Expect: len(class_ids) == 15
  Why:    The list must always be N entries — callers index into it by point number
```

**Run:**
```bash
python3 ros2_ws/src/point_painting/test/test_painting_node.py
```

---

## 7. The Calibration Math — How Projection Works

### 7.1 Coordinate Frames

```
┌──────────────────────────────────────────────────────────────────┐
│                      COORDINATE FRAMES                           │
│                                                                  │
│  LiDAR Frame (L)              Camera Frame (C)                  │
│  ─────────────                ────────────────                  │
│  Origin: LiDAR sensor         Origin: Camera optical center     │
│  X: forward                   X: right                         │
│  Y: left                      Y: down                          │
│  Z: up                        Z: forward (into scene)          │
│                                                                  │
│  Relationship: Tr_velo_to_cam transforms L → C                  │
│  R0_rect then aligns C to the stereo rectified frame            │
└──────────────────────────────────────────────────────────────────┘
```

### 7.2 The Three KITTI Matrices

**`Tr_velo_to_cam`** (3×4, padded to 4×4):
- A rigid body transform: rotation + translation
- Encodes the physical offset and orientation between the LiDAR and camera
- Measured once during sensor calibration — does not change

**`R0_rect`** (3×3, padded to 4×4):
- Rectification matrix for stereo camera alignment
- Makes the left camera image geometrically aligned to a canonical plane
- For monocular use, this is close to identity but not exactly

**`P2`** (3×4):
- The full camera projection matrix for the left colour camera
- Encodes focal lengths, principal point, and baseline offset
- Top-left 3×3 is the intrinsic matrix K

### 7.3 The Full Projection Equation

```
Given a LiDAR point p_L = [x, y, z]ᵀ:

Step 1: Homogeneous form
  p_h = [x, y, z, 1]ᵀ

Step 2: Transform LiDAR frame → rectified camera frame
  p_cam = R0_rect · Tr_velo_to_cam · p_h
  Result: [Xc, Yc, Zc]

Step 3: Depth check
  if Zc ≤ 0: discard (point is behind or on the camera plane)

Step 4: Project to pixel
  u = fx * (Xc / Zc) + cx
  v = fy * (Yc / Zc) + cy

Step 5: Bounds check
  if 0 ≤ u < W and 0 ≤ v < H: valid → sample seg_image[v, u]
  else: skipped → class_id = -1
```

### 7.4 What the `calib.txt` File Looks Like

```
P0: 7.215377e+02 0.000000e+00 6.095593e+02 0.000000e+00 ...
P1: 7.215377e+02 0.000000e+00 6.095593e+02 -3.875744e+02 ...
P2: 7.215377e+02 0.000000e+00 6.095593e+02 4.485728e+01 ...  ← left colour camera
P3: 7.215377e+02 0.000000e+00 6.095593e+02 -3.395242e+02 ...
R0_rect: 9.999239e-01 9.837760e-03 -7.445048e-03 ...
Tr_velo_to_cam: 7.533745e-03 -9.999714e-01 -6.166020e-04 -4.069766e-03 ...
```

`KittiLidarToImageProjector.load_kitti_calibration()` reads this file and reshapes each row into its matrix form.

### 7.5 How `calib.txt` Was Derived for This Project

No separate calibration file was provided with the bag. It was reconstructed from two sources already embedded in the data.

**Source 1 — Extrinsics from `/tf_static` (inside the bag)**

The bag contains a `/tf_static` topic with three static transforms. One of them is:

```
parent frame : velodyne
child frame  : cam0
translation  : x=0.692 m,  y=0.000 m,  z=-0.180 m
rotation     : x=-0.53433  y=0.54336  z=-0.46407  w=0.45154  (quaternion)
```

The translation says the camera sits **0.692 m in front** of and **0.180 m below** the LiDAR — a realistic roof-bar mounting.

The quaternion was converted to a 3×3 rotation matrix using the standard formula:

```
R[0][0] = 1 - 2(ry²+rz²)    R[0][1] = 2(rx·ry - rz·rw)   R[0][2] = 2(rx·rz + ry·rw)
R[1][0] = 2(rx·ry + rz·rw)  R[1][1] = 1 - 2(rx²+rz²)     R[1][2] = 2(ry·rz - rx·rw)
R[2][0] = 2(rx·rz - ry·rw)  R[2][1] = 2(ry·rz + rx·rw)   R[2][2] = 1 - 2(rx²+ry²)
```

**Convention correction — the critical step:** ROS tf stores the transform as "how to express the child frame in the parent frame" — geometrically, this is the *passive* (frame-orientation) rotation, which is the **transpose** of the *active* (point-rotating) matrix that KITTI expects. Using the quaternion directly gave a rotation that mapped LiDAR-forward into camera-sideways, putting all points behind the camera. The fix is to transpose R before building `Tr_velo_to_cam`.

The translation also needs to be re-expressed in the camera frame (not the LiDAR frame):

```
t_camera_frame = -R.T @ t_velodyne_frame
```

Validation: a LiDAR point at [10, 0, 0] (10 m straight ahead) projects to pixel (910, 174) on the 1920×1200 image — near the horizontal center and in the upper half, consistent with the camera being mounted below the LiDAR and angled slightly upward.

**Source 2 — Intrinsics from sensor datasheet (Blackfly S BFS-U3-51S5C)**

The bag has no `/camera_info` topic, so the camera intrinsics were estimated from hardware specs:

| Parameter | Value | Source |
|-----------|-------|--------|
| Sensor | Sony IMX250, 2/3 inch | Blackfly S datasheet |
| Pixel pitch | 3.45 µm | Datasheet |
| Image size | 1920 × 1200 px | Measured from bag messages |
| Lens | 8 mm C-mount (estimated) | Typical for this platform |
| `fx = fy` | 8.0 mm ÷ 0.00345 mm/px = **2318.8 px** | Calculated |
| `cx` | 959.5 px | Centre of 1920-wide rectified image |
| `cy` | 599.5 px | Centre of 1200-tall rectified image |

This gives a horizontal field of view of **45°** and vertical of **29°**, which is consistent with an 8 mm lens on this sensor.

**`R0_rect`** is set to identity (3×3) because this is a monocular camera — there is no stereo pair to rectify against.

**Important:** `fx` is the most uncertain value. If projected LiDAR points appear consistently shifted left/right on the image, the lens may be 6 mm (fx≈1739, HFOV≈58°) or 12 mm (fx≈3478, HFOV≈31°). You can tune it by overlaying the projected points on a saved camera frame and checking that road-surface points land at the bottom of the image and distant objects at the expected depth.

---

## 8. Dev Container — Your Execution Environment

### 8.1 Container Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                     macOS Host (your laptop)                     │
│                                                                  │
│  VS Code + Dev Containers extension                              │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │                 Docker Container                           │  │
│  │                                                            │  │
│  │  Base: ros:humble-ros-base (Ubuntu 22.04)                 │  │
│  │                                                            │  │
│  │  ROS 2 Packages:                                          │  │
│  │  ├── ros-humble-desktop                                   │  │
│  │  ├── ros-humble-rviz2                                     │  │
│  │  ├── ros-humble-vision-msgs                               │  │
│  │  ├── ros-humble-tf2-ros                                   │  │
│  │  └── ros-humble-message-filters                           │  │
│  │                                                            │  │
│  │  Python Packages:                                          │  │
│  │  ├── torch (CPU-only)    ← DeepLab, future PointPillars   │  │
│  │  ├── torchvision         ← DeepLab preprocessing          │  │
│  │  ├── ultralytics         ← future: YOLOv8 alternative     │  │
│  │  ├── open3d              ← future: 3D processing          │  │
│  │  ├── numpy, scipy        ← numeric computing              │  │
│  │  ├── opencv-python       ← projection via cv2             │  │
│  │  └── sensor_msgs_py      ← PointCloud2 parsing            │  │
│  │                                                            │  │
│  │  Workspace mount: /workspace → host repo                  │  │
│  │  ROS_DOMAIN_ID=42 (isolated from other ROS systems)       │  │
│  └────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

### 8.2 Build and Run

```bash
# Build both packages
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash

# Or use the alias (builds + sources in one step):
cw

# Run the painting node with calibration
ros2 run point_painting painting_node --ros-args -p calib_file:=/path/to/calib.txt

# In a second terminal — play your bag
ros2 bag play /workspace/studentProject/ --loop

# Monitor painting output
ros2 topic echo /painting/debug
```

`--symlink-install` means Python source edits take effect immediately without rebuilding.

---

## 9. Visualization with RViz2

### 9.1 Current Workflow

```bash
# Terminal 1 — play bag
ros2 bag play /workspace/studentProject/ --loop

# Terminal 2 — run painting node
source /workspace/ros2_ws/install/setup.bash
ros2 run point_painting painting_node --ros-args -p calib_file:=/path/to/calib.txt

# Terminal 3 — monitor
ros2 topic echo /painting/debug

# Terminal 4 — visualise raw data
rviz2
```

In RViz2:
- Set **Fixed Frame** to `velodyne` (or `velodyne_link`)
- Add **PointCloud2** → topic: `/velodyne/points_raw` → colour by **intensity**
- Add **Image** → topic: `/blackfly_s/cam0/image_rectified`

### 9.2 Future Visualization — Painted Cloud + Boxes

Once the painted cloud publisher is added:
```
/painted/points  → colour each point by class:
  Red   = car (class 7)
  Green = person (class 15)
  Blue  = cyclist (class 2)
  Gray  = background (class 0)

/tracked_objects/markers  → 3D bounding boxes as wireframes
                           → text labels with track ID
```

---

## 10. What's Left — Remaining Work

### 10.1 Implementation Status

```
DONE ✅
├── Package skeleton (setup.py, package.xml, __init__.py)
├── PaintingNode with ROS 2 lifecycle and three parameters:
│     ├── calib_file        — path to calib.txt
│     ├── deeplab_repo_path — path to pytorch-deeplab-xception clone
│     └── checkpoint_path   — path to deeplab-resnet.pth.tar
├── ApproximateTimeSynchronizer (image + LiDAR sync, slop=0.1s)
├── DeepLab segmentation wired into live callback
│     ├── load_model() called once at startup
│     └── segment_image() called per frame → (H,W) class indices
├── Graceful degraded mode — runs without model or calib file
├── KittiLidarToImageProjector (real KITTI calibration math)
│     ├── load_kitti_calibration() — parses calib.txt
│     ├── lidar_to_camera()        — R0_rect · Tr_velo_to_cam chain
│     └── project_lidar_to_image() — cv2.projectPoints + bounds filter
├── paint_points() — vectorised, wired to projector
├── Debug publisher (/painting/debug)
├── calib.txt derived from /tf_static inside the bag
├── Rosbag extractor — saves frames + LiDAR scan to disk
├── Standalone unit tests (3 passing)
└── Dev Container with all dependencies

TODO 🔲 (priority order)
├── 1. Painted PointCloud2 publisher
│       — package class_ids back into sensor_msgs/PointCloud2
│       — publish on /painted/points
│       — this is the input PointPillars needs
│
├── 2. PointPillars 3D detector
│       — pre-trained models available for KITTI
│       — input: painted cloud; output: 3D bounding boxes
│
├── 3. AB3DMOT tracker
│       — Kalman Filter + Hungarian Algorithm across frames
│       — input: per-frame detections; output: tracked objects with IDs
│
├── 4. RViz2 MarkerArray publisher
│       — CUBE markers for bounding boxes
│       — TEXT_VIEW_FACING markers for track IDs
│       — lifetime=0.15s so stale markers disappear
│
└── 5. Performance profiling
        — DeepLab on CPU adds ~2–5s per frame
        — measure per-stage latency, consider running segmentation
          at a lower rate than LiDAR (e.g. every 5th frame)

---

## 11. Quick Reference

### Topic Summary

| Topic | Direction | Type | Status |
|-------|-----------|------|--------|
| `/blackfly_s/cam0/image_rectified` | In | `sensor_msgs/Image` | Active |
| `/velodyne/points_raw` | In | `sensor_msgs/PointCloud2` | Active |
| `/painting/debug` | Out | `std_msgs/String` | Active |
| `/segmentation/label_map` | — | internal (in-node, not published) | N/A |
| `/painted/points` | Out | `sensor_msgs/PointCloud2` | TODO |
| `/tracked_objects/markers` | Out | `visualization_msgs/MarkerArray` | TODO |

### Key Commands

```bash
# Build workspace
cd /workspace/ros2_ws && colcon build --symlink-install && source install/setup.bash

# Run node — projection only (no segmentation model)
ros2 run point_painting painting_node --ros-args -p calib_file:=/workspace/calib.txt

# Run node — full pipeline with DeepLab segmentation
ros2 run point_painting painting_node --ros-args \
  -p calib_file:=/workspace/calib.txt \
  -p deeplab_repo_path:=/path/to/pytorch-deeplab-xception \
  -p checkpoint_path:=/path/to/deeplab-resnet.pth.tar

# Play bag data
ros2 bag play /workspace/studentProject/ --loop

# Monitor painting output
ros2 topic echo /painting/debug

# Check topic rates
ros2 topic hz /velodyne/points_raw
ros2 topic hz /blackfly_s/cam0/image_rectified

# Get camera calibration from bag
ros2 topic echo /blackfly_s/cam0/camera_info

# Run tests (no ROS needed)
python3 ros2_ws/src/point_painting/test/test_painting_node.py

# Run DeepLab offline on an extracted frame
python3 ros2_ws/src/point_painting/point_painting/segmentation/deeplab_segmentation.py \
    --checkpoint /path/to/deeplab-resnet.pth.tar \
    --image output_data/frame_0000.jpg

# Extract frames from bag for offline use
python3 -c "
from point_painting.rosbag_extractor import extract_bag_data
extract_bag_data('/workspace/studentProject/', '/blackfly_s/cam0/image_rectified', '/velodyne/points_raw')
"

# Open RViz2
rviz2
```

### File Map

| File | Author | Purpose |
|------|--------|---------|
| [painting_node.py](../ros2_ws/src/point_painting/point_painting/painting_node.py) | Ramez | ROS 2 node — message I/O, sync, publish |
| [painting_logic.py](../ros2_ws/src/point_painting/point_painting/painting_logic.py) | Ramez | Core algorithm — vectorised projection + painting |
| [lidar_to_image_projection.py](../ros2_ws/src/perception_framework/perception_framework/lidar_to_image_projection.py) | arpitashil676 | KITTI calibration math + projection |
| [deeplab_segmentation.py](../ros2_ws/src/point_painting/point_painting/segmentation/deeplab_segmentation.py) | BelenNuñez | DeepLab v3+ offline segmentation |
| [rosbag_extractor.py](../ros2_ws/src/point_painting/point_painting/rosbag_extractor.py) | carlosaterans-cmd | Extract frames + LiDAR from bag to disk |
| [test_painting_node.py](../ros2_ws/src/point_painting/test/test_painting_node.py) | Ramez | Standalone unit tests |
| [Architecture_Proposal.md](Architecture_Proposal.md) | Ramez | Design decisions: PointPainting vs AVOD |

### Glossary

| Term | Meaning |
|------|---------|
| PointPainting | Fusion method: project LiDAR onto segmented camera image, sample class labels, enrich 3D points |
| Extrinsics (`Tr_velo_to_cam`) | Rigid transform between LiDAR and camera — rotation + translation |
| Intrinsics (`P2`) | Camera projection matrix — encodes focal lengths and principal point |
| R0_rect | Stereo rectification matrix — aligns camera geometry to canonical plane |
| KITTI calibration | Standard format for autonomous driving sensor calibration files |
| ApproximateTimeSynchronizer | ROS 2 utility that pairs messages from multiple topics within a time tolerance |
| CvBridge | ROS package converting `sensor_msgs/Image` ↔ OpenCV numpy arrays |
| PointCloud2 | ROS message type for 3D point clouds — binary-encoded, multi-field |
| colcon | ROS 2 build tool (replaces catkin from ROS 1) |
| DeepLab v3+ | Semantic segmentation CNN (ResNet backbone, PASCAL VOC 21 classes) |
| PointPillars | Fast 3D detector — voxelises point cloud into vertical pillars, runs 2D CNN |
| AB3DMOT | 3D multi-object tracker using Kalman Filter + Hungarian Algorithm |
| BEV | Bird's Eye View — overhead 2D projection of 3D scene |
| DDS | Data Distribution Service — the middleware ROS 2 uses for topic communication |

---

*Updated to reflect codebase state after branch integration — April 2026*
