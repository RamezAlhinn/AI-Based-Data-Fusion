# PointPainting: System Engineering Learning Guide

**Project:** AI-Based Data Fusion — Camera + LiDAR Perception  
**Platform:** ROS 2 Humble · Docker Dev Container · Velodyne LiDAR · Blackfly S Camera  
**Author:** Ramez Alhinn  
**Date:** April 2026

---

## Table of Contents

1. [The Big Picture — Why Fuse Camera and LiDAR?](#1-the-big-picture)
2. [Sensor Primer — What Each Sensor Gives You](#2-sensor-primer)
3. [PointPainting Algorithm — The Core Idea](#3-pointpainting-algorithm)
4. [System Architecture — End-to-End Data Flow](#4-system-architecture)
5. [ROS 2 Communication Model](#5-ros-2-communication-model)
6. [Your Current Implementation — Module by Module](#6-your-current-implementation)
7. [The Calibration Problem — Projection Math](#7-the-calibration-problem)
8. [Dev Container — Your Execution Environment](#8-dev-container)
9. [Visualization with RViz2](#9-visualization-with-rviz2)
10. [What's Next — The Roadmap](#10-whats-next)
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
  │  ✅ Works at night, fog, rain (somewhat)         │
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

A 3D LiDAR point is just a location in space: `(x, y, z)`. Alone it has no semantic meaning — you cannot tell if it belongs to a car, tree, or road. But if you **project** that point onto the camera image, you can look up what the neural network thinks that pixel is. You then **paint** the 3D point with that semantic label. Now your point cloud entries are enriched: `(x, y, z, class_car, class_person, class_cyclist)`.

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

**Key camera model parameters (what you need for projection):**

| Symbol | Name | What it means |
|--------|------|---------------|
| `fx`   | Focal length X | How many pixels per meter at 1m distance (horizontal) |
| `fy`   | Focal length Y | Same but vertical |
| `cx`   | Principal point X | Pixel coordinate of the optical axis center |
| `cy`   | Principal point Y | Same but vertical |

These form the **intrinsic matrix K**:
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
     │  for each returned pulse:
     ▼
  azimuth angle (horizontal)
  elevation angle (vertical)
  range (distance)
  intensity (reflectivity)
  → converted to Cartesian (x, y, z, intensity)
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
        │
        └──────── X (forward, toward front of car)
       /
      /
     Y (left)

Coordinate system is right-handed.
Origin is at the LiDAR sensor center.
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
│  STEP 1: 2D Semantic Segmentation                              │
│  ┌──────────────────────────────────────────────────┐         │
│  │  Camera Image (H × W × 3)                        │         │
│  │         │                                        │         │
│  │         ▼                                        │         │
│  │  2D CNN (e.g., YOLOv8-seg, DeepLabV3)            │         │
│  │         │                                        │         │
│  │         ▼                                        │         │
│  │  Segmentation Map (H × W × C)                   │         │
│  │  Where C = number of classes                    │         │
│  │  Each pixel: [p_bg, p_car, p_ped, p_cyc, ...]   │         │
│  └──────────────────────────────────────────────────┘         │
│                           │                                    │
│                           │ seg scores per pixel               │
│                           ▼                                    │
│  STEP 2: Project LiDAR Points onto Image                       │
│  ┌──────────────────────────────────────────────────┐         │
│  │  LiDAR Point Cloud (N × 4) [x, y, z, intensity] │         │
│  │         │                                        │         │
│  │         │  For each point (x, y, z):             │         │
│  │         │  1. Transform to camera frame           │         │
│  │         │     p_cam = T_cam_lidar · [x,y,z,1]ᵀ  │         │
│  │         │  2. Project to pixel using K            │         │
│  │         │     [u·d, v·d, d] = K · p_cam          │         │
│  │         │     u = u·d / d,  v = v·d / d          │         │
│  │         │  3. Discard if outside image bounds     │         │
│  │         │  4. Discard if depth ≤ 0 (behind cam)  │         │
│  └──────────────────────────────────────────────────┘         │
│                           │                                    │
│                           │ (u, v) pixel coordinates           │
│                           ▼                                    │
│  STEP 3: Paint Points with Semantic Scores                     │
│  ┌──────────────────────────────────────────────────┐         │
│  │  For each in-bounds projected point:             │         │
│  │    seg_scores = seg_map[v, u, :]                 │         │
│  │    painted_point = [x, y, z, intensity,          │         │
│  │                     p_car, p_ped, p_cyc, ...]    │         │
│  └──────────────────────────────────────────────────┘         │
│                           │                                    │
│                           │ painted cloud (N × 4+C)            │
│                           ▼                                    │
│  STEP 4: 3D Object Detection                                   │
│  ┌──────────────────────────────────────────────────┐         │
│  │  PointPillars / PointRCNN etc.                   │         │
│  │  Input: enriched point cloud                     │         │
│  │  Output: 3D bounding boxes + class labels        │         │
│  └──────────────────────────────────────────────────┘         │
│                           │                                    │
│                           ▼                                    │
│  STEP 5: Multi-Object Tracking                                 │
│  ┌──────────────────────────────────────────────────┐         │
│  │  AB3DMOT (Kalman Filter + Hungarian Algorithm)   │         │
│  │  Input: detections per frame                     │         │
│  │  Output: tracked objects with IDs across frames  │         │
│  └──────────────────────────────────────────────────┘         │
│                           │                                    │
│                           ▼                                    │
│  STEP 6: Visualization (RViz2)                                 │
│  ┌──────────────────────────────────────────────────┐         │
│  │  MarkerArray: 3D boxes with track IDs            │         │
│  │  PointCloud2: painted colored cloud              │         │
│  │  Image: camera with projected detections         │         │
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
│ Camera use       │ Offline seg pass │ ResNet18 feature map      │
│ LiDAR use        │ Enriched cloud   │ BEV feature map          │
│ Feature maps     │ 1 (LiDAR only)   │ 2 simultaneously          │
│ OOM Risk         │ Very Low ✅       │ Moderate-High ❌          │
│ Complexity       │ Low ✅            │ High ❌                   │
│ Latency          │ Low ✅            │ Higher ❌                 │
│ Target hardware  │ Laptop (yours)✅  │ GPU workstation          │
└──────────────────┴──────────────────┴──────────────────────────┘

Decision: PointPainting chosen for laptop-class, CPU-only execution.
```

---

## 4. System Architecture — End-to-End Data Flow

### 4.1 Full System Block Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                         DATA SOURCES                                │
│                                                                     │
│   ┌──────────────────┐          ┌──────────────────────────────┐    │
│   │  Blackfly S Cam  │          │      Velodyne LiDAR          │    │
│   │  (or bag file)   │          │      (or bag file)           │    │
│   └────────┬─────────┘          └─────────────┬────────────────┘    │
│            │                                  │                     │
│   sensor_msgs/Image              sensor_msgs/PointCloud2            │
│   /blackfly_s/cam0/              /velodyne/points_raw               │
│   image_rectified                                                   │
└────────────┼────────────────────────────────  ┼─────────────────────┘
             │                                  │
             │ ROS 2 topics (DDS middleware)     │
             ▼                                  ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    POINTPAINTING NODE                               │
│                    (painting_node.py)                               │
│                                                                     │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │           ApproximateTimeSynchronizer                        │   │
│  │  Waits for image + cloud within 0.1s of each other          │   │
│  │  Queue: 10 messages each                                     │   │
│  └─────────────────────────┬────────────────────────────────────┘   │
│                             │ both messages synced                   │
│                             ▼                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  Image → CvBridge → numpy array (H×W) or (H×W×C)           │   │
│  │  If multi-channel: extract channel 0                         │   │
│  │                                                              │   │
│  │  PointCloud2 → read_points(x,y,z) → numpy (N×3) float32     │   │
│  └─────────────────────────┬────────────────────────────────────┘   │
│                             │                                       │
│                             ▼                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │              painting_logic.py                               │   │
│  │                                                              │   │
│  │  paint_points(points_xyz, seg_image)                         │   │
│  │    → for each point:                                         │   │
│  │        (u, v) = project_point_to_pixel(x, y, z)  [STUB]     │   │
│  │        if in bounds: class_id = seg_image[v, u]             │   │
│  │        else: class_id = -1 (skipped)                        │   │
│  │    → returns (painted_count, skipped_count, class_ids)       │   │
│  └─────────────────────────┬────────────────────────────────────┘   │
│                             │                                       │
│                             ▼                                       │
│  ┌──────────────────────────────────────────────────────────────┐   │
│  │  Publish: /painting/debug (std_msgs/String)                  │   │
│  │  "frame=42 painted=8500 skipped=1200"                        │   │
│  │  Log every 50 frames to ROS logger                           │   │
│  └──────────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────────┘
             │
             │  [FUTURE OUTPUTS — not yet implemented]
             ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    DOWNSTREAM PIPELINE                              │
│                                                                     │
│  /painted/points  (PointCloud2 with semantic features)              │
│         │                                                           │
│         ▼                                                           │
│  ┌──────────────────┐     ┌──────────────────────────────────┐      │
│  │   PointPillars   │────▶│         AB3DMOT Tracker          │      │
│  │  3D Detector     │     │  Kalman Filter + Hungarian Alg.  │      │
│  │  (to integrate)  │     │  (to integrate)                  │      │
│  └──────────────────┘     └──────────────────┬───────────────┘      │
│                                              │                      │
│                                              ▼                      │
│                            /tracked_objects  (FusedObjectArray)     │
│                                              │                      │
│                                              ▼                      │
│                                    ┌──────────────┐                 │
│                                    │    RViz2     │                 │
│                                    │ MarkerArray  │                 │
│                                    │  3D Boxes    │                 │
│                                    │  Track IDs   │                 │
│                                    └──────────────┘                 │
└─────────────────────────────────────────────────────────────────────┘
```

### 4.2 Time Synchronization — Why It Matters

Camera and LiDAR produce data at different rates and their timestamps drift. If you naively pair the "most recent" message from each, you get temporal misalignment — the LiDAR cloud is from 100ms ago while the image is fresh. Objects in motion would be in the wrong place when you project.

```
Timeline:
                            Camera frames (30 Hz, every 33ms)
  t=0ms    ─────────────────●────────────●────────────●──────────▶
                           f1           f2           f3

                            LiDAR scans (10 Hz, every 100ms)
  t=0ms    ─────────────────────────────────────────●─────────────▶
                                                   L1

  ApproximateTimeSynchronizer with slop=0.1s (100ms):
  Pairs: (f3, L1) ← both within 100ms of each other → CALLBACK FIRES
  Skips f1, f2 which had no matching LiDAR scan nearby
```

The `slop=0.1` parameter means "accept a timestamp difference up to 100ms." For a 10 Hz LiDAR this is exactly one scan interval, so you always get a valid pair.

---

## 5. ROS 2 Communication Model

### 5.1 Topics, Publishers, Subscribers

ROS 2 uses a **publish-subscribe** model over DDS (Data Distribution Service). Nodes don't call each other directly — they broadcast to named topics.

```
PUBLISHER                 TOPIC                  SUBSCRIBER(S)
   │                        │                         │
   │    publish(msg)        │                         │
   └───────────────────────▶│                         │
                            │  DDS delivers message   │
                            │────────────────────────▶│
                            │                         │   callback(msg) fires
```

Your node uses **two subscriber + one publisher**:

```
Topic: /blackfly_s/cam0/image_rectified   ──┐
  (sensor_msgs/Image)                       ├─▶ ApproximateTimeSynchronizer ─▶ _callback()
Topic: /velodyne/points_raw              ───┘
  (sensor_msgs/PointCloud2)

Topic: /painting/debug                   ◀── PaintingNode publishes here
  (std_msgs/String)
```

### 5.2 QoS (Quality of Service)

ROS 2 inherits DDS QoS policies. Sensor data typically uses:
- **Reliability:** Best effort (drop if subscriber is slow — no retransmit)
- **Durability:** Volatile (don't keep messages for late joiners)
- **History depth:** Keep last N messages

`message_filters.Subscriber` defaults to sensor-compatible QoS automatically.

### 5.3 Node Lifecycle

```
rclpy.init()
     │
     ▼
PaintingNode.__init__()
     ├── Create publishers
     ├── Create subscribers (message_filters)
     ├── Register synchronizer + callback
     └── Log "waiting for synced messages..."
     │
     ▼
rclpy.spin(node)   ← blocks here, event loop running
     │
     │  ┌─── on every synced (image, cloud) pair:
     │  │       _callback(img_msg, cloud_msg) fires
     │  │       ├── decode messages
     │  │       ├── call paint_points()
     │  │       └── publish debug string
     │  └───────────────────────────────────────────────
     │
     │  ┌─── on Ctrl+C (KeyboardInterrupt):
     │  │       break out of spin
     │  └───────────────────────────────────────────────
     │
     ▼
node.destroy_node()
rclpy.shutdown()
```

---

## 6. Your Current Implementation — Module by Module

### 6.1 Directory Layout

```
ros2_ws/src/point_painting/
├── point_painting/
│   ├── __init__.py          ← Python package marker
│   ├── painting_node.py     ← ROS 2 node (I/O, message handling)
│   └── painting_logic.py   ← Pure algorithm (no ROS, testable alone)
├── test/
│   └── test_painting_node.py  ← Standalone unit tests
├── package.xml              ← ROS 2 package manifest
├── setup.py                 ← Python entry points
└── setup.cfg                ← Build config
```

**Design decision:** Separating `painting_logic.py` from `painting_node.py` means your core math can be unit-tested without spinning up a ROS runtime. This is the right architectural choice.

### 6.2 `painting_logic.py` — The Core Algorithm

```python
# painting_logic.py (your current implementation)

def project_point_to_pixel(x, y, z):
    # STUB — returns hardcoded pixel (100, 100) for all points
    # TODO: replace with real calibration math (see Section 7)
    return (100, 100)

def paint_points(points_xyz, seg_image):
    """
    points_xyz  : numpy (N, 3) float32  — LiDAR points in sensor frame
    seg_image   : numpy (H, W) uint8    — 2D class label map from camera
    returns     : (painted_count, skipped_count, class_ids_list)
    """
    h, w = seg_image.shape
    painted = 0
    skipped = 0
    class_ids = []

    for point in points_xyz:
        x, y, z = point
        u, v = project_point_to_pixel(x, y, z)       # pixel column, row
        if 0 <= v < h and 0 <= u < w:                # bounds check
            class_ids.append(int(seg_image[v, u]))   # sample label
            painted += 1
        else:
            class_ids.append(-1)                      # out of view
            skipped += 1

    return painted, skipped, class_ids
```

**What the stub means in practice:**

```
Current behavior with stub projection (100, 100):
  ┌──────────────────────────────────────────────┐
  │ seg_image (200×200 in test)                  │
  │                                              │
  │    .....*....................                 │
  │    .....*....................   * = pixel     │
  │    ...(100,100)...........         (100,100) │
  │    .....*....................                 │
  │    ..............................            │
  └──────────────────────────────────────────────┘
  ALL N points project to the same single pixel.
  Every point gets the same class_id: seg_image[100, 100]
  
  With a real camera:
  Different 3D points → different pixel locations → different labels
```

### 6.3 `painting_node.py` — The ROS 2 Node

Key annotated flow:

```python
class PaintingNode(Node):
    def __init__(self):
        super().__init__('painting_node')
        self._bridge = CvBridge()       # converts ROS Image ↔ numpy
        self._frame_count = 0

        # Publisher: debug statistics as a string
        self._debug_pub = self.create_publisher(String, '/painting/debug', 10)

        # Two message_filter subscribers (NOT regular rclpy subscribers)
        # message_filters intercepts messages before the callback
        img_sub = message_filters.Subscriber(self, Image, '/blackfly_s/cam0/image_rectified')
        lidar_sub = message_filters.Subscriber(self, PointCloud2, '/velodyne/points_raw')

        # Synchronizer: fires _callback only when both arrive within 0.1s
        sync = ApproximateTimeSynchronizer([img_sub, lidar_sub], queue_size=10, slop=0.1)
        sync.registerCallback(self._callback)

    def _callback(self, img_msg, cloud_msg):
        # 1. Decode image
        seg_image = self._bridge.imgmsg_to_cv2(img_msg, desired_encoding='passthrough')
        if seg_image.ndim == 3:
            seg_image = seg_image[:, :, 0]   # collapse to single-channel label map

        # 2. Decode point cloud — extract only x, y, z fields
        gen = read_points(cloud_msg, field_names=('x', 'y', 'z'), skip_nans=True)
        xyz = np.array(list(gen), dtype=np.float32)   # shape (N, 3)

        # 3. Paint
        painted, skipped, _ = paint_points(xyz, seg_image)

        # 4. Publish debug stats
        self._frame_count += 1
        msg = String()
        msg.data = f'frame={self._frame_count} painted={painted} skipped={skipped}'
        self._debug_pub.publish(msg)
```

### 6.4 Tests — `test_painting_node.py`

Your tests validate the logic layer in isolation. Three test cases:

```
TEST 1: test_stub_projection_returns_valid_pixel
  Input:  project_point_to_pixel(1.0, 2.0, 5.0)
  Expect: returns tuple of two integers
  Status: PASSED ✅

TEST 2: test_all_points_get_class_id
  Setup:  200×200 seg image filled with class 7
          10 random (x,y,z) points
          stub always projects to (100,100) which is in-bounds
  Expect: painted=10, skipped=0, all class_ids=7
  Status: PASSED ✅

TEST 3: test_out_of_bounds_points_are_skipped
  Setup:  50×50 seg image (tiny)
          stub projects to (100,100) which is OUT of 50×50 bounds
          5 points
  Expect: painted=0, skipped=5
  Status: PASSED ✅
```

Run with:
```bash
python3 ros2_ws/src/point_painting/test/test_painting_node.py
```

---

## 7. The Calibration Problem — Projection Math

This is the most important technical section — the part the stub bypasses.

### 7.1 Coordinate Frames

You have three coordinate frames that must be aligned:

```
┌──────────────────────────────────────────────────────────────────┐
│                      COORDINATE FRAMES                           │
│                                                                  │
│  World Frame (W)                                                 │
│  ─────────────                                                   │
│  Fixed frame attached to map / initial position                  │
│                                                                  │
│  LiDAR Frame (L)           Camera Frame (C)                     │
│  ─────────────             ────────────────                      │
│  Origin: LiDAR sensor      Origin: Camera optical center        │
│  X: forward                X: right                             │
│  Y: left                   Y: down                              │
│  Z: up                     Z: forward (into scene)              │
│                                                                  │
│  Relationship: T_cam_lidar transforms L → C                     │
│  (extrinsic calibration, measured physically)                    │
└──────────────────────────────────────────────────────────────────┘
```

### 7.2 Projection Pipeline — Step by Step

**Given:** A LiDAR point `p_L = [x, y, z]` in LiDAR frame

**Step 1: Transform to camera frame**
```
                          [ R | t ]
p_cam = T_cam_lidar · [x, y, z, 1]ᵀ   where T = 4×4 matrix
                          [ 0 | 1 ]

R = 3×3 rotation matrix (how LiDAR axes align with camera axes)
t = 3×1 translation vector (physical offset between sensors)

Result: p_cam = [Xc, Yc, Zc]
```

**Step 2: Check depth**
```
if Zc ≤ 0:
    discard point  (behind the camera — would project inverted)
```

**Step 3: Apply camera intrinsics**
```
u = fx * (Xc / Zc) + cx     ← pixel column
v = fy * (Yc / Zc) + cy     ← pixel row

This is the pinhole camera model.
Zc divides out the perspective projection.
fx, fy scale from meters to pixels.
cx, cy shift origin from optical center to image corner.
```

**Step 4: Bounds check**
```
if 0 ≤ u < image_width and 0 ≤ v < image_height:
    pixel = (int(u), int(v))
    sample seg_image[pixel[1], pixel[0]]
else:
    discard point
```

### 7.3 What Your Stub Needs to Become

```python
# Current stub (painting_logic.py):
def project_point_to_pixel(x, y, z):
    return (100, 100)   # placeholder

# What it needs to be:
import numpy as np

# These come from your sensor calibration files
T_cam_lidar = np.array([...])   # 4×4 extrinsic matrix
K = np.array([                  # 3×3 intrinsic matrix
    [fx,  0, cx],
    [ 0, fy, cy],
    [ 0,  0,  1]
])

def project_point_to_pixel(x, y, z):
    # Step 1: Transform to camera frame
    p_lidar = np.array([x, y, z, 1.0])
    p_cam = T_cam_lidar @ p_lidar        # 4×1
    Xc, Yc, Zc = p_cam[:3]

    # Step 2: Depth check
    if Zc <= 0:
        return None   # behind camera

    # Step 3: Project to pixel
    u = K[0, 0] * (Xc / Zc) + K[0, 2]
    v = K[1, 1] * (Yc / Zc) + K[1, 2]

    return (int(u), int(v))
```

Calibration matrices come from:
- Running a camera-LiDAR calibration tool (e.g., `kalibr`, `lidar_camera_calibration`)
- Or loading them from your bag file's `/camera_info` topic

---

## 8. Dev Container — Your Execution Environment

### 8.1 Container Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                     macOS Host (your laptop)                     │
│                                                                  │
│  VS Code + Remote-Containers extension                           │
│  ┌────────────────────────────────────────────────────────────┐  │
│  │                 Docker Container                           │  │
│  │                                                            │  │
│  │  Base: ros:humble-ros-base (Ubuntu 22.04)                 │  │
│  │                                                            │  │
│  │  ROS 2 Packages:                                          │  │
│  │  ├── ros-humble-desktop         (core + tools)            │  │
│  │  ├── ros-humble-rviz2           (3D visualization)        │  │
│  │  ├── ros-humble-vision-msgs     (detection message types) │  │
│  │  ├── ros-humble-tf2-ros         (coordinate transforms)   │  │
│  │  └── ros-humble-message-filters (synchronization)         │  │
│  │                                                            │  │
│  │  Python Packages:                                          │  │
│  │  ├── torch (CPU-only)           (future: PointPillars)    │  │
│  │  ├── ultralytics                (future: YOLOv8 seg)      │  │
│  │  ├── open3d                     (future: 3D processing)   │  │
│  │  ├── numpy, scipy               (numeric computing)       │  │
│  │  └── sensor_msgs_py             (PointCloud2 parsing)     │  │
│  │                                                            │  │
│  │  Workspace: /workspace (→ host repo mount)                │  │
│  │  ROS_DOMAIN_ID=42 (isolated from other ROS systems)       │  │
│  └────────────────────────────────────────────────────────────┘  │
└──────────────────────────────────────────────────────────────────┘
```

### 8.2 Auto-Build on Container Start

When you open the dev container, VS Code runs:
```bash
cd /workspace/ros2_ws && colcon build --symlink-install 2>/dev/null || true
```

`--symlink-install` means Python files are symlinked (not copied) into the install directory — any edits to source files take effect without rebuilding.

### 8.3 Shell Aliases

```bash
cw   →  cd /workspace/ros2_ws && colcon build --symlink-install && source install/setup.bash
cs   →  cd /workspace/ros2_ws/src
```

### 8.4 Colcon Build System

```
colcon build --packages-select point_painting
     │
     ▼
reads setup.py / package.xml
     │
     ├── installs Python package into install/point_painting/
     ├── creates entry point script: install/lib/point_painting/painting_node
     └── writes install/setup.bash (sourced to set PATH, PYTHONPATH, etc.)

source install/setup.bash   ← must run after every build (or use cw alias)

ros2 run point_painting painting_node
     │
     └── resolves to: python3 <entry_point> → PaintingNode.main()
```

---

## 9. Visualization with RViz2

### 9.1 What RViz2 Displays

RViz2 is ROS 2's built-in 3D visualization tool. It subscribes to topics and renders their content in a 3D viewport.

```
┌──────────────────────────────────────────────────────────────────────┐
│                          RVIZ2 WINDOW                                │
│                                                                      │
│  ┌─────────────────────┐  ┌──────────────────────────────────────┐   │
│  │  Displays Panel     │  │         3D Viewport                  │   │
│  │                     │  │                                      │   │
│  │  [+] PointCloud2    │  │   · · · · · · · · · · ·              │   │
│  │      Topic:         │  │  · · · · · · [CAR BOX] · · · ·       │   │
│  │      /velodyne/     │  │   · · · · · · · · · · ·              │   │
│  │      points_raw     │  │       · · · · · · · · ·              │   │
│  │                     │  │                                      │   │
│  │  [+] Image          │  │  (LiDAR points as colored dots)      │   │
│  │      Topic:         │  │  (future: bounding boxes)            │   │
│  │      /blackfly_s/   │  │  (future: track ID labels)           │   │
│  │      cam0/...       │  │                                      │   │
│  │                     │  └──────────────────────────────────────┘   │
│  │  [+] MarkerArray    │                                             │
│  │      (future)       │  Fixed Frame: velodyne                      │
│  └─────────────────────┘                                             │
└──────────────────────────────────────────────────────────────────────┘
```

### 9.2 Current Visualization Workflow

1. **Play bag file** (Terminal 1):
   ```bash
   ros2 bag play <path-to-bag> --loop
   ```

2. **Check topics publishing** (Terminal 2):
   ```bash
   ros2 topic list
   ros2 topic hz /velodyne/points_raw        # should show ~10 Hz
   ros2 topic hz /blackfly_s/cam0/image_rectified
   ```

3. **Run painting node** (Terminal 3):
   ```bash
   source /workspace/ros2_ws/install/setup.bash
   ros2 run point_painting painting_node
   ```

4. **Monitor output** (Terminal 4):
   ```bash
   ros2 topic echo /painting/debug
   ```

5. **Open RViz2** (Terminal 5):
   ```bash
   rviz2
   ```
   In RViz2:
   - Set **Fixed Frame** to `velodyne` (or `velodyne_link`)
   - Add **PointCloud2** display → topic: `/velodyne/points_raw`
   - Add **Image** display → topic: `/blackfly_s/cam0/image_rectified`
   - Color points by **intensity** to see structure

### 9.3 Future Visualization — Painted Cloud + Bounding Boxes

Once projection and segmentation are implemented, you'll visualize:

```
Painted PointCloud2 (topic: /painted/points)
  → Color each point by its semantic class:
     Red   = car
     Green = pedestrian
     Blue  = cyclist
     Gray  = background

MarkerArray (topic: /tracked_objects/markers)
  → 3D bounding boxes as wireframes
  → Text labels with track ID and class
  → Arrows showing velocity vectors
```

---

## 10. What's Next — The Roadmap

### 10.1 Implementation Status

```
COMPLETED ✅
├── Package skeleton (setup.py, package.xml)
├── PaintingNode class with proper ROS 2 lifecycle
├── ApproximateTimeSynchronizer (image + LiDAR sync)
├── CvBridge integration (Image → numpy)
├── PointCloud2 field extraction (x, y, z)
├── paint_points() loop with bounds checking
├── Debug publisher (/painting/debug)
├── Standalone unit tests (3 passing)
└── Dev Container with all dependencies

IN PROGRESS / STUB ⚠️
└── project_point_to_pixel() — hardcoded (100,100)

TODO 🔲 (priority order)
├── 1. Real projection (calibration matrices)
├── 2. Camera segmentation (YOLOv8)
├── 3. Painted PointCloud2 publisher
├── 4. PointPillars 3D detector
├── 5. AB3DMOT tracker
├── 6. RViz2 MarkerArray visualization
└── 7. Performance profiling
```

### 10.2 Step 1 — Real Projection (Highest Priority)

**What to do:** Replace the stub in `painting_logic.py` with real camera projection.

**What you need:**
- Camera intrinsics: `fx, fy, cx, cy` from `/camera_info` topic or calibration file
- Extrinsic transform: `T_cam_lidar` (4×4 matrix) — physical measurement or calibration

**How to get calibration from a bag file:**
```bash
ros2 topic echo /blackfly_s/cam0/camera_info
```
This gives you K (intrinsics) and distortion coefficients.

For extrinsics (camera ↔ LiDAR transform), you need either:
- A KITTI-format `calib.txt` file
- A `/tf_static` broadcast in your bag file
- Manual calibration with a checkerboard + `kalibr`

**Where to make the change:**  
[painting_logic.py](ros2_ws/src/point_painting/point_painting/painting_logic.py) — `project_point_to_pixel()` function

### 10.3 Step 2 — 2D Semantic Segmentation

**What to do:** Add a YOLOv8 segmentation model that runs on each camera frame and produces a semantic label map.

**Design options:**

```
Option A: In-node inference
  PaintingNode._callback()
    ├── Run YOLOv8-seg on raw image
    ├── Extract class mask (H×W)
    └── Pass to paint_points()
  
  Pros: No extra node, simpler topology
  Cons: Slows callback; harder to swap models

Option B: Separate segmentation node (recommended)
  /blackfly_s/cam0/image_rectified
    → SegmentationNode (runs YOLOv8)
    → /segmentation/label_map (Image, mono8)
    → PaintingNode (subscribes instead of raw image)
  
  Pros: Clean separation; can run independently; swap model easily
  Cons: One more node to manage
```

**ultralytics is already in your Docker image** — no pip install needed:
```python
from ultralytics import YOLO
model = YOLO('yolov8n-seg.pt')   # nano = fastest on CPU
results = model(image)
```

### 10.4 Step 3 — Painted PointCloud2 Publisher

**What to do:** Package enriched points back into a `sensor_msgs/PointCloud2` message and publish it.

Each painted point becomes:
```
[x, y, z, intensity, p_background, p_car, p_pedestrian, p_cyclist]
   ─────────────────  ────────────────────────────────────────────
   from LiDAR          from segmentation neural network
```

**Why this matters:** PointPillars (Step 4) needs this enriched cloud as input.

### 10.5 Step 4 — PointPillars 3D Detector

PointPillars converts the irregular point cloud into a 2D pseudo-image (Bird's Eye View) by "pillaring" points vertically, then runs a 2D CNN on it for fast 3D detection.

```
Painted PointCloud (N × 8)
     │
     ▼
Voxelize into vertical pillars (P × max_pts × 8)
     │
     ▼
PointNet-style feature extraction per pillar → (P × C)
     │
     ▼
Scatter features into 2D BEV pseudo-image (H × W × C)
     │
     ▼
2D backbone CNN → detection head
     │
     ▼
3D bounding boxes + class labels + scores
```

**Pre-trained models exist for KITTI and nuScenes.** You may need to fine-tune if your sensor setup differs.

### 10.6 Step 5 — AB3DMOT Tracker

Tracking maintains object identity across frames using:

```
Frame t:
  Detections: [box_A, box_B, box_C]
       │
       ▼
  Kalman Filter: predict positions from frame t-1 tracks
       │
       ▼
  Hungarian Algorithm: match predictions ↔ detections (min cost)
       │
       ▼
  Update matched tracks, create new tracks, kill lost tracks
       │
       ▼
  Output: [track_1 (box_A, id=3), track_2 (box_B, id=7), ...]

Frame t+1:
  Even if a detection is briefly missed, the track persists
  via Kalman prediction (dead reckoning)
```

### 10.7 Step 6 — RViz2 MarkerArray

Publish `visualization_msgs/MarkerArray` with one `Marker` per tracked object:
- `CUBE` marker for bounding box wireframe
- `TEXT_VIEW_FACING` marker for track ID label
- Set `lifetime` to 0.15s so stale markers disappear automatically

---

## 11. Quick Reference

### Topic Summary

| Topic | Direction | Type | Status |
|-------|-----------|------|--------|
| `/blackfly_s/cam0/image_rectified` | In | `sensor_msgs/Image` | Active |
| `/velodyne/points_raw` | In | `sensor_msgs/PointCloud2` | Active |
| `/painting/debug` | Out | `std_msgs/String` | Active |
| `/painted/points` | Out | `sensor_msgs/PointCloud2` | TODO |
| `/tracked_objects` | Out | `fusion_msgs/FusedObjectArray` | TODO |
| `/tracked_objects/markers` | Out | `visualization_msgs/MarkerArray` | TODO |

### Key Commands

```bash
# Build
cd /workspace/ros2_ws && colcon build --packages-select point_painting
source install/setup.bash

# Run node
ros2 run point_painting painting_node

# Play bag data
ros2 bag play <path> --loop

# Monitor debug output
ros2 topic echo /painting/debug

# Check message rate
ros2 topic hz /velodyne/points_raw

# Get camera calibration
ros2 topic echo /blackfly_s/cam0/camera_info

# Run tests (no ROS needed)
python3 ros2_ws/src/point_painting/test/test_painting_node.py

# Open RViz2
rviz2
```

### File Map

| File | Purpose |
|------|---------|
| [painting_node.py](ros2_ws/src/point_painting/point_painting/painting_node.py) | ROS 2 node — message I/O, synchronization |
| [painting_logic.py](ros2_ws/src/point_painting/point_painting/painting_logic.py) | Core algorithm — projection, painting |
| [test_painting_node.py](ros2_ws/src/point_painting/test/test_painting_node.py) | Unit tests |
| [package.xml](ros2_ws/src/point_painting/package.xml) | ROS 2 dependencies |
| [Dockerfile](. devcontainer/Dockerfile) | Container environment |
| [Architecture_Proposal.md](docs/Architecture_Proposal.md) | Design decisions |

### Glossary

| Term | Meaning |
|------|---------|
| PointPainting | Fusion method: project LiDAR points onto segmented camera image, sample labels |
| Extrinsics | Rigid transform between two sensors (rotation + translation) |
| Intrinsics | Internal camera parameters: focal length, principal point |
| ApproximateTimeSynchronizer | ROS 2 utility that pairs messages from multiple topics within a time tolerance |
| CvBridge | ROS package that converts between `sensor_msgs/Image` and OpenCV numpy arrays |
| PointCloud2 | ROS message type for 3D point clouds — binary-encoded, multi-field |
| colcon | ROS 2 build tool (replaces catkin from ROS 1) |
| AB3DMOT | 3D multi-object tracker using Kalman Filter + Hungarian Algorithm |
| PointPillars | Fast 3D detector: voxelizes point cloud into vertical pillars, runs 2D CNN |
| BEV | Bird's Eye View — overhead 2D projection of 3D scene |
| DDS | Data Distribution Service — the middleware ROS 2 uses for topic communication |

---

*Document generated from codebase at commit `15f1ff9` (Point painting node) — April 2026*
