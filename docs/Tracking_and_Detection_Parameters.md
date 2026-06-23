# Tracking and Detection Parameters Reference

This document provides a comprehensive guide to the parameters and thresholds that govern the 3D detection and tracking pipeline in the **PointPainting + Frustum Detection** stack. 

The pipeline employs a **tracking-by-detection** methodology:
1. **YOLO segmentation & PointPainting** project semantic scores onto LiDAR points.
2. **Frustum clustering & DBSCAN** estimate 3D bounding boxes.
3. **Non-Maximum Suppression (NMS)** eliminates duplicate detections.
4. **AB3DMOT (3D Kalman Filter + Hungarian matching)** generates temporal tracking.

Optimizing these parameters requires balancing sensor noise, point density, and physical dynamics.

---

## 1. Upstream 2D & 3D Detection Parameters

These parameters define how the 3D point cloud is filtered, clustered, and shaped into bounding boxes. They reside in the [FrustumDetector](file:///workspace/frustum_detection/frustum_detector.py#L117) class.

### 2D Object Detection
* **`conf_thr`** *(ROS Parameter: `conf_thr` | Default: `0.40`)*:
  * **Description:** The confidence threshold required for a YOLO 2D instance segmentation mask to be processed.
  * **Impact:** 
    * *Too low:* Introduces false detections, leading to ghost/spurious tracks.
    * *Too high:* Misses far away or partially occluded objects, causing tracking drops.

### PointPainting Segmentation Filtering
* **PointPainting Score Threshold** *(Defined in [frustum_detector.py:L292](file:///workspace/frustum_detection/frustum_detector.py#L292) | Default: `0.25`)*:
  * **Description:** Within a projected frustum, only LiDAR points with a class score greater than `0.25` are kept for box estimation.
  * **Impact:** Discards background points (like buildings or trees) that fall inside a 2D bounding box projection. Raising this makes boxes tighter to the actual object but requires strong YOLO confidence.

### Frustum Depth-Window Gating
* **`depth_max`** and **`depth_win`** *(Defined in [_COCO_CFG](file:///workspace/frustum_detection/frustum_detector.py#L147-L154) | Varies by class)*:
  * **Description:** Determines the maximum depth range (`depth_max`) and the search window depth from the nearest point (`depth_win`) inside a frustum.
  * **Defaults:**
    * **Pedestrian / Cyclist / Motorcycle:** Max depth `40.0`m, Window `3.0`m
    * **Car:** Max depth `80.0`m, Window `6.0`m
    * **Bus:** Max depth `80.0`m, Window `10.0`m
    * **Truck:** Max depth `80.0`m, Window `8.0`m
  * **Impact:** Discards "background splash"—points that project onto the 2D mask but are located far behind the physical object.

### DBSCAN Clustering
* **`eps_m` (Epsilon)** *(Defined in [_COCO_CFG](file:///workspace/frustum_detection/frustum_detector.py#L147-L154) | Varies by class)*:
  * **Description:** The search radius in meters used by DBSCAN to cluster points.
  * **Defaults:**
    * **Pedestrian / Cyclist / Motorcycle:** `1.5`m
    * **Car:** `2.5`m
    * **Bus / Truck:** `3.0`m
  * **Impact:** 
    * *Too low:* Splinters a single vehicle into multiple small clusters, leading to multiple separate boxes/tracks.
    * *Too high:* Merges neighboring objects (e.g., two cars parked side-by-side) into a single large box/track.
* **`min_pts`** *(ROS Parameter: `min_pts` | Default: `4`)*:
  * **Description:** The minimum number of LiDAR points required to form a cluster and produce a 3D box.
  * **Impact:** Prevents small clusters of noise from generating fake boxes. Raise this if you see ghost boxes in dusty conditions or heavy sensor noise.

### Bounding Box Size Constraints
To prevent individual stray points from inflating a box, dimensions are clamped to class-specific minima and maxima:
* **`_DIM_MAX`** *(Maximum dx, dy, dz limits)*:
  * **Pedestrian:** `1.0`m × `1.0`m × `2.2`m
  * **Cyclist:** `2.0`m × `1.2`m × `2.0`m
  * **Car:** `6.0`m × `3.0`m × `2.5`m
* **`_DIM_MIN`** *(Minimum dx, dy, dz requirements)*:
  * **Pedestrian:** `0.15`m × `0.15`m × `0.4`m
  * **Cyclist:** `0.2`m × `0.2`m × `0.4`m
  * **Car:** `0.8`m × `0.6`m × `0.4`m

---

## 2. Non-Maximum Suppression (NMS) Parameters

NMS suppresses overlapping detections before they are sent to the tracker. It is implemented in [nms_3d](file:///workspace/frustum_detection/frustum_detector.py#L360-L385).

* **`nms_dist`** *(ROS Parameter: `nms_dist` | Default: `1.0`m)*:
  * **Description:** The threshold distance between box centers to trigger overlap suppression.
* **`_CLS_BASE` (Base Suppression Radii)** *(Defined in [nms_3d](file:///workspace/frustum_detection/frustum_detector.py#L368) | Varies by class)*:
  * **Pedestrian / Cyclist:** `1.5`m
  * **Car:** `2.5`m
  * **Impact:** If two YOLO masks trigger on the same physical object, NMS suppresses the lower-confidence box if its center falls within `max(base_radius, box_radius)` of the higher-confidence box.

---

## 3. Tracker Parameters (`AB3DMOT`)

These parameters dictate how tracks are initialized, matched, predicted, and deleted. They reside in the [AB3DMOT](file:///workspace/frustum_detection/frustum_detector.py#L487) class.

* **`iou_threshold`** *(Hardcoded parameter in `FrustumNode` | Default: `0.25`)*:
  * **Description:** The threshold for 2D Bird's Eye View (BEV) Intersection-over-Union (IoU) to associate an existing track with a new detection.
  * **Impact:** 
    * *Too low:* Encourages mismatched/incorrect associations (e.g. ID switches between adjacent cars).
    * *Too high:* Causes the tracker to ignore valid associations when objects accelerate or turn sharply, generating a new track ID instead.
* **`max_age`** *(ROS Parameter: `max_age` | Default: `3` frames)*:
  * **Description:** The maximum number of consecutive frames a track can go unmatched before being deleted.
  * **Impact:** Helps maintain track continuity through temporary occlusion. Increasing this keeps tracks alive longer but can cause "ghost tracks" to linger after an object has left the field of view.
* **`min_hits`** *(ROS Parameter: `min_hits` | Default: `3` frames)*:
  * **Description:** The number of consecutive frame updates a track must receive to be confirmed and published.
  * **Impact:** Filters out brief, spurious detections. A lower value confirms tracks faster but increases false positives.
* **Gating Distance Fallback** *(Implemented in [_assoc_cost](file:///workspace/frustum_detection/frustum_detector.py#L476-L485))*:
  * **Description:** If a track cannot achieve the `iou_threshold` (e.g., due to coordinate mismatches or rapid motion), it checks if the Euclidean center-distance is smaller than the detection's footprint radius: `gate = (length + width) / 2`. If so, they are matched with a fallback cost.
* **Kalman Filter Covariances (`_Q` and `_R`)** *(Defined in [KalmanBox3D](file:///workspace/frustum_detection/frustum_detector.py#L391-L432))*:
  * **`Q` (Process Noise Covariance):** Represents the uncertainty in the constant-velocity motion model. High values mean the object's velocity and direction changes rapidly (more trust in new detections).
  * **`R` (Measurement Noise Covariance):** Represents the uncertainty of the 3D detector box outputs. High values mean the bounding boxes fluctuate frame-to-frame (more trust in Kalman predictions).

---

## 4. Parameter Tuning Guidelines

| If you observe... | Possible Cause | Tuning Actions |
|---|---|---|
| **Track IDs switch frequently on the same vehicle** | Low IoU match scores due to fast motion or poor frame sync. | 1. Decrease `iou_threshold` to `0.20` or `0.15`. <br>2. Check camera-to-LiDAR timestamp synchronization offsets. |
| **Track drops and restarts when passing behind poles/vehicles** | Track deleted too quickly. | 1. Increase `max_age` from `3` to `5` or `7` frames. |
| **A delay in detecting new vehicles** | High confirmation latency. | 1. Lower `min_hits` to `2`. *Warning: may increase false positives.* |
| **"Ghost tracks" that linger after a vehicle has turned/left** | Tracker holding onto dead tracks. | 1. Lower `max_age` to `2` or `3`. <br>2. Check if the Kalman Filter process noise `Q` is too low. |
| **Large, oversized bounding boxes** | Noise/clutter included in DBSCAN clustering. | 1. Decrease `eps_m` (e.g., Car eps from `2.5` to `2.0` or `1.8`). <br>2. Increase PointPainting threshold from `0.25` to `0.30` or `0.35` to ignore poorly-painted points. |
| **A single vehicle split into multiple boxes** | DBSCAN cluster search radius is too small. | 1. Increase `eps_m` for the target class. <br>2. Ensure NMS distance threshold `nms_dist` matches the physical footprint. |
