# Quickstart — Running the Fused Perception Pipeline

This guide explains how to build, run, and visualize the **PointPainting + Frustum 3D Object Detection & Tracking** pipeline.

All ROS 2 commands must run **inside the Dev Container** (`><` → Reopen in Container). Open multiple terminal tabs in VS Code (`Ctrl+Shift+``) to run the components.

---

## Pipeline Overview

```
LiDAR Scan ──┐
             ▼
        [PaintingNode] ──► /painting/scored_cloud ──┐
             ▲                                      ▼
Camera ──────┴──────────► /blackfly_s/cam0/... ──► [FrustumNode] ──► /frustum/markers (3D Boxes)
                                                                 ──► /frustum/bev     (BEV panel)
```

1. **`PaintingNode`**: Receives LiDAR and camera feeds, runs YOLO instance segmentation, projects the point cloud onto the image, and publishes a point cloud enriched with semantic class scores.
2. **`FrustumNode`**: Subscribes to the camera feed and the scored point cloud. It uses the 2D YOLO bounding boxes to extract 3D point cloud frustums, clusters them using DBSCAN to estimate 3D bounding boxes, runs NMS, and tracks objects frame-to-frame using AB3DMOT.

---

## Step 0 — Host-Side DDS Performance Tuning (Once per Host Machine)

> [!TIP]
> **Automatic Tuning**: These network tuning parameters have been automated via the `"initializeCommand"` configuration in [.devcontainer/devcontainer.json](file:///workspace/.devcontainer/devcontainer.json). Every time you open or rebuild the Dev Container in VS Code, the commands are automatically run on your host system to keep the virtual machine settings up-to-date. The manual instructions below are kept for reference or verification.

To prevent ROS 2 from dropping large data packets (like point clouds and camera frames), you must tune the network buffer size limits on your host system:

### Linux (Ubuntu/Debian)
Run the following in your host terminal:
```bash
# Apply immediately
sudo sysctl -w net.core.rmem_max=2147483647
sudo sysctl -w net.ipv4.ipfrag_time=3
sudo sysctl -w net.ipv4.ipfrag_high_thresh=134217728

# Make persistent across reboots
sudo tee /etc/sysctl.d/10-cyclone-max.conf <<EOF
net.core.rmem_max=2147483647
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
```

### macOS (Docker Desktop)
Run the following command in your Mac terminal (this uses a temporary privileged container to configure the virtual machine running Docker):
```bash
docker run --rm --privileged --pid=host alpine nsenter -t 1 -m -u -i -n sysctl -w net.core.rmem_max=2147483647 net.ipv4.ipfrag_time=3 net.ipv4.ipfrag_high_thresh=134217728
```

### Windows (WSL2 / Docker Desktop)
Run the following in PowerShell on the host:
```powershell
wsl -d docker-desktop -u root sysctl -w net.core.rmem_max=2147483647
wsl -d docker-desktop -u root sysctl -w net.ipv4.ipfrag_time=3
wsl -d docker-desktop -u root sysctl -w net.ipv4.ipfrag_high_thresh=134217728
```
*(Alternatively, you can run the same `docker run --rm --privileged --pid=host alpine ...` command listed for macOS inside Command Prompt or PowerShell).*

---

## Step 1 — Build the Workspace (Once per container start)

Inside the container terminal:
```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Option A — Running via Helper Scripts (Recommended)

You can launch or stop the entire pipeline (including bag playback, Foxglove bridge, painting, and frustum nodes) automatically in the background using the provided helper scripts.

### 1. Start the Pipeline
```bash
bash /workspace/start_pipeline.sh
```
This runs all the nodes in the background, redirects output to `/workspace/log/*.log`, and waits ~12 seconds for the nodes to initialize.

### 2. Stop the Pipeline
To stop all background processes started by the pipeline:
```bash
bash /workspace/stop_pipeline.sh
```

### 3. Check Logs
To monitor the nodes in real time, you can tail the log files:
```bash
# Painting node logs
tail -f /workspace/log/painting_node_live.log

# Frustum detection node logs
tail -f /workspace/log/frustum_node_live.log
```

---

## Option B — Running Components Individually (Manual)

If you prefer to run each component in its own terminal to inspect standard output in real-time, open multiple terminal tabs in VS Code (`Ctrl+Shift+``) and run the following:

### Step 2 — Run the PointPainting Node (Terminal 1)

This node performs camera-LiDAR projection and YOLO segmentation to score the point cloud:

```bash
source /workspace/ros2_ws/install/setup.bash

ros2 run point_painting painting_node --ros-args \
  -p calib_file:=/workspace/calib.txt \
  -p checkpoint_path:=/workspace/models/yolo11m-seg.pt
```

*Note: If `/workspace/models/yolo11m-seg.pt` is not present, it will automatically fall back to downloading the lightweight default model.*

---

### Step 3 — Run the Frustum Detection Node (Terminal 2)

This node clusters the scored cloud, computes 3D bounding boxes, and tracks them:

```bash
source /workspace/ros2_ws/install/setup.bash

ros2 run frustum_detection frustum_node --ros-args \
  -p calib_file:=/workspace/calib.txt
```

---

### Step 4 — Play the ROS Bag (Terminal 3)

```bash
ros2 bag play /workspace/studentProject1/ --loop
```

---

## Step 5 — Visualize the Output

You can visualize the pipeline results using either **Foxglove Studio** (recommended for Web-based visualization) or **RViz2** (ROS native).

### Option A — Foxglove Studio

1. **Start Foxglove Bridge (Terminal 4):**
   ```bash
   ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=9090
   ```
2. **Open Foxglove:** Open [Foxglove Studio](https://app.foxglove.dev) in your browser or desktop app on your **Mac**.
3. **Connect:** Click **Open connection** → **Foxglove WebSocket** → `ws://localhost:9090`.
4. **Add Panels:**
   - **3D Panel**: Subscribe to `/painting/painted_cloud` (set **Color mode** to `RGB`) to see the point cloud painted by semantic classes.
   - **Image Panel**: Subscribe to `/frustum/bev` to view the camera overlay + Bird's Eye View detection panel.
   - **Image Panel**: Subscribe to `/painting/segmentation_overlay` to verify YOLO instance segmentation masks.
   - **Image Panel**: Subscribe to `/painting/points_overlay` to view raw points projected onto the camera.

---

### Option B — RViz2 (ROS Native Visualization)

> [!NOTE]  
> Since the Dev Container runs virtualized on macOS, launching GUI applications like RViz2 requires an X11 server (such as [XQuartz](https://www.xquartz.org/)) installed on your Mac, with display forwarding enabled.

1. **Launch RViz2:**
   ```bash
   source /workspace/ros2_ws/install/setup.bash
   rviz2
   ```
2. **Configure Global Options:**
   - Set **Fixed Frame** to `velodyne`.
3. **Visualize the Semantically Painted Cloud:**
   - Click **Add** (bottom left) → **By topic** → select `/painting/painted_cloud` -> `PointCloud2`.
   - In the display settings for this PointCloud2, change **Color Transformer** to `RGB8`. This displays points in class colors:
     - 🔴 **Red** = Pedestrians
     - 🟢 **Green** = Cars / Trucks
     - 🔵 **Blue** = Bicycles
     - 🟠 **Orange** = Motorcycles
4. **Visualize 3D Bounding Boxes & Tracks:**
   - Click **Add** → **By topic** → select `/frustum/markers` -> `MarkerArray`.
   - This displays wireframe 3D bounding boxes around tracked objects, color-coded by class.
5. **Visualize 2D Overlays & BEV Panels:**
   - Click **Add** → **By topic** → select `/frustum/bev` -> `Image`.
   - Click **Add** → **By topic** → select `/painting/segmentation_overlay` -> `Image`.

---

## Step 6 — Run the Offline Isolation Test (No ROS 2 needed)

To quickly verify the entire pipeline end-to-end on a single frame without running active ROS nodes or playing a bag, run the isolation test script. It extracts a frame from the bag, runs the projection, YOLO segmentation, point painting, frustum clustering, and tracking offline:

```bash
python3 /workspace/test_pipeline_isolation.py --seed 7
```
*(You can change `--seed <number>` to test different frames in the bag).*

### What is the `isolation_output/` directory?

The isolation test script outputs verification images to the `isolation_output/` directory at the root of the workspace. This directory is included in `.gitignore` so your local runs will never clutter the Git repository.

The output images generated are:
1. `01_raw_image.jpg`: The raw camera frame from the bag.
2. `02_yolo_mask.jpg`: YOLO class segmentation mask.
3. `03_overlay.jpg`: YOLO mask blended on top of the camera frame.
4. `04_lidar_projected.jpg`: LiDAR points projected onto the image plane, colored by class.
5. `05_painted_scores.jpg`: Heatmap representation of per-point painting scores.
6. `06_detections.jpg`: Camera and Bird's Eye View (BEV) panels showing `FrustumDetector` 3D boxes.
7. `07_tracked.jpg`: Camera and BEV panels showing tracked objects with tracking IDs.

If `07_tracked.jpg` shows correct bounding boxes around the vehicles/pedestrians, the pipeline is fully functional!

