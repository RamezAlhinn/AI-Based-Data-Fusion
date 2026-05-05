# Quickstart — See the Pipeline Running

All commands run **inside the Dev Container** (`><` → Reopen in Container).  
Open four terminal tabs in VS Code (`Ctrl+Shift+\`` to split).

> **Visualisation:** RViz2 has no display inside the container. We use **Foxglove Studio** on your Mac instead, connected over a WebSocket bridge on port `8765`.

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

`painted` counts are real — each point genuinely projects onto the image.  
`class_ids` values are raw pixel intensities (not semantic labels) until the model is loaded.

### Mode B — Full pipeline with DeepLab segmentation

The `pytorch-deeplab-xception` repo is baked into the image at `/opt/deeplab` — no extra cloning needed.  
You only need the checkpoint file. Download `deeplab-resnet.pth.tar` and place it in your project root on your Mac; it will appear at `/workspace/deeplab-resnet.pth.tar` inside the container.

```bash
source /workspace/ros2_ws/install/setup.bash

ros2 run point_painting painting_node --ros-args \
  -p calib_file:=/workspace/calib.txt \
  -p deeplab_repo_path:=/opt/deeplab \
  -p checkpoint_path:=/workspace/deeplab-resnet.pth.tar
```

Expected output:
```
[INFO] Loaded calibration from: /workspace/calib.txt
[INFO] Segmentation model loaded from: /workspace/deeplab-resnet.pth.tar
[INFO] PaintingNode started, waiting for synced messages...
```

`class_ids` are now real semantic labels (0=background, 7=car, 15=person, ...).  
Note: DeepLab on CPU adds ~2–5 seconds per frame — the node processes every frame it receives.

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

You will see a line every ~100ms:
```
data: 'frame=1 painted=8243 skipped=4901'
data: 'frame=2 painted=8019 skipped=5125'
```

- **painted** = LiDAR points that projected onto the camera image
- **skipped** = points behind the camera or outside the frame

Roughly 60–70% painted is expected given the camera's ~45° horizontal field of view.

---

## Step 5 — Visualise in Foxglove Studio (Terminal 4)

RViz2 has no display inside the container. Start the Foxglove WebSocket bridge instead:

```bash
ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765
```

Expected output:
```
[foxglove_bridge]: WebSocket server listening on ws://0.0.0.0:8765
```

Then on your **Mac**:
1. Open [Foxglove Studio](https://app.foxglove.dev) (browser or desktop app)
2. Click **Open connection** → **Foxglove WebSocket**
3. Enter `ws://localhost:8765` and click **Open**

Inside Foxglove:
- Click **+** → **3D** panel → subscribe to `/velodyne/points_raw` to see the point cloud
- Click **+** → **Image** panel → subscribe to `/blackfly_s/cam0/image_rectified` to see the camera feed
- Click **+** → **Raw Messages** panel → subscribe to `/painting/debug` to watch painted/skipped counts

Port `8765` is automatically forwarded by the devcontainer — no extra configuration needed.

---

## Optional — Extract frames from the bag without ROS running

Useful for running DeepLab offline on real data:

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

Output:
```
output_data/frame_0000.jpg  ← camera frames
...
output_data/frame_0009.jpg
output_data/lidar_points.npy   ← (N, 4) float32: x, y, z, intensity
output_data/lidar_metadata.txt
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
