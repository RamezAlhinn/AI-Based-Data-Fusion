# AI-Based Data Fusion

A ROS 2 Humble workspace for real-time multi-sensor perception: this project fuses camera, LiDAR, and radar data to produce tracked, classified 3-D objects, forming the perception backbone of an autonomous or assisted-driving system. The pipeline algorithm is still being finalised; this repository provides a clean, buildable foundation that the team can extend incrementally.

---

## Prerequisites

| Tool | Version / Notes |
|---|---|
| **Docker Desktop** | ≥ 4.x · Enable GPU access in Settings → Resources → GPUs |
| **VS Code** | Latest stable |
| **Dev Containers extension** | `ms-vscode-remote.remote-containers` |
| **NVIDIA drivers** (GPU host only) | 525+ recommended for CUDA 12 |

> **Apple Silicon / Windows WSL2** — the Dockerfile installs a CPU-only PyTorch wheel so the container runs without an NVIDIA GPU. Remove the `--gpus all` run-arg in `devcontainer.json` if your machine has no NVIDIA card.

---

## Getting Started

1. **Clone the repository**
   ```bash
   git clone https://github.com/<org>/AI-Based-Data-Fusion.git
   cd AI-Based-Data-Fusion
   ```

2. **Open in VS Code**
   ```bash
   code .
   ```

3. **How to Run: Open in Container**
   The project uses **VS Code Dev Containers**. This means you don't need to install ROS 2 on your host machine — it all runs inside a Docker container managed by VS Code.
   - When you open the folder, VS Code will ask: *"Folder contains a Dev Container configuration file. Reopen to folder to develop in a container."*
   - Click **Reopen in Container**.
   - **Alternative**: Click the green button in the bottom-left corner of VS Code (the "Remote" icon) and select **"Dev Containers: Reopen in Container"**.

### Did we create the container?
Yes and no! We created the **recipe** (`Dockerfile`) and the **connection settings** (`devcontainer.json`). 
- The first time you click "Reopen in Container", VS Code **builds** the Docker image (this takes ~10–15 mins).
- After that, it simply **starts** the existing container, which is near-instant.
- Your code is "mounted" from your computer into the container, so any changes you make in VS Code are saved locally.

### Manual Run (Terminal)
If you prefer to run it manually from the terminal (alternative to VS Code):
```bash
# Build the image first (only needed once)
docker build -t fusion-dev -f .devcontainer/Dockerfile .

# Run the container
docker run \
  --network=host \
  --ipc=host \
  --pid=host \
  --name "fusion-dev" \
  --gpus all \
  -it \
  -p 2200:22 \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v $(pwd):/workspace \
  fusion-dev \
  /bin/bash
```

Once the container starts, a terminal inside `/workspace` is ready to use. The workspace auto-builds via `colcon build` on every container start.

---

## Repository Structure

```
AI-Based-Data-Fusion/
├── .devcontainer/          # Docker + VS Code Dev Container configuration
├── .github/workflows/      # GitHub Actions CI (builds fusion_msgs on every PR)
├── ros2_ws/
│   └── src/
│       └── fusion_msgs/    # Custom ROS 2 message definitions (FusedObject, FusedObjectArray)
├── config/
│   ├── sensor_params.yaml  # Camera / LiDAR / radar hardware parameters
│   └── fusion_params.yaml  # Fusion algorithm and tracker tuning knobs
├── bags/                   # Placeholder for ROS 2 bag files (stored externally)
├── .env.example            # Environment variable template — copy to .env
└── README.md
```

---

## ROS 2 Workspace

The `ros2_ws/` workspace currently contains the **`fusion_msgs`** package only, which defines the shared message types used across the pipeline.

Detector and fusion nodes will be added to `ros2_ws/src/` once the pipeline algorithm is decided. The build system (`colcon`) and message interfaces are already in place so new packages can be dropped in without any infrastructure work.

### Building manually

```bash
cd /workspace/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Verifying the messages are available

```bash
ros2 interface show fusion_msgs/msg/FusedObject
ros2 interface show fusion_msgs/msg/FusedObjectArray
```