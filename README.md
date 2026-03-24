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
git clone https://github.com/<org>/AI-Based-Data-Fusion.git
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

## 💻 How the Environment Works

Once the container finishes building and connects, you are effectively working inside an isolated Ubuntu Linux machine with ROS 2 Humble pre-installed.

- **Live Code Syncing:** The files on your computer are "bind-mounted" into the container. Any code you write, save, or change in VS Code on your Mac/Windows machine is instantly available inside the container.
- **Integrated Terminal:** Open a new terminal in VS Code (`Ctrl + ~`). You will see you are operating as the `root` user inside `/workspace`. This is your ROS 2 environment!
- **Auto-Building:** Our configuration automatically builds the ROS 2 workspace (`colcon build`) every time the container starts, so your messages and nodes are always ready to go.

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