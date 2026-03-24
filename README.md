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
git clone https://github.com/RamezAlhinn/AI-Based-Data-Fusion.git
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

## 📂 Repository Structure

```
AI-Based-Data-Fusion/
├── .devcontainer/          # Docker + VS Code Dev Container configuration
├── ros2_ws/
│   └── src/                # Your ROS 2 packages will go here!
├── .gitignore
└── README.md
```

---

## 🤖 ROS 2 Workspace

The `ros2_ws/` workspace is currently empty. This repository provides the clean infrastructure so you can start creating packages immediately without worrying about dependencies or environment setup.

### Creating your first package

Open the VS Code terminal (`Ctrl + ~`) and run:
```bash
cd /workspace/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_first_package
```

### Building the workspace

```bash
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```