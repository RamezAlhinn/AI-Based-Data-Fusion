# Multi-arch base image: works on Apple Silicon (arm64) and WSL2/x86 (amd64).
# The official ROS image already includes ROS 2 Humble, Ubuntu 22.04, and the
# apt repository — no manual repo setup needed.
FROM ros:humble-ros-base

# ── System prerequisites ───────────────────────────────────────────────────────
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    ca-certificates \
    software-properties-common \
 && locale-gen en_US en_US.UTF-8 \
 && update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 \
 && rm -rf /var/lib/apt/lists/*

ENV LANG=en_US.UTF-8

# ── Install ROS 2 Humble desktop + tooling ────────────────────────────────────
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    ros-humble-vision-msgs \
    ros-humble-rviz2 \
    ros-humble-tf2-ros \
    ros-humble-message-filters \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
 && rosdep init || true \
 && rosdep update \
 && rm -rf /var/lib/apt/lists/*

# ── Python packages ────────────────────────────────────────────────────────────
# Install PyTorch CPU-only wheel first to keep image size down, then
# install the rest of the Python dependencies.
RUN pip3 install --no-cache-dir \
    torch torchvision torchaudio \
    --index-url https://download.pytorch.org/whl/cpu

RUN pip3 install --no-cache-dir \
    numpy \
    scipy \
    ultralytics \
    open3d

# ── Environment ────────────────────────────────────────────────────────────────
ENV ROS_DOMAIN_ID=42

# Source ROS on every interactive shell.
# The workspace setup is guarded so it does not fail on a fresh clone.
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
 && echo "source /workspace/ros2_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc

WORKDIR /workspace
