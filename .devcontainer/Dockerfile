# Multi-arch base image: works on Apple Silicon (arm64) and WSL2/x86 (amd64).
# The official ROS image already includes ROS 2 Humble, Ubuntu 22.04, and the
# apt repository — no manual repo setup needed.
FROM ros:humble-ros-base

# ── System prerequisites & CLI Tools ───────────────────────────────────────────
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    gnupg2 \
    lsb-release \
    locales \
    ca-certificates \
    software-properties-common \
    git \
    nano \
    htop \
    clang-format \
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
    "numpy<2.0" \
    scipy \
    ultralytics \
    open3d \
    pillow \
    matplotlib

# ── DeepLab segmentation model ─────────────────────────────────────────────────
RUN git clone https://github.com/jfzhang95/pytorch-deeplab-xception.git /opt/deeplab
ENV DEEPLAB_REPO=/opt/deeplab

# ── Environment & UX ───────────────────────────────────────────────────────────
ENV ROS_DOMAIN_ID=42
ENV TERM=xterm-256color

# Source ROS, configure bash prompt to show we are inside container, and add useful aliases
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo 'source /workspace/ros2_ws/install/setup.bash 2>/dev/null || true' >> /root/.bashrc \
 && echo 'export PS1="\[\e[32m\][DevContainer] \[\e[34m\]\w\[\e[m\] \$ "' >> /root/.bashrc \
 && echo 'alias cw="cd /workspace/ros2_ws && colcon build --symlink-install && source install/setup.bash"' >> /root/.bashrc \
 && echo 'alias cs="cd /workspace/ros2_ws/src"' >> /root/.bashrc

WORKDIR /workspace
