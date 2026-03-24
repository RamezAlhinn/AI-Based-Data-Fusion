FROM nvidia/cuda@sha256:dc24c05b29f220b7215a0443ce79cdccf6a67208ee2a478f7cdd79bebca9bb39

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

# ── Add ROS 2 Humble apt repository ───────────────────────────────────────────
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg \
 && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" \
     > /etc/apt/sources.list.d/ros2.list

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
