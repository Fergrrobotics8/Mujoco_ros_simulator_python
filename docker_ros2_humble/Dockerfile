FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-rviz2 \
    ros-humble-rqt-image-view \
    ros-humble-rqt \
    ros-humble-rosidl-generator-dds-idl \
    ros-humble-rosidl-default-generators \
    build-essential \
    libgl1-mesa-glx \
    libegl1-mesa-dev \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install --no-cache-dir mujoco numpy opencv-python scipy pandas pyarrow

WORKDIR /workspace

# Build ROS2 packages on first container startup
RUN /bin/bash -c 'source /opt/ros/humble/setup.bash && colcon build --symlink-install'

# Auto-source ROS 2 and kill any stale daemon on shell entry.
# (Needed because network_mode:host shares localhost with the host,
#  so a Jazzy daemon on the host would confuse Humble's ros2 CLI.)
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'source /workspace/install/setup.bash' >> /root/.bashrc && \
    echo 'ros2 daemon stop >/dev/null 2>&1; ros2 daemon start >/dev/null 2>&1' >> /root/.bashrc

CMD ["bash"]
