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

# Copy entrypoint script
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# Auto-source ROS 2 setup in bashrc
RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc && \
    echo 'if [ -f /workspace/ros2_ws/install/setup.bash ]; then source /workspace/ros2_ws/install/setup.bash; fi' >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
