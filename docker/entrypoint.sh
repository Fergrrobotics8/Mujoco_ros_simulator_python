#!/bin/bash


source /opt/ros/humble/setup.bash

# Build ROS2 packages
if [ -d "/workspace/ros2_ws" ]; then
    echo "Building ROS2 packages..."
    cd /workspace/ros2_ws && colcon build --symlink-install
fi

# Source the built packages if they exist
if [ -f "/workspace/ros2_ws/install/setup.bash" ]; then
    source /workspace/ros2_ws/install/setup.bash
fi

# Kill stale daemon and start fresh
ros2 daemon stop >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1

exec "$@"
