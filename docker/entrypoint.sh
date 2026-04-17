#!/bin/bash

# Create user if running as non-root
if [ "$(id -u)" != "0" ]; then
    USERNAME="dev"
    USERID=$(id -u)
    GROUPID=$(id -g)
    
    # Create group if it doesn't exist
    if ! getent group "${GROUPID}" > /dev/null; then
        groupadd -g "${GROUPID}" "${USERNAME}"
    fi
    
    # Create user if it doesn't exist
    if ! id -u "${USERNAME}" > /dev/null 2>&1; then
        useradd -l -u "${USERID}" -g "${GROUPID}" -d /home/"${USERNAME}" -s /bin/bash "${USERNAME}"
    fi
    
    # Setup home directory
    mkdir -p /home/"${USERNAME}"
    chown "${USERID}":"${GROUPID}" /home/"${USERNAME}"
fi

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
