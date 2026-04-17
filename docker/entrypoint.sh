#!/bin/bash

USERNAME="dev"

# Check if HOST_UID and HOST_GID are set
if [ -z "${HOST_UID}" ] || [ -z "${HOST_GID}" ]; then
    echo "ERROR: HOST_UID and HOST_GID environment variables must be set!"
    echo "Run: export HOST_UID=\$(id -u) HOST_GID=\$(id -g)"
    exit 1
fi

USERID="${HOST_UID}"
GROUPID="${HOST_GID}"

# Create group
if ! getent group "${GROUPID}" > /dev/null; then
    groupadd -g "${GROUPID}" "${USERNAME}"
fi

# Create user
if ! id -u "${USERNAME}" > /dev/null 2>&1; then
    useradd -l -u "${USERID}" -g "${GROUPID}" -d /home/"${USERNAME}" -s /bin/bash "${USERNAME}"
fi

# Setup home directory
mkdir -p /home/"${USERNAME}"
chown "${USERID}":"${GROUPID}" /home/"${USERNAME}"

# Give user sudo permissions for ros2 daemon
echo "${USERNAME} ALL=(ALL) NOPASSWD: /usr/bin/pkill -9 ros2_daemon" >> /etc/sudoers.d/"${USERNAME}" 2>/dev/null || true

source /opt/ros/humble/setup.bash

# Build ROS2 packages
if [ -d "/workspace/ros2_ws" ]; then
    echo "Building ROS2 packages..."
    cd /workspace/ros2_ws && colcon build --symlink-install
    chown -R "${USERID}":"${GROUPID}" /workspace/ros2_ws/build /workspace/ros2_ws/install /workspace/ros2_ws/log 2>/dev/null || true
fi

# Source the built packages if they exist
if [ -f "/workspace/ros2_ws/install/setup.bash" ]; then
    source /workspace/ros2_ws/install/setup.bash
fi

# Kill stale daemon and start fresh
ros2 daemon stop >/dev/null 2>&1
ros2 daemon start >/dev/null 2>&1

# Execute as the non-root user
if [ "$#" -gt 0 ]; then
    exec su - "${USERNAME}" -c "$(printf '%q ' "$@")"
else
    exec su - "${USERNAME}"
fi
