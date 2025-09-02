#!/bin/bash
# Lightweight entrypoint that sources the workspace setup if present, otherwise the system ROS
# then executes the provided command. This ensures `ros2` and workspace libs are available for
# any `docker exec` or `docker run` shell.

# Source the workspace install if present
if [ -f /home/user/ros_ws/install/setup.bash ]; then
  source /home/user/ros_ws/install/setup.bash
else
  # fall back to system ROS
  if [ -f /opt/ros/jazzy/setup.bash ]; then
    source /opt/ros/jazzy/setup.bash
  fi
fi

# Ensure librealsense is on the library path
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}

# Exec the incoming command
exec "$@"
