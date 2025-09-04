#!/bin/bash
# /etc/profile.d/ros.sh
# Conditionally source the workspace install or system ROS when a login shell starts.
if [ -f /home/user/ros_ws/install/setup.bash ]; then
  # prefer workspace install first
  source /home/user/ros_ws/install/setup.bash
elif [ -f /opt/ros/jazzy/setup.bash ]; then
  source /opt/ros/jazzy/setup.bash
fi

# Ensure librealsense libraries are on LD_LIBRARY_PATH for interactive shells
export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH:-}
