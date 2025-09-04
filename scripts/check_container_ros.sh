#!/usr/bin/env bash
# check_container_ros.sh - smoke-check that a running container has ros2 and tf2_ros importable
# Usage: scripts/check_container_ros.sh [container]
set -euo pipefail

CONTAINER=${1:-realsense_debug}

echo "Checking ROS availability in container: ${CONTAINER}"

# ensure container exists and is running
if ! docker inspect --format='{{.State.Running}}' "${CONTAINER}" >/dev/null 2>&1; then
  echo "ERROR: container ${CONTAINER} not found or not running"
  exit 2
fi

# run a remote check: source the entrypoint so the ROS env is set, verify ros2 command
# exists, then try importing tf2_ros using the container's python3
docker exec "${CONTAINER}" bash -lc '
  set -e
  # source entrypoint (works whether workspace or system ROS is present)
  source /usr/local/bin/realsense_entrypoint.sh >/dev/null 2>&1 || true
  if ! command -v ros2 >/dev/null 2>&1; then
    echo "ERROR: ros2 not found in container PATH"
    exit 3
  fi
  echo "ros2 found: $(command -v ros2)"
  # attempt to import tf2_ros
  python3 -c "import tf2_ros; print('tf2_ros import OK')"
'

echo "OK: container ${CONTAINER} has ros2 and tf2_ros available"
exit 0
