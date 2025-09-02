#!/usr/bin/env bash
set -euo pipefail

# run_test_and_collect_logs.sh
# Helper to run the manual realsense smoke test inside the realsense_ros image
# and persist logs to ~/realsense_test_logs on the host.

# Usage: ./run_test_and_collect_logs.sh [container-name]
# If container-name is omitted a temporary container will be created and removed.

IMAGE=${IMAGE:-realsense_ros:debug}
CONTAINER_NAME=${1:-}
HOST_LOG_DIR=${HOST_LOG_DIR:-$HOME/realsense_test_logs}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

mkdir -p "$HOST_LOG_DIR"
VIDEO_GID=$(getent group video | cut -d: -f3 || true)

if [ -n "$CONTAINER_NAME" ]; then
  echo "Starting ephemeral run in existing container: $CONTAINER_NAME"
  docker exec -it --workdir /home/user/ros_ws "$CONTAINER_NAME" bash -lc "chmod +x /home/user/manual_realsense_test.sh || true; /home/user/manual_realsense_test.sh && mv /home/user/ros_ws/realsense_test_output_* /home/user/ros_ws/logs/ || true"
  echo "If logs were moved into /home/user/ros_ws/logs inside the container that host-mount points to, they should appear in $HOST_LOG_DIR"
  exit 0
fi

# Run a new ephemeral container and run the script inside it; logs will be saved to HOST_LOG_DIR
DOCKER_CMD=(docker run --rm --init -it --privileged -v /dev/bus/usb:/dev/bus/usb -v "$SCRIPT_DIR/manual_realsense_test.sh":/home/user/manual_realsense_test.sh:ro -v "$SCRIPT_DIR/run_realsense_test.sh":/home/user/run_realsense_test.sh:ro -v "$HOST_LOG_DIR":/home/user/ros_ws/logs:rw --workdir /home/user/ros_ws)
if [ -n "$VIDEO_GID" ]; then
  DOCKER_CMD+=(--group-add "$VIDEO_GID")
fi
DOCKER_CMD+=($IMAGE)

# Build final command string
CMD_STR='bash -lc "chmod +x /home/user/manual_realsense_test.sh || true; /home/user/manual_realsense_test.sh && mv /home/user/ros_ws/realsense_test_output_* /home/user/ros_ws/logs/ || true"'

# Run
echo "Running: ${DOCKER_CMD[*]} $CMD_STR"
# shellcheck disable=SC2086
${DOCKER_CMD[*]} $CMD_STR

echo "Done. Logs (if any) are in: $HOST_LOG_DIR"
ls -l "$HOST_LOG_DIR" || true
