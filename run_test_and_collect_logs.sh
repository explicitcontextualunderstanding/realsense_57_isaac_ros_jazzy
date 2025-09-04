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
  docker exec -it --workdir /home/user/ros_ws "$CONTAINER_NAME" bash -lc "chmod +x /home/user/manual_realsense_test.sh || true; /home/user/manual_realsense_test.sh && mv /home/user/ros_ws/realsense_test_outputs/wrapper/* /home/user/ros_ws/logs/ 2>/dev/null || true"
  echo "If logs were moved into /home/user/ros_ws/logs inside the container that host-mount points to, they should appear in $HOST_LOG_DIR"
  exit 0
fi

echo "Invoking consolidated scripts/runner.sh (ephemeral container mode). Logs will be in: $HOST_LOG_DIR"
cd "$SCRIPT_DIR" || exit 1
HOST_LOG_DIR="$HOST_LOG_DIR" IMAGE="$IMAGE" bash ./scripts/runner.sh

echo "Done. Logs (if any) are in: $HOST_LOG_DIR"
ls -l "$HOST_LOG_DIR" || true
