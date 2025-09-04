#!/usr/bin/env bash
set -euo pipefail

# Automated RealSense test runner
# - Ensures no host-side processes are claiming the camera (kills them by default)
# - Launches a privileged container with USB access
# - Starts the realsense2_camera_node inside the container
# - Waits for color+depth topics to appear
# - Runs `validate_realsense_plus.py` and writes JSON + log to the host-mounted logs
#
# Usage:
#   ./scripts/run_automated_realsense_test.sh [--image IMAGE] [--timeout N] [--no-kill]
#

IMAGE=${IMAGE:-realsense_ros:debug}
TIMEOUT=${TIMEOUT:-20}
HOST_LOG_DIR=$(pwd)/realsense_test_outputs
NO_KILL=0

while [[ ${#} -gt 0 ]]; do
  case "$1" in
    --image) IMAGE="$2"; shift 2;;
    --timeout) TIMEOUT="$2"; shift 2;;
    --no-kill) NO_KILL=1; shift;;
    -h|--help) echo "Usage: $0 [--image IMAGE] [--timeout N] [--no-kill]"; exit 0;;
    *) echo "Unknown arg: $1"; exit 2;;
  esac
done

mkdir -p "$HOST_LOG_DIR"

TS=$(date -u +%Y%m%d_%H%M%S)

echo "Automated RealSense test (ts=$TS)"
echo "Host log dir: $HOST_LOG_DIR"
echo "Image: $IMAGE  timeout: $TIMEOUT  no-kill: $NO_KILL"

# Source helper functions for detecting/killing host claimers using script dir
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/claimers.sh" || true

CLAIMERS=$(detect_host_claimers || true)
if [ -n "$CLAIMERS" ]; then
  echo "Found potential host-side RealSense processes:\n$CLAIMERS"
  if [ "$NO_KILL" -eq 0 ]; then
    echo "Attempting to stop them so the container can claim the device..."
    # Try to kill gracefully, then force
    kill_host_claimers || true
    echo "Re-checking for any remaining claimers..."
    detect_host_claimers || true
  else
    echo "--no-kill specified: leaving host processes running (may cause RS2_USB_STATUS_BUSY)"
  fi
else
  echo "No host-side RealSense claimers detected."
fi

# Determine video GID so we can add the group to the container if useful
VIDEO_GID=$(getent group video | cut -d: -f3 || true)

CONTAINER_CMD=$(cat <<'EOF'
set -euo pipefail
TS="${TS}"
LOGDIR=/home/user/ros_ws/logs
mkdir -p "$LOGDIR"
VALIDATOR_JSON="$LOGDIR/validate_realsense_plus_${TS}.json"
VALIDATOR_LOG="$LOGDIR/validate_realsense_plus_${TS}.log"
NODE_LOG="$LOGDIR/realsense_node_${TS}.log"

echo "Container: starting realsense node"
set +e
if command -v ros2 >/dev/null 2>&1 && ros2 --help 2>&1 | grep -q -E '\srun\b'; then
  ros2 run realsense2_camera realsense2_camera_node &> "$NODE_LOG" &
  RS_PID=$!
else
  echo "ros2 run not available, trying to find executable in install tree" > "$NODE_LOG"
  EXEC_PATH=$(find /home/user/ros_ws/install -type f -executable -name 'realsense2_camera_node' 2>/dev/null | head -n1 || true)
  if [ -n "$EXEC_PATH" ]; then
    "$EXEC_PATH" &> "$NODE_LOG" &
    RS_PID=$!
  else
    echo "Could not start realsense node inside container" >> "$NODE_LOG"
    RS_PID=0
  fi
fi
set -e

if [ "$RS_PID" -ne 0 ]; then
  echo "Started realsense node pid=$RS_PID" >> "$NODE_LOG"
  # give node time to initialize
  sleep 7
else
  echo "No realsense node started (RS_PID=0); validator may not see topics" >> "$NODE_LOG"
fi

echo "Waiting for /camera topics to appear (timeout ${TIMEOUT}s)"
SECONDS=0
COLOR_TOPIC="/camera/camera/color/image_raw"
DEPTH_TOPIC="/camera/camera/depth/image_rect_raw"
while [ $SECONDS -lt ${TIMEOUT} ]; do
  # Redirect ros2 stderr to /dev/null to avoid BrokenPipeError traces when grepping
  if ros2 topic list --include-hidden 2>/dev/null | grep -q "${COLOR_TOPIC}" && ros2 topic list --include-hidden 2>/dev/null | grep -q "${DEPTH_TOPIC}"; then
    echo "Found topics: ${COLOR_TOPIC} and ${DEPTH_TOPIC}" > /dev/stderr
    break
  fi
  sleep 1
done

if [ $SECONDS -ge ${TIMEOUT} ]; then
  echo "Timed out waiting for topics (waited ${TIMEOUT}s)" >> "$NODE_LOG"
fi

echo "Running extended validator (timeout ${TIMEOUT}s) -> $VALIDATOR_JSON"
# Prefer validators/ location inside the workspace; fall back to scripts/
if [ -f /home/user/workspace/validators/validate_realsense_plus.py ]; then
  python3 /home/user/workspace/validators/validate_realsense_plus.py --color ${COLOR_TOPIC} --depth ${DEPTH_TOPIC} --caminfo /camera/camera/color/camera_info --timeout ${TIMEOUT} --out-file "$VALIDATOR_JSON" > "$VALIDATOR_LOG" 2>&1 || true
else
  python3 /home/user/workspace/scripts/validate_realsense_plus.py --color ${COLOR_TOPIC} --depth ${DEPTH_TOPIC} --caminfo /camera/camera/color/camera_info --timeout ${TIMEOUT} --out-file "$VALIDATOR_JSON" > "$VALIDATOR_LOG" 2>&1 || true
fi

echo "Validator log saved to: $VALIDATOR_LOG"

# cleanup: stop the node if we started it
if [ "$RS_PID" -ne 0 ] && ps -p $RS_PID >/dev/null 2>&1; then
  echo "Killing realsense node pid=$RS_PID" >> "$NODE_LOG"
  kill $RS_PID || true
fi

echo "Container run finished"
EOF
)

# Substitute host-side values for placeholders (only TS and TIMEOUT)
# Replace the literal placeholders "${TS}" and "${TIMEOUT}" that are
# present in the single-quoted heredoc above with the host-side values.
CONTAINER_CMD="${CONTAINER_CMD//\$\{TS\}/$TS}"
CONTAINER_CMD="${CONTAINER_CMD//\$\{TIMEOUT\}/$TIMEOUT}"

DOCKER_RUN_CMD=(docker run --rm --privileged -v /dev/bus/usb:/dev/bus/usb -v "$(pwd)":/home/user/workspace:ro -v "$HOST_LOG_DIR":/home/user/ros_ws/logs:rw)
if [ -n "$VIDEO_GID" ]; then
  DOCKER_RUN_CMD+=(--group-add "$VIDEO_GID")
fi
DOCKER_RUN_CMD+=("$IMAGE" /bin/bash -lc "$CONTAINER_CMD")

echo "Running container and test (this may take up to ${TIMEOUT}+20s)"
"${DOCKER_RUN_CMD[@]}"

echo "Test completed. Logs are in: $HOST_LOG_DIR"
echo "Validator JSON (if produced): $(ls -1 "$HOST_LOG_DIR"/validate_realsense_plus_*.json 2>/dev/null | tail -n1 || true)"

exit 0
