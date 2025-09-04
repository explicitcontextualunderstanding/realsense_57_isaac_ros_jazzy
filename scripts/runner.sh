#!/usr/bin/env bash
set -euo pipefail
# scripts/runner.sh
# Consolidated runner to start the realsense smoke test either by
#  - creating an ephemeral container and running the packaged manual test
#  - executing the manual test inside an existing container (exec mode)
# The script preserves the previous behavior of saving wrapper logs into
# HOST_LOG_DIR/wrapper and copying validator/time logs into the wrapper dir.

IMAGE=${IMAGE:-realsense_ros:debug}
HOST_LOG_DIR=${HOST_LOG_DIR:-"$(pwd)/realsense_test_outputs"}
# Flags: --host-kill will run the host-side claimant cleaner before launching the ephemeral
# container. This is a host-only action and is skipped when running in exec-mode
# against an existing container.
HOST_KILL=0
SKIP_CLAIMERS=${SKIP_CLAIMERS:-1}

# Parse optional flags. Any remaining positional argument is treated as the
# container name (exec mode). Backwards compatible: previous callers that pass
# a single positional container name continue to work.
POSITIONAL=()
while [[ $# -gt 0 ]]; do
  case "$1" in
    --host-kill)
      HOST_KILL=1; shift ;;
    --device)
      DEVICE_ID="$2"; shift 2;;
    -h|--help)
      echo "Usage: $0 [--host-kill] [container-name]"; exit 0 ;;
    --)
      shift; break ;;
    -*)
      echo "Unknown option: $1" >&2; exit 2 ;;
    *)
      POSITIONAL+=("$1"); shift ;;
  esac
done
set -- "${POSITIONAL[@]:-}"
CONTAINER_NAME=${1:-}
DEVICE_ID=${DEVICE_ID:-8086:0b07}

mkdir -p "$HOST_LOG_DIR"
# Per-run subfolder to avoid collisions and make scraping simpler
TS=$(date +%Y%m%d_%H%M%S)
OUT_DIR="$HOST_LOG_DIR/wrapper/$TS"
mkdir -p "$OUT_DIR"
# Single wrapper log file per run (keeps filenames stable inside the run folder)
OUT_FILE="$OUT_DIR/wrapper.log"

VIDEO_GID=$(getent group video | cut -d: -f3 || true)
if [ -z "$VIDEO_GID" ]; then
  echo "Could not determine video group GID on host. Ensure 'video' group exists." >&2
  exit 1
fi

if [ -n "$CONTAINER_NAME" ]; then
  echo "Running manual test inside existing container: $CONTAINER_NAME"
  # Try to execute the packaged manual test inside the provided container
  docker exec -it --workdir /home/user/ros_ws "$CONTAINER_NAME" bash -lc "chmod +x /home/user/manual_realsense_test.sh || true; /home/user/manual_realsense_test.sh && mv /home/user/ros_ws/realsense_test_outputs/wrapper/* /home/user/ros_ws/logs/ 2>/dev/null || true"
  rc=$?
  echo "Finished (exit code $rc). If the container moved logs into a host-mounted path they should appear in: $HOST_LOG_DIR"
  exit $rc
fi

echo "Running realsense smoke test (ephemeral container). Output will be saved to: $OUT_FILE"

# If requested, run the host-side claimant cleaner before launching the privileged
# ephemeral container. This is a host-only operation and will not be attempted
# when running in exec-mode against an existing container.
if [ "$HOST_KILL" -eq 1 ]; then
  HOST_HELPER="$(pwd)/scripts/host_kill_claimers.sh"
  if [ ! -x "$HOST_HELPER" ]; then
    echo "Host-kill requested but helper not found or not executable: $HOST_HELPER" >&2
    echo "Ensure the file exists and is executable, or run the helper manually." >&2
    exit 2
  fi
  echo "Running host claimant cleaner (this may require sudo): $HOST_HELPER --device $DEVICE_ID --yes --stop-containers"
  # Prefer sudo if available; allow the helper to run without sudo if the user
  # already has sufficient privileges (e.g. root shell).
  if command -v sudo >/dev/null 2>&1; then
    sudo "$HOST_HELPER" --device "$DEVICE_ID" --yes --stop-containers
  else
    "$HOST_HELPER" --device "$DEVICE_ID" --yes --stop-containers
  fi
  echo "Host claimant cleaner finished. Continuing to launch ephemeral container."
fi

# Preflight: for ephemeral runs, verify the selected image can run basic ros2 and
# pyrealsense2 checks. This catches images that lack the runtime pieces before
# starting a privileged container.
echo "Running image preflight checks against image: $IMAGE"
set +e
docker run --rm --entrypoint bash "$IMAGE" -lc 'if command -v ros2 >/dev/null 2>&1; then ros2 --version >/dev/null 2>&1 && echo ros2_ok || (echo ros2_missing >&2; exit 2); else echo ros2_missing >&2; exit 2; fi' >/dev/null 2>&1
RET_ROS=$?
docker run --rm --entrypoint bash "$IMAGE" -lc 'python3 - <<EOF
try:
  import pyrealsense2 as rs
  print("pyrealsense2_ok")
except Exception as e:
  print("pyrealsense2_missing", e)
  raise
EOF' >/dev/null 2>&1
RET_PYRS=$?
set -e
if [ $RET_ROS -ne 0 ]; then
  echo "Preflight failed: 'ros2' not available in image $IMAGE. Ensure the image includes ROS or use an image with ROS installed." >&2
  exit 3
fi
if [ $RET_PYRS -ne 0 ]; then
  echo "Preflight notice: 'pyrealsense2' import failed inside image $IMAGE. This may be expected on platforms without a wheel; set INSTALL_PYREALSENSE2=true at build time or ensure the runtime provides pyrealsense2." >&2
  # not a hard failure; continue but warn
fi

docker run --rm --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -v "$(pwd)/manual_realsense_test.sh":/home/user/manual_realsense_test.sh:ro \
  -v "$(pwd)":/home/user/workspace:ro \
  -v "$HOST_LOG_DIR":/home/user/ros_ws/logs:rw \
  -e SKIP_CLAIMERS="$SKIP_CLAIMERS" \
  --group-add "$VIDEO_GID" \
  "$IMAGE" /bin/bash -lc 'chmod +x /home/user/manual_realsense_test.sh || true; /home/user/manual_realsense_test.sh' > "$OUT_FILE" 2>&1

rc=$?

echo "Finished (exit code $rc). Log saved to: $OUT_FILE"

echo "---- Log preview (first 200 lines) ----"
head -n 200 "$OUT_FILE" || true
echo "---- End preview ----"

# Create a small time.log in the wrapper output dir to make it obvious when the wrapper ran
mkdir -p "$OUT_DIR/data" || true
echo "wrapper_run $(date -u +%Y-%m-%dT%H:%M:%SZ)" > "$OUT_DIR/data/time.log" || true

# Attempt to copy the latest validator logs and JSON results (prefer the newer "plus" validator)
LATEST_PLUS_LOG=$(ls -1t "$HOST_LOG_DIR"/validate_realsense_plus_*.log 2>/dev/null | head -n1 || true)
if [ -n "$LATEST_PLUS_LOG" ]; then
  cp -v "$LATEST_PLUS_LOG" "$OUT_DIR/" || true
  ln -sf "$(basename "$LATEST_PLUS_LOG")" "$OUT_DIR/validate_realsense_plus.log" || true
  LATEST_PLUS_JSON=$(ls -1t "$HOST_LOG_DIR"/validate_realsense_plus_*.json 2>/dev/null | head -n1 || true)
  if [ -n "$LATEST_PLUS_JSON" ]; then
    cp -v "$LATEST_PLUS_JSON" "$OUT_DIR/" || true
    ln -sf "$(basename "$LATEST_PLUS_JSON")" "$OUT_DIR/validate_realsense_plus.json" || true
  fi
else
  LATEST_VALIDATOR_LOG=$(ls -1t "$HOST_LOG_DIR"/validate_realsense_ros_*.log 2>/dev/null | head -n1 || true)
  if [ -n "$LATEST_VALIDATOR_LOG" ]; then
    cp -v "$LATEST_VALIDATOR_LOG" "$OUT_DIR/" || true
    ln -sf "$(basename "$LATEST_VALIDATOR_LOG")" "$OUT_DIR/validate_realsense_ros.log" || true
  fi
  LATEST_VALIDATOR_JSON=$(ls -1t "$HOST_LOG_DIR"/validate_realsense_ros_*.json 2>/dev/null | head -n1 || true)
  if [ -n "$LATEST_VALIDATOR_JSON" ]; then
    cp -v "$LATEST_VALIDATOR_JSON" "$OUT_DIR/" || true
    ln -sf "$(basename "$LATEST_VALIDATOR_JSON")" "$OUT_DIR/validate_realsense_ros.json" || true
  fi
fi

# Also, if there is a plus JSON even when plus log wasn't present, try copying it too
if [ -z "$LATEST_PLUS_LOG" ]; then
  ALT_PLUS_JSON=$(ls -1t "$HOST_LOG_DIR"/validate_realsense_plus_*.json 2>/dev/null | head -n1 || true)
  if [ -n "$ALT_PLUS_JSON" ]; then
    cp -v "$ALT_PLUS_JSON" "$OUT_DIR/" || true
    ln -sf "$(basename "$ALT_PLUS_JSON")" "$OUT_DIR/validate_realsense_plus.json" || true
  fi
fi

# copy the latest time_*.log from host logs into wrapper output dir and create latest symlink
LATEST_TIME=$(ls -1t "$HOST_LOG_DIR"/time_*.log 2>/dev/null | head -n1 || true)
if [ -n "$LATEST_TIME" ]; then
  cp -v "$LATEST_TIME" "$OUT_DIR/" || true
  ln -sf "$(basename "$LATEST_TIME")" "$OUT_DIR/time.log" || true
fi

exit $rc
