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
VALIDATOR_TIMEOUT_CLI=""

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
    --image)
      IMAGE="$2"; shift 2;;
    --timeout)
      VALIDATOR_TIMEOUT_CLI="$2"; shift 2;;
    -h|--help)
      echo "Usage: $0 [--host-kill] [--device <vendor:product>] [--image <name>] [--timeout <seconds>] [container-name]"; exit 0 ;;
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
  # Try to execute the packaged manual test inside the provided container.
  # Be CI-friendly: only add -i/-t when stdin/stdout are TTYs.
  DOCKER_EXEC_ARGS=(--workdir /home/user/ros_ws)
  if [ -t 0 ]; then DOCKER_EXEC_ARGS+=(-i); fi
  if [ -t 1 ]; then DOCKER_EXEC_ARGS+=(-t); fi
  docker exec "${DOCKER_EXEC_ARGS[@]}" "$CONTAINER_NAME" bash -lc "chmod +x /home/user/manual_realsense_test.sh || true; /home/user/manual_realsense_test.sh && mv /home/user/ros_ws/realsense_test_outputs/wrapper/* /home/user/ros_ws/logs/ 2>/dev/null || true"
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
# pyrealsense checks. Allow SKIP_PREFLIGHT=1 to bypass for interactive/debug use.
SKIP_PREFLIGHT=${SKIP_PREFLIGHT:-0}
VERBOSE=${VERBOSE:-0}

if [ "$SKIP_PREFLIGHT" = "1" ]; then
  echo "SKIP_PREFLIGHT=1 -> skipping image preflight checks"
else
  # Build the test command: source profile.d if present, else try any /opt/ros/*/setup.bash,
  # then check ros2. Also perform a pyrealsense2 import test but treat it as a warning only.
  PRE_CMD='
    if [ -f /etc/profile.d/ros.sh ]; then
      source /etc/profile.d/ros.sh 2>/dev/null || true;
    else
      for s in /opt/ros/*/setup.bash; do
        [ -f "$s" ] && source "$s" 2>/dev/null && break || true;
      done;
    fi;
    if command -v ros2 >/dev/null 2>&1; then
      echo ros2_ok
    else
      echo ros2_missing >&2; exit 2;
    fi;
    # warn-only pyrealsense2 check (print a warning but do not fail preflight)
    python3 - <<PY || true
try:
    import pyrealsense2 as rs
    print('pyrealsense2_ok')
except Exception:
    print('warning: pyrealsense2 import failed (python binding may be missing)')
    # non-fatal for preflight
    pass
PY
  '

  if [ "$VERBOSE" = "1" ]; then
    # Show output for debugging
    if docker run --rm --entrypoint bash "$IMAGE" -lc "$PRE_CMD"; then
      :
    else
      echo "Preflight failed: 'ros2' not available in image $IMAGE. Ensure the image includes ROS or use an image with ROS installed." >&2
      echo "To bypass this check for interactive debugging, re-run with SKIP_PREFLIGHT=1" >&2
      exit 1
    fi
  else
    # Silent check for normal runs
    if docker run --rm --entrypoint bash "$IMAGE" -lc "$PRE_CMD" >/dev/null 2>&1; then
      :
    else
      echo "Preflight failed: 'ros2' not available in image $IMAGE. Ensure the image includes ROS or use an image with ROS installed." >&2
      echo "To debug, re-run with VERBOSE=1 to see preflight output or bypass with SKIP_PREFLIGHT=1" >&2
      exit 1
    fi
  fi
fi

docker run --rm --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -v "$(pwd)/manual_realsense_test.sh":/home/user/manual_realsense_test.sh:ro \
  -v "$(pwd)":/home/user/workspace:ro \
  -v "$HOST_LOG_DIR":/home/user/ros_ws/logs:rw \
  -e SKIP_CLAIMERS="$SKIP_CLAIMERS" \
  ${VALIDATOR_TIMEOUT_CLI:+-e VALIDATOR_TIMEOUT="$VALIDATOR_TIMEOUT_CLI"} \
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

# Convenience: tell the user where the JSON/logs landed and which symlinks to check
echo "Artifacts directory: $OUT_DIR"
if [ -e "$OUT_DIR/validate_realsense_plus.json" ] || [ -e "$OUT_DIR/validate_realsense_ros.json" ]; then
  echo "Prefer these for quick inspection:"
  [ -e "$OUT_DIR/validate_realsense_plus.json" ] && echo "  JSON: $OUT_DIR/validate_realsense_plus.json" || true
  [ -e "$OUT_DIR/validate_realsense_ros.json" ] && echo "  JSON: $OUT_DIR/validate_realsense_ros.json" || true
  [ -e "$OUT_DIR/validate_realsense_plus.log" ] && echo "  LOG : $OUT_DIR/validate_realsense_plus.log" || true
  [ -e "$OUT_DIR/validate_realsense_ros.log" ] && echo "  LOG : $OUT_DIR/validate_realsense_ros.log" || true
else
  echo "No validator artifacts copied into wrapper dir (check $OUT_FILE for details)."
fi
echo "Wrapper log: $OUT_DIR/wrapper.log"
echo "Run timestamp: $OUT_DIR/time.log"
