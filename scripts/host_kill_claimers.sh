#!/usr/bin/env bash
set -euo pipefail

# host_kill_claimers.sh
# Safely detect and optionally kill host-side processes or containers that are
# claiming a RealSense USB device. Designed to be run on the host (not inside
# the non-interactive test container). Default is dry-run; pass --yes to
# actually perform kills/stops.

DEVICE_ID_DEFAULT="8086:0b07"
DRY_RUN=1
DEVICE_ID="$DEVICE_ID_DEFAULT"
STOP_CONTAINERS=0

usage() {
  cat <<EOF
Usage: $0 [--device <vendor:product>] [--yes] [--stop-containers]

Options:
  --device <vendor:product>  USB id to target (default: $DEVICE_ID_DEFAULT)
  --yes                      Actually perform kills/stops; otherwise dry-run only
  --stop-containers          Also stop docker containers that have /dev/bus/usb mounted
  -h,--help                  Show this help

Examples:
  # Dry-run (show what would be killed)
  $0 --device 8086:0b07

  # Perform kills and stop containers
  sudo $0 --device 8086:0b07 --yes --stop-containers
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device) DEVICE_ID="$2"; shift 2;;
    --yes) DRY_RUN=0; shift 1;;
    --stop-containers) STOP_CONTAINERS=1; shift 1;;
    -h|--help) usage; exit 0;;
    *) echo "Unknown arg: $1" >&2; usage; exit 2;;
  esac
done

echo "Target device ID: $DEVICE_ID"
echo "Dry run: $([ "$DRY_RUN" -eq 1 ] && echo yes || echo NO)"
echo "Stop containers: $([ "$STOP_CONTAINERS" -eq 1 ] && echo yes || echo no)"

if ! command -v lsusb >/dev/null 2>&1; then
  echo "lsusb not found; please install usbutils or run equivalent checks" >&2
  exit 2
fi

# Collect matches from lsusb (lines like: Bus 002 Device 011: ID 8086:0b07 ...)
mapfile -t LSUSB_LINES < <(lsusb | grep -i "$DEVICE_ID" || true)
if [ ${#LSUSB_LINES[@]} -eq 0 ]; then
  echo "No USB device with id $DEVICE_ID found by lsusb." >&2
  echo "Output of lsusb (first 60 lines):"
  lsusb | sed -n '1,60p'
  exit 1
fi

echo "Found device lines:"; printf '%s\n' "${LSUSB_LINES[@]}"

declare -a DEVICE_PATHS=()
for line in "${LSUSB_LINES[@]}"; do
  # extract bus and device numbers
  BUS=$(echo "$line" | awk '{print $2}')
  DEV=$(echo "$line" | awk '{print $4}' | tr -d ':')
  # lsusb prints bus/device as zero-padded 3 digits, but node uses same formatting
  DEVICE_PATH="/dev/bus/usb/${BUS}/${DEV}"
  DEVICE_PATHS+=("$DEVICE_PATH")
done

echo "Checking device node paths:"; printf '%s\n' "${DEVICE_PATHS[@]}"

PIDS_TO_KILL=()

for devnode in "${DEVICE_PATHS[@]}"; do
  if [ ! -e "$devnode" ]; then
    echo "Warning: device node $devnode does not exist (may be transient)"
    continue
  fi
  echo "Processes holding $devnode:"
  if command -v lsof >/dev/null 2>&1; then
    # list processes and PIDs
    LSOF_OUT=$(lsof "$devnode" 2>/dev/null || true)
    if [ -n "$LSOF_OUT" ]; then
      echo "$LSOF_OUT" | sed -n '1,120p'
      mapfile -t PIDS < <(echo "$LSOF_OUT" | awk 'NR>1 {print $2}' | sort -u)
      for p in "${PIDS[@]}"; do
        PIDS_TO_KILL+=("$p")
      done
    else
      echo "  (no lsof holders)"
    fi
  else
    echo "lsof not available; try 'fuser -v $devnode' to inspect holders"
    FUSER_OUT=$(fuser -v "$devnode" 2>/dev/null || true)
    if [ -n "$FUSER_OUT" ]; then
      echo "$FUSER_OUT" | sed -n '1,120p'
      mapfile -t PIDS < <(echo "$FUSER_OUT" | awk 'NR>1 {print $2}' | tr -d 'PID' | tr -s ' ' | cut -d' ' -f2)
      for p in "${PIDS[@]}"; do
        PIDS_TO_KILL+=("$p")
      done
    fi
  fi
done

# Also include common realsense process names if found by pgrep
echo "Checking for well-known realsense processes via pgrep..."
mapfile -t NAME_PIDS < <(pgrep -af 'realsense2_camera|realsense-viewer|rs-enumerate-devices|rs-fw-update|realsense' 2>/dev/null | awk '{print $1}' || true)
for p in "${NAME_PIDS[@]}"; do
  PIDS_TO_KILL+=("$p")
done

# Deduplicate PIDs
if [ ${#PIDS_TO_KILL[@]} -gt 0 ]; then
  # unique
  IFS=$'\n' PIDS_TO_KILL=($(echo "${PIDS_TO_KILL[@]}" | tr ' ' '\n' | sort -u))
else
  echo "No PIDs detected that are holding the device or matching common realsense names."
fi

if [ ${#PIDS_TO_KILL[@]} -gt 0 ]; then
  echo "PIDs to consider killing: ${PIDS_TO_KILL[*]}"
  if [ "$DRY_RUN" -eq 1 ]; then
    echo "Dry-run mode: not killing. Re-run with --yes to perform kills."
  else
    echo "Killing PIDs (TERM then KILL if needed): ${PIDS_TO_KILL[*]}"
    sudo kill -TERM "${PIDS_TO_KILL[@]}" || true
    sleep 2
    # check which remain
    mapfile -t STILL_ALIVE < <(ps -o pid= -p "${PIDS_TO_KILL[@]}" 2>/dev/null || true)
    if [ ${#STILL_ALIVE[@]} -gt 0 ]; then
      echo "Some PIDs still alive, sending KILL: ${STILL_ALIVE[*]}"
      sudo kill -KILL "${STILL_ALIVE[@]}" || true
    fi
    echo "Kill attempts complete."
  fi
fi

if [ "$STOP_CONTAINERS" -eq 1 ]; then
  echo "Inspecting running docker containers for /dev/bus/usb mounts..."
  if command -v docker >/dev/null 2>&1; then
    mapfile -t DOCKER_IDS < <(docker ps -q || true)
    for cid in "${DOCKER_IDS[@]}"; do
      MOUNTS_JSON=$(docker inspect --format '{{json .Mounts}}' "$cid" 2>/dev/null || echo '')
      if echo "$MOUNTS_JSON" | grep -q '/dev/bus/usb'; then
        echo "Container $cid appears to have /dev/bus/usb mounted"
        if [ "$DRY_RUN" -eq 1 ]; then
          echo "  (dry-run) would stop container $cid"
        else
          echo "  stopping container $cid"
          docker stop "$cid" || true
        fi
      fi
    done
  else
    echo "docker not found; skipping container inspection"
  fi
fi

# Final verification: list any remaining holders
echo "Final check: processes holding device nodes after attempted kills:"
for devnode in "${DEVICE_PATHS[@]}"; do
  if [ -e "$devnode" ]; then
    if command -v lsof >/dev/null 2>&1; then
      lsof "$devnode" 2>/dev/null || echo "  none"
    else
      fuser -v "$devnode" 2>/dev/null || echo "  none"
    fi
  fi
done

echo "Done. If device still reports as in-use, try re-plugging the camera and re-running this script."
