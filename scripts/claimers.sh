#!/usr/bin/env bash
set -euo pipefail

# claimers.sh
# Helper functions to detect and optionally stop host-side processes that may
# be claiming the RealSense device (used by host-side orchestration scripts).

detect_host_claimers() {
  # Return combined evidence: pgrep results + lsof/fuser outputs where available.
  local out
  out=$(pgrep -af 'realsense2_camera|realsense-viewer|rs-enumerate-devices|rs-fw-update|realsense' 2>/dev/null || true)

  if command -v lsof >/dev/null 2>&1; then
    out="$out\n$(lsof /dev/bus/usb 2>/dev/null || true)"
    out="$out\n$(lsof /dev/video* 2>/dev/null || true)"
  elif command -v fuser >/dev/null 2>&1; then
    out="$out\n$(fuser -v /dev/bus/usb 2>/dev/null || true)"
    out="$out\n$(fuser -v /dev/video* 2>/dev/null || true)"
  fi

  echo "$out" | sed '/^\s*$/d' || true
}

kill_host_claimers() {
  # Attempt to stop known claimer processes gracefully, then forcibly.
  # Accepts no args. Uses sudo when available for broader effect.
  # Honor SKIP_CLAIMERS env var when set by callers
  if [ "${SKIP_CLAIMERS:-0}" = "1" ]; then
    echo "SKIP_CLAIMERS=1 -> skipping kill_host_claimers"
    return 0
  fi

  # Prefer non-interactive sudo when available (sudo -n succeeds). If that
  # isn't available, fall back to pkill without sudo so we don't hang waiting
  # for a password in non-interactive containers.
  if command -v sudo >/dev/null 2>&1 && sudo -n true 2>/dev/null; then
    sudo pkill -f realsense2_camera || true
    sudo pkill -f realsense-viewer || true
    sudo pkill -f rs-enumerate-devices || true
    sudo pkill -f rs-fw-update || true
  else
    pkill -f realsense2_camera || true
    pkill -f realsense-viewer || true
    pkill -f rs-enumerate-devices || true
    pkill -f rs-fw-update || true
  fi

  # allow kernel to release device nodes
  sleep 1
}

ensure_device_free() {
  # Ensure there are no remaining claimers; return 0 if clear, 1 otherwise.
  local remaining
  remaining=$(detect_host_claimers || true)
  if [ -z "$(echo "$remaining" | sed '/^\s*$/d')" ]; then
    return 0
  else
    echo "$remaining"
    return 1
  fi
}

# Expose functions when sourced
return 0
