#!/usr/bin/env bash
set -euo pipefail

# validate_runner.sh
# Small wrapper to run the Python validators in a canonical way so callers
# don't duplicate output path and logging logic.

USAGE="Usage: $0 --validator (plus|ros) --logdir <dir> [--timeout N] [--out <json>]"

VALIDATOR="plus"
LOGDIR="/tmp"
TIMEOUT=15
OUTFILE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --validator) VALIDATOR="$2"; shift 2;;
    --logdir) LOGDIR="$2"; shift 2;;
    --timeout) TIMEOUT="$2"; shift 2;;
    --out) OUTFILE="$2"; shift 2;;
    -h|--help) echo "$USAGE"; exit 0;;
    *) echo "Unknown arg: $1"; echo "$USAGE"; exit 2;;
  esac
done

mkdir -p "$LOGDIR"
TS=$(date -u +%Y%m%d_%H%M%S)
if [ -z "$OUTFILE" ]; then
  OUTFILE="$LOGDIR/validate_realsense_${VALIDATOR}_${TS}.json"
fi
LOGFILE="$LOGDIR/validate_realsense_${VALIDATOR}_${TS}.log"

echo "Running validator '$VALIDATOR' -> json:$OUTFILE log:$LOGFILE timeout:$TIMEOUT"

# Optional grayscale/TF validation knobs (useful for VSLAM readiness). Defaults are safe for D435 (no IMU):
VALIDATE_TF=${VALIDATE_TF:-1}
REQUIRE_GRAY=${REQUIRE_GRAY:-1}
GRAYSCALE_TOPIC=${GRAYSCALE_TOPIC:-/camera/camera/infra1/image_rect_raw}

EXTRA_ARGS=(--timeout "$TIMEOUT" --out-file "$OUTFILE")
if [ "$VALIDATE_TF" = "1" ]; then EXTRA_ARGS+=(--check-tf); fi
if [ "$REQUIRE_GRAY" = "1" ]; then EXTRA_ARGS+=(--require-gray); fi
if [ -n "$GRAYSCALE_TOPIC" ]; then EXTRA_ARGS+=(--grayscale "$GRAYSCALE_TOPIC"); fi

# Prefer new validators/ location; fall back to scripts/ for backward compatibility
VALIDATOR_DIR_PRIMARY=/home/user/workspace/validators
VALIDATOR_DIR_FALLBACK=/home/user/workspace/scripts

if [ "$VALIDATOR" = "plus" ]; then
  if [ -x "$VALIDATOR_DIR_PRIMARY/validate_realsense_plus.py" ] || [ -f "$VALIDATOR_DIR_PRIMARY/validate_realsense_plus.py" ]; then
    python3 "$VALIDATOR_DIR_PRIMARY/validate_realsense_plus.py" "${EXTRA_ARGS[@]}" > "$LOGFILE" 2>&1 || true
  else
    python3 "$VALIDATOR_DIR_FALLBACK/validate_realsense_plus.py" "${EXTRA_ARGS[@]}" > "$LOGFILE" 2>&1 || true
  fi
elif [ "$VALIDATOR" = "ros" ]; then
  if [ -x "$VALIDATOR_DIR_PRIMARY/validate_realsense_ros.py" ] || [ -f "$VALIDATOR_DIR_PRIMARY/validate_realsense_ros.py" ]; then
    python3 "$VALIDATOR_DIR_PRIMARY/validate_realsense_ros.py" --timeout "$TIMEOUT" --out-file "$OUTFILE" > "$LOGFILE" 2>&1 || true
  else
    python3 "$VALIDATOR_DIR_FALLBACK/validate_realsense_ros.py" --timeout "$TIMEOUT" --out-file "$OUTFILE" > "$LOGFILE" 2>&1 || true
  fi
else
  echo "Unknown validator: $VALIDATOR" >&2
  exit 2
fi

echo "Validator finished. JSON: $OUTFILE Log: $LOGFILE"
echo "$OUTFILE"
exit 0
