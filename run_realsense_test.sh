#!/usr/bin/env bash
# run_realsense_test.sh
# Convenience wrapper to run the manual_realsense_test.sh inside the realsense_ros image
# with USB access and the host `video` group, capturing full output to a host file.

set -euo pipefail
# Allow overriding the image used to run the container (useful for CI/custom builds)
echo "---- End preview ----"
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
# Delegate to consolidated runner. Preserve IMAGE and HOST_LOG_DIR env vars.
HOST_LOG_DIR=${HOST_LOG_DIR:-"$(pwd)/realsense_test_outputs"} IMAGE=${IMAGE:-realsense_ros:debug} bash "$SCRIPT_DIR/scripts/runner.sh"
