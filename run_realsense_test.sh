#!/usr/bin/env bash
# run_realsense_test.sh
# Convenience wrapper to run the manual_realsense_test.sh inside the realsense_ros image
# with USB access and the host `video` group, capturing full output to a host file.

set -euo pipefail
OUT_FILE="$(pwd)/realsense_test_output_$(date +%Y%m%d_%H%M%S).log"

VIDEO_GID=$(getent group video | cut -d: -f3 || true)
if [ -z "$VIDEO_GID" ]; then
  echo "Could not determine video group GID on host. Ensure 'video' group exists." >&2
  exit 1
fi

echo "Running realsense test. Output will be saved to: $OUT_FILE"

docker run --rm --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -v "$(pwd)/manual_realsense_test.sh":/home/user/manual_realsense_test.sh:ro \
  --group-add "$VIDEO_GID" \
  realsense_ros:debug /bin/bash -lc 'chmod +x /home/user/manual_realsense_test.sh || true; /home/user/manual_realsense_test.sh' > "$OUT_FILE" 2>&1

rc=$?

echo "Finished (exit code $rc). Log saved to: $OUT_FILE"

# Print a short preview
echo "---- Log preview (first 200 lines) ----"
head -n 200 "$OUT_FILE" || true

echo "---- End preview ----"

# Exit with the same return code as the docker run
exit $rc
