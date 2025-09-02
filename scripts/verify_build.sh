#!/usr/bin/env bash
set -euo pipefail

# verify_build.sh - quick verification script for the realsense_ros image/workspace
# Runs a small set of checks: rs-enumerate-devices, pyrealsense2 smoke test, starts the
# realsense2_camera node, runs the ROS validator (rs2_test.py), collects logs and exits
# non-zero on failure.

TIMEOUT=${1:-45}
SCRIPTS_DIR=${SCRIPTS_DIR:-/home/user/ros_ws/scripts}
LOGDIR=/tmp/verify_build_logs_$(date +%s)
mkdir -p "$LOGDIR"

echo "Verify build script starting"
echo "LOGDIR=$LOGDIR"

# Helper to timestamp
ts() { date +"%Y-%m-%d %H:%M:%S"; }

echo "$(ts) - LD_LIBRARY_PATH=${LD_LIBRARY_PATH:-}" | tee "$LOGDIR/summary.txt"

# 1) rs-enumerate-devices
echo "$(ts) - Running rs-enumerate-devices --verbose" | tee -a "$LOGDIR/summary.txt"
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  rs-enumerate-devices --verbose > "$LOGDIR/rs-enumerate.txt" 2>&1 || true
  echo "rs-enumerate-devices output saved to $LOGDIR/rs-enumerate.txt" | tee -a "$LOGDIR/summary.txt"
else
  echo "rs-enumerate-devices not found in PATH" | tee -a "$LOGDIR/summary.txt"
fi

# 2) Python SDK smoke test (pyrealsense2)
echo "$(ts) - Running pyrealsense2 smoke test (3 frames)" | tee -a "$LOGDIR/summary.txt"
python3 - <<PY > "$LOGDIR/pyrealsense2_smoke.txt" 2>&1 || true
import sys
try:
    import pyrealsense2 as rs
    print('pyrealsense2', rs.__version__)
    p = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, 848, 480, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
    print('Starting pipeline...')
    profile = p.start(cfg)
    for i in range(3):
        frames = p.wait_for_frames(5000)
        depth = frames.get_depth_frame()
        color = frames.get_color_frame()
        print('frame', i, 'depth_present=', depth is not None, 'color_present=', color is not None)
    p.stop()
    print('SDK smoke test OK')
except Exception as e:
    print('SDK smoke test ERROR:', e)
    sys.exit(2)
PY

if grep -q "SDK smoke test OK" "$LOGDIR/pyrealsense2_smoke.txt" 2>/dev/null; then
  echo "pyrealsense2 smoke test: PASS" | tee -a "$LOGDIR/summary.txt"
  SDK_OK=1
else
  echo "pyrealsense2 smoke test: FAIL (see $LOGDIR/pyrealsense2_smoke.txt)" | tee -a "$LOGDIR/summary.txt"
  SDK_OK=0
fi

# 3) Start realsense2_camera node (background) and collect logs
echo "$(ts) - Sourcing ROS/workspace and launching realsense2_camera node" | tee -a "$LOGDIR/summary.txt"
if [ -f /home/user/ros_ws/install/setup.bash ]; then
  # Some setup scripts reference variables that may be unset; temporarily
  # disable "nounset" so sourcing them doesn't abort this verifier.
  set +u || true
  # shellcheck source=/dev/null
  source /home/user/ros_ws/install/setup.bash || true
  set -u
else
  set +u || true
  # shellcheck source=/dev/null
  source /opt/ros/jazzy/setup.bash || true
  set -u
fi

NODE_LOG="$LOGDIR/realsense_node.log"
# Try to run the node; tolerates failure
ros2 run realsense2_camera realsense2_camera_node > "$NODE_LOG" 2>&1 &
RS_NODE_PID=$!
sleep 6

echo "$(ts) - ros2 node list:" > "$LOGDIR/ros2_nodes.txt" 2>&1 || true
ros2 node list >> "$LOGDIR/ros2_nodes.txt" 2>&1 || true

echo "$(ts) - camera topics:" > "$LOGDIR/topics.txt" 2>&1 || true
ros2 topic list | grep camera >> "$LOGDIR/topics.txt" 2>&1 || true

# 4) Run the ROS validator script (rs2_test.py expects to find validate_realsense_ros.py in the same dir)
echo "$(ts) - Running ROS validator (rs2_test.py) with timeout ${TIMEOUT}s" | tee -a "$LOGDIR/summary.txt"
if [ -x "$SCRIPTS_DIR/rs2_test.py" ] || [ -f "$SCRIPTS_DIR/rs2_test.py" ]; then
  python3 "$SCRIPTS_DIR/rs2_test.py" --mode ros --timeout "$TIMEOUT" > "$LOGDIR/ros_validator.log" 2>&1 || true
  echo "ROS validator output saved to $LOGDIR/ros_validator.log" | tee -a "$LOGDIR/summary.txt"
else
  echo "rs2_test.py not found at $SCRIPTS_DIR/rs2_test.py" | tee -a "$LOGDIR/summary.txt"
fi

# 5) Summarize results
echo "\n=== VERIFY BUILD SUMMARY ===" | tee -a "$LOGDIR/summary.txt"
if [ $SDK_OK -eq 1 ]; then
  echo "SDK smoke test: PASS" | tee -a "$LOGDIR/summary.txt"
else
  echo "SDK smoke test: FAIL" | tee -a "$LOGDIR/summary.txt"
fi

if grep -q "Validation SUCCE" "$LOGDIR/ros_validator.log" 2>/dev/null || grep -q 'ROS validator result: True' "$LOGDIR/ros_validator.log" 2>/dev/null; then
  echo "ROS validator: PASS" | tee -a "$LOGDIR/summary.txt"
  ROS_OK=1
else
  echo "ROS validator: FAIL (see $LOGDIR/ros_validator.log)" | tee -a "$LOGDIR/summary.txt"
  ROS_OK=0
fi

echo "Realsense node last 200 lines (saved to $NODE_LOG):" | tee -a "$LOGDIR/summary.txt"
tail -n 200 "$NODE_LOG" >> "$LOGDIR/summary.txt" 2>&1 || true

# Stop the node
echo "$(ts) - Stopping realsense node" | tee -a "$LOGDIR/summary.txt"
pkill -f realsense2_camera || true

# Print summary location and exit code
cat "$LOGDIR/summary.txt"

if [ "$SDK_OK" -eq 1 ] && [ "$ROS_OK" -eq 1 ]; then
  echo "ALL CHECKS PASSED"
  exit 0
else
  echo "ONE OR MORE CHECKS FAILED"
  exit 3
fi
