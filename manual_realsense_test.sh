#!/usr/bin/env bash
set -uo pipefail
  # Choose a log directory. Prefer a host-mounted logs directory (created by
  # `run_test_and_collect_logs.sh`) if available at either /home/user/ros_ws/logs
  # or $HOME/ros_ws/logs so logs are visible on the host. Fall back to /tmp.
  LOGDIR=/tmp
  if [ -d /home/user/ros_ws/logs ] && [ -w /home/user/ros_ws/logs ]; then
    LOGDIR=/home/user/ros_ws/logs
  elif [ -d "$HOME/ros_ws/logs" ] && [ -w "$HOME/ros_ws/logs" ]; then
    LOGDIR="$HOME/ros_ws/logs"
  fi
  # log file for the realsense node (declare before any attempts to redirect to it)
  LOG="$LOGDIR/realsense_node.log"
  # generate a stable timestamp for this run so logs are uniquely named
  TS=$(date -u +%Y%m%d_%H%M%S)
  # write per-run time file at the top-level LOGDIR so all run artifacts live together
  DATA_TIME_FILE="$LOGDIR/time_${TS}.log"
  mkdir -p "$(dirname "$DATA_TIME_FILE")" || true
  # write an initial start timestamp so callers can see when this test run began
  echo "start $(date -u +%Y-%m-%dT%H:%M:%SZ) $(hostname) $$" > "$DATA_TIME_FILE" || true
  # keep a 'latest' symlink for backwards compatibility (point to top-level time.log)
  ln -sf "$(basename "$DATA_TIME_FILE")" "$LOGDIR/time.log" || cp -f "$DATA_TIME_FILE" "$LOGDIR/time.log" || true
  mkdir -p "$(dirname "$LOG")" || true
  rm -f "$LOG"
  # Initialize RS_PID so cleanup code later can safely check it. The ROS
  # driver will be started after the SDK-level pyrealsense2 checks to avoid
  # race conditions where both librealsense and the driver attempt to open
  # the device at the same time.
  RS_PID=0
# Note: we avoid `-e` here so that non-fatal runtime diagnostics (USB/permission failures)
# don't immediately terminate the script; the test prints useful diagnostics and continues
# to the ROS part even when pyrealsense2 streaming fails. Use exit codes where meaningful.

# manual_realsense_test.sh
# Improved software/hardware smoke test for librealsense (pyrealsense2) and realsense-ros driver.
# Run this inside the container with access to USB devices (see docker run below).

echo "== Environment: user=$(whoami) pwd=$(pwd) =="
python3 -c "import sys, platform; print('python', sys.version); print('platform', platform.platform())"

echo "\n== Pre-check: stop any running realsense nodes to avoid device conflicts =="
# If a realsense2_camera process is already running it can hold the device and
# cause pyrealsense2 to fail with 'failed to set power state'. Kill any such
# processes before running the SDK-level checks so we get a clean test.
source /home/user/workspace/scripts/claimers.sh >/dev/null 2>&1 || true
# Allow caller to skip claimant detection/kill (useful when running inside a
# non-interactive container where sudo is unavailable). Set SKIP_CLAIMERS=1 to
# avoid attempting to kill host processes from inside the container.
if [ "${SKIP_CLAIMERS:-0}" != "1" ]; then
  CLAIMERS_OUTPUT=$(detect_host_claimers || true)
  if [ -n "$(echo "$CLAIMERS_OUTPUT" | sed '/^\s*$/d')" ]; then
    echo "Found running realsense-related processes; attempting to kill them"
    echo "$CLAIMERS_OUTPUT"
    kill_host_claimers || true
    sleep 1
  else
    echo "No host claimers detected"
  fi
else
  echo "SKIP_CLAIMERS=1 -> skipping host claimer detection/kill"
fi

echo "\n== pyrealsense2 basic check =="
python3 - <<'PY'
import pyrealsense2 as rs, sys, traceback, time

def list_devices(ctx):
  try:
    devs = ctx.devices
    return devs
  except Exception as e:
    print('Error accessing ctx.devices:', e)
    return []

ctx = rs.context()
print('pyrealsense2', rs.__version__)
devs = list_devices(ctx)
if len(devs) == 0:
  print('No RealSense devices detected by pyrealsense2 (ctx.devices == 0)')
  # return non-zero so callers can detect absence
  sys.exit(2)

for idx, dev in enumerate(devs):
  try:
    name = dev.get_info(rs.camera_info.name)
  except Exception:
    name = '<unknown>'
  try:
    sn = dev.get_info(rs.camera_info.serial_number)
  except Exception:
    sn = '<unknown>'
  try:
    fw = dev.get_info(rs.camera_info.firmware_version)
  except Exception:
    fw = '<unknown>'
  try:
    port = dev.get_info(rs.camera_info.physical_port)
  except Exception:
    port = '<unknown>'
  print(f'Device[{idx}]: name={name} serial={sn} fw={fw} port={port}')

print('\nAttempting short streaming test (depth+color, 3 frames)')
p = rs.pipeline()
cfg = rs.config()
cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
cfg.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

stream_ok = False
try:
  profile = p.start(cfg)
  for i in range(3):
    frames = p.wait_for_frames(timeout_ms=5000)
    depth = frames.get_depth_frame()
    color = frames.get_color_frame()
    print('frame', i, 'depth_ok', bool(depth), 'color_ok', bool(color))
    time.sleep(0.1)
  stream_ok = True
except Exception as e:
  print('\nFailed to start or capture frames from pipeline:')
  traceback.print_exc()
  # Provide extra diagnostics: probe each device and its sensors/options
  try:
    for dev in devs:
      try:
        print('\n-- Device diagnostic --')
        print('Name:', dev.get_info(rs.camera_info.name))
        print('Serial:', dev.get_info(rs.camera_info.serial_number))
        try:
          print('Firmware:', dev.get_info(rs.camera_info.firmware_version))
        except Exception:
          pass
        try:
          print('Physical port:', dev.get_info(rs.camera_info.physical_port))
        except Exception:
          pass
        try:
          sensors = dev.query_sensors()
        except Exception:
          sensors = []
        for s in sensors:
          try:
            sname = s.get_info(rs.camera_info.name)
          except Exception:
            sname = '<sensor>'
          print(' Sensor:', sname)
          # iterate known options and show supported/current
          for opt in list(rs.option):
            try:
              if s.supports_option(opt):
                try:
                  val = s.get_option(opt)
                except Exception:
                  val = '<value-error>'
                print('  supports', opt.name, 'value=', val)
            except Exception:
              # some options may not be queryable; ignore
              pass
      except Exception as ex_dev:
        print('Device diagnostic error:', ex_dev)
  except Exception as ex_all:
    print('Error while gathering diagnostics:', ex_all)

  print('\nSuggested actions:')
  print(' - Ensure the container has USB access (e.g. -v /dev/bus/usb:/dev/bus/usb or --device entries) and appropriate privileges.')
  print(' - Make sure the host is not already using the camera (e.g. host processes, other containers, or the OS camera stack).')
  print(' - Check udev rules and permissions on /dev/bus/usb (ls -l /dev/bus/usb).')
  print(' - Try re-plugging the camera or rebooting the device; check firmware compatibility with the SDK.')
  # continue to ROS section so user gets additional info
finally:
  try:
    if stream_ok and p is not None:
      p.stop()
  except Exception:
    pass

# Exit code 0 means continue to next checks; caller can decide based on previous return code
sys.exit(0)
PY

# Detect whether the connected device exposes a motion/IMU sensor. If so,
# enable IMU checks for the validator; otherwise omit IMU args (D435 doesn't
# have an IMU, D435i does).
HAS_IMU=0
if command -v python3 >/dev/null 2>&1; then
  HAS_IMU=$(python3 - <<'PY'
import pyrealsense2 as rs
has_imu = 0
try:
    ctx = rs.context()
    devs = ctx.devices
    if len(devs) > 0:
        dev = devs[0]
        try:
            sensors = dev.query_sensors()
        except Exception:
            sensors = []
        for s in sensors:
            try:
                name = s.get_info(rs.camera_info.name)
            except Exception:
                name = ''
            if name and ('motion' in name.lower() or 'gyro' in name.lower() or 'accel' in name.lower() or 'imu' in name.lower()):
                has_imu = 1
                break
except Exception:
    pass
print(has_imu)
PY
  ) || HAS_IMU=0
fi

VALIDATOR_IMU_ARGS=""
if [ "${HAS_IMU:-0}" = "1" ]; then
  echo "Device appears to have IMU -> enabling IMU validator checks"
  VALIDATOR_IMU_ARGS="--enable-imu-check --imu /camera/imu"
else
  echo "No IMU detected -> skipping IMU validator checks"
fi

echo "\n== ROS driver smoke test =="
# Source ROS and workspace (tolerant)
if [ -f /opt/ros/jazzy/setup.sh ]; then
  # Some ROS setup scripts reference variables without defaults and can trigger
  # 'unbound variable' when 'set -u' is in effect. Temporarily disable -u while
  # sourcing to avoid aborting the test.
  set +u
  . /opt/ros/jazzy/setup.sh
  set +e
  # Try to start the node using `ros2 run` when available. Some images may not
  # have the 'run' subcommand available (older/stripped ros2cli). If so, fall
  # back to executing the installed node binary directly from the workspace.
  RS_PID=0
  if command -v ros2 >/dev/null 2>&1 && ros2 --help 2>&1 | grep -q -E '\srun\b'; then
    echo 'Using `ros2 run` to start realsense2_camera_node'
    ros2 run realsense2_camera realsense2_camera_node &> "$LOG" &
    RS_PID=$!
  else
    echo 'ros2 run not available — falling back to direct executable start'
    # Try to locate the executable inside the local install tree
    EXEC_PATH=$(find /home/user/ros_ws/install -type f -executable -name 'realsense2_camera_node' 2>/dev/null | head -n1 || true)
    if [ -n "$EXEC_PATH" ]; then
      echo "Found executable at: $EXEC_PATH"
      "$EXEC_PATH" &> "$LOG" &
      RS_PID=$!
    else
      # Try to extract the executable name from `ros2 pkg executables` if available
      if command -v ros2 >/dev/null 2>&1; then
        EXE_NAME=$(ros2 pkg executables realsense2_camera 2>/dev/null | awk 'NR==1{print $1}') || true
        if [ -n "$EXE_NAME" ]; then
          # search for this executable name in the install tree
          EXEC_PATH=$(find /home/user/ros_ws/install -type f -executable -name "$EXE_NAME" 2>/dev/null | head -n1 || true)
          if [ -n "$EXEC_PATH" ]; then
            echo "Found executable via ros2 pkg executables: $EXEC_PATH"
            "$EXEC_PATH" &> "$LOG" &
            RS_PID=$!
          fi
        fi
      fi
    fi
  fi
  set -e
  if [ "$RS_PID" -ne 0 ]; then
    echo "Started realsense node pid=$RS_PID, waiting 7s for startup..."
    # record node start time to this run's data file and update latest
    echo "node_started $(date -u +%Y-%m-%dT%H:%M:%SZ) pid=$RS_PID" >> "$DATA_TIME_FILE" || true
    ln -sf "$(basename "$DATA_TIME_FILE")" "$LOGDIR/time.log" || cp -f "$DATA_TIME_FILE" "$LOGDIR/time.log" || true
    sleep 7
  else
    echo 'Could not start realsense2_camera_node (neither ros2 run nor direct exec found).' >&2
  fi
  . /home/user/ros_ws/install/setup.bash
  set -u
else
  echo '/home/user/ros_ws/install/setup.bash not found; workspace may not be built' >&2
fi

# Show USB device permissions to help diagnose permission issues
echo "\n== USB device permissions =="
ls -l /dev/bus/usb || true
groups || true

echo "\n== librealsense CLI tools check =="
if command -v rs-enumerate-devices >/dev/null 2>&1; then
  echo 'Found librealsense tool: rs-enumerate-devices; running to collect output...'
  # some rs-enumerate-devices variants do not support --brief, run without it
  rs-enumerate-devices > /tmp/rs_enumerate.out 2>&1 || true
  sed -n '1,200p' /tmp/rs_enumerate.out || true
  # Look for common license/eula wording in the tool output
  if grep -qiE 'license|eula|accept' /tmp/rs_enumerate.out 2>/dev/null; then
    echo "\nDetected license/EULA wording in librealsense tool output."
    echo "Some firmware/firmware-update tools may require interactive acceptance of a license/EULA before they can operate."
    echo "If you need to accept an agreement, you can either:"
    echo "  - Run the toolbox interactively on the host (e.g. 'realsense-viewer' or 'rs-fw-update') and follow prompts; or"
    echo "  - Run a privileged container and run the firmware tool there to accept the EULA:"
    echo "      docker run --rm --privileged -v /dev/bus/usb:/dev/bus/usb -it realsense_ros:debug rs-fw-update" 
    echo "After accepting the license/firmware EULA, retry the test."
  fi
else
  echo 'No librealsense CLI tools (rs-enumerate-devices) found in PATH; firmware/update tools may not be installed in this image.'
  echo 'If you need to update firmware or accept a license, either install librealsense tools on the host or run a privileged container that has them.'
fi

# List topics
ros2 topic list --include-hidden 2>/dev/null | sed -n '1,200p' > /tmp/topics.txt || true
cat /tmp/topics.txt || true

# Try to find an image or depth topic to echo one message
TOPIC=$(grep -E '/camera/.+image|/camera/.+depth|image_raw' /tmp/topics.txt | head -n1 || true)
if [ -n "$TOPIC" ]; then
  echo "Echoing one message from $TOPIC"
  # prefer --once, fall back to -n 1, otherwise run briefly and show first lines
  if ros2 topic echo "$TOPIC" --once 2>/dev/null; then
    true
  elif ros2 topic echo "$TOPIC" -n 1 2>/dev/null; then
    true
  else
    timeout 5 ros2 topic echo "$TOPIC" | head -n 200 || true
  fi
else
  echo "No camera topics found; showing node log tail:"
  tail -n 80 "$LOG" || true
fi

# If the validator script was mounted into the container, run it to perform VSLAM-focused checks
if [ -f /home/user/workspace/scripts/validate_realsense_ros.py ]; then
  echo "\n== Running validate_realsense_ros.py validator via validate_runner.sh =="
  VALIDATOR_TIMEOUT=${VALIDATOR_TIMEOUT:-15}
  mkdir -p "$LOGDIR"
  /home/user/workspace/scripts/validate_runner.sh --validator ros --logdir "$LOGDIR" --timeout "$VALIDATOR_TIMEOUT" || true
else
  echo "Validator script not found at /home/user/workspace/scripts/validate_realsense_ros.py; skipping validator run"
fi
if [ -f /home/user/workspace/validators/validate_realsense_ros.py ]; then
  echo "\n== Running validate_realsense_ros.py validator via validate_runner.sh =="
  VALIDATOR_TIMEOUT=${VALIDATOR_TIMEOUT:-15}
  mkdir -p "$LOGDIR"
  /home/user/workspace/scripts/validate_runner.sh --validator ros --logdir "$LOGDIR" --timeout "$VALIDATOR_TIMEOUT" || true
elif [ -f /home/user/workspace/scripts/validate_realsense_ros.py ]; then
  echo "\n== Running validate_realsense_ros.py validator via validate_runner.sh =="
  VALIDATOR_TIMEOUT=${VALIDATOR_TIMEOUT:-15}
  mkdir -p "$LOGDIR"
  /home/user/workspace/scripts/validate_runner.sh --validator ros --logdir "$LOGDIR" --timeout "$VALIDATOR_TIMEOUT" || true
else
  echo "Validator script not found at validators/ or scripts/; skipping validator run"
fi

# Cleanup: stop the node
if [ "$RS_PID" -ne 0 ] && ps -p $RS_PID >/dev/null 2>&1; then
  echo "Killing realsense node pid=$RS_PID"
  kill $RS_PID || true
  sleep 1
fi
  # write an end timestamp so runs can be correlated easily
echo "end $(date -u +%Y-%m-%dT%H:%M:%SZ) $(hostname) $$" >> "$DATA_TIME_FILE" || true
ln -sf "$(basename "$DATA_TIME_FILE")" "$LOGDIR/time.log" || cp -f "$DATA_TIME_FILE" "$LOGDIR/time.log" || true

echo "\nManual realsense test finished"
exit 0
