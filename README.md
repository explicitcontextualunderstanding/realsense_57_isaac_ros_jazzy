# realsense_ros (Docker image + verifier)

This repository contains a reproducible Dockerfile that builds
`librealsense` (pinned to v2.57.2) and the `realsense-ros` driver
for ROS Jazzy (v4.57.2), plus small runtime verifier scripts.

Verifier
--------
A lightweight verifier script is provided at `scripts/verify_build.sh`.
It performs these checks inside a running container:

- `rs-enumerate-devices --verbose` (device enumeration)
- A short `pyrealsense2` smoke test (capture 3 frames)
- Launch `realsense2_camera` (background), run the ROS validator
  (`rs2_test.py`), and collect logs.

Quick steps to run the verifier
--------------------------------

1. Build the Docker image (example -- adjust `BASE_IMAGE` if needed):

```bash
docker build -t realsense_ros:debug \
  --build-arg BASE_IMAGE=dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04 \
  --build-arg INSTALL_PYREALSENSE2=false \
  -f Dockerfile .
```

2. Start a container with USB passthrough so a connected RealSense camera is visible:

```bash
docker run -d --name realsense_debug --privileged \
  -v /dev/bus/usb:/dev/bus/usb --network host realsense_ros:debug sleep infinity
```

3. Run the verifier (the image already copies `scripts/` into the image at `/home/user/ros_ws/scripts` by default):

```bash
docker exec -it realsense_debug bash -lc '/home/user/ros_ws/scripts/verify_build.sh 45'
```

The script writes logs into a timestamped directory under `/tmp` inside the container, for example:

```
/tmp/verify_build_logs_<timestamp>/
  rs-enumerate.txt
  pyrealsense2_smoke.txt
  realsense_node.log
  ros_validator.log
  summary.txt
```

4. (Optional) Copy logs from the container to the host for inspection:

```bash
# Find the verify dir name (adjust timestamp as needed) and copy it
docker exec realsense_debug bash -lc 'ls -dt /tmp/verify_build_logs_* | head -n1'
docker cp realsense_debug:/tmp/verify_build_logs_<timestamp> ./verify_logs
```

Notes and troubleshooting
-------------------------
- Ensure a RealSense device is attached to the host and not used by other processes.
- The Dockerfile uses `ARG SCRIPTS_DEST` to control where `scripts/` are copied inside the image
  (default: `/home/user/ros_ws/scripts`). If you changed that at build-time, use the corresponding path.
- If the validator fails because of missing Python wheels (e.g., `numpy-quaternion`), provide the wheel
  for your target architecture and install it in a custom build stage (we can add a `docker_wheels/` flow if you want).

Want me to also copy the latest verify logs out of the running container into `./verify_logs/`? Reply "copy logs" and I'll do it.
