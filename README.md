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

Quick smoke test
----------------
Fastest way to validate hardware + driver + Python bindings and capture artifacts:

```bash
# 1) Build image (adjust args as needed)
docker build -t realsense_ros:debug \
  --build-arg BASE_IMAGE=dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04 \
  --build-arg INSTALL_PYREALSENSE2=false \
  -f Dockerfile .

# 2) Run the automated end-to-end validator (creates ./realsense_test_outputs/*)
./scripts/run_automated_realsense_test.sh --image realsense_ros:debug --timeout 20

# 3) Inspect the latest JSON result
ls -1 realsense_test_outputs/validate_realsense_plus_*.json | tail -n1 | xargs -r jq .
```

VSLAM readiness (grayscale + TF)
--------------------------------

- The automated test launches the node with IR (infrared) enabled and TF publishing, and validates grayscale + TF by default.
- Run: `./scripts/run_automated_realsense_test.sh --image realsense_ros:debug --timeout 20`
- Check the JSON summary fields (latest file under `realsense_test_outputs/`):
  - `gray_received: true`, `gray_freq_hz >= 15.0`
  - `tf_ok: true` and `tf_gray_ok: true`
- Infra topics are typically `/camera/camera/infra1/image_rect_raw` (mono8 848x480@30Hz) and `/camera/camera/infra1/camera_info`.
- See `docs/realsense_vslam_readiness.md` for the latest captured evidence.

Profiles (IR and Sync)
----------------------

You can choose an IR (grayscale) profile and tune the validator’s timestamp sync threshold when needed:

- Launch with a specific IR profile (inside container):
  - 848x480 @ 30 Hz (default in our examples)
    - `ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=true -p enable_depth:=true -p enable_infra:=true -p infra_width:=848 -p infra_height:=480 -p infra_fps:=30 -p publish_tf:=true -p tf_publish_rate:=10.0 -p enable_sync:=true`
  - 1280x720 @ 30 Hz
    - `ros2 run realsense2_camera realsense2_camera_node --ros-args -p enable_color:=true -p enable_depth:=true -p enable_infra:=true -p infra_width:=1280 -p infra_height:=720 -p infra_fps:=30 -p publish_tf:=true -p tf_publish_rate:=10.0 -p enable_sync:=true`

- Run validator with custom sync threshold (inside container):
  - Default threshold is 20 ms. Increase if your higher-res profile shows benign offsets.
  - `python3 validators/validate_realsense_plus.py --color /camera/camera/color/image_raw --depth /camera/camera/depth/image_rect_raw --caminfo /camera/camera/color/camera_info --grayscale /camera/camera/infra1/image_rect_raw --require-gray --check-tf --sync-threshold-ms 40 --timeout 20 --out-file /home/user/ros_ws/logs/validate_custom.json`

Notes:
- Infra topics are typically `/camera/camera/infra1/image_rect_raw` and `/camera/camera/infra1/camera_info`. Use `infra2` if you prefer the second IR stream.
- Larger resolutions can increase inter-stream timestamp offsets; `enable_sync:=true` plus a slightly higher validator threshold can help if your VSLAM tolerates it.

Prebuilt image (GHCR)
---------------------
We plan to publish this image to GitHub Container Registry (GHCR). Publishing will be done manually. Once available, pull and use it directly:

```bash
# pull the prebuilt image
docker pull ghcr.io/explicitcontextualunderstanding/realsense_57_ros_jazzy:debug

# run the automated end-to-end validator using the prebuilt image
./scripts/run_automated_realsense_test.sh --image ghcr.io/explicitcontextualunderstanding/realsense_57_ros_jazzy:debug --timeout 20
```

Developer build notes (internals): see `docs/BUILD_ISSUES.md` for multi-stage build, ccache, and caching guidance.

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

3. Run the verifier (use the helper to ensure ROS is sourced inside the exec session):

```bash
# from the repo root on the host
./scripts/exec_with_ros.sh realsense_debug -- /home/user/ros_ws/scripts/verify_build.sh 45
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

Mounting your repo and persisting logs to the host (recommended for development)
--------------------------------------------------------------------------

If you want `manual_realsense_test.sh` and the runtime validator to write logs
directly into a host-accessible directory so you can inspect results without
copying files from the container, start the container with an explicit bind
mount for a host log directory. The `manual_realsense_test.sh` script prefers
to write logs into `/home/user/ros_ws/logs` (or `$HOME/ros_ws/logs`) when that
path exists and is writable inside the container.

Example: run a long-lived container that mounts your repository (so
`/home/user/workspace/scripts/validate_realsense_ros.py` is available) and maps
the host `realsense_test_outputs/` directory into the container's
`/home/user/ros_ws/logs` so logs are persisted on the host:

```bash
# from the repository root on the host
docker run -d --name realsense_debug --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -v "$(pwd)":/home/user/workspace:ro \
  -v "$(pwd)/realsense_test_outputs":/home/user/ros_ws/logs:rw \
  --group-add "$(getent group video | cut -d: -f3)" \
  realsense_ros:debug sleep infinity
```

Notes:
- `$(pwd)` is mounted read-only at `/home/user/workspace` so `scripts/` and
  other repo files are available inside the container without risking accidental
  modifications. Adjust to `:rw` if you need write access from inside the
  container.
- The host directory `realsense_test_outputs/` will receive files written by the
  script (e.g. `realsense_node.log`, `validate_realsense_ros.out`, etc.). Make
  sure that directory exists and is writable by your user on the host.

After starting the container you can run the manual test inside it (with ROS sourced):

```bash
./scripts/exec_with_ros.sh realsense_debug -- bash -lc "chmod +x /home/user/manual_realsense_test.sh || true; /home/user/manual_realsense_test.sh"
```

When the script finishes you will find the logs on the host under
`./realsense_test_outputs/` (or the path you used for the bind mount). If the
container was started without mounting a host logs directory, logs will live in
`/tmp` inside the container and must be copied out with `docker cp`.

Quick check inside a running container to confirm mounts:

```bash
docker inspect --format='{{range .Mounts}}{{println .Source "->" .Destination}}{{end}}' realsense_debug
docker exec -it realsense_debug bash -lc "ls -lah /home/user/workspace || true; ls -lah /home/user/ros_ws/logs || true"
```

Note about `ros2` and `docker exec`
----------------------------------
The image sets an `ENTRYPOINT` (`/usr/local/bin/realsense_entrypoint.sh`) that
sources either the built workspace (`/home/user/ros_ws/install/setup.bash`) or
the system ROS setup (`/opt/ros/jazzy/setup.bash`) when the container process
starts. However, `docker exec` starts a new process inside the running
container and does not re-run the entrypoint script for that new shell. That
is why you may see `bash: line 1: ros2: command not found` or `import
tf2_ros` failing when you `docker exec` without first sourcing the ROS
environment.

To run ROS tooling inside an interactive `docker exec` session, source the
entrypoint or the ROS setup manually. Example:

```bash
# Recommended: source the bundled entrypoint which sets up workspace or system ROS
docker exec -it realsense_debug bash -lc "source /usr/local/bin/realsense_entrypoint.sh && ros2 --version"

# Or explicitly source the workspace (if you built the workspace inside the image)
docker exec -it realsense_debug bash -lc "source /home/user/ros_ws/install/setup.bash && python3 -c 'import tf2_ros; print("tf2_ros OK")'"
```

If you prefer not to source each time, start a new container process with a
command that runs the entrypoint (for example use `docker run` with the
container's entrypoint) so the ROS environment is initialized for that shell.

Convenience helper: `scripts/exec_with_ros.sh`
------------------------------------------------
To avoid retyping the `source` command when using `docker exec`, a small
helper script has been added at `scripts/exec_with_ros.sh`. It runs
`docker exec` and sources the container entrypoint so `ros2` and workspace
packages are available in the session.

Usage examples:

```bash
# open an interactive shell with ROS sourced (defaults to container 'realsense_debug')
./scripts/exec_with_ros.sh

# open an interactive shell in a specific container
./scripts/exec_with_ros.sh realsense_debug

# run a specific command in the container with ROS sourced
./scripts/exec_with_ros.sh realsense_debug -- python3 -c "import tf2_ros; print('tf2_ros OK')"
```

If you'd like, you can also use the included helper `run_test_and_collect_logs.sh`
which constructs an appropriately mounted `docker run` command and populates a
host log directory (default `~/realsense_test_logs`).

Passing a custom base image and log mount (brief)
------------------------------------------------
You can override the build base image using the `--build-arg BASE_IMAGE=...`
option to `docker build` (this is useful to pick a specific ROS distro or CUDA
variant). Example build command:

```bash
docker build -t realsense_ros:debug \
  --build-arg BASE_IMAGE=dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04 \
  --build-arg INSTALL_PYREALSENSE2=false \
  -f Dockerfile .
```

Logging improvements: when you mount a host directory at `/home/user/ros_ws/logs` (the
wrapper/helpers do this), the runner now creates a per-run subfolder under
`realsense_test_outputs/wrapper/<timestamp>/` to avoid collisions and make it
easy for automation to scrape results.

Per-run layout (example):

```
realsense_test_outputs/
├─ validate_realsense_plus_20250903_053029.json
├─ validate_realsense_plus_20250903_053029.log
├─ sdk_only_20250903_044755.log
└─ wrapper/
   └─ 20250903_185240/
      ├─ wrapper.log
      ├─ validate_realsense_plus_20250903_185240.log
      ├─ validate_realsense_plus_20250903_185240.json
      ├─ validate_realsense_plus.log -> validate_realsense_plus_20250903_185240.log
      └─ data/time.log
```

Notes:
- The runner prefers `validate_realsense_plus_*` artifacts and copies both `.log` and
  `.json` into the per-run `wrapper/<timestamp>/` folder. If those are not present it
  falls back to `validate_realsense_ros_*` artifacts.
- `wrapper/<timestamp>/wrapper.log` contains the wrapper stdout/stderr (docker
  invocation, claimer handling, etc.) and is the first place to check for run-level
  problems.
- Use the top-level wrapper or the consolidated runner to create runs:

```bash
# ephemeral container run (uses IMAGE and HOST_LOG_DIR env vars if provided)
./run_realsense_test.sh

# or call the runner directly
HOST_LOG_DIR="$(pwd)/realsense_test_outputs" IMAGE="realsense_ros:debug" bash ./scripts/runner.sh

New runner options
------------------
The runner now supports a small set of host-friendly options to help automate
common preflight steps:

- `--host-kill` (host-only): before launching the privileged ephemeral container
  the runner will invoke `scripts/host_kill_claimers.sh --yes --stop-containers`
  to attempt to free any host-side processes or containers that are claiming the
  RealSense device. This is intended for use when you run the runner on the
  host (not inside a CI container) and want an automated cleanup step.
- `--device <vendor:product>`: override the USB device id passed to the host
  claimant cleaner (default: `8086:0b07`). Example: `--device 8086:0b07`.

Preflight checks
----------------
For ephemeral runs the runner performs lightweight image preflight checks:

- Verifies `ros2` is available in the chosen image (hard failure) so you get a
  fast actionable message if the image lacks ROS runtime tooling.
- Attempts to import `pyrealsense2` in the image and warns if that import
  fails (soft warning). Some builds intentionally omit `pyrealsense2` (e.g.
  platforms without a prebuilt wheel) so this is informational by default.

If preflight fails for `ros2`, fix your image (or pick the correct base image)
before proceeding. If `pyrealsense2` is missing, consider building the wheel or
setting `--build-arg INSTALL_PYREALSENSE2=true` when building the image.
```

Quick helper to inspect the latest run:

```bash
# find the latest per-run wrapper folder
LATEST_RUN_DIR=$(ls -1dt realsense_test_outputs/wrapper/*/ 2>/dev/null | head -n1)
echo "Latest run folder: $LATEST_RUN_DIR"

# preview wrapper log
head -n 200 "$LATEST_RUN_DIR/wrapper.log"

# show the most recent validator JSON under that run (prefer plus)
ls -1 "$LATEST_RUN_DIR"/validate_realsense_plus*.json "$LATEST_RUN_DIR"/validate_realsense_ros*.json 2>/dev/null | head -n1 | xargs -r jq .
```



Notes and troubleshooting
-------------------------
- Ensure a RealSense device is attached to the host and not used by other processes.
- The Dockerfile uses `ARG SCRIPTS_DEST` to control where `scripts/` are copied inside the image
  (default: `/home/user/ros_ws/scripts`). If you changed that at build-time, use the corresponding path.
- If the validator fails because of missing Python wheels (e.g., `numpy-quaternion`), provide the wheel
  for your target architecture and install it in a custom build stage (we can add a `docker_wheels/` flow if you want).

ROS apt distro (Ubuntu codename)
--------------------------------

- The ROS apt archive under `http://packages.ros.org/ros2/ubuntu/dists/` is organized by Ubuntu/apt
  codenames (for example `jammy`, `noble`, `bookworm`), not by ROS release names (for example
  `jazzy`). The Dockerfile by default auto-detects the base image's Ubuntu codename (runs
  `lsb_release -cs` inside the build image) and uses that when creating the apt `deb ... <codename> main`
  entry.
- For reproducible builds or when you need to pin a specific apt path, pass the codename explicitly
  with the `ROS_APT_DISTRO` build-arg. Example:

```bash
DOCKER_BUILDKIT=1 docker build --progress=plain -t realsense_ros:debug \
  --build-arg BASE_IMAGE=dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04 \
  --build-arg INSTALL_PYREALSENSE2=false \
  --build-arg ROS_APT_DISTRO=jammy \
  -f Dockerfile .
```

If you omit `ROS_APT_DISTRO` the Dockerfile will automatically select the codename that matches the
base image (recommended for portability); otherwise specify `jammy`, `noble`, etc. to pin the
apt source explicitly (recommended for CI/reproducible builds).

Want me to also copy the latest verify logs out of the running container into `./verify_logs/`? Reply "copy logs" and I'll do it.

Build examples
--------------
This repository was developed and tested using the following base image:

```
dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04
```

You can build the image using the `Dockerfile` in the repo. Example variants:

- Default build (uses the repository defaults):

```bash
docker build -t realsense_ros:debug \
  --build-arg BASE_IMAGE=dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04 \
  --build-arg INSTALL_PYREALSENSE2=false \
  -f Dockerfile .
```

- Build while changing the destination where `scripts/` are placed inside the image:

```bash
docker build -t realsense_ros:debug \
  --build-arg BASE_IMAGE=dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04 \
  --build-arg SCRIPTS_DEST=/opt/realsense/scripts \
  -f Dockerfile .
```

- Build and attempt to install `pyrealsense2` wheel during image build (only if a compatible wheel is available):

```bash
docker build -t realsense_ros:debug \
  --build-arg BASE_IMAGE=dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04 \
  --build-arg INSTALL_PYREALSENSE2=true \
  -f Dockerfile .
```

Notes:
- Use `--build-arg UID=$(id -u) --build-arg GID=$(id -g)` if you want files in the image owned by your host UID/GID.
- If you change `BASE_IMAGE` to a different ROS/Jazzy variant, you may need to adapt CUDA / platform-specific wheel choices.

Quick end-to-end example (build -> run -> runner)
-------------------------------------------------
This sequence shows a typical developer workflow: build the image, start a long-lived
container that mounts your repo and a host log directory, then run the runner which
creates a per-run wrapper folder under the host-mounted log dir.

1) Build the image (this may take several minutes while librealsense compiles):

```bash
DOCKER_BUILDKIT=1 docker build --progress=plain -t realsense_ros:debug \
  --build-arg BASE_IMAGE=dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04 \
  --build-arg INSTALL_PYREALSENSE2=false \
  -f Dockerfile .
```

2) Start a long-lived container and mount the host `realsense_test_outputs/` so logs are persisted:

```bash
mkdir -p ./realsense_test_outputs
docker run -d --name realsense_debug --privileged \
  -v /dev/bus/usb:/dev/bus/usb \
  -v "$(pwd)":/home/user/workspace:ro \
  -v "$(pwd)/realsense_test_outputs":/home/user/ros_ws/logs:rw \
  --group-add "$(getent group video | cut -d: -f3)" \
  realsense_ros:debug sleep infinity
```

3) Run the runner from the host (it will create a per-run folder under `realsense_test_outputs/wrapper/<TIMESTAMP>/`):

```bash
# simple run
HOST_LOG_DIR="$(pwd)/realsense_test_outputs" SKIP_CLAIMERS=1 ./scripts/runner.sh

# or run with automated host claimant cleanup for device 8086:0b07
HOST_LOG_DIR="$(pwd)/realsense_test_outputs" SKIP_CLAIMERS=1 ./scripts/runner.sh --host-kill --device 8086:0b07
```

After the runner finishes you will find `wrapper.log` and validator JSONs under the `wrapper/<TIMESTAMP>/` folder.

Automated end-to-end test (single command)
-----------------------------------------
Prefer a one-shot automation that launches the container, starts the driver, waits for topics, and runs the enhanced validator? Use:

```bash
# runs privileged with USB passthrough and writes artifacts to ./realsense_test_outputs
./scripts/run_automated_realsense_test.sh --image realsense_ros:debug --timeout 20
```

Notes:
- Artifacts are written to `realsense_test_outputs/validate_realsense_plus_<timestamp>.json|.log`.
- For CI, the integration test wrapper at `tests/test_integration_realsense_end_to_end.py` can be enabled with `RUN_INTEGRATION=1`.

CI hints
--------
Basic unit tests (no hardware):

```bash
python3 -m pip install -r requirements-test.txt
pytest -q  # runs unit tests; integration is skipped by default
```

Enable hardware-in-the-loop integration test (requires Docker + USB camera available on CI runner):

```bash
RUN_INTEGRATION=1 pytest -q tests/test_integration_realsense_end_to_end.py -s
```

Build caching for faster CI:

```bash
# BuildKit + buildx with local cache directory
docker buildx build --progress=plain \
  --cache-to type=local,dest=.buildx-cache \
  --cache-from type=local,src=.buildx-cache \
  -t realsense_ros:debug -f Dockerfile . --load
```

GitHub Actions (registry cache via Makefile):

```yaml
name: build-image
on: [push, pull_request]
permissions:
  contents: read
  packages: write  # needed to push/pull GHCR cache
jobs:
  build:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Set up Docker Buildx
        uses: docker/setup-buildx-action@v3
      - name: Login to GHCR
        uses: docker/login-action@v3
        with:
          registry: ghcr.io
          username: ${{ github.actor }}
          password: ${{ secrets.GITHUB_TOKEN }}
      - name: Build with registry cache (does not push final image)
        run: |
          IMAGE=ghcr.io/explicitcontextualunderstanding/realsense_57_ros_jazzy:debug \
          CACHE_REF=ghcr.io/explicitcontextualunderstanding/realsense_57_ros_jazzy:buildcache \
          make build-cache-reg IMAGE="$IMAGE" CACHE_REF="$CACHE_REF"
```

Tip: run the quick smoke test in CI after build to verify camera + driver when a USB device is available on the runner:

```bash
./scripts/run_automated_realsense_test.sh --image realsense_ros:debug --timeout 20 || true
test -f realsense_test_outputs/validate_realsense_plus_*.json && jq . realsense_test_outputs/validate_realsense_plus_*.json | tail -n1 || true
```

Build issues & history
----------------------
We collected a history of build issues encountered while iterating on the Dockerfile; the full details are in `docs/BUILD_ISSUES.md`. That document lists symptoms, root causes, fixes applied, current status, and recommended next steps for each issue.
