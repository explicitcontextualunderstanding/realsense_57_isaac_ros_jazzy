# Realsense ROS Docker build — Issues solved and notes

This document summarizes build problems encountered while building the `realsense_ros` container from `packages/robots/isaac-ros/Dockerfile.bak`, the root causes, the fixes applied, current status, and recommended next steps to speed and stabilize builds.

> Relevant file: `packages/robots/isaac-ros/Dockerfile.bak` (multiple edits during troubleshooting)

---

## Summary checklist (high level)
- [x] dpkg/OpenCV file-overlap error during ROS package install
- [x] Apt `RUN` line continuation bug that caused `apt-get`/`clean` to be interpreted as packages
- [x] Handled UID/GID collisions when creating `user` in base image
- [x] Fixed `chown` failure when the `user` group name did not exist
- [x] Resolved Dockerfile path/context error when building from incorrect working directory
- [x] Guidance and small changes for long librealsense compile (in-progress: build reached the compile step)
- [x] Recommendations to avoid `--no-cache` and use BuildKit/buildx caches

---

## 1) dpkg / OpenCV file-overlap failure

- Symptom
	- Docker build aborted with dpkg error while unpacking `libopencv-dev` (ROS Jazzy packages required OpenCV 4.6).
	- Error: unpacking failed due to file overlaps with OpenCV files present in base image.
- Root cause
	- Base image included OpenCV files/packages (different version) that conflict with ROS packages' `libopencv-dev`.
- Fix applied
	- Attempted to purge common OpenCV packages early in the Dockerfile:
	  ```sh
	  apt-get purge -y opencv-dev 'libopencv-*' 'python3-opencv' || true
	  ```
	- Kept a fallback for installs using dpkg overwrite:
	  ```sh
	  apt-get -o Dpkg::Options::="--force-overwrite" install -y ...
	  ```
	- (Optionally recommended) defensive removal of stray headers/libs if packages are not present in dpkg.
- Status
	- Done — dpkg no longer aborts the build at ROS package install step.
- Tradeoffs / risk
	- Forcing overwrite can produce ABI mismatches if code expects the base image OpenCV; acceptable for disposable build images but note runtime incompatibility risks.
- How to inspect if the issue recurs
	- Inspect base image for OpenCV packages and file owners:
	  ```bash
	  docker run --rm <base-image> dpkg -l | grep -i opencv || true
	  docker run --rm <base-image> dpkg -S /usr/include/opencv4 || true
	  ```

---

## 2) Apt `RUN` continuation bug (parsed cleanup tokens as packages)

- Symptom
	- Build failed with `E: Unable to locate package apt-get` and `clean` (these were interpreted as package names).
- Root cause
	- Malformed multi-line `RUN` where line-continuation logic caused cleanup commands to be parsed as package names.
- Fix applied
	- Corrected `RUN` block to properly terminate the `apt-get install` command and run cleanup afterwards (ensure `&& \` placement).
	- Example corrected snippet:
	  ```dockerfile
	  RUN apt-get update && \
	      apt-get install -y ... && \
	      rm -rf /var/lib/apt/lists/* && apt-get clean
	  ```
- Status
	- Done — cleanup runs after install and no longer interpreted as packages.
	
---

## 3) librealsense long compile (performance / OOM concerns)

- Symptom
	- Build progresses to `make -j$(nproc)` and spends many minutes; possibility of OOM on constrained hardware.
- Root cause
	- librealsense is large; default parallel builds can exceed available memory on devices like Jetson Nano.
- Actions / guidance given
	- Use BuildKit `--progress=plain` to see full compiler output and detect OOMs.
	- Suggested Dockerfile improvements:
		- Install `ninja-build` and `ccache`.
		- Use `cmake -GNinja` and `ninja -jN` to control parallelism more reliably.
		- Split configure and build across separate `RUN` layers (caching benefit).
		- Use BuildKit cache mounts for `ccache` (`--mount=type=cache`) or `docker buildx` `--cache-to/--cache-from`.
	- Temporary mitigation: lower parallelism, e.g., `-j4` or `-j2`.
- Status
	- In progress — build now reaches the compile step (expected), recommendations provided to improve speed and reliability.
- Example BuildKit commands
	- Verbose build:
	  ```bash
	  export DOCKER_BUILDKIT=1
	  docker build --progress=plain -f packages/robots/isaac-ros/Dockerfile.bak -t realsense_ros:local .
	  ```
	- Buildx with persistent local cache:
	  ```bash
	  export DOCKER_BUILDKIT=1
	  docker buildx build --progress=plain \
	    --cache-to type=local,dest=.buildx-cache \
	    --cache-from type=local,src=.buildx-cache \
	    -f packages/robots/isaac-ros/Dockerfile.bak -t realsense_ros:local .
	  ```
    	
---

## 4) UID/GID collisions when creating `user` (groupadd/useradd errors)

- Symptom
	- `groupadd: GID '1000' already exists` and `useradd: UID 1000 is not unique` errors during `RUN groupadd/useradd`.
- Root cause
	- The base image already had UID/GID 1000 assigned to some account/group.
- Fix applied
	- Replaced naive `groupadd`/`useradd` with a robust shell block that:
		- Checks for existing GID (`getent group`) before `groupadd`.
		- Checks for existing `user` account before `useradd`.
		- If UID already exists but username not present, creates `user` with `useradd -o -u <UID>` to allow non-unique UID so the `user` account exists with the expected numeric UID.
	- Example (applied in Dockerfile):
	  ```dockerfile
	  RUN set -eux; \
	    if ! getent group "${GID}" >/dev/null; then groupadd -g "${GID}" user; fi; \
	    if ! id -u user >/dev/null 2>&1; then \
	      if getent passwd "${UID}" >/dev/null; then \
	        useradd -m -o -u "${UID}" -g "${GID}" -s /bin/bash user; \
	      else \
	        useradd -m -u "${UID}" -g "${GID}" -s /bin/bash user; \
	      fi; \
	    fi
	  ```
- Status
	- Done — user/group creation no longer fails.
	    
---

## 5) `chown: invalid group: 'user:user'`

- Symptom
	- `chown -R user:user /home/user/ros_ws` failed with `invalid group` because the group name `user` did not exist (numeric GID existed under a different name).
- Root cause
	- `user` group name was not present even though numeric GID existed.
- Fix applied
	- Use a robust ownership resolution step:
		- Resolve `TARGET_USER`: prefer `user`, else use the existing username that owns `${UID}`, else create `user`.
		- Resolve `TARGET_GROUP`: use group name for `${GID}` if present, else use numeric GID.
		- Call `chown -R "${TARGET_USER}:${TARGET_GROUP}" /home/user/ros_ws`.
	- Example snippet included in Dockerfile.
- Status
	- Done — chown step now uses resolved names or numeric GID and does not error.
	    
---

## 6) Docker build `lstat packages: no such file or directory`

- Symptom
	- Running `docker build -f [nanosaur](http://_vscodecontentref_/1).` from the wrong working directory produced `lstat packages: no such file or directory`.
- Root cause
	- `-f` path is relative to current working directory; the build context (`.`) must also be correct.
- Fix / guidance
	- Build from the repo root (where `packages/robots/...` exists), or use absolute paths for `-f` and the build context.
	- Example correct commands:
		- From repo root:
		  ```bash
		  cd ~/workspace/jetson-containers
		  export DOCKER_BUILDKIT=1
		  docker build --progress=plain -f packages/robots/isaac-ros/Dockerfile.bak -t realsense_ros:local .
		  ```
		- Or use absolute paths:
		  ```bash
		  docker build -f /home/<user>/workspace/jetson-containers/packages/robots/isaac-ros/Dockerfile.bak \
		    -t realsense_ros:local /home/<user>/workspace/jetson-containers
		  ```
- Status
	- Done — you now have the right commands to run from the Jetson Nano.
	    
---

## 7) Caching and `--no-cache` discussion

- Problem
	- Using `--no-cache` rebuilds all layers and forces a complete rebuild (very slow for librealsense).
- Recommendation
	- Do NOT use `--no-cache` for iterative development.
	- Use BuildKit + `buildx` and either:
		- `--cache-to/--cache-from` for a persistent local layer cache, or
		- `--mount=type=cache` (BuildKit) for `ccache` persistence.
- Example buildx command (persistent cache):
  ```bash
  docker buildx build --progress=plain \
    --cache-to type=local,dest=.buildx-cache \
    --cache-from type=local,src=.buildx-cache \
    -f packages/robots/isaac-ros/Dockerfile.bak -t realsense_ros:local .
- [[Nvidia/Nano/Nanosaur/Stereo Cameras/RealSense/librealsense/LibRS Rest API server]]
  collapsed:: true
	- https://github.com/IntelRealSense/librealsense/tree/development/wrappers/rest-api
- [[Nvidia/Nano/Nanosaur/Stereo Cameras/RealSense/ROS/Dockerfile/Image]]
  collapsed:: true
	- https://hub.docker.com/r/husarion/realsense
	- https://hub.docker.com/layers/nicosander/realsense-ros/l4t-r36.2.0-ros_humble-desktop/images/sha256-9d2501999a2bacb62d77a8935312bf3735d27bb78a8cbe652929a800ccecadad
		- ```
		  Image Layers
		  1
		  ARG RELEASE
		  0 B
		  2
		  ARG LAUNCHPAD_BUILD_ARCH
		  0 B
		  3
		  LABEL org.opencontainers.image.ref.name=ubuntu
		  0 B
		  4
		  LABEL org.opencontainers.image.version=22.04
		  0 B
		  5
		  ADD file ... in /
		  26.09 MB
		  6
		  CMD ["/bin/bash"]
		  0 B
		  7
		  ENV DEBIAN_FRONTEND=noninteractive LANGUAGE=en_US:en LANG=en_US.UTF-8
		  0 B
		  8
		  /bin/sh -c apt-get update &&
		  213.09 MB
		  9
		  ARG CUDA_URL
		  0 B
		  10
		  ARG CUDA_DEB
		  0 B
		  11
		  ARG CUDA_PACKAGES
		  0 B
		  12
		  ARG CUDA_ARCH_LIST
		  0 B
		  13
		  |4 CUDA_ARCH_LIST=87 CUDA_DEB=cuda-tegra-repo-ubuntu2204-12-2-local CUDA_PACKAGES=cuda-toolkit* CUDA_URL=https://nvidia.box.com/shared/static/uvqtun1sc0bq76egarc8wwuh6c23e76e.deb
		  3.24 GB
		  14
		  ENV NVIDIA_VISIBLE_DEVICES=all NVIDIA_DRIVER_CAPABILITIES=all CUDAARCHS=87
		  0 B
		  15
		  ENV PYTHONIOENCODING=utf-8
		  0 B
		  16
		  /bin/sh -c apt-get update &&
		  22.3 MB
		  17
		  /bin/sh -c pip3 install --upgrade
		  24.53 MB
		  18
		  /bin/sh -c cmake --version &&
		  12.36 KB
		  19
		  /bin/sh -c apt-get update &&
		  233.29 MB
		  20
		  ARG LIBREALSENSE_VERSION=master
		  0 B
		  21
		  |1 LIBREALSENSE_VERSION=master /bin/sh -c git
		  274.33 MB
		  22
		  |1 LIBREALSENSE_VERSION=master /bin/sh -c python3
		  18.69 KB
		  23
		  ARG CUDNN_URL
		  0 B
		  24
		  ARG CUDNN_DEB
		  0 B
		  25
		  ARG CUDNN_PACKAGES
		  0 B
		  26
		  |3 CUDNN_DEB=cudnn-local-tegra-repo-ubuntu2204-8.9.4.25 CUDNN_PACKAGES=libcudnn*-dev libcudnn*-samples CUDNN_URL=https://nvidia.box.com/shared/static/ht4li6b0j365ta7b76a6gw29rk5xh8cy.deb
		  1.53 GB
		  27
		  |3 CUDNN_DEB=cudnn-local-tegra-repo-ubuntu2204-8.9.4.25 CUDNN_PACKAGES=libcudnn*-dev libcudnn*-samples CUDNN_URL=https://nvidia.box.com/shared/static/ht4li6b0j365ta7b76a6gw29rk5xh8cy.deb
		  120.11 KB
		  28
		  ARG TENSORRT_URL
		  0 B
		  29
		  ARG TENSORRT_DEB
		  0 B
		  30
		  ARG TENSORRT_PACKAGES
		  0 B
		  31
		  |3 TENSORRT_DEB=nv-tensorrt-local-repo-l4t-8.6.2-cuda-12.2 TENSORRT_PACKAGES=tensorrt tensorrt-libs python3-libnvinfer-dev
		  1.71 GB
		  32
		  /bin/sh -c apt-get update &&
		  13.23 MB
		  33
		  ENV OPENBLAS_CORETYPE=ARMV8
		  0 B
		  34
		  /bin/sh -c pip3 show numpy
		  16.3 MB
		  35
		  ARG OPENCV_URL
		  0 B
		  36
		  ARG OPENCV_DEB
		  0 B
		  37
		  COPY file:ff2a8223cd53f232409d12a29425e3b84d2a424a5c34934b62529319d7e94449 in /opt/opencv_install.sh
		  1.05 KB
		  38
		  |2 OPENCV_DEB=OpenCV-4.8.1-aarch64.tar.gz OPENCV_URL=https://nvidia.box.com/shared/static/ngp26xb9hb7dqbu6pbs7cs9flztmqwg0.gz /bin/sh -c
		  1.05 KB

---

## Additional resolved issues

These historical items were resolved during Dockerfile troubleshooting and are recorded here for completeness.

### Expired ROS 2 GPG Key Error
- Symptom: `apt` failed to update due to an expired Open Robotics GPG key (`EXPKEYSIG F42ED6FBAB17C654`) for the `noble` (Jazzy) repository.
- Fix: Remove old ROS keys and install the updated ROS 2 keyring from the official `ros-distro` repository. Replaced `apt-key add` usage with `signed-by` in the apt sources list.
- Notes: Ensures secure package verification and prevents `apt update` failures.

### Incorrect or Missing Apt Key Management
- Symptom: Replacing keys with deprecated `apt-key add` caused inconsistent keyrings and signature failures.
- Fix: Migrate to storing keys under `/usr/share/keyrings/` and reference them in `/etc/apt/sources.list.d/*.list` with `signed-by=/usr/share/keyrings/ros2-archive-keyring.gpg`.
- Action: Clean out conflicting keys from `/etc/apt/trusted.gpg.d/` and `/etc/apt/trusted.gpg` to avoid duplicate signatures.

### Image runtime conveniences added

- Installed `exec_with_ros` helper into the image (`/usr/local/bin/exec_with_ros`) so callers can run `docker exec ... exec_with_ros ...` to obtain a shell or run commands with ROS sourced without retyping source commands.
- Added `/etc/profile.d/ros.sh` (copied from `docker/ros_profile.sh`) which sources the workspace or system ROS setup when login shells start; this helps `bash -lc` invocations pick up ROS automatically.

### Missing X11 Development Libraries for GLFW
- Symptom: librealsense CMake configuration failed due to missing X11 headers/libs required by GLFW.
- Fix: Add X11 developer packages to apt installs: `libx11-dev libxrandr-dev libxi-dev libxcursor-dev libxinerama-dev`.

### Use of `jetson-containers build` with Unregistered Dockerfile
- Symptom: `jetson-containers build` failed when passed an arbitrary Dockerfile path; it expects registered package names.
- Fix / Guidance: Use plain `docker build` for ad-hoc Dockerfile builds and pass required build args explicitly (e.g., `BASE_IMAGE`). Documented build examples in the README.

### Proper Passing of Base Image Argument
- Symptom: Dockerfile used `ARG BASE_IMAGE` with `FROM ${BASE_IMAGE}` but build failed when `BASE_IMAGE` was not provided.
- Fix: Always pass `--build-arg BASE_IMAGE=<image>` during builds; README examples include the tested base image:

```
dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04
```

### User Permission Management
- Symptom: Permission issues when mounting host volumes or building ROS workspaces as non-root.
- Fix: Create a non-root user/group in the image that matches the host UID/GID; handle collisions robustly (checks for existing UID/GID and create or reuse accordingly). This avoids needing `chown` on mounted volumes and helps `colcon` builds.

---

If you want these items converted into a compact changelog with dates or a triage checklist for contributors, I can generate that next.
*** End Patch