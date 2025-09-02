#---
# name: realsense_ros
# group: hardware
# depends: [cuda, python, cmake, ros2]
# notes:
#   - Builds librealsense from source (pin with LIBREALSENSE_VERSION)
#   - Builds NVIDIA-ISAAC-ROS `realsense-ros` in the same container (REALSENSE_ROS_BRANCH)
#   - Host expected to manage kernel modules (JetsonHacks or similar)
#---
ARG BASE_IMAGE=dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04
FROM ${BASE_IMAGE}

LABEL maintainer="you@example.com" \
      org.opencontainers.image.title="realsense_ros" \
      org.opencontainers.image.source="https://github.com/NVIDIA-ISAAC-ROS/realsense-ros"

# Use matching minor/patch releases for SDK and ROS driver to ensure compatibility.
# Intel librealsense v2.57.2 matches realsense-ros 4.57.2 (see releases: v2.57.2 / 4.57.2)
ARG LIBREALSENSE_VERSION=v2.57.2
ARG REALSENSE_ROS_BRANCH=4.57.2
ARG INSTALL_PYREALSENSE2=false
ARG UID=1000
ARG GID=1000
ARG SCRIPTS_DEST=/home/user/ros_ws/scripts

ENV DEBIAN_FRONTEND=noninteractive

# Use the Jetson AI Lab custom PyPI index (new URL) for platform-specific wheels
# This prevents pip from trying the old .dev domain seen in the environment.
ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/simple
ENV PIP_TRUSTED_HOST=pypi.jetson-ai-lab.io

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Fix expired ROS 2 key and apt sources using the idempotent helper in docker/
# Install minimal tools then copy and invoke the helper script which writes the
# keyring into /usr/share/keyrings and the signed-by apt sources.list entry.
RUN apt-get update && apt-get install -y curl lsb-release && \
    rm -f /etc/apt/trusted.gpg.d/ros-archive-keyring.gpg /usr/share/keyrings/ros-archive-keyring.gpg || true

# Copy idempotent keyring installer from the build context and run it
COPY --chmod=0755 docker/add_ros_keyring.sh /tmp/add_ros_keyring.sh
RUN /tmp/add_ros_keyring.sh --distro jazzy --repo-url http://packages.ros.org/ros2/ubuntu && apt-get update

# Step 2: Remove conflicting OpenCV packages completely
RUN apt-get update && \
    apt-get purge -y opencv-dev 'libopencv-*' 'python3-opencv' || true && \
    apt-get autoremove -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Step 3: Install build dependencies, ROS jazzy packages, and required X11 dev libs
# NOTE: add Dpkg::Options to allow dpkg to overwrite files from conflicting OpenCV packages as a fallback
RUN apt-get update && \
    apt-get -o Dpkg::Options::="--force-overwrite" install -y --no-install-recommends \
    build-essential cmake git wget curl \
    libssl-dev libusb-1.0-0-dev libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    libx11-dev libxrandr-dev libxi-dev libxcursor-dev libxinerama-dev \
    udev python3-pip python3-dev python3-colcon-common-extensions python3-rosdep python3-argcomplete \
    iputils-ping net-tools \
    ros-jazzy-ros2cli ros-jazzy-ros2pkg ros-jazzy-ros2run \
    ros-jazzy-rqt ros-jazzy-rqt-common-plugins \
    ros-jazzy-rclcpp ros-jazzy-rclpy ros-jazzy-sensor-msgs ros-jazzy-cv-bridge ros-jazzy-tf2-ros \
    ros-jazzy-image-transport ros-jazzy-camera-info-manager ros-jazzy-compressed-image-transport && \
    rm -rf /var/lib/apt/lists/* && apt-get clean

# Install ROS Jazzy packages separately
RUN apt-get update && apt-get install -y --no-install-recommends \
ros-jazzy-ros2cli ros-jazzy-ros2pkg ros-jazzy-ros2run \
ros-jazzy-rqt ros-jazzy-rqt-common-plugins \
    ros-jazzy-rclcpp ros-jazzy-rclpy ros-jazzy-sensor-msgs ros-jazzy-cv-bridge ros-jazzy-tf2-ros \
    ros-jazzy-image-transport ros-jazzy-camera-info-manager ros-jazzy-compressed-image-transport && \
    rm -rf /var/lib/apt/lists/* && apt-get clean


# Initialize rosdep for dependency resolution
RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init || true; fi && rosdep update

# Clone and build librealsense from source
RUN git clone --branch ${LIBREALSENSE_VERSION} --depth=1 https://github.com/IntelRealSense/librealsense.git /tmp/librealsense

WORKDIR /tmp/librealsense
RUN mkdir -p build && cd build && \
    cmake .. \
      -DBUILD_EXAMPLES=ON \
      -DFORCE_RSUSB_BACKEND=ON \
      -DBUILD_WITH_CUDA=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_PYTHON_BINDINGS=ON \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 \
      -DPYTHON_INSTALL_DIR=$(python3 -c 'import sys; print(f"/usr/lib/python{sys.version_info.major}.{sys.version_info.minor}/dist-packages")')

WORKDIR /tmp/librealsense/build
RUN make -j$(nproc)

RUN make install && \
    # Copy any built librealsense tools (rs-enumerate-devices, etc.) into /usr/local/bin
    if [ -d /tmp/librealsense/build/tools ]; then \
        find /tmp/librealsense/build/tools -type f -executable -exec cp {} /usr/local/bin/ \; || true; \
        chmod a+rx /usr/local/bin/rs-* || true; \
    fi && \
    cp /tmp/librealsense/config/99-realsense-libusb.rules /etc/udev/rules.d/ || true && \
    rm -rf /tmp/librealsense

ENV PATH="/usr/local/bin:${PATH}"

# Ensure system linker and pkg-config can find the librealsense we built in /usr/local
RUN ldconfig || true
# Define empty defaults to avoid Docker build warnings when expanding these variables
ENV PKG_CONFIG_PATH=""
ENV CMAKE_PREFIX_PATH=""
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"
ENV PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:${PKG_CONFIG_PATH}"
ENV CMAKE_PREFIX_PATH="/usr/local:${CMAKE_PREFIX_PATH}"

# Optional pyrealsense2 wheel install (use custom index if set)
RUN if [ "${INSTALL_PYREALSENSE2}" = "true" ]; then \
            python3 -m pip install --no-cache-dir --index-url ${PIP_INDEX_URL} --trusted-host ${PIP_TRUSTED_HOST} pyrealsense2; \
        fi

# Verify pyrealsense2 import (non-fatal)
RUN python3 - <<EOF || true
try:
    import pyrealsense2 as rs
    print("pyrealsense2 import OK:", getattr(rs, '__version__', 'unknown'))
except Exception as e:
    print("pyrealsense2 import failed (expected on some Jetson builds without wheel):", e)
EOF

# Install additional native build deps and python packages required by upstream
# test scripts (e.g. numpy-quaternion -> import name `quaternion`). Install via
# pip into the image in a single RUN to reduce layers and clean apt lists.
RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config libatlas-base-dev libopenblas-dev liblapack-dev && \
    python3 -m pip install --upgrade pip setuptools wheel && \
    python3 -m pip install --no-cache-dir --index-url ${PIP_INDEX_URL} --trusted-host ${PIP_TRUSTED_HOST} numpy-quaternion && \
    rm -rf /var/lib/apt/lists/* || true

# Create non-root user with matching UID/GID to avoid permission issues
# Step 8: Create a non-root user matching host UID/GID (robust against existing UID/GID)
RUN set -eux; \
    # if the requested GID doesn't exist create a 'user' group with that GID
    if ! getent group "${GID}" >/dev/null 2>&1; then \
        groupadd -g "${GID}" user; \
    else \
        echo "GID ${GID} already exists; skipping groupadd"; \
    fi; \
    # if a 'user' account doesn't already exist, create it
    if ! id -u user >/dev/null 2>&1; then \
        if getent passwd "${UID}" >/dev/null 2>&1; then \
            # UID already exists (assigned to another username) — create 'user' with the same UID
            # use -o to allow non-unique UID so container has the expected numeric UID
            useradd -m -o -u "${UID}" -g "${GID}" -s /bin/bash user; \
        else \
            # UID not present, create normally
            useradd -m -u "${UID}" -g "${GID}" -s /bin/bash user; \
        fi; \
    else \
        echo "user account already exists; skipping useradd"; \
    fi

# Step 9: Setup ROS workspace ownership and clone realsense-ros repo
# Create workspace dirs and set ownership in a robust way when UID/GID may already exist
RUN set -eux; \
    mkdir -p /home/user/ros_ws/src; \
    # ensure a scripts directory exists for runtime validators and examples
    mkdir -p /home/user/ros_ws/scripts; \
    # Resolve the username to use: prefer 'user' (created earlier), otherwise use the username that already has UID
    if id -u user >/dev/null 2>&1; then \
        TARGET_USER="user"; \
    elif getent passwd "${UID}" >/dev/null 2>&1; then \
        TARGET_USER="$(getent passwd "${UID}" | cut -d: -f1)"; \
    else \
        # fallback: create 'user' if it does not exist (should be handled earlier, but guard here)
        useradd -m -u "${UID}" -g "${GID}" -s /bin/bash user; \
        TARGET_USER="user"; \
    fi; \
    # Resolve the group name for the numeric GID, fall back to numeric GID if necessary
    if getent group "${GID}" >/dev/null 2>&1; then \
        TARGET_GROUP="$(getent group "${GID}" | cut -d: -f1)"; \
    else \
        TARGET_GROUP="${GID}"; \
    fi; \
    echo "Setting ownership: ${TARGET_USER}:${TARGET_GROUP} on /home/user/ros_ws"; \
    chown -R "${TARGET_USER}:${TARGET_GROUP}" /home/user/ros_ws || true; \
    chown -R "${TARGET_USER}:${TARGET_GROUP}" /home/user/ros_ws/scripts || true

WORKDIR /home/user/ros_ws/src

# Clone realsense-ros repo. Prefer NVIDIA-ISAAC-ROS fork if it has the requested branch,
# otherwise fall back to IntelRealSense upstream (tag/branch name is in ${REALSENSE_ROS_BRANCH}).
RUN set -eux; \
    # NVIDIA fork does not contain the requested tag/branch for 4.57.2; clone upstream IntelRealSense tag instead
    git clone --branch "${REALSENSE_ROS_BRANCH}" --depth=1 https://github.com/IntelRealSense/realsense-ros.git

WORKDIR /home/user/ros_ws
RUN apt-get update && rosdep install --from-paths src --ignore-src -r -y
RUN chown -R ${UID}:${GID} /home/user/ros_ws || true
# Copy local test scripts into the image at the configurable destination and set ownership
COPY --chown=${UID}:${GID} --chmod=0644 scripts/ ${SCRIPTS_DEST}/
# Remove any conflicting apt-installed librealsense or realsense ROS packages so the
# workspace-built driver is used at runtime. Do this as root before switching user.
RUN apt-get update && apt-get purge -y 'librealsense2*' 'realsense2*' 'ros-jazzy-realsense*' || true && rm -rf /var/lib/apt/lists/* || true

# Ensure the ROS setup scripts are readable/executable by root (and others).
RUN if [ -d /opt/ros/jazzy ]; then chmod -R a+rX /opt/ros/jazzy || true; fi
# Build the workspace as root to avoid permission errors when install steps
# attempt to change permissions under /opt/ros or other system paths. Force a
# clean build so the driver is compiled against /usr/local librealsense.
RUN bash -lc 'export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH} && \
    export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:${PKG_CONFIG_PATH} && \
    export CMAKE_PREFIX_PATH=/usr/local:${CMAKE_PREFIX_PATH} && \
    cd /home/user/ros_ws && rm -rf build install log || true && \
    source /opt/ros/jazzy/setup.sh && colcon build --symlink-install'

# Switch to non-root user after the workspace is built
USER user
ENV HOME=/home/user
WORKDIR /home/user/ros_ws

# Copy a small entrypoint that sources the workspace or system ROS and exports LD_LIBRARY_PATH
## Copy entrypoint; use COPY --chmod if supported by the builder to set mode, otherwise copy as-is
COPY --chmod=0755 docker/entrypoint.sh /usr/local/bin/realsense_entrypoint.sh
# Some builders may ignore chmod; invoke the script through bash so it runs even if not executable
ENTRYPOINT ["/bin/bash", "/usr/local/bin/realsense_entrypoint.sh"]

# NOTE: kernel modules and udev rules for librealsense must be installed on the host
# Firmware compatibility: D400-series firmware 5.17.0.10 or later is recommended
# for librealsense SDK >= v2.56.5 (we use v2.57.2). Ensure camera firmware on the
# host matches these requirements; firmware cannot be updated from inside the container.

