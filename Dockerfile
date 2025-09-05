# syntax=docker/dockerfile:1.6
#---
# name: realsense_ros
# group: hardware
# depends: [cuda, python, cmake, ros2]
# notes:
#   - Builds librealsense from source (pin with LIBREALSENSE_VERSION)
#   - Builds NVIDIA-ISAAC-ROS `realsense-ros` in the same container (REALSENSE_ROS_BRANCH)
#   - Host expected to manage kernel modules (JetsonHacks or similar)
#   - Uses ccache and multi-stage build to speed up rebuilds
#---
ARG BASE_IMAGE=dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04

# =====================
# Stage 1: librealsense builder (with ccache)
# =====================
FROM ${BASE_IMAGE} AS builder

LABEL maintainer="you@example.com" \
      org.opencontainers.image.title="realsense_ros" \
      org.opencontainers.image.source="https://github.com/explicitcontextualunderstanding/realsense_57_ros_jazzy"

# Use matching minor/patch releases for SDK and ROS driver to ensure compatibility.
# Intel librealsense v2.57.2 matches realsense-ros 4.57.2 (see releases: v2.57.2 / 4.57.2)
ARG LIBREALSENSE_VERSION=v2.57.2
ARG REALSENSE_ROS_BRANCH=4.57.2
ARG INSTALL_PYREALSENSE2=false
ARG UID=1000
ARG GID=1000
ARG SCRIPTS_DEST=/home/user/ros_ws/scripts
ARG ROS_APT_DISTRO=""

ENV DEBIAN_FRONTEND=noninteractive

# Use the Jetson AI Lab custom PyPI index (new URL) for platform-specific wheels
# This prevents pip from trying the old .dev domain seen in the environment.
ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/simple
ENV PIP_TRUSTED_HOST=pypi.jetson-ai-lab.io

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Fix expired ROS 2 key and apt sources in builder stage as well (idempotent)
# This base image includes ROS apt sources which may have an expired key; refresh it here so apt works.
RUN apt-get update && apt-get install -y curl lsb-release gnupg ca-certificates && \
    rm -f /etc/apt/trusted.gpg.d/ros-archive-keyring.gpg /usr/share/keyrings/ros-archive-keyring.gpg || true
COPY --chmod=0755 docker/add_ros_keyring.sh /tmp/add_ros_keyring.sh
RUN set -eux; \
    if [ -z "${ROS_APT_DISTRO:-}" ]; then \
        DISTRO="$(lsb_release -cs)"; \
    else \
        DISTRO="${ROS_APT_DISTRO}"; \
    fi; \
    echo "[builder] Using ROS apt distro: ${DISTRO}"; \
    /tmp/add_ros_keyring.sh --distro "${DISTRO}" --repo-url http://packages.ros.org/ros2/ubuntu && apt-get update

## Only minimal deps for librealsense build are needed in the builder
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget curl \
    ccache \
    libssl-dev libusb-1.0-0-dev libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    libx11-dev libxrandr-dev libxi-dev libxcursor-dev libxinerama-dev \
    udev python3 python3-pip python3-dev && \
    rm -rf /var/lib/apt/lists/* && apt-get clean

## Clone and build librealsense from source
RUN git clone --branch ${LIBREALSENSE_VERSION} --depth=1 https://github.com/IntelRealSense/librealsense.git /tmp/librealsense

WORKDIR /tmp/librealsense
RUN mkdir -p build && cd build && \
    cmake .. \
      -DBUILD_EXAMPLES=ON \
      -DFORCE_RSUSB_BACKEND=ON \
      -DBUILD_WITH_CUDA=ON \
      -DCMAKE_BUILD_TYPE=Release \
      -DBUILD_PYTHON_BINDINGS=ON \
      -DCMAKE_C_COMPILER_LAUNCHER=ccache \
      -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
      -DPYTHON_EXECUTABLE=/usr/bin/python3 \
      -DPYTHON_INSTALL_DIR=$(python3 -c 'import sys; print(f"/usr/lib/python{sys.version_info.major}.{sys.version_info.minor}/dist-packages")')

WORKDIR /tmp/librealsense/build
RUN --mount=type=cache,target=/root/.cache/ccache make -j$(nproc)

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

## Keep PATH and linker caches after install in this stage
ENV PATH="/usr/local/bin:${PATH}"
RUN ldconfig || true

# =====================
# Stage 2: runtime with ROS + realsense-ros, copy SDK from builder
# =====================
FROM ${BASE_IMAGE}

LABEL maintainer="you@example.com" \
      org.opencontainers.image.title="realsense_ros" \
      org.opencontainers.image.source="https://github.com/explicitcontextualunderstanding/realsense_57_ros_jazzy"

ARG LIBREALSENSE_VERSION=v2.57.2
ARG REALSENSE_ROS_BRANCH=4.57.2
ARG INSTALL_PYREALSENSE2=false
ARG UID=1000
ARG GID=1000
ARG SCRIPTS_DEST=/home/user/ros_ws/scripts
ARG ROS_APT_DISTRO=""

ENV DEBIAN_FRONTEND=noninteractive
ENV PIP_INDEX_URL=https://pypi.jetson-ai-lab.io/simple
ENV PIP_TRUSTED_HOST=pypi.jetson-ai-lab.io
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

# Fix expired ROS 2 key and apt sources using the idempotent helper in docker/
RUN apt-get update && apt-get install -y curl lsb-release && \
    rm -f /etc/apt/trusted.gpg.d/ros-archive-keyring.gpg /usr/share/keyrings/ros-archive-keyring.gpg || true
COPY --chmod=0755 docker/add_ros_keyring.sh /tmp/add_ros_keyring.sh
RUN set -eux; \
    if [ -z "${ROS_APT_DISTRO:-}" ]; then \
        DISTRO="$(lsb_release -cs)"; \
    else \
        DISTRO="${ROS_APT_DISTRO}"; \
    fi; \
    echo "Using ROS apt distro: ${DISTRO}"; \
    /tmp/add_ros_keyring.sh --distro "${DISTRO}" --repo-url http://packages.ros.org/ros2/ubuntu && apt-get update

# Remove conflicting OpenCV (idempotent)
RUN apt-get update && \
    apt-get purge -y opencv-dev 'libopencv-*' 'python3-opencv' || true && \
    apt-get autoremove -y && \
    apt-get clean && rm -rf /var/lib/apt/lists/*

# Install ROS + runtime deps
RUN apt-get update && \
    apt-get -o Dpkg::Options::="--force-overwrite" install -y --no-install-recommends \
    build-essential cmake git wget curl \
    iputils-ping net-tools \
    python3-pip python3-dev python3-colcon-common-extensions python3-rosdep python3-argcomplete \
    ros-jazzy-ros2cli ros-jazzy-ros2pkg ros-jazzy-ros2run \
    ros-jazzy-rqt ros-jazzy-rqt-common-plugins \
    ros-jazzy-rclcpp ros-jazzy-rclpy ros-jazzy-sensor-msgs ros-jazzy-cv-bridge ros-jazzy-tf2-ros \
    ros-jazzy-image-transport ros-jazzy-camera-info-manager ros-jazzy-compressed-image-transport && \
    rm -rf /var/lib/apt/lists/* && apt-get clean

# Copy the built SDK from the builder stage
COPY --from=builder /usr/local/ /usr/local/
ENV LD_LIBRARY_PATH="/usr/local/lib:${LD_LIBRARY_PATH}"
ENV PKG_CONFIG_PATH="/usr/local/lib/pkgconfig:${PKG_CONFIG_PATH}"
ENV CMAKE_PREFIX_PATH="/usr/local:${CMAKE_PREFIX_PATH}"
RUN ldconfig || true

# Optional: try installing pyrealsense2 wheel
RUN if [ "${INSTALL_PYREALSENSE2}" = "true" ]; then \
            python3 -m pip install --no-cache-dir --index-url ${PIP_INDEX_URL} --trusted-host ${PIP_TRUSTED_HOST} pyrealsense2; \
        fi || true

# Verify pyrealsense2 import (non-fatal)
RUN python3 - <<'EOF' || true
try:
    import pyrealsense2 as rs
    print("pyrealsense2 import OK:", getattr(rs, '__version__', 'unknown'))
except Exception as e:
    print("pyrealsense2 import failed (expected on some Jetson builds without wheel):", e)
EOF

# Extra native/python deps used by validators
RUN apt-get update && apt-get install -y --no-install-recommends \
    pkg-config libatlas-base-dev libopenblas-dev liblapack-dev && \
    python3 -m pip install --upgrade pip setuptools wheel && \
    python3 -m pip install --no-cache-dir --index-url ${PIP_INDEX_URL} --trusted-host ${PIP_TRUSTED_HOST} numpy-quaternion && \
    rm -rf /var/lib/apt/lists/* || true

# Create non-root user matching host UID/GID
RUN set -eux; \
    if ! getent group "${GID}" >/dev/null 2>&1; then \
        groupadd -g "${GID}" user; \
    fi; \
    if ! id -u user >/dev/null 2>&1; then \
        if getent passwd "${UID}" >/dev/null 2>&1; then \
            useradd -m -o -u "${UID}" -g "${GID}" -s /bin/bash user; \
        else \
            useradd -m -u "${UID}" -g "${GID}" -s /bin/bash user; \
        fi; \
    fi

# Setup ROS workspace and build realsense-ros
RUN set -eux; \
    mkdir -p /home/user/ros_ws/src /home/user/ros_ws/scripts; \
    chown -R ${UID}:${GID} /home/user/ros_ws
WORKDIR /home/user/ros_ws/src
RUN git clone --branch "${REALSENSE_ROS_BRANCH}" --depth=1 https://github.com/IntelRealSense/realsense-ros.git
WORKDIR /home/user/ros_ws
RUN apt-get update && rosdep init || true && rosdep update && rosdep install --from-paths src --ignore-src -r -y
RUN bash -lc 'export LD_LIBRARY_PATH=/usr/local/lib:${LD_LIBRARY_PATH} && \
    export PKG_CONFIG_PATH=/usr/local/lib/pkgconfig:${PKG_CONFIG_PATH} && \
    export CMAKE_PREFIX_PATH=/usr/local:${CMAKE_PREFIX_PATH} && \
    cd /home/user/ros_ws && rm -rf build install log || true && \
    source /opt/ros/jazzy/setup.sh && colcon build --symlink-install'

# Copy local scripts into image
COPY --chown=${UID}:${GID} --chmod=0644 scripts/ ${SCRIPTS_DEST}/
# Ensure scripts remain executable/traversable even if the builder strips exec bits
RUN chmod -R a+rx ${SCRIPTS_DEST} || true

# Entry and user context
USER user
ENV HOME=/home/user
WORKDIR /home/user/ros_ws
COPY --chmod=0755 docker/entrypoint.sh /usr/local/bin/realsense_entrypoint.sh
ENTRYPOINT ["/bin/bash", "/usr/local/bin/realsense_entrypoint.sh"]

# NOTE: kernel modules and udev rules for librealsense must be installed on the host.
# Ensure camera firmware is compatible with the chosen SDK version.
