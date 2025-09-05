Manual Release Guide (Jetson Orin)
==================================

We release images manually from the Jetson (no GitHub Actions builds). This guide captures the exact steps and checks.

Prerequisites
-------------
- Docker (or Podman) configured on the Jetson.
- GitHub Personal Access Token in `GHCR_TOKEN` with scopes:
  - `read:packages` (pull)
  - `write:packages` (push)
- Dockerfile already embeds `org.opencontainers.image.source=https://github.com/explicitcontextualunderstanding/realsense_57_ros_jazzy` so GHCR can link the package to this repo.

Login
------

```bash
printf "%s" "$GHCR_TOKEN" | docker login ghcr.io -u explicitcontextualunderstanding --password-stdin
# If using Podman:
# printf "%s" "$GHCR_TOKEN" | podman login ghcr.io -u explicitcontextualunderstanding --password-stdin
```

Build (if not already built locally)
------------------------------------

```bash
DOCKER_BUILDKIT=1 docker build --progress=plain \
  -t realsense_ros:debug \
  --build-arg BASE_IMAGE=dustynv/ros:jazzy-ros-base-r36.4.0-cu128-24.04 \
  --build-arg INSTALL_PYREALSENSE2=false \
  -f Dockerfile .
```

Tag and push
------------

```bash
# Set version and image base
VERSION=0.1.0
IMAGE_BASE=ghcr.io/explicitcontextualunderstanding/realsense_57_ros_jazzy

# Tag local image to GHCR
docker tag realsense_ros:debug ${IMAGE_BASE}:${VERSION}
docker tag realsense_ros:debug ${IMAGE_BASE}:debug
docker tag realsense_ros:debug ${IMAGE_BASE}:latest

# Push tags
docker push ${IMAGE_BASE}:${VERSION}
docker push ${IMAGE_BASE}:debug
docker push ${IMAGE_BASE}:latest
```

Verify and link
---------------
- Open the package page:
  - https://github.com/users/explicitcontextualunderstanding/packages/container/realsense_57_ros_jazzy
- If the package isn’t listed under the repo, link it: Package settings → Connect repository → select `explicitcontextualunderstanding/realsense_57_ros_jazzy`.
- Adjust visibility to Public if desired.

Post-push test
--------------

```bash
docker pull ${IMAGE_BASE}:${VERSION}
./scripts/run_automated_realsense_test.sh --image ${IMAGE_BASE}:${VERSION} --timeout 20
```

Optional: tag the repository
----------------------------
Create a git tag to correspond to the image version so consumers can map code → image.

```bash
git tag -a v${VERSION} -m "realsense_ros ${VERSION}"
git push origin v${VERSION}
```

Notes
-----
- All image path components must be lowercase for GHCR.
- If `docker login` complains about non-TTY, ensure you’re using the `--password-stdin` form (or use Podman’s equivalent). If a credential helper intercepts, temporarily override `DOCKER_CONFIG` with a minimal config.

