# Changelog

All notable changes to this project will be documented in this file.

The format is based on Keep a Changelog, and this project adheres to Semantic Versioning.

## [0.1.0] - 2025-09-05
### Added
- Initial public GHCR image for ROS Jazzy RealSense stack built for Jetson Orin.
- Dockerfile builds and installs Intel librealsense v2.57.2; builds `realsense-ros` 4.57.2.
- Runtime verifier scripts and end-to-end validator wrapper.
- README usage, Buildx caching guidance, and hardware test instructions.
- Manual release guide (`RELEASING.md`) for pushing images from Jetson to GHCR.

### Changed
- Dockerfile OCI label `org.opencontainers.image.source` points to this repository to link GHCR package.

