# RealSense VSLAM readiness

This document explains the validator scripts in `validators/` and how their
JSON outputs map to VSLAM readiness checks used by CI or manual inspection.

Validators
----------
- `validate_realsense_ros.py` (ROS-focused): subscribes to camera/image, camera_info,
  and IMU topics when available. Checks include:
  - Topic presence and message types
  - Frame rate estimates for color/depth/infra/imu
  - Timestamp monotonicity and inter-topic synchronization checks
  - CameraInfo validity and intrinsic presence
  - Basic image-content sanity checks (non-empty frames)

- `validate_realsense_plus.py` (VSLAM readiness): extends the ROS validator with
  additional checks useful for VSLAM readiness:
  - Color+gray frame rate and exposure/brightness heuristics
  - Depth-to-color alignment sampling
  - IMU-to-image timestamp jitter checks and sensor rate validation
  - Summary `overall_ok` boolean and `fail_reasons` list for CI scraping

JSON output contract
--------------------
Validators write a JSON object containing:

- `meta`: metadata such as `timestamp`, `image_serial`, `image_fw_version`, and `image_port`.
- `result`: top-level results including:
  - `overall_ok`: boolean, true only if all critical checks passed
  - `validity_ok`: boolean, true if camera streams and CameraInfo are valid
  - `frame_rates`: per-stream estimated frequencies (hz)
  - `fail_reasons`: array of short strings explaining failed critical checks
  - `notes`: optional human-readable notes

Example usage
-------------
- After running the wrapper/runner, find the per-run folder under `realsense_test_outputs/wrapper/<TIMESTAMP>/`.
- The runner prefers `validate_realsense_plus_*` files and copies both `.log` and `.json` into the per-run folder.
- Use `jq` to inspect the JSON quickly, for example:

```bash
jq .result.overall_ok realsense_test_outputs/wrapper/<TIMESTAMP>/validate_realsense_plus_*.json
```

CI integration guidance
-----------------------
- CI systems should fail the pipeline if `result.overall_ok` is `false`.
- For flaky hardware or warm-up conditions, consider running the validator twice
  and allowing one warm-up run to stabilize exposure and auto-exposure settings.
- Store the full JSON artifact alongside build logs to facilitate offline
  debugging.

Next steps / Improvements
-------------------------
- Add a short unit-test harness for validators that feeds pre-recorded bag files
  or saved frames to exercise edge cases (e.g., missing IMU, zero-depth frames).
- Provide an optional `--strict` mode that treats non-critical checks as failures
  for stricter CI gating.

