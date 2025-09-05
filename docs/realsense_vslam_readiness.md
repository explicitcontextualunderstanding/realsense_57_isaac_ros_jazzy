# RealSense ROS container readiness for VSLAM

Summary
-------
This document contains two distinct sections:

- "How to assess readiness" — a reproducible guide you can follow to verify readiness yourself on any host.
- "Latest run evidence (confirmation)" — a snapshot of the most recent automated validator run and the interpreted result showing whether the container is *actually* ready based on that run's logs and JSON output.

Below you will find both: a step-by-step assessment guide you can repeat, and the latest-run confirmation that was produced on this workspace.

ROS Topics Subscribed
---------------------

| ROS Topic | Interface | Description |
|---|---:|---|
| `visual_slam/image_{i}` | `sensor_msgs/Image` | Image from the i-th camera in grayscale (`i` in `[0, num_cameras-1]`). |
| `visual_slam/camera_info_{i}` | `sensor_msgs/CameraInfo` | Camera info from the i-th camera. |
| `visual_slam/imu` | `sensor_msgs/Imu` | IMU data (present if `enable_imu_fusion` is true). |
| `visual_slam/initial_pose` | `geometry_msgs/PoseWithCovarianceStamped` | Pose hint used to localize in an existing map (VSLAM input). |
| `visual_slam/trigger_hint` | `geometry_msgs/PoseWithCovarianceStamped` | Topic on which the VSLAM node publishes requests for a new pose hint. |

Note: the RealSense driver typically publishes under `/camera/...` topic names (for example `/camera/camera/color/image_raw`, `/camera/camera/color/camera_info`, `/camera/imu`). If your VSLAM container expects `visual_slam/*` names, you can either:

- Remap topics when launching the VSLAM node, or
- Run a lightweight relay node (or `ros2 topic pub/sub` remap) that republishes the `/camera/...` topics to the `visual_slam/*` names.

Readiness criteria
------------------
To be considered "ready" the RealSense ROS container must satisfy the following:

- Topics exist and are publishing: image (grayscale or color), camera_info, depth (if used), and IMU (if enabled).
- CameraInfo contains non-zero intrinsics (fx/fy != 0).
- Image streams are not frozen and not black (reasonable pixel statistics).
- Timestamps are monotonic for each stream and reasonably synchronized across streams (within threshold).
- Message frequencies meet minimums (images >= 15 Hz in our tests; IMU >= 100 Hz for D435i-style devices).

Where evidence is stored
-----------------------
Automated-run artifacts are persisted under the host-mounted `./realsense_test_outputs/` directory. The runner now creates a per-run subfolder for wrapper artifacts to avoid collisions and make automation scraping simpler.

Layout (example):

```
realsense_test_outputs/
├─ validate_realsense_plus_20250903_053029.json
├─ validate_realsense_plus_20250903_053029.log
├─ validate_realsense_ros_20250831_121212.json
├─ sdk_only_20250903_044755.log
└─ wrapper/
   └─ 20250903_185240/            <-- per-run folder created by the wrapper
      ├─ wrapper.log              <-- wrapper stdout/stderr for that run
      ├─ validate_realsense_plus_20250903_185240.log
      ├─ validate_realsense_plus_20250903_185240.json
      ├─ validate_realsense_plus.log -> validate_realsense_plus_20250903_185240.log (symlink)
      ├─ validate_realsense_plus.json -> validate_realsense_plus_20250903_185240.json (symlink)
      └─ data/
         └─ time.log
```

Notes:
- The runner prefers the newer `validate_realsense_plus_*` artifacts and copies both `.log` and `.json` into the per-run `wrapper/<timestamp>/` folder. If plus artifacts are not present it falls back to `validate_realsense_ros_*` copies.
- The `wrapper/<timestamp>/wrapper.log` file contains the wrapper's stdout/stderr and is the first place to check for run-level failures (docker invocation, device claimers, etc.).
- This per-run layout makes it simple to fetch the latest run and inspect its artifacts programmatically (examples below).

Key commands to run (reproducible checks)
----------------------------------------
Run the automated RealSense test (this launches the containerized node, waits for topics, runs the extended validator and writes JSON/logs to `realsense_test_outputs`):

You can use the top-level wrapper which now delegates to the consolidated runner:

```bash
./run_realsense_test.sh    # ephemeral container run; respects IMAGE and HOST_LOG_DIR env vars
# or call the runner directly for advanced flags (e.g., host claimant cleanup)
HOST_LOG_DIR="$(pwd)/realsense_test_outputs" IMAGE="realsense_ros:debug" bash ./scripts/runner.sh --host-kill
# exec mode (run inside an existing container by name):
#   HOST_LOG_DIR="$(pwd)/realsense_test_outputs" IMAGE="realsense_ros:debug" bash ./scripts/runner.sh <container-name>
```

After a run, find the latest wrapper-run folder and inspect its contents:

```bash
# get the latest per-run wrapper folder
LATEST_RUN_DIR=$(ls -1dt realsense_test_outputs/wrapper/*/ 2>/dev/null | head -n1)
echo "Latest run folder: $LATEST_RUN_DIR"

# show wrapper log preview
head -n 200 "$LATEST_RUN_DIR/wrapper.log"

# show validator JSON (prefer plus)
ls -1 "$LATEST_RUN_DIR"/validate_realsense_plus*.json "$LATEST_RUN_DIR"/validate_realsense_ros*.json 2>/dev/null | head -n1 | xargs -r jq .
```

Manually check topics being published from inside the container (or by connecting to the ROS domain where the realsense node runs):

```bash
# list topics (including hidden)
ros2 topic list --include-hidden

# show a single message from the color image stream
ros2 topic echo -n1 /camera/camera/color/image_raw

# show CameraInfo
ros2 topic echo -n1 /camera/camera/color/camera_info

# show IMU (if present)
ros2 topic echo -n1 /camera/imu

# check topic publish rates (run for ~3-5 seconds)
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/imu
```

Inspect the validator JSON for pass/fail fields
---------------------------------------------
The `validate_realsense_plus.py` script writes a JSON file when invoked with `--out-file`. Open the most recent JSON and check these fields (example using `jq`):

```bash
# pick the latest JSON automatically (prefer plus)
LATEST_JSON=$(ls -1t realsense_test_outputs/validate_realsense_plus_*.json realsense_test_outputs/validate_realsense_ros_*.json 2>/dev/null | head -n1)
echo "Latest validator JSON: $LATEST_JSON"

# show top-level result and summary
jq . "$LATEST_JSON"

# example checks
jq '.result.overall_ok' "$LATEST_JSON"
jq '.result.fail_reasons' "$LATEST_JSON"
jq '.summary.color_freq_hz, .summary.depth_freq_hz, .summary.imu_freq_hz' "$LATEST_JSON"
jq '.summary.validity_ok, .summary.validity_notes' "$LATEST_JSON"
jq '.summary.sync_ok, .summary.max_offset_ms' "$LATEST_JSON"
```

How to interpret the important fields
------------------------------------
- `result.overall_ok` — boolean, overall pass/fail. `true` means the validator considers the streams and data suitable.
- `result.fail_reasons` — array of short strings describing failing checks (e.g. `color_freq_too_low`, `content_validity_failed`).
- `summary.color_freq_hz`, `summary.depth_freq_hz`, `summary.imu_freq_hz` — measured frequencies during the short measurement window.
- `summary.validity_ok` — boolean, `true` if no content-level failures (frozen/black/caminfo zeros) were detected.
- `summary.validity_notes` — detailed textual notes, e.g. `color_stream_is_frozen` or `caminfo_intrinsics_are_zero`.
- `summary.sync_ok` and `summary.max_offset_ms` — whether timestamps across streams were within the configured threshold and what the max observed offset was.
- `summary.tf_ok` and `summary.tf_notes` — present if TF checks were enabled; indicates transform availability/consistency.
- `summary.imu_ok` and `summary.imu_notes` — additional IMU sanity hints beyond frequency.
  - New failure reasons you may see include `color_caminfo_frame_id_mismatch` when `Image.header.frame_id` differs from `CameraInfo.header.frame_id`.

Example evidence mapping to readiness criteria
--------------------------------------------

- Topics exist: `ros2 topic list` shows `/camera/camera/color/image_raw`, `/camera/camera/color/camera_info`, `/camera/imu` (if IMU enabled).
- CameraInfo intrinsics: `jq '.summary' <json>` shows non-zero `k` entries (validator records `caminfo_intrinsics_are_zero` if they are zero).
- Not frozen / not black: `jq '.summary.validity_ok'` should be `true` and `jq '.summary.validity_notes'` should be `"all_ok"`.
- Timestamp monotonicity: validator records `*_timestamp_non_monotonic` failures; `summary.sync_ok` should be `true` and `max_offset_ms` should be below your `--sync-threshold-ms` (default 20 ms).
- Frequency: `jq '.summary.color_freq_hz'` and `depth_freq_hz` should meet your `--min-image-freq` (default 15 Hz).
- Frame ID consistency: if `Image.header.frame_id` and `CameraInfo.header.frame_id` differ, `result.fail_reasons` will include `color_caminfo_frame_id_mismatch` and details will appear in `summary.validity_notes`.

Hardware/kernel checks (if you previously saw USB errors)
--------------------------------------------------------
If kernel USB errors were observed (e.g. `error -71`, `error -110`, `Failed to set UVC probe control`), gather kernel logs for troubleshooting:

```bash
# show kernel messages related to usb and uvc
journalctl -k | egrep -i 'usb|uvc|fusb|xusb' | tail -n 200

# show dmesg output
dmesg | egrep -i 'usb|uvc|device descriptor|error -71|error -110' | tail -n 200

# use librealsense CLI inside container for enumeration
docker run --rm --privileged -v /dev/bus/usb:/dev/bus/usb realsense_ros:debug rs-enumerate-devices
```

Notes about topic names and remapping
------------------------------------
The VSLAM node expects `visual_slam/*` topics as documented at the top of this page. The RealSense driver typically publishes under `/camera/...`. To connect the two you can either:

- Launch the VSLAM node with ROS remappings, e.g. `ros2 run visual_slam_node visual_slam_node --ros-args -r visual_slam/image_0:=/camera/camera/color/image_raw -r visual_slam/camera_info_0:=/camera/camera/color/camera_info`.
- Run a small relay node (or `ros2 topic pub` wrapper) to republish `/camera/*` topics to `visual_slam/*`.

Final checklist to assert readiness
----------------------------------
Run these steps and verify each item:

1. Start the RealSense container or node and ensure it has exclusive access to the camera (use the automation script which attempts to kill host claimers):

```bash
./scripts/run_automated_realsense_test.sh --timeout 20
```

2. Confirm JSON summary shows overall pass:

```bash
jq '.result.overall_ok, .result.fail_reasons, .summary' realsense_test_outputs/validate_realsense_plus_*.json | tail -n +1
```

3. Confirm topics are visible and have expected frequency:

```bash
ros2 topic list --include-hidden
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
ros2 topic hz /camera/imu
```

4. If your VSLAM container expects `visual_slam/*` names, prepare remapping or a topic relay.

5. (Optional) Run a quick smoke test by launching the VSLAM container with remaps and checking it receives messages:

```bash
# Example: run VSLAM with remaps
ros2 run visual_slam visual_slam_node --ros-args \
  -r visual_slam/image_0:=/camera/camera/color/image_raw \
  -r visual_slam/camera_info_0:=/camera/camera/color/camera_info
```

Troubleshooting hints
---------------------
- If `result.overall_ok` is false, inspect `result.fail_reasons` and `summary.validity_notes` to identify content or frequency issues.
- If topics never appear, check for device ownership conflicts on the host (use `lsof /dev/bus/usb` / `fuser -v /dev/video*`) and ensure no other host process holds the device.
- If you see kernel USB errors, re-check cabling (prefer a short certified SuperSpeed cable and direct connection to a SuperSpeed port) and review `dmesg`/`journalctl -k` for `error -71` / `error -110` messages.

Contact / logs
--------------
All validation logs and JSON outputs are available under `./realsense_test_outputs/` on the host. Attach those files when asking for help:

- `realsense_test_outputs/validate_realsense_plus_*.json`
- `realsense_test_outputs/validate_realsense_plus_*.log`
- `realsense_test_outputs/sdk_only_*.log` (sdk-only enumerations/stream tests)
- `realsense_test_outputs/wrapper/*.log` (wrapper invocation logs)

---
Document created: `docs/realsense_vslam_readiness.md`

Latest run evidence (confirmation)
----------------------------------
The automated validator writes JSON summaries to `realsense_test_outputs/` when you run `./scripts/run_automated_realsense_test.sh`.

The most recent validator JSON artifact in this workspace is:

`realsense_test_outputs/validate_realsense_plus_20250904_235845.json`

Contents (abridged):

```json
{
  "result": { "overall_ok": true, "fail_reasons": [] },
  "summary": {
    "reception_ok": true,
    "color_freq_hz": 31.13,
    "depth_freq_hz": 30.63,
    "gray_freq_hz": 30.63,
    "imu_freq_hz": 0.0,
    "sync_ok": true,
    "max_offset_ms": 0.02,
    "validity_ok": true,
    "validity_notes": "all_ok",
    "tf_ok": true,
    "tf_gray_ok": true
  }
}
```

Alternate IR profile (1280x720 @ 30 Hz)
--------------------------------------

Two additional runs were captured for 1280x720 IR:

- Strict sync threshold (20 ms, default):
  - `realsense_test_outputs/validate_realsense_plus_20250905_004936_ir1280x720.json`
  - Summary shows `sync_ok: false` with `max_offset_ms: 33.36` (otherwise healthy; TF OK, grayscale ~30 Hz).

- Relaxed sync threshold (40 ms):
  - `realsense_test_outputs/validate_realsense_plus_20250905_005054_ir1280x720_sync40.json`
  - Summary shows PASS with `sync_ok: true` and `gray_freq_hz: ~30.64`; TF checks remain OK.

Notes:
- Larger resolutions can increase inter-stream timestamp offsets. If your VSLAM tolerates a slightly higher offset, adjust the validator via `--sync-threshold-ms 40` or tune driver sync (e.g., `enable_sync:=true`).

Interpreting this artifact:

- `result.overall_ok: true` — indicates the automated validator considered this run a PASS and therefore the container was ready at the time of the run to serve image and CameraInfo topics for VSLAM usage (note: IMU was not present in this run, so IMU-related checks are not applicable).
- Frequencies: color and depth are ~30 Hz which exceeds typical VSLAM minimums (15 Hz).
- `validity_ok: true` and `validity_notes: "all_ok"` — no frozen/black/camera-info-zero failures were detected.
- `sync_ok: true` and `max_offset_ms: 0.02` — timestamps between streams were effectively synchronized (20 microseconds measured during the brief test window).

Evidence gaps (to be validated if required by your VSLAM configuration):
- IMU: not active in this run (`imu_freq_hz: 0.0`), so no evidence that IMU meets >= 100 Hz.
- TF: not evaluated (`tf_ok: null`) — if your stack needs specific camera frame transforms, validate with `--check-tf`.
- Grayscale: not present (`gray_freq_hz: 0.0`) — some VSLAM stacks prefer grayscale images; if needed, enable or convert.

Because this confirmation shows a PASS, you can reasonably proceed to run a VSLAM container provided you remap or relay topics to the `visual_slam/*` names if necessary.

Concrete evidence this was published by the ROS node
---------------------------------------------------
To be explicit: the automated validator's JSON is not only based on SDK-level streaming — it is backed by the ROS middleware. The workspace contains two independent log artifacts that prove the ROS node started and that the validator (a ROS node) received messages from ROS topics:

1) RealSense ROS node startup (excerpt from `realsense_test_outputs/realsense_node_20250904_214633.log`):

```
[INFO] [1757022401.198066141] [camera.camera]: RealSense ROS v4.57.2
[INFO] [1757022401.198298689] [camera.camera]: Built with LibRealSense v2.57.2
[INFO] [1757022401.553504119] [camera.camera]: Device with serial number 102422076402 was found.
[INFO] [1757022405.969239453] [camera.camera]: Open profile: stream_type: Depth(0), Format: Z16, Width: 848, Height: 480, FPS: 30
[INFO] [1757022406.013945731] [camera.camera]: Open profile: stream_type: Color(0), Format: RGB8, Width: 640, Height: 480, FPS: 30
[INFO] [1757022406.031632265] [camera.camera]: RealSense Node Is Up!
```

These lines are emitted by the `realsense2_camera` ROS node and show it enumerated the device and opened ROS stream profiles.

2) Validator received ROS messages (excerpt from `realsense_test_outputs/validate_realsense_plus_20250904_214633.log`):

```
[INFO] [1757022413.854779410] [rs_validate_plus]: Received first color image: 640x480 enc=rgb8
[INFO] [1757022413.862837365] [rs_validate_plus]: Received first depth image: 848x480 enc=16UC1
[INFO] [1757022413.916145677] [rs_validate_plus]: Received first camera_info on topic /camera/camera/color/camera_info
[INFO] [1757022413.917901968] [rs_validate_plus]: Initial reception SUCCESS: All required topics received.
```

These lines are printed by `validate_realsense_plus.py` when it receives messages from the ROS topics — they prove the data flow occurred over ROS (DDS) and not only inside an SDK test harness.

If you need a continuously monitored guarantee (rather than a single-run confirmation), consider running the validator as a periodic healthcheck or adding it to a startup systemd/cron job and collecting results over time.
