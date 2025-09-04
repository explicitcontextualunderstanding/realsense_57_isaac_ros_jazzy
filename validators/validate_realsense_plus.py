#!/usr/bin/env python3
"""
validate_realsense_plus.py

An enhanced ROS 2 (rclpy) smoke test for the realsense2_camera driver.
Subscribes to standard RealSense topics and performs deep validation to ensure
the driver is suitable for robust applications like VSLAM.

This script validates:
- Reception of all required topics (color, depth, caminfo, imu).
- Minimum data frequency for all streams.
- Synchronization of timestamps between streams.
- Sanity of IMU data content.
- Existence of TF transforms between camera frames.
- **NEW: Image Content Validity:** Detects black or frozen video streams.
- **NEW: CameraInfo Validity:** Checks for valid, non-zero intrinsic data.
- **NEW: Timestamp Monotonicity:** Ensures timestamps are always increasing.
- **NEW: Frame ID Consistency:** Verifies that image and caminfo frames match.
"""

import argparse
import sys
import time
import math
import json
import hashlib

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo, Imu
import numpy as np


def _ros_time_to_secs(rostime) -> float:
    """Convert ROS2 builtin_interfaces/Time to float seconds."""
    if not rostime:
        return 0.0
    sec = getattr(rostime, 'sec', None)
    nsec = getattr(rostime, 'nanosec', None)
    if sec is None or nsec is None:
        sec = getattr(rostime, 'secs', 0)
        nsec = getattr(rostime, 'nsecs', 0)
    return float(sec) + float(nsec) * 1e-9


class RSValidateNode(Node):
    def __init__(self, color_topic, depth_topic, caminfo_topic, grayscale_topic, imu_topic, enable_imu_check, timeout, sync_threshold_ms, min_image_freq, min_imu_freq, check_tf=False, tf_timeout=1.0):
        super().__init__('rs_validate_plus')
        self.color_topic = color_topic
        self.depth_topic = depth_topic
        self.caminfo_topic = caminfo_topic
        self.grayscale_topic = grayscale_topic
        self.imu_topic = imu_topic
        self.enable_imu_check = enable_imu_check
        self.timeout = timeout
        self.sync_threshold_ms = sync_threshold_ms
        self.min_image_freq = min_image_freq
        self.min_imu_freq = min_imu_freq

        # latest messages
        self.color_msg = None
        self.depth_msg = None
        self.caminfo_msg = None
        self.gray_msg = None
        self.imu_msg = None

        # counts and timing for frequency calculation
        self.color_count = 0
        self.depth_count = 0
        self.gray_count = 0
        self.imu_count = 0
        self.start_time = None

        # last timestamps (float seconds)
        self.last_color_ts = 0.0
        self.last_depth_ts = 0.0
        self.last_gray_ts = 0.0
        self.last_imu_ts = 0.0
        # frame ids (from message headers)
        self.color_frame_id = None
        self.depth_frame_id = None
        self.caminfo_frame_id = None

        ## NEW ##: State for enhanced validity checks
        self.validity_failures = []
        self.last_color_hash = None
        self.last_depth_hash = None
        self.frozen_color_streak = 0
        self.black_color_streak = 0

        # TF check
        self.check_tf = check_tf
        self.tf_timeout = tf_timeout
        self._tf_available = False
        self._tf_buffer = None
        self._tf_listener = None
        if self.check_tf:
            try:
                import tf2_ros
                self._tf_buffer = tf2_ros.Buffer()
                self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
                self._tf_available = True
            except Exception:
                self.get_logger().warning('TF2 (tf2_ros) not available; --check-tf will be skipped')

        # subscriptions
        self.create_subscription(Image, self.color_topic, self.color_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(CameraInfo, self.caminfo_topic, self.caminfo_cb, 10)
        if self.grayscale_topic:
            self.create_subscription(Image, self.grayscale_topic, self.gray_cb, 10)
        if self.enable_imu_check and self.imu_topic:
            self.create_subscription(Imu, self.imu_topic, self.imu_cb, 50)

    def _record_failure(self, reason):
        """Helper to record a unique failure reason."""
        if reason not in self.validity_failures:
            self.validity_failures.append(reason)
            self.get_logger().error(f"VALIDATION FAILURE: {reason}")

    def color_cb(self, msg: Image):
        if self.start_time is None:
            self.start_time = time.time()
        if not self.color_msg:
            self.get_logger().info(f'Received first color image: {msg.width}x{msg.height} enc={msg.encoding}')

        ## NEW ##: Timestamp Monotonicity Check
        new_ts = _ros_time_to_secs(msg.header.stamp)
        if self.last_color_ts > 0 and new_ts <= self.last_color_ts:
            self._record_failure('color_timestamp_non_monotonic')
        self.last_color_ts = new_ts

        ## NEW ##: Image Content Validity Checks
        # Black Frame Check
        try:
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            if img_data.size > 0 and np.mean(img_data) < 2.0:
                self.black_color_streak += 1
            else:
                self.black_color_streak = 0
            if self.black_color_streak > 5:
                self._record_failure('color_stream_is_black')
        except Exception:
            # If we cannot interpret buffer, record but keep going
            self._record_failure('color_data_unreadable')

        # Frozen Frame Check (hash of raw payload)
        try:
            current_hash = hashlib.sha1(msg.data).hexdigest()
            if self.last_color_hash is not None and current_hash == self.last_color_hash:
                self.frozen_color_streak += 1
            else:
                self.frozen_color_streak = 0
            if self.frozen_color_streak > 5:
                self._record_failure('color_stream_is_frozen')
            self.last_color_hash = current_hash
        except Exception:
            self._record_failure('color_hash_failed')

        ## NEW ##: Frame ID Consistency Check
        self.color_frame_id = msg.header.frame_id
        if self.caminfo_frame_id and self.color_frame_id != self.caminfo_frame_id:
            self._record_failure('color_caminfo_frame_id_mismatch')

        self.color_msg = msg
        self.color_count += 1

    def depth_cb(self, msg: Image):
        if self.start_time is None:
            self.start_time = time.time()
        if not self.depth_msg:
            self.get_logger().info(f'Received first depth image: {msg.width}x{msg.height} enc={msg.encoding}')

        ## NEW ##: Timestamp Monotonicity Check
        new_ts = _ros_time_to_secs(msg.header.stamp)
        if self.last_depth_ts > 0 and new_ts <= self.last_depth_ts:
            self._record_failure('depth_timestamp_non_monotonic')
        self.last_depth_ts = new_ts
        
        self.depth_frame_id = msg.header.frame_id
        self.depth_msg = msg
        self.depth_count += 1

    def gray_cb(self, msg: Image):
        if self.start_time is None:
            self.start_time = time.time()
        if not self.gray_msg:
            self.get_logger().info(f'Received first grayscale image: {msg.width}x{msg.height} enc={msg.encoding}')

        ## NEW ##: Timestamp Monotonicity Check
        new_ts = _ros_time_to_secs(msg.header.stamp)
        if self.last_gray_ts > 0 and new_ts <= self.last_gray_ts:
            self._record_failure('gray_timestamp_non_monotonic')
        self.last_gray_ts = new_ts
        
        self.gray_msg = msg
        self.gray_count += 1
        
    def caminfo_cb(self, msg: CameraInfo):
        if not self.caminfo_msg:
            self.get_logger().info(f'Received first camera_info on topic {self.caminfo_topic}')
            
            ## NEW ##: CameraInfo Content Validity Check
            # K[0] is fx, K[4] is fy. If they are zero, calibration is invalid.
            try:
                if msg.k[0] == 0.0 or msg.k[4] == 0.0:
                    self._record_failure('caminfo_intrinsics_are_zero')
            except Exception:
                self._record_failure('caminfo_unreadable')
        
        self.caminfo_frame_id = msg.header.frame_id
        self.caminfo_msg = msg

    def imu_cb(self, msg: Imu):
        if self.start_time is None:
            self.start_time = time.time()
        if not self.imu_msg:
            self.get_logger().info('Received first IMU message')
        
        ## NEW ##: Timestamp Monotonicity Check
        new_ts = _ros_time_to_secs(msg.header.stamp)
        if self.last_imu_ts > 0 and new_ts <= self.last_imu_ts:
            self._record_failure('imu_timestamp_non_monotonic')
        self.last_imu_ts = new_ts
        
        self.imu_msg = msg
        self.imu_count += 1

    def run(self):
        self.get_logger().info(f"Waiting for messages for up to {self.timeout} seconds...")
        end_time = time.time() + self.timeout
        
        # Main validation loop
        while time.time() < end_time:
            rclpy.spin_once(self, timeout_sec=0.1)

            # Check for success condition: all required messages have been received at least once
            imu_ok = not (self.enable_imu_check and self.imu_topic) or self.imu_msg
            if self.color_msg and self.depth_msg and self.caminfo_msg and imu_ok:
                self.get_logger().info('Initial reception SUCCESS: All required topics received.')
                # Continue spinning for a bit more to gather better frequency stats
                measure_end = time.time() + 2.0
                while time.time() < measure_end:
                    rclpy.spin_once(self, timeout_sec=0.1)
                return True
        
        # If loop finishes, it means we timed out
        return False

    def evaluate(self):
        if self.start_time is None:
            self.get_logger().error("No messages were received at all. Cannot evaluate.")
            return {
                'reception_ok': False,
                'validity_ok': False,
                'validity_notes': 'no_messages_received',
            }

        elapsed = max(1e-6, time.time() - self.start_time)
        color_freq = self.color_count / elapsed
        depth_freq = self.depth_count / elapsed
        gray_freq = self.gray_count / elapsed
        imu_freq = self.imu_count / elapsed

        # Timestamp sync check
        ts = [t for t in [self.last_color_ts, self.last_depth_ts, self.last_gray_ts] if t > 0]
        if self.enable_imu_check and self.imu_topic and self.last_imu_ts > 0:
            ts.append(self.last_imu_ts)

        sync_ok = True
        max_offset_ms = None
        if len(ts) >= 2:
            max_offset_ms = (max(ts) - min(ts)) * 1000.0
            sync_ok = max_offset_ms <= self.sync_threshold_ms

        # IMU sanity checks
        imu_ok = True
        imu_notes = ''
        if self.enable_imu_check and self.imu_topic:
            if not self.imu_msg:
                imu_ok = False
                imu_notes = 'no_imu_messages_received'
            else:
                imu = self.imu_msg
                vals = [
                    imu.angular_velocity.x, imu.angular_velocity.y, imu.angular_velocity.z,
                    imu.linear_acceleration.x, imu.linear_acceleration.y, imu.linear_acceleration.z,
                ]
                if not all(math.isfinite(v) for v in vals):
                    imu_ok = False
                    imu_notes = 'imu_contains_non_finite_values'

        # TF check
        tf_ok = None
        tf_notes = ''
        if self.check_tf and self._tf_available and self.color_frame_id and self.depth_frame_id:
            try:
                t = rclpy.time.Time()
                tf_ok = self._tf_buffer.can_transform(
                    self.color_frame_id, self.depth_frame_id, t, timeout=Duration(seconds=self.tf_timeout))
            except Exception as e:
                tf_ok = False
                tf_notes = str(e)

        summary = {
            'reception_ok': bool(self.color_msg and self.depth_msg and self.caminfo_msg and (not (self.enable_imu_check and self.imu_topic) or self.imu_msg)),
            'color_freq_hz': round(color_freq, 2),
            'depth_freq_hz': round(depth_freq, 2),
            'gray_freq_hz': round(gray_freq, 2),
            'imu_freq_hz': round(imu_freq, 2),
            'sync_ok': sync_ok,
            'max_offset_ms': round(max_offset_ms, 2) if max_offset_ms is not None else None,
            'tf_ok': tf_ok,
            'tf_notes': tf_notes,
            'imu_ok': imu_ok,
            'imu_notes': imu_notes,
            'validity_ok': len(self.validity_failures) == 0,
            'validity_notes': '; '.join(self.validity_failures) if self.validity_failures else 'all_ok',
        }
        return summary


def main(argv=None):
    parser = argparse.ArgumentParser(formatter_class=argparse.RawTextHelpFormatter, description=__doc__)
    parser.add_argument('--color', default='/camera/camera/color/image_raw', help='Color image topic')
    parser.add_argument('--depth', default='/camera/camera/depth/image_rect_raw', help='Depth image topic')
    parser.add_argument('--caminfo', default='/camera/camera/color/camera_info', help='Camera info topic for the color stream')
    parser.add_argument('--grayscale', default=None, help='Grayscale image topic (if published)')
    parser.add_argument('--imu', default=None, help='IMU topic (e.g. /camera/imu)')
    parser.add_argument('--enable-imu-check', action='store_true', help='Enable IMU validation (for D435i, etc.)')
    parser.add_argument('--timeout', type=float, default=10.0, help='Timeout in seconds to wait for messages')
    parser.add_argument('--sync-threshold-ms', type=float, default=20.0, help='Max allowed timestamp offset in ms')
    parser.add_argument('--min-image-freq', type=float, default=15.0, help='Minimum acceptable image frequency (Hz)')
    parser.add_argument('--min-imu-freq', type=float, default=100.0, help='Minimum acceptable IMU frequency (Hz)')
    parser.add_argument('--out-file', default=None, help='Optional path to write JSON summary of the run')
    parser.add_argument('--check-tf', action='store_true', help='Verify TF transforms exist between camera frames')
    parser.add_argument('--tf-timeout', type=float, default=1.0, help='Timeout in seconds for TF lookup')
    args = parser.parse_args(argv)

    rclpy.init()
    node = RSValidateNode(
        args.color, args.depth, args.caminfo, args.grayscale, args.imu,
        args.enable_imu_check, args.timeout, args.sync_threshold_ms,
        args.min_image_freq, args.min_imu_freq,
        check_tf=args.check_tf, tf_timeout=args.tf_timeout
    )
    
    # Run the validation and evaluate results
    node.run()
    summary = node.evaluate()

    # Determine overall pass/fail status
    overall_ok = True
    fail_reasons = []
    
    if not summary.get('reception_ok', False):
        overall_ok = False
        fail_reasons.append('topic_reception_failed')
    if not summary.get('validity_ok', False):
        overall_ok = False
        fail_reasons.append('content_validity_failed')
    if not summary.get('sync_ok', True):
        overall_ok = False
        fail_reasons.append('timestamp_sync_failed')
    if summary.get('color_freq_hz', 0) < args.min_image_freq:
        overall_ok = False
        fail_reasons.append('color_freq_too_low')
    if summary.get('depth_freq_hz', 0) < args.min_image_freq:
        overall_ok = False
        fail_reasons.append('depth_freq_too_low')
    if args.enable_imu_check and args.imu:
        if not summary.get('imu_ok', True):
            overall_ok = False
            fail_reasons.append('imu_sanity_failed')
        if summary.get('imu_freq_hz', 0) < args.min_imu_freq:
            overall_ok = False
            fail_reasons.append('imu_freq_too_low')
    if args.check_tf and not summary.get('tf_ok', True):
        overall_ok = False
        fail_reasons.append('tf_check_failed')

    # Log summary and final result
    print("\n--- Validation Summary ---")
    print(json.dumps(summary, indent=2))
    print("--------------------------")
    if overall_ok:
        node.get_logger().info("OVERALL RESULT: PASS")
    else:
        node.get_logger().error(f"OVERALL RESULT: FAIL. Reasons: {', '.join(fail_reasons)}")
        
    # Write JSON summary if requested
    if args.out_file:
        full_output = {
            'result': {
                'overall_ok': overall_ok,
                'fail_reasons': fail_reasons
            },
            'summary': summary
        }
        try:
            with open(args.out_file, 'w') as fh:
                json.dump(full_output, fh, indent=2)
            node.get_logger().info(f"Wrote JSON summary to {args.out_file}")
        except Exception as e:
            node.get_logger().warning(f'Failed to write out-file {args.out_file}: {e}')

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if overall_ok else 1)


if __name__ == '__main__':
    main()
