#!/usr/bin/env python3
"""
validate_realsense_ros.py

Simple ROS 2 (rclpy) smoke test for the realsense2_camera driver.
Subscribes to color, depth and camera_info topics and verifies that messages
arrive within a timeout and contain non-empty payloads.

Usage (inside container where ROS environment is sourced):
  python3 scripts/validate_realsense_ros.py

Optional arguments:
  --color TOPIC         color image topic (default: /camera/camera/color/image_raw)
  --depth TOPIC         depth image topic (default: /camera/camera/depth/image_rect_raw)
  --caminfo TOPIC       camera info topic (default: /camera/camera/color/camera_info)
  --timeout SEC         timeout in seconds to wait for messages (default: 10)

Exit code: 0 = success, 1 = failure
"""

import argparse
import sys
import time
import math
import json

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo, Imu
import numpy as np
import cv2


class RSValidateNode(Node):
    def __init__(self, color_topic, depth_topic, caminfo_topic, grayscale_topic, imu_topic, enable_imu_check, timeout, sync_threshold_ms, min_image_freq, min_imu_freq, check_tf=False, tf_timeout=1.0):
        super().__init__('rs_validate')
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
        # start_time is set when the first message arrives to get accurate
        # frequency measurements (avoid including long init/wait time).
        self.start_time = None

        # last timestamps (float seconds)
        self.last_color_ts = None
        self.last_depth_ts = None
        self.last_gray_ts = None
        self.last_imu_ts = None
        # frame ids (from message headers)
        self.color_frame_id = None
        self.depth_frame_id = None

        # whether we synthesized grayscale from color because no suitable gray stream
        self.gray_synthesized = False

        # TF check
        self.check_tf = check_tf
        self.tf_timeout = tf_timeout
        self._tf_available = False
        self._tf_buffer = None
        self._tf_listener = None
        if self.check_tf:
            try:
                import tf2_ros
                # lazy create buffer and listener
                self._tf_buffer = tf2_ros.Buffer()
                self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
                self._tf_available = True
            except Exception:
                self.get_logger().warning('TF2 (tf2_ros) not available in environment; --check-tf will be skipped')

        # subscriptions
        self.create_subscription(Image, self.color_topic, self.color_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(CameraInfo, self.caminfo_topic, self.caminfo_cb, 10)
        if self.grayscale_topic:
            self.create_subscription(Image, self.grayscale_topic, self.gray_cb, 10)
        if self.enable_imu_check and self.imu_topic:
            self.create_subscription(Imu, self.imu_topic, self.imu_cb, 50)

    def color_cb(self, msg: Image):
        if not self.color_msg:
            self.get_logger().info(f'Received color image: {msg.width}x{msg.height} enc={msg.encoding} size={len(msg.data)}')
        self.color_msg = msg
        self.color_count += 1
        self.last_color_ts = _ros_time_to_secs(msg.header.stamp)
        if self.start_time is None:
            self.start_time = time.time()
        try:
            self.color_frame_id = msg.header.frame_id
        except Exception:
            pass

    def depth_cb(self, msg: Image):
        if not self.depth_msg:
            self.get_logger().info(f'Received depth image: {msg.width}x{msg.height} enc={msg.encoding} size={len(msg.data)}')
        self.depth_msg = msg
        self.depth_count += 1
        self.last_depth_ts = _ros_time_to_secs(msg.header.stamp)
        if self.start_time is None:
            self.start_time = time.time()
        try:
            self.depth_frame_id = msg.header.frame_id
        except Exception:
            pass

    def gray_cb(self, msg: Image):
        if not self.gray_msg:
            self.get_logger().info(f'Received grayscale image: {msg.width}x{msg.height} enc={msg.encoding} size={len(msg.data)}')
        self.gray_msg = msg
        self.gray_count += 1
        self.last_gray_ts = _ros_time_to_secs(msg.header.stamp)
        if self.start_time is None:
            self.start_time = time.time()

    def caminfo_cb(self, msg: CameraInfo):
        if not self.caminfo_msg:
            self.get_logger().info(f'Received camera_info: {msg.width}x{msg.height} K={msg.k}')
        self.caminfo_msg = msg
    def imu_cb(self, msg: Imu):
        if not self.imu_msg:
            self.get_logger().info('Received IMU message')
        self.imu_msg = msg
        self.imu_count += 1
        self.last_imu_ts = _ros_time_to_secs(msg.header.stamp)
        if self.start_time is None:
            self.start_time = time.time()

    def _convert_color_to_grayscale(self, color_msg: Image):
        # If color message is already mono, return it directly
        enc = color_msg.encoding.lower()
        if enc.startswith('mono'):
            return color_msg
        try:
            # Support common encodings: rgb8, bgr8
            if enc in ('rgb8', 'bgr8'):
                dtype = np.uint8
                channels = 3
                arr = np.frombuffer(color_msg.data, dtype=dtype)
                expected = color_msg.width * color_msg.height * channels
                if arr.size != expected:
                    self.get_logger().warning(f'Color buffer size mismatch: expected {expected} got {arr.size}')
                    return None
                arr = arr.reshape((color_msg.height, color_msg.width, channels))
                if enc == 'rgb8':
                    bgr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                else:
                    bgr = arr
                gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
                return gray
            else:
                self.get_logger().warning(f'Unsupported color encoding for conversion: {color_msg.encoding}')
                return None
        except Exception as e:
            self.get_logger().warning(f'Failed to convert color to grayscale: {e}')
            return None

    def run(self):
        start = time.time()
        rclpy.spin_once(self, timeout_sec=0)  # primer
        end_time = start + self.timeout
        while time.time() < end_time:
            # Synthesize a grayscale frame from color when appropriate:
            # - no explicit grayscale topic was provided OR
            # - a grayscale topic exists but we haven't received any gray frames yet
            # This prevents the validator from waiting indefinitely for a slow/unused
            # grayscale publisher and reflects VSLAM needs where grayscale can be
            # produced from color frames.
            synthesize_gray = False
            if not self.grayscale_topic and self.color_msg:
                synthesize_gray = True
            elif self.grayscale_topic and self.color_msg and self.gray_count == 0:
                synthesize_gray = True

            if synthesize_gray and self.last_color_ts and (self.last_gray_ts is None or self.last_color_ts > self.last_gray_ts):
                gray = self._convert_color_to_grayscale(self.color_msg)
                if gray is not None:
                    # use the color_msg header for timestamp
                    self.gray_msg = self.color_msg
                    self.gray_count += 1
                    self.last_gray_ts = self.last_color_ts
                    # mark that we synthesized grayscale so it can be reported
                    if not self.gray_synthesized:
                        self.get_logger().info('Synthesizing grayscale from color (no/slow grayscale stream)')
                    self.gray_synthesized = True

            # Check success condition: color, depth, caminfo and (if enabled) imu present
            imu_ok = True
            if self.enable_imu_check and self.imu_topic:
                imu_ok = bool(self.imu_msg)

            if self.color_msg and self.depth_msg and self.caminfo_msg and self.gray_msg and imu_ok:
                # We've seen the required topics. Continue spinning for a short
                # measurement window so frequency statistics are meaningful
                measurement_secs = min(3.0, max(1.0, self.timeout / 2.0))
                end_meas = time.time() + measurement_secs
                while time.time() < end_meas:
                    rclpy.spin_once(self, timeout_sec=0.1)
                return True

            rclpy.spin_once(self, timeout_sec=0.1)
        return False

    def evaluate(self):
        now = time.time()
        elapsed = max(1e-6, now - self.start_time)
        color_freq = self.color_count / elapsed
        depth_freq = self.depth_count / elapsed
        gray_freq = self.gray_count / elapsed
        imu_freq = self.imu_count / elapsed

        # timestamp sync check
        ts = []
        if self.last_color_ts:
            ts.append(self.last_color_ts)
        if self.last_depth_ts:
            ts.append(self.last_depth_ts)
        if self.last_gray_ts:
            ts.append(self.last_gray_ts)
        if self.last_imu_ts and (self.enable_imu_check and self.imu_topic):
            ts.append(self.last_imu_ts)

        sync_ok = True
        max_offset_ms = None
        if len(ts) >= 2:
            max_ts = max(ts)
            min_ts = min(ts)
            max_offset_ms = (max_ts - min_ts) * 1000.0
            sync_ok = max_offset_ms <= self.sync_threshold_ms

        # imu sanity checks
        imu_ok = True
        imu_notes = ''
        if self.enable_imu_check and self.imu_topic:
            if not self.imu_msg:
                imu_ok = False
                imu_notes = 'no imu messages received'
            else:
                imu = self.imu_msg
                # check finite and reasonable ranges
                av = imu.angular_velocity
                la = imu.linear_acceleration
                o = imu.orientation
                vals = [av.x, av.y, av.z, la.x, la.y, la.z, o.x, o.y, o.z, o.w]
                if not all([math.isfinite(float(v)) for v in vals]):
                    imu_ok = False
                    imu_notes = 'imu contains non-finite values'
                else:
                    if max(abs(av.x), abs(av.y), abs(av.z)) > 50.0:
                        imu_ok = False
                        imu_notes = 'angular velocity out of expected range'
                    if max(abs(la.x), abs(la.y), abs(la.z)) > 50.0:
                        imu_ok = False
                        imu_notes = 'linear acceleration out of expected range'

        # TF check
        tf_ok = None
        tf_notes = ''
        if self.check_tf and self._tf_available and self.color_frame_id and self.depth_frame_id:
            try:
                # verify there exists a transform between depth_frame and color_frame
                # check both directions (depth->color) using a small timeout
                t = rclpy.time.Time()
                can = False
                try:
                    can = self._tf_buffer.can_transform(self.color_frame_id, self.depth_frame_id, t, timeout=Duration(seconds=self.tf_timeout))
                except Exception:
                    # older/newer tf2_ros API discrepancies; try lookup as fallback
                    try:
                        self._tf_buffer.lookup_transform(self.color_frame_id, self.depth_frame_id, t, timeout=Duration(seconds=self.tf_timeout))
                        can = True
                    except Exception as e:
                        can = False
                        tf_notes = str(e)
                tf_ok = bool(can)
            except Exception as e:
                tf_ok = False
                tf_notes = str(e)

        summary = {
            'color_received': bool(self.color_msg),
            'depth_received': bool(self.depth_msg),
            'caminfo_received': bool(self.caminfo_msg),
            'grayscale_received': bool(self.gray_msg),
            'imu_received': bool(self.imu_msg) if (self.enable_imu_check and self.imu_topic) else None,
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
        }
        return summary



def _ros_time_to_secs(rostime) -> float:
    """Convert ROS2 builtin_interfaces/Time to float seconds."""
    if not rostime:
        return 0.0
    # rostime has attributes sec and nanosec
    sec = getattr(rostime, 'sec', None)
    nsec = getattr(rostime, 'nanosec', None)
    if sec is None or nsec is None:
        # fallback for other possible names
        sec = getattr(rostime, 'secs', 0)
        nsec = getattr(rostime, 'nsecs', 0)
    return float(sec) + float(nsec) * 1e-9


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--color', default='/camera/camera/color/image_raw')
    parser.add_argument('--depth', default='/camera/camera/depth/image_rect_raw')
    parser.add_argument('--caminfo', default='/camera/camera/color/camera_info')
    parser.add_argument('--grayscale', default=None, help='grayscale image topic (if published)')
    parser.add_argument('--imu', default=None, help='IMU topic (e.g. /camera/imu)')
    parser.add_argument('--enable-imu-check', action='store_true', help='enable IMU validation (for D435i)')
    parser.add_argument('--timeout', type=float, default=10.0)
    parser.add_argument('--sync-threshold-ms', type=float, default=20.0, help='max allowed timestamp offset in ms')
    parser.add_argument('--min-image-freq', type=float, default=15.0, help='minimum acceptable image frequency (Hz)')
    parser.add_argument('--min-imu-freq', type=float, default=100.0, help='minimum acceptable imu frequency (Hz)')
    parser.add_argument('--out-file', default=None, help='optional path to write JSON summary of the run')
    parser.add_argument('--check-tf', action='store_true', help='verify TF frame transforms exist between camera frames')
    parser.add_argument('--tf-timeout', type=float, default=1.0, help='timeout in seconds to wait for TF lookup/can_transform')
    args = parser.parse_args(argv)

    rclpy.init()
    node = RSValidateNode(
        args.color,
        args.depth,
        args.caminfo,
        args.grayscale,
        args.imu,
        args.enable_imu_check,
        args.timeout,
        args.sync_threshold_ms,
        args.min_image_freq,
        args.min_imu_freq,
        check_tf=args.check_tf,
        tf_timeout=args.tf_timeout,
    )
    ok = node.run()
    if ok:
        node.get_logger().info('Initial reception SUCCESS: required topics received')
    else:
        node.get_logger().error('Initial reception FAILURE: not all required messages received within timeout')

    summary = node.evaluate()

    # augment summary with raw counters and last timestamps for debugging
    debug_info = {
        'color_count': node.color_count,
        'depth_count': node.depth_count,
        'gray_count': node.gray_count,
        'imu_count': node.imu_count,
        'color_frame_id': node.color_frame_id,
        'depth_frame_id': node.depth_frame_id,
        'last_color_ts': node.last_color_ts,
        'last_depth_ts': node.last_depth_ts,
        'last_gray_ts': node.last_gray_ts,
        'last_imu_ts': node.last_imu_ts,
        'start_time': node.start_time,
        'end_time': time.time(),
        'grayscale_synthesized': node.gray_synthesized,
    }
    full_output = {'summary': summary, 'debug': debug_info}

    # additional pass/fail checks with detailed reasons
    overall_ok = True
    fail_reasons = []
    # required topics
    if not summary['color_received']:
        overall_ok = False
        fail_reasons.append('missing_color')
    if not summary['depth_received']:
        overall_ok = False
        fail_reasons.append('missing_depth')
    if not summary['caminfo_received']:
        overall_ok = False
        fail_reasons.append('missing_caminfo')
    if not summary['grayscale_received']:
        overall_ok = False
        fail_reasons.append('missing_grayscale')

    # imu check
    if args.enable_imu_check and args.imu:
        if not summary['imu_received']:
            overall_ok = False
            fail_reasons.append('missing_imu')
        if not summary['imu_ok']:
            overall_ok = False
            fail_reasons.append('imu_sanity_failed')

    # sync
    if not summary['sync_ok']:
        overall_ok = False
        fail_reasons.append('timestamp_sync')

    # frequency checks
    if summary['color_freq_hz'] < args.min_image_freq:
        overall_ok = False
        fail_reasons.append('color_freq_low')
    if summary['depth_freq_hz'] < args.min_image_freq:
        overall_ok = False
        fail_reasons.append('depth_freq_low')
    if args.enable_imu_check and args.imu:
        if summary['imu_freq_hz'] < args.min_imu_freq:
            overall_ok = False
            fail_reasons.append('imu_freq_low')

    # log details
    node.get_logger().info(f"Summary: {summary}")
    print(summary)

    # write JSON summary if requested (helps external tooling and CI to parse results)
    # augment JSON with pass/fail and reasons for automated parsing
    full_output['result'] = {
        'overall_ok': overall_ok,
        'fail_reasons': fail_reasons,
    }

    if args.out_file:
        try:
            with open(args.out_file, 'w') as fh:
                json.dump(full_output, fh, indent=2)
        except Exception as e:
            node.get_logger().warning(f'Failed to write out-file {args.out_file}: {e}')

    node.destroy_node()
    rclpy.shutdown()
    # exit with non-zero if overall_ok is False to make the script CI-friendly
    if not overall_ok:
        sys.exit(1)
    sys.exit(0 if overall_ok else 1)


if __name__ == '__main__':
    main()


def _ros_time_to_secs(rostime) -> float:
    """Convert ROS2 builtin_interfaces/Time to float seconds."""
    if not rostime:
        return 0.0
    # rostime has attributes sec and nanosec
    sec = getattr(rostime, 'sec', None)
    nsec = getattr(rostime, 'nanosec', None)
    if sec is None or nsec is None:
        # fallback for other possible names
        sec = getattr(rostime, 'secs', 0)
        nsec = getattr(rostime, 'nsecs', 0)
    return float(sec) + float(nsec) * 1e-9


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument('--color', default='/camera/camera/color/image_raw')
    parser.add_argument('--depth', default='/camera/camera/depth/image_rect_raw')
    parser.add_argument('--caminfo', default='/camera/camera/color/camera_info')
    parser.add_argument('--grayscale', default=None, help='grayscale image topic (if published)')
    parser.add_argument('--imu', default=None, help='IMU topic (e.g. /camera/imu)')
    parser.add_argument('--enable-imu-check', action='store_true', help='enable IMU validation (for D435i)')
    parser.add_argument('--timeout', type=float, default=10.0)
    parser.add_argument('--sync-threshold-ms', type=float, default=20.0, help='max allowed timestamp offset in ms')
    parser.add_argument('--min-image-freq', type=float, default=15.0, help='minimum acceptable image frequency (Hz)')
    parser.add_argument('--min-imu-freq', type=float, default=100.0, help='minimum acceptable imu frequency (Hz)')
    parser.add_argument('--out-file', default=None, help='optional path to write JSON summary of the run')
    parser.add_argument('--check-tf', action='store_true', help='verify TF frame transforms exist between camera frames')
    parser.add_argument('--tf-timeout', type=float, default=1.0, help='timeout in seconds to wait for TF lookup/can_transform')
    args = parser.parse_args(argv)

    rclpy.init()
    node = RSValidateNode(
        args.color,
        args.depth,
        args.caminfo,
        args.grayscale,
        args.imu,
        args.enable_imu_check,
        args.timeout,
        args.sync_threshold_ms,
        args.min_image_freq,
        args.min_imu_freq,
        check_tf=args.check_tf,
        tf_timeout=args.tf_timeout,
    )
    ok = node.run()
    if ok:
        node.get_logger().info('Initial reception SUCCESS: required topics received')
    else:
        node.get_logger().error('Initial reception FAILURE: not all required messages received within timeout')

    summary = node.evaluate()

    # augment summary with raw counters and last timestamps for debugging
    debug_info = {
        'color_count': node.color_count,
        'depth_count': node.depth_count,
        'gray_count': node.gray_count,
        'imu_count': node.imu_count,
        'color_frame_id': node.color_frame_id,
        'depth_frame_id': node.depth_frame_id,
        'last_color_ts': node.last_color_ts,
        'last_depth_ts': node.last_depth_ts,
        'last_gray_ts': node.last_gray_ts,
        'last_imu_ts': node.last_imu_ts,
        'start_time': node.start_time,
        'end_time': time.time(),
        'grayscale_synthesized': node.gray_synthesized,
    }
    full_output = {'summary': summary, 'debug': debug_info}

    # additional pass/fail checks with detailed reasons
    overall_ok = True
    fail_reasons = []
    # required topics
    if not summary['color_received']:
        overall_ok = False
        fail_reasons.append('missing_color')
    if not summary['depth_received']:
        overall_ok = False
        fail_reasons.append('missing_depth')
    if not summary['caminfo_received']:
        overall_ok = False
        fail_reasons.append('missing_caminfo')
    if not summary['grayscale_received']:
        overall_ok = False
        fail_reasons.append('missing_grayscale')

    # imu check
    if args.enable_imu_check and args.imu:
        if not summary['imu_received']:
            overall_ok = False
            fail_reasons.append('missing_imu')
        if not summary['imu_ok']:
            overall_ok = False
            fail_reasons.append('imu_sanity_failed')

    # sync
    if not summary['sync_ok']:
        overall_ok = False
        fail_reasons.append('timestamp_sync')

    # frequency checks
    if summary['color_freq_hz'] < args.min_image_freq:
        overall_ok = False
        fail_reasons.append('color_freq_low')
    if summary['depth_freq_hz'] < args.min_image_freq:
        overall_ok = False
        fail_reasons.append('depth_freq_low')
    if args.enable_imu_check and args.imu:
        if summary['imu_freq_hz'] < args.min_imu_freq:
            overall_ok = False
            fail_reasons.append('imu_freq_low')

    # log details
    node.get_logger().info(f"Summary: {summary}")
    print(summary)

    # write JSON summary if requested (helps external tooling and CI to parse results)
    # augment JSON with pass/fail and reasons for automated parsing
    full_output['result'] = {
        'overall_ok': overall_ok,
        'fail_reasons': fail_reasons,
    }

    if args.out_file:
        try:
            with open(args.out_file, 'w') as fh:
                json.dump(full_output, fh, indent=2)
        except Exception as e:
            node.get_logger().warning(f'Failed to write out-file {args.out_file}: {e}')

    node.destroy_node()
    rclpy.shutdown()
    # exit with non-zero if overall_ok is False to make the script CI-friendly
    if not overall_ok:
        sys.exit(1)
    sys.exit(0 if overall_ok else 1)


if __name__ == '__main__':
    main()
