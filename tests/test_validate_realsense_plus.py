import hashlib
import numpy as np
import time
import os
import sys

# Ensure the repository 'scripts' package is importable during tests
ROOT = os.path.dirname(os.path.dirname(__file__))
sys.path.insert(0, ROOT)

# Provide lightweight fakes for ROS2 runtime and message packages so unit tests
# can import the validator logic without a full ROS 2 installation.
import types

# Fake rclpy with minimal API used by the node
fake_rclpy = types.ModuleType('rclpy')
class FakeLogger:
    def info(self, *a, **k): pass
    def error(self, *a, **k): pass
    def warning(self, *a, **k): pass

class FakeNodeBase:
    def __init__(self, name='fake'):
        self._logger = FakeLogger()
    def get_logger(self):
        return self._logger
    def create_subscription(self, *_a, **_k):
        return None
    def destroy_node(self):
        return None

fake_rclpy.node = types.SimpleNamespace(Node=FakeNodeBase)
def fake_spin_once(node, timeout_sec=0.0):
    return None
fake_rclpy.spin_once = fake_spin_once
fake_rclpy.shutdown = lambda : None
fake_rclpy.init = lambda : None

import sys as _sys
_sys.modules['rclpy'] = fake_rclpy
_sys.modules['rclpy.node'] = fake_rclpy.node

# Fake Duration used in validator
class FakeDuration:
    def __init__(self, seconds=0):
        self._s = seconds
fake_rclpy.duration = types.SimpleNamespace(Duration=FakeDuration)
_sys.modules['rclpy.duration'] = fake_rclpy.duration

# Provide fake sensor_msgs.msg types (Image, CameraInfo, Imu) used as annotations
fake_sensor_msgs = types.ModuleType('sensor_msgs')
fake_sensor_msgs.msg = types.SimpleNamespace(Image=object, CameraInfo=object, Imu=object)
_sys.modules['sensor_msgs'] = fake_sensor_msgs
_sys.modules['sensor_msgs.msg'] = fake_sensor_msgs.msg

from validators.validate_realsense_plus import RSValidateNode, _ros_time_to_secs


class DummyHeader:
    def __init__(self, sec, nsec, frame_id="camera"):
        self.stamp = type('t', (), {'sec': sec, 'nanosec': nsec})
        self.frame_id = frame_id


class DummyImage:
    def __init__(self, data: bytes, width=4, height=4, encoding='rgb8', sec=0, nsec=0, frame_id='camera'):
        self.data = data
        self.width = width
        self.height = height
        self.encoding = encoding
        self.header = DummyHeader(sec, nsec, frame_id)


class DummyCameraInfo:
    def __init__(self, k0=600.0, k4=600.0, sec=0, nsec=0, frame_id='camera'):
        self.k = [0.0]*9
        self.k[0] = k0
        self.k[4] = k4
        self.header = DummyHeader(sec, nsec, frame_id)


class DummyImu:
    def __init__(self, ax=0.0, ay=0.0, az=9.8, gx=0.0, gy=0.0, gz=0.0, sec=0, nsec=0, frame_id='imu'):
        class V:
            def __init__(self, x, y, z):
                self.x = x
                self.y = y
                self.z = z
        self.angular_velocity = V(gx, gy, gz)
        self.linear_acceleration = V(ax, ay, az)
        self.header = DummyHeader(sec, nsec, frame_id)


def test_frozen_frame_detector():
    # Create a node but don't init rclpy (we only use its logic)
    node = RSValidateNode('/c', '/d', '/ci', None, None, False, 1.0, 20.0, 15.0, 100.0)

    # create two identical images (frozen) and feed them repeatedly
    payload = bytes([10]*16)
    img1 = DummyImage(payload, sec=1, nsec=0)
    img2 = DummyImage(payload, sec=1, nsec=1000000)

    # feed the same payload 7 times to exceed frozen threshold (>5)
    for i in range(7):
        node.color_cb(img1)
    # After repeated identical data we expect a frozen_frame failure
    assert 'color_stream_is_frozen' in node.validity_failures


def test_timestamp_monotonicity():
    node = RSValidateNode('/c', '/d', '/ci', None, '/imu', True, 1.0, 20.0, 15.0, 100.0)

    # send increasing timestamps for color and depth
    node.color_cb(DummyImage(bytes([1]*16), sec=1, nsec=0))
    node.depth_cb(DummyImage(bytes([2]*16), sec=1, nsec=100000))

    # send an initial IMU timestamp (later)
    node.imu_cb(DummyImu(sec=2, nsec=0))
    # send a non-monotonic imu timestamp (earlier than previous)
    node.imu_cb(DummyImu(sec=1, nsec=500000))

    assert 'imu_timestamp_non_monotonic' in node.validity_failures
