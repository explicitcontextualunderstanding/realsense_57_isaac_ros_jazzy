# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

#!/usr/bin/env python3
"""
rs2_test.py

Lightweight Intel RealSense smoke test for use inside the container image.
This script supports two modes:
  - sdk : use the pyrealsense2 SDK to enumerate devices and capture a few frames
  - ros : run the existing `scripts/validate_realsense_ros.py` ROS validator
  - both: run sdk then ros

It returns exit code 0 on success (all requested checks passed) and 1 on failure.

Examples:
  python3 scripts/rs2_test.py --mode sdk --frames 3
  python3 scripts/rs2_test.py --mode ros --timeout 20
  python3 scripts/rs2_test.py --mode both --frames 5 --timeout 15
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time


def sdk_smoke_test(frames: int = 3, timeout_sec: float = 5.0) -> bool:
	"""Run a minimal pyrealsense2 smoke test: enumerate devices and capture N frames.

	Returns True on success, False on failure.
	"""
	try:
		import pyrealsense2 as rs
	except Exception as e:  # ImportError or runtime errors
		print(f"SDK test: failed to import pyrealsense2: {e}")
		return False

	try:
		ctx = rs.context()
		devices = ctx.devices if hasattr(ctx, 'devices') else ctx.query_devices()
		if len(devices) == 0:
			print("SDK test: no RealSense device detected")
			return False

		dev = devices[0]
		print(f"SDK test: Device found: {dev.get_info(rs.camera_info.name)} serial={dev.get_info(rs.camera_info.serial_number)} firmware={dev.get_info(rs.camera_info.firmware_version)}")

		pipeline = rs.pipeline()
		cfg = rs.config()
		# request common color+depth streams at modest resolution
		cfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
		try:
			cfg.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
		except Exception:
			# some cameras may not support RGB8 alias; ignore and continue
			pass

		profile = pipeline.start(cfg)
		ok = True
		for i in range(frames):
			try:
				frameset = pipeline.wait_for_frames(timeout_ms=int(timeout_sec * 1000))
			except Exception as e:
				print(f"SDK test: timeout/wait_for_frames failed at frame {i}: {e}")
				ok = False
				break

			depth = frameset.get_depth_frame()
			color = frameset.get_color_frame()
			depth_ok = depth is not None and depth.get_width() > 0 and depth.get_height() > 0
			color_ok = color is not None and color.get_width() > 0 and color.get_height() > 0
			print(f"SDK test: frame {i} depth_ok={depth_ok} color_ok={color_ok}")
			if not (depth_ok or color_ok):
				ok = False
				break

		try:
			pipeline.stop()
		except Exception:
			pass

		return bool(ok)
	except Exception as e:
		print(f"SDK test: unexpected error: {e}")
		return False


def ros_validator(timeout: float = 10.0, color: str = None, depth: str = None, caminfo: str = None) -> bool:
	"""Run the existing ROS validator script as a subprocess and return its success.

	The validator is `scripts/validate_realsense_ros.py` in this repository.
	"""
	cmd = [sys.executable, 'scripts/validate_realsense_ros.py', '--timeout', str(timeout)]
	if color:
		cmd += ['--color', color]
	if depth:
		cmd += ['--depth', depth]
	if caminfo:
		cmd += ['--caminfo', caminfo]

	try:
		print(f"ROS validator: running: {' '.join(cmd)}")
		proc = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, check=False)
		print(proc.stdout)
		return proc.returncode == 0
	except FileNotFoundError:
		print("ROS validator: script not found or python interpreter missing")
		return False
	except Exception as e:
		print(f"ROS validator: unexpected error: {e}")
		return False


def main(argv=None):
	parser = argparse.ArgumentParser()
	parser.add_argument('--mode', choices=['sdk', 'ros', 'both'], default='both')
	parser.add_argument('--frames', type=int, default=3, help='number of frames to capture in SDK test')
	parser.add_argument('--sdk-timeout', type=float, default=5.0, help='timeout (s) for waiting frames in SDK test')
	parser.add_argument('--timeout', type=float, default=10.0, help='timeout (s) for ROS validator')
	parser.add_argument('--color', default='/camera/camera/color/image_raw')
	parser.add_argument('--depth', default='/camera/camera/depth/image_rect_raw')
	parser.add_argument('--caminfo', default='/camera/camera/color/camera_info')
	args = parser.parse_args(argv)

	summary = {'sdk': None, 'ros': None}

	if args.mode in ('sdk', 'both'):
		print('Starting SDK smoke test...')
		summary['sdk'] = sdk_smoke_test(frames=args.frames, timeout_sec=args.sdk_timeout)
		print(f"SDK test result: {summary['sdk']}")

	if args.mode in ('ros', 'both'):
		print('Starting ROS validator...')
		summary['ros'] = ros_validator(timeout=args.timeout, color=args.color, depth=args.depth, caminfo=args.caminfo)
		print(f"ROS validator result: {summary['ros']}")

	print('\nTest summary:')
	print(json.dumps(summary, indent=2))

	ok = True
	if args.mode == 'sdk':
		ok = bool(summary['sdk'])
	elif args.mode == 'ros':
		ok = bool(summary['ros'])
	else:
		ok = bool(summary['sdk']) and bool(summary['ros'])

	sys.exit(0 if ok else 1)


if __name__ == '__main__':
	main()
