#!/usr/bin/env python3
"""
Backward-compatibility wrapper for validate_realsense_ros.

Delegates to the canonical implementation in validators/validate_realsense_ros.py
so existing callers importing from scripts/ continue to work during migration.
"""

from validators.validate_realsense_ros import (
    RSValidateNode,            # re-exported for importers/tests
    _ros_time_to_secs,         # re-exported for importers/tests
    main as _main,
)

if __name__ == '__main__':
    _main()

