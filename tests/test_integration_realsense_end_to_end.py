import os
import subprocess
import json
import glob
import pytest

# This integration test is intentionally disabled by default because it requires
# hardware access and docker privileges. Enable by setting RUN_INTEGRATION=1.
pytestmark = pytest.mark.skipif(os.environ.get('RUN_INTEGRATION') != '1', reason='Integration tests disabled; set RUN_INTEGRATION=1 to enable')


def test_end_to_end_runs_validator_and_passes():
    # Run the automation script which launches the container, starts the ROS node,
    # waits for topics, and writes a validator JSON into realsense_test_outputs/.
    proc = subprocess.run(["./scripts/run_automated_realsense_test.sh", "--timeout", "20"], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    # We allow the script to return non-zero if it failed; we'll inspect artifacts.
    # Find the most recent validator JSON
    files = sorted(glob.glob('realsense_test_outputs/validate_realsense_plus_*.json'))
    assert files, 'No validator JSON produced by automation script; check script output and logs.\n' + proc.stdout
    latest = files[-1]
    with open(latest, 'r') as fh:
        data = json.load(fh)
    assert data.get('result', {}).get('overall_ok', False) is True, f"Validator reported failure: {data}\nScript output:\n{proc.stdout}"
