#!/usr/bin/env bash
# exec_with_ros.sh - convenience wrapper to run `docker exec` with the image's
# entrypoint sourced so `ros2` and workspace Python packages (e.g. tf2_ros)
# are available inside the exec'd shell.
#
# Usage:
#   ./scripts/exec_with_ros.sh [container]             # open interactive shell with ROS sourced
#   ./scripts/exec_with_ros.sh [container] -- <command>  # run a single command with ROS sourced
#
set -euo pipefail

CONTAINER=${1:-realsense_debug}

if [ "$#" -eq 0 ]; then
  # default: open interactive shell in the default container
  docker exec -it "${CONTAINER}" bash -lc "source /usr/local/bin/realsense_entrypoint.sh && exec bash"
  exit 0
fi

if [ "$#" -eq 1 ]; then
  # one argument supplied -> container name, open interactive shell
  CONTAINER="$1"
  docker exec -it "${CONTAINER}" bash -lc "source /usr/local/bin/realsense_entrypoint.sh && exec bash"
  exit 0
fi

# If we reach here there are >=2 args: first is container, remaining are the command
CONTAINER="$1"
shift

# If the user passed a literal --, remove it for clarity
if [ "${1:-}" = "--" ]; then
  shift
fi

CMD=("${@}")
echo "Running in container '${CONTAINER}': ${CMD[*]}"
# Forward the remaining args to a remote bash which will execute them as "$@".
# This preserves user quoting (we use the bash -lc '... && "$@"' -- arg1 arg2 ... form).
docker exec -it "${CONTAINER}" bash -lc 'source /usr/local/bin/realsense_entrypoint.sh >/dev/null 2>&1 && "$@"' -- "${CMD[@]}"
