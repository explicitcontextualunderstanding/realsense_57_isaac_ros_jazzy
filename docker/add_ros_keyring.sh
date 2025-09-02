#!/usr/bin/env bash
set -euo pipefail

# add_ros_keyring.sh
# Idempotent helper to install the ROS apt keyring and add a signed-by apt source
# Usage (defaults are suitable for the project):
#   add_ros_keyring.sh [--key-url URL] [--distro jazzy] [--repo-url http://packages.ros.org/ros2/ubuntu]

KEY_URL_DEFAULT="https://raw.githubusercontent.com/ros/rosdistro/master/ros.key"
DISTRO_DEFAULT="jazzy"
REPO_URL_DEFAULT="http://packages.ros.org/ros2/ubuntu"

KEY_URL="${KEY_URL_DEFAULT}"
DISTRO="${DISTRO_DEFAULT}"
REPO_URL="${REPO_URL_DEFAULT}"

while [ "$#" -gt 0 ]; do
  case "$1" in
    --key-url) KEY_URL="$2"; shift 2 ;;
    --distro) DISTRO="$2"; shift 2 ;;
    --repo-url) REPO_URL="$2"; shift 2 ;;
    -h|--help) echo "Usage: $0 [--key-url URL] [--distro jazzy] [--repo-url URL]"; exit 0 ;;
    *) echo "Unknown arg: $1" >&2; exit 2 ;;
  esac
done

KEYRING_DIR="/usr/share/keyrings"
KEYRING_PATH="$KEYRING_DIR/ros2-archive-keyring.gpg"
SOURCES_LIST_DIR="/etc/apt/sources.list.d"
SOURCES_LIST_PATH="$SOURCES_LIST_DIR/ros2.list"

echo "[add_ros_keyring] distro=${DISTRO} repo=${REPO_URL} key_url=${KEY_URL}"

# Ensure required tools exist (installing gnupg if missing is typically done in the Dockerfile stage)
if ! command -v gpg >/dev/null 2>&1; then
  echo "gpg not found; attempting to install gnupg" >&2
  if command -v apt-get >/dev/null 2>&1; then
    apt-get update && apt-get install -y --no-install-recommends gnupg dirmngr curl ca-certificates || true
  else
    echo "apt-get not available; please install gpg and curl before running this script" >&2
    exit 1
  fi
fi

# Create keyring dir
mkdir -p "$KEYRING_DIR"

# Idempotent keyring install: if an existing keyring is present, skip download unless it differs.
if [ -f "$KEYRING_PATH" ]; then
  echo "Keyring already exists at $KEYRING_PATH — leaving in place (idempotent)"
else
  echo "Downloading ROS key and writing keyring to $KEYRING_PATH"
  # Download and dearmor into the keyring path
  curl -fsSL "$KEY_URL" | gpg --dearmor -o "$KEYRING_PATH"
  chmod 644 "$KEYRING_PATH" || true
fi

# Write apt sources list entry (signed-by uses our keyring)
ARCH="$(dpkg --print-architecture 2>/dev/null || echo amd64)"
SOURCES_LINE="deb [arch=${ARCH} signed-by=${KEYRING_PATH}] ${REPO_URL} ${DISTRO} main"

mkdir -p "$SOURCES_LIST_DIR"

if [ -f "$SOURCES_LIST_PATH" ]; then
  if grep -Fq "$SOURCES_LINE" "$SOURCES_LIST_PATH"; then
    echo "Sources list $SOURCES_LIST_PATH already contains the correct entry — idempotent"
  else
    echo "Updating $SOURCES_LIST_PATH with ROS repository entry"
    printf "%s\n" "$SOURCES_LINE" > "$SOURCES_LIST_PATH"
    chmod 644 "$SOURCES_LIST_PATH" || true
  fi
else
  echo "Creating $SOURCES_LIST_PATH"
  printf "%s\n" "$SOURCES_LINE" > "$SOURCES_LIST_PATH"
  chmod 644 "$SOURCES_LIST_PATH" || true
fi

echo "ROS apt keyring and source configured (list: $SOURCES_LIST_PATH, keyring: $KEYRING_PATH)"
echo "Run 'apt-get update' next to refresh package lists."

exit 0
