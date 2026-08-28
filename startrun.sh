#!/usr/bin/env bash

set -e

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd -- "${SCRIPT_DIR}/.." && pwd)"

# Load this workspace when the caller has not sourced ROS 2 manually.
if [[ -f "${WORKSPACE_DIR}/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "${WORKSPACE_DIR}/install/setup.bash"
fi

if ! command -v ros2 >/dev/null 2>&1; then
    echo "Error: ros2 was not found. Build/source the ROS 2 workspace first." >&2
    exit 1
fi

echo "Waiting for the real-car RRT node, then enabling vehicle motion..."
exec ros2 topic pub --once --wait-matching-subscriptions 1 \
    /motion_planning/control \
    std_msgs/msg/String \
    "{data: start}"
