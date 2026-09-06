#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

# Source workspace if present (runtime may contain a baked install; dev mounts may build it)
if [ -f "/workspaces/mars-rover-prop-m-blueprint/ros_ws/install/setup.bash" ]; then
  source /workspaces/mars-rover-prop-m-blueprint/ros_ws/install/setup.bash
fi

exec "$@"
