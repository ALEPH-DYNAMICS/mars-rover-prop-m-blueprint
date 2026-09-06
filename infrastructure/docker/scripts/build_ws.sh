#!/bin/bash
set -eo pipefail

echo "[docker] Building ROS workspace..."

cd /workspaces/mars-rover-prop-m-blueprint/ros_ws
source /opt/ros/humble/setup.bash

# rosdep may already be initialized in image; safe to run
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi
rosdep update
rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install

echo "[docker] Build complete."
