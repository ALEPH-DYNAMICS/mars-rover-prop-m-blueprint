#!/bin/bash
set -eo pipefail
source /opt/ros/humble/setup.bash
cd /workspaces/mars-rover-prop-m-blueprint
source ros_ws/install/setup.bash
export GAZEBO_MODEL_PATH="$PWD/ros_ws/src/rover_sim_gazebo/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_PLUGIN_PATH="$PWD/ros_ws/install/rover_sim_gazebo/lib:${GAZEBO_PLUGIN_PATH:-}"
python3 scripts/sim_smoke.py --out analysis/sim-smoke.json
