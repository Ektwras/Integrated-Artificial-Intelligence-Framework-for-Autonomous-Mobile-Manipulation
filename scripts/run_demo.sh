#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source install/setup.bash
# TODO: replace with your real package + launch
ros2 launch jackal_bringup jackal_sim.launch.py
