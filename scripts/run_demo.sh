#!/usr/bin/env bash
set -eo pipefail

# Source ROS & this workspace safely even if nounset is enabled in the shell
set +u
source /opt/ros/humble/setup.bash
source install/setup.bash
set -u

# Launch the demo
ros2 launch jackal_bringup jackal_sim.launch.py
