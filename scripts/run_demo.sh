#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
source install/setup.bash
# TODO: replace with your real package + launch
ros2 launch your_package your_launch_file.launch.py
