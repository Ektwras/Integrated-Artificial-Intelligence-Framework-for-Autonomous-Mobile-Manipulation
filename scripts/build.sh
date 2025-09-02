#!/usr/bin/env bash
# Safer flags, but avoid nounset while sourcing ROS setup
set -eo pipefail

# Temporarily disable nounset (in case a shell has it set) for ROS setup
set +u
source /opt/ros/humble/setup.bash
set -u

./scripts/install_rosdeps.sh

# Optional Python deps
if [ -f requirements.txt ]; then
  python3 -m pip install --user -r requirements.txt
fi

colcon build --symlink-install
echo "Build complete. To use, run: source install/setup.bash"
