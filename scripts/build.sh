#!/usr/bin/env bash
set -euo pipefail
source /opt/ros/humble/setup.bash
./scripts/install_rosdeps.sh
if [ -f requirements.txt ]; then
  python3 -m pip install --user -r requirements.txt
fi
colcon build --symlink-install
echo "Build complete. To use, run: source install/setup.bash"
