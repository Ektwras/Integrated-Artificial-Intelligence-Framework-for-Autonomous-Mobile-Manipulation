#!/usr/bin/env bash
set -euo pipefail
if ! command -v rosdep >/dev/null; then
  sudo apt update
  sudo apt install -y python3-rosdep
fi
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --rosdistro humble --ignore-src -r -y
