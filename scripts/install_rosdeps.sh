#!/usr/bin/env bash
set -euo pipefail

if ! command -v rosdep >/dev/null; then
  sudo apt update
  sudo apt install -y python3-rosdep
fi

# Initialize once; ignore if already done
sudo rosdep init 2>/dev/null || true

# rosdep sometimes complains about a local debian.yaml path; don't fail the build for that
rosdep update || true

# Install dependencies for all packages in src
rosdep install --from-paths src --rosdistro humble --ignore-src -r -y || true
