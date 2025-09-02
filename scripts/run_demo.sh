#!/usr/bin/env bash
set -eo pipefail

# Resolve workspace root from this script location
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
WS_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Use an isolated HOME inside the repo so any "~" writes stay local
TMP_HOME="$WS_ROOT/.demo_home"
mkdir -p "$TMP_HOME"
# Keep ROS caches/logs local too
export ROS_HOME="$TMP_HOME/.ros"

# If generators hard-code ~/ros2_clearpath_ws, redirect that path back into this repo
mkdir -p "$TMP_HOME"
rm -f "$TMP_HOME/ros2_clearpath_ws"
ln -s "$WS_ROOT" "$TMP_HOME/ros2_clearpath_ws"

# Preserve GUI if available; else force offscreen so Qt doesn't crash
EXTRA_ENV=()
if [ -n "${DISPLAY:-}" ]; then
  EXTRA_ENV+=(DISPLAY="$DISPLAY")
else
  EXTRA_ENV+=(QT_QPA_PLATFORM=offscreen)
fi
# Some Qt/GL setups expect XDG_RUNTIME_DIR
EXTRA_ENV+=(XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/tmp}")

env -i HOME="$TMP_HOME" USER="$USER" TERM="${TERM:-xterm-256color}" PATH="/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin" "${EXTRA_ENV[@]}" \
bash --noprofile --norc -lc "
  set +u
  source /opt/ros/humble/setup.bash
  source \"$WS_ROOT/install/setup.bash\"
  set -u
  echo \"jackal_bringup prefix: \$(ros2 pkg prefix jackal_bringup)\"
  ros2 launch jackal_bringup jackal_sim.launch.py
"
