#!/usr/bin/env bash
set -euo pipefail

fail=0
ok() { printf '[PASS] %s\n' "$*"; }
bad() { printf '[FAIL] %s\n' "$*"; fail=1; }

if [[ -r /etc/os-release ]]; then
  . /etc/os-release
  [[ "${VERSION_ID:-}" == "22.04" ]] && ok "Ubuntu 22.04" || bad "Expected Ubuntu 22.04; got ${PRETTY_NAME:-unknown}"
else
  bad "/etc/os-release missing"
fi

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

for cmd in git cmake colcon python3 gz ros2 MicroXRCEAgent; do
  command -v "$cmd" >/dev/null 2>&1 && ok "$cmd: $(command -v "$cmd")" || bad "$cmd not found"
done

if command -v python3 >/dev/null 2>&1; then
  py="$(python3 -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
  [[ "$py" == "3.10" ]] && ok "Python $py" || bad "Expected Python 3.10; got $py"
fi

if command -v gz >/dev/null 2>&1; then
  gzv="$(gz sim --versions 2>/dev/null | head -n1 || true)"
  [[ "$gzv" == 8.* ]] && ok "Gazebo Sim $gzv" || bad "Expected Gazebo Sim 8; got $gzv"
fi

[[ -f /opt/ros/humble/setup.bash ]] && ok "ROS 2 Humble setup" || bad "/opt/ros/humble/setup.bash missing"

if command -v ros2 >/dev/null 2>&1; then
  ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1 && ok "ros_gz_bridge" || bad "ros_gz_bridge package missing"
fi

if command -v nvidia-smi >/dev/null 2>&1; then
  ok "NVIDIA GPU: $(nvidia-smi --query-gpu=name,driver_version --format=csv,noheader | head -n1)"
else
  printf '[WARN] nvidia-smi not found; headless/direct CPU rendering may still be possible\n'
fi

exit "$fail"
