#!/usr/bin/env bash
set -e

export AMSIM_WORKSPACE="${AMSIM_WORKSPACE:-/opt/amsim}"
export AMSIM_PX4_DIR="${AMSIM_PX4_DIR:-$AMSIM_WORKSPACE/PX4-Autopilot}"
export AMSIM_ROS_WS="${AMSIM_ROS_WS:-$AMSIM_WORKSPACE/ros2_px4_ws}"
export AMSIM_VENV="${AMSIM_VENV:-$AMSIM_WORKSPACE/amsim-venv}"
export AMSIM_RESULTS_DIR="${AMSIM_RESULTS_DIR:-/opt/amsim/project/runs}"
export GZ_PARTITION="${GZ_PARTITION:-amsim_default}"

exec "$@"
