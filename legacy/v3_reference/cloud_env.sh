#!/usr/bin/env bash
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin
unset PYTHONHOME PYTHONPATH VIRTUAL_ENV
source /opt/ros/humble/setup.bash
source /root/autodl-tmp/ros2_px4_ws/install/setup.bash
export PX4_DIR=/root/autodl-tmp/PX4-Autopilot
export RESULTS_DIR=/root/autodl-tmp/px4_baseline_bags
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export HEADLESS=1 PX4_SIM_SPEED_FACTOR=1
unset PX4_GZ_STANDALONE PX4_GZ_MODEL_NAME PX4_IMU_BATCH_TRIAL
export GZ_PARTITION=cloud_aerial_testbed
