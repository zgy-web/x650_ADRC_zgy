#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/autodl-tmp/ros2_px4_ws/install/setup.bash
source /root/autodl-tmp/flyarm_ros2_live_v1_20261023/install/setup.bash
cd /root/autodl-tmp/safe-aerial-manipulator-testbed
export PYTHONPATH=/usr/lib/python3/dist-packages:/root/autodl-tmp/amsim-venv/lib/python3.10/site-packages:$PYTHONPATH
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
set -e
/usr/bin/python3 simulations/x650_isolated_live_v1/launch.py --task /root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/isolated_live_20261024/quiet_task_cpp_v10.json --dry-run --transport-proof /root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/isolated_live_20261024/transport_v11/report.json
