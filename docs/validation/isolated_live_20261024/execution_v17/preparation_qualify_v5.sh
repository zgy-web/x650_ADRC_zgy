#!/bin/bash
source /opt/ros/humble/setup.bash
source /root/autodl-tmp/ros2_px4_ws/install/setup.bash
source /root/autodl-tmp/flyarm_ros2_live_v1_20261023/install/setup.bash
cd /root/autodl-tmp/safe-aerial-manipulator-testbed
export PYTHONPATH=/usr/lib/python3/dist-packages:/root/autodl-tmp/amsim-venv/lib/python3.10/site-packages:$PYTHONPATH
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1
set -e
taskset -c 87,89,91,97 /usr/bin/python3 scripts/replay_x650_native_numerics.py --failure-evidence artifacts/ts20_recording_chain_20261023/quiet_failure_v1/failure_evidence.json --failure-sha256 392889481ace0d41616187c81b2b72e3f221a463f2e80eda2c0b357610ba4f43 --native-library /root/autodl-tmp/x650_native_optimization_build_v3/libx650_optimization.so --native-sha256 b9cbdf78c76e92f973800e07c6219611759fcb4471a1a39192c5cbf30f5e2e48 --output artifacts/isolated_live_20261024/preparation_replay_v5
recordsha=$(/usr/bin/sha256sum artifacts/isolated_live_20261024/preparation_replay_v5/replay_report.json | cut -d " " -f1)
taskset -c 87,89,91,97 /usr/bin/python3 scripts/qualify_x650_native_backend.py --previous-proof artifacts/isolated_live_20261023/native_fix_software_v1/software_acceptance.json --previous-sha256 61bd5f9fc872cc7da93dfe73e850ec1d8a435d8e42ac884f25c6b1f812268265 --recorded-report artifacts/isolated_live_20261024/preparation_replay_v5/replay_report.json --recorded-sha256 "$recordsha" --native-library /root/autodl-tmp/x650_native_optimization_build_v3/libx650_optimization.so --native-sha256 b9cbdf78c76e92f973800e07c6219611759fcb4471a1a39192c5cbf30f5e2e48 --output artifacts/isolated_live_20261024/preparation_software_v5

