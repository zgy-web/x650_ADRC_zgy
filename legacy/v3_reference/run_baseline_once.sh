#!/usr/bin/env bash
set -o pipefail

cd "$(dirname "$0")"
source ./cloud_env.sh

# ROS setup scripts are not nounset-safe.
# Enable nounset only after the environment is loaded.
set -u

SIM_PID=""

cleanup() {
    RC=$?

    if [[ -n "${SIM_PID}" ]] &&
       kill -0 "$SIM_PID" 2>/dev/null; then

        echo
        echo "============================================================"
        echo " Stopping owned PX4/Gazebo session normally"
        echo "============================================================"

        kill -INT "$SIM_PID" 2>/dev/null || true
        wait "$SIM_PID" 2>/dev/null || true
    fi

    echo
    echo "============================================================"
    echo " Finalizing PX4 ULog"
    echo "============================================================"

    /usr/bin/python3 ./finalize_pending_ulog.py || true

    exit "$RC"
}

trap cleanup EXIT INT TERM

if [[ -f pending_ulog_finalize.json ]]; then
    echo "[FAIL] Previous baseline still has pending ULog."
    echo "Resolve it before starting another run."
    exit 1
fi

echo "============================================================"
echo " UNDERFOLD BASELINE - ONE RUN"
echo "============================================================"
echo "1. start fresh PX4/Gazebo"
echo "2. readiness check"
echo "3. stationary baseline"
echo "4. land + auto-disarm"
echo "5. stop simulator"
echo "6. finalize ULog"
echo
echo "No ground_stow."
echo "No arm motion."
echo "============================================================"

SUPERVISOR_LOG="/root/autodl-tmp/desktop_logs/underfold-v3-supervisor-$(date +%Y%m%d_%H%M%S).log"

mkdir -p /root/autodl-tmp/desktop_logs

/usr/bin/python3 ./auto_sim.py \
    > >(tee "$SUPERVISOR_LOG") \
    2>&1 &

SIM_PID=$!

sleep 2

if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "[FAIL] simulator supervisor exited during startup"
    wait "$SIM_PID" || true
    exit 1
fi

echo
echo "============================================================"
echo " READINESS CHECK"
echo "============================================================"

/root/autodl-tmp/cloud-arm-venv/bin/python3 \
    ./check_underfold.py

echo
echo "============================================================"
echo " BASELINE FLIGHT"
echo "============================================================"

/usr/bin/python3 ./baseline_runner.py

echo
echo "============================================================"
echo " BASELINE FLIGHT COMPLETE"
echo " Simulator will now be closed and ULog finalized."
echo "============================================================"
