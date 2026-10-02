#!/usr/bin/env bash
set -o pipefail

cd "$(dirname "$0")"
source ./cloud_env.sh

# ROS setup isn't nounset-safe, so enable this afterwards.
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

        for _ in $(seq 1 40); do
            if ! kill -0 "$SIM_PID" 2>/dev/null; then
                break
            fi
            sleep 0.25
        done

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
    echo "[FAIL] Previous experiment still has pending ULog."
    exit 1
fi

echo "============================================================"
echo " UNDERFOLD MOTION V1 - ONE RUN"
echo "============================================================"
echo "1. fresh PX4/Gazebo"
echo "2. readiness check"
echo "3. 2.5 m hover"
echo "4. 20 s stationary control window"
echo "5. q1: 0 -> ${MOTION_Q1_TARGET_DEG:--5.0} deg -> 0 deg"
echo "   max speed: ${MOTION_MAX_SPEED_DEG_S:-0.5} deg/s"
echo "6. 20 s post-motion window"
echo "7. land + auto-disarm"
echo "8. stop simulator + finalize ULog"
echo
echo "q2 remains at zero."
echo "No ground_stow."
echo "============================================================"

SUPERVISOR_LOG="/root/autodl-tmp/desktop_logs/underfold-motion-v1-$(date +%Y%m%d_%H%M%S).log"

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
echo " MOTION FLIGHT"
echo "============================================================"

/usr/bin/python3 ./motion_runner.py

echo
echo "============================================================"
echo " MOTION V1 COMPLETE"
echo " Simulator will now close and ULog will be finalized."
echo "============================================================"
