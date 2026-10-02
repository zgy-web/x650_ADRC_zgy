#!/usr/bin/env bash
set -o pipefail

cd "$(dirname "$0")"
source ./cloud_env.sh
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


TARGET="${MOTION_Q1_TARGET_DEG:--5.0}"
SPEED="${MOTION_MAX_SPEED_DEG_S:-0.5}"
ACCEL="${MOTION_MAX_ACCEL_DEG_S2:-0.25}"


echo "============================================================"
echo " UNDERFOLD EXCITATION V2 - ONE RUN"
echo "============================================================"
echo " q1 target    : ${TARGET} deg"
echo " max speed    : ${SPEED} deg/s"
echo " max accel    : ${ACCEL} deg/s^2"
echo
echo " Explicit acceleration-limited trajectory"
echo " q2 remains at zero"
echo " No ground_stow"
echo "============================================================"


SUPERVISOR_LOG="/root/autodl-tmp/desktop_logs/excitation-v2-$(date +%Y%m%d_%H%M%S).log"

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
echo " EXCITATION V2 FLIGHT"
echo "============================================================"

/usr/bin/python3 ./excitation_runner.py


echo
echo "============================================================"
echo " EXCITATION V2 COMPLETE"
echo " Simulator will close and ULog will be finalized."
echo "============================================================"
