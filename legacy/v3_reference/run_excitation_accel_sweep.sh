#!/usr/bin/env bash
set -o pipefail

cd "$(dirname "$0")"
source ./cloud_env.sh
set -u

CONFIG="${1:-excitation_accel_explore_v1.csv}"
SWEEP_ID="${SWEEP_ID:-excitation_accel_explore_v1}"
SWEEP_DIR="${RESULTS_DIR}/sweeps/${SWEEP_ID}"

mkdir -p "$SWEEP_DIR"

MANIFEST="$SWEEP_DIR/manifest.csv"

if [[ ! -f "$MANIFEST" ]]; then
    echo \
"run_id,status,target_deg,speed_deg_s,accel_deg_s2,result_dir,motor_max,time_slip_max_s" \
    > "$MANIFEST"
fi


while IFS=, read -r RUN_ID TARGET SPEED ACCEL; do

    [[ "$RUN_ID" == "run_id" ]] && continue

    if grep -q "^${RUN_ID},PASS," "$MANIFEST"; then
        echo "[SKIP] $RUN_ID already PASS"
        continue
    fi

    echo
    echo "============================================================"
    echo " EXCITATION ACCEL SWEEP: $RUN_ID"
    echo " target = $TARGET deg"
    echo " speed  = $SPEED deg/s"
    echo " accel  = $ACCEL deg/s^2"
    echo "============================================================"

    BEFORE="$(mktemp)"

    find "$RESULTS_DIR" \
      -maxdepth 1 \
      -type d \
      -name 'excitation_underfold_v2_*' \
      -printf '%p\n' \
      | sort > "$BEFORE"

    RUNLOG="$SWEEP_DIR/${RUN_ID}_flight.log"

    if ! \
      MOTION_Q1_TARGET_DEG="$TARGET" \
      MOTION_MAX_SPEED_DEG_S="$SPEED" \
      MOTION_MAX_ACCEL_DEG_S2="$ACCEL" \
      bash ./run_excitation_once.sh \
      2>&1 | tee "$RUNLOG"
    then
        echo \
"${RUN_ID},FAIL,${TARGET},${SPEED},${ACCEL},,," \
        >> "$MANIFEST"

        rm -f "$BEFORE"

        echo "[FAIL] Flight run failed."
        echo "Sweep stopped."
        exit 1
    fi

    AFTER="$(mktemp)"

    find "$RESULTS_DIR" \
      -maxdepth 1 \
      -type d \
      -name 'excitation_underfold_v2_*' \
      -printf '%p\n' \
      | sort > "$AFTER"

    RESULT_DIR="$(
        comm -13 "$BEFORE" "$AFTER"
    )"

    rm -f "$BEFORE" "$AFTER"

    if [[ "$(printf '%s\n' "$RESULT_DIR" | grep -c .)" -ne 1 ]]; then
        echo "[FAIL] Cannot identify unique result directory."
        printf '%s\n' "$RESULT_DIR"
        exit 1
    fi

    echo
    echo "Trajectory validation..."

    if ! \
      python3 ./analyze_excitation_trajectory.py \
      "$RESULT_DIR" \
      2>&1 | tee "$SWEEP_DIR/${RUN_ID}_trajectory.log"
    then
        echo \
"${RUN_ID},FAIL,${TARGET},${SPEED},${ACCEL},${RESULT_DIR},," \
        >> "$MANIFEST"

        echo "[FAIL] Trajectory gate failed."
        echo "Sweep stopped."
        exit 1
    fi

    echo
    echo "Flight health validation..."

    if ! \
      /root/autodl-tmp/px4-venv/bin/python3 \
      ./quick_check_excitation_run.py \
      "$RESULT_DIR" \
      2>&1 | tee "$SWEEP_DIR/${RUN_ID}_health.log"
    then
        echo \
"${RUN_ID},FAIL,${TARGET},${SPEED},${ACCEL},${RESULT_DIR},," \
        >> "$MANIFEST"

        echo "[FAIL] Health gate failed."
        echo "Sweep stopped."
        exit 1
    fi

    echo
    echo "Response analysis..."

    /root/autodl-tmp/cloud-arm-venv/bin/python3 \
      ./analyze_motion_simtime.py \
      "$RESULT_DIR" \
      2>&1 | tee "$SWEEP_DIR/${RUN_ID}_response.log"

    /root/autodl-tmp/px4-venv/bin/python3 \
      ./analyze_motion_ulog.py \
      "$RESULT_DIR" \
      2>&1 | tee "$SWEEP_DIR/${RUN_ID}_ulog.log"

    python3 - \
      "$MANIFEST" \
      "$RUN_ID" \
      "$TARGET" \
      "$SPEED" \
      "$ACCEL" \
      "$RESULT_DIR" <<'PY'
import csv
import json
import sys
from pathlib import Path

manifest = Path(sys.argv[1])
run_id = sys.argv[2]
target = sys.argv[3]
speed = sys.argv[4]
accel = sys.argv[5]
folder = Path(sys.argv[6])

health = json.loads(
    (folder / "quick_health_excitation_v2.json").read_text()
)

with manifest.open("a", newline="") as f:
    csv.writer(f).writerow([
        run_id,
        "PASS",
        target,
        speed,
        accel,
        str(folder),
        health["motor_max"],
        health["time_slip_max_s"],
    ])
PY

    echo
    echo "[ OK ] $RUN_ID PASS"

done < "$CONFIG"


echo
echo "============================================================"
echo " ACCELERATION EXPLORATION COMPLETE"
echo "============================================================"

cat "$MANIFEST"
