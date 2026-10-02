#!/usr/bin/env bash
set -o pipefail

cd "$(dirname "$0")"
source ./cloud_env.sh
set -u

CONFIG="${1:-motion_sweep_explore_v1.csv}"
APPROVED="approved_motion_conditions.csv"

SWEEP_ID="${SWEEP_ID:-motion_explore_v1}"
SWEEP_DIR="${RESULTS_DIR}/sweeps/${SWEEP_ID}"

mkdir -p "$SWEEP_DIR"

MANIFEST="$SWEEP_DIR/manifest.csv"

if [[ ! -f "$MANIFEST" ]]; then
    echo \
"run_id,status,target_deg,speed_deg_s,result_dir,pair_diff_delta_hold,max_motor,time_slip_max_s" \
    > "$MANIFEST"
fi

if [[ ! -f "$APPROVED" ]]; then
    echo "[FAIL] $APPROVED does not exist."
    echo "Ground-test and approve conditions first."
    exit 1
fi

if [[ -f pending_ulog_finalize.json ]]; then
    echo "[FAIL] pending_ulog_finalize.json exists."
    exit 1
fi


approved() {
python3 - "$1" "$2" "$APPROVED" <<'PY'
import csv
import sys

target = float(sys.argv[1])
speed = float(sys.argv[2])
path = sys.argv[3]

with open(path) as f:
    rows = list(csv.DictReader(f))

ok = any(
    abs(float(r["target_deg"]) - target) < 1e-9
    and
    abs(float(r["speed_deg_s"]) - speed) < 1e-9
    for r in rows
)

raise SystemExit(0 if ok else 1)
PY
}


while IFS=, read -r RUN_ID TARGET SPEED; do

    if [[ "$RUN_ID" == "run_id" ]]; then
        continue
    fi

    if grep -q "^${RUN_ID},PASS," "$MANIFEST"; then
        echo "[SKIP] $RUN_ID already PASS"
        continue
    fi

    echo
    echo "============================================================"
    echo " SWEEP $RUN_ID"
    echo " target = $TARGET deg"
    echo " speed  = $SPEED deg/s"
    echo "============================================================"

    if ! approved "$TARGET" "$SPEED"; then
        echo "[FAIL] condition is not ground-approved:"
        echo "target=$TARGET speed=$SPEED"
        exit 1
    fi

    BEFORE="$(mktemp)"

    find "$RESULTS_DIR" \
      -maxdepth 1 \
      -type d \
      -name 'motion_underfold_v1_*' \
      -printf '%p\n' \
      | sort > "$BEFORE"

    RUNLOG="$SWEEP_DIR/${RUN_ID}.log"

    if ! \
      MOTION_Q1_TARGET_DEG="$TARGET" \
      MOTION_MAX_SPEED_DEG_S="$SPEED" \
      bash ./run_motion_once.sh \
      2>&1 | tee "$RUNLOG"
    then
        echo "${RUN_ID},FAIL,${TARGET},${SPEED},,,,">> "$MANIFEST"
        rm -f "$BEFORE"
        echo "[FAIL] flight run failed. Sweep stopped."
        exit 1
    fi

    AFTER="$(mktemp)"

    find "$RESULTS_DIR" \
      -maxdepth 1 \
      -type d \
      -name 'motion_underfold_v1_*' \
      -printf '%p\n' \
      | sort > "$AFTER"

    RESULT_DIR="$(
        comm -13 "$BEFORE" "$AFTER"
    )"

    rm -f "$BEFORE" "$AFTER"

    if [[ "$(printf '%s\n' "$RESULT_DIR" | grep -c .)" -ne 1 ]]; then
        echo "[FAIL] could not uniquely identify result directory"
        printf '%s\n' "$RESULT_DIR"
        exit 1
    fi

    echo
    echo "Quick health check:"
    echo "$RESULT_DIR"

    if ! \
      /root/autodl-tmp/px4-venv/bin/python3 \
        ./quick_check_motion_run.py \
        "$RESULT_DIR" \
        "$TARGET" \
        "$SPEED"
    then
        echo "${RUN_ID},FAIL,${TARGET},${SPEED},${RESULT_DIR},,,">> "$MANIFEST"
        echo "[FAIL] health gate failed. Sweep stopped."
        exit 1
    fi

    python3 - \
      "$MANIFEST" \
      "$RUN_ID" \
      "$TARGET" \
      "$SPEED" \
      "$RESULT_DIR" <<'PY'
import csv
import json
import sys
from pathlib import Path

manifest = Path(sys.argv[1])
run_id = sys.argv[2]
target = sys.argv[3]
speed = sys.argv[4]
folder = Path(sys.argv[5])

health = json.loads(
    (folder / "quick_health.json").read_text()
)

with manifest.open("a", newline="") as f:
    w = csv.writer(f)

    w.writerow([
        run_id,
        "PASS",
        target,
        speed,
        str(folder),
        health["pair_diff_delta_hold"],
        health["max_normalized_motor_command"],
        health["time_slip_max_s"],
    ])
PY

    echo "[ OK ] $RUN_ID PASS"

done < "$CONFIG"

echo
echo "============================================================"
echo " SWEEP COMPLETE"
echo "============================================================"
cat "$MANIFEST"
