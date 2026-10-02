#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source ./cloud_env.sh

echo "============================================================"
echo " UNDERFOLD V3 SIMULATION"
echo "============================================================"
echo "No arming or flight commands are sent by this process."
echo "After services are ready, use another terminal:"
echo
echo "  bash run_cloud.sh --check"
echo
echo "Do NOT run reference_v2 scripts."
echo "============================================================"

exec /usr/bin/python3 auto_sim.py
