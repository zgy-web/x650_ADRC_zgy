#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
source ./cloud_env.sh
echo 'HEADLESS cloud simulation. No arming or flight commands are sent here.'
echo 'Terminal 2: bash run_cloud.sh --check; then bash run_cloud.sh --stowed'
exec /usr/bin/python3 auto_sim.py
