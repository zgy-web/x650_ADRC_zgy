#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"

if [[ $# -ne 1 || ( "$1" != "--check" && "$1" != "--baseline" ) ]]; then
    echo "Usage:"
    echo "  bash run_cloud.sh --check"
    echo "  bash run_cloud.sh --baseline"
    echo
    echo "--check    : readiness only; NO ARM / NO FLIGHT"
    echo "--baseline : reviewed underfold stationary-arm flight"
    echo
    echo "No ground_stow. No root-small. No arm trajectory."
    exit 2
fi

source ./cloud_env.sh

if [[ "$1" == "--check" ]]; then
    exec /root/autodl-tmp/cloud-arm-venv/bin/python3 \
        check_underfold.py
fi

exec /root/autodl-tmp/cloud-arm-venv/bin/python3 \
    baseline_underfold.py
