#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
if [[ $# -ne 1 || ( "$1" != "--check" && "$1" != "--stowed" && "$1" != "--root-small" ) ]]; then
  echo 'Usage: bash run_cloud.sh --check | --stowed | --root-small'
  echo '--check does not fly. --stowed folds the arm, arms, takes off, records, and lands.'
  exit 2
fi
source ./cloud_env.sh
exec /root/autodl-tmp/cloud-arm-venv/bin/python3 auto_experiment.py "$@"
