#!/bin/bash
set -eo pipefail
source /root/autodl-tmp/x650-nmpc-standard-runtime/environment.sh
cd /root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/standard_nmpc_20261027/source_numeric_v3
python -m unittest discover -s tests/unit -p test_x650_standard_solver.py -v > ../solver_contract_tests_v3.log 2>&1
python -u scripts/qualify_x650_standard_nmpc.py \
  --selection ../hold_000_selected.json \
  --selection ../hold_001_selected.json \
  --selection ../wrench_000_selected.json \
  --selection ../wrench_001_selected.json \
  --output ../numerical_v3
