set -eo pipefail
source /root/autodl-tmp/x650-nmpc-standard-runtime/environment.sh
S=/root/autodl-tmp/safe-aerial-manipulator-testbed/artifacts/standard_nmpc_20261027
cd "$S/source_numeric_v4_audit"
python -m unittest discover -s tests/unit -p test_x650_standard_solver.py -v > "$S/solver_contract_tests_v4_audit.log" 2>&1
python -u scripts/qualify_x650_standard_nmpc.py --mode ipopt_only --selection "$S/hold_000_selected.json" --selection "$S/hold_001_selected.json" --selection "$S/wrench_000_selected.json" --selection "$S/wrench_001_selected.json" --output "$S/numerical_v4_ipopt_audit"
