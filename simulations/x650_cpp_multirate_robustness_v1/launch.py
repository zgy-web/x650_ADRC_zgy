#!/usr/bin/env python3
"""Ground-admit each independent rigid-mismatch trial with the accepted backend."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_cpp_live_v1.launch import main as launch
from simulations.x650_cpp_multirate_robustness_v1 import task, qualification


def main(argv=None):
    import argparse
    parser = argparse.ArgumentParser(add_help=False)
    parser.add_argument('--task', type=Path, required=True)
    selected, _ = parser.parse_known_args(argv)
    cfg = task.validate_task_config(task.read(selected.task))
    return launch(argv, task_api=task, qualification_api=qualification, scenario=cfg['scenario'],
        extra_sources=('simulations/x650_cpp_multirate_robustness_v1','simulations/'+cfg['scenario'],
            'simulations/x650_cpp_multirate_v1','simulations/x650_cpp_robustness_v1',
            'simulations/x650_cpp_fast_v1','simulations/x650_cpp_normal_v1',
            'simulations/x650_cpp_recovery_v1','simulations/x650_cpp_interval_v1',
            'scripts/qualify_x650_multirate_robustness.py',
            'scripts/audit_x650_cpp_multirate_robustness.py','scripts/audit_x650_cpp_multirate_followup.py',
            'scripts/audit_x650_cpp_multirate.py','scripts/audit_x650_interval_constraints.py',
            'scripts/audit_x650_interval_robustness.py','scripts/audit_x650_recovery_robustness.py',
            'scripts/audit_x650_cpp_robustness.py','scripts/prepare_x650_robustness_models.py',
            'scripts/verify_x650_robustness_model.py','scenarios/'+cfg['scenario']+'.yaml',
            'benchmark/whole_body_closed_loop_v1/disturbance.cpp',
            'benchmark/whole_body_closed_loop_v1/disturbance_loader.py',
            'benchmark/whole_body_closed_loop_v1/gz_world_entity_binding.py'),
        launch_prefix='x650_cpp_multirate_robustness_launch_')


if __name__ == '__main__':
    raise SystemExit(main())
