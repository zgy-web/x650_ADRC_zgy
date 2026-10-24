#!/usr/bin/env python3
"""Launch the separately admitted, complete 24 s fast C++ figure eight."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_cpp_live_v1.launch import main as launch
from simulations.x650_cpp_fast_v1 import task, qualification


def main(argv=None):
    return launch(argv, task_api=task, qualification_api=qualification, scenario=task.SCENARIO,
        default_protocol=ROOT/'configs/experiments/x650_cpp_fast_v1.json',
        extra_sources=('simulations/x650_cpp_normal_v1', 'simulations/x650_cpp_fast_v1',
                       'configs/experiments/x650_cpp_fast_v1.json', 'scenarios/x650_cpp_fast_v1.yaml'),
        launch_prefix='x650_cpp_fast_launch_')


if __name__ == '__main__':
    raise SystemExit(main())
