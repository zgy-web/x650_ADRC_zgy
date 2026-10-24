#!/usr/bin/env python3
"""Independent dynamic-reference math fixtures; never a physical recording."""
from copy import deepcopy
from pathlib import Path
import argparse
import json
import sys

import numpy as np
from export_x650_cpp_inputs import case, configuration, file_identity
from profile_x650_ts20_matrix_fixture import fixture

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from simulations.x650_wholebody_mpc_v1.run_case import FigureEight
from x650_wholebody.api import ControlState, EndpointTarget

PHASES = (0., 1.99, 2., 3.99, 4., 8., 11., 17.99, 18., 19.99, 20., 23.98)
DTS = (.012, .016, .020, .024)
TRAJECTORY = dict(kind='fast_figure8', amplitude_x=.25, amplitude_z=.10,
                  period_s=8., cycles=2, start_s=2., ramp_s=2., duration_s=24.)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--accepted-startup', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    startup = json.loads(args.accepted_startup.read_text())
    predictor = Path(startup['predictor_library']['path'])
    assert file_identity(predictor)['sha256'] == startup['predictor_library']['sha256']
    args.output.mkdir(parents=True, exist_ok=False)
    sources = [Path(__file__), ROOT/'scripts/export_x650_cpp_inputs.py',
               ROOT/'scripts/profile_x650_ts20_matrix_fixture.py',
               ROOT/'simulations/x650_wholebody_mpc_v1/run_case.py',
               *sorted((ROOT/'control_packages/x650_wholebody_v1/x650_wholebody').glob('*.py'))]
    before = [file_identity(p) for p in sources]
    report = dict(kind='cpp_fast_dynamic_raw_input_oracle_v1', complete=False,
                  synthetic_inputs=True, no_flight=True, physical_acceptance=False,
                  accepted_startup=file_identity(args.accepted_startup), sources=before,
                  trajectory=TRAJECTORY, phases_s=PHASES, first_steps_s=DTS, rows=[],
                  scope='Independent Python dynamic targets, matrices, QP and nonlinear candidates. '
                        'Synthetic reference-following states, shifted fixture histories; not measured plant data.')
    for t in PHASES:
        for dt in DTS:
            c, p, state, old_target = fixture(predictor, startup['predictor_library']['sha256'])
            # Use the accepted flight controller gains; all other fixture
            # configuration fields must already equal the accepted startup.
            c.base_position_gain = np.asarray(startup['config']['base_position_gain'])
            assert configuration(c) == startup['config']
            trajectory = FigureEight(old_target.position, **{k:v for k,v in TRAJECTORY.items()
                                                            if k not in ('kind','duration_s')})
            goal = EndpointTarget(*trajectory.sample(t))
            c.trajectory = trajectory
            original = deepcopy(c.context_provider(state, 0.))
            context = deepcopy(original)
            context['reference_state']['time_s'] += t
            context['bus_origin_s'] += t
            context['command_history'] = [(when+t, value) for when,value in context['command_history']]
            for packet in context['packet_history']:
                packet['send_time_s'] += t
            c.context_provider = lambda _s, _t, context=context: deepcopy(context)
            c._plan_t += t
            synthetic = ControlState(state.position + goal.position-old_target.position, state.rotation,
                goal.velocity, state.omega_body, state.q, state.dq, state.stamp_s+t,
                acceleration_enu=goal.acceleration, control_time_s=state.control_time_s+t,
                joint_stamp_s=state.joint_stamp_s+t)
            i = len(report['rows'])
            value = case(c,p,synthetic,goal,t=t,dt=dt,label=dict(index=i,phase_s=t,first_dt_s=dt,
                synthetic_state=True,history_time_shift_s=t,trajectory=TRAJECTORY))
            path = args.output/f'case_{i:03d}.json'
            path.write_text(json.dumps(value,separators=(',', ':'),allow_nan=False))
            report['rows'].append(file_identity(path))
            print(json.dumps(dict(exported=i+1,total=len(PHASES)*len(DTS))),flush=True)
    report.update(complete=True,source_files_unchanged=before == [file_identity(p) for p in sources])
    (args.output/'manifest.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    return 0 if report['source_files_unchanged'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
