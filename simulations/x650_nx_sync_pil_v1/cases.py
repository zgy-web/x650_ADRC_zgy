"""Declared trajectory changes and a bound, new physical hold prerequisite."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path


NORMAL = {'kind': 'normal_figure8', 'amplitude_x': .25, 'amplitude_z': .10,
          'period_s': 12., 'cycles': 2, 'start_s': 2., 'ramp_s': 2., 'duration_s': 32.}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def bound(binding):
    if set(binding) != {'path', 'sha256'} or digest(binding['path']) != binding['sha256']:
        raise ValueError('Changed bound evidence')
    return json.loads(Path(binding['path']).read_bytes())


def compose(original, plan):
    cfg = deepcopy(original)
    cfg['scenario'] = 'x650_nx_sync_pil_v1'
    if plan['case'] == 'tip_hold':
        return cfg, {'kind': 'hold'}
    if plan['case'] != 'normal':
        raise ValueError('Only full hold followed by normal figure eight is admitted')
    gate = plan['prior_sync_hold']
    report, launch = bound(gate['report']), bound(gate['launch'])
    if not (report.get('passed') is True and report.get('complete') is True
            and report.get('physical_closed_loop') is True
            and report.get('wall_clock_qualified') is False
            and report.get('nx_steps') == 749
            and report.get('endpoint', {}).get('duration_s') == 30.
            and launch.get('complete') is True and launch.get('runtime_closed') is True
            and launch['plan']['case'] == 'tip_hold'):
        raise ValueError('New complete synchronous physical hold required')
    for field in ('source_task', 'startup_manifest', 'nx_source_sha256', 'physics_tick_ns', 'solve_ticks', 'dispatch_ticks'):
        if plan[field] != launch['plan'][field]:
            raise ValueError('Normal must retain the accepted model, controller and clock: ' + field)
    paths = report.get('artifacts', {})
    if not paths or any(digest(p) != h for p, h in paths.items()):
        raise ValueError('Accepted physical hold records changed')
    cfg.update(stage='normal', trajectory='normal', duration_s=NORMAL['duration_s'],
               trajectory_parameters={k: v for k, v in NORMAL.items() if k not in {'kind', 'duration_s'}})
    return cfg, deepcopy(NORMAL)
