"""One named experimental timing contract; legacy constants are not aliases."""
from copy import deepcopy

NAME = 'mpc25_dispatch50_v1'
EFFECTIVE = dict(name=NAME, nominal_solve_period_ns=40_000_000,
    output_period_ns=20_000_000, command_lease_ns=160_000_000, command_hold_s=.16,
    control_dt_max_s=.1, solve_roundtrip_max_ns=50_000_000,
    source_wall_age_max_ns=25_000_000, consumption_progress_max_ns=40_000_000,
    hard_realtime_qualified=False)


def effective():
    return deepcopy(EFFECTIVE)


def validate(value):
    if value != EFFECTIVE or any(type(value.get(k)) is not type(v) for k, v in EFFECTIVE.items()):
        raise ValueError('Explicit mpc25_dispatch50_v1 timing values required')
    return effective()
