"""Frozen, simulation-time experiment definitions. No simulator dependencies."""
from __future__ import annotations

import numpy as np

CASES = {"hover": 30., "step": 35., "arm": 30., "force": 30.,
         "z_smoke": 20., "identify": 45.}
ALGORITHMS = ("pid", "ladrc", "mpc")


def reference(case: str, t: float):
    if case not in CASES:
        raise ValueError(f"Unknown case: {case}")
    offset, q, force = np.zeros(3), np.zeros(2), np.zeros(3)
    if case == "step":
        for axis, begin in enumerate((5., 15., 25.)):
            if begin <= t < begin + 5.:
                offset[axis] = .2
    elif case == "z_smoke" and 5. <= t < 12.:
        offset[2] = .1
    elif case == "arm" and 5. <= t < 13.:
        q[:] = [.15, .10]
    elif case == "force" and 10. <= t < 12.:
        force[0] = 1.
    return offset, q, force


def identification_command(t, position, target):
    command = np.clip(1.2 * (target - position), -.2, .2)
    for axis, begin in enumerate((5., 18., 31.)):
        elapsed = t - begin
        if 0 <= elapsed < 8:
            command[axis] = .1 if elapsed < 2 else (-.1 if 4 <= elapsed < 6 else 0.)
    return command


def limit_command(raw, previous, dt, speed=.4, slew=1.):
    raw = np.asarray(raw, dtype=float)
    previous = np.asarray(previous, dtype=float)
    if raw.shape != (3,) or previous.shape != (3,) or not np.all(np.isfinite([raw, previous])):
        raise ValueError("Invalid three-axis velocity command")
    if not np.isfinite(speed) or not np.isfinite(slew) or speed <= 0 or slew <= 0:
        raise ValueError('Speed and slew must be positive finite scalars')
    if np.any(abs(previous) > speed + 1e-9):
        raise ValueError('Previous command exceeds speed limit')
    if not np.isfinite(dt) or not 0 < dt <= .15:
        raise ValueError(f"Invalid control timestep: {dt}")
    limited = np.clip(np.clip(raw, -speed, speed), previous - slew * dt, previous + slew * dt)
    return limited, bool(np.any(abs(limited - raw) > 1e-7))


def enu_ned(vector):
    x, y, z = vector
    return np.array([y, x, -z], dtype=float)
