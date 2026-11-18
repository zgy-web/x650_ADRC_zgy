"""Fit the no-pure-delay prediction model actually used by the outer loops.

Only PX4 estimated velocities and actually transmitted commands are used.
Training uses the positive pulse and subsequent zero command. The following
negative pulse is held out for recursive prediction, never parameter selection.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares


GAIN_BOUNDS = (0.1, 3.0)
TAU_BOUNDS = (0.04, 2.0)
VALIDATION_LIMIT = 0.08


def _values(rows, key):
    try:
        values = np.array([float(row[key]) for row in rows])
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"Identification requires numeric {key}") from exc
    if not np.all(np.isfinite(values)):
        raise ValueError(f"Identification contains nonfinite {key}")
    return values


def _held_intervals(measurement_times, command_times, commands):
    """Split each observed interval at actual command changes (zero-order hold)."""
    intervals = []
    for begin, end in zip(measurement_times[:-1], measurement_times[1:]):
        changes = command_times[(command_times > begin+1e-10) & (command_times < end-1e-10)]
        boundaries = np.r_[begin, changes, end]
        indices = np.searchsorted(command_times, boundaries[:-1]+1e-10, side="right")-1
        indices = np.clip(indices, 0, len(commands)-1)
        intervals.append((np.diff(boundaries), commands[indices]))
    return intervals


def _prediction(parameters, initial_velocity, intervals):
    gain, tau = parameters
    value = float(initial_velocity)
    values = [value]
    for durations, commands in intervals:
        for dt, command in zip(durations, commands):
            decay = np.exp(-dt/tau)
            value = decay*value+(1-decay)*gain*command
        values.append(value)
    return np.array(values)


def fit_response(rows) -> dict:
    """Return the effective gain/tau model and independent validation evidence.

    The time basis is the command/observation receipt clock ``t``, not a claim of
    physical actuator time. ``px4_stamp_us`` only deduplicates repeated source
    measurements. ``delay_s`` is exactly zero by model definition: the fitted
    time constant approximates the observed rise time including inner loops and
    transport. Boundary-hitting or poorly predicting fits are not admitted.
    """
    rows = list(rows)
    if not rows:
        raise ValueError("Empty identification trace")
    all_times = _values(rows, "t")
    rows = [row for row, t in zip(rows, all_times) if t >= 0.]
    if len(rows) < 2:
        raise ValueError("No in-trial identification samples")
    command_t = _values(rows, "t")
    if np.any(np.diff(command_t) <= 0):
        raise ValueError("Identification command times must strictly increase")
    unique = {}
    last_stamp = None
    for row in rows:
        try:
            stamp = int(row["px4_stamp_us"])
        except (KeyError, ValueError, TypeError) as exc:
            raise ValueError("Identification requires PX4 source timestamps") from exc
        if last_stamp is not None and stamp < last_stamp:
            raise ValueError("PX4 measurement timestamps moved backwards")
        unique.setdefault(stamp, row)
        last_stamp = stamp
    measurements = list(unique.values())
    times = _values(measurements, "t")
    if len(times) < 2 or np.any(np.diff(times) > 0.15+1e-9):
        raise ValueError("Identification has insufficient/far-spaced independent measurements")

    model = dict(
        schema_version=1, identified=False, gain=[], tau=[], delay_s=[0., 0., 0.],
        bias_velocity=[0., 0., 0.], fit_rmse_m_s=[], validation_rmse_m_s=[],
        fit_boundary_hit=[], fit_samples=[], validation_samples=[],
        equation="p_dot=v; tau*v_dot=-v+gain*u; zero pure delay and zero fitted bias",
        fitting="Recursive positive-pulse trajectory least squares for gain/tau; fixed bias=0 and delay=0; negative pulse held out",
        time_basis="Effective observed response at control receipt time; source timestamps deduplicate samples",
        delay_policy="Effective tau absorbs observed inner-loop/transport rise time; no claim of a physical delay measurement",
        parameter_bounds={"gain": list(GAIN_BOUNDS), "tau_s": list(TAU_BOUNDS)},
        validation_limit_m_s=VALIDATION_LIMIT,
    )
    lower = np.array([GAIN_BOUNDS[0], TAU_BOUNDS[0]])
    upper = np.array([GAIN_BOUNDS[1], TAU_BOUNDS[1]])
    for axis, begin in zip("xyz", (5., 18., 31.)):
        velocity = _values(measurements, "v"+axis)
        commands = _values(rows, "v"+axis+"_cmd")
        train = np.flatnonzero((times >= begin-1e-9) & (times < begin+4.-1e-9))
        validate = np.flatnonzero((times >= begin+4.-1e-9) & (times < begin+8.-1e-9))
        if len(train) < 40 or len(validate) < 40:
            raise ValueError(f"Too few independent samples for {axis} fit/validation")
        training_u = commands[(command_t >= begin) & (command_t < begin+4.)]
        validation_u = commands[(command_t >= begin+4.) & (command_t < begin+8.)]
        if (not len(training_u) or not len(validation_u) or np.ptp(training_u) < .03
                or np.max(training_u) < .03 or np.min(validation_u) > -.03):
            raise ValueError(f"Insufficient positive/negative command excitation for {axis}")
        train_intervals = _held_intervals(times[train], command_t, commands)
        validation_intervals = _held_intervals(times[validate], command_t, commands)
        fit = least_squares(
            lambda parameters: _prediction(parameters, velocity[train[0]], train_intervals)-velocity[train],
            np.array([1., .5]), bounds=(lower, upper),
            ftol=1e-10, xtol=1e-10, gtol=1e-10, max_nfev=300,
        )
        if not fit.success or not np.all(np.isfinite(fit.x)):
            raise ValueError(f"Recursive response fit failed for {axis}: {fit.message}")
        boundary = bool(np.any(fit.x-lower < (upper-lower)*1e-3)
                        or np.any(upper-fit.x < (upper-lower)*1e-3))
        prediction = _prediction(fit.x, velocity[validate[0]], validation_intervals)
        model["gain"].append(float(fit.x[0]))
        model["tau"].append(float(fit.x[1]))
        model["fit_rmse_m_s"].append(float(np.sqrt(np.mean(fit.fun**2))))
        model["validation_rmse_m_s"].append(float(np.sqrt(np.mean((prediction-velocity[validate])**2))))
        model["fit_boundary_hit"].append(boundary)
        model["fit_samples"].append(int(len(train)))
        model["validation_samples"].append(int(len(validate)))
    model["validation_accepted"] = bool(not any(model["fit_boundary_hit"])
                                        and max(model["validation_rmse_m_s"]) < VALIDATION_LIMIT)
    model["identified"] = model["validation_accepted"]
    return model
