"""Frozen fresh response experiments, separately bound from operating conditions.

This protocol changes the command sequence and model identification procedure,
not the physical model, estimator, PID, interface or existing safety limits.
It is an input specification; its presence never certifies a successful flight
or an accepted response model. Archived tasks without it retain their behavior.
"""
from __future__ import annotations

from copy import deepcopy
import hashlib
import json
import math


PROTOCOL_NAME = "xy_second_order_z_first_order_v1"
ROLES = ("train", "holdout")


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False)


def _build(role):
    if role not in ROLES:
        raise ValueError("Response protocol role must be train or holdout")
    intervals = ([(2., 2.8, 1.), (5.3, 6.7, -1.), (9.2, 10.6, 1.), (13.1, 13.9, -1.)]
                 if role == "train" else
                 [(2., 3.1, 1.), (5.4, 7.1, -1.), (9.4, 11.1, 1.), (13.4, 14.5, -1.)])
    return {
        "kind": "x650_frozen_response_protocol", "schema_version": 1,
        "name": PROTOCOL_NAME, "role": role,
        "duration_s": 60., "prediction_window_s": [0., 54.],
        "excitation_end_s": 54., "return_anchor_start_s": 54.,
        "axis_windows_s": [[0., 18.], [18., 36.], [36., 54.]],
        "amplitude_m_s": [.08, .08, .05],
        "local_pulses": [{"start_s": start, "end_s": end, "sign": sign}
                         for start, end, sign in intervals],
        "outside_pulse_command": "zero_velocity_until_return_anchor",
        "return_anchor": {"position_gain_s_inv": 1.5, "velocity_limit_m_s": .12},
        "initial_zero_s": 2.,
        "model_structure": {"x": "unit_gain_zero_delay_second_order",
                            "y": "unit_gain_zero_delay_second_order",
                            "z": "unit_gain_zero_delay_first_order"},
        "parameter_selection": {
            "training_only": True, "gain": 1., "command_delay_s": 0., "velocity_bias_m_s": 0.,
            "fit_windows_s": [[1., 18.], [18., 36.], [36., 54.]],
            "xy": {"method": "scipy.optimize.least_squares_log_parameters",
                   "wn_bounds_rad_s": [.3, 30.], "zeta_bounds": [.1, 5.],
                   "multistarts": [[2., .5], [5., 1.], [12., 2.]],
                   "ftol": 1e-10, "xtol": 1e-10, "gtol": 1e-10, "max_nfev": 150},
            "z": {"method": "scipy.optimize.minimize_scalar_bounded_log_tau",
                  "tau_bounds_s": [.01, 1.5], "xatol": 1e-8},
            "minimum_bound_log_fraction": 1e-4,
            "sensitivity_parameter_factors": [.8, 1.2],
            "minimum_sensitivity_mse_increase_fraction": .02,
        },
        "initialization": {
            "velocity": "first_native_sample_of_each_run", "acceleration_m_s2": [0., 0., 0.],
            "quiet_window_s": [0., 1.], "quiet_velocity_peak_limit_m_s": .01,
            "quiet_velocity_slope_limit_m_s2": .01,
            "reset_between_axes": False, "reset_between_pulses": False,
        },
        "acceptance": {
            "score_windows_s": [[1., 18.], [18., 36.], [36., 54.]],
            "score_each_active_pulse": True, "score_each_following_zero_rest": True,
            "minimum_axis_samples": 120, "maximum_native_sample_gap_s": .008,
            "minimum_velocity_span_fraction": .50,
            "nrmse_denominator": "fixed_declared_command_amplitude",
            "maximum_nrmse": .20, "maximum_rmse_xy_m_s": .015, "maximum_rmse_z_m_s": .010,
            "maximum_error_fraction": .35, "maximum_error_floor_m_s": .002,
            "maximum_bias_fraction": .10, "maximum_bias_floor_m_s": .001,
            "maximum_peak_error_fraction": .25, "maximum_peak_error_floor_m_s": .002,
            "maximum_cross_axis_rms_fraction": .30, "maximum_cross_axis_rms_floor_m_s": .003,
            "delay_diagnostic": {
                "minimum_s": 0., "maximum_s": .12, "step_s": .005,
                "frozen_parameters": True, "maximum_material_delay_s": .030,
                "material_improvement_fraction": .20, "material_improvement_m_s": .001,
            },
        },
        "independent_holdout": True,
        "reset_between_pulses": False,
    }


def build_response_protocol(role):
    """Return a fresh copy of the reviewed role-specific specification."""
    return deepcopy(_build(role))


def validate_response_protocol(value):
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError("Response probe protocol must be a reviewed object")
    expected = _build(value.get("role"))
    try:
        matches = _canonical(value) == _canonical(expected)
    except (TypeError, ValueError) as exc:
        raise ValueError("Response probe protocol contains invalid values") from exc
    if not matches:
        raise ValueError("Response probe protocol differs from the frozen specification")
    return deepcopy(expected)


def response_protocol_hash(value):
    validated = validate_response_protocol(value)
    if validated is None:
        raise ValueError("An explicit response protocol is required for its hash")
    return hashlib.sha256(_canonical(validated).encode()).hexdigest()


def protocol_velocity(t, protocol):
    """Evaluate an already validated specification; no position return is hidden here.

    Pulses use half-open intervals [start, end). The runner switches back to its
    existing measured-position anchor controller at return_anchor_start_s.
    """
    if not math.isfinite(t):
        raise ValueError("Response protocol time must be finite")
    velocity = [0., 0., 0.]
    if t < 0. or t >= protocol["excitation_end_s"]:
        return velocity
    for axis, (start, end) in enumerate(protocol["axis_windows_s"]):
        if start <= t < end:
            local = t-start
            for pulse in protocol["local_pulses"]:
                if pulse["start_s"] <= local < pulse["end_s"]:
                    velocity[axis] = pulse["sign"]*protocol["amplitude_m_s"][axis]
                    return velocity
            return velocity
    return velocity


def protocol_annotation(cfg):
    """Attach independent protocol identity without altering operating-state bytes."""
    value = validate_response_protocol(cfg.get("response_probe_protocol"))
    if value is None:
        return {}
    return {"response_probe_protocol": value,
            "response_probe_protocol_sha256": response_protocol_hash(value)}
