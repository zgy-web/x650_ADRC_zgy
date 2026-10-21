"""Declared six-step PX4 position-loop commissioning protocol, without IO.

Times are elapsed simulation seconds from the start of the stationary hover
measurement, not from arming or takeoff. The caller fixes the world anchor once,
adds these offsets, and encodes each declared target against the same verified
PX4 origin. This module never observes measured positions or adjusts a target.
"""
from __future__ import annotations
from copy import deepcopy
import hashlib
import math
from pathlib import Path

STATIC_HOVER_S = 20.
SEGMENT_DURATION_S = 6.
PROBE_DURATION_S = 36.
TOTAL_DURATION_S = STATIC_HOVER_S+PROBE_DURATION_S
_ZERO = (0., 0., 0.)
_TARGETS = ((.15, 0., 0.), _ZERO, (0., .15, 0.), _ZERO, (0., 0., .10), _ZERO)
_AXES = ("x", "x", "y", "y", "z", "z")


def _elapsed(value):
    if isinstance(value, bool):
        raise ValueError("Elapsed simulation time must be a finite nonnegative number")
    try:
        value = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError("Elapsed simulation time must be a finite nonnegative number") from exc
    if not math.isfinite(value) or value < 0.:
        raise ValueError("Elapsed simulation time must be a finite nonnegative number")
    return value


def _segments():
    segments = []
    previous = _ZERO
    for index, (axis, target) in enumerate(zip(_AXES, _TARGETS)):
        delta = tuple(a-b for a, b in zip(target, previous))
        axis_index = "xyz".index(axis)
        segments.append({
            "segment_index": index, "segment_id": f"{index+1:02d}_{axis}_"+("out" if index % 2 == 0 else "return"),
            "axis": axis, "axis_index": axis_index,
            "start_s": STATIC_HOVER_S+index*SEGMENT_DURATION_S,
            "end_s": STATIC_HOVER_S+(index+1)*SEGMENT_DURATION_S,
            "duration_s": SEGMENT_DURATION_S,
            "target_offset_world_enu_m": list(target),
            "previous_target_offset_world_enu_m": list(previous),
            "command_delta_world_enu_m": list(delta),
            "signed_step_m": delta[axis_index], "step_amplitude_m": abs(delta[axis_index]),
            "direction": 1 if delta[axis_index] > 0 else -1,
        })
        previous = target
    return segments


def segment_at(elapsed_s):
    """Fresh segment dict for [start,end); None before/after the 36 s probe."""
    elapsed_s = _elapsed(elapsed_s)
    if elapsed_s < STATIC_HOVER_S or elapsed_s >= TOTAL_DURATION_S:
        return None
    return _segments()[min(5, int((elapsed_s-STATIC_HOVER_S)//SEGMENT_DURATION_S))]


def world_offset(elapsed_s):
    """Declared [east,north,up] offset, held constant in each six-second step."""
    segment = segment_at(elapsed_s)
    return list(_ZERO) if segment is None else segment["target_offset_world_enu_m"]


def probe_metadata():
    """Serializable contract for controller logs and independent response audit."""
    return deepcopy({
        "kind": "x650_fixed_world_position_step_probe", "schema_version": 1,
        "profile": "single_axis_out_return_6x6s_v1",
        "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
        "time_basis": "Elapsed simulation seconds from stationary hover measurement start, excluding takeoff",
        "coordinate_frame": "Gazebo world ENU: x=east, y=north, z=up",
        "static_hover_s": STATIC_HOVER_S, "probe_start_s": STATIC_HOVER_S,
        "probe_duration_s": PROBE_DURATION_S, "total_duration_s": TOTAL_DURATION_S,
        "segment_duration_s": SEGMENT_DURATION_S, "segment_count": 6,
        "boundary_policy": "Each segment includes start and excludes end; final target remains zero offset",
        "anchor_policy": "Fixed declared world anchor; never move it to measured position between segments",
        "command_policy": "Add absolute declared offset to anchor; preserve the same verified PX4 origin for every target",
        "sequence_rationale": "Each axis has a positive step and an equal negative return step, each observed for 6 s; no simultaneous commanded axis changes",
        "negative_direction_semantics": "Return steps have negative increments but never demand a negative absolute world offset",
        "segments": _segments(),
        "analysis_contract": {
            "rise_fraction": [.10, .90],
            "step_size_basis": "Adjacent declared targets, not fitted measured start/end positions",
            "settling_position_error_m": .02,
            "settling_speed_m_s": .03,
            "settling_continuous_duration_s": 2.,
            "settling_policy": "Use the declared target and measured speed; report not settled if the observation window is insufficient",
            "no_fitted_recentring": True,
            "report_cross_axis_response": True,
        },
        "simulation_only": True,
    })
