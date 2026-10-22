"""Shared state and servo-consistent setpoint contract for the X650.

Positions, linear velocities and optional measured accelerations are world ENU. Rotation maps body FLU to
world ENU; angular velocity is body FLU. Joint coordinates are measured
increments from the folded model home, in rad/rad/s. No controller or gains
are selected by constructing these records.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import numpy as np


def vector(value, n, name):
    a = np.asarray(value, dtype=float)
    if a.shape != (n,) or not np.isfinite(a).all():
        raise ValueError(f"{name} must be a finite {n}-vector")
    a = a.copy()
    a.setflags(write=False)
    return a


def rotation(value):
    a = np.asarray(value, dtype=float)
    if (a.shape != (3, 3) or not np.isfinite(a).all()
            or not np.allclose(a.T @ a, np.eye(3), atol=1e-6)
            or not np.isclose(np.linalg.det(a), 1., atol=1e-6)):
        raise ValueError("rotation must map body FLU to world ENU as a proper rotation")
    a = a.copy()
    a.setflags(write=False)
    return a


def enu_to_ned(value):
    x, y, z = vector(value, 3, "ENU vector")
    return np.array([y, x, -z])


@dataclass(frozen=True)
class ControlState:
    position: np.ndarray
    rotation: np.ndarray
    velocity: np.ndarray
    omega_body: np.ndarray
    q: np.ndarray
    dq: np.ndarray
    stamp_s: float
    acceleration_enu: np.ndarray | None = None
    control_time_s: float | None = None
    joint_stamp_s: float | None = None

    def __post_init__(self):
        for name, n in (("position", 3), ("velocity", 3), ("omega_body", 3), ("q", 2), ("dq", 2)):
            object.__setattr__(self, name, vector(getattr(self, name), n, name))
        object.__setattr__(self, "rotation", rotation(self.rotation))
        if self.acceleration_enu is not None:
            object.__setattr__(self, "acceleration_enu", vector(self.acceleration_enu, 3, "acceleration_enu"))
        if not np.isfinite(self.stamp_s) or self.stamp_s < 0:
            raise ValueError("stamp_s must be a nonnegative simulation timestamp")
        for name in ("control_time_s", "joint_stamp_s"):
            value = getattr(self, name)
            if value is not None and (isinstance(value, (bool, np.bool_)) or not np.isfinite(value) or value < 0):
                raise ValueError(name+" must be a nonnegative finite simulation timestamp")


@dataclass(frozen=True)
class EndpointTarget:
    position: np.ndarray
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3))

    def __post_init__(self):
        for name in ("position", "velocity", "acceleration"):
            object.__setattr__(self, name, vector(getattr(self, name), 3, name))


@dataclass(frozen=True)
class Px4Setpoint:
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray
    type_mask: int = 2048  # p/v/a/yaw enabled; yaw-rate ignored; acceleration mode.


@dataclass(frozen=True)
class ControlCommand:
    position_reference: np.ndarray
    velocity_feedforward: np.ndarray
    acceleration_feedforward: np.ndarray
    base_velocity_target: np.ndarray
    joint_velocity: np.ndarray
    diagnostics: dict
    _feedback_position: np.ndarray = field(repr=False)
    _feedback_velocity: np.ndarray = field(repr=False)

    def __post_init__(self):
        for name, n in (("position_reference", 3), ("velocity_feedforward", 3),
                        ("acceleration_feedforward", 3), ("base_velocity_target", 3),
                        ("joint_velocity", 2), ("_feedback_position", 3), ("_feedback_velocity", 3)):
            object.__setattr__(self, name, vector(getattr(self, name), n, name))

    @classmethod
    def from_executed_velocity(cls, state, base_velocity, joint_velocity, *,
                               nominal_position, acceleration_feedforward,
                               position_gains, diagnostics=None):
        """Subtract the known PX4 position P before sending velocity FF.

        PX4's combined position/velocity reference reconstructs exactly
        base_velocity. Joint velocity is the final executed command; callers
        must not add a second joint-position P term.
        """
        gains = vector(position_gains, 3, "position_gains")
        if np.any(gains <= 0):
            raise ValueError("position gains must be the positive actual PX4 readback")
        p_ref = vector(nominal_position, 3, "nominal_position")
        desired = vector(base_velocity, 3, "base_velocity")
        v_ff = desired - gains * (p_ref - state.position)
        return cls(p_ref, v_ff, acceleration_feedforward, desired, joint_velocity,
                   dict(diagnostics or {}), state.position, state.velocity)

    def px4_setpoint(self, px4_position_ned, px4_velocity_ned):
        """Preserve the state snapshot's actual feedback errors in PX4 NED."""
        ep = vector(px4_position_ned, 3, "PX4 position")
        ev = vector(px4_velocity_ned, 3, "PX4 velocity")
        return Px4Setpoint(
            ep + enu_to_ned(self.position_reference - self._feedback_position),
            ev + enu_to_ned(self.velocity_feedforward - self._feedback_velocity),
            enu_to_ned(self.acceleration_feedforward))
