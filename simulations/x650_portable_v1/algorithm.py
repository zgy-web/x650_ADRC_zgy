"""Algorithm development seam: ENU metres, seconds, radians.

Replace PositionBaseline.step with your ADRC. PX4 retains its velocity,
attitude and rate loops. The arm input is velocity into a force position servo.
No Gazebo truth is fed into this baseline's aircraft controller.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class State:
    position: np.ndarray  # PX4 estimated position transformed NED -> ENU
    velocity: np.ndarray  # PX4 estimated velocity, ENU
    q: np.ndarray         # upper_arm_joint, fore_crank_joint
    dq: np.ndarray
    stamp_s: float


@dataclass
class Target:
    position: np.ndarray
    q: np.ndarray


@dataclass
class Command:
    velocity: np.ndarray  # desired aircraft ENU velocity, m/s
    joint_velocity: np.ndarray  # rad/s


class PositionBaseline:
    def step(self, t: float, dt: float, state: State, target: Target) -> Command:
        if not 0 < dt <= .15:
            raise ValueError(f'Invalid control timestep: {dt}')
        return Command(np.clip(1.2*(target.position-state.position),-.4,.4),
                       np.clip(1.5*(target.q-state.q),-.15,.15))
