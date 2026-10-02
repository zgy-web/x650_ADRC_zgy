from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional


@dataclass(frozen=True)
class VehicleEstimate:
    x: float
    y: float
    z: float
    vx: float
    vy: float
    vz: float
    armed: bool
    landed: bool
    failsafe: bool


@dataclass(frozen=True)
class GroundTruth:
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float


@dataclass(frozen=True)
class JointStateValue:
    position_rad: float
    velocity_rad_s: float
    effort: Optional[float]


@dataclass(frozen=True)
class Observation:
    sim_time_s: float
    vehicle: VehicleEstimate
    arm: Dict[str, JointStateValue]
    ground_truth: Optional[GroundTruth]
