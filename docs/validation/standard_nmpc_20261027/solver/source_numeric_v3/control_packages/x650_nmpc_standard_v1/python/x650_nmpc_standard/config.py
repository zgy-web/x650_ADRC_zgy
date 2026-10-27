"""Explicit configuration for the new smooth OCP, not the TS20 event model.

Physical values are copied from a bound ABI-4 Config.  The smoothing width and
integration choices are additional *model* assumptions and must be reported.
They are not hardware identification results or relaxed safety limits.
"""
from __future__ import annotations

from copy import deepcopy
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from typing import Mapping

import numpy as np


def canonical_sha256(value) -> str:
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


@dataclass(frozen=True)
class SurrogateConfig:
    """New, deliberately declared smooth optimization surrogate.

    f(v)=(Fc+delta*exp(-(v/vs)**2))*tanh(v/epsilon).  This has no
    static sticking state.  Fs remains source metadata; it is not silently
    reinterpreted as an identified smooth coefficient.  Future quantization is
    replaced by the identity map.  Already sent packets retain their values.
    """
    velocity_smoothing_rad_s: float = 0.05
    rk4_substeps_per_bus: int = 10
    history_rk4_substeps_per_bus: int = 20
    event_slots: int = 22
    rotation_norm_regularizer: float = 1e-12
    q_padding_rad: float = 0.0003
    dq_padding_rad_s: float = 0.03
    attitude_decay_s: float = 0.15
    gravity_m_s2: float = 9.8

    def __post_init__(self):
        if not (math.isfinite(self.velocity_smoothing_rad_s)
                and self.velocity_smoothing_rad_s > 0):
            raise ValueError("Positive explicit smooth-friction width required")
        if type(self.rk4_substeps_per_bus) is not int or self.rk4_substeps_per_bus < 10:
            raise ValueError("At least ten RK4 substeps per 5 ms bus interval required")
        if type(self.history_rk4_substeps_per_bus) is not int or self.history_rk4_substeps_per_bus < 20:
            raise ValueError("At least twenty substeps required for initial actual-history alignment")
        if self.event_slots != 22:
            raise ValueError("The v1 stage schema has exactly 22 fixed event slots")
        if (self.q_padding_rad != .0003 or self.dq_padding_rad_s != .03
                or self.attitude_decay_s != .15 or self.gravity_m_s2 != 9.8
                or self.rotation_norm_regularizer != 1e-12):
            raise ValueError("Changing safety/geometry conventions requires a new schema")

    def as_dict(self):
        return asdict(self)


class ModelConfig:
    """Validated copy of the existing native Config; never changes its values."""
    VECTOR_LENGTHS = {
        "geometry": 10, "response": 5, "motion": 8, "weights": 6,
        "velocity_limit": 5, "acceleration_limit": 5, "q_lower": 2,
        "q_upper": 2, "physical_lower": 2, "physical_upper": 2,
        "kp": 2, "kd": 2, "base_position_gain": 3,
    }

    def __init__(self, values: Mapping):
        self._values = deepcopy(dict(values))
        for key, length in self.VECTOR_LENGTHS.items():
            vector = np.asarray(self._values[key], dtype=float)
            if vector.shape != (length,) or not np.isfinite(vector).all():
                raise ValueError(f"Invalid native configuration vector: {key}")
            setattr(self, key, vector.copy())
        for key in ("node_dt", "max_dt", "command_hold_s", "bus_period",
                    "integration_step", "arm_share", "feasibility_tolerance",
                    "kkt_tolerance", "joint_position_gain"):
            value = float(self._values[key])
            if not math.isfinite(value):
                raise ValueError(f"Invalid native scalar: {key}")
            setattr(self, key, value)
        self.nodes = self._values["nodes"]
        if self.nodes != 15 or self.node_dt != .04 or self.max_dt != .1:
            raise ValueError("v1 requires original 15-node/40 ms/max-dt 100 ms configuration")
        if self.bus_period != .005 or not 0 < self.motion[4] < self.bus_period:
            raise ValueError("v1 pending-packet state requires 0 < delay < 5 ms bus period")
        if self.command_hold_s != .16:
            raise ValueError("Original 160 ms held-command contract required")
        if (np.any(self.velocity_limit <= 0) or np.any(self.acceleration_limit <= 0)
                or np.any(self.response <= 0) or self.motion[0] <= 0
                or np.any(self.kp <= 0) or np.any(self.kd < 0)
                or np.any(self.weights < 0) or np.any(self.geometry[:2] <= 0)
                or np.any(self.q_lower >= self.q_upper)
                or np.any(self.physical_lower > self.q_lower)
                or np.any(self.physical_upper < self.q_upper)):
            raise ValueError("Invalid physical scales or limits")
        if not np.allclose(self.velocity_limit[3:]-.03, [.42, .32], atol=1e-14, rtol=0):
            raise ValueError("Original 0.42/0.32 rad/s prediction inner bounds required")
        if not (self.motion[1] >= 0 and self.motion[2] >= 0 and self.motion[5] >= 0
                and self.motion[6] >= 0 and self.motion[7] > 0):
            raise ValueError("Invalid source friction coefficients")
        for value in vars(self).values():
            if isinstance(value, np.ndarray):
                value.flags.writeable = False

    @classmethod
    def from_dict(cls, value):
        return value if isinstance(value, cls) else cls(value)

    def as_dict(self):
        return deepcopy(self._values)

    @property
    def sha256(self):
        return canonical_sha256(self._values)


def validate_surrogate(config: ModelConfig, surrogate: SurrogateConfig):
    """Conservative local RK4 stiffness screen, not a global accuracy proof."""
    c, s = config, surrogate
    step = c.bus_period/s.rk4_substeps_per_bus
    # Bound |f'| by the tanh slope plus maximum Gaussian slope.  Include
    # damping and spring coupling.  Accuracy is independently checked by
    # halving steps; this screen only rejects obviously unstable selections.
    friction_slope = ((c.motion[2]+c.motion[6])/s.velocity_smoothing_rad_s
                      + math.sqrt(2/math.e)*c.motion[6]/c.motion[7])
    screen = step * (c.motion[1]+float(c.kd.max())+friction_slope
                     +float(c.kp.max())*step)/c.motion[0]
    if screen > 2.0:
        raise ValueError("Smooth friction is too stiff for the declared RK4 step; increase substeps")
    return dict(maximum_joint_substep_s=step, conservative_stiffness_number=screen,
                maximum_history_substep_s=c.bus_period/s.history_rk4_substeps_per_bus,
                step_halving_validation_required=True, empirical_event_fit_verified=False)


NativeConfig = ModelConfig
