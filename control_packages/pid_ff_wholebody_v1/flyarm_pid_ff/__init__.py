"""Validated x500_underfold whole-body PID + feedforward simulation interface."""
from .api import (PidFeedforwardController, ControlState, EndpointTarget,
                  ControlCommand, Px4Setpoint, frozen_pid_parameters, provenance,
                  PID_SHA256, VERSION)
from ._core.trajectory import FigureEight
from ._core.feedforward import ArmParameters
from ._core.kinematics import tip_world

__version__ = VERSION
__all__ = ["PidFeedforwardController", "ControlState", "EndpointTarget", "ControlCommand",
           "Px4Setpoint", "frozen_pid_parameters", "provenance", "FigureEight",
           "ArmParameters", "tip_world", "PID_SHA256"]
