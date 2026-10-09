"""Fail-closed HITL arming gate. Pure logic: never connects to a flight controller."""
from __future__ import annotations
from dataclasses import dataclass
from typing import Mapping

@dataclass(frozen=True)
class HitlGate:
    allowed: bool
    reasons: tuple[str, ...]

def check_hitl_gate(config: Mapping, *, hb_hil_enabled: bool, heartbeat_age_s: float,
                    sensor_age_s: float, actuator_age_s: float) -> HitlGate:
    """Prevent accidental enabling of hardware feedback in unvalidated setups."""
    reasons=[]
    if config.get("backend") != "hardware": reasons.append("hardware backend not selected")
    if config.get("control_enabled") is not True: reasons.append("control approval missing")
    if config.get("hil_injection_enabled") is not True: reasons.append("HITL injection approval missing")
    if config.get("physical_actuators_disconnected") is not True: reasons.append("physical actuators not verified disconnected")
    if config.get("hardware_board_verified") is not True: reasons.append("hardware identity not verified")
    if config.get("hitl_firmware_verified") is not True: reasons.append("PX4 HITL firmware not verified")
    if config.get("operator_present") is not True: reasons.append("operator not confirmed")
    if not hb_hil_enabled: reasons.append("PX4 HIL heartbeat flag absent")
    for label,age,limit in (("heartbeat",heartbeat_age_s,2.0),("sensor",sensor_age_s,0.1),("actuator",actuator_age_s,0.1)):
        if not 0 <= age <= limit: reasons.append(f"{label} stale (age {age:.3f}s > {limit:.3f}s)")
    return HitlGate(not reasons,tuple(reasons))
