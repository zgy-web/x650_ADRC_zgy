from __future__ import annotations

import math
import time

from .node import ApiNode


class ArmAPI:
    def __init__(self, node: ApiNode):
        self.node = node

    @property
    def joints(self) -> list[str]:
        return list(self.node.joint_names)

    def state(self):
        return self.node.joint_values()

    def set_velocity(self, joint: str, value: float, unit: str = "rad/s"):
        if unit == "deg/s":
            value = math.radians(value)
        elif unit != "rad/s":
            raise ValueError("unit must be 'rad/s' or 'deg/s'")
        self.node.publish_velocity(joint, float(value))

    def stop(self, joint: str | None = None):
        if joint is None:
            self.node.zero_all()
        else:
            self.node.publish_velocity(joint, 0.0)

    def move_position(
        self,
        joint: str,
        target: float,
        *,
        unit: str = "deg",
        max_speed: float = 0.5,
        speed_unit: str = "deg/s",
        tolerance: float = 0.05,
        timeout_s: float = 60.0,
        kp: float = 1.5,
    ) -> float:
        if unit == "deg":
            target_rad = math.radians(target)
            tol_rad = math.radians(tolerance)
        elif unit == "rad":
            target_rad = float(target)
            tol_rad = float(tolerance)
        else:
            raise ValueError("unit must be 'deg' or 'rad'")

        if speed_unit == "deg/s":
            max_speed_rad_s = math.radians(max_speed)
        elif speed_unit == "rad/s":
            max_speed_rad_s = float(max_speed)
        else:
            raise ValueError("speed_unit must be 'deg/s' or 'rad/s'")

        if max_speed_rad_s <= 0 or tol_rad <= 0:
            raise ValueError("max_speed and tolerance must be > 0")

        deadline = time.monotonic() + timeout_s
        try:
            while time.monotonic() < deadline:
                self.node.spin_once()
                current = self.node.joint_values()[joint].position_rad
                error = target_rad - current

                if abs(error) <= tol_rad:
                    self.node.publish_velocity(joint, 0.0)
                    return current

                cmd = max(-max_speed_rad_s, min(max_speed_rad_s, kp * error))
                self.node.publish_velocity(joint, cmd)

            raise RuntimeError(
                f"Timeout moving {joint}: target={target_rad:.6f} rad"
            )
        finally:
            self.node.publish_velocity(joint, 0.0)
