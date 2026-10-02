#!/usr/bin/env python3
import json
import math

from amsim import ScenarioContext


def main():
    with ScenarioContext() as ctx:
        ctx.vehicle.wait_ready()
        before = ctx.observe(ground_truth=True)

        if "arm_joint_3" not in ctx.arm.joints:
            raise RuntimeError(f"arm_joint_3 missing: {ctx.arm.joints}")

        q0 = before.arm["arm_joint_3"].position_rad
        ctx.arm.move_position(
            "arm_joint_3",
            10.0,
            unit="deg",
            max_speed=8.0,
            speed_unit="deg/s",
            tolerance=0.2,
            timeout_s=10.0,
        )
        after = ctx.observe(ground_truth=True)
        q1 = after.arm["arm_joint_3"].position_rad

        print(json.dumps({
            "joints": ctx.arm.joints,
            "q3_before_deg": math.degrees(q0),
            "q3_after_deg": math.degrees(q1),
            "vehicle_estimate": {
                "x": after.vehicle.x,
                "y": after.vehicle.y,
                "z": after.vehicle.z,
                "landed": after.vehicle.landed,
            },
            "ground_truth_present": after.ground_truth is not None,
        }, indent=2))

        if abs(math.degrees(q1) - 10.0) > 0.5:
            raise RuntimeError("Arm API position move did not reach requested target")


if __name__ == "__main__":
    main()
