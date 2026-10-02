"""V3 underfold integration check.

READINESS CHECK ONLY:
- no arming
- no takeoff
- no arm motion
"""

import hashlib
import json
import math
import os
from pathlib import Path
import socket
import time

from ..paths import PROJECT_ROOT, px4_dir, results_dir, model_dir

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from nav_msgs.msg import Odometry
from px4_msgs.msg import (
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)
from pymavlink import mavutil
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64


JOINT_TOPIC = "/arm/joint_states"
CMD1_TOPIC = "/arm/arm_joint_1/cmd_vel"
CMD2_TOPIC = "/arm/arm_joint_2/cmd_vel"

PX4_POSITION = "/fmu/out/vehicle_local_position_v1"
PX4_STATUS = "/fmu/out/vehicle_status_v1"
PX4_LAND = "/fmu/out/vehicle_land_detected"
TRUTH = "/ground_truth/odometry"


class Telemetry(Node):
    def __init__(self):
        super().__init__("underfold_v3_check")

        self.data = {}
        self.received = {}

        for key, typ, topic in (
            ("position", VehicleLocalPosition, PX4_POSITION),
            ("status", VehicleStatus, PX4_STATUS),
            ("land", VehicleLandDetected, PX4_LAND),
            ("joints", JointState, JOINT_TOPIC),
            ("truth", Odometry, TRUTH),
        ):
            self.create_subscription(
                typ,
                topic,
                lambda msg, key=key: self.receive(key, msg),
                qos_profile_sensor_data,
            )

        self.pub1 = self.create_publisher(
            Float64,
            CMD1_TOPIC,
            10,
        )

        self.pub2 = self.create_publisher(
            Float64,
            CMD2_TOPIC,
            10,
        )

    def receive(self, key, msg):
        self.data[key] = msg
        self.received[key] = time.monotonic()

    def fresh(self):
        now = time.monotonic()

        return all(
            now - self.received.get(key, 0) < (
                3 if key in ("status", "land") else 2
            )
            for key in (
                "position",
                "status",
                "land",
                "joints",
                "truth",
            )
        )

    def zero(self):
        msg = Float64()
        msg.data = 0.0
        self.pub1.publish(msg)
        self.pub2.publish(msg)


class Check:
    def __init__(self):
        # Same protection as v2: do not share QGC's UDP port silently.
        with socket.socket(
            socket.AF_INET,
            socket.SOCK_DGRAM
        ) as probe:
            probe.bind(("0.0.0.0", 14550))

        self.link = mavutil.mavlink_connection(
            "udpin:127.0.0.1:14550",
            source_system=255,
            source_component=190,
        )

        self.target = None
        self.last_heartbeat = 0.0
        self.gcs_heartbeat_time = 0.0

        self.node = Telemetry()

        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)

    def pump(self):
        self.executor.spin_once(timeout_sec=0.02)

        now = time.monotonic()

        for _ in range(100):
            msg = self.link.recv_match(blocking=False)

            if msg is None:
                break

            if (
                msg.get_type() == "HEARTBEAT"
                and msg.autopilot
                == mavutil.mavlink.MAV_AUTOPILOT_PX4
            ):
                identity = (
                    msg.get_srcSystem(),
                    msg.get_srcComponent(),
                )

                if (
                    self.target is not None
                    and identity != self.target
                ):
                    raise RuntimeError(
                        "Multiple PX4 systems detected."
                    )

                self.target = identity
                self.last_heartbeat = now

        if (
            self.target
            and now - self.gcs_heartbeat_time >= 0.5
        ):
            self.link.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )

            self.gcs_heartbeat_time = now

    def wait(self, predicate, timeout, description):
        print(description, flush=True)
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            self.pump()

            if predicate():
                return

        raise RuntimeError("Timeout: " + description)

    def close(self):
        self.node.zero()
        self.executor.shutdown()
        self.node.destroy_node()
        self.link.close()


def main():
    here = PROJECT_ROOT

    result_dir = (
        results_dir()
        / (
            "check_underfold_"
            + time.strftime("%Y%m%d_%H%M%S")
        )
    )

    result_dir.mkdir(parents=True, exist_ok=False)

    expected_sha = "8c86eb60a3e8ed494ec54d08d2ad6a04eb44a5f5e3f8e4ec5bc2dcc5ccc226c8"

    bundled_model = model_dir() / "model.sdf"
    if hashlib.sha256(bundled_model.read_bytes()).hexdigest() != expected_sha:
        raise RuntimeError("Bundled reviewed model hash mismatch.")

    model = (
        px4_dir()
        / "Tools/simulation/gz/models/"
          "x500_underfold/model.sdf"
    )

    actual_sha = hashlib.sha256(
        model.read_bytes()
    ).hexdigest()

    result = {
        "success": False,
        "check_only": True,
        "arming_commands_sent": False,
        "flight_commands_sent": False,
        "arm_motion_commands_sent": False,
        "model_sha256": actual_sha,
    }

    if actual_sha != expected_sha:
        raise RuntimeError(
            "Installed model hash does not match reviewed model."
        )

    rclpy.init()

    check = None

    try:
        check = Check()

        check.wait(
            lambda: check.target is not None,
            45,
            "1/7 Waiting for PX4 MAVLink heartbeat",
        )

        check.wait(
            lambda: check.node.fresh(),
            45,
            "2/7 Waiting for PX4, joint and truth telemetry",
        )

        check.wait(
            lambda:
                check.node.data["status"].pre_flight_checks_pass,
            60,
            "3/7 Waiting for normal PX4 preflight checks",
        )

        status = check.node.data["status"]
        land = check.node.data["land"]

        if (
            status.arming_state
            == VehicleStatus.ARMING_STATE_ARMED
        ):
            raise RuntimeError(
                "Aircraft unexpectedly armed."
            )

        if not land.landed:
            raise RuntimeError(
                "Aircraft is not confirmed landed."
            )

        print(
            "4/7 Checking reviewed underfold joint zero",
            flush=True,
        )

        joints = check.node.data["joints"]

        values = {}

        for name in ("arm_joint_1", "arm_joint_2"):
            if name not in joints.name:
                raise RuntimeError(
                    f"Missing joint feedback: {name}"
                )

            i = joints.name.index(name)

            if (
                i >= len(joints.position)
                or i >= len(joints.velocity)
            ):
                raise RuntimeError(
                    f"Incomplete JointState arrays: {name}"
                )

            q = joints.position[i]
            v = joints.velocity[i]

            if not (
                math.isfinite(q)
                and math.isfinite(v)
            ):
                raise RuntimeError(
                    f"Nonfinite joint state: {name}"
                )

            if abs(q) > math.radians(0.4):
                raise RuntimeError(
                    f"{name} is not near new q=0: "
                    f"{math.degrees(q):.4f} deg"
                )

            if abs(v) > 0.01:
                raise RuntimeError(
                    f"{name} moving too fast: "
                    f"{v:.6f} rad/s"
                )

            values[name] = {
                "q_rad": q,
                "q_deg": math.degrees(q),
                "velocity_rad_s": v,
            }

        check.wait(
            lambda:
                check.node.pub1.get_subscription_count() > 0
                and
                check.node.pub2.get_subscription_count() > 0,
            15,
            "5/7 Checking ROS-to-Gazebo command bridge",
        )

        # Zero only. No arm motion command is issued.
        for _ in range(10):
            check.node.zero()
            check.pump()
            time.sleep(0.05)

        check.wait(
            lambda: check.node.fresh(),
            5,
            "6/7 Rechecking telemetry freshness",
        )

        p = check.node.data["position"]
        truth = check.node.data["truth"]

        if not all(
            math.isfinite(x)
            for x in (
                p.x,
                p.y,
                p.z,
                p.vx,
                p.vy,
                p.vz,
                truth.pose.pose.position.x,
                truth.pose.pose.position.y,
                truth.pose.pose.position.z,
            )
        ):
            raise RuntimeError(
                "Nonfinite PX4 or ground-truth state."
            )

        if (
            time.monotonic()
            - check.last_heartbeat
            > 3
        ):
            raise RuntimeError(
                "PX4 MAVLink heartbeat stale."
            )

        print(
            "7/7 CHECK PASSED: "
            "no arming or flight commands sent.",
            flush=True,
        )

        for name, item in values.items():
            print(
                f"{name}: "
                f"q={item['q_rad']:+.7f} rad "
                f"({item['q_deg']:+.4f} deg), "
                f"dq={item['velocity_rad_s']:+.7f} rad/s",
                flush=True,
            )

        result["joints"] = values
        result["success"] = True

    except Exception as exc:
        result["error"] = str(exc)
        print(
            "CHECK FAILED: " + str(exc),
            flush=True,
        )

    finally:
        if check is not None:
            try:
                check.close()
            except Exception as exc:
                result["cleanup_error"] = str(exc)
                result["success"] = False

        if rclpy.ok():
            rclpy.shutdown()

        (result_dir / "result.json").write_text(
            json.dumps(result, indent=2)
        )

        print(
            f"Results: {result_dir}",
            flush=True,
        )

    raise SystemExit(
        0 if result["success"] else 1
    )


if __name__ == "__main__":
    main()
