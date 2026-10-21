"""Readiness check for the active model, including vehicles without an arm.

READINESS CHECK ONLY:
- no arming
- no takeoff
- no arm motion
"""

import hashlib
import json
import math
import socket
import time

from ..paths import px4_dir, results_dir, model_dir
from .model_metadata import (
    REVIEWED_NAME, REVIEWED_SHA, initial_joint_values,
    required_telemetry, resolve_active_model,
)

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
PX4_POSITION = "/fmu/out/vehicle_local_position_v1"
PX4_STATUS = "/fmu/out/vehicle_status_v1"
PX4_LAND = "/fmu/out/vehicle_land_detected"
TRUTH = "/ground_truth/odometry"


class Telemetry(Node):
    def __init__(self, joint_names):
        super().__init__("amsim_readiness_check")

        self.joint_names = list(joint_names)
        self.data = {}
        self.received = {}

        for key, typ, topic in (
            ("position", VehicleLocalPosition, PX4_POSITION),
            ("status", VehicleStatus, PX4_STATUS),
            ("land", VehicleLandDetected, PX4_LAND),
            ("joints", JointState, JOINT_TOPIC),
            ("truth", Odometry, TRUTH),
        ):
            if key == "joints" and not self.joint_names:
                continue
            self.create_subscription(
                typ,
                topic,
                lambda msg, key=key: self.receive(key, msg),
                qos_profile_sensor_data,
            )

        self.joint_publishers = {
            name: self.create_publisher(Float64, f"/arm/{name}/cmd_vel", 10)
            for name in self.joint_names
        }

    def receive(self, key, msg):
        self.data[key] = msg
        self.received[key] = time.monotonic()

    def fresh(self):
        now = time.monotonic()

        return all(
            now - self.received.get(key, 0) < (
                3 if key in ("status", "land") else 2
            )
            for key in required_telemetry(self.joint_names)
        )

    def zero(self):
        msg = Float64()
        msg.data = 0.0
        for publisher in self.joint_publishers.values():
            publisher.publish(msg)


class Check:
    def __init__(self, joint_names):
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

        self.node = Telemetry(joint_names)

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
    model_metadata = resolve_active_model()
    if not model_metadata["flight_ready"]:
        raise RuntimeError("Model has no flight profile; PX4 readiness is unavailable")
    joint_names = model_metadata["joint_names"]
    check_label = "underfold" if model_metadata["model"] == REVIEWED_NAME else model_metadata["model"]

    result_dir = (
        results_dir()
        / (
            "check_" + check_label + "_"
            + time.strftime("%Y%m%d_%H%M%S")
        )
    )

    result_dir.mkdir(parents=True, exist_ok=False)

    expected_sha = model_metadata["model_sha256"]

    if model_metadata["model"] == REVIEWED_NAME:
        bundled_model = model_dir() / "model.sdf"
        if hashlib.sha256(bundled_model.read_bytes()).hexdigest() != REVIEWED_SHA:
            raise RuntimeError("Bundled reviewed model hash mismatch.")

    model = (
        px4_dir()
        / "Tools/simulation/gz/models" / model_metadata["model"] / "model.sdf"
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
        "model": model_metadata,
    }

    if actual_sha != expected_sha:
        raise RuntimeError(
            "Installed model hash does not match active runtime model."
        )

    rclpy.init()

    check = None

    try:
        check = Check(joint_names)

        check.wait(
            lambda: check.target is not None,
            45,
            "1/7 Waiting for PX4 MAVLink heartbeat",
        )

        check.wait(
            lambda: check.node.fresh(),
            45,
            "2/7 Waiting for model telemetry: " + ", ".join(required_telemetry(joint_names)),
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
            "4/7 Checking configured initial joint positions" if joint_names else "4/7 No controlled arm joints",
            flush=True,
        )

        values = initial_joint_values(
            joint_names, model_metadata["initial_joint_positions"], check.node.data.get("joints")
        )

        check.wait(
            lambda: all(pub.get_subscription_count() > 0 for pub in check.node.joint_publishers.values()),
            15,
            "5/7 Checking ROS-to-Gazebo command bridge" if joint_names else "5/7 No arm command bridge required",
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
