from __future__ import annotations

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleLandDetected, VehicleLocalPosition, VehicleStatus
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from .observation import GroundTruth, JointStateValue, Observation, VehicleEstimate
from ..runtime.model_metadata import required_telemetry, validate_joint_names


class ApiNode(Node):
    def __init__(self, joint_names: list[str]):
        super().__init__("amsim_public_api")
        self.joint_names = validate_joint_names(joint_names)
        self.data = {}
        self.received = {}

        for key, typ, topic in (
            ("position", VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1"),
            ("status", VehicleStatus, "/fmu/out/vehicle_status_v1"),
            ("land", VehicleLandDetected, "/fmu/out/vehicle_land_detected"),
            ("joints", JointState, "/arm/joint_states"),
            ("truth", Odometry, "/ground_truth/odometry"),
        ):
            if key == "joints" and not self.joint_names:
                continue
            self.create_subscription(
                typ,
                topic,
                lambda msg, key=key: self._receive(key, msg),
                qos_profile_sensor_data,
            )

        self.joint_publishers = {
            name: self.create_publisher(
                Float64,
                f"/arm/{name}/cmd_vel",
                10,
            )
            for name in self.joint_names
        }

        self._executor = SingleThreadedExecutor()
        self._executor.add_node(self)

    def _receive(self, key, msg):
        self.data[key] = msg
        self.received[key] = time.monotonic()

    def spin_once(self, timeout_sec: float = 0.02):
        self._executor.spin_once(timeout_sec=timeout_sec)

    def wait_for_data(self, timeout_s: float = 45.0, require_truth: bool = True):
        required = required_telemetry(self.joint_names, require_truth)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin_once()
            if all(k in self.data for k in required):
                return
        raise RuntimeError("Timed out waiting for AMSIM API telemetry")

    def publish_velocity(self, joint: str, velocity_rad_s: float):
        if joint not in self.joint_publishers:
            raise KeyError(f"Unknown joint: {joint}")
        if not math.isfinite(velocity_rad_s):
            raise ValueError("joint velocity must be finite")
        msg = Float64()
        msg.data = float(velocity_rad_s)
        self.joint_publishers[joint].publish(msg)

    def zero_all(self):
        for joint in self.joint_names:
            self.publish_velocity(joint, 0.0)

    def joint_values(self) -> dict[str, JointStateValue]:
        if not self.joint_names:
            return {}
        msg = self.data.get("joints")
        if msg is None:
            raise RuntimeError("No joint state available")
        out = {}
        for joint in self.joint_names:
            if joint not in msg.name:
                raise RuntimeError(f"Joint missing from feedback: {joint}")
            i = msg.name.index(joint)
            if i >= len(msg.position) or i >= len(msg.velocity):
                raise RuntimeError(f"Incomplete JointState for {joint}")
            effort = msg.effort[i] if i < len(msg.effort) else None
            out[joint] = JointStateValue(
                position_rad=float(msg.position[i]),
                velocity_rad_s=float(msg.velocity[i]),
                effort=float(effort) if effort is not None else None,
            )
        return out

    def sim_time_s(self) -> float:
        truth = self.data.get("truth")
        if truth is None:
            raise RuntimeError("Ground-truth clock not available")
        return truth.header.stamp.sec + truth.header.stamp.nanosec * 1e-9

    def observation(self, include_ground_truth: bool = False) -> Observation:
        p = self.data.get("position")
        st = self.data.get("status")
        land = self.data.get("land")
        if p is None or st is None or land is None:
            raise RuntimeError("Vehicle telemetry not ready")

        vehicle = VehicleEstimate(
            x=float(p.x),
            y=float(p.y),
            z=float(p.z),
            vx=float(p.vx),
            vy=float(p.vy),
            vz=float(p.vz),
            armed=(st.arming_state == VehicleStatus.ARMING_STATE_ARMED),
            landed=bool(land.landed),
            failsafe=bool(st.failsafe),
        )

        truth_obj = None
        if include_ground_truth:
            truth = self.data.get("truth")
            if truth is None:
                raise RuntimeError("Ground truth requested but unavailable")
            pose = truth.pose.pose
            truth_obj = GroundTruth(
                x=float(pose.position.x),
                y=float(pose.position.y),
                z=float(pose.position.z),
                qx=float(pose.orientation.x),
                qy=float(pose.orientation.y),
                qz=float(pose.orientation.z),
                qw=float(pose.orientation.w),
            )

        return Observation(
            sim_time_s=self.sim_time_s(),
            vehicle=vehicle,
            arm=self.joint_values(),
            ground_truth=truth_obj,
        )

    def close(self):
        self.zero_all()
        self._executor.shutdown()
        self.destroy_node()
