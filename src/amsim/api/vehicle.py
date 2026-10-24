from __future__ import annotations

import math
import time

from pymavlink import mavutil
from px4_msgs.msg import VehicleStatus

from .node import ApiNode


class VehicleAPI:
    """Stable high-level vehicle API backed by PX4 MAVLink + ROS telemetry."""

    def __init__(self, node: ApiNode):
        self.node = node
        self.link = mavutil.mavlink_connection(
            "udpin:127.0.0.1:14550",
            source_system=255,
            source_component=190,
        )
        self.target = None
        self.ack = {}
        self.global_position = None
        self.global_time = 0.0
        self.heartbeat_time = 0.0

    def _pump(self, ros_timeout_s=0.02):
        self.node.spin_once(timeout_sec=ros_timeout_s)
        now = time.monotonic()

        for _ in range(500):
            msg = self.link.recv_match(blocking=False)
            if msg is None:
                break

            if (
                msg.get_type() == "HEARTBEAT"
                and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4
            ):
                identity = (msg.get_srcSystem(), msg.get_srcComponent())
                if self.target is not None and identity != self.target:
                    raise RuntimeError("Multiple PX4 systems detected")
                self.target = identity

            if (
                self.target
                and (msg.get_srcSystem(), msg.get_srcComponent()) == self.target
            ):
                if msg.get_type() == "COMMAND_ACK":
                    self.ack[msg.command] = (now, msg.result)
                elif msg.get_type() == "GLOBAL_POSITION_INT":
                    self.global_position = msg
                    self.global_time = now

        if self.target and now - self.heartbeat_time >= 0.5:
            self.link.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0,
                0,
                mavutil.mavlink.MAV_STATE_ACTIVE,
            )
            self.heartbeat_time = now

    def wait(self, predicate, timeout_s: float, description: str):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self._pump()
            if predicate():
                return
        raise RuntimeError("Timeout: " + description)

    def wait_ready(self, timeout_s: float = 60.0):
        self.wait(lambda: self.target is not None, timeout_s, "PX4 heartbeat")
        self.node.wait_for_data(timeout_s=timeout_s, require_truth=True)
        self.wait(
            lambda: self.node.data["status"].pre_flight_checks_pass,
            timeout_s,
            "PX4 preflight checks",
        )

    def _send_command(self, command: int, params: list[float]):
        if self.target is None:
            raise RuntimeError("PX4 target not discovered")
        self.link.mav.command_long_send(
            *self.target,
            command,
            0,
            *((params + [0.0] * 7)[:7]),
        )

    def _command(self, command: int, params: list[float]):
        if self.target is None:
            raise RuntimeError("PX4 target not discovered")

        for _attempt in range(3):
            self.ack.pop(command, None)
            self._send_command(command, params)

            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                self._pump()
                if command not in self.ack:
                    continue

                result = self.ack[command][1]
                if result in (
                    mavutil.mavlink.MAV_RESULT_ACCEPTED,
                    mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                ):
                    return
                raise RuntimeError(
                    f"PX4 command {command} rejected: result={result}"
                )

        raise RuntimeError(
            f"Timeout: command {command} acknowledgement after 3 attempts"
        )

    def arm(self):
        self._command(
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            [1, 0],
        )
        self.wait(
            lambda: self.node.data["status"].arming_state
            == VehicleStatus.ARMING_STATE_ARMED,
            8.0,
            "armed state",
        )

    def takeoff(self, height_m: float = 2.5, stable_s: float = 3.0):
        if height_m <= 0:
            raise ValueError("height_m must be > 0")

        self.wait_ready()

        self._command(
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            [
                mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,
                200000,
            ],
        )
        self.wait(
            lambda: self.global_position is not None
            and time.monotonic() - self.global_time < 2,
            15.0,
            "global altitude reference",
        )

        p = self.node.data["position"]
        target_z = float(p.z) - height_m
        altitude_amsl = self.global_position.alt / 1000 + height_m

        self.arm()
        self._command(
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            [
                0,
                0,
                0,
                math.nan,
                math.nan,
                math.nan,
                altitude_amsl,
            ],
        )

        stable_since = None
        deadline = time.monotonic() + 180.0
        while time.monotonic() < deadline:
            self._pump()
            p = self.node.data["position"]
            speed = math.sqrt(p.vx * p.vx + p.vy * p.vy + p.vz * p.vz)
            ok = (
                abs(p.z - target_z) < 0.30
                and speed < 0.25
                and not self.node.data["land"].landed
            )
            sim_t = self.node.sim_time_s()
            if ok:
                if stable_since is None:
                    stable_since = sim_t
                elif sim_t - stable_since >= stable_s:
                    return target_z
            else:
                stable_since = None

        raise RuntimeError("Takeoff did not reach stable hover")

    def land(self):
        """Land and confirm landed/disarmed state.

        PX4 state confirmation is authoritative. NAV_LAND COMMAND_ACK can be
        lost on a busy UDP link, so a missing ACK does not fail the operation
        if the vehicle demonstrably enters the landed state.
        """
        self.node.zero_all()

        command = mavutil.mavlink.MAV_CMD_NAV_LAND
        self.ack.pop(command, None)

        for _attempt in range(3):
            self._send_command(
                command,
                [0, 0, 0, math.nan, math.nan, math.nan, math.nan],
            )

            short_deadline = time.monotonic() + 4.0
            while time.monotonic() < short_deadline:
                self._pump()

                if self.node.data["land"].landed:
                    break

                if command in self.ack:
                    result = self.ack[command][1]
                    if result not in (
                        mavutil.mavlink.MAV_RESULT_ACCEPTED,
                        mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                    ):
                        raise RuntimeError(
                            f"PX4 NAV_LAND rejected: result={result}"
                        )
                    break

            if self.node.data["land"].landed or command in self.ack:
                break

        self.wait(
            lambda: self.node.data["land"].landed,
            180.0,
            "confirmed landing",
        )
        self.wait(
            lambda: self.node.data["status"].arming_state
            != VehicleStatus.ARMING_STATE_ARMED,
            30.0,
            "automatic disarm",
        )

    def close(self):
        self.link.close()
