from __future__ import annotations

import argparse
import json
import math
import signal
import threading
import time
from dataclasses import dataclass
from pathlib import Path

from gz.transport13 import Node
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.fluid_pressure_pb2 import FluidPressure
from gz.msgs10.imu_pb2 import IMU
from gz.msgs10.magnetometer_pb2 import Magnetometer
from gz.msgs10.navsat_pb2 import NavSat
from pymavlink import mavutil


HIL_SENSOR_ALL_FIELDS = 0x1FFF


@dataclass
class SensorState:
    imu: IMU | None = None
    mag: Magnetometer | None = None
    pressure: FluidPressure | None = None
    navsat: NavSat | None = None


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


def motor_controls_to_velocity(controls, max_rot_velocity: float = 1000.0):
    values = list(controls)
    return [
        _clamp(float(values[i]) if i < len(values) else 0.0, 0.0, 1.0)
        * max_rot_velocity
        for i in range(4)
    ]


def imu_frd(msg: IMU):
    a = msg.linear_acceleration
    g = msg.angular_velocity
    return (
        float(a.x), -float(a.y), -float(a.z),
        float(g.x), -float(g.y), -float(g.z),
    )


def mag_gauss_frd(msg: Magnetometer):
    f = msg.field_tesla
    # Match the PX4 Gazebo bridge frame conversion; Tesla -> Gauss.
    return (
        -float(f.y) * 1e4,
        -float(f.x) * 1e4,
        float(f.z) * 1e4,
    )


class HILBridge:
    def __init__(
        self,
        mavlink_connection: str,
        model_name: str,
        baud: int = 921600,
        world: str = "default",
        max_rot_velocity: float = 1000.0,
        status_file: Path | None = None,
    ):
        self.connection_name = mavlink_connection
        self.model_name = model_name
        self.baud = baud
        self.world = world
        self.max_rot_velocity = max_rot_velocity
        self.status_file = status_file
        self.node = Node()
        self.lock = threading.Lock()
        self.state = SensorState()
        self.stop_requested = False
        self.sensor_count = 0
        self.gps_count = 0
        self.actuator_count = 0

        self.motor_pub = self.node.advertise(
            f"/{model_name}/command/motor_speed", Actuators
        )

        prefix = f"/world/{world}/model/{model_name}/link/base_link/sensor"
        self.node.subscribe(
            IMU, f"{prefix}/imu_sensor/imu", self._on_imu
        )
        self.node.subscribe(
            Magnetometer,
            f"{prefix}/magnetometer_sensor/magnetometer",
            self._on_mag,
        )
        self.node.subscribe(
            FluidPressure,
            f"{prefix}/air_pressure_sensor/air_pressure",
            self._on_pressure,
        )
        self.node.subscribe(
            NavSat,
            f"{prefix}/navsat_sensor/navsat",
            self._on_navsat,
        )

        self.master = mavutil.mavlink_connection(
            mavlink_connection,
            baud=baud,
            autoreconnect=True,
            source_system=255,
        )

    def _on_imu(self, msg):
        with self.lock:
            self.state.imu = msg

    def _on_mag(self, msg):
        with self.lock:
            self.state.mag = msg

    def _on_pressure(self, msg):
        with self.lock:
            self.state.pressure = msg

    def _on_navsat(self, msg):
        with self.lock:
            self.state.navsat = msg

    def _status(self, **extra):
        if not self.status_file:
            return
        payload = {
            "mavlink": self.connection_name,
            "model": self.model_name,
            "sensor_count": self.sensor_count,
            "gps_count": self.gps_count,
            "actuator_count": self.actuator_count,
            **extra,
        }
        self.status_file.parent.mkdir(parents=True, exist_ok=True)
        self.status_file.write_text(json.dumps(payload, indent=2))

    def wait_heartbeat(self, timeout_s: float = 20.0):
        hb = self.master.wait_heartbeat(timeout=timeout_s)
        if hb is None:
            raise RuntimeError(
                f"No MAVLink heartbeat on {self.connection_name} "
                f"within {timeout_s:.1f}s"
            )
        self._status(
            heartbeat=True,
            system_id=int(self.master.target_system),
            component_id=int(self.master.target_component),
            hil_enabled=bool(
                int(hb.base_mode)
                & mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED
            ),
        )
        return hb

    def _publish_motors(self, controls):
        msg = Actuators()
        msg.velocity.extend(
            motor_controls_to_velocity(
                controls, self.max_rot_velocity
            )
        )
        self.motor_pub.publish(msg)
        self.actuator_count += 1

    def _send_sensor(self, now_us: int):
        with self.lock:
            imu = self.state.imu
            mag = self.state.mag
            pressure = self.state.pressure
            navsat = self.state.navsat
        if imu is None or mag is None or pressure is None:
            return False

        xacc, yacc, zacc, xgyro, ygyro, zgyro = imu_frd(imu)
        xmag, ymag, zmag = mag_gauss_frd(mag)
        abs_pressure = float(pressure.pressure) / 100.0
        pressure_alt = float(navsat.altitude) if navsat is not None else 0.0

        self.master.mav.hil_sensor_send(
            now_us,
            xacc, yacc, zacc,
            xgyro, ygyro, zgyro,
            xmag, ymag, zmag,
            abs_pressure,
            0.0,
            pressure_alt,
            20.0,
            HIL_SENSOR_ALL_FIELDS,
        )
        self.sensor_count += 1
        return True

    def _send_gps(self, now_us: int):
        with self.lock:
            nav = self.state.navsat
        if nav is None:
            return False

        vn = float(nav.velocity_north)
        ve = float(nav.velocity_east)
        vd = -float(nav.velocity_up)
        speed = math.sqrt(vn * vn + ve * ve + vd * vd)
        cog = math.degrees(math.atan2(ve, vn)) % 360.0

        self.master.mav.hil_gps_send(
            now_us,
            3,
            int(float(nav.latitude_deg) * 1e7),
            int(float(nav.longitude_deg) * 1e7),
            int(float(nav.altitude) * 1000),
            50,
            80,
            int(speed * 100),
            int(vn * 100),
            int(ve * 100),
            int(vd * 100),
            int(cog * 100),
            12,
        )
        self.gps_count += 1
        return True

    def run(self):
        hb = self.wait_heartbeat()
        if not (
            int(hb.base_mode)
            & mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED
        ):
            raise RuntimeError(
                "PX4 heartbeat received, but HIL flag is not enabled. "
                "Set SYS_HITL=1 and reboot the flight controller."
            )

        sensor_period = 1.0 / 250.0
        gps_period = 1.0 / 10.0
        heartbeat_period = 1.0
        next_sensor = next_gps = next_hb = time.monotonic()

        while not self.stop_requested:
            now = time.monotonic()
            now_us = time.monotonic_ns() // 1000

            if now >= next_sensor:
                self._send_sensor(now_us)
                next_sensor += sensor_period

            if now >= next_gps:
                self._send_gps(now_us)
                next_gps += gps_period

            if now >= next_hb:
                self.master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                next_hb += heartbeat_period

            msg = self.master.recv_match(blocking=False)
            while msg is not None:
                if msg.get_type() == "HIL_ACTUATOR_CONTROLS":
                    self._publish_motors(msg.controls)
                msg = self.master.recv_match(blocking=False)

            if self.sensor_count and self.sensor_count % 250 == 0:
                self._status(heartbeat=True, running=True)
            time.sleep(0.001)

    def close(self):
        self.stop_requested = True
        self.master.close()


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--mavlink", required=True)
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--model", default="x500_underfold")
    p.add_argument("--world", default="default")
    p.add_argument("--max-rot-velocity", type=float, default=1000.0)
    p.add_argument("--status-file")
    args = p.parse_args()

    bridge = HILBridge(
        args.mavlink,
        args.model,
        baud=args.baud,
        world=args.world,
        max_rot_velocity=args.max_rot_velocity,
        status_file=Path(args.status_file) if args.status_file else None,
    )

    def stop_handler(_sig, _frame):
        bridge.stop_requested = True

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)

    try:
        bridge.run()
    finally:
        bridge.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
