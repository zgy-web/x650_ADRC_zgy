#!/usr/bin/env python3
"""Software-only smoke test for the HIL transport path.

This does not replace validation with a physical PX4 flight controller. It
uses a tiny fake MAVLink autopilot endpoint to verify Gazebo sensor -> MAVLink
HIL_SENSOR/HIL_GPS and MAVLink HIL_ACTUATOR_CONTROLS -> Gazebo motor command.
"""

from __future__ import annotations

import json
import os
import subprocess
import threading
import time
from pathlib import Path

from pymavlink import mavutil

ROOT = Path(__file__).resolve().parents[2]
URL = "udpout:127.0.0.1:14560"


class FakePX4:
    def __init__(self):
        self.conn = mavutil.mavlink_connection(
            URL,
            source_system=1,
            source_component=1,
        )
        self.stop = False
        self.sensor = 0
        self.gps = 0
        self.thread = threading.Thread(target=self.run, daemon=True)

    def start(self):
        self.thread.start()

    def run(self):
        next_hb = next_act = 0.0
        while not self.stop:
            now = time.monotonic()
            if now >= next_hb:
                self.conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_QUADROTOR,
                    mavutil.mavlink.MAV_AUTOPILOT_PX4,
                    mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED,
                    0,
                    mavutil.mavlink.MAV_STATE_ACTIVE,
                )
                next_hb = now + 0.2
            if now >= next_act:
                controls = [0.22, 0.23, 0.24, 0.25] + [0.0] * 12
                self.conn.mav.hil_actuator_controls_send(
                    time.monotonic_ns() // 1000,
                    controls,
                    mavutil.mavlink.MAV_MODE_FLAG_HIL_ENABLED,
                    0,
                )
                next_act = now + 0.02

            msg = self.conn.recv_match(blocking=False)
            while msg is not None:
                if msg.get_type() == "HIL_SENSOR":
                    self.sensor += 1
                elif msg.get_type() == "HIL_GPS":
                    self.gps += 1
                msg = self.conn.recv_match(blocking=False)
            time.sleep(0.001)

    def close(self):
        self.stop = True
        self.thread.join(timeout=2)
        self.conn.close()


def run_cmd(*args):
    env = os.environ.copy()
    return subprocess.run(
        [str(ROOT / "scripts/amsim"), *args],
        cwd=ROOT,
        env=env,
        text=True,
        capture_output=True,
    )


def main():
    fake = FakePX4()
    fake.start()
    try:
        run_cmd("hil-stop")
        p = run_cmd(
            "hil-start",
            "--mavlink",
            "udpin:127.0.0.1:14560",
            "--model",
            "x500_underfold",
            "--wait",
            "50",
        )
        if p.returncode != 0:
            raise RuntimeError(
                "hil-start failed:\nSTDOUT:\n"
                + p.stdout
                + "\nSTDERR:\n"
                + p.stderr
            )

        deadline = time.monotonic() + 8
        bridge_status = None
        while time.monotonic() < deadline:
            folders = sorted(
                (ROOT / "runs").glob("hil_runtime_*"),
                key=lambda x: x.stat().st_mtime,
            )
            if folders:
                status_path = folders[-1] / "hil_bridge_status.json"
                if status_path.is_file():
                    bridge_status = json.loads(status_path.read_text())
            if (
                fake.sensor >= 100
                and fake.gps >= 3
                and bridge_status
                and bridge_status.get("actuator_count", 0) >= 20
            ):
                break
            time.sleep(0.25)

        result = {
            "hil_sensor_received_by_fake_px4": fake.sensor,
            "hil_gps_received_by_fake_px4": fake.gps,
            "bridge_status": bridge_status,
        }
        print(json.dumps(result, indent=2))

        if fake.sensor < 100:
            raise RuntimeError("HIL_SENSOR rate/path not validated")
        if fake.gps < 3:
            raise RuntimeError("HIL_GPS path not validated")
        if not bridge_status or bridge_status.get("actuator_count", 0) < 20:
            raise RuntimeError("HIL_ACTUATOR_CONTROLS path not validated")
    finally:
        run_cmd("hil-stop")
        fake.close()


if __name__ == "__main__":
    main()
