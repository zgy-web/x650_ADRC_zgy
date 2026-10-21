from __future__ import annotations

import argparse
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from pathlib import Path

from ..modeling.manager import IMPORTED_SOURCES, env_for, install, resolve
from ..paths import PROJECT_ROOT, px4_dir, results_dir


def _make_world(model_name: str, folder: Path) -> Path:
    src = px4_dir() / "Tools/simulation/gz/worlds/default.sdf"
    text = src.read_text(encoding="utf-8-sig")
    include = f"""
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <plugin filename="gz-sim-air-pressure-system" name="gz::sim::systems::AirPressure"/>
    <plugin filename="gz-sim-magnetometer-system" name="gz::sim::systems::Magnetometer"/>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"/>
    <include>
      <uri>model://{model_name}</uri>
      <name>{model_name}</name>
      <pose>0 0 0.18 0 0 0</pose>
    </include>
"""
    if "</world>" not in text:
        raise RuntimeError(f"invalid Gazebo world: {src}")
    dst = folder / "hil_world.sdf"
    dst.write_text(text.replace("</world>", include + "  </world>"), encoding="utf-8")
    return dst


def _topics(env: dict) -> list[str]:
    p = subprocess.run(
        ["gz", "topic", "-l"],
        env=env,
        text=True,
        capture_output=True,
        timeout=8,
    )
    if p.returncode:
        return []
    return [x.strip() for x in p.stdout.splitlines() if x.strip()]


def _wait_topics(env: dict, model_name: str, joints: list[str], timeout_s: float = 45.0):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        topics = _topics(env)
        imu = f"/world/default/model/{model_name}/link/base_link/sensor/imu_sensor/imu"
        mag = f"/world/default/model/{model_name}/link/base_link/sensor/magnetometer_sensor/magnetometer"
        nav = f"/world/default/model/{model_name}/link/base_link/sensor/navsat_sensor/navsat"
        pressure = f"/world/default/model/{model_name}/link/base_link/sensor/air_pressure_sensor/air_pressure"
        motor = f"/{model_name}/command/motor_speed"
        commands = {
            j: f"/model/{model_name}/joint/{j}/cmd_vel"
            for j in joints
        }
        states = [
            t for t in topics
            if t.endswith(f"/model/{model_name}/joint_state")
        ]
        needed = [imu, mag, nav, pressure, motor, *commands.values()]
        if all(t in topics for t in needed) and len(states) == 1:
            return {
                "imu": imu,
                "mag": mag,
                "navsat": nav,
                "pressure": pressure,
                "motor": motor,
                "joint_state_gz": states[0],
                "joint_commands_gz": commands,
            }
        time.sleep(0.5)
    raise RuntimeError(
        "Gazebo HIL topics did not become ready for "
        f"{model_name}; relevant topics: "
        + "\n".join(t for t in _topics(env) if model_name in t)
    )


def main() -> int:
    p = argparse.ArgumentParser()
    p.add_argument("--mavlink", required=True)
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--model", default="x500_underfold")
    p.add_argument("--xrce-serial")
    p.add_argument("--xrce-baud", type=int, default=921600)
    args = p.parse_args()

    signal.signal(signal.SIGINT, signal.default_int_handler)

    spec = resolve(args.model)
    if spec.source in IMPORTED_SOURCES:
        raise RuntimeError("Imported URDF models are supported by preview/SITL only; HIL binding has not been validated")
    install(spec, require_sitl_airframe=False)
    model_name = spec.model_name
    joints = list(spec.joints)

    stamp = time.strftime("%Y%m%d_%H%M%S")
    folder = results_dir() / f"hil_runtime_{stamp}"
    folder.mkdir(parents=True, exist_ok=False)
    world = _make_world(model_name, folder)

    env = os.environ.copy()
    env.update(env_for(spec, require_sitl_airframe=False))
    models = px4_dir() / "Tools/simulation/gz/models"
    env["GZ_SIM_RESOURCE_PATH"] = (
        str(models)
        + ":"
        + str(px4_dir() / "Tools/simulation/gz/worlds")
        + ":"
        + env.get("GZ_SIM_RESOURCE_PATH", "")
    )
    env["HEADLESS"] = "1"

    children: list[tuple[str, subprocess.Popen]] = []
    logs = []

    def launch(name: str, cmd: list[str]):
        f = open(folder / f"{name}.log", "w")
        logs.append(f)
        proc = subprocess.Popen(
            cmd,
            cwd=PROJECT_ROOT,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=f,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        children.append((name, proc))
        return proc

    def ensure_alive():
        for name, proc in children:
            if proc.poll() is not None:
                raise RuntimeError(
                    f"{name} exited rc={proc.returncode}; see {folder / (name + '.log')}"
                )

    try:
        launch(
            "gazebo",
            ["gz", "sim", "--force-version", "8", "-r", "-s", str(world)],
        )

        deadline = time.monotonic() + 45
        discovered = None
        while time.monotonic() < deadline:
            ensure_alive()
            try:
                discovered = _wait_topics(env, model_name, joints, timeout_s=1.0)
                break
            except RuntimeError:
                pass
        if discovered is None:
            raise RuntimeError("Gazebo HIL world did not become ready")

        bridge_args = [
            sys.executable,
            "-m",
            "amsim.hil.bridge",
            "--mavlink",
            args.mavlink,
            "--baud",
            str(args.baud),
            "--model",
            model_name,
            "--status-file",
            str(folder / "hil_bridge_status.json"),
        ]
        launch("hil_bridge", bridge_args)

        if args.xrce_serial:
            launch(
                "xrce_agent",
                [
                    "MicroXRCEAgent",
                    "serial",
                    "--dev",
                    args.xrce_serial,
                    "-b",
                    str(args.xrce_baud),
                ],
            )

        ros_bridge = [
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",
        ]
        for j in joints:
            ros_bridge.append(
                discovered["joint_commands_gz"][j]
                + "@std_msgs/msg/Float64]gz.msgs.Double"
            )
        ros_bridge.extend([
            discovered["joint_state_gz"]
            + "@sensor_msgs/msg/JointState[gz.msgs.Model",
            "/ground_truth/odometry"
            + "@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "--ros-args",
        ])
        for j in joints:
            ros_bridge.extend([
                "-r",
                discovered["joint_commands_gz"][j]
                + f":=/arm/{j}/cmd_vel",
            ])
        ros_bridge.extend([
            "-r",
            discovered["joint_state_gz"] + ":=/arm/joint_states",
        ])
        launch("ros_bridge", ros_bridge)

        runtime = {
            "backend": "px4_hil",
            "model": model_name,
            "model_sha256": spec.sha256,
            "joint_names": joints,
            "mavlink": args.mavlink,
            "baud": args.baud,
            "xrce_serial": args.xrce_serial,
            "xrce_baud": args.xrce_baud if args.xrce_serial else None,
            "world": str(world),
            **discovered,
        }
        (folder / "runtime.json").write_text(json.dumps(runtime, indent=2))
        print(json.dumps({"ready": True, "runtime": str(folder / "runtime.json")}, indent=2), flush=True)

        while True:
            ensure_alive()
            time.sleep(1)

    except KeyboardInterrupt:
        pass
    finally:
        for _, proc in reversed(children):
            try:
                os.killpg(proc.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
        time.sleep(1.5)
        for _, proc in reversed(children):
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        for f in logs:
            f.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
