"""V3 underfold SITL supervisor: startup + dynamic Gazebo topic discovery.

No arming or flight commands are sent here.
"""

import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import socket
import subprocess
import time

from ..paths import PROJECT_ROOT, px4_dir, results_dir, model_dir


MODEL = "x500_underfold"


def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    # auto_sim.py may be launched as a background job by bash.
    # Non-interactive bash can start background jobs with SIGINT ignored.
    # Explicitly restore Python's SIGINT handler so the supervisor can
    # always perform its own orderly child-process cleanup.
    signal.signal(signal.SIGINT, signal.default_int_handler)

    here = PROJECT_ROOT
    px4 = px4_dir()

    models = px4 / "Tools/simulation/gz/models"
    installed_model_dir = models / MODEL
    model_sdf = installed_model_dir / "model.sdf"
    source_model_sdf = model_dir() / "model.sdf"
    binary = px4 / "build/px4_sitl_default/bin/px4"

    expected_sha = "8c86eb60a3e8ed494ec54d08d2ad6a04eb44a5f5e3f8e4ec5bc2dcc5ccc226c8"

    for path in (binary, model_sdf, source_model_sdf):
        if not path.is_file():
            raise RuntimeError(f"Missing: {path}")

    source_sha = sha256(source_model_sdf)
    if source_sha != expected_sha:
        raise RuntimeError(f"Bundled reviewed model hash mismatch: {source_sha}")

    actual_sha = sha256(model_sdf)
    if actual_sha != expected_sha:
        raise RuntimeError(
            "Installed x500_underfold model hash differs from reviewed model.\n"
            f"expected={expected_sha}\n"
            f"actual={actual_sha}"
        )

    for cmd in ("MicroXRCEAgent", "ros2", "gz"):
        if not shutil.which(cmd):
            raise RuntimeError(f"Not found: {cmd}")

    # Same safety checks used by reviewed v2 supervisor.
    for port in (8888, 18570):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.bind(("0.0.0.0", port))

    for process in ("px4", "MicroXRCEAgent"):
        if subprocess.run(
            ["pgrep", "-x", process],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        ).returncode == 0:
            raise RuntimeError(
                f"{process} already running. Close old session first."
            )

    existing = subprocess.run(
        ["pgrep", "-af", "gz sim|gz-sim"],
        capture_output=True,
        text=True,
    )

    if existing.returncode == 0:
        raise RuntimeError(
            "Old Gazebo processes exist; close them before starting:\n"
            + existing.stdout
        )

    env = os.environ.copy()

    env.update(
        PX4_SYS_AUTOSTART="4001",
        PX4_SIM_MODEL=MODEL,
    )

    env["GZ_SIM_RESOURCE_PATH"] = (
        str(models)
        + ":"
        + str(px4 / "Tools/simulation/gz/worlds")
        + ":"
        + env.get("GZ_SIM_RESOURCE_PATH", "")
    )

    logs = results_dir() / ("runtime_" + time.strftime("%Y%m%d_%H%M%S"))

    logs.mkdir(parents=True, exist_ok=False)

    children = []
    files = []

    def launch(name, args, cwd=None):
        log = open(logs / (name + ".log"), "w")
        files.append(log)

        process = subprocess.Popen(
            args,
            cwd=cwd,
            env=env,
            stdin=subprocess.DEVNULL,
            stdout=log,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )

        children.append((name, process))
        return process

    def ensure_children_alive():
        for name, process in children:
            if process.poll() is not None:
                raise RuntimeError(
                    f"{name} exited ({process.returncode}); "
                    f"see {logs / (name + '.log')}"
                )

    def gz_topics():
        result = subprocess.run(
            ["gz", "topic", "-l"],
            env=env,
            capture_output=True,
            text=True,
            timeout=8,
        )

        if result.returncode != 0:
            return []

        return [
            line.strip()
            for line in result.stdout.splitlines()
            if line.strip()
        ]

    def discover_underfold_topics(timeout=60):
        deadline = time.monotonic() + timeout
        last_topics = []

        while time.monotonic() < deadline:
            ensure_children_alive()

            topics = gz_topics()
            last_topics = topics

            cmd1_matches = []

            pattern = re.compile(
                r"^/model/(x500_underfold(?:_[0-9]+)?)/"
                r"joint/arm_joint_1/cmd_vel$"
            )

            for topic in topics:
                match = pattern.match(topic)
                if match:
                    cmd1_matches.append(
                        (match.group(1), topic)
                    )

            if len(cmd1_matches) == 1:
                instance, cmd1 = cmd1_matches[0]

                cmd2 = (
                    f"/model/{instance}/joint/"
                    "arm_joint_2/cmd_vel"
                )

                states = [
                    topic for topic in topics
                    if topic.endswith(
                        f"/model/{instance}/joint_state"
                    )
                ]

                if cmd2 in topics and len(states) == 1:
                    return {
                        "instance": instance,
                        "cmd1_gz": cmd1,
                        "cmd2_gz": cmd2,
                        "joint_state_gz": states[0],
                    }

            time.sleep(1)

        relevant = [
            t for t in last_topics
            if "underfold" in t or "arm_joint" in t
        ]

        raise RuntimeError(
            "Could not uniquely discover x500_underfold Gazebo topics.\n"
            + "\n".join(relevant)
        )

    try:
        print(
            f"Reviewed model SHA256: {actual_sha}",
            flush=True
        )

        launch(
            "agent",
            ["MicroXRCEAgent", "udp4", "-p", "8888"]
        )

        time.sleep(1)

        launch(
            "px4",
            [str(binary), "-d"],
            px4
        )

        print(
            "PX4 started; discovering actual Gazebo model instance...",
            flush=True
        )

        discovered = discover_underfold_topics()

        print(
            "Gazebo model instance: "
            + discovered["instance"],
            flush=True
        )

        print(
            "Gazebo joint state: "
            + discovered["joint_state_gz"],
            flush=True
        )

        bridge_args = [
            "ros2",
            "run",
            "ros_gz_bridge",
            "parameter_bridge",

            discovered["cmd1_gz"]
            + "@std_msgs/msg/Float64]gz.msgs.Double",

            discovered["cmd2_gz"]
            + "@std_msgs/msg/Float64]gz.msgs.Double",

            discovered["joint_state_gz"]
            + "@sensor_msgs/msg/JointState[gz.msgs.Model",

            "/ground_truth/odometry"
            + "@nav_msgs/msg/Odometry[gz.msgs.Odometry",

            "/clock"
            + "@rosgraph_msgs/msg/Clock[gz.msgs.Clock",

            "--ros-args",

            "-r",
            discovered["cmd1_gz"]
            + ":=/arm/arm_joint_1/cmd_vel",

            "-r",
            discovered["cmd2_gz"]
            + ":=/arm/arm_joint_2/cmd_vel",

            "-r",
            discovered["joint_state_gz"]
            + ":=/arm/joint_states",
        ]

        launch("bridge", bridge_args)

        runtime = {
            "model": MODEL,
            "model_sha256": actual_sha,
            **discovered,
            "ros_joint_state": "/arm/joint_states",
            "ros_cmd1": "/arm/arm_joint_1/cmd_vel",
            "ros_cmd2": "/arm/arm_joint_2/cmd_vel",
        }

        (logs / "runtime.json").write_text(
            json.dumps(runtime, indent=2)
        )

        print("", flush=True)
        print(
            f"Services started. Logs: {logs}",
            flush=True
        )
        print("", flush=True)
        print(
            "Platform runtime is ready for amsim check/run.",
            flush=True
        )
        print(
            "  ./scripts/amsim check",
            flush=True
        )
        print("", flush=True)
        print(
            "CHECK ONLY. No arming or flight is enabled.",
            flush=True
        )
        print(
            "Stop this terminal with Ctrl+C after check exits.",
            flush=True
        )

        while True:
            ensure_children_alive()
            time.sleep(1)

    except KeyboardInterrupt:
        print(
            "Stopping owned simulation processes.",
            flush=True
        )

    finally:
        for _, process in reversed(children):
            try:
                os.killpg(process.pid, signal.SIGINT)
            except ProcessLookupError:
                pass

        time.sleep(2)

        for _, process in reversed(children):
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass

        for log in files:
            log.close()


if __name__ == "__main__":
    main()
