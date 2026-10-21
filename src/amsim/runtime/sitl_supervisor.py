"""SITL supervisor: model-aware startup and dynamic Gazebo topic discovery.

No arming or flight commands are sent here.
"""

import hashlib
import json
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import time

from ..paths import px4_dir, results_dir
from .model_metadata import bridge_arguments, discover_model_topics as match_model_topics, model_from_env




def sha256(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    # auto_sim.py may be launched as a background job by bash.
    # Non-interactive bash can start background jobs with SIGINT ignored.
    # Explicitly restore Python's SIGINT handler so the supervisor can
    # always perform its own orderly child-process cleanup.
    signal.signal(signal.SIGINT, signal.default_int_handler)

    px4 = px4_dir()

    model = model_from_env()
    if not model["flight_ready"]:
        raise RuntimeError("This imported model has no flight profile; use model-preview before configuring PX4")
    model_name = model["model"]
    expected_sha = model["model_sha256"]
    joint_names = model["joint_names"]

    models = px4 / "Tools/simulation/gz/models"
    installed_model_dir = models / model_name
    model_sdf = installed_model_dir / "model.sdf"
    binary = px4 / "build/px4_sitl_default/bin/px4"

    for path in (binary, model_sdf):
        if not path.is_file():
            raise RuntimeError(f"Missing: {path}")

    actual_sha = sha256(model_sdf)
    if actual_sha != expected_sha:
        raise RuntimeError(
            f"Installed model hash differs for {model_name}.\n"
            f"expected={expected_sha}\n"
            f"actual={actual_sha}"
        )

    if model_name.startswith("amsim_urdf_") or model["source"] == "urdf_flight":
        # Also enforce the import contract for direct supervisor invocation;
        # environment variables alone are not evidence of a flight binding.
        from ..modeling.manager import env_for, resolve
        installed = resolve(str(installed_model_dir / "import_manifest.json"))
        installed_metadata = model_from_env(env_for(installed))
        if installed_metadata != model:
            raise RuntimeError("Installed URDF flight manifest conflicts with requested runtime model metadata")

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
        PX4_SYS_AUTOSTART=model["px4_sys_autostart"],
        PX4_SIM_MODEL=model_name,
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

    def discover_model_topics(timeout=60):
        deadline = time.monotonic() + timeout
        last_topics = []

        while time.monotonic() < deadline:
            ensure_children_alive()
            topics = gz_topics()
            last_topics = topics

            discovered = match_model_topics(topics, model_name, joint_names, model["base_link"])
            if discovered is not None:
                return discovered

            time.sleep(1)

        relevant = [
            t for t in last_topics
            if model_name in t or "arm_joint" in t
        ]
        raise RuntimeError(
            f"Could not uniquely discover Gazebo topics for {model_name}.\n"
            + "\n".join(relevant)
        )

    try:
        print(
            f"Model {model_name} SHA256: {actual_sha}",
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

        discovered = discover_model_topics()

        print(
            "Gazebo model instance: "
            + discovered["instance"],
            flush=True
        )

        print(
            "Gazebo joint state: "
            + (discovered["joint_state_gz"] or "not required (no controlled arm joints)"),
            flush=True
        )

        bridge_args = bridge_arguments(discovered, joint_names)

        launch("bridge", bridge_args)

        runtime = {
            **model,
            **discovered,
            "ros_joint_state": "/arm/joint_states" if joint_names else None,
            "ros_joint_commands": {
                joint_name: f"/arm/{joint_name}/cmd_vel"
                for joint_name in joint_names
            },
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
