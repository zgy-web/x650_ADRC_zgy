"""Model identity and topic contracts shared by SITL, readiness and the API.

This module deliberately has no ROS/Gazebo imports so the model contract can be
validated before starting PX4, and exercised without a running simulator.
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import re
from typing import Mapping


REVIEWED_NAME = "x500_underfold"
REVIEWED_SHA = "8c86eb60a3e8ed494ec54d08d2ad6a04eb44a5f5e3f8e4ec5bc2dcc5ccc226c8"
REVIEWED_JOINTS = ["arm_joint_1", "arm_joint_2"]
ENV_FIELDS = {
    "AMSIM_MODEL_NAME": "model",
    "AMSIM_MODEL_SHA256": "model_sha256",
    "AMSIM_MODEL_JOINTS": "joint_names",
    "AMSIM_MODEL_SOURCE": "source",
    "AMSIM_MODEL_BASE_LINK": "base_link",
    "AMSIM_PX4_SYS_AUTOSTART": "px4_sys_autostart",
    "AMSIM_MODEL_INITIAL_JOINT_POSITIONS": "initial_joint_positions",
    "AMSIM_MODEL_FLIGHT_READY": "flight_ready",
}


def _identifier(value, label: str, *, model: bool = False) -> str:
    pattern = r"[A-Za-z_][A-Za-z0-9_-]*" if model else r"[A-Za-z_][A-Za-z0-9_]*"
    if not isinstance(value, str) or not re.fullmatch(pattern, value):
        raise RuntimeError(f"Invalid {label}: {value!r}")
    return value


def validate_joint_names(value) -> list[str]:
    if not isinstance(value, list):
        raise RuntimeError("AMSIM_MODEL_JOINTS must be a JSON list (possibly empty)")
    names = [_identifier(item, "joint name") for item in value]
    if len(set(names)) != len(names):
        raise RuntimeError("AMSIM_MODEL_JOINTS contains duplicate joint names")
    return names


def normalize_model(metadata: Mapping) -> dict:
    name = _identifier(metadata.get("model", REVIEWED_NAME), "model name", model=True)
    reviewed = name == REVIEWED_NAME
    imported = name.startswith("amsim_urdf_") or metadata.get("source") in ("urdf_import", "urdf_flight")
    if imported:
        required = {"source", "flight_ready", "px4_sys_autostart", "joint_names"}
        missing = sorted(required - set(metadata))
        if missing:
            raise RuntimeError("Imported URDF runtime requires explicit flight metadata: " + ", ".join(missing))
        if metadata["source"] != "urdf_flight":
            raise RuntimeError("Imported URDF has no flight profile; only an explicit urdf_flight binding can start PX4")
    sha = metadata.get("model_sha256", REVIEWED_SHA if reviewed else None)
    if not isinstance(sha, str) or not re.fullmatch(r"[0-9a-fA-F]{64}", sha):
        raise RuntimeError("A valid AMSIM_MODEL_SHA256 is required for the selected model")
    sha = sha.lower()
    joints = validate_joint_names(metadata.get("joint_names", REVIEWED_JOINTS if reviewed else []))
    if reviewed and (sha != REVIEWED_SHA or joints != REVIEWED_JOINTS):
        raise RuntimeError("Reviewed x500 model hash and joint contract must remain unchanged")
    source = metadata.get("source", "reviewed" if reviewed else "generated")
    if not isinstance(source, str) or not source:
        raise RuntimeError("Model source must be a non-empty string")
    base = _identifier(metadata.get("base_link", "base_link"), "base link")
    airframe = metadata.get("px4_sys_autostart", "4001")
    if isinstance(airframe, bool) or not re.fullmatch(r"[1-9][0-9]*", str(airframe)):
        raise RuntimeError("AMSIM_PX4_SYS_AUTOSTART must be a positive airframe number")
    ready = metadata.get("flight_ready", True)
    if ready in ("1", 1, True):
        ready = True
    elif ready in ("0", 0, False):
        ready = False
    else:
        raise RuntimeError("AMSIM_MODEL_FLIGHT_READY must be 0 or 1")
    if imported and not ready:
        raise RuntimeError("Imported URDF has no flight profile marked ready; use model-preview")
    initial = metadata.get("initial_joint_positions", {})
    if not isinstance(initial, dict) or set(initial) - set(joints):
        raise RuntimeError("Initial joint positions must be an object containing only controlled joints")
    positions = {}
    for joint in joints:
        value = initial.get(joint, 0.0)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise RuntimeError(f"Invalid initial position for {joint}: {value!r}")
        positions[joint] = float(value)
    return {
        "model": name,
        "model_sha256": sha,
        "joint_names": joints,
        "source": source,
        "base_link": base,
        "px4_sys_autostart": str(airframe),
        "initial_joint_positions": positions,
        "flight_ready": ready,
    }


def _env_values(environ: Mapping[str, str]) -> dict:
    values = {}
    for key, field in ENV_FIELDS.items():
        if key not in environ:
            continue
        value = environ[key]
        if field in ("joint_names", "initial_joint_positions"):
            try:
                value = json.loads(value)
            except (TypeError, json.JSONDecodeError) as exc:
                raise RuntimeError(f"Invalid {key} JSON") from exc
        values[field] = value
    return values


def model_from_env(environ: Mapping[str, str] | None = None) -> dict:
    return normalize_model(_env_values(os.environ if environ is None else environ))


def resolve_active_model(environ: Mapping[str, str] | None = None, runtime_state: dict | None = None) -> dict:
    """Use the running model as authority and reject conflicting explicit env.

    Older reviewed-model runtime metadata is supported. An active runtime with
    missing/unreadable metadata is an error; silently falling back could command
    the wrong set of joints. The no-runtime fallback supports direct API use.
    """
    environ = os.environ if environ is None else environ
    if runtime_state is None:
        from .process import status
        runtime_state = status()
    if not runtime_state.get("running"):
        return model_from_env(environ)
    path = runtime_state.get("runtime_meta")
    if not path:
        raise RuntimeError("Active runtime has no model metadata; wait for startup to complete")
    try:
        raw = json.loads(Path(path).read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"Cannot read active runtime model metadata: {path}") from exc
    if not isinstance(raw, dict):
        raise RuntimeError("Active runtime model metadata must be an object")
    if not {"model", "model_sha256", "joint_names"}.issubset(raw):
        raise RuntimeError("Active runtime metadata is missing model identity or controlled joints")
    active = normalize_model(raw)
    supplied = _env_values(environ)
    expected = normalize_model({**active, **supplied})
    for field in supplied:
        if expected[field] != active[field]:
            raise RuntimeError(f"Model environment conflicts with active runtime: {field}")
    state_model = runtime_state.get("model")
    if state_model is not None and state_model != active["model"]:
        raise RuntimeError("Runtime state and runtime model metadata disagree")
    return active


def required_telemetry(joint_names: list[str], require_truth: bool = True) -> list[str]:
    keys = ["position", "status", "land"]
    if joint_names:
        keys.append("joints")
    if require_truth:
        keys.append("truth")
    return keys


def discover_model_topics(topics: list[str], model_name: str, joint_names: list[str], base_link: str = "base_link") -> dict | None:
    """Identify one real model instance, including a vehicle with no arm.

    Sensor topics carry the instance name even when no arm command/state topics
    exist. The original joint-based discovery remains valid for legacy models.
    """
    pattern = re.compile(rf"(?:^|/)model/({re.escape(model_name)}(?:_[0-9]+)?)/(?:joint/|joint_state$|link/{re.escape(base_link)}/sensor/)")
    instances = sorted({match.group(1) for topic in topics if (match := pattern.search(topic))})
    if len(instances) != 1:
        return None
    instance = instances[0]
    commands = {joint: f"/model/{instance}/joint/{joint}/cmd_vel" for joint in joint_names}
    if any(topic not in topics for topic in commands.values()):
        return None
    states = [topic for topic in topics if topic.endswith(f"/model/{instance}/joint_state")]
    if joint_names and len(states) != 1:
        return None
    if not joint_names:
        sensor_suffix = f"/model/{instance}/link/{base_link}/sensor/imu_sensor/imu"
        if not any(topic.endswith(sensor_suffix) for topic in topics):
            return None
    return {
        "instance": instance,
        "joint_commands_gz": commands,
        "joint_state_gz": states[0] if len(states) == 1 and joint_names else None,
    }


def bridge_arguments(discovered: dict, joint_names: list[str]) -> list[str]:
    args = ["ros2", "run", "ros_gz_bridge", "parameter_bridge"]
    for joint in joint_names:
        args.append(discovered["joint_commands_gz"][joint] + "@std_msgs/msg/Float64]gz.msgs.Double")
    if joint_names:
        args.append(discovered["joint_state_gz"] + "@sensor_msgs/msg/JointState[gz.msgs.Model")
    args.extend([
        "/ground_truth/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
    ])
    if joint_names:
        args.append("--ros-args")
        for joint in joint_names:
            args.extend(["-r", discovered["joint_commands_gz"][joint] + f":=/arm/{joint}/cmd_vel"])
        args.extend(["-r", discovered["joint_state_gz"] + ":=/arm/joint_states"])
    return args


def initial_joint_values(joint_names: list[str], initial_positions: dict, message) -> dict:
    values = {}
    for name in joint_names:
        if message is None or name not in message.name:
            raise RuntimeError(f"Missing joint feedback: {name}")
        i = message.name.index(name)
        if i >= len(message.position) or i >= len(message.velocity):
            raise RuntimeError(f"Incomplete JointState arrays: {name}")
        q, velocity = message.position[i], message.velocity[i]
        if not (math.isfinite(q) and math.isfinite(velocity)):
            raise RuntimeError(f"Nonfinite joint state: {name}")
        target = initial_positions[name]
        if abs(q - target) > math.radians(0.4):
            raise RuntimeError(f"{name} differs from initial position {target:.6f}: {q:.6f} rad")
        if abs(velocity) > 0.01:
            raise RuntimeError(f"{name} moving too fast: {velocity:.6f} rad/s")
        values[name] = {"q_rad": q, "q_deg": math.degrees(q), "velocity_rad_s": velocity}
    return values
