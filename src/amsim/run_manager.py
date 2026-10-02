from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import time
from pathlib import Path

from . import __version__
from .modeling.manager import (
    env_for as model_env,
    install as install_model,
    resolve as resolve_model,
)
from .paths import PROJECT_ROOT, px4_dir, results_dir, workspace_root
from .recording import RosbagRecorder
from .runtime.process import start as runtime_start
from .runtime.process import status as runtime_status
from .runtime.process import stop as runtime_stop
from .scenario_registry import get as get_scenario
from .visualization import start_gui, stop_gui


def _sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(4 * 1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def _git_head(path: Path) -> str | None:
    p = subprocess.run(
        ["git", "rev-parse", "HEAD"],
        cwd=path,
        text=True,
        capture_output=True,
    )
    return p.stdout.strip() if p.returncode == 0 else None


def _scenario_python() -> Path:
    explicit = os.environ.get("AMSIM_SCENARIO_PYTHON")
    if explicit:
        return Path(explicit)
    candidate = workspace_root() / "cloud-arm-venv" / "bin" / "python3"
    if candidate.is_file():
        return candidate
    return Path("/usr/bin/python3")


def _active_ulog() -> Path:
    root = px4_dir() / "build/px4_sitl_default/rootfs/log"
    files = list(root.rglob("*.ulg"))
    if not files:
        raise RuntimeError("No PX4 ULog found after runtime startup.")
    src = max(files, key=lambda p: p.stat().st_mtime)
    age = time.time() - src.stat().st_mtime
    if age > 20:
        raise RuntimeError(f"Newest PX4 ULog is stale (age={age:.1f}s): {src}")
    return src.resolve()


def _readiness(env: dict) -> None:
    cmd = [str(_scenario_python()), "-m", "amsim.runtime.readiness_check"]
    p = subprocess.run(cmd, cwd=PROJECT_ROOT, env=env)
    if p.returncode != 0:
        raise RuntimeError(f"readiness check failed with rc={p.returncode}")


def _finalize_ulog(folder: Path, marker: dict) -> None:
    src = Path(marker["source"])
    if not src.is_file():
        raise RuntimeError(f"ULog source missing after PX4 shutdown: {src}")
    st = src.stat()
    if int(st.st_dev) != marker["dev"] or int(st.st_ino) != marker["inode"]:
        raise RuntimeError("ULog source identity changed; refusing archive.")

    dst = folder / "px4.ulg"
    tmp = folder / ".px4.ulg.tmp"
    shutil.copy2(src, tmp)
    os.replace(tmp, dst)

    result_path = folder / "result.json"
    result = json.loads(result_path.read_text()) if result_path.is_file() else {}
    result.update({
        "ulog_source": str(src),
        "ulog_copied": True,
        "ulog_archive_method": "copy_after_px4_shutdown",
        "ulog_sha256": _sha256(dst),
        "ulog_size_bytes": dst.stat().st_size,
        "ulog_finalized_wall_time": time.time(),
    })
    result_path.write_text(json.dumps(result, indent=2))


def _scenario_command(module: str) -> list[str]:
    py = str(_scenario_python())
    path = Path(module)
    if path.suffix == ".py":
        if not path.is_absolute():
            path = PROJECT_ROOT / path
        if not path.is_file():
            raise RuntimeError(f"Scenario Python file not found: {path}")
        return [py, str(path)]
    return [py, "-m", module]


def _apply_builtin_compat(name: str, config: dict, env: dict) -> None:
    if name == "arm_motion":
        joint = config["arm"]["joints"]["arm_joint_1"]
        env["MOTION_Q1_TARGET_DEG"] = str(joint["target_deg"])
        env["MOTION_MAX_SPEED_DEG_S"] = str(joint["max_speed_deg_s"])
        env["MOTION_HOLD_S"] = str(config["timeline"]["target_hold_s"])
        env["MOTION_PRE_HOLD_S"] = str(config["timeline"]["pre_hold_s"])
        env["MOTION_POST_HOLD_S"] = str(config["timeline"]["post_hold_s"])


def _managed_folder(name: str) -> Path:
    folder = (
        results_dir()
        / (f"{name}_" + time.strftime("%Y%m%d_%H%M%S") + f"_{os.getpid()}")
    )
    folder.mkdir(parents=True, exist_ok=False)
    return folder


def _snapshot_inputs(folder: Path, spec, model_spec) -> None:
    shutil.copy2(spec.config_path, folder / "scenario.yaml")

    if model_spec.config_path:
        shutil.copy2(model_spec.config_path, folder / "model.yaml")

    if not (folder / "model_snapshot.sdf").exists():
        shutil.copy2(model_spec.sdf_path, folder / "model_snapshot.sdf")

    module_path = Path(spec.module)
    if module_path.suffix == ".py":
        if not module_path.is_absolute():
            module_path = PROJECT_ROOT / module_path
        if module_path.is_file():
            shutil.copy2(module_path, folder / "experiment.py")


def run(name: str, headless: bool = False) -> Path:
    spec = get_scenario(name)
    config_path = spec.config_path
    config = spec.config
    model_spec = resolve_model(str(config.get("model", "x500_underfold")))
    install_model(model_spec)

    if runtime_status().get("running"):
        raise RuntimeError(
            "An amsim runtime is already running; stop it before 'amsim run'."
        )

    env = os.environ.copy()
    prior_pp = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = str(PROJECT_ROOT / "src") + (
        ":" + prior_pp if prior_pp else ""
    )
    env["AMSIM_SCENARIO_CONFIG"] = str(config_path)
    env["AMSIM_SCENARIO_NAME"] = name
    env.update(model_env(model_spec))
    _apply_builtin_compat(name, config, env)

    before = {
        p.resolve()
        for p in results_dir().glob(f"{name}_*")
        if p.is_dir()
    }

    runtime_owned = False
    gui_owned = False
    recorder = None
    scenario_rc = None
    folder = None
    marker = None
    error_text = None
    started_wall = time.time()

    try:
        runtime_start(wait_s=75, env_overrides=model_env(model_spec))
        runtime_owned = True
        _readiness(env)

        if not headless:
            start_gui()
            gui_owned = True

        src = _active_ulog()
        st = src.stat()
        marker = {
            "source": str(src),
            "dev": int(st.st_dev),
            "inode": int(st.st_ino),
        }

        if spec.source == "user":
            folder = _managed_folder(name)
            env["AMSIM_RUN_DIR"] = str(folder)
            _snapshot_inputs(folder, spec, model_spec)

            record_cfg = config.get("record") or {}
            if bool(record_cfg.get("rosbag", True)):
                recorder = RosbagRecorder(
                    folder,
                    model_spec.joints,
                    include_ground_truth=bool(
                        record_cfg.get("ground_truth", True)
                    ),
                )
                recorder.start()

        scenario_rc = subprocess.run(
            _scenario_command(spec.module),
            cwd=PROJECT_ROOT,
            env=env,
        ).returncode

        if spec.source != "user":
            after = {
                p.resolve()
                for p in results_dir().glob(f"{name}_*")
                if p.is_dir()
            }
            created = sorted(after - before, key=lambda p: p.stat().st_mtime)
            if len(created) != 1:
                raise RuntimeError(
                    "Could not uniquely identify scenario result directory: "
                    + repr([str(p) for p in created])
                )
            folder = created[0]

        if folder is None:
            raise RuntimeError("Scenario result directory was not created.")

    except Exception as exc:
        error_text = str(exc) or exc.__class__.__name__
        if scenario_rc is None:
            scenario_rc = 2

    finally:
        if recorder is not None:
            try:
                recorder.stop()
            except Exception as exc:
                if error_text is None:
                    error_text = f"rosbag stop failed: {exc}"

        if gui_owned:
            try:
                stop_gui()
            except Exception:
                pass

        if runtime_owned:
            runtime_stop()

        if folder is not None:
            result_path = folder / "result.json"
            result = (
                json.loads(result_path.read_text())
                if result_path.is_file()
                else {}
            )
            result.setdefault("experiment", name)
            result["success"] = bool(
                scenario_rc == 0 and error_text is None
            )
            result["scenario_returncode"] = scenario_rc
            if error_text is not None:
                result["platform_error"] = error_text
            result_path.write_text(json.dumps(result, indent=2))

    if folder is None:
        raise RuntimeError("Scenario did not produce a result directory.")

    if marker is not None and bool((config.get("record") or {}).get("px4_ulog", True)):
        _finalize_ulog(folder, marker)

    _snapshot_inputs(folder, spec, model_spec)

    manifest = {
        "schema_version": 1,
        "platform_version": __version__,
        "platform_git_commit": _git_head(PROJECT_ROOT),
        "px4_git_commit": _git_head(px4_dir()),
        "model_name": model_spec.model_name,
        "model_sha256": model_spec.sha256,
        "model_source": model_spec.source,
        "model_config": (
            str(model_spec.config_path)
            if model_spec.config_path
            else None
        ),
        "scenario": name,
        "scenario_source": spec.source,
        "scenario_module": spec.module,
        "headless": headless,
        "started_wall_time": started_wall,
        "finished_wall_time": time.time(),
        "scenario_returncode": scenario_rc,
        "record": config.get("record") or {},
        "config": config,
    }
    (folder / "run_manifest.json").write_text(json.dumps(manifest, indent=2))

    if error_text is not None:
        raise RuntimeError(
            f"Scenario '{name}' platform failure: {error_text}; results: {folder}"
        )
    if scenario_rc != 0:
        raise RuntimeError(
            f"Scenario '{name}' failed with rc={scenario_rc}; results: {folder}"
        )

    return folder
