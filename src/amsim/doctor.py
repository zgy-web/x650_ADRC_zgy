from __future__ import annotations
import hashlib
import os
import shutil
import subprocess
from pathlib import Path
from .paths import PROJECT_ROOT, px4_dir, ros_ws, results_dir, model_dir

EXPECTED_MODEL_SHA = "8c86eb60a3e8ed494ec54d08d2ad6a04eb44a5f5e3f8e4ec5bc2dcc5ccc226c8"
EXPECTED_PX4_COMMIT = "d6f12ad1c4f70ad3230afd7d86e971421e02fef4"

def run(cmd: list[str], cwd: Path | None = None, env: dict | None = None) -> tuple[int, str]:
    p = subprocess.run(cmd, cwd=cwd, env=env, text=True, capture_output=True)
    return p.returncode, (p.stdout + p.stderr).strip()

def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

def resolve_ros2() -> Path | None:
    found = shutil.which("ros2")
    if found:
        return Path(found)
    distro = os.environ.get("ROS_DISTRO", "humble")
    candidate = Path("/opt/ros") / distro / "bin" / "ros2"
    return candidate if candidate.is_file() else None

def check() -> tuple[bool, list[dict]]:
    checks: list[dict] = []
    def add(name: str, ok: bool, detail: str):
        checks.append({"name": name, "ok": ok, "detail": detail})

    add("project_root", PROJECT_ROOT.is_dir(), str(PROJECT_ROOT))
    python = shutil.which("python3")
    add("command:python3", bool(python), python or "not found")
    ros2 = resolve_ros2()
    add("command:ros2", ros2 is not None, str(ros2) if ros2 else "not found")
    for exe in ("gz", "MicroXRCEAgent"):
        p = shutil.which(exe)
        add(f"command:{exe}", bool(p), p or "not found")

    px4 = px4_dir()
    add("px4_dir", px4.is_dir(), str(px4))
    binary = px4 / "build/px4_sitl_default/bin/px4"
    add("px4_binary", binary.is_file(), str(binary))

    if (px4 / ".git").exists():
        rc, out = run(["git", "rev-parse", "HEAD"], cwd=px4)
        head = out.splitlines()[0] if out else ""
        add("px4_commit", rc == 0 and head == EXPECTED_PX4_COMMIT,
            f"{head or 'unknown'} expected={EXPECTED_PX4_COMMIT}")
    else:
        add("px4_commit", False, "PX4 directory is not a git checkout")

    model = model_dir() / "model.sdf"
    if model.is_file():
        actual = sha256(model)
        add("reviewed_model", actual == EXPECTED_MODEL_SHA,
            f"{actual} expected={EXPECTED_MODEL_SHA}")
    else:
        add("reviewed_model", False, str(model))

    ros = ros_ws() / "install/setup.bash"
    add("ros_workspace", ros.is_file(), str(ros))

    results = results_dir()
    try:
        results.mkdir(parents=True, exist_ok=True)
        probe = results / ".write_test"
        probe.write_text("ok")
        probe.unlink()
        add("results_writable", True, str(results))
    except Exception as exc:
        add("results_writable", False, f"{results}: {exc}")

    require_gpu = os.environ.get("AMSIM_REQUIRE_GPU", "1").lower() not in {"0", "false", "no"}
    gpu = shutil.which("nvidia-smi")
    if gpu:
        rc, out = run([gpu, "--query-gpu=name,driver_version", "--format=csv,noheader"])
        add("gpu", rc == 0, out or "nvidia-smi failed")
    elif require_gpu:
        add("gpu", False, "nvidia-smi not found")
    else:
        add("gpu", True, "not required (AMSIM_REQUIRE_GPU=0)")

    return all(x["ok"] for x in checks), checks
