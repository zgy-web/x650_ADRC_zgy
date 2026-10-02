from __future__ import annotations
import json
import os
import signal
import shutil
import subprocess
import time
from pathlib import Path

from .paths import PROJECT_ROOT, px4_dir, results_dir, workspace_root

STATE_DIR = PROJECT_ROOT / ".amsim"
GUI_STATE = STATE_DIR / "gui.json"

def _run(cmd, env=None, timeout=10):
    return subprocess.run(cmd, env=env, text=True, capture_output=True, timeout=timeout)

def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False

def _desktop_env() -> dict:
    env = os.environ.copy()
    env["DISPLAY"] = os.environ.get("AMSIM_DISPLAY", ":1")
    env["XAUTHORITY"] = os.environ.get("AMSIM_XAUTHORITY", "/root/.Xauthority")
    return env

def desktop_status() -> dict:
    env = _desktop_env()
    display_ok = _run(["xset", "q"], env=env).returncode == 0 if shutil.which("xset") else False
    ss = _run(["ss", "-ltnp"]).stdout if shutil.which("ss") else ""
    vnc = "127.0.0.1:5901" in ss
    novnc = "127.0.0.1:6080" in ss
    xfce = subprocess.run(["pgrep", "-x", "xfce4-session"], capture_output=True).returncode == 0
    xfwm = subprocess.run(["pgrep", "-x", "xfwm4"], capture_output=True).returncode == 0
    return {
        "display": env["DISPLAY"],
        "display_ok": display_ok,
        "vnc_5901": vnc,
        "novnc_6080": novnc,
        "xfce_session": xfce,
        "xfwm4": xfwm,
    }

def gpu_render_status() -> dict:
    ws = workspace_root()
    egl_dir = Path(os.environ.get("AMSIM_EGL_DIR", str(ws / "egl-test")))
    egl_json = egl_dir / "10_nvidia.json"
    vglrun = Path(os.environ.get("AMSIM_VGLRUN", "/opt/VirtualGL/bin/vglrun"))
    driver = ""
    gpu = ""
    if shutil.which("nvidia-smi"):
        p = _run(["nvidia-smi", "--query-gpu=name,driver_version", "--format=csv,noheader"])
        if p.returncode == 0 and p.stdout.strip():
            line = p.stdout.strip().splitlines()[0]
            parts = [x.strip() for x in line.split(",")]
            gpu = parts[0]
            driver = parts[1] if len(parts) > 1 else ""
    gpucomp = egl_dir / "lib" / f"libnvidia-gpucomp.so.{driver}" if driver else Path("")
    return {
        "gpu": gpu,
        "driver": driver,
        "virtualgl": vglrun.is_file(),
        "vglrun": str(vglrun),
        "egl_json": str(egl_json),
        "egl_json_ok": egl_json.is_file(),
        "gpucomp": str(gpucomp) if driver else "",
        "gpucomp_ok": gpucomp.is_file() if driver else False,
        "ready": bool(gpu and vglrun.is_file() and egl_json.is_file() and driver and gpucomp.is_file()),
    }

def gui_status() -> dict:
    if not GUI_STATE.is_file():
        return {"running": False, "reason": "no gui state file"}
    try:
        state = json.loads(GUI_STATE.read_text())
    except Exception:
        return {"running": False, "reason": "invalid gui state file"}
    pid = int(state.get("pid", -1))
    return {**state, "running": _alive(pid)}

def start_gui() -> dict:
    current = gui_status()
    if current.get("running"):
        raise RuntimeError(f"GUI already running (pid={current['pid']})")

    from .runtime.process import status as runtime_status
    rt = runtime_status()
    if not rt.get("running"):
        raise RuntimeError("simulation runtime is not running; run 'amsim start' first")

    desktop = desktop_status()
    if not desktop["display_ok"]:
        raise RuntimeError(f"X display {desktop['display']} is unavailable")

    render = gpu_render_status()
    if not render["ready"]:
        raise RuntimeError("GPU rendering prerequisites are not ready: " + json.dumps(render))

    env = _desktop_env()
    env["GZ_PARTITION"] = os.environ.get("GZ_PARTITION", "amsim_default")
    env["GZ_SIM_RESOURCE_PATH"] = (
        str(px4_dir() / "Tools/simulation/gz/models") + ":" +
        str(px4_dir() / "Tools/simulation/gz/worlds")
    )
    env.pop("HEADLESS", None)
    env.pop("LIBGL_ALWAYS_SOFTWARE", None)
    env.pop("SESSION_MANAGER", None)
    env.pop("DBUS_SESSION_BUS_ADDRESS", None)

    egl_dir = Path(os.environ.get("AMSIM_EGL_DIR", str(workspace_root() / "egl-test")))
    env["LD_LIBRARY_PATH"] = str(egl_dir / "lib") + (
        ":" + env["LD_LIBRARY_PATH"] if env.get("LD_LIBRARY_PATH") else ""
    )
    env["__EGL_VENDOR_LIBRARY_FILENAMES"] = str(egl_dir / "10_nvidia.json")

    topics = _run(["gz", "topic", "-l"], env=env, timeout=8)
    if topics.returncode != 0 or "/world/default/" not in topics.stdout:
        raise RuntimeError("Gazebo server is not visible on the current GZ_PARTITION")

    control = results_dir() / "_control"
    control.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    log_path = control / f"gui_{stamp}.log"
    log = open(log_path, "w")
    cmd = [
        "/usr/bin/dbus-run-session", "--",
        render["vglrun"], "-d", "egl0",
        "gz", "sim", "--force-version", "8", "-g",
    ]
    proc = subprocess.Popen(
        cmd, env=env, cwd=PROJECT_ROOT,
        stdin=subprocess.DEVNULL, stdout=log, stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    time.sleep(3)
    if proc.poll() is not None:
        log.close()
        tail = log_path.read_text(errors="replace")[-5000:]
        raise RuntimeError(f"Gazebo GUI exited early ({proc.returncode})\n{tail}")

    STATE_DIR.mkdir(parents=True, exist_ok=True)
    state = {
        "pid": proc.pid,
        "started_at": time.time(),
        "log": str(log_path),
        "display": env["DISPLAY"],
        "partition": env["GZ_PARTITION"],
    }
    GUI_STATE.write_text(json.dumps(state, indent=2))
    log.close()
    return {**state, "running": True}

def stop_gui() -> dict:
    state = gui_status()
    if not state.get("running"):
        GUI_STATE.unlink(missing_ok=True)
        return {"stopped": True, "reason": "GUI not running"}
    pid = int(state["pid"])
    os.killpg(pid, signal.SIGINT)
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        if not _alive(pid):
            GUI_STATE.unlink(missing_ok=True)
            return {"stopped": True, "pid": pid, "signal": "SIGINT"}
        time.sleep(0.25)
    os.killpg(pid, signal.SIGTERM)
    time.sleep(1)
    GUI_STATE.unlink(missing_ok=True)
    return {"stopped": not _alive(pid), "pid": pid, "signal": "SIGTERM"}
