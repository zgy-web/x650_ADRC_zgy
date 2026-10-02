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
    return subprocess.run(
        cmd,
        env=env,
        text=True,
        capture_output=True,
        timeout=timeout,
    )


def _alive(pid: int) -> bool:
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def _desktop_env() -> dict:
    env = os.environ.copy()
    env["DISPLAY"] = os.environ.get("AMSIM_DISPLAY", env.get("DISPLAY", ":1"))
    default_xauth = str(Path.home() / ".Xauthority")
    env["XAUTHORITY"] = os.environ.get(
        "AMSIM_XAUTHORITY",
        env.get("XAUTHORITY", default_xauth),
    )
    return env


def desktop_status() -> dict:
    env = _desktop_env()
    display_ok = (
        _run(["xset", "q"], env=env).returncode == 0
        if shutil.which("xset")
        else False
    )
    ss = _run(["ss", "-ltnp"]).stdout if shutil.which("ss") else ""
    xfce = (
        subprocess.run(
            ["pgrep", "-x", "xfce4-session"],
            capture_output=True,
        ).returncode
        == 0
    )
    xfwm = (
        subprocess.run(
            ["pgrep", "-x", "xfwm4"],
            capture_output=True,
        ).returncode
        == 0
    )
    return {
        "display": env["DISPLAY"],
        "xauthority": env["XAUTHORITY"],
        "display_ok": display_ok,
        "vnc_5901": "127.0.0.1:5901" in ss,
        "novnc_6080": "127.0.0.1:6080" in ss,
        "xfce_session": xfce,
        "xfwm4": xfwm,
    }


def _nvidia_info() -> tuple[str, str]:
    if not shutil.which("nvidia-smi"):
        return "", ""
    p = _run([
        "nvidia-smi",
        "--query-gpu=name,driver_version",
        "--format=csv,noheader",
    ])
    if p.returncode != 0 or not p.stdout.strip():
        return "", ""
    parts = [x.strip() for x in p.stdout.strip().splitlines()[0].split(",")]
    return parts[0], parts[1] if len(parts) > 1 else ""


def gpu_render_status() -> dict:
    backend_req = os.environ.get("AMSIM_RENDER_BACKEND", "auto").lower()
    if backend_req not in {"auto", "virtualgl", "direct"}:
        raise ValueError(
            "AMSIM_RENDER_BACKEND must be auto, virtualgl, or direct"
        )

    desktop = desktop_status()
    gpu, driver = _nvidia_info()
    ws = workspace_root()
    egl_dir = Path(os.environ.get("AMSIM_EGL_DIR", str(ws / "egl-test")))
    egl_json = egl_dir / "10_nvidia.json"
    vglrun = Path(
        os.environ.get("AMSIM_VGLRUN", "/opt/VirtualGL/bin/vglrun")
    )
    gpucomp = (
        egl_dir / "lib" / f"libnvidia-gpucomp.so.{driver}"
        if driver
        else None
    )
    bundled_egl_ready = bool(
        driver
        and egl_json.is_file()
        and gpucomp
        and gpucomp.is_file()
    )

    system_egl_json = Path("/usr/share/glvnd/egl_vendor.d/10_nvidia.json")
    system_gpucomp_candidates = [
        Path(f"/usr/lib/x86_64-linux-gnu/libnvidia-gpucomp.so.{driver}"),
        Path(f"/usr/lib64/libnvidia-gpucomp.so.{driver}"),
    ] if driver else []
    system_gpucomp = next(
        (p for p in system_gpucomp_candidates if p.is_file()),
        None,
    )
    system_egl_ready = bool(
        driver
        and system_egl_json.is_file()
        and system_gpucomp
    )

    egl_source = (
        "bundled"
        if bundled_egl_ready
        else "system"
        if system_egl_ready
        else "missing"
    )
    virtualgl_ready = bool(
        gpu
        and driver
        and vglrun.is_file()
        and egl_source != "missing"
        and desktop["display_ok"]
    )

    direct_ready = False
    direct_renderer = ""
    if desktop["display_ok"] and shutil.which("glxinfo"):
        p = _run(["glxinfo", "-B"], env=_desktop_env(), timeout=10)
        if p.returncode == 0:
            for line in p.stdout.splitlines():
                if "OpenGL renderer string:" in line:
                    direct_renderer = line.split(":", 1)[1].strip()
                    break
            direct_ready = bool(direct_renderer)

    if backend_req == "virtualgl":
        selected = "virtualgl"
        ready = virtualgl_ready
    elif backend_req == "direct":
        selected = "direct"
        ready = direct_ready
    else:
        if virtualgl_ready:
            selected = "virtualgl"
            ready = True
        else:
            selected = "direct"
            ready = direct_ready

    return {
        "requested_backend": backend_req,
        "selected_backend": selected,
        "ready": ready,
        "gpu": gpu,
        "driver": driver,
        "direct_ready": direct_ready,
        "direct_renderer": direct_renderer,
        "virtualgl_ready": virtualgl_ready,
        "virtualgl": vglrun.is_file(),
        "vglrun": str(vglrun),
        "egl_source": egl_source,
        "egl_json": str(
            egl_json if bundled_egl_ready else system_egl_json
        ),
        "egl_json_ok": bundled_egl_ready or system_egl_ready,
        "gpucomp": str(
            gpucomp
            if bundled_egl_ready
            else system_gpucomp
            if system_gpucomp
            else ""
        ),
        "gpucomp_ok": bundled_egl_ready or bool(system_gpucomp),
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
        raise RuntimeError(
            "simulation runtime is not running; run 'amsim start' first"
        )

    desktop = desktop_status()
    if not desktop["display_ok"]:
        raise RuntimeError(f"X display {desktop['display']} is unavailable")

    render = gpu_render_status()
    if not render["ready"]:
        raise RuntimeError(
            "rendering prerequisites are not ready: "
            + json.dumps(render)
        )

    env = _desktop_env()
    env["GZ_PARTITION"] = os.environ.get("GZ_PARTITION", "amsim_default")
    env["GZ_SIM_RESOURCE_PATH"] = (
        str(px4_dir() / "Tools/simulation/gz/models")
        + ":"
        + str(px4_dir() / "Tools/simulation/gz/worlds")
    )
    env.pop("HEADLESS", None)
    env.pop("LIBGL_ALWAYS_SOFTWARE", None)
    env.pop("SESSION_MANAGER", None)
    env.pop("DBUS_SESSION_BUS_ADDRESS", None)

    dbus = shutil.which("dbus-run-session")
    prefix = [dbus, "--"] if dbus else []

    if render["selected_backend"] == "virtualgl":
        if render.get("egl_source") == "bundled":
            egl_dir = Path(
                os.environ.get(
                    "AMSIM_EGL_DIR",
                    str(workspace_root() / "egl-test"),
                )
            )
            env["LD_LIBRARY_PATH"] = str(egl_dir / "lib") + (
                ":" + env["LD_LIBRARY_PATH"]
                if env.get("LD_LIBRARY_PATH")
                else ""
            )
            env["__EGL_VENDOR_LIBRARY_FILENAMES"] = str(
                egl_dir / "10_nvidia.json"
            )
        app = [
            render["vglrun"],
            "-d",
            "egl0",
            "gz",
            "sim",
            "--force-version",
            "8",
            "-g",
        ]
    else:
        app = [
            "gz",
            "sim",
            "--force-version",
            "8",
            "-g",
        ]

    topics = _run(["gz", "topic", "-l"], env=env, timeout=8)
    if topics.returncode != 0 or "/world/default/" not in topics.stdout:
        raise RuntimeError(
            "Gazebo server is not visible on the current GZ_PARTITION"
        )

    control = results_dir() / "_control"
    control.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    log_path = control / f"gui_{stamp}.log"
    log = open(log_path, "w")

    proc = subprocess.Popen(
        prefix + app,
        env=env,
        cwd=PROJECT_ROOT,
        stdin=subprocess.DEVNULL,
        stdout=log,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    time.sleep(3)

    if proc.poll() is not None:
        log.close()
        tail = log_path.read_text(errors="replace")[-5000:]
        raise RuntimeError(
            f"Gazebo GUI exited early ({proc.returncode})\n{tail}"
        )

    STATE_DIR.mkdir(parents=True, exist_ok=True)
    state = {
        "pid": proc.pid,
        "started_at": time.time(),
        "log": str(log_path),
        "display": env["DISPLAY"],
        "partition": env["GZ_PARTITION"],
        "render_backend": render["selected_backend"],
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
            return {
                "stopped": True,
                "pid": pid,
                "signal": "SIGINT",
            }
        time.sleep(0.25)

    os.killpg(pid, signal.SIGTERM)
    time.sleep(1)
    GUI_STATE.unlink(missing_ok=True)
    return {
        "stopped": not _alive(pid),
        "pid": pid,
        "signal": "SIGTERM",
    }


DESKTOP_STATE = STATE_DIR / "desktop.json"


def _desktop_owned_state() -> dict | None:
    if not DESKTOP_STATE.is_file():
        return None
    try:
        return json.loads(DESKTOP_STATE.read_text())
    except Exception:
        return None


def start_virtual_desktop() -> dict:
    """Start a localhost-only Xvfb + VNC + noVNC stack for remote servers."""
    required = ["Xvfb", "x11vnc", "websockify"]
    missing = [name for name in required if not shutil.which(name)]
    if missing:
        raise RuntimeError(
            "virtual desktop tools missing: " + ", ".join(missing)
            + "; run deploy/host/install_ubuntu22.sh"
        )

    current = _desktop_owned_state()
    if current and any(
        _alive(int(pid))
        for pid in current.get("pids", {}).values()
        if pid
    ):
        return {**current, **desktop_status(), "owned": True}

    env = _desktop_env()
    display = env["DISPLAY"]
    control = results_dir() / "_control"
    control.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    pids: dict[str, int | None] = {
        "xvfb": None,
        "x11vnc": None,
        "novnc": None,
    }
    logs: dict[str, str] = {}

    if not desktop_status()["display_ok"]:
        xvfb_log = control / f"xvfb_{stamp}.log"
        f = open(xvfb_log, "w")
        proc = subprocess.Popen(
            [
                "Xvfb", display,
                "-screen", "0", "1280x800x24",
                "-ac", "+extension", "GLX", "+render", "-noreset",
            ],
            stdin=subprocess.DEVNULL,
            stdout=f,
            stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        f.close()
        pids["xvfb"] = proc.pid
        logs["xvfb"] = str(xvfb_log)
        time.sleep(1.0)

    if not desktop_status()["display_ok"]:
        raise RuntimeError(f"failed to start X display {display}")

    vnc_log = control / f"x11vnc_{stamp}.log"
    f = open(vnc_log, "w")
    proc = subprocess.Popen(
        [
            "x11vnc", "-display", display, "-localhost",
            "-rfbport", "5901", "-forever", "-shared", "-nopw",
        ],
        stdin=subprocess.DEVNULL,
        stdout=f,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    f.close()
    pids["x11vnc"] = proc.pid
    logs["x11vnc"] = str(vnc_log)

    novnc_log = control / f"novnc_{stamp}.log"
    f = open(novnc_log, "w")
    proc = subprocess.Popen(
        [
            "websockify", "--web=/usr/share/novnc/",
            "127.0.0.1:6080", "127.0.0.1:5901",
        ],
        stdin=subprocess.DEVNULL,
        stdout=f,
        stderr=subprocess.STDOUT,
        start_new_session=True,
    )
    f.close()
    pids["novnc"] = proc.pid
    logs["novnc"] = str(novnc_log)
    time.sleep(1.5)

    status = desktop_status()
    state = {
        "display": display,
        "pids": pids,
        "logs": logs,
        "started_at": time.time(),
        "localhost_only": True,
    }
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    DESKTOP_STATE.write_text(json.dumps(state, indent=2))
    if not (
        status["display_ok"]
        and status["vnc_5901"]
        and status["novnc_6080"]
    ):
        raise RuntimeError(
            "virtual desktop did not become ready: "
            + json.dumps(status)
        )
    return {**state, **status, "owned": True}


def stop_virtual_desktop() -> dict:
    state = _desktop_owned_state()
    if not state:
        return {"stopped": True, "reason": "no owned desktop state"}

    stopped = {}
    for name in ("novnc", "x11vnc", "xvfb"):
        pid = state.get("pids", {}).get(name)
        if not pid or not _alive(int(pid)):
            stopped[name] = True
            continue
        try:
            os.killpg(int(pid), signal.SIGTERM)
        except OSError:
            pass
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline and _alive(int(pid)):
            time.sleep(0.1)
        stopped[name] = not _alive(int(pid))

    DESKTOP_STATE.unlink(missing_ok=True)
    return {"stopped": all(stopped.values()), "components": stopped}
