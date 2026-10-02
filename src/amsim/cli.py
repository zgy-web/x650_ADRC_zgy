from __future__ import annotations
import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from . import __version__
from .doctor import check
from .paths import PROJECT_ROOT, px4_dir, ros_ws, results_dir, reference_dir, workspace_root

def cmd_info(_: argparse.Namespace) -> int:
    payload = {
        "version": __version__,
        "project_root": str(PROJECT_ROOT),
        "px4_dir": str(px4_dir()),
        "ros_ws": str(ros_ws()),
        "results_dir": str(results_dir()),
        "reference_dir": str(reference_dir()),
    }
    print(json.dumps(payload, indent=2))
    return 0

def cmd_doctor(args: argparse.Namespace) -> int:
    ok, checks = check()
    if args.json:
        print(json.dumps({"ok": ok, "checks": checks}, indent=2))
    else:
        print("Aerial Manipulator Simulation Platform — doctor")
        print("=" * 64)
        for item in checks:
            mark = "PASS" if item["ok"] else "FAIL"
            print(f"[{mark}] {item['name']}: {item['detail']}")
        print("=" * 64)
        print("READY" if ok else "NOT READY")
    return 0 if ok else 2

def cmd_start(args: argparse.Namespace) -> int:
    from .runtime.process import start
    state = start(wait_s=args.wait)
    print(json.dumps(state, indent=2))
    return 0

def cmd_stop(_: argparse.Namespace) -> int:
    from .runtime.process import stop
    result = stop()
    print(json.dumps(result, indent=2))
    return 0 if result.get("stopped") else 2

def cmd_status(_: argparse.Namespace) -> int:
    from .runtime.process import status
    result = status()
    print(json.dumps(result, indent=2))
    return 0 if result.get("running") else 1

def _check_python() -> Path:
    explicit = os.environ.get("AMSIM_CHECK_PYTHON")
    if explicit:
        return Path(explicit)
    candidate = workspace_root() / "cloud-arm-venv" / "bin" / "python3"
    if candidate.is_file():
        return candidate
    return Path(sys.executable)

def cmd_check(_: argparse.Namespace) -> int:
    from .runtime.process import status
    st = status()
    if not st.get("running"):
        print("[ERROR] runtime is not running; use 'amsim start' first", file=sys.stderr)
        return 2
    env = os.environ.copy()
    existing = env.get("PYTHONPATH", "")
    env["PYTHONPATH"] = str(PROJECT_ROOT / "src") + (":" + existing if existing else "")
    cmd = [str(_check_python()), "-m", "amsim.runtime.readiness_check"]
    return subprocess.run(cmd, cwd=PROJECT_ROOT, env=env).returncode

def cmd_desktop_status(_: argparse.Namespace) -> int:
    from .visualization import desktop_status
    result = desktop_status()
    print(json.dumps(result, indent=2))
    return 0 if result.get("display_ok") else 2

def cmd_render_status(_: argparse.Namespace) -> int:
    from .visualization import gpu_render_status
    result = gpu_render_status()
    print(json.dumps(result, indent=2))
    return 0 if result.get("ready") else 2

def cmd_gui_start(_: argparse.Namespace) -> int:
    from .visualization import start_gui
    result = start_gui()
    print(json.dumps(result, indent=2))
    return 0

def cmd_gui_status(_: argparse.Namespace) -> int:
    from .visualization import gui_status
    result = gui_status()
    print(json.dumps(result, indent=2))
    return 0 if result.get("running") else 1

def cmd_gui_stop(_: argparse.Namespace) -> int:
    from .visualization import stop_gui
    result = stop_gui()
    print(json.dumps(result, indent=2))
    return 0 if result.get("stopped") else 2

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="amsim")
    p.add_argument("--version", action="version", version=__version__)
    sp = p.add_subparsers(dest="command", required=True)

    a = sp.add_parser("info", help="show resolved platform paths")
    a.set_defaults(func=cmd_info)

    d = sp.add_parser("doctor", help="check platform prerequisites")
    d.add_argument("--json", action="store_true")
    d.set_defaults(func=cmd_doctor)

    s = sp.add_parser("start", help="start PX4 SITL/Gazebo runtime")
    s.add_argument("--wait", type=float, default=75.0)
    s.set_defaults(func=cmd_start)

    c = sp.add_parser("check", help="run non-flight readiness check")
    c.set_defaults(func=cmd_check)

    x = sp.add_parser("stop", help="stop the owned runtime")
    x.set_defaults(func=cmd_stop)

    st = sp.add_parser("status", help="show runtime status")
    st.set_defaults(func=cmd_status)

    ds = sp.add_parser("desktop-status", help="check remote desktop/X availability")
    ds.set_defaults(func=cmd_desktop_status)

    rs = sp.add_parser("render-status", help="check GPU/VirtualGL rendering prerequisites")
    rs.set_defaults(func=cmd_render_status)

    gs = sp.add_parser("gui-start", help="attach GPU-rendered Gazebo GUI to running simulation")
    gs.set_defaults(func=cmd_gui_start)

    gst = sp.add_parser("gui-status", help="show Gazebo GUI status")
    gst.set_defaults(func=cmd_gui_status)

    gx = sp.add_parser("gui-stop", help="stop only the Gazebo GUI")
    gx.set_defaults(func=cmd_gui_stop)

    return p

def main() -> int:
    args = build_parser().parse_args()
    try:
        return args.func(args)
    except KeyboardInterrupt:
        return 130
    except Exception as exc:
        print(f"[ERROR] {exc}", file=sys.stderr)
        return 2

if __name__ == "__main__":
    sys.exit(main())
