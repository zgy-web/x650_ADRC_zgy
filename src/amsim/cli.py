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

def cmd_setup(args: argparse.Namespace) -> int:
    script = PROJECT_ROOT / "deploy" / "host" / "bootstrap_workspace.sh"
    cmd = [str(script)]
    if args.check_only:
        cmd.append("--check-only")
    if args.skip_px4_build:
        cmd.append("--skip-px4-build")
    return subprocess.run(cmd, cwd=PROJECT_ROOT).returncode


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
    from .modeling.manager import resolve, install, env_for
    spec = resolve(args.model)
    model_env = env_for(spec)
    install(spec)
    state = start(wait_s=args.wait, env_overrides=model_env)
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

def cmd_desktop_start(_: argparse.Namespace) -> int:
    from .visualization import start_virtual_desktop
    result = start_virtual_desktop()
    print(json.dumps(result, indent=2))
    return 0


def cmd_desktop_stop(_: argparse.Namespace) -> int:
    from .visualization import stop_virtual_desktop
    result = stop_virtual_desktop()
    print(json.dumps(result, indent=2))
    return 0 if result.get("stopped") else 2


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


def cmd_scenarios(_: argparse.Namespace) -> int:
    from .scenario_registry import discover
    rows = []
    for name, spec in sorted(discover().items()):
        rows.append({
            "name": name,
            "source": spec.source,
            "config": str(spec.config_path),
            "module": spec.module,
        })
    print(json.dumps(rows, indent=2))
    return 0

def cmd_new_scenario(args: argparse.Namespace) -> int:
    from .scenario_registry import create_template
    py_path, yaml_path = create_template(args.name)
    print(json.dumps({
        "created": True,
        "python": str(py_path),
        "config": str(yaml_path),
    }, indent=2))
    return 0

def cmd_run(args: argparse.Namespace) -> int:
    from .run_manager import run
    folder = run(args.scenario, headless=args.headless)
    print(json.dumps({"success": True, "scenario": args.scenario, "results": str(folder)}, indent=2))
    return 0

def cmd_model_list(_: argparse.Namespace) -> int:
    from .modeling.manager import inventory
    print(json.dumps(inventory(), indent=2))
    return 0


def _urdf_options(args: argparse.Namespace) -> dict:
    packages = {}
    for item in args.package:
        name, separator, path = item.partition("=")
        if not separator or not name or not path or name in packages:
            raise ValueError("--package requires a unique NAME=PATH")
        packages[name] = Path(path)
    return {
        "urdf_entry": args.urdf_entry,
        "root_xyz": args.root_xyz,
        "root_rpy": args.root_rpy,
        "package_paths": packages,
    }


def cmd_model_inspect(args: argparse.Namespace) -> int:
    from .modeling.urdf_importer import inspect_urdf
    report = inspect_urdf(Path(args.source), **_urdf_options(args))
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 0


def cmd_model_import(args: argparse.Namespace) -> int:
    from .modeling.urdf_importer import import_urdf
    model = import_urdf(Path(args.source), args.name, **_urdf_options(args))
    print(json.dumps({
        "model_name": model.model_name, "directory": str(model.directory),
        "manifest": str(model.manifest_path), "sdf": str(model.sdf_path),
        "sha256": model.sha256, "total_mass_kg": model.total_mass_kg,
        "flight_ready": False,
        "next": "model-preview; bind an explicit flight profile before PX4 use",
    }, indent=2))
    return 0


def cmd_model_bind_flight(args: argparse.Namespace) -> int:
    from .modeling.manager import resolve
    from .modeling.urdf_flight import bind_flight_profile
    spec = resolve(args.model)
    model = bind_flight_profile(spec.sdf_path.parent, Path(args.profile))
    print(json.dumps({
        "model_name": model.name, "directory": str(model.output_dir),
        "manifest": str(model.manifest_path), "sha256": model.sha256,
        "flight_ready": True, "flight_validated": False,
        "px4_sys_autostart": model.px4_sys_autostart,
    }, indent=2))
    return 0


def cmd_model_preview(args: argparse.Namespace) -> int:
    from .modeling.manager import resolve
    from .modeling.urdf_preview import build_preview, check_preview
    spec = resolve(args.model)
    world = build_preview(spec.sdf_path.parent, height=args.height, static=args.static)
    if args.check:
        result = check_preview(world, iterations=args.iterations)
        print(json.dumps(result, indent=2))
        return 0 if result.get("ok") else 2
    metadata = json.loads((world.parent / "preview_manifest.json").read_text(encoding="utf-8"))
    print(json.dumps({"world": str(world), "open_command": metadata["gui_command"],
                      "px4_started": False}, indent=2))
    return 0

def cmd_hil_doctor(args: argparse.Namespace) -> int:
    from .hil.doctor import probe
    result = probe(args.mavlink, args.baud, args.timeout)
    if args.require_hil and result.get("ok") and not result.get("hil_enabled"):
        result["ok"] = False
        result["error"] = "PX4 heartbeat present but HIL mode is not enabled"
    print(json.dumps(result, indent=2))
    return 0 if result.get("ok") else 2


def cmd_hil_start(args: argparse.Namespace) -> int:
    from .hil.process import start
    result = start(
        args.mavlink,
        baud=args.baud,
        model=args.model,
        xrce_serial=args.xrce_serial,
        xrce_baud=args.xrce_baud,
        wait_s=args.wait,
    )
    print(json.dumps(result, indent=2))
    return 0


def cmd_hil_status(_: argparse.Namespace) -> int:
    from .hil.process import status
    result = status()
    print(json.dumps(result, indent=2))
    return 0 if result.get("running") else 1


def cmd_hil_stop(_: argparse.Namespace) -> int:
    from .hil.process import stop
    result = stop()
    print(json.dumps(result, indent=2))
    return 0 if result.get("stopped") else 2


def cmd_model_build(args: argparse.Namespace) -> int:
    from .modeling.manager import resolve, validate_sdf
    spec = resolve(args.model)
    ok, detail = validate_sdf(spec)
    payload = {
        "ok": ok,
        "ref": spec.ref,
        "model_name": spec.model_name,
        "source": spec.source,
        "sdf": str(spec.sdf_path),
        "sha256": spec.sha256,
        "joints": spec.joints,
        "total_arm_mass_kg": spec.total_arm_mass_kg,
        "total_mass_kg": spec.total_mass_kg,
        "flight_ready": spec.flight_ready,
    }
    print(json.dumps(payload, indent=2))
    if not ok:
        print(detail, file=sys.stderr)
    return 0 if ok else 2

def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(prog="amsim")
    p.add_argument("--version", action="version", version=__version__)
    sp = p.add_subparsers(dest="command", required=True)

    setup = sp.add_parser("setup", help="prepare or validate a portable host workspace")
    setup.add_argument("--check-only", action="store_true")
    setup.add_argument("--skip-px4-build", action="store_true")
    setup.set_defaults(func=cmd_setup)

    a = sp.add_parser("info", help="show resolved platform paths")
    a.set_defaults(func=cmd_info)

    d = sp.add_parser("doctor", help="check platform prerequisites")
    d.add_argument("--json", action="store_true")
    d.set_defaults(func=cmd_doctor)

    s = sp.add_parser("start", help="start PX4 SITL/Gazebo runtime")
    s.add_argument("--wait", type=float, default=75.0)
    s.add_argument("--model", default="x500_underfold", help="reviewed model or configurable model ref")
    s.set_defaults(func=cmd_start)

    c = sp.add_parser("check", help="run non-flight readiness check")
    c.set_defaults(func=cmd_check)

    x = sp.add_parser("stop", help="stop the owned runtime")
    x.set_defaults(func=cmd_stop)

    st = sp.add_parser("status", help="show runtime status")
    st.set_defaults(func=cmd_status)

    ds = sp.add_parser("desktop-status", help="check remote desktop/X availability")
    ds.set_defaults(func=cmd_desktop_status)

    dstart = sp.add_parser("desktop-start", help="start localhost-only Xvfb/VNC/noVNC")
    dstart.set_defaults(func=cmd_desktop_start)

    dstop = sp.add_parser("desktop-stop", help="stop the owned virtual desktop")
    dstop.set_defaults(func=cmd_desktop_stop)

    rs = sp.add_parser("render-status", help="check GPU/VirtualGL rendering prerequisites")
    rs.set_defaults(func=cmd_render_status)

    gs = sp.add_parser("gui-start", help="attach GPU-rendered Gazebo GUI to running simulation")
    gs.set_defaults(func=cmd_gui_start)

    gst = sp.add_parser("gui-status", help="show Gazebo GUI status")
    gst.set_defaults(func=cmd_gui_status)

    gx = sp.add_parser("gui-stop", help="stop only the Gazebo GUI")
    gx.set_defaults(func=cmd_gui_stop)

    rn = sp.add_parser("run", help="run a configured scenario")
    rn.add_argument("scenario")
    rn.add_argument("--headless", action="store_true", help="explicitly disable Gazebo GUI")
    rn.set_defaults(func=cmd_run)

    ls = sp.add_parser("scenarios", help="list discovered built-in and user scenarios")
    ls.set_defaults(func=cmd_scenarios)

    ns = sp.add_parser("new-scenario", help="create a user scenario template")
    ns.add_argument("name")
    ns.set_defaults(func=cmd_new_scenario)

    ml = sp.add_parser("model-list", help="list reviewed and configurable models")
    ml.set_defaults(func=cmd_model_list)

    mb = sp.add_parser("model-build", help="build and validate a configurable model")
    mb.add_argument("model")
    mb.set_defaults(func=cmd_model_build)

    def urdf_arguments(parser):
        parser.add_argument("source", help="self-contained .urdf or .zip")
        parser.add_argument("--urdf-entry", help="URDF path inside a ZIP with multiple robots")
        parser.add_argument("--root-xyz", nargs=3, type=float, default=(0., 0., 0.),
                            metavar=("X", "Y", "Z"), help="root-frame translation in metres")
        parser.add_argument("--root-rpy", nargs=3, type=float, default=(0., 0., 0.),
                            metavar=("ROLL", "PITCH", "YAW"), help="root-frame rotation in radians")
        parser.add_argument("--package", action="append", default=[], metavar="NAME=PATH",
                            help="explicit local package:// resource mapping; repeatable")

    mi = sp.add_parser("model-inspect", help="inspect URDF mass, joints, frames and local mesh resources")
    urdf_arguments(mi)
    mi.set_defaults(func=cmd_model_inspect)

    imp = sp.add_parser("model-import", help="convert a URDF/ZIP into a portable Gazebo model")
    urdf_arguments(imp)
    imp.add_argument("--name", required=True)
    imp.set_defaults(func=cmd_model_import)

    bind = sp.add_parser("model-bind-flight", help="bind explicit propulsion/PX4 parameters to an imported model")
    bind.add_argument("model")
    bind.add_argument("profile")
    bind.set_defaults(func=cmd_model_bind_flight)

    preview = sp.add_parser("model-preview", help="prepare an isolated Gazebo world; optionally run a bounded physics check")
    preview.add_argument("model")
    preview.add_argument("--height", type=float, default=.5)
    preview.add_argument("--static", action="store_true")
    preview.add_argument("--check", action="store_true")
    preview.add_argument("--iterations", type=int, default=100)
    preview.set_defaults(func=cmd_model_preview)

    hd = sp.add_parser("hil-doctor", help="probe a real PX4 MAVLink HIL connection")
    hd.add_argument("--mavlink", required=True, help="serial device or pymavlink connection URL")
    hd.add_argument("--baud", type=int, default=921600)
    hd.add_argument("--timeout", type=float, default=8.0)
    hd.add_argument("--require-hil", action="store_true")
    hd.set_defaults(func=cmd_hil_doctor)

    hs = sp.add_parser("hil-start", help="start Gazebo against a PX4 HIL MAVLink endpoint")
    hs.add_argument("--mavlink", required=True, help="serial device or pymavlink connection URL")
    hs.add_argument("--baud", type=int, default=921600)
    hs.add_argument("--model", default="x500_underfold")
    hs.add_argument("--xrce-serial", help="optional second serial link for Micro-XRCE-DDS")
    hs.add_argument("--xrce-baud", type=int, default=921600)
    hs.add_argument("--wait", type=float, default=60.0)
    hs.set_defaults(func=cmd_hil_start)

    hst = sp.add_parser("hil-status", help="show HIL runtime status")
    hst.set_defaults(func=cmd_hil_status)

    hsp = sp.add_parser("hil-stop", help="stop the owned HIL runtime")
    hsp.set_defaults(func=cmd_hil_stop)

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
