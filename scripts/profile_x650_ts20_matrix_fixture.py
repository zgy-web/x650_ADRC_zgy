"""Profile the unchanged +5mm, age-zero, moving-warm TS20 software fixture.

Instrumentation overhead is reported and cannot qualify the 50ms gate. The
original 51.064ms failure is retained separately; this tool never edits it.
"""
import argparse
import cProfile
import hashlib
import json
from dataclasses import asdict
from pathlib import Path
import pstats
import shutil
import sys
from time import perf_counter
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/"control_packages/x650_wholebody_v1"),
               str(ROOT/"control_packages/x650_wholebody_v1/tests")]
TOOL_VERSION = "software_moving_warm_plus5mm_profile_v2"
ACTIVE_OUTPUT = None


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def file_identity(path):
    path = Path(path).resolve(strict=True)
    return {"path": str(path), "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
            "bytes": path.stat().st_size}


def fresh_output(path):
    path = Path(path).absolute()
    path.mkdir(parents=True, exist_ok=False)
    return path.resolve(strict=True)


def loaded_identity(output, predictor):
    modules = {name: file_identity(module.__file__) for name, module in sorted(sys.modules.items())
        if (name.startswith("x650_wholebody.") or name in
            {"test_ts20_mpc", "test_ts20_prediction", "test_mixed_mpc", "test_model"})
        and getattr(module, "__file__", None)}
    harness = file_identity(__file__)
    archive = output/"executed_sources"
    archive.mkdir()
    for name, item in {"harness": harness, **modules}.items():
        dest = archive/(name.replace(".", "_")+Path(item["path"]).suffix)
        shutil.copyfile(item["path"], dest)
        if hashlib.sha256(dest.read_bytes()).hexdigest() != item["sha256"]:
            raise ValueError("Source changed while archiving "+item["path"])
        item["archive_path"] = str(dest)
    native = predictor.metadata()["native_accelerator"]
    if (native is None or predictor._native is None
            or Path(predictor._native._name).resolve(strict=True) != Path(native["path"]).resolve(strict=True)):
        raise ValueError("Native receipt does not describe the actually loaded ctypes library")
    return {"tool_version": TOOL_VERSION, "harness": harness, "loaded_modules": modules,
        "runtime": None, "runtime_used": False,
        "runtime_scope": "static software-fixture context provider; no CSV runtime in this benchmark",
        "native_backend": dict(native),
        "python": {"executable": sys.executable, "version": sys.version,
            "numpy_version": np.__version__, "numpy_path": np.__file__,
            "scipy_version": sys.modules["scipy"].__version__, "scipy_path": sys.modules["scipy"].__file__},
        "historical_evidence_relabelled": False}


def verify_identity(identity):
    for item in [identity["harness"], identity["native_backend"], *identity["loaded_modules"].values()]:
        if hashlib.sha256(Path(item["path"]).read_bytes()).hexdigest() != item["sha256"]:
            raise ValueError("Loaded source/library changed during profiling: "+item["path"])


def fixture(library, expected_sha256=None):
    from test_ts20_mpc import setup
    from test_ts20_prediction import metadata
    from x650_wholebody.api import ControlState
    from x650_wholebody.ts20_prediction import Ts20PredictionModel
    from x650_wholebody.ts20 import quantize_half_away, POSITION_STEP_RAD, VELOCITY_STEP_RAD_S
    c, _, s, target = setup()
    c.nodes = 15
    p = Ts20PredictionModel(metadata(), kp=6., kd=.25)
    c.actuator_predictor = p
    # The CLI requires a caller-pinned identity. The retained fixture() helper
    # also supports exploratory callers, whose caller-pinning status is not a
    # gate claim; actual loaded bytes are always recorded by the CLI report.
    p.enable_native(library, expected_sha256=expected_sha256 or hashlib.sha256(library.read_bytes()).hexdigest())
    u, r = np.array([.05, -.04]), np.array([.013, -.013])
    packets = [{"send_time_s": float(t),
        "q_ref": quantize_half_away(r+t*u, POSITION_STEP_RAD),
        "dq_ref": quantize_half_away(u, VELOCITY_STEP_RAD_S),
        "tau_ff": np.zeros(2), "kp": np.full(2, 6.), "kd": np.full(2, .25)}
        for t in np.arange(-.025, 0., .005)]
    c.context_provider = lambda state, t: {"reference_state": {"time_s": 0., "r": r.copy()},
        "command_history": [(-.025, u.copy())], "packet_history": packets, "bus_origin_s": 0.}
    c.previous_command = np.r_[np.zeros(3), u]
    c._plan = np.tile(c.previous_command, (15, 1))
    c._plan_t, c._plan_h = -.02, np.r_[.02, np.full(14, .04)]
    state = ControlState(np.asarray(s.position)+[.005, 0., 0.], s.rotation,
        s.velocity, s.omega_body, s.q, u, s.stamp_s, acceleration_enu=s.acceleration_enu,
        control_time_s=s.control_time_s, joint_stamp_s=s.joint_stamp_s)
    return c, p, state, target


def main(argv=None):
    global ACTIVE_OUTPUT
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--native-library", type=Path, required=True)
    ap.add_argument("--native-sha256", required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--profile", action="store_true")
    args = ap.parse_args(argv)
    args.output = fresh_output(args.output)
    ACTIVE_OUTPUT = args.output
    (args.output/"invocation.json").write_text(json.dumps({"tool_version": TOOL_VERSION,
        "harness": file_identity(__file__), "argv": sys.argv if argv is None else argv,
        "complete": False, "not_a_stage_gate": True}, indent=2))
    from x650_wholebody import prediction_chain as chain
    c, p, state, target = fixture(args.native_library, args.native_sha256)
    identity = loaded_identity(args.output, p)
    (args.output/"execution_identity.json").write_text(json.dumps(identity, indent=2))
    controller_metadata = c.metadata()
    context = c.context_provider(state, 0.)
    timings, captured, originals = {}, [], []
    def instrument(obj, name):
        original = getattr(obj, name)
        def wrapped(*a, **kw):
            begin = perf_counter()
            try:
                out = original(*a, **kw)
                if name == "_validate_candidate":
                    captured.append({"plan": a[3].copy(), "predicted": out[0].copy(), "checks": out[1]})
                return out
            finally:
                timings.setdefault(name, []).append((perf_counter()-begin)*1000)
        setattr(obj, name, wrapped)
        originals.append((obj, name, original))
    for obj, names in ((c, ("_problem", "_horizon", "_joint_prediction", "_validate_candidate", "_quality")),
        (p, ("predict", "_future_packets", "_rollout_native", "candidate_rollout", "_event_jacobian", "_transition")),
        (chain, ("endpoint_outputs", "_geometry", "thrust_rotation"))):
        for name in names:
            instrument(obj, name)
    profiler = cProfile.Profile() if args.profile else None
    if profiler:
        profiler.enable()
    begin = perf_counter()
    try:
        result = c.step(0., .02, state, target)
        status = {"passed": True, "diagnostics": result["diagnostics"]}
    except Exception as exc:
        status = {"passed": False, "exception": repr(exc), "traceback": traceback.format_exc(),
                  "diagnostics": getattr(exc, "diagnostics", {})}
    wall_ms = (perf_counter()-begin)*1000
    if profiler:
        profiler.disable()
        profiler.dump_stats(str(args.output/"fixture.pstats"))
        with (args.output/"fixture_profile.txt").open("w") as stream:
            pstats.Stats(profiler, stream=stream).sort_stats("cumulative").print_stats(65)
    for obj, name, original in originals:
        setattr(obj, name, original)
    verify_identity(identity)
    report = {"kind": TOOL_VERSION, "complete": True, "profiled": bool(profiler),
        "execution_identity": identity, "native_identity_caller_pinned": True,
        "fixture": {"position_deviation_m": .005, "source_age_s": 0., "mode": "moving_warm",
            "nodes": 15, "first_dt_s": .02, "later_node_dt_s": .04, "task_t_s": 0.,
            "state": asdict(state), "target": asdict(target), "context": context,
            "controller_metadata": controller_metadata, "predictor_metadata": p.metadata()},
        "instrumentation_not_a_deadline_qualification": True, "wall_ms": wall_ms,
        "parts_ms": {k: {"calls": len(v), "total": sum(v), "values": v if len(v) < 10 else None}
            for k, v in timings.items()}, "status": status, "captured_candidates": captured,
        "source_sha256": {str(path): hashlib.sha256(path.read_bytes()).hexdigest()
            for path in (ROOT/"control_packages/x650_wholebody_v1/x650_wholebody/mpc.py",
                         ROOT/"control_packages/x650_wholebody_v1/x650_wholebody/ts20_mpc.py",
                         ROOT/"control_packages/x650_wholebody_v1/x650_wholebody/prediction_chain.py")}}
    (args.output/"profile_summary.json").write_text(json.dumps(plain(report), indent=2))
    print(json.dumps({"kind": TOOL_VERSION, "wall_ms": wall_ms, "passed": status["passed"],
                      "parts_ms": report["parts_ms"], "output": str(args.output)}, indent=2))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        if ACTIVE_OUTPUT is not None:
            with (ACTIVE_OUTPUT/"failure_report.json").open("x") as stream:
                json.dump({"complete": False, "tool_version": TOOL_VERSION,
                    "harness": file_identity(__file__), "exception": repr(exc), "traceback": traceback.format_exc()}, stream, indent=2)
        raise
