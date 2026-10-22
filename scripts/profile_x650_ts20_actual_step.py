"""Offline replay of the first failed real TS20 MPC call, never a flight.

Read the immutable run and exact execution trace. Prefix fixtures exclude
landing and future samples. Every fresh runtime owns its CSV dictionaries so
first context destruction is measured rather than hidden by fixture aliases.
"""
from __future__ import annotations

import argparse
import cProfile
import csv
import hashlib
import importlib.util
import inspect
import json
from pathlib import Path
from dataclasses import asdict, replace
import pstats
import shutil
import sys
from time import perf_counter
import traceback

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"control_packages/x650_wholebody_v1"))
TOOL_VERSION = "actual_first_step_offline_timing_v2"
HISTORICAL_SERVER_TOOL_SHA256 = "d00c07b17cecac8e8da3a30654148e5b639f7ab876d56fd6c7412085bbabb09f"
ACTIVE_OUTPUT = None


def read(path):
    return json.loads(Path(path).read_text())


def plain(value):
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, dict):
        return {k: plain(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [plain(v) for v in value]
    if isinstance(value, np.generic):
        return value.item()
    return value


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def require(condition, message):
    if not condition:
        raise ValueError(message)


def fresh_output(path):
    """Refuse even an existing empty directory; no historical file is reused."""
    path = Path(path).absolute()
    path.mkdir(parents=True, exist_ok=False)
    return path.resolve(strict=True)


def file_identity(path):
    path = Path(path).resolve(strict=True)
    return {"path": str(path), "sha256": sha(path), "bytes": path.stat().st_size}


def loaded_identity(output, runtime_class, historical_archives):
    modules = {name: file_identity(module.__file__) for name, module in sorted(sys.modules.items())
        if name.startswith("x650_wholebody.") and getattr(module, "__file__", None)}
    runtime = file_identity(inspect.getfile(runtime_class))
    harness = file_identity(__file__)
    archive = output/"executed_sources"
    archive.mkdir()
    for name, item in {"harness": harness, "loaded_runtime": runtime, **modules}.items():
        dest = archive/(name.replace(".", "_")+Path(item["path"]).suffix)
        shutil.copyfile(item["path"], dest)
        require(sha(dest) == item["sha256"], "Source changed while archiving "+item["path"])
        item["archive_path"] = str(dest)
    return {"tool_version": TOOL_VERSION, "harness": harness, "runtime": runtime,
        "loaded_modules": modules, "native_load_receipts": [],
        "python": {"executable": sys.executable, "version": sys.version,
                   "numpy_version": np.__version__, "numpy_path": np.__file__,
                   "scipy_version": sys.modules["scipy"].__version__,
                   "scipy_path": sys.modules["scipy"].__file__},
        "historical_server_execution": {
            "harness_sha256": HISTORICAL_SERVER_TOOL_SHA256,
            "report_reference": "docs/validation/ts20_prediction_chain_20261022/incremental_runtime_timing_summary.json",
            "source_archive_references": [file_identity(p) for p in historical_archives],
            "byte_equivalence_to_current_tool_claimed": False,
            "note": "Historical SHA/report are unchanged. An empty archive list means no historical source archive was supplied; current executed_sources contains only this run's sources."}}


def verify_identity(identity):
    entries = [identity["harness"], identity["runtime"], *identity["loaded_modules"].values()]
    for receipt in identity["native_load_receipts"]:
        entries.append(receipt)
    for item in entries:
        require(sha(item["path"]) == item["sha256"], "Loaded source/library changed during profiling: "+item["path"])


def prefix(source, dest, field, cutoff):
    require(np.isfinite(cutoff), "Finite prefix cutoff required")
    with source.open(newline="") as stream, dest.open("x", newline="") as out:
        rows = csv.DictReader(stream)
        writer = csv.DictWriter(out, rows.fieldnames)
        writer.writeheader()
        count = 0
        for row in rows:
            when = float(row[field])
            require(np.isfinite(when), "Nonfinite audit source timestamp")
            if when <= cutoff+1e-10:
                writer.writerow(row)
                count += 1
    return count


class ReplayTail:
    """Replay complete, already numeric audit rows with causal read batches."""
    def __init__(self, path, field):
        self.path = Path(path)
        self.stream = self.path.open(newline="")
        self.reader = iter(csv.DictReader(self.stream))
        self.pending = next(self.reader, None)
        self.field, self.until = field, -np.inf

    def read(self):
        rows = []
        while self.pending is not None and float(self.pending[self.field]) <= self.until+1e-10:
            row = {k: float(v) for k, v in self.pending.items()}
            require(np.isfinite(list(row.values())).all(), "Nonfinite audit row in incremental replay")
            rows.append(row)
            self.pending = next(self.reader, None)
        return rows

    def close(self):
        self.stream.close()


def main(argv=None):
    global ACTIVE_OUTPUT
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run-dir", type=Path, required=True)
    ap.add_argument("--failure-dir", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    ap.add_argument("--repeats", type=int, default=5)
    ap.add_argument("--profile", action="store_true")
    ap.add_argument("--runtime-module", type=Path)
    ap.add_argument("--incremental-observations", action="store_true")
    ap.add_argument("--historical-source-archive", type=Path, action="append", default=[],
                    help="Optional existing historical executed-source archive; identity only, never relabelled as current")
    args = ap.parse_args(argv)
    require(args.repeats > 0, "At least one fresh-runtime repeat is required")
    args.output = fresh_output(args.output)
    ACTIVE_OUTPUT = args.output
    (args.output/"invocation.json").write_text(json.dumps({"tool_version": TOOL_VERSION,
        "harness": file_identity(__file__), "argv": sys.argv if argv is None else argv,
        "complete": False, "not_a_stage_gate": True}, indent=2))
    # CLI validation and output protection work without SciPy. Heavy imports
    # happen only after a genuinely new evidence directory has been obtained.
    from x650_wholebody.api import ControlState, EndpointTarget
    from x650_wholebody.model import ArmModel
    from x650_wholebody.ts20 import FrozenTs20Model
    from x650_wholebody.ts20_prediction import Ts20PredictionModel
    from x650_wholebody.ts20_runtime import Ts20Runtime
    from x650_wholebody.ts20_mpc import Ts20MpcController
    runtime_class = Ts20Runtime
    if args.runtime_module:
        spec = importlib.util.spec_from_file_location("profile_runtime_candidate", args.runtime_module)
        require(spec is not None and spec.loader is not None, "Runtime override must be an importable Python source")
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        runtime_class = module.Ts20Runtime
    identity = loaded_identity(args.output, runtime_class, args.historical_source_archive)
    (args.output/"execution_identity.json").write_text(json.dumps(identity, indent=2))
    cfg = read(args.run_dir/"controller_task.json")
    require(cfg.get("stage") == "tip_hold" and cfg.get("trajectory") == "hover"
            and not cfg.get("disturbance_enabled", False), "This tool replays the quiet constant-target hold fixture only")
    meta_path = args.run_dir/"tip_hold.json"
    meta = read(meta_path)["metadata"]
    if isinstance(meta, str):
        meta_path = args.run_dir/meta
        meta = read(meta_path)
    traces = read(args.failure_dir/"measure_execution_trace.json")
    traces = [row for row in traces if row.get("event") == "controller_step_returned" and row.get("phase") == "measure"]
    require(bool(traces), "No measured controller-step record in the execution trace")
    trace = traces[0]
    evidence = read(args.failure_dir/"failure_evidence.json")
    state = ControlState(**trace["state"])
    target = EndpointTarget(**trace["target"])
    require(np.all(target.velocity == 0.) and np.all(target.acceleration == 0.),
            "Constant-target hold replay requires zero reference velocity and acceleration")
    inputs = cfg["ts20_inputs"]
    params = read(inputs["plant_manifest"])["parameters"]
    source = FrozenTs20Model(inputs["source_dir"], read(inputs["source_manifest"]))
    input_files = [args.run_dir/"ts20_state.csv", args.run_dir/"ts20_packets.csv",
        args.run_dir/"controller_task.json", meta_path, args.failure_dir/"measure_execution_trace.json",
        args.failure_dir/"failure_evidence.json", Path(inputs["plant_manifest"]), Path(inputs["source_manifest"]),
        Path(inputs["base_response_prior"])]
    input_identities = [file_identity(path) for path in input_files]
    def predictor():
        result = Ts20PredictionModel(source.metadata()["motion_model"],
            kp=params["kp"], kd=params["kd"], bus_period_s=params["bus_period_s"])
        result.enable_native(inputs["native_library"], expected_sha256=inputs["native_library_sha256"])
        receipt = result.metadata()["native_accelerator"]
        require(receipt is not None and result._native is not None, "Native propagation must actually be enabled")
        require(Path(result._native._name).resolve(strict=True) == Path(receipt["path"]).resolve(strict=True),
                "Native receipt does not describe the loaded ctypes library")
        identity["native_load_receipts"].append(dict(receipt))
        return result
    sfile, pfile = args.output/"state_prefix.csv", args.output/"packet_prefix.csv"
    counts = [prefix(args.run_dir/"ts20_state.csv", sfile, "sim_s", state.joint_stamp_s),
              prefix(args.run_dir/"ts20_packets.csv", pfile, "send_s", state.control_time_s)]
    expected = evidence["diagnostics"][0]["diagnostics"]
    expected_trace = evidence["diagnostics"][0]
    require(abs(float(expected_trace["t_s"])-float(trace["t_s"])) < 1e-10
            and abs(float(expected_trace["sim_s"])-state.control_time_s) < 1e-10,
            "Recorded diagnostic and first measured step have different clocks")
    def runtime():
        rt = runtime_class(sfile, pfile, predictor())
        rt._profile_max_buffers = [0, 0, 0]
        try:
            if args.incremental_observations:
                rt.states, rt.packets = ReplayTail(sfile, "sim_s"), ReplayTail(pfile, "send_s")
                require(rt.states.pending is not None, "Empty state prefix cannot initialize a measured replay")
                first = float(rt.states.pending["sim_s"])
                for now in np.r_[np.arange(first, state.control_time_s-1e-10, .02), state.control_time_s]:
                    rt.states.until = rt.packets.until = float(now)
                    rt.measured_state(replace(state, control_time_s=float(now)))
                    rt._profile_max_buffers = np.maximum(rt._profile_max_buffers,
                        [len(rt._rows), len(rt._packets), len(rt._command_events)]).tolist()
            observed, stamp = rt.measured_state(state)
            np.testing.assert_array_equal(observed.q, state.q)
            np.testing.assert_array_equal(observed.dq, state.dq)
            require(stamp == state.joint_stamp_s, "Runtime selected a different joint source stamp")
        except Exception:
            rt.close()
            raise
        return rt
    context_results, canonical_context = [], None
    for _ in range(args.repeats):
        rt = runtime()
        before = [len(rt._rows), len(rt._packets), len(rt._command_events)]
        try:
            at = perf_counter(); context = rt.context(state, trace["t_s"])
            elapsed = (perf_counter()-at)*1000
            if canonical_context is None:
                canonical_context = plain(context)
            require(canonical_context == plain(context), "Fresh runtime repeats produced different causal contexts")
            context_results.append({"ms": elapsed, "before": before,
                "incremental_max_buffers": rt._profile_max_buffers,
                "after": [len(rt._rows), len(rt._packets), len(rt._command_events)]})
        finally:
            rt.close()
    (args.output/"context_timing.json").write_text(json.dumps({"results": context_results,
        "context": canonical_context, "execution_identity": identity}, indent=2))
    mpc_cfg = dict(cfg["mpc_config"])
    mpc_cfg["response_model"] = meta["controller_details"]["allocator"]["response_model"]
    model = ArmModel(**meta["physical_profile"])
    def controller(rt):
        result = Ts20MpcController(model, meta["actual_pid_parameters"], predictor=rt.predictor,
            context_provider=rt.context, trajectory=lambda t: target, config=mpc_cfg)
        result.reset(meta["reference_base_enu_m"], meta["reference_rotation"], [0., 0.])
        result.prime_command_history([(r["control_time_s"], r["command"])
            for r in meta["activation_command_history"]], meta["task_start_sim_s"])
        return result
    results = []
    for index in range(args.repeats):
        rt = runtime()
        mpc = controller(rt)
        timings, captured = {}, []
        def instrument(obj, name):
            original = getattr(obj, name)
            def wrapped(*a, **kw):
                start = perf_counter()
                try:
                    out = original(*a, **kw)
                    if name == "_validate_candidate":
                        captured.append({"plan": a[3].copy(), "predicted": out[0].copy(), "checks": out[1]})
                    return out
                finally:
                    timings.setdefault(name, []).append((perf_counter()-start)*1000)
            setattr(obj, name, wrapped)
        for obj, names in ((rt, ["context"]), (mpc, ["_problem", "_horizon", "_joint_prediction", "_validate_candidate"]),
                           (rt.predictor, ["predict", "_future_packets", "_rollout_native", "candidate_rollout"])):
            for name in names:
                instrument(obj, name)
        mpc.context_provider = rt.context
        profiler = cProfile.Profile() if args.profile and index == 0 else None
        if profiler:
            profiler.enable()
        at, result = perf_counter(), None
        try:
            result = mpc.step(trace["t_s"], trace["dt_s"], state, target)
            status = {"returned": True}
        except Exception as exc:
            status = {"returned": False, "exception": repr(exc), "traceback": traceback.format_exc(),
                      "diagnostics": getattr(exc, "diagnostics", {})}
        finally:
            elapsed = (perf_counter()-at)*1000
            if profiler:
                profiler.disable(); profiler.dump_stats(str(args.output/"first_step.pstats"))
                with (args.output/"first_step_profile.txt").open("w") as stream:
                    pstats.Stats(profiler, stream=stream).sort_stats("cumulative").print_stats(50)
            rt.close()
        diag = result["diagnostics"] if result is not None else status["diagnostics"]
        differences = {key: float(np.max(np.abs(np.asarray(diag[key])-expected[key])))
            for key in ("predicted_commands", "predicted_states") if key in diag}
        checks = {key: diag.get(key) for key in ("ts20_candidate_q_linearization_error_rad",
            "ts20_candidate_dq_linearization_error_rad_s", "ts20_candidate_actual_bounds_passed",
            "ts20_nonlinear_check_passed", "solver")}
        results.append({"wall_ms": elapsed, "profiled": bool(profiler), "parts_ms": timings,
                        "recorded_output_peak_differences": differences, "checks": checks,
                        "status": status, "captured_candidates": captured})
    require(bool(identity["native_load_receipts"]), "No actual native load receipt was collected")
    verify_identity(identity)
    for item in input_identities:
        require(sha(item["path"]) == item["sha256"], "Recorded input changed during profiling: "+item["path"])
    (args.output/"execution_identity.json").write_text(json.dumps(identity, indent=2))
    report = {"kind": TOOL_VERSION, "complete": True, "run_dir": str(args.run_dir.resolve()),
        "execution_identity": identity, "all_steps_returned": all(row["status"]["returned"] for row in results),
        "instrumentation_not_a_deadline_qualification": True,
        "not_a_flight_or_realtime_qualification": True, "counts": counts,
        "replay_contract": {"reference_preview": "constant quiet hold target", "folded_reset_q0_rad": [0., 0.],
            "reset_anchor_scope": "original folded-home quiet-hold fixture, not arbitrary initial posture", "state": asdict(state),
            "target": asdict(target), "controller_task_time_s": trace["t_s"], "actual_dt_s": trace["dt_s"],
            "packet_prefix_rule": "all sends <= control clock; recorded-plan differences expose reconstruction ambiguity"},
        "external_model_metadata": source.metadata(),
        "source_stamp_s": state.joint_stamp_s, "control_time_s": state.control_time_s,
        "incremental_observations": args.incremental_observations,
        "runtime_source_path": identity["runtime"]["path"],
        "runtime_source_sha256": identity["runtime"]["sha256"],
        "context_results": context_results, "full_step_results": results,
        "context": canonical_context,
        "source_hashes": {item["path"]: item["sha256"] for item in input_identities}}
    (args.output/"profile_summary.json").write_text(json.dumps(plain(report), indent=2))
    print(json.dumps({"kind": TOOL_VERSION, "complete": True,
        "all_steps_returned": report["all_steps_returned"], "output": str(args.output),
        "first_step_ms": [row["wall_ms"] for row in results]}, indent=2))


if __name__ == "__main__":
    try:
        main()
    except Exception as exc:
        if ACTIVE_OUTPUT is not None:
            with (ACTIVE_OUTPUT/"failure_report.json").open("x") as stream:
                json.dump({"complete": False, "tool_version": TOOL_VERSION,
                    "harness": file_identity(__file__), "exception": repr(exc), "traceback": traceback.format_exc()}, stream, indent=2)
        raise
