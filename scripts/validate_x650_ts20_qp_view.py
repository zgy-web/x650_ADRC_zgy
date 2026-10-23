"""Compare TS20 QP views and rerun the original frozen 30-case script.

Original script bytes and historical results are immutable. The comparison
uses its exact constructor AST; the full-matrix run executes every original
statement and redirects only its one result write into a fresh directory.
"""
import argparse
import ast
from contextlib import redirect_stdout
from dataclasses import asdict
import hashlib
import io
import json
from pathlib import Path
import runpy
import shutil
import sys
from time import perf_counter
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/"scripts"), str(ROOT/"control_packages/x650_wholebody_v1")]
from profile_x650_ts20_matrix_fixture import fresh_output, file_identity, plain, loaded_identity, verify_identity

PROTOCOL = {"normalized_plan_peak_max": 1e-7, "predicted_state_peak_max": 1e-7,
            "objective_error_max": "1e-8*(1+abs(full_original_objective))",
            "nonlinear_packets_and_forecast_required_exact": True,
            "original_controller_timing_limit_ms": 50., "full_matrix_cases": 30}
WORST = {"position_deviation_m": .005, "source_age_s": 0., "mode": "moving_warm"}


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def constructor(script):
    """Use the original setup statements, without running or editing its loop."""
    tree = ast.parse(script.read_text(), filename=str(script))
    loop_index = next(i for i, node in enumerate(tree.body)
        if isinstance(node, ast.For) and isinstance(node.target, ast.Name) and node.target.id == "spec")
    loop = tree.body[loop_index]
    stop = next(i for i, node in enumerate(loop.body) if isinstance(node, ast.Assign)
        and any(isinstance(target, ast.Name) and target.id == "a" for target in node.targets))
    prefix = compile(ast.Module(body=tree.body[:loop_index], type_ignores=[]), str(script), "exec")
    setup = compile(ast.Module(body=loop.body[:stop], type_ignores=[]), str(script), "exec")
    def build():
        namespace = {"__file__": str(script), "__name__": "_frozen_matrix_constructor"}
        exec(prefix, namespace)
        choices = [spec for spec in namespace["fixtures"] if spec == WORST]
        if len(choices) != 1:
            raise ValueError("Frozen matrix does not contain exactly the original failed fixture")
        namespace["spec"] = choices[0]
        exec(setup, namespace)
        return namespace["c"], namespace["p"], namespace["shifted"], namespace["target"], namespace["fixtures"]
    return build


def select_view(controller, view):
    if view == "full":
        from x650_wholebody.mpc import MpcController
        controller._solver_constraint_view = MpcController._solver_constraint_view


def equivalent_run(build, view):
    c, p, state, target, _ = build()
    select_view(c, view)
    captured = {"view": view, "state": asdict(state), "target": asdict(target)}
    originals = {name: getattr(c, name) for name in ("_problem", "_quality", "_validate_candidate")}
    original_rollout = p.candidate_rollout
    def problem(*args, **kw):
        out = originals["_problem"](*args, **kw); captured["problem"] = out; return out
    def quality(problem, z):
        out = originals["_quality"](problem, z)
        captured.update(z=z.copy(), quality=out)
        return out
    def validate(*args, **kw):
        out = originals["_validate_candidate"](*args, **kw)
        captured.update(plan=args[3].copy(), predicted=out[0].copy(), nonlinear_checks=out[1])
        return out
    def rollout(*args, **kw):
        out = original_rollout(*args, **kw); captured["nonlinear"] = out; return out
    c._problem, c._quality, c._validate_candidate = problem, quality, validate
    p.candidate_rollout = rollout
    start = perf_counter()
    try:
        out = c.step(0., .02, state, target)
        captured["step_status"] = {"returned": True, "diagnostics": out["diagnostics"]}
    except Exception as exc:
        captured["step_status"] = {"returned": False, "exception": repr(exc),
                                   "diagnostics": getattr(exc, "diagnostics", {})}
    captured["instrumented_wall_ms"] = (perf_counter()-start)*1000
    if not all(name in captured for name in ("z", "plan", "predicted", "nonlinear")):
        raise RuntimeError("No complete candidate/quality for equivalence: "+str(captured["step_status"]))
    problem = captured.pop("problem")
    captured["problem_arrays"] = {key: problem[key] for key in ("H", "g", "A", "b", "C", "lo", "hi")}
    captured["objective"] = float(np.linalg.norm(problem["A"]@captured["z"]-problem["b"])**2)
    captured["quadratic_objective_without_constant"] = float(.5*captured["z"]@problem["H"]@captured["z"]+problem["g"]@captured["z"])
    C = np.vstack((problem["C"], problem["identity"]))
    lo, hi = np.r_[problem["lo"], -np.ones(len(captured["z"]))], np.r_[problem["hi"], np.ones(len(captured["z"]))]
    captured["constraint_view"] = c._solver_constraint_view(C, lo, hi)[3]
    captured["original_quality_passed"] = bool(captured["quality"]["primal_violation"] <= c.feasibility_tolerance
        and captured["quality"]["kkt_stationarity_inf"] <= c.kkt_tolerance)
    return captured


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--matrix-script", type=Path, required=True)
    ap.add_argument("--matrix-sha256", required=True)
    ap.add_argument("--historical-failure", type=Path, required=True)
    ap.add_argument("--historical-sha256", required=True)
    ap.add_argument("--native-sha256", required=True)
    ap.add_argument("--original-result", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    out = fresh_output(args.output)
    if digest(args.matrix_script) != args.matrix_sha256 or digest(args.historical_failure) != args.historical_sha256:
        raise ValueError("Original matrix/history identity mismatch")
    history = json.loads(args.historical_failure.read_text())
    build = constructor(args.matrix_script)
    c, p, state, target, fixtures = build()
    if len(fixtures) != 30 or fixtures != history["fixtures"]:
        raise ValueError("Original 30 fixtures differ from preserved historical run")
    identity = loaded_identity(out, p)
    if identity["native_backend"]["sha256"] != args.native_sha256:
        raise ValueError("Loaded native library differs from frozen identity")
    identity["acceptance_harness"] = file_identity(__file__)
    shutil.copyfile(__file__, out/"executed_sources/validate_x650_ts20_qp_view.py")
    provenance = {"execution_identity": identity, "matrix": file_identity(args.matrix_script),
        "historical_failure": file_identity(args.historical_failure), "protocol": PROTOCOL,
        "fixture": WORST, "fixtures": fixtures, "runtime_used": False,
        "scope": "software only; no flights; original controller deadline retained"}
    (out/"protocol_and_identity.json").write_text(json.dumps(plain(provenance), indent=2))
    full, view = equivalent_run(build, "full"), equivalent_run(build, "view")
    array_exact = {key: bool(np.array_equal(full["problem_arrays"][key], view["problem_arrays"][key])) for key in full["problem_arrays"]}
    delta = {"normalized_plan_peak": float(np.max(abs(full["z"]-view["z"]))),
        "predicted_state_peak": float(np.max(abs(full["predicted"]-view["predicted"]))),
        "objective_error": abs(full["objective"]-view["objective"]),
        "quadratic_objective_error": abs(full["quadratic_objective_without_constant"]-view["quadratic_objective_without_constant"]),
        "nonlinear_q_peak": float(np.max(abs(full["nonlinear"]["q"]-view["nonlinear"]["q"]))),
        "nonlinear_dq_peak": float(np.max(abs(full["nonlinear"]["dq"]-view["nonlinear"]["dq"]))) }
    packets_exact = plain(full["nonlinear"]["packets"]) == plain(view["nonlinear"]["packets"])
    passed = (all(array_exact.values()) and delta["normalized_plan_peak"] <= 1e-7
        and delta["predicted_state_peak"] <= 1e-7 and delta["objective_error"] <= 1e-8*(1+abs(full["objective"]))
        and packets_exact and delta["nonlinear_q_peak"] == 0. and delta["nonlinear_dq_peak"] == 0.
        and all(row["original_quality_passed"] and row["nonlinear_checks"]["ts20_nonlinear_check_passed"]
                and row["nonlinear_checks"]["ts20_candidate_actual_bounds_passed"] for row in (full, view)))
    equivalence = {"passed": passed, "problem_arrays_exact": array_exact, "differences": delta,
                   "encoded_packets_exact": packets_exact, "full": full, "view": view}
    (out/"equivalence.json").write_text(json.dumps(plain(equivalence), indent=2))
    timings = []
    for selection in ("full", "view", "view", "full", "full", "view"):
        c, _, state, target, _ = build();select_view(c, selection);start = perf_counter()
        try:
            result = c.step(0., .02, state, target)
            row = {"view": selection, "passed": True, "wall_ms": (perf_counter()-start)*1000,
                   "compute_ms": result["diagnostics"]["compute_ms"], "solve_ms": result["diagnostics"]["solve_ms"]}
        except Exception as exc:
            row = {"view": selection, "passed": False, "wall_ms": (perf_counter()-start)*1000,
                   "exception": repr(exc), "diagnostics": getattr(exc, "diagnostics", {})}
        timings.append(row)
    (out/"independent_timing.json").write_text(json.dumps(plain(timings), indent=2))
    old_result_identity = file_identity(args.original_result)
    original_write = Path.write_text
    def redirected_write(path, content, *a, **kw):
        if path.resolve() != args.original_result.resolve():
            raise RuntimeError("Unexpected original-script write: "+str(path))
        with (out/"matrix_result.json").open("x") as stream: return stream.write(content)
    log = io.StringIO()
    with patch.object(Path, "write_text", redirected_write), redirect_stdout(log):
        runpy.run_path(str(args.matrix_script), run_name="__main__")
    original_write(out/"matrix_stdout.txt", log.getvalue())
    if file_identity(args.original_result) != old_result_identity or digest(args.historical_failure) != args.historical_sha256:
        raise ValueError("Historical results were changed")
    matrix = json.loads((out/"matrix_result.json").read_text())
    if matrix["fixtures"] != fixtures or len(matrix["rows"]) != 30:
        raise ValueError("Full original matrix did not run unchanged")
    verify_identity(identity)
    summary = {"kind": "ts20_qp_view_software_acceptance_v1", "equivalence_passed": passed,
        "differences": delta, "constraint_view": view["constraint_view"], "independent_timing": timings,
        "matrix_total": 30, "matrix_passed": sum(row["pass"] for row in matrix["rows"]),
        "matrix_max_wall_ms": max(row["wall_ms"] for row in matrix["rows"]),
        "matrix_failures": [row for row in matrix["rows"] if not row["pass"]],
        "original_failure_preserved": True, "original_script_sha256": args.matrix_sha256,
        "hardware_or_flight_acceptance": False, "protocol": PROTOCOL, "execution_identity": identity}
    (out/"validation_summary.json").write_text(json.dumps(plain(summary), indent=2))
    print(json.dumps({k:v for k,v in summary.items() if k != "execution_identity"}, indent=2))


if __name__ == "__main__":
    main()
