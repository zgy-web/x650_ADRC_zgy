"""Differential verification of explicit C++ numerics, with no simulator I/O.

The frozen matrix constructor is executed from its hash-pinned AST. Archived
controller sources and experiment results are never overwritten. Real-record
replay uses only consumed packets available at each recorded snapshot.
"""
from __future__ import annotations
import argparse
import ast
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import sys
from time import perf_counter
import traceback
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/"scripts"),str(ROOT/"control_packages/x650_wholebody_v1")]
from profile_x650_ts20_matrix_fixture import plain, file_identity
from x650_wholebody.native_mpc import NativeTs20MpcController
from x650_wholebody.native_optimization import NativeOptimization
from x650_wholebody.mpc import MpcController

PROTOCOL_PATH = ROOT/"control_packages/x650_wholebody_v1/native/validation_protocol.json"
PROTOCOL = json.loads(PROTOCOL_PATH.read_text())
MEMORY = ("_plan","_plan_t","_plan_h","previous_command","last_t","_previous_stamp",
          "step_count","_joint_history","_chain_clock_origin")


def frozen_constructor(path, expected):
    if file_identity(path)["sha256"] != expected:
        raise ValueError("Original frozen matrix script hash differs")
    tree = ast.parse(path.read_text(),filename=str(path))
    at = next(i for i,v in enumerate(tree.body) if isinstance(v,ast.For)
              and isinstance(v.target,ast.Name) and v.target.id == "spec")
    loop = tree.body[at]
    stop = next(i for i,v in enumerate(loop.body) if isinstance(v,ast.Assign)
                and any(isinstance(x,ast.Name) and x.id == "a" for x in v.targets))
    prefix = compile(ast.Module(body=tree.body[:at],type_ignores=[]),str(path),"exec")
    setup = compile(ast.Module(body=loop.body[:stop],type_ignores=[]),str(path),"exec")
    def build(spec=None):
        ns = {"__file__":str(path),"__name__":"_frozen_constructor"}
        exec(prefix,ns)
        if spec is None:
            return ns["fixtures"]
        ns["spec"] = spec; exec(setup,ns)
        return ns["c"],ns["p"],ns["shifted"],ns["target"]
    return build


def promote(c, native):
    # Verification only: start from byte-identical reference memory/config.
    c.__class__ = NativeTs20MpcController
    c.native_optimization = native
    return c


def matrices(c,native,t,dt,state,target):
    a = c._problem(t,dt,state,target)
    b = native.assemble(c,t,dt,state,target)
    errors = {}
    for key in ("A","b","H","g","C","lo","hi","offsets","jacobians",
                "linearization","rotations","first_lower","first_upper"):
        x,y = np.asarray(a[key]),np.asarray(b[key])
        errors[key] = float(np.max(abs(x-y)))
        np.testing.assert_allclose(y,x,rtol=PROTOCOL["matrix_relative_tolerance"],
                                   atol=PROTOCOL["matrix_absolute_tolerance"],err_msg=key)
    return errors


def pair(build,native,t=0.,dt=.02):
    c,p,state,target = build()
    nc,npredictor,ns,nt = build()
    errors = matrices(c,native,t,dt,state,target)
    promote(nc,native)
    captures = []
    for controller,predictor in ((c,p),(nc,npredictor)):
        cap = {"problems":[],"candidate_packets":[],"candidates":[]}
        old_problem,old_candidate = controller._problem,controller._validate_candidate
        overrides = {name:controller.__dict__.get(name) for name in ("_problem","_validate_candidate")}
        rollout_override = predictor.__dict__.get("candidate_rollout")
        old_rollout = predictor.candidate_rollout
        def problem(*args,_fn=old_problem,_cap=cap):
            out = _fn(*args); _cap["problems"].append(out); return out
        def candidate(*args,_fn=old_candidate,_cap=cap):
            try:
                out = _fn(*args);_cap["candidates"].append(True);return out
            except Exception:
                _cap["candidates"].append(False);raise
        def rollout(*args,_fn=old_rollout,_cap=cap):
            out = _fn(*args);_cap["candidate_packets"].append(plain(out["packets"]));return out
        controller._problem,controller._validate_candidate = problem,candidate
        predictor.candidate_rollout = rollout
        start = perf_counter()
        try:
            out = controller.step(t,dt,state,target)
        except Exception as exc:
            backend = "native" if controller is nc else "reference"
            detail = getattr(exc,"diagnostics",{})
            raise RuntimeError(backend+" instrumented call: "+repr(exc)+
                "; outer_ms="+str(detail.get("ts20_total_compute_ms"))) from exc
        finally:
            # Restore the instance dictionaries, not bound methods. Otherwise
            # controller -> wrapper -> bound method -> controller is a cycle
            # introduced by this verifier, not present in the real runtime.
            for name,previous in overrides.items():
                if previous is None:
                    controller.__dict__.pop(name,None)
                else:
                    setattr(controller,name,previous)
            if rollout_override is None:
                predictor.__dict__.pop("candidate_rollout",None)
            else:
                predictor.candidate_rollout = rollout_override
        cap.update(out=out,wall_ms=(perf_counter()-start)*1000)
        captures.append(cap)
    old,new = captures
    assert old["candidates"] == new["candidates"], "Candidate classification changed"
    assert old["candidate_packets"] == new["candidate_packets"], "Actual MIT packets changed"
    a,b = old["out"]["diagnostics"],new["out"]["diagnostics"]
    scale = np.tile(c.velocity_limit,c.nodes)
    plan_error = float(np.max(abs((np.asarray(a["predicted_commands"])-b["predicted_commands"]).ravel()/scale)))
    state_error = float(np.max(abs(np.asarray(a["predicted_states"])-b["predicted_states"])))
    objective_error = abs(a["objective"]-b["objective"])/(1+abs(a["objective"]))
    assert plan_error <= PROTOCOL["normalized_plan_peak_max"], ("plan",plan_error)
    assert state_error <= PROTOCOL["predicted_state_peak_max"], ("state",state_error)
    assert objective_error <= PROTOCOL["objective_relative_scale_max"], ("objective",objective_error)
    z = np.asarray(b["predicted_commands"]).ravel()/scale
    # Independent original full-row verifier; never trusts the native flags.
    quality = MpcController._quality(new["problems"][-1],z)
    assert quality["primal_violation"] <= PROTOCOL["original_full_constraint_primal_max"]
    assert quality["kkt_stationarity_inf"] <= PROTOCOL["original_full_constraint_stationarity_max"]
    return {"matrix_peak_errors":errors,"normalized_plan_peak":plan_error,
            "predicted_state_peak":state_error,"objective_relative_scale_error":objective_error,
            "candidate_and_mit_packets_equal":True,"independent_python_quality":quality,
            "reference_instrumented_ms":old["wall_ms"],"native_instrumented_ms":new["wall_ms"]}


def source_receipt():
    paths = [Path(__file__),PROTOCOL_PATH]
    paths += [ROOT/p for p in (
        "scripts/replay_x650_native_numerics.py",
        "simulations/x650_ts20_hold_v1/task.py",
        "simulations/x650_ts20_hold_v1/run_case.py",
        "ros2_ws/src/flyarm_mpc_reference/flyarm_mpc_reference/factory.py",
        "control_packages/x650_wholebody_v1/tests/test_native_optimization.py")]
    paths += sorted((ROOT/"control_packages/x650_wholebody_v1/x650_wholebody").glob("*.py"))
    paths += sorted(p for p in (ROOT/"control_packages/x650_wholebody_v1/native").rglob("*") if p.is_file())
    return {p.relative_to(ROOT).as_posix():file_identity(p)["sha256"] for p in paths}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--matrix-script",type=Path,required=True)
    ap.add_argument("--matrix-sha256",required=True)
    ap.add_argument("--native-library",type=Path,required=True)
    ap.add_argument("--native-sha256",required=True)
    ap.add_argument("--output",type=Path,required=True)
    args = ap.parse_args(argv)
    args.output.mkdir(parents=True,exist_ok=False)
    native = NativeOptimization(args.native_library,args.native_sha256)
    build = frozen_constructor(args.matrix_script,args.matrix_sha256)
    fixtures = build();assert len(fixtures)==PROTOCOL["frozen_matrix_case_count"]
    sources = source_receipt()
    report = {"kind":"native_numerics_matrix_v1","no_flight":True,"complete":False,
              "protocol":PROTOCOL,"deployment_sources":sources,
              "native_library":native.receipt,"original_matrix":file_identity(args.matrix_script),"rows":[]}
    for spec in fixtures:
        row = {"fixture":spec,"pass":False}
        try:
            row.update(pair(lambda:build(spec),native))
            # Fresh uninstrumented call; preserve the original deadline.
            c,p,s,t = build(spec);promote(c,native);start=perf_counter();out=c.step(0.,.02,s,t)
            row["native_uninstrumented_ms"] = (perf_counter()-start)*1000
            row["native_outer_ms"] = out["diagnostics"]["ts20_total_compute_ms"]
            assert row["native_uninstrumented_ms"] <= PROTOCOL["original_total_controller_deadline_ms"]
            row["pass"] = True
        except Exception as exc:
            row.update(error=repr(exc),traceback=traceback.format_exc())
        report["rows"].append(row)
    report.update(complete=True,passed=sum(x["pass"] for x in report["rows"]),total=len(fixtures),
                  sources_unchanged=sources==source_receipt())
    report["pass"] = report["passed"]==len(fixtures) and report["sources_unchanged"]
    (args.output/"matrix_report.json").write_text(json.dumps(plain(report),indent=2,allow_nan=False))
    print(json.dumps({"pass":report["pass"],"passed":report["passed"],"total":len(fixtures),
                      "failures":[x for x in report["rows"] if not x["pass"]]},indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
