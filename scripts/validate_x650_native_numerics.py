"""Differential verification of explicit C++ numerics, with no simulator I/O.

The frozen matrix constructor is executed from its hash-pinned AST. Archived
controller sources and experiment results are never overwritten. Real-record
replay uses only consumed packets available at each recorded snapshot.
"""
from __future__ import annotations
import argparse
import ast
import builtins
from copy import deepcopy
from contextlib import nullcontext
import hashlib
import dis
import json
from pathlib import Path
import sys
from time import perf_counter
import traceback
from types import CodeType, FunctionType
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/"scripts"),str(ROOT/"control_packages/x650_wholebody_v1")]
from profile_x650_ts20_matrix_fixture import plain, file_identity
from packed_candidate_capture import CandidateExecutionCapture, compare_execution_records
from qualification_recording import (RowEvidenceStore, NaturalGcTrace, verify_streamed_evidence,
                                     collect_startup_garbage, STARTUP_TIMING_CONDITION)
from x650_wholebody.native_mpc import NativeTs20MpcController
from x650_wholebody.native_optimization import NativeOptimization
from x650_wholebody.native_joint_prediction import NativeJointMatrices
from x650_wholebody.mpc import MpcController

for _module in ('native_mpc', 'native_optimization', 'native_joint_prediction'):
    _loaded = Path(sys.modules['x650_wholebody.'+_module].__file__).resolve()
    if _loaded != (ROOT/'control_packages/x650_wholebody_v1/x650_wholebody'/(_module+'.py')).resolve():
        raise RuntimeError('Native qualification loaded the wrong production source: '+str(_loaded))

PROTOCOL_PATH = ROOT/"control_packages/x650_wholebody_v1/native/validation_protocol.json"
PROTOCOL = json.loads(PROTOCOL_PATH.read_text())
MEMORY = ("_plan","_plan_t","_plan_h","previous_command","last_t","_previous_stamp",
          "step_count","_joint_history","_chain_clock_origin")


def _release_fixture_bindings(ns):
    """Keep the frozen callbacks/code/data, remove only constructor roots.

    A callback's __globals__ is this exec namespace. Leaving ns['c'] or the
    callback's own name in it retains an otherwise finished fixture until GC.
    Fail closed if the pinned callback ever depends on a removed binding.
    """
    owned = {name: value for name, value in ns.items()
             if isinstance(value, FunctionType) and value.__globals__ is ns}
    if set(owned) != {'context', 'packet'}:
        raise ValueError('Frozen fixture local function structure changed')
    removed = {'c', 'p', 's', 'target', 'shifted', 'context', 'packet'}
    dynamic = {'globals', 'locals', 'eval', 'exec', 'vars', '__import__'}
    for function in owned.values():
        names, pending = set(), [function.__code__]
        while pending:
            code = pending.pop()
            names.update(i.argval for i in dis.get_instructions(code)
                         if i.opname in ('LOAD_GLOBAL', 'LOAD_NAME'))
            pending.extend(value for value in code.co_consts if isinstance(value, CodeType))
        aliases = any(ns.get(name) is getattr(builtins, key) for name in names for key in dynamic)
        if (function.__closure__ or function.__defaults__ or function.__kwdefaults__
                or names & (removed | dynamic) or aliases):
            raise ValueError('Frozen fixture callback depends on constructor roots or dynamic globals')
    if ns['c'].context_provider is not owned['context']:
        raise ValueError('Frozen fixture context provider changed')
    for name in removed: ns.pop(name, None)


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
        returned = False
        try:
            exec(prefix,ns)
            if spec is None:
                fixtures = ns['fixtures']; ns.clear(); return fixtures
            ns["spec"] = spec; exec(setup,ns)
            result = ns["c"],ns["p"],ns["shifted"],ns["target"]
            _release_fixture_bindings(ns)
            returned = True
            return result
        finally:
            # No controller is returned after setup/dependency-check failure.
            # Release this private namespace even when exec raised halfway.
            if not returned: ns.clear()
    return build


def promote(c, native):
    # Verification only: start from byte-identical reference memory/config.
    c.__class__ = NativeTs20MpcController
    c.native_optimization = native
    if not isinstance(c.actuator_predictor,NativeJointMatrices):
        c.actuator_predictor = NativeJointMatrices(c.actuator_predictor,native)
    return c


def matrices(c,native,t,dt,state,target):
    a = c._problem(t,dt,state,target)
    previous, previous_class = c.actuator_predictor, c.__class__
    marker = object()
    previous_native = c.__dict__.get("native_optimization", marker)
    try:
        # Exercise the actual private packed entry, not reference.predict.
        promote(c, native)
        b = c._problem(t,dt,state,target)
        assert b["joint_prediction"].get("_native_packed") is True
    finally:
        c.actuator_predictor, c.__class__ = previous, previous_class
        if previous_native is marker:
            c.__dict__.pop("native_optimization", None)
        else:
            c.native_optimization = previous_native
    errors = {}
    for key in ("A","b","H","g","C","lo","hi","offsets","jacobians",
                "linearization","rotations","first_lower","first_upper"):
        x,y = np.asarray(a[key]),np.asarray(b[key])
        errors[key] = float(np.max(abs(x-y)))
        np.testing.assert_allclose(y,x,rtol=PROTOCOL["matrix_relative_tolerance"],
                                   atol=PROTOCOL["matrix_absolute_tolerance"],err_msg=key)
    return errors


def pair(build,native,t=0.,dt=.02,*,gc_trace=None):
    c,p,state,target = build()
    nc,npredictor,ns,nt = build()
    errors = matrices(c,native,t,dt,state,target)
    promote(nc,native)
    captures = []
    for controller,predictor in ((c,p),(nc,npredictor)):
        predictor = controller.actuator_predictor
        cap = {"problems":[],"candidate_packets":[],"candidates":[]}
        execution = CandidateExecutionCapture(predictor)
        old_problem,old_candidate = controller._problem,controller._validate_candidate
        overrides = {name:controller.__dict__.get(name) for name in ("_problem","_validate_candidate")}
        def problem(*args,_fn=old_problem,_cap=cap):
            out = _fn(*args); _cap["problems"].append(out); return out
        def candidate(*args,_fn=old_candidate,_cap=cap):
            try:
                out = _fn(*args);_cap["candidates"].append(True);return out
            except Exception:
                _cap["candidates"].append(False);raise
        controller._problem,controller._validate_candidate = problem,candidate
        start = perf_counter()
        backend = "native" if controller is nc else "reference"
        try:
            with execution, (gc_trace.step(backend) if gc_trace is not None else nullcontext()):
                call_start = perf_counter()
                try:
                    out = controller.step(t,dt,state,target)
                finally:
                    call_end = perf_counter()
        except Exception as exc:
            detail = getattr(exc,"diagnostics",{})
            raise RuntimeError(backend+" instrumented call: "+repr(exc)+
                "; outer_ms="+str(detail.get("ts20_total_compute_ms"))+
                "; actual_step_wall_ms="+str((call_end-call_start)*1000)) from exc
        finally:
            # Restore the instance dictionaries, not bound methods. Otherwise
            # controller -> wrapper -> bound method -> controller is a cycle
            # introduced by this verifier, not present in the real runtime.
            for name,previous in overrides.items():
                if previous is None:
                    controller.__dict__.pop(name,None)
                else:
                    setattr(controller,name,previous)
        cap["candidate_packets"] = [plain(record["inputs"]["packets"]) for record in execution.records]
        cap["candidate_executions"] = execution.records
        cap.update(out=out,wall_ms=(call_end-call_start)*1000,
                   observation_total_ms=(perf_counter()-start)*1000)
        captures.append(cap)
    old,new = captures
    assert old["candidates"] == new["candidates"], "Candidate classification changed"
    assert old["candidate_packets"] == new["candidate_packets"], "Actual MIT packets changed"
    execution_evidence = compare_execution_records(old["candidate_executions"], new["candidate_executions"])
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
            "candidate_execution_evidence":execution_evidence,
            "reference_instrumented_ms":old["wall_ms"],"native_instrumented_ms":new["wall_ms"],
            "reference_controller_gate_ms":a['ts20_total_compute_ms'],
            "native_controller_gate_ms":b['ts20_total_compute_ms'],
            "reference_observation_total_ms":old['observation_total_ms'],
            "native_observation_total_ms":new['observation_total_ms'],
            "timing_semantics":"instrumented_ms measures only controller.step including boundary copies; "
                "controller_gate_ms is the untouched controller diagnostic; observation_total_ms also "
                "includes hook entry/exit, post-step packet expansion and plain conversion"}


def source_receipt():
    paths = [Path(__file__),PROTOCOL_PATH]
    paths += [ROOT/p for p in (
        "scripts/replay_x650_native_numerics.py",
        "scripts/packed_candidate_capture.py",
        "scripts/qualification_recording.py",
        "control_packages/x650_wholebody_v1/tests/test_native_packed_preparation.py",
        "control_packages/x650_wholebody_v1/tests/test_packed_candidate_capture.py",
        "control_packages/x650_wholebody_v1/tests/test_qualification_recording.py",
        "control_packages/x650_wholebody_v1/tests/test_frozen_fixture_lifecycle.py",
        "simulations/x650_ts20_hold_v1/task.py",
        "simulations/x650_ts20_hold_v1/run_case.py",
        "ros2_ws/src/flyarm_mpc_reference/flyarm_mpc_reference/factory.py",
        "control_packages/x650_wholebody_v1/tests/test_native_optimization.py")]
    paths += sorted((ROOT/"control_packages/x650_wholebody_v1/x650_wholebody").glob("*.py"))
    paths += sorted(p for p in (ROOT/"control_packages/x650_wholebody_v1/native").rglob("*") if p.is_file())
    return {p.relative_to(ROOT).as_posix():file_identity(p)["sha256"] for p in paths}


def prepare_matrix_startup(build, spec, native, output):
    # Complete frozen setup imports and native wrapper construction, without
    # _problem/predict/step or a warmed plan. Drop this preparation-only fixture.
    prepared = build(spec)
    before = prepared[0].step_count
    promote(prepared[0], native)
    after = prepared[0].step_count
    del prepared
    preparation = dict(fixture_index=0,fixture=plain(spec),actions=['constructor','promote'],
                       controller_steps_before=before,controller_steps_after=after,
                       discarded_before_collection=True)
    return collect_startup_garbage(output,preparation)


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
              "evidence_layout":"streamed_rows_v1",
              "protocol":PROTOCOL,"deployment_sources":sources,
              "native_library":native.receipt,"original_matrix":file_identity(args.matrix_script),"rows":[]}
    store = RowEvidenceStore(args.output)
    report['startup_timing_condition'] = STARTUP_TIMING_CONDITION
    report['startup_gc'] = prepare_matrix_startup(build, fixtures[0], native, args.output)
    with NaturalGcTrace() as gc_trace:
        for index,spec in enumerate(fixtures):
            gc_trace.begin_row(index)
            row = {"index":index,"fixture":spec,"pass":False}
            try:
                row.update(pair(lambda:build(spec),native,gc_trace=gc_trace))
                # Fresh uninstrumented call; preserve the original deadline.
                c,p,s,t = build(spec);promote(c,native);start=perf_counter()
                with gc_trace.step('native_uninstrumented'):
                    out=c.step(0.,.02,s,t)
                row["native_uninstrumented_ms"] = (perf_counter()-start)*1000
                row["native_outer_ms"] = out["diagnostics"]["ts20_total_compute_ms"]
                assert row["native_uninstrumented_ms"] <= PROTOCOL["original_total_controller_deadline_ms"]
                row["pass"] = True
            except Exception as exc:
                row.update(error=repr(exc),traceback=traceback.format_exc())
            finally:
                row['natural_gc'] = gc_trace.end_row()
            report["rows"].append(store.write(index,row))
            # Do not keep 30 full candidate evidence graphs until run end.
            del row
    report['natural_gc'] = gc_trace.write(args.output)
    report.update(complete=True,passed=sum(x["pass"] for x in report["rows"]),total=len(fixtures),
                  sources_unchanged=sources==source_receipt())
    try:
        report['row_evidence_verification'] = verify_streamed_evidence(args.output,report)
    except Exception as exc:
        report['row_evidence_verification'] = dict(passed=False,error=repr(exc),traceback=traceback.format_exc())
    report["pass"] = (report["passed"]==len(fixtures) and report["sources_unchanged"]
                      and report['natural_gc']['complete'] and report['natural_gc']['policy_unchanged']
                      and report['row_evidence_verification']['passed'])
    (args.output/"matrix_report.json").write_text(json.dumps(plain(report),indent=2,allow_nan=False))
    print(json.dumps({"pass":report["pass"],"passed":report["passed"],"total":len(fixtures),
                      "failures":[x for x in report["rows"] if not x["pass"]]},indent=2))
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
