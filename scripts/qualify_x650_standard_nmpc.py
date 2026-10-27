#!/usr/bin/env python3
"""Offline bound-fixture qualification; no ROS, physics or actuator publication.

The selected fixtures must be prepared before this script is run. Every selected
case is evaluated, including failures; no case is omitted because it is difficult.
"""
from __future__ import annotations

import argparse
from dataclasses import asdict
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import traceback

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT/"control_packages/x650_nmpc_standard_v1/python"))


def identity(path):
    path=Path(path).resolve()
    return dict(path=str(path),bytes=path.stat().st_size,sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def canonical_hash(value):
    return hashlib.sha256(json.dumps(value,sort_keys=True,separators=(",",":"),allow_nan=False).encode()).hexdigest()


def write_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix(path.suffix+".partial")
    temporary.write_text(json.dumps(value,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    temporary.replace(path)


def load_selections(paths):
    rows=[];config=None;config_sha=None;identities=[];seen=set()
    for path in paths:
        binding=identity(path);identities.append(binding)
        document=json.loads(Path(path).read_text(encoding="utf-8"))
        if document.get("kind")!="standard_nmpc_bound_fixture_selection_v1":
            raise ValueError("Expected a predeclared bound selection, not an arbitrary replay log")
        new_sha=canonical_hash(document["config"])
        if config_sha is not None and new_sha!=config_sha:
            raise ValueError("Selected cases must use one identical fixed configuration")
        config,config_sha=document["config"],new_sha
        for wrapped in document["cases"]:
            key=(wrapped["source_sha256"],int(wrapped["source_index"]))
            if key in seen: raise ValueError("Duplicate source case in qualification selection")
            seen.add(key)
            case=wrapped["case"]
            if not all(key in case for key in ("input","memory","oracle")):
                raise ValueError("Missing original StepInput/Memory/oracle binding")
            rows.append(dict(selection=binding,source=wrapped["source"],source_sha256=wrapped["source_sha256"],
                source_index=int(wrapped["source_index"]),case=case,case_sha256=canonical_hash(case)))
    if not rows: raise ValueError("No selected cases")
    return config,config_sha,rows,identities


def compact_solver(result):
    return {key:result[key] for key in ("backend","raw_status","solver_success","solve_elapsed_ms",
        "objective","numerically_feasible","first_order_kkt_passed","residuals",
        "multiple_shooting_dynamics_residual_inf","rollout_disagreement_inf","details")}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selection",action="append",required=True,type=Path)
    parser.add_argument("--output",required=True,type=Path)
    parser.add_argument("--max-iterations",type=int,default=100)
    parser.add_argument("--mode",choices=("all","ipopt_only"),default="all")
    parser.add_argument("--verbose-build",action="store_true")
    args=parser.parse_args(argv)
    import casadi as ca
    import numpy as np
    import scipy
    from x650_nmpc_standard.model import build_model,build_case
    from x650_nmpc_standard.solver import CondensedNlp,IpoptReference,AcadosSolver,SolverOptions,check_derivatives
    if ca.__version__!="3.7.2" or np.__version__!="1.26.4" or scipy.__version__!="1.13.1":
        raise ValueError("Qualification requires pinned CasADi 3.7.2/numpy 1.26.4/scipy 1.13.1")
    acados_source=Path(os.environ["ACADOS_SOURCE_DIR"]).resolve()
    acados_tag=subprocess.check_output(["git","-C",str(acados_source),"describe","--tags","--exact-match"],text=True).strip()
    if acados_tag!="v0.6.0": raise ValueError("Qualification requires exact acados v0.6.0 source tag")
    acados_commit=subprocess.check_output(["git","-C",str(acados_source),"rev-parse","HEAD"],text=True).strip()
    config,config_sha,rows,bindings=load_selections(args.selection)
    options=SolverOptions(max_iterations=args.max_iterations);options.validate()
    out=args.output.resolve();out.mkdir(parents=True,exist_ok=True)
    if (out/"report.json").exists(): raise ValueError("Use a fresh output directory; previous report is evidence")
    report=dict(kind="x650_standard_nmpc_offline_qualification_v1",complete=False,passed=False,
        physical_acceptance=False,online_acceptance=False,accepted_for_publication=False,
        configuration_sha256=config_sha,options=asdict(options),selections=bindings,
        versions=dict(casadi=ca.__version__,numpy=np.__version__,scipy=scipy.__version__,
                      acados=acados_tag,acados_commit=acados_commit),
        reference_objective_relative_tolerance=1e-5,mode=args.mode,selected_count=len(rows),cases=[],
        acados_hessian_approximation="GAUSS_NEWTON",acados_cost_type="NONLINEAR_LS",
        ipopt_and_independent_derivatives="exact automatic differentiation of the same nonlinear OCP",
        scope="Prespecified independent snapshots; one smooth OCP; no global-optimal or real-time claim",
        remaining=["Independent native event-model candidate audit","Generated C/C++ reproduction",
                   "Live clock, ROS interface and actual actuator acceptance","Fresh closed-loop experiments"])
    report["source_artifacts"]=[identity(path) for path in sorted(
        (ROOT/"control_packages/x650_nmpc_standard_v1/python/x650_nmpc_standard").glob("*.py"))]
    report["source_artifacts"].append(identity(Path(__file__)))
    write_json(out/"report.json",report)
    start=time.monotonic()
    def phase(name,**extra):
        print(json.dumps(dict(phase=name,elapsed_wall_s=time.monotonic()-start,**extra)),flush=True)
    phase("build_stage_model_begin")
    model=build_model(config)
    phase("build_stage_model_complete")
    first=build_case(config,rows[0]["case"])
    phase("build_condensed_graph_begin")
    graph=CondensedNlp(model,first)
    phase("build_condensed_graph_complete",nx=graph.nx,nu=graph.nu,np=graph.np,nh=graph.nh)
    phase("create_ipopt_begin")
    ipopt=IpoptReference(graph,options)
    phase("create_ipopt_complete")
    sqp=rti=None
    if args.mode=="all":
        phase("generate_build_acados_sqp_begin")
        sqp=AcadosSolver(graph,out/"generated_sqp",mode="SQP",options=options,verbose=args.verbose_build)
        phase("generate_build_acados_sqp_complete")
        phase("generate_build_acados_rti_begin")
        rti=AcadosSolver(graph,out/"generated_rti",mode="SQP_RTI",options=options,verbose=args.verbose_build)
        phase("generate_build_acados_rti_complete")
    for index,bound in enumerate(rows):
        case_out=out/f"case_{index:03d}";case_out.mkdir(exist_ok=True)
        row={key:value for key,value in bound.items() if key!="case"}
        row.update(index=index,case_id=bound["case"].get("id"),passed=False)
        phase("case_begin",index=index,case_id=row["case_id"])
        try:
            numerical=build_case(config,bound["case"])
            nlp=graph.with_case(numerical)
            row["numeric_case"]=nlp.dump_case_json(case_out/"numeric_case.json",
                source_identity={key:bound[key] for key in ("source","source_sha256","source_index","case_sha256")},
                config_sha256=config_sha)
            row["graph_sha256"]=nlp.graph_sha256;row["binding_sha256"]=nlp.binding_sha256
            row["derivatives_at_initial"]=check_derivatives(nlp,nlp.initial_plan,seed=650+index)
            ipopt.bind(nlp);reference=ipopt.solve()
            write_json(case_out/"ipopt.json",reference)
            row["ipopt"]=compact_solver(reference)
            row["derivatives_at_reference"]=check_derivatives(nlp,reference["plan"],seed=1650+index)
            reference_pass=reference["solver_success"] and reference["numerically_feasible"] and reference["first_order_kkt_passed"]
            row["reference_passed"]=bool(reference_pass)
            if sqp is not None:
                sqp.bind(nlp);candidate=sqp.solve()
                write_json(case_out/"acados_sqp.json",candidate)
                row["sqp"]=compact_solver(candidate)
                objective_gap=candidate["objective"]-reference["objective"]
                allowance=report["reference_objective_relative_tolerance"]*max(1.,abs(reference["objective"]))
                row["sqp_reference_comparison"]=dict(objective_gap=objective_gap,allowance=allowance,
                    not_worse_than_local_reference=bool(objective_gap<=allowance),
                    interpretation="IPOPT is a local numerical reference, not a proven global minimum")
                row["sqp_qualified"]=bool(candidate["solver_success"] and candidate["numerically_feasible"]
                    and candidate["first_order_kkt_passed"] and objective_gap<=allowance)
                rti.bind(nlp);single_step=rti.solve()
                write_json(case_out/"acados_rti.json",single_step)
                row["rti"]=compact_solver(single_step)
                row["rti_single_step_qualified"]=bool(single_step["solver_success"] and single_step["numerically_feasible"]
                    and single_step["first_order_kkt_passed"])
                row["rti_single_step_scope"]="Independent initial plan; no claim of converged RTI tracking or online acceptance"
            row["passed"]=bool(row["derivatives_at_initial"]["passed"] and row["derivatives_at_reference"]["passed"]
                and reference_pass and (sqp is None or row["sqp_qualified"]))
        except Exception as error:
            row["error"]=dict(type=type(error).__name__,message=str(error),traceback=traceback.format_exc())
        # Commit one report only after case work, while preserving every failed case.
        if canonical_hash(bound["case"])!=bound["case_sha256"]:
            raise RuntimeError("Qualification unexpectedly mutated an original fixture case")
        report["cases"].append(row);write_json(out/"report.json",report)
        print(json.dumps(dict(index=index,case_id=row["case_id"],passed=row["passed"],error=row.get("error",{}).get("message"))),flush=True)
    report["source_files_unchanged"]=all(identity(item["path"])["sha256"]==item["sha256"]
        for item in bindings+report["source_artifacts"])
    report["complete"]=len(report["cases"])==len(rows)
    report["passed"]=bool(report["complete"] and report["source_files_unchanged"] and all(row["passed"] for row in report["cases"]))
    report["elapsed_wall_s"]=time.monotonic()-start
    report["passed_count"]=sum(bool(row["passed"]) for row in report["cases"])
    report["rti_single_step_qualified_count"]=sum(bool(row.get("rti_single_step_qualified",False)) for row in report["cases"])
    write_json(out/"report.json",report)
    artifacts=[identity(path) for path in sorted(out.rglob("*.json")) if path.name!="delivery_manifest.json"]
    write_json(out/"delivery_manifest.json",dict(kind="x650_standard_nmpc_offline_manifest_v1",files=artifacts))
    print(json.dumps(dict(complete=report["complete"],passed=report["passed"],passed_count=report["passed_count"],selected_count=len(rows))))
    return 0 if report["passed"] else 2


if __name__=="__main__":
    raise SystemExit(main())
