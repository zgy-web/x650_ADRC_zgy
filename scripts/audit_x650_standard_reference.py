#!/usr/bin/env python3
"""Bind a full-dual reference audit to unmodified, completed solver evidence.

This is a supplement, not an overwrite of an earlier failed active-set audit.
It never reruns a solver, changes tolerances or promotes RTI/physical acceptance.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def identity(path):
    path=Path(path).resolve()
    return dict(path=str(path),bytes=path.stat().st_size,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest())


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--original",type=Path,required=True)
    parser.add_argument("--full-dual-audit",type=Path,required=True)
    parser.add_argument("--loader-audit",type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    args=parser.parse_args(argv)
    if args.output.exists(): raise ValueError("Use a new supplement path; preserve all earlier evidence")
    files=[args.original,args.full_dual_audit,args.loader_audit]
    old,new,loader=[json.loads(path.read_text(encoding="utf-8")) for path in files]
    for report in (old,new):
        if report.get("kind")!="x650_standard_nmpc_offline_qualification_v1" or not report.get("complete"):
            raise ValueError("Both numerical reports must be complete bound-fixture qualifications")
        if report.get("online_acceptance") or report.get("physical_acceptance"):
            raise ValueError("Offline numerical evidence cannot carry physical/online acceptance")
    if old["mode"]!="all" or new["mode"]!="ipopt_only":
        raise ValueError("Expected original complete all-solver run plus independent IPOPT audit")
    common_keys=("configuration_sha256","options","versions","selections","selected_count")
    if any(old[key]!=new[key] for key in common_keys):
        raise ValueError("Cannot replace mathematical configuration, tolerances, dependencies or selection")
    if len(old["cases"])!=len(new["cases"]): raise ValueError("Case count mismatch")
    rows=[]
    for previous,current in zip(old["cases"],new["cases"]):
        keys=("index","case_id","case_sha256","source_sha256","source_index","graph_sha256","binding_sha256")
        if any(previous[key]!=current[key] for key in keys):
            raise ValueError("Full-dual audit must bind exactly to the original nonlinear OCP case")
        index=previous["index"]
        previous_file=args.original.parent/f"case_{index:03d}"/"ipopt.json"
        current_file=args.full_dual_audit.parent/f"case_{index:03d}"/"ipopt.json"
        before=json.loads(previous_file.read_text(encoding="utf-8"))
        after=json.loads(current_file.read_text(encoding="utf-8"))
        plan_difference=float(np.max(abs(np.asarray(before["plan"])-np.asarray(after["plan"]))))
        state_difference=float(np.max(abs(np.asarray(before["states"])-np.asarray(after["states"]))))
        objective_difference=abs(before["objective"]-after["objective"])
        exact_result_preserved=plan_difference==0. and state_difference==0. and objective_difference==0.
        residuals=after["residuals"]
        independent_full_duals_present=("supplied_multipliers" in residuals and
            residuals.get("kkt_multiplier_source")=="supplied full signed multipliers; independently evaluated")
        row=dict(index=index,case_id=previous["case_id"],graph_sha256=previous["graph_sha256"],
            binding_sha256=previous["binding_sha256"],original_reference=identity(previous_file),
            repeated_reference=identity(current_file),plan_max_abs_difference=plan_difference,
            state_max_abs_difference=state_difference,objective_abs_difference=objective_difference,
            exact_solution_preserved=exact_result_preserved,
            prior_reference_passed=previous["reference_passed"],
            independently_checked_full_duals_present=independent_full_duals_present,
            reference_primal_violation=residuals["primal_violation"],
            reference_stationarity_inf=residuals["stationarity_inf"],
            reference_complementarity_inf=residuals["complementarity_inf"],
            reference_dual_violation=residuals["dual_nonnegativity_violation"],
            reference_full_dual_kkt_passed=after["first_order_kkt_passed"],
            original_sqp_qualified=previous["sqp_qualified"],
            original_rti_single_step_qualified=previous["rti_single_step_qualified"])
        row["passed"]=bool(exact_result_preserved and independent_full_duals_present and current["passed"]
            and previous["sqp_qualified"] and previous["derivatives_at_initial"]["passed"]
            and previous["derivatives_at_reference"]["passed"])
        rows.append(row)
    result=dict(kind="x650_standard_nmpc_reference_audit_supplement_v1",complete=True,
        passed=bool(old["source_files_unchanged"] and new["source_files_unchanged"] and
                    loader["passed"] and all(row["passed"] for row in rows)),
        physical_acceptance=False,online_acceptance=False,accepted_for_publication=False,
        original_report=identity(args.original),full_dual_report=identity(args.full_dual_audit),
        loader_audit=identity(args.loader_audit),auditor_source=identity(__file__),
        common_binding={key:old[key] for key in common_keys},cases=rows,
        original_reports_modified=False,original_full_audit_passed_count=old["passed_count"],
        supplement_passed_count=sum(row["passed"] for row in rows),
        rti_single_step_qualified_count=old["rti_single_step_qualified_count"],
        explanation="Original active-set audit is preserved. A separate identical IPOPT solve stores full "
                    "multipliers and independently evaluates every primal, stationarity, complementarity "
                    "and dual-sign condition. The supplement requires exactly identical plans/states/costs; "
                    "it does not relax numerical limits, qualify RTI, or authorize actuator publication.")
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(result,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    print(json.dumps({key:result[key] for key in ("passed","supplement_passed_count",
        "original_full_audit_passed_count","rti_single_step_qualified_count")}))
    return 0 if result["passed"] else 2


if __name__=="__main__":
    raise SystemExit(main())
