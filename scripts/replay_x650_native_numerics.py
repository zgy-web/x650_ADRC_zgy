"""Compare native/Python numerics on every returned call in a pinned real run.

This is an offline differential replay, never a replacement flight record.
Each call starts with the previously recorded plan and identical causal state.
The rejected 56 ms interval remains rejected; it is not shortened or replayed
as an accepted controller call.
"""
from __future__ import annotations
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys
import traceback
import numpy as np
from validate_x650_native_numerics import ROOT,PROTOCOL,pair,source_receipt,file_identity,plain
from qualification_recording import RowEvidenceStore, NaturalGcTrace, verify_streamed_evidence
from profile_x650_ts20_actual_step import ReplayTail
from x650_wholebody.native_optimization import NativeOptimization
from x650_wholebody.ts20_mpc import Ts20MpcController
from x650_wholebody.api import ControlState,EndpointTarget
from x650_wholebody.model import ArmModel
from x650_wholebody.ts20 import FrozenTs20Model
from x650_wholebody.ts20_prediction import Ts20PredictionModel
from x650_wholebody.ts20_runtime import Ts20Runtime


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--failure-evidence",type=Path,required=True)
    ap.add_argument("--failure-sha256",required=True)
    ap.add_argument("--native-library",type=Path,required=True)
    ap.add_argument("--native-sha256",required=True)
    ap.add_argument("--output",type=Path,required=True)
    args=ap.parse_args(argv)
    args.output.mkdir(parents=True,exist_ok=False)
    if file_identity(args.failure_evidence)["sha256"] != args.failure_sha256:
        raise ValueError("Historical failure identity differs")
    evidence=json.loads(args.failure_evidence.read_text());run=Path(evidence["run_directory"])
    names=("controller_metadata.json","controller_task.json","control_execution_trace.jsonl",
           "controller_diagnostics.jsonl","ts20_state.csv","ts20_packets.csv")
    identities={}
    for name in names:
        identity=file_identity(run/name)
        if identity["sha256"] != evidence["raw_artifacts"][str(run/name)]:
            raise ValueError("Recorded source changed: "+name)
        identities[name]=identity
    read=lambda p:json.loads(p.read_text())
    meta,cfg=read(run/names[0]),read(run/names[1])
    # Keep only fields used by replay. Loading every feedback/diagnostic graph
    # makes GC scan data that the real streaming runtime never retains.
    traces=[]
    for line in (run/names[2]).open():
        v=json.loads(line)
        if v.get("phase")=="measure" and v.get("event")=="controller_step_returned":
            traces.append({k:v[k] for k in ("t_s","dt_s","state","target")})
    diagnostics=[]
    for line in (run/names[3]).open():
        v=json.loads(line)
        if v.get("phase")=="measure":
            diagnostics.append({"t_s":v["t_s"],"diagnostics":{
                k:v["diagnostics"][k] for k in ("predicted_commands","node_intervals_s")}})
    assert len(traces)==len(diagnostics)>0
    inputs=cfg["ts20_inputs"];plant=read(Path(inputs["plant_manifest"]))["parameters"]
    source=FrozenTs20Model(inputs["source_dir"],read(Path(inputs["source_manifest"])))
    motion=source.metadata()["motion_model"]
    def predictor():
        p=Ts20PredictionModel(motion,kp=plant["kp"],kd=plant["kd"],bus_period_s=plant["bus_period_s"])
        p.enable_native(inputs["native_library"],expected_sha256=inputs["native_library_sha256"])
        return p
    model=ArmModel(**meta["physical_profile"])
    mpc_cfg=deepcopy(cfg["mpc_config"])
    mpc_cfg["response_model"]=meta["controller_details"]["allocator"]["response_model"]
    native=NativeOptimization(args.native_library,args.native_sha256)
    rt=Ts20Runtime(run/"ts20_state.csv",run/"ts20_packets.csv",predictor())
    rt.states.close();rt.packets.close()
    rt.states,rt.packets=ReplayTail(run/"ts20_state.csv","sim_s"),ReplayTail(run/"ts20_packets.csv","send_s")
    sources=source_receipt()
    sources[Path(__file__).relative_to(ROOT).as_posix()]=file_identity(__file__)["sha256"]
    report={"kind":"native_numerics_recorded_calls_v1","no_flight":True,"complete":False,
            "evidence_layout":"streamed_rows_v1",
            "inputs":identities,"historical_failure":file_identity(args.failure_evidence),
            "historical_failure_unchanged":True,"native_library":native.receipt,
            "deployment_sources":sources,"protocol":PROTOCOL,"rows":[]}
    sent=[(r["control_time_s"],r["command"]) for r in meta["activation_command_history"]]
    origin=meta["task_start_sim_s"]
    context,target=None,None
    # Like the live runtime, load each predictor/library once. Repeated CDLL
    # construction is not part of a control callback and creates loader cycles.
    pool=[]
    for _ in range(2):
        p=predictor()
        c=Ts20MpcController(model,meta["actual_pid_parameters"],predictor=p,
            context_provider=lambda s,t:deepcopy(context),trajectory=lambda t:target,config=mpc_cfg)
        pool.append((c,p))
    build_index=0
    store=RowEvidenceStore(args.output)
    gc_trace=NaturalGcTrace()
    gc_trace.__enter__()
    try:
        for index,(trace,dg) in enumerate(zip(traces,diagnostics)):
            gc_trace.begin_row(index)
            row={"index":index,"t_s":trace["t_s"],"dt_s":trace["dt_s"],"pass":False}
            try:
                assert abs(trace["t_s"]-dg["t_s"])<1e-9
                state,target=ControlState(**trace["state"]),EndpointTarget(**trace["target"])
                rt.states.until=state.joint_stamp_s;rt.packets.until=state.control_time_s
                observed,stamp=rt.measured_state(state)
                np.testing.assert_array_equal(observed.q,state.q)
                np.testing.assert_array_equal(observed.dq,state.dq)
                assert stamp==state.joint_stamp_s
                context=rt.context(state,trace["t_s"])
                def build():
                    nonlocal build_index
                    c,p=pool[build_index%2];build_index+=1
                    c.reset(meta["reference_base_enu_m"],meta["reference_rotation"],[0.,0.])
                    c.prime_command_history(sent,origin)
                    if index:
                        previous=diagnostics[index-1];pd=previous["diagnostics"]
                        c._plan=np.asarray(pd["predicted_commands"])
                        c._plan_t=float(previous["t_s"]);c._plan_h=np.asarray(pd["node_intervals_s"])
                        c.previous_command=c._plan[0].copy();c.last_t=c._plan_t
                        c._previous_stamp=traces[index-1]["state"]["stamp_s"]
                        c.step_count=index;c._chain_clock_origin=origin
                    return c,p,state,target
                row.update(pair(build,native,trace["t_s"],trace["dt_s"],gc_trace=gc_trace))
                row["pass"]=True
            except Exception as exc:
                row.update(error=repr(exc),traceback=traceback.format_exc())
            finally:
                row['natural_gc']=gc_trace.end_row()
            report["rows"].append(store.write(index,row))
            # The full packets/extrema evidence is durable; release this row.
            del row
            if index%25==0:
                print(json.dumps({"completed":index+1,"failures":sum(not v["pass"] for v in report["rows"])}),flush=True)
    finally:
        gc_trace.__exit__(None,None,None)
        rt.close()
    report['natural_gc']=gc_trace.write(args.output)
    report.update(complete=True,passed=sum(v["pass"] for v in report["rows"]),total=len(traces))
    report["pass"]=report["passed"]==len(traces)
    # Verify all actually loaded source files after the complete replay.
    report["sources_unchanged"]=all(file_identity(ROOT/p)["sha256"]==h for p,h in sources.items())
    try:
        report['row_evidence_verification']=verify_streamed_evidence(args.output,report)
    except Exception as exc:
        report['row_evidence_verification']=dict(passed=False,error=repr(exc),traceback=traceback.format_exc())
    report["pass"]=(report["pass"] and report["sources_unchanged"]
                    and report['natural_gc']['complete'] and report['natural_gc']['policy_unchanged']
                    and report['row_evidence_verification']['passed'])
    (args.output/"replay_report.json").write_text(json.dumps(plain(report),indent=2,allow_nan=False))
    print(json.dumps({"pass":report["pass"],"passed":report["passed"],"total":len(traces),
                      "failures":[v for v in report["rows"] if not v["pass"]][:5]},indent=2))
    return 0 if report["pass"] else 1


if __name__=="__main__":
    raise SystemExit(main())
