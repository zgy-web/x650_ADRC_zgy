#!/usr/bin/env python3
"""Conditionally replay accepted MPC commands through TS20, with body held to logs.

No Gazebo flight, new MPC solve, hardware I/O, or physical torque calibration.
"""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/"control_packages/x650_wholebody_v1"), str(ROOT/"scripts")]
from x650_wholebody.ts20 import FrozenTs20Model, build_mit_commands
import plot_x650_mixed_stage_comparison as accepted
from render_x650_arm_replay import tip_from_report


def write_json(path, obj):
    Path(path).write_text(json.dumps(obj, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def metrics(t, error):
    integrate = getattr(np, "trapezoid", np.trapz)
    e = np.asarray(error)
    if e.ndim == 1: e=e[:,None]
    norm = np.linalg.norm(e, axis=1)
    return dict(dimension=e.shape[1], rms_norm=float(np.sqrt(integrate(norm**2,t)/(t[-1]-t[0]))),
                peak_norm=float(norm.max()), axis_rms=np.sqrt(integrate(e**2,t,axis=0)/(t[-1]-t[0])).tolist())


def trials():
    t = np.arange(0, 1.0001, .005)
    return t, np.tile([.2, -.2], (len(t),1))


def smoke(model, out):
    t, v = trials()
    bundle=build_mit_commands(t,v,[0,0])
    results={}
    for mode in ("pure_velocity", "integrated_position_and_velocity"):
        columns=[]
        for j in range(2):
            command={k:np.asarray(a)[:,j].copy() for k,a in bundle["command"].items()}
            if mode=="pure_velocity":
                command["kp"][:]=0;command["q_ref"][:]=0
            trial=dict(input_time_s=bundle["input_time_s"],observation_time_s=t,
                       initial_state=[0.,0.],initial_time_s=0.,command=command)
            r=model.simulate_motion(trial,max_seconds=30,max_rhs=1000000)
            columns.append(r["prediction"])
        a=np.stack(columns,axis=1)
        results[mode]={"final_q":a[-1,:,0].tolist(),"final_dq":a[-1,:,1].tolist(),
                       "max_abs_q":np.max(abs(a[:,:,0]),axis=0).tolist()}
    assert max(results["pure_velocity"]["max_abs_q"])==0
    assert min(results["integrated_position_and_velocity"]["max_abs_q"])>.10
    result={"complete":True,"kind":"synthetic_interface_smoke_only","hardware_validation":False,
            "command_velocity_rad_s":[.2,-.2],"duration_s":1.,"results":results}
    write_json(out/"interface_smoke.json",result)
    return result


def run_case(args, model, stage):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    original=args.accepted_root/("mpc_"+stage+"_native_replay")/"native_replay_manifest.json"
    binding=json.loads(original.read_text())
    run=accepted.load_run(binding["source_audit"],binding["source_run_directory"],"mpc")
    data=run["physical"];t=data["t"];duration=float(t[-1])
    q,dq=data["q"],data["dq"]
    ct=np.r_[0.,run["control_t"]]
    cmd=np.vstack((dq[0],run["joint_command"]))
    # The first real receipt can be at zero. Keep the command at that time.
    keep=np.r_[np.diff(ct)>1e-9,True]
    ct,cmd=ct[keep],cmd[keep]
    if ct[-1]<duration-1e-9:
        ct=np.r_[ct,duration];cmd=np.vstack((cmd,cmd[-1]))
    elif ct[-1]>duration+1e-9:
        raise ValueError("Original control time exceeds accepted window")
    # Limits from the previously frozen joint range, checked without clipping.
    mit=build_mit_commands(ct,cmd,q[0],bus_period_s=.005,kp=6.,kd=3276/32768,
                          qlimits=([-.75,-.35],[.45,.85]),vlimits=[.45,.35])
    positions=[];velocities=[];diagnostics=[]
    for j in range(2):
        trial=dict(input_time_s=mit["input_time_s"],observation_time_s=t,
                   initial_state=[float(q[0,j]),float(dq[0,j])],initial_time_s=float(t[0]),
                   command={k:np.asarray(a)[:,j] for k,a in mit["command"].items()})
        result=model.simulate_motion(trial,max_seconds=120,max_rhs=5000000)
        positions.append(result["prediction"][:,0]);velocities.append(result["prediction"][:,1])
        diagnostics.append(result["diagnostics"])
    modeled_q=np.column_stack(positions);modeled_dq=np.column_stack(velocities)
    geometry={"controller_model":run["metadata"]["physical_profile"]}
    modeled_tip=np.asarray([tip_from_report(p,r,qq,geometry)
                            for p,r,qq in zip(data["position"],data["quaternion"],modeled_q)])
    delta=modeled_tip-data["tip"]
    report={"complete":True,"kind":"ts20_frozen_body_command_replay","stage":stage,
            "new_closed_loop_flight":False,"new_mpc_solve":False,
            "hardware_voltage_v":24.,"source_identification_voltage_v":20.,
            "validated_at_hardware_voltage":False,
            "physical_load_transfer_validated":False,"independent_output_shaft_angle_validated":False,
            "source_run":run["report"]["source_run"],"source_audit":binding["source_audit"],
            "source_input_hashes":run["inputs"],
            "model":model.metadata(),"bridge":mit["metadata"],
            "time_contract":"control consumption timestamps from old logs, not measured TS20 dispatch times; no added old 24ms delay",
            "fixed_body_log":True,"feedback_recomputed":False,"voltage_scaling_applied":False,
            "initial_conditions":"first native q,dq only; no subsequent measured joint reset",
            "scope":"candidate motor-interface angles treated as joint angles only for sensitivity analysis",
            "joint_difference_rad":metrics(t,modeled_q-q),
            "tip_change_vs_original_m":metrics(t,delta),
            "conditional_tip_error_vs_original_reference_m":metrics(t,modeled_tip-run["target"]),
            "native_original_tip_error_m":run["replay"],
            "limits":{"q_lower":[-.75,-.35],"q_upper":[.45,.85],"dq_limit":[.45,.35],
                      "modeled_q_min":modeled_q.min(0).tolist(),"modeled_q_max":modeled_q.max(0).tolist(),
                      "modeled_dq_peak":np.max(abs(modeled_dq),axis=0).tolist()},
            "diagnostics":diagnostics}
    folder=args.output/stage;folder.mkdir()
    with (folder/"conditional_replay.csv").open("w",newline="") as stream:
        writer=csv.writer(stream)
        writer.writerow(["t","original_q1","original_q2","modeled_q1","modeled_q2",
                         "original_dq1","original_dq2","modeled_dq1","modeled_dq2",
                         "tip_delta_norm_m","conditional_tip_error_norm_m"])
        writer.writerows(np.column_stack((t,q,modeled_q,dq,modeled_dq,np.linalg.norm(delta,axis=1),
                                           np.linalg.norm(modeled_tip-run["target"],axis=1))))
    fig,axes=plt.subplots(4,1,figsize=(13,12),sharex=True,constrained_layout=True)
    for j in range(2):
        axes[j].plot(t,q[:,j],label="Original ideal-joint log",color="#397dbc")
        axes[j].plot(t,modeled_q[:,j],label="TS20 conditional interface replay",color="#dc8b2d")
        axes[j].set_ylabel(f"J{j+1} angle (rad)")
        axes[j].legend()
    axes[2].plot(t,np.linalg.norm(delta,axis=1)*1000,color="#dc8b2d")
    axes[2].set_ylabel("Tip CHANGE (mm)")
    axes[3].plot(t,np.linalg.norm(modeled_tip-run["target"],axis=1)*1000,color="#dc8b2d")
    axes[3].set_ylabel("Conditional tip error (mm)");axes[3].set_xlabel("Original elapsed time (s)")
    for ax in axes:ax.grid(alpha=.25)
    fig.suptitle(stage.upper()+" | TS20-50 interface sensitivity, fixed original body log\n"
                 "20 V bench model, requested hardware 24 V UNVALIDATED; not a new closed-loop flight")
    for ext in (".png",".pdf"):fig.savefig(folder/("conditional_replay"+ext),dpi=150)
    plt.close(fig)
    write_json(folder/"report.json",report)
    for p,digest in run["inputs"].items():
        if accepted.sha(p)!=digest:raise ValueError("Original input changed during replay")
    return {k:report[k] for k in ("stage","source_run","joint_difference_rad",
                                 "tip_change_vs_original_m","conditional_tip_error_vs_original_reference_m")}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for key in ("source","manifest","accepted-root","output"):
        parser.add_argument("--"+key,type=Path,required=True)
    args=parser.parse_args()
    if args.output.exists():raise ValueError("Use a new output directory")
    args.output.mkdir(parents=True)
    model=FrozenTs20Model(args.source,json.loads(args.manifest.read_text()))
    summary={"complete":False,"model":model.metadata(),"runs":[],
             "generator_sha256":accepted.sha(__file__),
             "adapter_sha256":accepted.sha(ROOT/"control_packages/x650_wholebody_v1/x650_wholebody/ts20.py")}
    write_json(args.output/"status.json",summary)
    try:
        summary["smoke"]=smoke(model,args.output)
        for stage in ("small","normal"):
            summary["current"]=stage;write_json(args.output/"status.json",summary)
            summary["runs"].append(run_case(args,model,stage));write_json(args.output/"status.json",summary)
        summary.update(complete=True,current=None)
        write_json(args.output/"status.json",summary)
        print(json.dumps(summary,indent=2))
    except BaseException as e:
        summary["error"]=repr(e);write_json(args.output/"status.json",summary);raise


if __name__=="__main__":main()
