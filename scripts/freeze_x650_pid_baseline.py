"""Read-only engineering PID baseline freeze from six completed native runs.

A passed default certificate permits upper-MPC commissioning only. It never
claims this PID is fully tuned or globally optimal. Candidate application and
parameter optimization are not performed by this evidence audit.
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import hashlib
import json
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for directory in (ROOT/"scripts",ROOT/"control_packages/x650_wholebody_v1"):
    if str(directory) not in sys.path:
        sys.path.insert(0,str(directory))
from identify_x650_upper_response import (Criteria as ResponseCriteria, axis_data, load_run,
                                         sha, json_read, require_hash, safe_sibling)
from x650_wholebody.model import ArmModel,tip_world

ROLES = {"hover":"hover_folded","joint":"joint_probe","tip":"tip_hold","small":"small",
         "response_train":"response_probe","response_holdout":"response_probe"}
CRITICAL_SOURCES = tuple("control_packages/x650_wholebody_v1/x650_wholebody/"+name+".py"
                         for name in ("model","baseline","adapter","feedforward","api"))
REQUIRED_PID_GAINS = frozenset((
    "MPC_XY_P","MPC_Z_P","MPC_XY_VEL_P_ACC","MPC_XY_VEL_I_ACC","MPC_XY_VEL_D_ACC",
    "MPC_Z_VEL_P_ACC","MPC_Z_VEL_I_ACC","MPC_Z_VEL_D_ACC","MC_ROLL_P","MC_PITCH_P","MC_YAW_P",
    *(f"MC_{axis}RATE_{term}" for axis in ("ROLL","PITCH","YAW") for term in ("P","I","D","K","FF"))))


@dataclass(frozen=True)
class Acceptance:
    hover_rmse_m: float = .10
    hover_peak_m: float = .20
    tip_hold_rmse_m: float = .05
    tip_hold_peak_m: float = .10
    small_rmse_m: float = .04
    small_peak_m: float = .10
    maximum_tilt_deg: float = 8.
    maximum_native_interval_s: float = .008
    maximum_control_interval_s: float = .05
    joint_command_limits_rad_s: tuple = (.45,.35)
    base_command_limits_m_s: tuple = (.55,.35,.45)


def small_reference(times,center,parameters):
    required = dict(amplitude_x=.075,amplitude_z=.03,period_s=16.,cycles=2,start_s=2.,ramp_s=2.)
    if parameters != required:
        raise ValueError("Engineering small-trajectory protocol differs from the frozen preset")
    a,b,period,cycles,start,ramp = (parameters[k] for k in ("amplitude_x","amplitude_z","period_s","cycles","start_s","ramp_s"))
    duration,total,w = cycles*period+ramp,2*np.pi*cycles,2*np.pi/period
    phase = np.empty(len(times))
    for i,t in enumerate(times):
        s = t-start
        if s <= 0.: p = 0.
        elif s >= duration: p = total
        elif s < ramp:
            u = s/ramp; p = w*ramp*(2.5*u**4-3*u**5+u**6)
        elif s > duration-ramp:
            u = (duration-s)/ramp; p = total-w*ramp*(2.5*u**4-3*u**5+u**6)
        else: p = w*(s-.5*ramp)
        phase[i] = p
    return np.asarray(center)+np.column_stack((a*np.sin(phase),np.zeros(len(times)),b*np.sin(2*phase)))


def joint_reference(times,duration):
    active = duration-4.
    result = np.zeros((len(times),2))
    for i,t in enumerate(times):
        if t <= 2. or t >= duration-2.: continue
        index = min(2,int((t-2.)/(active/3.)))
        angle = 2*np.pi*(t-2.-index*active/3.)/(active/3.)
        amplitude = np.array([.08,0.] if index == 0 else [0.,.08] if index == 1 else [.06,-.06])
        result[i] = amplitude*np.sin(angle)**3
    return result


def native_metrics(run,role,limits):
    folder,meta = Path(run["folder"]),run["metadata"]
    required = {"native_trace.csv","native_trace_manifest.json","flight_samples.csv","flight_samples_manifest.json"}
    if not required.issubset(run["stage_evidence"]["artifacts"]):
        raise ValueError("Baseline audit needs independent native and flight sample artifacts")
    native_manifest,flight_manifest = json_read(folder/"native_trace_manifest.json"),json_read(folder/"flight_samples_manifest.json")
    if any(manifest.get("complete") is not True for manifest in (native_manifest,flight_manifest)):
        raise ValueError("Independent native/flight recording is incomplete")
    for manifest in (native_manifest,flight_manifest):
        for key in ("model_sha256","source_urdf_sha256"):
            if manifest.get(key) != run["identity"][key]: raise ValueError("Recording manifest belongs to another model")
        file = safe_sibling(folder,manifest["file"])
        if sha(file) != manifest.get("sha256"): raise ValueError("Recording manifest file hash mismatch")
    start,end = float(meta["task_start_sim_s"]),float(meta["task_end_sim_s"])
    expected = float(meta["requested_duration_s"])
    if not np.isfinite([start,end,expected]).all() or end-start < expected-.008:
        raise ValueError("Native task duration is incomplete")
    with (folder/"native_trace.csv").open(newline="",encoding="utf-8") as stream:
        rows = [r for r in csv.DictReader(stream) if start-1e-9 <= float(r["sim_s"]) <= end+1e-9]
    fields = ["sim_s",*["base_p"+a for a in "xyz"],*["base_q"+a for a in "wxyz"],
              "arm_joint_1_q","arm_joint_2_q","arm_joint_1_qd","arm_joint_2_qd",*["tip_"+a for a in "xyz"]]
    values = np.array([[float(r[k]) for k in fields] for r in rows])
    if (len(rows) < 2 or values.shape != (len(rows),15) or not np.isfinite(values).all()
            or np.any(np.diff(values[:,0]) <= 0) or values[0,0] > start+.008+1e-9
            or values[-1,0] < end-.008-1e-9
            or np.max(np.diff(values[:,0])) > limits.maximum_native_interval_s+1e-9
            or any(any(r[k] != "1" for k in ("valid_base","valid_joint_state","valid_tip")) for r in rows)):
        raise ValueError("Independent native samples have gaps, invalid flags or incomplete task coverage")
    if meta.get("folded_before_landing") is not True:
        raise ValueError("Folded-before-landing physical transition was not completed")
    profile = meta.get("physical_profile")
    if not isinstance(profile,dict) or profile.get("source_report_sha256") != run["identity"]["preparation_report_sha256"]:
        raise ValueError("Recorded physical profile is not bound to this preparation report")
    model = ArmModel(**profile)
    q,qd = values[:,8:10],values[:,10:12]
    norms = np.linalg.norm(values[:,4:8],axis=1)
    if np.any(norms < 1e-9): raise ValueError("Invalid native orientation")
    quat = values[:,4:8]/norms[:,None]
    tilt = np.rad2deg(np.arccos(np.clip(1-2*(quat[:,1]**2+quat[:,2]**2),-1.,1.)))
    control = run["control_rows"]
    command_times = np.array([float(r["sim_s"]) for r in control])
    joint_command = np.array([[float(r[k]) for k in ("qd1_cmd","qd2_cmd")] for r in control])
    base_command = run["commands"]
    if (not np.isfinite(joint_command).all() or command_times[0] > start+.05
            or command_times[-1] < end-.05):
        raise ValueError("Control trace does not cover the native task")
    with (folder/"flight_samples.csv").open(newline="",encoding="utf-8") as stream:
        flight = [r for r in csv.DictReader(stream) if r["phase"] == "measure"]
    if not flight or any(r["failsafe"] != "0" or r["armed"] != "1" or r["landed"] != "0" for r in flight):
        raise ValueError("Measurement flight state contains failsafe, landed or disarmed samples")
    metrics = {"sample_count":len(rows),"duration_s":float(values[-1,0]-values[0,0]),
               "native_max_interval_s":float(np.max(np.diff(values[:,0]))),
               "control_max_interval_s":float(np.max(np.diff(command_times))),"tilt_max_deg":float(tilt.max()),
               "joint_command_max_rad_s":np.max(abs(joint_command),axis=0).tolist(),
               "base_command_max_m_s":np.max(abs(base_command),axis=0).tolist()}
    checks = {"native_tilt_within_engineering_limit":float(tilt.max()) <= limits.maximum_tilt_deg,
              "control_interval_within_existing_limit":metrics["control_max_interval_s"] <= limits.maximum_control_interval_s+1e-9,
              "joint_commands_within_common_caps":bool(np.all(abs(joint_command) <= np.array(limits.joint_command_limits_rad_s)+1e-8)),
              "base_commands_within_common_caps":bool(np.all(abs(base_command) <= np.array(limits.base_command_limits_m_s)+1e-8)),
              "physical_joint_limits_respected":bool(np.all(q >= np.array(model.joint_lower)-1e-8) and np.all(q <= np.array(model.joint_upper)+1e-8))}
    task_t = values[:,0]-start
    integrate = getattr(np,"trapezoid",np.trapz)
    if role == "hover":
        desired = np.array([[float(r["target_world_"+a+"_m"]) for a in "xyz"] for r in flight])
        if not np.isfinite(desired).all() or np.max(abs(desired-desired[0])) > 1e-9:
            raise ValueError("Hover target is missing or was redefined during measurement")
        error,rmse_limit,peak_limit = values[:,1:4]-desired[0],limits.hover_rmse_m,limits.hover_peak_m
    elif role in {"tip","small"}:
        anchor,reference_rotation = np.asarray(meta["reference_base_enu_m"]),np.asarray(meta["reference_rotation"])
        center = tip_world(anchor,reference_rotation,[0.,0.],model)
        desired = np.tile(center,(len(task_t),1)) if role == "tip" else small_reference(task_t,center,meta["task"]["trajectory_parameters"])
        selected = np.ones(len(task_t),dtype=bool) if role == "tip" else ((task_t >= 2.-1e-9)&(task_t <= 36.+1e-9))
        error = values[selected,12:15]-desired[selected]
        task_t = task_t[selected]
        rmse_limit,peak_limit = ((limits.tip_hold_rmse_m,limits.tip_hold_peak_m) if role == "tip"
                                 else (limits.small_rmse_m,limits.small_peak_m))
    elif role == "joint":
        desired = joint_reference(task_t,expected)
        error = q-desired
        rmse = np.sqrt(integrate(error**2,task_t,axis=0)/(task_t[-1]-task_t[0]))
        corr = np.sum(q*desired,axis=0)/np.sqrt(np.maximum(1e-24,np.sum(q*q,axis=0)*np.sum(desired*desired,axis=0)))
        displacement = np.linalg.norm(values[:,1:4]-np.asarray(meta["reference_base_enu_m"]),axis=1)
        metrics.update(joint_tracking_rmse_rad=rmse.tolist(),joint_tracking_peak_rad=np.max(abs(error),axis=0).tolist(),
                       joint_sign_correlation=corr.tolist(),joint_velocity_max_rad_s=np.max(abs(qd),axis=0).tolist(),
                       base_displacement_max_m=float(displacement.max()))
        checks.update(both_joints_move_both_signs=bool(np.all(q.max(axis=0) >= .035) and np.all(q.min(axis=0) <= -.035)),
                      joint_tracking_within_existing_caps=bool(np.all(rmse <= .02) and np.max(abs(error)) <= .05),
                      joint_probe_amplitude_speed_within_existing_caps=bool(np.max(abs(q)) <= .12 and np.max(abs(qd)) <= .2),
                      joint_reference_sign_correct=bool(np.all(corr >= .8)),
                      base_displacement_within_existing_joint_probe_cap=float(displacement.max()) <= .3)
        return {"passed":all(checks.values()),"criteria":checks,"metrics":metrics}
    else:
        excitation = [axis_data(run,i,ResponseCriteria()) for i in range(3)]
        checks["all_axes_have_measured_bipolar_excitation"] = all(v["passed"] for v in excitation)
        metrics["excitation"] = [{k:v for k,v in item.items() if k not in {"times","velocity","all_velocity"}} for item in excitation]
        return {"passed":all(checks.values()),"criteria":checks,"metrics":metrics}
    rmse = float(np.sqrt(integrate(np.sum(error**2,axis=1),task_t)/(task_t[-1]-task_t[0])))
    peak = float(np.linalg.norm(error,axis=1).max())
    metrics.update(position_rmse_m=rmse,position_peak_m=peak,position_rmse_limit_m=rmse_limit,position_peak_limit_m=peak_limit)
    checks.update(same_time_position_rmse_within_engineering_limit=rmse <= rmse_limit,
                  position_peak_within_engineering_limit=peak <= peak_limit)
    return {"passed":all(checks.values()),"criteria":checks,"metrics":metrics}


def tuning_evidence_audit(path,identity,selected_runs):
    """Validate an external finite-search audit, never infer tuning from stability."""
    if path is None:
        return {"passed":False,"provided":False,"reason":"No independent native candidate-search and held-out selection evidence"}
    path = Path(path).resolve(strict=True)
    data = json_read(path)
    for key in ("model_sha256","source_urdf_sha256","pid_parameters_sha256"):
        if data.get(key) != identity[key]: raise ValueError("Tuning evidence identity does not match frozen PID")
    if (data.get("kind") != "x650_pid_tuning_audit" or data.get("complete") is not True or data.get("passed") is not True
            or data.get("selection_training_only") is not True or data.get("independent_holdout") is not True):
        raise ValueError("Strict tuning requires a completed native finite-search audit")
    candidates = data.get("candidate_runs",[])
    validation_hashes = {run["command_csv_sha256"] for run in selected_runs.values()}
    pid_hashes,artifacts = set(),{str(path):sha(path)}
    for candidate in candidates:
        run_path = Path(candidate["run_directory"])
        run_path = run_path if run_path.is_absolute() else path.parent/run_path
        run = load_run(run_path)
        if any(run["identity"][k] != identity[k] for k in identity if k != "pid_parameters_sha256"):
            raise ValueError("Native tuning candidate belongs to another physical plant")
        if run["command_csv_sha256"] in validation_hashes:
            raise ValueError("Tuning training data overlap the six selected validation runs")
        pid_hashes.add(run["identity"]["pid_parameters_sha256"])
        artifacts.update(run["artifacts"])
    if len(pid_hashes) < 2 or identity["pid_parameters_sha256"] not in pid_hashes:
        raise ValueError("Tuning audit has no distinct actual native PID alternatives including the selected profile")
    declared = {require_hash(v,"candidate_pid_hash") for v in data.get("candidate_pid_parameters_sha256",[])}
    if declared != pid_hashes:
        raise ValueError("Declared candidate PID hashes differ from actual native readbacks")
    return {"passed":True,"provided":True,"kind":"finite_native_candidate_search",
            "candidate_pid_count":len(pid_hashes),"evidence_path":str(path),"artifacts":artifacts,
            "scope":"External search audit with independent native runs; no global optimum claim"}


def audit_baseline(paths,*,strict_tuning=False,tuning_evidence=None):
    if set(paths) != set(ROLES): raise ValueError("Supply exactly the six required baseline run roles")
    runs = {role:load_run(path,ROLES[role]) for role,path in paths.items()}
    identity = runs["hover"]["identity"]
    if not REQUIRED_PID_GAINS.issubset(runs["hover"]["pid_parameters"]):
        raise ValueError("Actual readback is missing position/velocity/attitude/rate PID gain names: "+
                         str(sorted(REQUIRED_PID_GAINS-set(runs["hover"]["pid_parameters"]))))
    if any(run["identity"] != identity for run in runs.values()):
        raise ValueError("All validation runs must share physical model and actual MC/MPC parameter hashes")
    if len({run["folder"] for run in runs.values()}) != len(ROLES): raise ValueError("Each validation role requires a separate native run")
    first,second = runs["response_train"],runs["response_holdout"]
    if (first["command_csv_sha256"] == second["command_csv_sha256"]
            or first["normalized_measurement_sha256"] == second["normalized_measurement_sha256"]):
        raise ValueError("Two response probes must be independent measurements")
    source_maps = [run["metadata"].get("provenance",{}).get("source_files_sha256",{}) for run in runs.values()]
    for source in CRITICAL_SOURCES:
        digests = {source_map.get(source) for source_map in source_maps}
        if len(digests) != 1 or None in digests:
            raise ValueError("Shared PID/FF/model source differs across validation runs: "+source)
        require_hash(next(iter(digests)),source)
    limits = Acceptance()
    outcomes = {role:native_metrics(run,role,limits) for role,run in runs.items()}
    stable = all(item["passed"] for item in outcomes.values())
    tuning = tuning_evidence_audit(tuning_evidence,identity,runs)
    passed = stable and (not strict_tuning or tuning["passed"])
    artifacts = {name:digest for run in runs.values() for name,digest in run["artifacts"].items()}
    artifacts.update(tuning.get("artifacts",{}))
    baseline = {"kind":"x650_native_stage_evidence","schema_version":1,"stage":"pid_baseline",
                "simulation_backend":"gazebo_px4","complete":True,"passed":passed,"success":passed,
                **identity,"engineering_stability_passed":stable,"baseline_scope":"engineering_stability_only",
                "tuning_claim":False,"formal_fully_tuned_comparison_ready":False,"globally_optimal_pid":False,
                "strict_tuning_mode":bool(strict_tuning),"tuning_evidence_audit":tuning,
                "control_source_files_sha256":{name:source_maps[0][name] for name in CRITICAL_SOURCES},
                "criteria":asdict(limits),"criteria_status":"Predeclared engineering commissioning acceptance, not a precision/optimality guarantee",
                "stage_metrics":outcomes,"validation_runs":{role:{"directory":run["folder"],"control_trace_sha256":run["command_csv_sha256"]} for role,run in runs.items()},
                "parameter_writes":False,"artifacts":artifacts,"auditor_source_sha256":sha(__file__),
                "readback_scope":"All captured MC/MPC entries including the required 26 gain names; not a full PX4 parameter export",
                "scope":"Permits upper-MPC debugging under a measured stable PID profile; does not complete fully-tuned fair comparison",
                "formal_comparison_work_remaining":["Declare finite PID candidates/scoring and apply actual values before arming with readback",
                    "Run training probes and select without held-out results","Run new independent selected-profile validation and confirm declared comparison coverage"]}
    profile = {"kind":"x650_pid_profile","schema_version":1,**identity,"parameters":runs["hover"]["pid_parameters"],
               "engineering_stability_passed":stable,"baseline_scope":"engineering_stability_only","tuning_claim":False,
               "control_source_files_sha256":{name:source_maps[0][name] for name in CRITICAL_SOURCES},
               "formal_fully_tuned_comparison_ready":False,"parameter_writes":False,
               "evidence":"pid_baseline.json","scope":"Frozen actual readback for upper-MPC commissioning; not an optimized or globally best PID"}
    return profile,baseline


def write_baseline(paths,output,*,strict_tuning=False,tuning_evidence=None):
    output = Path(output)
    if output.exists(): raise ValueError("Use a new output directory; baseline evidence is never overwritten")
    try:
        profile,report = audit_baseline(paths,strict_tuning=strict_tuning,tuning_evidence=tuning_evidence)
    except (ValueError,KeyError,OSError,TypeError) as exc:
        profile = {"kind":"x650_pid_profile","engineering_stability_passed":False,"parameters":{},"tuning_claim":False,
                   "formal_fully_tuned_comparison_ready":False,"reason":str(exc)}
        report = {"kind":"x650_native_stage_evidence","stage":"pid_baseline","simulation_backend":"gazebo_px4",
                  "complete":False,"passed":False,"success":False,"engineering_stability_passed":False,
                  "baseline_scope":"engineering_stability_only","tuning_claim":False,"formal_fully_tuned_comparison_ready":False,
                  "reason":str(exc),"artifacts":{},"strict_tuning_mode":bool(strict_tuning)}
    output.mkdir(parents=True,exist_ok=False)
    profile_path = output/"pid_profile.json"
    profile_path.write_text(json.dumps(profile,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    report["artifacts"]["pid_profile.json"] = sha(profile_path)
    (output/"pid_baseline.json").write_text(json.dumps(report,indent=2,allow_nan=False)+"\n",encoding="utf-8")
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    for role in ROLES: parser.add_argument("--"+role.replace("_","-"),type=Path,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--strict-tuning",action="store_true")
    parser.add_argument("--tuning-evidence",type=Path)
    args = parser.parse_args(argv)
    report = write_baseline({role:getattr(args,role) for role in ROLES},args.output,
                            strict_tuning=args.strict_tuning,tuning_evidence=args.tuning_evidence)
    print(json.dumps({"output":str(args.output),"passed":report["passed"],
                      "engineering_stability_passed":report["engineering_stability_passed"],
                      "tuning_claim":False,"formal_fully_tuned_comparison_ready":False,"reason":report.get("reason")},indent=2))
    return 0 if report["passed"] else 2


if __name__ == "__main__": raise SystemExit(main())
