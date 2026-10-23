#!/usr/bin/env python3
"""Scientific figures from ACCEPTED actual damped-TS20 simulation stages.

No flight, controller edit, fit, phase alignment, or synthetic data is created.
Requires run-dir + final stage certificate. joint_probe always recomputes
first-state free propagation through the hash-bound actual MIT packet log.
tip_hold uses independently recorded native tip and the fixed declared target
throughout the certificate's exact task-relative scoring interval; actual
runtime completion time is retained separately in the exported metadata.

Example after actual flights/certificates exist (not run during preparation):
  python scripts/plot_x650_ts20_commissioning_results.py --run-dir RUN \
    --certificate AUDIT/ts20_stage_certificate.json --output NEW_OUTPUT
Use --export-only to prepare numeric NPZ without importing Matplotlib.
External forecast arrays are not accepted as a substitute for recomputation.
The imported predictor source must match the accepted, hash-bound oracle.
Base-response prior identity comes only from the certified run metadata.

Tool validation currently covers synthetic regression fixtures only. It has
not yet been verified against actual server-stage data or produced actual
experimental results.

Both figures are conditional simulation evidence: the actuator model originates
from a nominal-20 V small-disc bench condition. Real 24 V loaded hardware is
not qualified. Native interval-mean dq and modeled endpoint dq stay distinct.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import importlib
import json
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
PROFILE_SHA256 = "9d23be903984c38373e1ea0488fb3c36f09d89504e4b9c78ae85820a415dcb97"
PROFILE_NAME = "ts20_50_24v_damped_v1"
DATA_KIND = "x650_ts20_certified_actual_stage_plot_data_v1"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def require(value, message):
    if not value:
        raise ValueError(message)


def array(rows, names):
    a = np.array([[float(row[name]) for name in names] for row in rows])
    require(a.ndim == 2 and np.isfinite(a).all(), "Missing/nonfinite recorded fields: "+str(names))
    return a


def window_rows(path, window, stamp="sim_s"):
    with Path(path).open(newline="", encoding="utf-8") as handle:
        rows = [row for row in csv.DictReader(handle)
                if window[0]-1e-9 <= float(row[stamp]) <= window[1]+1e-9]
    t = array(rows, [stamp]).ravel()
    require(len(t) >= 2 and np.all(np.diff(t) > 0), "No complete, increasing actual measurement record")
    require(t[0] <= window[0]+.008+1e-9 and t[-1] >= window[1]-.008-1e-9
            and np.diff(t).max() <= .008+1e-9, "Actual data omit part of the certified <=8ms whole window")
    return t, rows


def predictor_source_binding(cfg, cert):
    """Identify the module actually imported for this plot's propagation."""
    oracle_path = Path(cfg["ts20_inputs"]["oracle_validation"])
    oracle_sha = sha(oracle_path)
    require(oracle_sha == cert["ts20_operating_condition"]["oracle_validation_sha256"],
            "Oracle validation differs from the accepted actual run")
    oracle = read(oracle_path)
    require(oracle.get("complete") is True and oracle.get("pass") is True,
            "Complete accepted predictor oracle is required for plotting")
    sys.path.insert(0, str(ROOT/"control_packages/x650_wholebody_v1"))
    module = importlib.import_module("x650_wholebody.ts20_prediction")
    source = Path(module.__file__).resolve(strict=True)
    source_sha = sha(source)
    require(source_sha == oracle.get("candidate_sha256"),
            "Imported TS20 prediction source differs from the accepted oracle candidate")
    return {"path": str(source), "sha256": source_sha,
            "oracle_validation_path": str(oracle_path.resolve()),
            "oracle_validation_sha256": oracle_sha,
            "oracle_candidate_sha256": oracle["candidate_sha256"],
            "matches_accepted_oracle": True}


def base_prior_binding(meta, stage):
    """Copy recorded provenance, without reopening or inventing a prior model."""
    provenance = meta.get("provenance", {})
    require(isinstance(provenance, dict), "Recorded run provenance must be a mapping")
    declarations = [value for value in (meta.get("base_response_is_prior"),
                    provenance.get("base_response_is_prior")) if value is not None]
    require(all(type(value) is bool for value in declarations)
            and (not declarations or all(value == declarations[0] for value in declarations)),
            "Recorded base-response prior flags are invalid or inconsistent")
    is_prior = declarations[0] if declarations else None
    prior = provenance.get("base_response_prior")
    if prior is None:
        require(stage != "tip_hold", "Endpoint hold lacks recorded base-response prior identity")
        return {"status": "unknown", "base_response_is_prior": is_prior,
                "path": None, "sha256": None, "source_model_sha256": None,
                "new_plant_identified": None,
                "metadata_source": "controller_metadata.json.provenance.base_response_prior",
                "reason": "Prior identity was not recorded for this stage"}
    require(isinstance(prior, dict), "Recorded base-response prior must be a mapping")
    def digest(value):
        return isinstance(value, str) and len(value) == 64 and all(c in "0123456789abcdef" for c in value)
    require(isinstance(prior.get("path"), str) and bool(prior["path"])
            and digest(prior.get("sha256")) and digest(prior.get("source_model_sha256"))
            and type(prior.get("new_plant_identified")) is bool,
            "Recorded base-response prior identity is incomplete")
    require(is_prior is True and prior["new_plant_identified"] is False,
            "Expected the recorded base-response prior, not new-plant identification")
    return {"status": "recorded", "base_response_is_prior": is_prior,
            **{key: prior[key] for key in ("path", "sha256", "source_model_sha256", "new_plant_identified")},
            "metadata_source": "controller_metadata.json.provenance.base_response_prior"}


def base_prior_caption(binding):
    prior = binding["base_response_prior"]
    if prior["status"] == "unknown":
        return "Base-response prior identity: unknown in recorded metadata for this stage"
    return ("Base response: prior from model "+prior["source_model_sha256"][:12]
            +" | prior "+prior["sha256"][:12]+" | new-plant identification: no")


def bound_run(folder, certificate):
    cfg, meta = read(folder/"controller_task.json"), read(folder/"controller_metadata.json")
    cert = read(certificate)
    require(cert.get("kind") == "x650_ts20_conditional_stage_certificate_v1"
            and cert.get("complete") is True and cert.get("passed") is True,
            "A complete accepted actual-flight simulation certificate is required")
    stage = cfg.get("stage")
    require(stage in ("joint_probe", "tip_hold") and cert.get("stage") == stage,
            "Only certified actual joint_probe/tip_hold stages can be drawn")
    require(cfg.get("scenario") == "x650_ts20_damped_hold_v1" and meta.get("task") == cfg,
            "Expected the explicit actual damped scenario, not an offline counterfactual")
    require(all(meta.get(k) is True for k in ("complete", "landed_and_disarmed", "diagnostic_parameters_restored"))
            and meta.get("diagnostic_restore_pending") is False, "Run did not finish and restore parameters")
    require(cert.get("actuator_log_verified") is True and cert.get("full_window_ulog_verified") is True
            and cert.get("new_joint_prediction_verified") is True, "Actual actuator/full-window gates are missing")
    require(meta.get("ts20_operating_condition") == cert.get("ts20_operating_condition"),
            "Run/certificate actuator identities differ")
    window = np.array(cert["window_sim_s"], float)
    expected = np.array([meta["task_start_sim_s"], meta["task_end_sim_s"]], float)
    require(window.shape == (2,) and np.isfinite(window).all() and np.array_equal(window, expected)
            and abs(window[1]-window[0]-cfg["duration_s"]) <= .05
            and abs(cfg["duration_s"]-30.) <= 1e-12, "Whole declared 30s interval is required")
    source_directory = str(cert["source_run_directory"]).replace("\\", "/").rstrip("/")
    require(folder.name == source_directory.split("/")[-1], "Certificate belongs to a different actual run")
    names = ["controller_task.json", "controller_metadata.json", "stage_result.json", "native_trace.csv",
             "flight_samples.csv", "ts20_state.csv", "ts20_packets.csv"]
    inputs = {}
    for name in names:
        digest = sha(folder/name)
        require(cert.get("artifacts", {}).get(source_directory+"/"+name) == digest,
                "Certificate/input bytes differ: "+name)
        inputs[name] = digest
    qualification = cert.get("qualification", {})
    require(qualification.get("supply_24v_qualified") is False
            and qualification.get("actual_aerial_arm_load_qualified") is False,
            "Conditional-model qualification fields are missing or changed")
    return cfg, meta, cert, window, {"kind": DATA_KIND, "stage": stage,
        "source_run_id": folder.name, "source_run_directory": source_directory,
        "certificate_sha256": sha(certificate), "source_sha256": inputs,
        "window_sim_s": window.tolist(), "actual_task_window_sim_s": window.tolist(),
        "actual_task_end_sim_s": float(window[1]),
        "operating_condition": cert["ts20_operating_condition"],
        "prediction_source": predictor_source_binding(cfg, cert),
        "base_response_prior": base_prior_binding(meta, stage),
        "profile": {"name": PROFILE_NAME, "sha256": PROFILE_SHA256, "kp": 6., "kd": .25, "bus_period_s": .005},
        "simulation_conditional": True, "hardware_24v_loaded_qualified": False,
        "display_scope": "actual certified simulation; model originated at nominal 20V bench condition"}


def free_forecast(cfg, cert, t, q0, dq0, packet_rows, args):
    sys.path.insert(0, str(ROOT/"control_packages/x650_wholebody_v1"))
    from x650_wholebody.ts20 import FrozenTs20Model
    from x650_wholebody.ts20_prediction import Ts20PredictionModel
    inputs = cfg["ts20_inputs"]
    plant_path = args.plant_manifest or Path(inputs["plant_manifest"])
    plant = read(plant_path)
    require(sha(plant_path) == cert["ts20_operating_condition"]["plant_manifest_sha256"],
            "Plant manifest differs from accepted actual run")
    require(plant.get("candidate_profile_sha256") == PROFILE_SHA256,
            "Wrong effective-gain profile; counterfactual/historical data cannot substitute")
    p = plant["parameters"]
    require((p["kp"], p["kd"], p["bus_period_s"]) == (6., .25, .005), "Wrong actual MIT gain/bus condition")
    source = args.source_dir or Path(inputs["source_dir"])
    manifest_path = args.source_manifest or Path(inputs["source_manifest"])
    require(sha(manifest_path) == cert["ts20_operating_condition"]["source_manifest_sha256"],
            "External frozen source manifest differs from actual run")
    motion = FrozenTs20Model(source, read(manifest_path))
    predictor = Ts20PredictionModel(motion.metadata()["motion_model"], kp=p["kp"], kd=p["kd"], bus_period_s=p["bus_period_s"])
    lib = args.native_library or Path(inputs["native_library"])
    predictor.enable_native(lib, expected_sha256=inputs["native_library_sha256"])
    packets = [{"send_time_s": float(row["send_s"]), **{field: [float(row[short+str(j)]) for j in (1,2)]
        for field, short in (("q_ref","qref"),("dq_ref","dqref"),("tau_ff","tau"),("kp","kp"),("kd","kd"))}}
        for row in packet_rows]
    sends = np.array([row["send_time_s"] for row in packets])
    q, v = np.empty((len(t),2)), np.empty((len(t),2))
    q[0], v[0], cursor = q0, dq0, 0
    while cursor < len(t)-1:
        end = min(len(t)-1, int(np.searchsorted(t, t[cursor]+1.9, side="right")-1))
        require(end > cursor, "Cannot partition complete first-state-only free forecast")
        first_packet = max(0, int(np.searchsorted(sends+predictor.delay_s, t[cursor]+1e-11, side="right")-1))
        last_packet = int(np.searchsorted(sends, t[end]+1e-11, side="right"))
        out = predictor.rollout(float(t[cursor]), q[cursor].copy(), v[cursor].copy(),
                                packets[first_packet:last_packet], t[cursor:end+1])
        q[cursor:end+1], v[cursor:end+1] = out["q"], out["dq"]
        cursor = end  # Continue our own predicted state; never reset to measured q/dq.
    return q, v


def joint_data(folder, cfg, cert, window, binding, args):
    t, states = window_rows(folder/"ts20_state.csv", window)
    q = array(states, ["q1_physical", "q2_physical"])
    endpoint_v = array(states, ["dq1_internal", "dq2_internal"])
    interval_v = array(states, ["dq1_physical", "dq2_physical"])
    with (folder/"ts20_packets.csv").open(newline="", encoding="utf-8") as handle:
        packet_rows = list(csv.DictReader(handle))
    predicted_q, predicted_v = free_forecast(cfg, cert, t, q[0], endpoint_v[0], packet_rows, args)
    require(predicted_q.shape == predicted_v.shape == q.shape and np.isfinite([predicted_q,predicted_v]).all(),
            "Invalid/partial forecast data")
    require(np.max(abs(predicted_q[0]-q[0])) <= 1e-12 and np.max(abs(predicted_v[0]-endpoint_v[0])) <= 1e-12,
            "Forecast did not start at the actual first observation")
    qerr, verr = predicted_q-q, predicted_v-endpoint_v
    expected = cert["actuator"]["prediction"]
    require(abs(float(abs(qerr).max())-expected["q_peak_abs_rad"]) <= 1e-9
            and abs(float(abs(verr).max())-expected["dq_peak_abs_rad_s"]) <= 1e-7,
            "Reproduced first-state prediction differs from accepted whole-window certificate")
    binding.update(first_state_only=True, later_measured_state_resets=False,
        velocity_semantics={"physical_interval_dq": "Gazebo actual interval-mean velocity",
                            "endpoint_dq": "modeled servo endpoint speed report at source stamp"},
        q_peak_prediction_error_rad_per_joint=np.max(abs(qerr),axis=0).tolist(),
        dq_peak_prediction_error_rad_s_per_joint=np.max(abs(verr),axis=0).tolist())
    return {"time_sim_s":t,"actual_q_rad":q,"actual_endpoint_dq_rad_s":endpoint_v,
        "actual_physical_interval_dq_rad_s":interval_v,"predicted_q_rad":predicted_q,
        "predicted_endpoint_dq_rad_s":predicted_v,"q_error_rad":qerr,"dq_error_rad_s":verr}


def hold_data(folder, cfg, cert, window, binding):
    expected = cert["result"]["endpoint"]
    require("interval_task_s" in expected, "Certificate lacks its exact endpoint scoring interval")
    interval = np.asarray(expected["interval_task_s"], dtype=float)
    require(interval.shape == (2,) and np.isfinite(interval).all()
            and np.all(abs(interval-np.array([0.,cfg["duration_s"]])) <= 1e-12),
            "Certificate must score the complete declared task-relative interval")
    metric_window = float(window[0])+interval
    t, native = window_rows(folder/"native_trace.csv", metric_window)
    require(all(row[key] == "1" for row in native for key in ("valid_tip","valid_joint_state","valid_base")),
            "Invalid actual native tip/joint/base sample")
    tip = array(native, ["tip_x","tip_y","tip_z"])
    q = array(native, ["arm_joint_1_q","arm_joint_2_q"])
    with (folder/"flight_samples.csv").open(newline="",encoding="utf-8") as handle:
        selected=[row for row in csv.DictReader(handle) if row.get("phase") == "measure"
                  and metric_window[0]-1e-9 <= float(row["sim_time_s"]) <= metric_window[1]+1e-9]
    target = array(selected, ["tip_target_world_x_m","tip_target_world_y_m","tip_target_world_z_m"])
    require(len(target)>=2 and np.max(abs(target-target[0])) <= 1e-10, "Tip hold needs a fixed actual declared target")
    error = tip-target[0]
    integrate = getattr(np,"trapezoid",None) or np.trapz
    rms=float(np.sqrt(integrate(np.sum(error*error,axis=1),t)/(t[-1]-t[0])))
    peak=float(np.linalg.norm(error,axis=1).max())
    require(abs(rms-expected["position_rmse_m"])<=1e-10 and abs(peak-expected["position_max_m"])<=1e-10,
            "Whole-window native tip scores differ from the accepted certificate")
    binding.update(native_tip_source=True,same_time_error=True,no_offset_or_phase_fit=True,
        window_sim_s=metric_window.tolist(),metric_interval_task_s=interval.tolist(),
        actual_task_window_sim_s=np.asarray(window,float).tolist(),actual_task_end_sim_s=float(window[1]),
        position_rmse_m=rms,position_peak_m=peak,constant_target_world_enu_m=target[0].tolist(),
        disturbance_enabled=bool(cfg["disturbance_enabled"]))
    return {"time_sim_s":t,"native_tip_world_enu_m":tip,"tip_error_world_enu_m":error,
            "actual_q_rad":q,"constant_target_world_enu_m":target[0]}


def draw(data, binding, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({"font.size":10,"axes.grid":True,"grid.alpha":.25,
                         "axes.spines.top":False,"axes.spines.right":False})
    t=data["time_sim_s"]-binding["window_sim_s"][0]
    if binding["stage"]=="joint_probe":
        fig,axes=plt.subplots(3,2,figsize=(13.5,10),sharex=True)
        for j in range(2):
            axes[0,j].set_title(f"Joint {j+1}",fontweight="bold")
            axes[0,j].plot(t,data["actual_q_rad"][:,j],color="#172554",label="Actual physical q")
            axes[0,j].plot(t,data["predicted_q_rad"][:,j],"--",color="#ea580c",label="First-state free prediction")
            axes[0,j].set_ylabel("Joint angle (rad)")
            axes[1,j].plot(t,1e3*data["q_error_rad"][:,j],color="#0284c7")
            axes[1,j].set_ylabel("Predicted - actual q (mrad)")
            axes[2,j].plot(t,data["actual_endpoint_dq_rad_s"][:,j],color="#be123c",label="Modeled endpoint dq report")
            axes[2,j].plot(t,data["actual_physical_interval_dq_rad_s"][:,j],color="#64748b",alpha=.65,label="Actual physical interval-mean dq")
            axes[2,j].set_ylabel("Joint speed (rad/s)");axes[2,j].set_xlabel("Time from task start (s)")
            axes[0,j].legend(fontsize=8);axes[2,j].legend(fontsize=8)
        title="X650 + TS20-50 | actual joint probe and free prediction"
    else:
        fig,axes=plt.subplots(3,1,figsize=(13.5,9.8),sharex=True)
        for j,(axis,color) in enumerate(zip("XYZ",("#0284c7","#b45309","#047857"))):
            axes[0].plot(t,1e3*data["tip_error_world_enu_m"][:,j],color=color,label=axis)
        axes[0].set_ylabel("Native tip error (mm)");axes[0].legend(ncol=3)
        axes[1].plot(t,1e3*np.linalg.norm(data["tip_error_world_enu_m"],axis=1),color="#172554",label="3D same-time error norm")
        axes[1].set_ylabel("3D error (mm)");axes[1].legend()
        for j,color in enumerate(("#ea580c","#8b5cf6")):
            axes[2].plot(t,data["actual_q_rad"][:,j],color=color,label=f"Actual physical q{j+1}")
        axes[2].set_ylabel("Joint angle (rad)");axes[2].set_xlabel("Time from task start (s)");axes[2].legend(ncol=2)
        title="X650 + TS20-50 | actual full-window endpoint hold"
        fig.text(.5,.025,f"Full-window 3D RMS {binding['position_rmse_m']*1000:.3f} mm | peak {binding['position_peak_m']*1000:.3f} mm | no phase/origin fit",ha="center")
    for ax in np.asarray(axes).ravel():
        ax.set_xlim(0.,binding["window_sim_s"][1]-binding["window_sim_s"][0]);ax.axhline(0.,color="#94a3b8",linewidth=.6)
    fig.suptitle(title,fontweight="bold",fontsize=16,y=.99)
    fig.text(.5,.955,"Certified native simulation | effective Kp=6, Kd=0.25 | nominal-20V model; 24V loaded hardware not qualified",ha="center",fontsize=9)
    fig.text(.5,.933,base_prior_caption(binding),ha="center",fontsize=9)
    fig.text(.5,.008,binding["source_run_id"]+" | complete certified measurement interval",ha="center",fontsize=8,color="#64748b")
    fig.tight_layout(rect=(0.,.045,1.,.91))
    path=output/(binding["stage"]+"_certified_actual.png")
    require(not path.exists(),"Figure already exists; use a fresh output directory")
    fig.savefig(path,dpi=160,facecolor="white");plt.close(fig)
    return path


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ("run-dir","certificate","output"):
        parser.add_argument("--"+name,type=Path,required=True)
    for name in ("plant-manifest","source-dir","source-manifest","native-library"):
        parser.add_argument("--"+name,type=Path)
    parser.add_argument("--export-only",action="store_true")
    args=parser.parse_args()
    folder=args.run_dir.resolve();certificate=args.certificate.resolve()
    cfg,meta,cert,window,binding=bound_run(folder,certificate)
    data=(joint_data(folder,cfg,cert,window,binding,args) if cfg["stage"]=="joint_probe"
          else hold_data(folder,cfg,cert,window,binding))
    args.output.mkdir(parents=True,exist_ok=True)
    basename=cfg["stage"]+"_certified_actual"
    numeric=args.output/(basename+".npz");manifest=args.output/(basename+"_manifest.json")
    require(not numeric.exists() and not manifest.exists(),"Never overwrite prior plot evidence")
    with numeric.open("xb") as handle:
        np.savez_compressed(handle,metadata_json=json.dumps(binding,sort_keys=True),**data)
    binding["numeric_npz_sha256"]=sha(numeric)
    binding["figure_path"]=None if args.export_only else str(draw(data,binding,args.output))
    binding["plot_script_sha256"]=sha(__file__)
    manifest.write_text(json.dumps(binding,indent=2),encoding="utf-8")
    print(json.dumps({"numeric_npz":str(numeric),"manifest":str(manifest),"figure":binding["figure_path"],"scope":"actual certified conditional simulation"}))


if __name__=="__main__":
    main()
