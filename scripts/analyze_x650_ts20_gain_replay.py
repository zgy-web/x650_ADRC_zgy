#!/usr/bin/env python3
"""First-state-only gain counterfactual from recorded, consumed probe inputs.

No new flight, controller modification, reference reduction or parameter fit.
The same actual upper velocities and continuous adapter r produce MIT packets;
only the explicitly evaluated MIT Kd changes. Independent source oracle is
used for full-window free predictions. Results remain 20 V conditional-model
evidence, not qualification of a real 24 V loaded actuator.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import time

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "control_packages/x650_wholebody_v1"))
from x650_wholebody.ts20 import (FrozenTs20Model, POSITION_STEP_RAD,
                                 VELOCITY_STEP_RAD_S, quantize_half_away)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ("run", "source", "manifest", "output"):
        parser.add_argument("--"+name, required=True, type=Path)
    args = parser.parse_args()
    if args.output.exists():
        raise ValueError("Use a new output folder; evidence is never overwritten")
    args.output.mkdir(parents=True)
    metadata = json.loads((args.run / "controller_metadata.json").read_text())
    start, end = (float(metadata[key]) for key in ("task_start_sim_s", "task_end_sim_s"))
    states = np.genfromtxt(args.run/"ts20_state.csv", delimiter=",", names=True)
    packets = np.genfromtxt(args.run/"ts20_packets.csv", delimiter=",", names=True)
    window = (states["sim_s"] >= start-1e-9) & (states["sim_s"] <= end+1e-9)
    observed = states[window]
    if abs(observed["sim_s"][0]-start)>1e-9 or abs(observed["sim_s"][-1]-end)>1e-9:
        raise ValueError("Source states do not cover the complete declared measurement window")
    oracle = FrozenTs20Model(args.source, json.loads(args.manifest.read_text()))
    motion = oracle.metadata()["motion_model"]
    delay = motion["fixed_parameters"]["combined_interface_delay_s"]
    first = max(0, int(np.searchsorted(packets["send_s"]+delay, start+1e-10, side="right")-1))
    use = packets[first:int(np.searchsorted(packets["send_s"], end+1e-10, side="right"))]
    query = observed["sim_s"]
    kp, kd_original, kd_candidate = 6., 3276/32768, .25
    q0 = np.array([observed[f"q{j}_physical"][0] for j in (1,2)])
    dq0 = np.array([observed[f"dq{j}_internal"][0] for j in (1,2)])
    rebuilt_q, rebuilt_v, bridge = [], [], []
    for j in (1,2):
        # Start at actual continuous r, and integrate each actually consumed
        # velocity switch at its receipt boundary, independent of q after t0.
        active = float(observed[f"u{j}"][0])
        r, cursor = float(observed[f"r{j}"][0]), start
        source = states[states["sim_s"] >= start-1e-9]
        changes = []
        last_sequence = int(observed[f"upper_sequence{j}"][0])
        for row in source:
            sequence = int(row[f"upper_sequence{j}"])
            if sequence != last_sequence:
                when = float(row[f"receipt{j}_s"])
                if when > start+1e-10 and when <= end:
                    changes.append((when,float(row[f"u{j}"])))
                last_sequence = sequence
        output_q, output_v, index = [], [], 0
        for packet in use:
            when = float(packet["send_s"])
            if when < start-1e-10:
                output_q.append(float(packet[f"qref{j}"])); output_v.append(float(packet[f"dqref{j}"])); continue
            while index < len(changes) and changes[index][0] <= when+1e-10:
                switch, value = changes[index]
                r += (switch-cursor)*active
                cursor, active = switch, value
                index += 1
            r += (when-cursor)*active; cursor = when
            output_q.append(float(quantize_half_away(r, POSITION_STEP_RAD)))
            output_v.append(float(quantize_half_away(active, VELOCITY_STEP_RAD_S)))
        rebuilt_q.append(output_q); rebuilt_v.append(output_v)
        bridge.append({"joint":j,"q_ref_peak_difference_rad":float(np.max(abs(np.array(output_q)-use[f"qref{j}"]))),
                       "dq_ref_peak_difference_rad_s":float(np.max(abs(np.array(output_v)-use[f"dqref{j}"])))})
    if max(b["q_ref_peak_difference_rad"] for b in bridge)>POSITION_STEP_RAD*1.01 or max(b["dq_ref_peak_difference_rad_s"] for b in bridge)>1e-10:
        raise ValueError("Actual consumed upper/r reconstruction differs from actual MIT packets")
    output = {"schema":"ts20_probe_gain_counterfactual_v1", "complete":False,
        "window_start_s":start,"window_end_s":end,"window_duration_s":end-start,
        "samples":len(query), "bus_packets":len(use), "first_state_only":True,
        "later_observed_state_resets":False, "same_upper_velocity_and_integrated_r":True,
        "source_run":str(args.run), "bridge_reconstruction":bridge,
        "gain_original":{"kp":kp,"kd":kd_original},"gain_candidate":{"kp":kp,"kd":kd_candidate},
        "hardware_accessed":False,"supply_24v_qualified":False,"actual_load_qualified":False,
        "gain_counterfactual_is_open_loop_not_a_new_closed_loop_flight":True,
        "source_hashes":{n:hashlib.sha256((args.run/n).read_bytes()).hexdigest() for n in ["ts20_state.csv","ts20_packets.csv","controller_metadata.json"]},
        "model":oracle.metadata(),"results":{}}
    delta, vs = motion["friction_parameters"]["delta_static"],motion["friction_parameters"]["stribeck_velocity"]
    b = motion["fixed_parameters"]["b_over_ka"]
    max_negative_slope = math.sqrt(2/math.e)*delta/vs
    output["damping_analysis"]={"max_negative_stribeck_slope":max_negative_slope,
        "slope_max_at_abs_speed_rad_s":vs/math.sqrt(2),"original_b_plus_kd":b+kd_original,
        "candidate_b_plus_kd":b+kd_candidate,"candidate_margin_ratio":(b+kd_candidate)/max_negative_slope,
        "interpretation":"sliding differential damping bound only; not proof of full UAV stability"}
    def save():
        (args.output/"gain_replay_summary.json").write_text(json.dumps(output,indent=2))
    save()
    arrays = {"time_s":query,"observed_q":np.column_stack([observed[f"q{j}_physical"] for j in (1,2)]),
              "observed_dq":np.column_stack([observed[f"dq{j}_internal"] for j in (1,2)])}
    for label,kd in (("historical",kd_original),("damped_candidate",kd_candidate)):
        joint_results=[];predictions=[]
        for index,j in enumerate((1,2)):
            command={"q_ref":np.array(rebuilt_q[index]),"dq_ref":np.array(rebuilt_v[index]),
                "tau_ff":use[f"tau{j}"],"kp":np.full(len(use),kp),"kd":np.full(len(use),kd)}
            trial={"input_time_s":use["send_s"],"observation_time_s":query,"initial_time_s":start,
                "initial_state":[float(q0[index]),float(dq0[index])],"command":command}
            begin=time.perf_counter()
            result=oracle.simulate_motion(trial,max_seconds=120.,max_rhs=5000000)
            prediction=np.array(result["prediction"]);predictions.append(prediction)
            commanded_r=np.interp(query,states["sim_s"],states[f"r{j}"])
            actual_q,actual_v=observed[f"q{j}_physical"],observed[f"dq{j}_internal"]
            joint_results.append({"joint":j,"peak_abs_velocity_rad_s":float(np.max(abs(prediction[:,1]))),
                "small_probe_velocity_pass":bool(np.max(abs(prediction[:,1]))<=.2),
                "r_tracking_rms_rad":float(np.sqrt(np.trapz((prediction[:,0]-commanded_r)**2,query)/(end-start))),
                "r_tracking_peak_rad":float(np.max(abs(prediction[:,0]-commanded_r))),
                "vs_recorded_q_peak_rad":float(np.max(abs(prediction[:,0]-actual_q))),
                "vs_recorded_dq_peak_rad_s":float(np.max(abs(prediction[:,1]-actual_v))),
                "events":result["diagnostics"]["event_counts"],"modeled_stick_duration_s":result["diagnostics"]["modeled_stick_duration_s"],
                "compute_s":time.perf_counter()-begin})
        output["results"][label]=joint_results
        arrays[label+"_q"]=np.column_stack([a[:,0] for a in predictions]);arrays[label+"_dq"]=np.column_stack([a[:,1] for a in predictions])
        save()
    output.update(complete=True,candidate_pass=all(row["small_probe_velocity_pass"] for row in output["results"]["damped_candidate"]))
    np.savez_compressed(args.output/"gain_replay_timeseries.npz",**arrays)
    save()
    print(json.dumps({k:output[k] for k in ["complete","candidate_pass","window_duration_s","bridge_reconstruction","damping_analysis","results"]},indent=2))


if __name__ == "__main__":
    main()
