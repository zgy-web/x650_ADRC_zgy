#!/usr/bin/env python3
"""Plot frozen free-prediction evidence; never executes or resets a controller.

The independent event oracle is a 20 V bench-condition model reference, not
measured hardware. Numeric export permits local figure rendering without
downloading remote media. No experiment, fitting or oracle rerun occurs.
"""
from __future__ import annotations
import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def export(folder, case, output):
    summary = json.loads((folder / "validation_summary.json").read_text())
    match = next(row for row in summary["cases"] if row["name"] == case)
    if not summary["complete"] or not summary["pass"] or not match["pass"]:
        raise ValueError("Only a complete accepted oracle run can supply this figure")
    archive = folder / (case + ".npz")
    inputs_path = folder / (case + "_inputs.json")
    raw = np.load(archive, allow_pickle=False)
    inputs = json.loads(inputs_path.read_text())
    time = raw["time_s"]
    index = np.unique(np.r_[np.arange(0, len(time), 5), len(time)-1])
    q_error = raw["candidate_q"] - raw["oracle_q"]
    dq_error = raw["candidate_dq"] - raw["oracle_dq"]
    packets = inputs["packets"]
    q0 = np.array(inputs["initial_q_rad"])
    result = {"schema": "x650_ts20_prediction_figure_v1", "case": case,
        "scope": "20 V frozen small-disc interface model, not hardware or 24 V loaded-arm validation",
        "state_conditioning": "first q/dq only; no later oracle-state resets",
        "reference_curve": "independent event Stribeck oracle model; not measured hardware",
        "identification_supply_v": 20, "hardware_accessed": False,
        "supply_24v_qualified": False, "aerial_arm_load_qualified": False,
        "initial_q_rad": q0.tolist(), "initial_dq_rad_s": inputs["initial_dq_rad_s"],
        "time_s": time[index].tolist(), "full_sample_count": len(time),
        "plot_sample_count": len(index), "plot_step_s": float(time[5]-time[0]),
        "source_times_s": raw["source_times"].tolist(),
        "source_velocity_rad_s": raw["source_velocity"].tolist(),
        "bus_send_time_s": [p["send_time_s"] for p in packets],
        "bus_q_ref_rad": [p["q_ref"] for p in packets],
        "bus_dq_ref_rad_s": [p["dq_ref"] for p in packets],
        "combined_interface_delay_s": summary["model"]["motion_model"]["fixed_parameters"]["combined_interface_delay_s"],
        "protocol_sha256": summary["protocol_sha256"],
        "candidate_sha256": summary["candidate_sha256"],
        "native_backend": summary["native_backend"],
        "source_npz_sha256": hashlib.sha256(archive.read_bytes()).hexdigest(),
        "source_inputs_sha256": hashlib.sha256(inputs_path.read_bytes()).hexdigest(),
        "full_record_metrics": {"q_peak_rad_per_joint": np.max(abs(q_error), axis=0).tolist(),
            "dq_peak_rad_s_per_joint": np.max(abs(dq_error), axis=0).tolist(),
            "q_rms_rad_per_joint": np.sqrt(np.trapz(q_error*q_error, time, axis=0)/(time[-1]-time[0])).tolist(),
            "dq_rms_rad_s_per_joint": np.sqrt(np.trapz(dq_error*dq_error, time, axis=0)/(time[-1]-time[0])).tolist()}}
    for name in ("oracle_q", "candidate_q", "oracle_dq", "candidate_dq"):
        result[name] = np.round(raw[name][index], 10).tolist()
    output.mkdir(parents=True, exist_ok=True)
    path = output / ("ts20_" + case + "_prediction_data.json")
    path.write_text(json.dumps(result, separators=(",", ":"), allow_nan=False), encoding="utf-8")
    return result, path


def plot(data, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.ticker import MaxNLocator
    plt.rcParams.update({"font.size": 10, "axes.grid": True, "grid.alpha": .22,
                         "axes.spines.top": False, "axes.spines.right": False})
    fig, axes = plt.subplots(5, 2, figsize=(14, 13.7), sharex=True)
    t = np.array(data["time_s"])
    bt = np.array(data["bus_send_time_s"])
    st = np.array(data["source_times_s"])
    q0 = np.array(data["initial_q_rad"])
    oq, pq = np.array(data["oracle_q"]), np.array(data["candidate_q"])
    ov, pv = np.array(data["oracle_dq"]), np.array(data["candidate_dq"])
    busq, busv = np.array(data["bus_q_ref_rad"]), np.array(data["bus_dq_ref_rad_s"])
    sourcev = np.array(data["source_velocity_rad_s"])
    for j in range(2):
        axes[0, j].set_title(f"Joint {j+1} | initial q={q0[j]:+.2f} rad", fontweight="bold")
        axes[0, j].step(st, sourcev[:, j], where="post", color="#64748b", label="Upper velocity command")
        axes[0, j].step(bt, busv[:, j], where="post", color="#8b5cf6", linestyle="--", label="Sent MIT velocity target")
        axes[0, j].set_ylabel("Command (rad/s)")
        for row, ref, predicted, ylabel in ((1, ov[:, j], pv[:, j], "Joint speed (rad/s)"),
                                           (2, oq[:, j]-q0[j], pq[:, j]-q0[j], "Angle change (rad)")):
            axes[row, j].plot(t, ref, color="#172554", linewidth=2, label="Independent oracle reference")
            axes[row, j].plot(t, predicted, color="#ea580c", linewidth=1.4, linestyle="--", label="Free predictor")
            axes[row, j].set_ylabel(ylabel)
        axes[2, j].step(bt, busq[:, j]-q0[j], where="post", color="#94a3b8", linewidth=1., alpha=.85, label="Sent MIT position target")
        axes[3, j].plot(t, 1e3*(pq[:, j]-oq[:, j]), color="#0284c7")
        axes[3, j].set_ylabel("q error (mrad)")
        axes[4, j].plot(t, 1e3*(pv[:, j]-ov[:, j]), color="#be123c")
        axes[4, j].set_ylabel("dq error (mrad/s)")
        axes[4, j].set_xlabel("Time (s)")
        for row in range(5):
            axes[row, j].set_xlim(0., t[-1])
            axes[row, j].yaxis.set_major_locator(MaxNLocator(5))
            axes[row, j].axhline(0., color="#94a3b8", linewidth=.6)
            if row < 3:
                axes[row, j].legend(loc="upper right", fontsize=7.8, framealpha=.8)
    fig.suptitle("TS20-50 joint prediction chain | signed reversal", fontsize=18, fontweight="bold", y=.99)
    fig.text(.5, .959, "20 V conditional bench model | first-state free prediction | oracle curve is model reference, not hardware", ha="center", fontsize=10)
    metrics = data["full_record_metrics"]
    fig.text(.5, .023, "Full 1 ms record peaks: q="+str([round(v*1000, 4) for v in metrics["q_peak_rad_per_joint"]])+" mrad; dq="+str([round(v*1000, 4) for v in metrics["dq_peak_rad_s_per_joint"]])+" mrad/s", ha="center", fontsize=9)
    fig.text(.5, .007, "Plots use 5 ms samples; scores use all "+str(data["full_sample_count"])+" samples. 24 V / loaded aerial arm not qualified.", ha="center", fontsize=9, color="#64748b")
    fig.tight_layout(rect=(0., .04, 1., .943), h_pad=1.2)
    target = output / ("ts20_"+data["case"]+"_prediction_chain.png")
    fig.savefig(target, dpi=160, facecolor="white")
    plt.close(fig)
    return target


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--oracle-directory", type=Path, required=True)
    parser.add_argument("--case", default="reversal")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--export-only", action="store_true")
    args = parser.parse_args()
    data, numeric = export(args.oracle_directory, args.case, args.output)
    image = None if args.export_only else str(plot(data, args.output))
    print(json.dumps({"numeric_json": str(numeric), "plot": image,
                      "source_npz_sha256": data["source_npz_sha256"], "metrics": data["full_record_metrics"]}))


if __name__ == "__main__":
    main()
