#!/usr/bin/env python3
"""Scientific plots from delivered X650 audit JSON; sampled rows are curves only.

All displayed RMS, peak, overshoot and settling values are copied from the
full-resolution native/ULog reports. No metric is recomputed from 10 Hz samples.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
DEFAULT_INPUT = ROOT/"artifacts/x650_hover_precision_20261021"
COLORS = ["#2462ad", "#e57a25", "#299563", "#a050a5", "#c04958", "#787d24", "#348e9e", "#694938"]
CANDIDATE_GAINS = ("MPC_XY_P", "MPC_Z_P", "MPC_XY_VEL_P_ACC", "MPC_Z_VEL_P_ACC")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def load_run(path):
    folder = Path(path)
    names = ("controller_metadata.json", "hover_reference.json", "plot_samples.json", "native_trace_manifest.json")
    data = {name[:-5]: load_json(folder/name) for name in names}
    meta, plot, manifest = (data[key] for key in ("controller_metadata", "plot_samples", "native_trace_manifest"))
    if meta.get("complete") is not True or meta.get("landed_and_disarmed") is not True or manifest.get("complete") is not True:
        raise ValueError(folder.name+": incomplete or unrecovered source")
    if (plot.get("visualization_only") is not True or plot.get("metrics_must_use_complete_native_csv") is not True
            or plot["source_csv_sha256"] != manifest["sha256"]
            or plot["source_metadata_sha256"] != sha(folder/"controller_metadata.json")
            or plot["source_native_manifest_sha256"] != sha(folder/"native_trace_manifest.json")):
        raise ValueError(folder.name+": decimated curve source/hash mismatch")
    for key in ("model_sha256", "source_urdf_sha256"):
        if manifest[key] != meta[key]:
            raise ValueError(folder.name+": model identity mismatch")
    for name in ("precision_audit.json", "position_steps_audit.json", "diagnostic.json", "audit_summary.json"):
        if (folder/name).exists():
            data[name[:-5]] = load_json(folder/name)
            names += (name,)
    extra_inputs = {}
    full56_path = folder.with_name(folder.name+"_full56")/"precision_audit.json"
    if "precision_audit" in data and full56_path.exists():
        original, upgraded = data["precision_audit"], load_json(full56_path)
        if (upgraded["source_run"] != original["source_run"]
                or upgraded["inputs_sha256"]["native"] != original["inputs_sha256"]["native"]
                or upgraded["inputs_sha256"]["metadata"] != original["inputs_sha256"]["metadata"]
                or upgraded["position_native_world_enu"] != original["position_native_world_enu"]
                or upgraded.get("full_experiment_ulog_guard", {}).get("verified") is not True):
            raise ValueError(folder.name+": upgraded 56 s report changes evidence or primary metrics")
        data["precision_audit_original"] = original
        data["precision_audit"] = upgraded
        extra_inputs["../"+full56_path.parent.name+"/precision_audit.json"] = sha(full56_path)
    target = np.asarray(data["hover_reference"]["target_world_enu_m"], float)
    if not np.allclose(target, meta["hover_metrics"]["fixed_target_enu_m"], rtol=0, atol=1e-9):
        raise ValueError(folder.name+": declared target mismatch")
    if "precision_audit" in data:
        report = data["precision_audit"]
        if (report["inputs_sha256"]["native"] != plot["source_csv_sha256"]
                or report["inputs_sha256"]["metadata"] != plot["source_metadata_sha256"]):
            raise ValueError(folder.name+": precision-report source mismatch")
        measured_pid = report.get("measurement_period_pid", {}).get("parameters", {})
        declared_pid = meta["actual_pid_parameters"]
        if (len(measured_pid) != 42 or set(measured_pid) != set(declared_pid)
                or any(not np.isclose(float(measured_pid[key]), float(declared_pid[key]), rtol=2e-6, atol=1e-7)
                       for key in declared_pid)):
            raise ValueError(folder.name+": measurement-period ULog PID differs from metadata; never substitute the restore profile")
        static = report["position_native_world_enu"]
        overall = dict(rms=static["rmse_3d"], peak=static["peak_3d"])
        chain = report["ulog_control_chain"]["position_world_enu"]
        decomposition = [chain[key]["rmse_3d"] for key in ("native_minus_estimate",
            "estimate_minus_actual_position_setpoint", "actual_position_setpoint_minus_declared_world_target")]
    else:
        report = data["diagnostic"]
        if report["source_files"]["native"]["sha256"] != plot["source_csv_sha256"]:
            raise ValueError(folder.name+": GPS diagnostic source mismatch")
        static = report["metrics"]["truth_minus_task_target_enu"]
        overall = dict(rms=static["rmse_3d_m"], peak=static["peak_3d_m"])
        decomposition = [report["metrics"][key]["rmse_3d_m"] for key in
            ("truth_minus_estimate_world_enu", "estimate_minus_executed_setpoint_world_enu", "executed_setpoint_minus_task_world_enu")]
    if (abs(overall["rms"]-meta["hover_metrics"]["position_rmse_m"]) > 1e-10
            or abs(overall["peak"]-meta["hover_metrics"]["position_max_m"]) > 1e-10):
        raise ValueError(folder.name+": full-resolution hover reports disagree")
    if "position_steps_audit" in data:
        steps = data["position_steps_audit"]
        if (steps["inputs_sha256"]["native"] != plot["source_csv_sha256"]
                or steps["inputs_sha256"]["metadata"] != plot["source_metadata_sha256"]
                or plot["actual_step_send_events"] != meta["position_step_send_events"]):
            raise ValueError(folder.name+": step-report/curve command source mismatch")
    rows = plot["samples"]
    t = np.asarray([row["sim_s"] for row in rows], float)
    position = np.asarray([row["base_pxyz"] for row in rows], float)
    if len(t) != plot["sample_count"] or not np.isfinite(position).all() or np.any(np.diff(t) <= 0):
        raise ValueError(folder.name+": invalid visualization rows")
    return dict(name=folder.name, folder=folder, target=target, t=t, position=position,
        static=overall, decomposition=decomposition, data=data,
        inputs_sha256={**{name: sha(folder/name) for name in names}, **extra_inputs})


def verify_comparison(gps, candidates):
    baseline = candidates[0]
    base_meta = baseline["data"]["controller_metadata"]
    for run in [gps, *candidates]:
        meta = run["data"]["controller_metadata"]
        for key in ("model_sha256", "source_urdf_sha256"):
            if meta[key] != base_meta[key]:
                raise ValueError("Comparison changes the physical model: "+key)
        if not np.allclose(run["target"], baseline["target"], rtol=0, atol=1e-6):
            raise ValueError("Comparison changes the fixed world anchor")
        params, base = meta["actual_pid_parameters"], base_meta["actual_pid_parameters"]
        if len(params) != 42 or set(params) != set(base):
            raise ValueError("Every run must identify the same 42 control parameters")
        allowed = () if run is gps else CANDIDATE_GAINS
        changed = [key for key in params if key not in allowed and
                   not np.isclose(float(params[key]), float(base[key]), rtol=2e-6, atol=1e-7)]
        if changed:
            raise ValueError("Unregistered control changes: "+str(changed))
    for candidate in candidates:
        if candidate["data"]["hover_reference"]["frame_mode"] != "raw_ev_world":
            raise ValueError("PID comparison requires the same explicit ideal-EV frame mode")


def configure():
    plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": ["SimHei", "DejaVu Sans"],
        "axes.unicode_minus": False, "axes.spines.top": False, "axes.spines.right": False,
        "font.size": 11, "axes.titlesize": 12, "figure.facecolor": "white", "pdf.fonttype": 42})


def save(fig, output, basename):
    paths = []
    for suffix in ("png", "pdf"):
        path = output/(basename+"."+suffix)
        fig.savefig(path, dpi=180, facecolor="white")
        paths.append(path)
    plt.close(fig)
    return paths


def localization_figure(gps, ev, output):
    fig = plt.figure(figsize=(13.8, 9.2))
    grid = fig.add_gridspec(2, 2, left=.075, right=.97, bottom=.14, top=.88, hspace=.43, wspace=.28)
    time_ax, xy_ax, bar_ax = fig.add_subplot(grid[0, :]), fig.add_subplot(grid[1, 0]), fig.add_subplot(grid[1, 1])
    labels = ["原 GPS/IMU + 原 PID", "理想 EV 位置/速度/高度 + 原 PID"]
    for run, label, color in zip((gps, ev), labels, (COLORS[1], COLORS[0])):
        meta = run["data"]["controller_metadata"]
        mask = (run["t"] >= meta["task_start_sim_s"]-1e-9)&(run["t"] <= meta["task_end_sim_s"]+1e-9)
        t, error = run["t"][mask]-meta["task_start_sim_s"], (run["position"][mask]-run["target"])*100
        # Norm is a pointwise visualization; all RMS/peak labels come from the
        # full-resolution audit reports above, never from these selected rows.
        time_ax.plot(t, np.linalg.norm(error, axis=1), lw=2., color=color,
            label=f"{label}：RMS {run['static']['rms']*100:.3f} cm，峰值 {run['static']['peak']*100:.3f} cm")
        xy_ax.plot(error[:, 0], error[:, 1], color=color, lw=1.9, label=label)
        xy_ax.scatter(error[0, 0], error[0, 1], color=color, s=25, zorder=4)
    time_ax.set(xlim=(0, 20), xlabel="悬停测量时间（s）", ylabel="真实三维位置误差（cm）",
                title="A  同一个预先声明的世界目标：20 s 静态悬停")
    time_ax.grid(alpha=.22); time_ax.legend(loc="upper left", frameon=True, fontsize=10)
    xy_ax.scatter(0, 0, marker="+", color="black", s=80, label="固定目标", zorder=5)
    xy_ax.set(xlabel="真实 X - 目标 X（cm）", ylabel="真实 Y - 目标 Y（cm）", title="B  水平轨迹：相对声明目标，不按实测位置居中")
    xy_ax.set_aspect("equal", adjustable="datalim"); xy_ax.grid(alpha=.22)
    inset = xy_ax.inset_axes([.60, .56, .36, .37])
    meta = ev["data"]["controller_metadata"]
    mask = ev["t"] <= meta["task_end_sim_s"]+1e-9
    error = (ev["position"][mask]-ev["target"])*100
    inset.plot(error[:, 0], error[:, 1], color=COLORS[0], lw=1.3)
    inset.scatter(0, 0, marker="+", color="black", s=25)
    inset.set(xlim=(-.3, .3), ylim=(-.3, .3), title="EV 放大（cm）")
    inset.tick_params(labelsize=8); inset.title.set_fontsize(9); inset.grid(alpha=.2)
    categories = ["真实-任务目标", "真实-估计", "估计-飞控目标", "飞控目标-任务目标"]
    x, width = np.arange(4), .35
    for offset, run, color in ((-width/2, gps, COLORS[1]), (width/2, ev, COLORS[0])):
        values = np.array([run["static"]["rms"], *run["decomposition"]])*100
        bars = bar_ax.bar(x+offset, values, width, color=color, alpha=.88)
        for bar, value in zip(bars, values):
            text = f"{value:.3f}" if value >= .001 else "<0.001"
            bar_ax.text(bar.get_x()+bar.get_width()/2, bar.get_height()+.035, text,
                        ha="center", va="bottom", fontsize=8.5, rotation=0)
    bar_ax.set(xticks=x, xticklabels=categories, ylabel="完整 250 Hz 报告：三维 RMS（cm）", title="C  误差向量链的描述性分解")
    bar_ax.tick_params(axis="x", labelrotation=12, labelsize=9)
    bar_ax.set_ylim(0, max(gps["static"]["rms"], *gps["decomposition"])*100*1.22)
    bar_ax.grid(axis="y", alpha=.22)
    fig.suptitle("X650：固定世界目标后，定位条件对悬停误差的影响", y=.963, fontsize=17)
    fig.text(.5, .915, "两轮使用相同 42 项 PID / 内环参数、同一物理模型；此图未调整 PID。每种条件当前各 1 轮。", ha="center", fontsize=11)
    fig.text(.075, .075, "理想 EV 使用仿真真值辅助 EKF（位置、速度、高度），关闭 GPS/气压高度融合；毫米结果不代表实机定位精度。", fontsize=10)
    fig.text(.075, .043, "曲线为原生行的 10 Hz 显示抽样；所有数值来自完整 250 Hz 报告。分解项 RMS 不能相加，也不能作为因果占比。", fontsize=10)
    return save(fig, output, "localization_fixed_target_zh")


def candidate_label(run):
    p = run["data"]["controller_metadata"]["actual_pid_parameters"]
    name = short_label(run)
    return (f"{name}：位置 P={p['MPC_XY_P']:.2g}/{p['MPC_Z_P']:.2g}，"
            f"速度 P={p['MPC_XY_VEL_P_ACC']:.2g}/{p['MPC_Z_VEL_P_ACC']:.2g}")


def short_label(run):
    return {"ev_repeat_baseline": "原 PID 复测", "ev_repeat_selected": "选中参数复测"}.get(
        run["name"], run["name"].removeprefix("ev_"))


def step_figures(candidates, output):
    fig, axes = plt.subplots(1, 3, figsize=(15.8, 6.1))
    fig.subplots_adjust(left=.065, right=.98, bottom=.27, top=.78, wspace=.25)
    for axis, ax in enumerate(axes):
        for run, color in zip(candidates, COLORS):
            rows = run["data"]["position_steps_audit"]["segments"]
            outward, returning = rows[axis*2:axis*2+2]
            start, end = outward["actual_window_sim_s"][0], returning["actual_window_sim_s"][1]
            mask = (run["t"] >= start-.2)&(run["t"] <= end+1e-9)
            t, displacement = run["t"][mask]-start, (run["position"][mask, axis]-run["target"][axis])*100
            ax.plot(t, displacement, color=color, lw=1.9, label=candidate_label(run))
            ret_time = returning["actual_window_sim_s"][0]-start
            amplitude = outward["commanded_step_m"]*100
            ax.step([-.2, 0, ret_time, end-start], [0, amplitude, 0, 0], where="post", color=color, alpha=.4, ls=":", lw=1.6)
        ax.set(xlabel="从该轴首次发出阶跃开始（s）", ylabel=f"真实 {'XYZ'[axis]} 相对固定锚点（cm）",
               title=f"{'ABC'[axis]}  {'XYZ'[axis]}：+{15 if axis < 2 else 10} cm，再返回原目标", xlim=(-.2, 12.04))
        ax.grid(alpha=.22)
    handles = [Line2D([], [], color=color, lw=2, label=candidate_label(run)) for run, color in zip(candidates, COLORS)]
    handles.append(Line2D([], [], color="#555555", ls=":", lw=1.7, label="目标（按各组实际首次发送边界）"))
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, .06), ncol=2, fontsize=10, frameon=False)
    fig.suptitle("X650 原生位置环：同一理想 EV 定位下的 PID 候选响应", y=.965, fontsize=17)
    fig.text(.5, .88, "单轴参考阶跃：X/Y 各 ±15 cm 增量，Z ±10 cm 增量；“负向”表示返回固定原目标。每段 6 s。", ha="center", fontsize=11)
    fig.text(.065, .018, "曲线仅作 10 Hz 显示抽样；每条曲线对应独立 1 轮位置参考响应，未注入外力，不能宣称全局最优。", fontsize=10)
    outputs = save(fig, output, "pid_step_response_zh")

    fig, (over_ax, settle_ax) = plt.subplots(1, 2, figsize=(15.2, 7.4))
    fig.subplots_adjust(left=.065, right=.98, bottom=.27, top=.79, wspace=.22)
    x, width = np.arange(6), .75/len(candidates)
    for index, (run, color) in enumerate(zip(candidates, COLORS)):
        rows = run["data"]["position_steps_audit"]["segments"]
        offset = (index-(len(candidates)-1)/2)*width
        overshoots = [row["overshoot_percent_of_commanded_step"] for row in rows]
        over_ax.bar(x+offset, overshoots, width, color=color, label=short_label(run))
        for xx, row in zip(x+offset, rows):
            result = row["settling"]
            confirmed = result["first_qualification_confirmed_time_s"]
            if confirmed is None:
                settle_ax.scatter(xx, 6.15, color=color, marker="x", s=65, linewidths=2)
                settle_ax.text(xx, 6.28, "未满足", ha="center", fontsize=8, rotation=90, color=color)
            else:
                entry = result["first_qualified_entry_time_s"]
                settle_ax.plot([xx, xx], [entry, confirmed], color=color, lw=2.1, alpha=.8)
                settle_ax.scatter(xx, entry, s=24, facecolors="white", edgecolors=color, zorder=4)
                settle_ax.scatter(xx, confirmed, s=38, color=color, zorder=4)
    labels = ["X 外移", "X 返回", "Y 外移", "Y 返回", "Z 上移", "Z 返回"]
    for ax in (over_ax, settle_ax):
        ax.set_xticks(x, labels); ax.grid(axis="y", alpha=.22)
    over_ax.set(ylabel="沿指令方向的超调 / 指令增量（%）", title="A  超调（完整原生数据）")
    over_ax.legend(frameon=False, ncol=min(3, len(candidates)))
    settle_ax.axhline(6., color="#555555", ls="--", lw=1.)
    settle_ax.set(ylabel="从实际发送开始的时间（s）", title="B  稳定进入时间 → 保持 2 s 后确认", ylim=(0, 7.1))
    settle_ax.legend(handles=[Line2D([], [], marker="o", color="#555555", markerfacecolor="white", lw=0, label="首次进入合格区间"),
        Line2D([], [], marker="o", color="#555555", lw=0, label="连续 2 s 后确认"),
        Line2D([], [], marker="x", color="#555555", lw=0, label="6 s 内无完整合格区间")], fontsize=9, frameon=False)
    fig.suptitle("PID 候选：超调与稳定条件分别比较", y=.965, fontsize=17)
    fig.text(.5, .88, "稳定条件：真实三维位置误差 ≤2 cm 且真实速度 ≤3 cm/s，连续至少 2 s；不是只看单轴位置曲线。", ha="center", fontsize=11)
    for index, run in enumerate(candidates):
        fig.text(.065+.46*(index % 2), .19-.035*(index//2), candidate_label(run), color=COLORS[index], fontsize=10)
    fig.text(.065, .047, "数值直接读取完整 250 Hz position_steps_audit.json；未从抽样曲线重算。未稳定不替换成 6 s 数值。", fontsize=10)
    fig.text(.065, .02, "空心点是进入时刻，实心点是经过 2 s 观察后的确认时刻；因此进入约 3 s，对应约 5 s 才完成稳定确认。", fontsize=10)
    outputs += save(fig, output, "pid_step_metrics_zh")
    return outputs


def render(input_dir, output, candidate_names):
    input_dir, output = Path(input_dir), Path(output)
    if output.exists():
        raise ValueError("Output exists; preserve previously delivered figures and choose a new directory")
    gps = load_run(input_dir/"gps_fixed_A1")
    candidates = [load_run(input_dir/name) for name in candidate_names]
    if not candidates or candidates[0]["name"] != "ev_S0" or len(candidates) > len(COLORS):
        raise ValueError("Provide ev_S0 first and at most eight explicitly delivered candidates/runs")
    verify_comparison(gps, candidates)
    configure(); output.mkdir(parents=True)
    files = localization_figure(gps, candidates[0], output)+step_figures(candidates, output)
    report = dict(kind="x650_delivered_precision_comparison_plot_audit", generator_sha256=sha(__file__),
        full_resolution_metrics_recomputed=False, curve_sampling_hz=10,
        curves_only_source="Selected original native rows from delivered plot_samples.json; no dynamics, re-centering or filtering",
        localization_comparison=dict(groups=[gps["name"], candidates[0]["name"]], same42_pid=True,
            simulation_truth_aided_ev=True, formal_gps_or_hardware_accuracy_claim=False, repeats_per_condition=1),
        pid_candidates=[run["name"] for run in candidates], optimal_pid_claimed=False,
        pid_values_source="Each EV run's metadata is checked against its measurement-period ULog 42-parameter audit; after-restore profiles are never substituted",
        step_type="Declared position-reference changes, not externally applied force disturbances",
        sources={run["name"]: dict(inputs_sha256=run["inputs_sha256"], hover_metrics=run["static"],
            rms_vector_chain=[run["static"]["rms"], *run["decomposition"]],
            full_experiment_ulog_guard_present="full_experiment_ulog_guard" in run["data"].get("precision_audit", {}))
            for run in [gps, *candidates]},
        outputs_sha256={path.name: sha(path) for path in files})
    (output/"comparison_plot_audit.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--candidates", nargs="+", default=["ev_S0", "ev_S1"])
    args = parser.parse_args()
    report = render(args.input, args.output, args.candidates)
    print(json.dumps({key: value for key, value in report.items() if key != "sources"}, indent=2))


if __name__ == "__main__":
    main()
