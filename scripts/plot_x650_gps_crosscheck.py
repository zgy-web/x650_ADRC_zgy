#!/usr/bin/env python3
"""Read-only GPS/WGS84 transfer plots from delivered audited reports.

This loader deliberately does not call the raw-EV loader or inverse helpers.
Native-only precision, full-experiment GPS guards, WGS84 decomposition and step
response reports are cross-bound to one original native/ULog/metadata source.
"""
from __future__ import annotations

import argparse
import importlib.util
import json
from pathlib import Path

import numpy as np


spec = importlib.util.spec_from_file_location("x650_gps_plot_style", Path(__file__).with_name("plot_x650_precision_comparison.py"))
style = importlib.util.module_from_spec(spec)
spec.loader.exec_module(style)
plt = style.plt
GPS_FLAGS = dict(cs_gnss_pos=1, cs_gnss_vel=1, cs_gps_hgt=1, cs_baro_hgt=1,
                 cs_ev_pos=0, cs_ev_vel=0, cs_ev_hgt=0)
FILES = dict(metadata="controller_metadata.json", task="controller_task.json", reference="hover_reference.json",
    manifest="native_trace_manifest.json", plot="plot_samples.json", precision="precision_audit.json",
    guard="gps_crosscheck_guard.json", chain="gps_chain_diagnostic.json", steps="position_steps_audit.json",
    status="analysis_status.json", independent="audit_summary.json")


def guard_source_hash(guard, filename):
    """Exclude identically named files from the older reference run."""
    ending = "/"+guard["source_run"]+"/"+filename
    matches = [value for key, value in guard["inputs_sha256"].items() if key.replace("\\", "/").endswith(ending)]
    if len(matches) != 1:
        raise ValueError("GPS guard must bind exactly one current-run source: "+filename)
    return matches[0]


def same_window(actual, desired, label):
    value = np.asarray(actual, float)
    if value.shape != (2,) or not np.isfinite(value).all() or not np.allclose(value, desired, rtol=0, atol=1e-8):
        raise ValueError("GPS source window differs: "+label)


def verify_gps_mode(documents):
    meta, ref, precision, guard = (documents[key] for key in ("metadata", "reference", "precision", "guard"))
    if (ref.get("frame_mode") != "global_wgs84" or precision.get("native_only") is not True
            or meta.get("ground_truth_aided") is not False or meta.get("aiding_confirmed") is not False
            or meta["task"].get("diagnostic_profile") is not None or guard.get("ground_truth_aided") is not False):
        raise ValueError("GPS transfer figures require unaided global-WGS84 runs, never raw-EV conversion")
    cfg = guard.get("configuration", {})
    if (guard.get("verified") is not True or cfg.get("verified") is not True
            or guard.get("all_42_boot_and_restored_parameters_match_original_gps") is not True
            or cfg.get("fusion_flags_entire_experiment") != GPS_FLAGS
            or cfg.get("origin_and_reset", {}).get("origin_and_reset_stable") is not True
            or cfg.get("effective_parameters", {}).get("parameter_changes_in_window") != []):
        raise ValueError("Full-experiment ordinary GPS fusion/origin/parameter guard is absent or failed")
    effective = cfg["effective_parameters"]["parameters_at_measurement_start"]
    expected = dict(EKF2_EV_CTRL=0, EKF2_GPS_CTRL=7, EKF2_BARO_CTRL=1, EKF2_HGT_REF=1)
    if any(effective.get(key) != value for key, value in expected.items()):
        raise ValueError("GPS effective estimator parameters differ from the declared condition")
    actual, declared = cfg.get("actual_control_42", {}), meta.get("actual_pid_parameters", {})
    if (len(actual) != 42 or set(actual) != set(declared)
            or any(not np.isclose(float(actual[key]), float(declared[key]), rtol=2e-6, atol=1e-7) for key in declared)):
        raise ValueError("GPS measurement-period ULog 42 gains differ from recorded metadata")


def load_gps_run(folder):
    folder = Path(folder)
    data = {key: style.load_json(folder/name) for key, name in FILES.items()}
    meta, ref, native, plot = (data[key] for key in ("metadata", "reference", "manifest", "plot"))
    precision, guard, chain, steps = (data[key] for key in ("precision", "guard", "chain", "steps"))
    if (meta.get("complete") is not True or meta.get("landed_and_disarmed") is not True
            or native.get("complete") is not True or data["status"].get("complete") is not True):
        raise ValueError("GPS transfer source is incomplete or unrecovered")
    verify_gps_mode(data)
    run_id = guard["source_run"]
    hover = [float(meta[key]) for key in ("task_start_sim_s", "task_end_sim_s")]
    experiment = [hover[0], float(meta["experiment_end_sim_s"])]
    if abs(hover[1]-hover[0]-20.) > .05 or abs(experiment[1]-experiment[0]-56.) > .05:
        raise ValueError("GPS comparison needs fixed 20 s hover plus six 6 s steps")
    for report in (precision, steps):
        if (report.get("source_run") != run_id or report["model_sha256"] != meta["model_sha256"]
                or report["source_urdf_sha256"] != meta["source_urdf_sha256"]
                or report["inputs_sha256"]["native"] != native["sha256"]
                or report["inputs_sha256"]["metadata"] != style.sha(folder/FILES["metadata"])):
            raise ValueError("GPS native reports differ in run/model/source identity")
    for key in ("model_sha256", "source_urdf_sha256"):
        if native[key] != meta[key] or guard[key] != meta[key]:
            raise ValueError("GPS guard/native model differs: "+key)
    for filename in ("controller_metadata.json", "controller_task.json", "hover_reference.json", "native_trace_manifest.json"):
        if guard_source_hash(guard, filename) != style.sha(folder/filename):
            raise ValueError("GPS guard does not bind delivered file: "+filename)
    if guard_source_hash(guard, "native_trace.csv") != native["sha256"]:
        raise ValueError("GPS guard binds a different original native CSV")
    for key, basename in (("metadata", "controller_metadata.json"), ("native", "native_trace.csv"), ("ulog", "px4.ulg")):
        source = chain["source_files"][key]
        if (source["sha256"] != guard_source_hash(guard, basename)
                or not source["path"].replace("\\", "/").endswith("/"+run_id+"/"+basename)):
            raise ValueError("WGS84 hover diagnostic differs from guarded original source: "+key)
    if (chain.get("world_chain_available") is not True or chain.get("uncertainties") != []
            or chain["world_datum"]["sha256"] != ref["world"]["source_sha256"]
            or chain["px4_source"].get("runtime_commit_verified") is not True
            or chain["time_alignment"].get("phase_fitted") is not False
            or chain["time_alignment"].get("origin_fitted") is not False):
        raise ValueError("GPS hover decomposition has an unverified WGS84 source/frame/clock")
    for key, desired in (("precision", hover), ("chain", hover)):
        same_window(data[key]["analysis_window_sim_s"], desired, key)
    for report, key in ((guard, "experiment_window_sim_s"), (steps, "experiment_window_sim_s"), (plot, "window_sim_s")):
        same_window(report[key], experiment, key)
    same_window(steps["primary_hover_window_sim_s"], hover, "step primary window")
    captured = np.asarray(native["task_window_sim_s"], float)
    if captured[0] > experiment[0]+1e-8 or captured[1] < experiment[1]-1e-8:
        raise ValueError("Native manifest does not cover GPS step experiment")
    if (plot.get("visualization_only") is not True or plot.get("metrics_must_use_complete_native_csv") is not True
            or plot["source_csv_sha256"] != native["sha256"]
            or plot["source_metadata_sha256"] != style.sha(folder/FILES["metadata"])
            or plot["source_native_manifest_sha256"] != style.sha(folder/FILES["manifest"])
            or plot["actual_step_send_events"] != meta["position_step_send_events"]):
        raise ValueError("GPS visualization rows differ from original native and actual command evidence")
    analysis_names = dict(gps_guard="guard", native_hover="precision", native_steps="steps", gps_chain="chain")
    found = set()
    for row in data["status"]["analyses"]:
        name = row["name"]
        if name not in analysis_names or row.get("exit_code") != 0 or row.get("verified") is not True:
            raise ValueError("GPS analysis lifecycle did not complete successfully")
        if row["sha256"] != style.sha(folder/FILES[analysis_names[name]]):
            raise ValueError("Delivered GPS report differs from completed analysis manifest")
        found.add(name)
    if found != set(analysis_names):
        raise ValueError("GPS analysis lifecycle lacks one of the four reports")
    target = np.asarray(ref["target_world_enu_m"], float)
    stat = precision["position_native_world_enu"]
    chain_stat = chain["metrics"]["truth_minus_task_target_enu"]
    if (abs(stat["rmse_3d"]-chain_stat["rmse_3d_m"]) > 1e-10
            or abs(stat["peak_3d"]-chain_stat["peak_3d_m"]) > 1e-10
            or not np.allclose(target, meta["hover_metrics"]["fixed_target_enu_m"], rtol=0, atol=1e-9)):
        raise ValueError("Independent GPS full-resolution hover metrics/target disagree")
    # A valid GPS run may fail the performance settling criterion. Do not reject
    # it or select only successful steps when preparing the comparison.
    rows = plot["samples"]
    t, p = np.asarray([row["sim_s"] for row in rows]), np.asarray([row["base_pxyz"] for row in rows])
    if not np.isfinite(p).all() or len(t) != plot["sample_count"] or np.any(np.diff(t) <= 0):
        raise ValueError("GPS visualization rows are not finite and ordered")
    return dict(name=folder.name, folder=folder, data=data, target=target, t=t, position=p,
        input_sha256={filename: style.sha(folder/filename) for filename in FILES.values()})


def gains(run):
    return run["data"]["guard"]["configuration"]["actual_control_42"]


def verify_pair(baseline, selected, frozen):
    a, b = baseline["data"], selected["data"]
    for key in ("model_sha256", "source_urdf_sha256"):
        if a["metadata"][key] != b["metadata"][key]:
            raise ValueError("GPS comparison changes physical model")
    if (a["reference"]["world"]["source_sha256"] != b["reference"]["world"]["source_sha256"]
            or not np.allclose(baseline["target"], selected["target"], rtol=0, atol=1e-6)):
        raise ValueError("GPS comparison changes world datum or declared fixed anchor")
    original, chosen = gains(baseline), gains(selected)
    if set(original) != set(chosen) or any(not np.isclose(original[key], chosen[key], rtol=2e-6, atol=1e-7)
                                        for key in original if key not in style.CANDIDATE_GAINS):
        raise ValueError("GPS comparison changes control gains outside the four registered candidates")
    for key, desired in frozen["selected_parameters"].items():
        if not np.isclose(chosen[key], desired, rtol=2e-6, atol=1e-7):
            raise ValueError("GPS selected condition differs from the frozen EV selection")


def label(run, selected=False):
    p = gains(run)
    return ("已冻结选中参数" if selected else "原 PID")+f"（位置 P {p['MPC_XY_P']:.2g}/{p['MPC_Z_P']:.2g}，速度 P {p['MPC_XY_VEL_P_ACC']:.2g}/{p['MPC_Z_VEL_P_ACC']:.2g}）"


def hover_figure(runs, output):
    fig, (time_ax, chain_ax) = plt.subplots(1, 2, figsize=(15., 7.2), gridspec_kw={"width_ratios": [1.15, 1.]})
    fig.subplots_adjust(left=.065, right=.98, bottom=.24, top=.77, wspace=.23)
    x, width = np.arange(4), .35
    for index, (run, color) in enumerate(zip(runs, (style.COLORS[0], style.COLORS[2]))):
        meta, chain = run["data"]["metadata"], run["data"]["chain"]
        mask = (run["t"] >= meta["task_start_sim_s"]-1e-9)&(run["t"] <= meta["task_end_sim_s"]+1e-9)
        t, error = run["t"][mask]-meta["task_start_sim_s"], run["position"][mask]-run["target"]
        time_ax.plot(t, np.linalg.norm(error, axis=1)*100, color=color, lw=2.)
        metrics = chain["metrics"]
        values = [metrics[key]["rmse_3d_m"]*100 for key in ("truth_minus_task_target_enu", "truth_minus_estimate_world_enu",
            "estimate_minus_executed_setpoint_world_enu", "executed_setpoint_minus_task_world_enu")]
        bars = chain_ax.bar(x+(index-.5)*width, values, width, color=color)
        for bar, value in zip(bars, values):
            chain_ax.text(bar.get_x()+bar.get_width()/2, value+.07, f"{value:.3f}" if value >= .001 else "<0.001", ha="center", fontsize=8)
    time_ax.set(title="A  固定目标：20 s 真实三维误差", xlabel="从悬停测量开始（s）", ylabel="真实三维位置误差（cm）", xlim=(0, 20), ylim=(0, None))
    chain_ax.set(title="B  悬停误差向量链：完整报告 RMS", ylabel="三维 RMS（cm）", xticks=x,
        xticklabels=["真实-任务目标", "真实-估计", "估计-飞控目标", "飞控目标-任务目标"])
    chain_ax.tick_params(axis="x", labelsize=9, labelrotation=12)
    chain_ax.margins(y=.22)
    for ax in (time_ax, chain_ax):
        ax.grid(axis="y", alpha=.22)
    handles = []
    for index, (run, color) in enumerate(zip(runs, (style.COLORS[0], style.COLORS[2]))):
        metric = run["data"]["precision"]["position_native_world_enu"]
        handles.append(style.Line2D([], [], color=color, lw=2, label=label(run, index == 1)+f"：RMS {metric['rmse_3d']*100:.3f} cm，峰值 {metric['peak_3d']*100:.3f} cm"))
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, .105), fontsize=10, frameon=False)
    fig.suptitle("X650 原 GPS/IMU 条件：已冻结 PID 参数的迁移检查", y=.96, fontsize=17)
    fig.text(.5, .86, "两轮均为原 GPS/气压高度融合，未使用真值 EV；固定世界目标使用 WGS84 转换。每种增益各 1 轮。", ha="center", fontsize=11)
    fig.text(.065, .043, "曲线为原生行的 10 Hz 显示抽样；数值来自完整 250 Hz 报告。RMS 分解项不能相加或用于因果占比。", fontsize=10)
    fig.text(.065, .018, "两轮间的静态误差差异不能直接归因于 PID 改动；该图不声称定位精度改善或统计显著性。", fontsize=10)
    return style.save(fig, output, "gps_hover_transfer_zh")


def step_figures(runs, output):
    colors = (style.COLORS[0], style.COLORS[2])
    fig, axes = plt.subplots(1, 3, figsize=(15.8, 6.2))
    fig.subplots_adjust(left=.065, right=.98, bottom=.27, top=.77, wspace=.25)
    for axis, ax in enumerate(axes):
        for run, color in zip(runs, colors):
            outward, returning = run["data"]["steps"]["segments"][axis*2:axis*2+2]
            start, end = outward["actual_window_sim_s"][0], returning["actual_window_sim_s"][1]
            mask = (run["t"] >= start-.2)&(run["t"] <= end+1e-9)
            ax.plot(run["t"][mask]-start, (run["position"][mask, axis]-run["target"][axis])*100, lw=2., color=color)
            ax.step([-.2, 0., returning["actual_window_sim_s"][0]-start, end-start],
                [0., outward["commanded_step_m"]*100, 0., 0.], where="post", color=color, ls=":", alpha=.5)
        ax.set(title=f"{'ABC'[axis]}  {'XYZ'[axis]}：外移，再回原目标", xlabel="从该轴首次发出指令开始（s）",
            ylabel=f"真实 {'XYZ'[axis]} 相对固定锚点（cm）", xlim=(-.2, 12.04))
        ax.grid(alpha=.22)
    handles = [style.Line2D([], [], color=color, lw=2, label=label(run, i == 1)) for i, (run, color) in enumerate(zip(runs, colors))]
    handles.append(style.Line2D([], [], color="#555555", ls=":", lw=1.5, label="目标：各组实际首次发送边界"))
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, .065), frameon=False, fontsize=10)
    fig.suptitle("原 GPS 条件下的六段位置参考响应", y=.96, fontsize=17)
    fig.text(.5, .86, "X/Y 各 +15 cm 后返回，Z +10 cm 后返回；每段 6 s。位置参考变化，未注入外力。", ha="center", fontsize=11)
    fig.text(.065, .02, "曲线仅作 10 Hz 显示抽样；未达到 2 cm / 3 cm/s 连续 2 s 的稳定条件，是有效性能结果，不能作为剔除该轮的理由。", fontsize=10)
    outputs = style.save(fig, output, "gps_step_response_zh")
    fig, (over, settle) = plt.subplots(1, 2, figsize=(15., 7.2))
    fig.subplots_adjust(left=.065, right=.98, bottom=.25, top=.77, wspace=.24)
    x, width = np.arange(6), .32
    for index, (run, color) in enumerate(zip(runs, colors)):
        rows = run["data"]["steps"]["segments"]
        xx = x+(index-.5)*width
        over.bar(xx, [row["overshoot_m"]*100 for row in rows], width, color=color, label="选中参数" if index else "原 PID")
        for position, row in zip(xx, rows):
            value = row["settling"]
            entry, confirmed = value["first_qualified_entry_time_s"], value["first_qualification_confirmed_time_s"]
            if entry is None:
                settle.scatter(position, 6.12, marker="x", s=70, color=color)
                settle.text(position, 6.26, "未满足", ha="center", color=color, rotation=90, fontsize=8)
            else:
                settle.plot([position, position], [entry, confirmed], lw=2, color=color)
                settle.scatter(position, entry, s=28, facecolors="white", edgecolors=color, zorder=4)
                settle.scatter(position, confirmed, s=42, color=color, zorder=4)
    labels = ["X 外移", "X 返回", "Y 外移", "Y 返回", "Z 上移", "Z 返回"]
    for ax in (over, settle):
        ax.set_xticks(x, labels); ax.grid(axis="y", alpha=.22)
    over.set(title="A  指令方向上的超调", ylabel="完整报告：超调（cm）")
    over.legend(frameon=False)
    settle.axhline(6., ls="--", color="#555555", lw=1.)
    settle.set(title="B  稳定条件：进入 → 保持 2 s 后确认", ylabel="从实际发送开始（s）", ylim=(0, 7.1))
    fig.suptitle("原 GPS 条件：超调与未稳定步骤全部保留", y=.96, fontsize=17)
    fig.text(.5, .86, "合格区间要求真实三维位置误差 ≤2 cm 且真实速度 ≤3 cm/s，连续至少 2 s。", ha="center", fontsize=11)
    fig.legend(handles=[style.Line2D([], [], marker="o", markerfacecolor="white", color="#555555", lw=0, label="空心：进入合格区间"),
        style.Line2D([], [], marker="o", color="#555555", lw=0, label="实心：2 s 后确认"),
        style.Line2D([], [], marker="x", color="#555555", lw=0, label="叉号：该段未观察到完整合格区间")], loc="lower center", bbox_to_anchor=(.5, .105), frameon=False, fontsize=10)
    fig.text(.065, .047, "所有数值来自完整原生报告，未从抽样曲线重算。未满足条件不记为 6 s，也不判作无效数据。", fontsize=10)
    fig.text(.065, .02, "每种 GPS 增益各 1 轮；定位状态、噪声和响应耦合存在，本对照不重新挑选参数，不声称统计显著性。", fontsize=10)
    outputs += style.save(fig, output, "gps_step_metrics_zh")
    return outputs


def summary(run):
    physical = run["data"]["precision"]["position_native_world_enu"]
    chain = run["data"]["chain"]["metrics"]
    rows = run["data"]["steps"]["segments"]
    return dict(hover_rms_m=physical["rmse_3d"], hover_peak_m=physical["peak_3d"],
        hover_axis_rms_m=physical["axis_rmse"],
        hover_vector_chain_rms_m={key: chain[key]["rmse_3d_m"] for key in ("truth_minus_estimate_world_enu",
            "estimate_minus_executed_setpoint_world_enu", "executed_setpoint_minus_task_world_enu")},
        step_qualified_count=sum(row["settling"]["settled_2s_observed"] for row in rows), step_count=len(rows),
        steps=[dict(id=row["segment_id"], overshoot_m=row["overshoot_m"], rms_3d_m=row["position_error_world_enu"]["rmse_3d"],
            entry_s=row["settling"]["first_qualified_entry_time_s"], confirmed_s=row["settling"]["first_qualification_confirmed_time_s"],
            remained_qualified_to_end=row["settling"]["settled_through_window_end"]) for row in rows])


def render(input_dir, output, baseline_name="gps_repeat_baseline", selected_name="gps_repeat_selected"):
    folder, output = Path(input_dir), Path(output)
    if output.exists():
        raise ValueError("Output exists; preserve previous evidence and use a new directory")
    runs = [load_gps_run(folder/name) for name in (baseline_name, selected_name)]
    frozen_path = folder/"selection_frozen.json"
    frozen = style.load_json(frozen_path)
    verify_pair(*runs, frozen)
    style.configure(); output.mkdir(parents=True)
    files = hover_figure(runs, output)+step_figures(runs, output)
    report = dict(kind="x650_original_gps_frozen_pid_transfer_plot_audit", generator_sha256=style.sha(__file__),
        selection_frozen_sha256=style.sha(frozen_path), independent_global_wgs84_loader=True,
        raw_ev_inverse_used=False, ulog_reparsed=False, server_or_parameters_modified=False,
        full_experiment_source_guard_verified=True, same_model_world_anchor=True,
        step_settling_is_performance_not_data_validity=True, metric_recomputed_from_decimated_curves=False,
        trials_per_gain=1, statistical_significance_claimed=False, localization_gain_causality_claimed=False,
        reselected_using_gps=False, results={run["name"]: summary(run) for run in runs},
        inputs_sha256={run["name"]: run["input_sha256"] for run in runs},
        outputs_sha256={path.name: style.sha(path) for path in files})
    (output/"gps_transfer_plot_audit.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=style.DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--baseline", default="gps_repeat_baseline")
    parser.add_argument("--selected", default="gps_repeat_selected")
    args = parser.parse_args()
    print(json.dumps(render(args.input, args.output, args.baseline, args.selected), indent=2))


if __name__ == "__main__":
    main()
