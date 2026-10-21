#!/usr/bin/env python3
"""Plot frozen PID selection against independent repeats, without re-selecting.

Reads only delivered completed runs. The holdout runs must start after the
selection document was frozen and must match the original/selected 42 gains.
No performance metric is computed from visualization samples.
"""
from __future__ import annotations

import argparse
from datetime import datetime
import importlib.util
import json
from pathlib import Path

import numpy as np


spec = importlib.util.spec_from_file_location("x650_precision_plot_sources", Path(__file__).with_name("plot_x650_precision_comparison.py"))
plots = importlib.util.module_from_spec(spec)
spec.loader.exec_module(plots)
plt = plots.plt


def verify_holdout(frozen, baseline, selected, repeats, manifests):
    if (frozen.get("selection_frozen_before_holdout") is not True or frozen.get("holdout_runs_used") != []
            or frozen.get("global_optimality_claimed") is not False):
        raise ValueError("Selection was not explicitly frozen before independent repeats")
    frozen_time = datetime.fromisoformat(frozen["created_utc"]).timestamp()
    chosen = [row for row in frozen["rows"] if row["candidate"] == frozen["selected_candidate"]]
    if len(chosen) != 1 or chosen[0]["label"] != selected["name"]:
        raise ValueError("Selected screening run differs from the frozen decision")
    for key, value in frozen["selected_parameters"].items():
        actual = selected["data"]["controller_metadata"]["actual_pid_parameters"][key]
        if not np.isclose(float(actual), float(value), rtol=2e-6, atol=1e-7):
            raise ValueError("Selected screening gains differ from frozen selected parameters")
    for original, repeat, manifest in zip((baseline, selected), repeats, manifests):
        if float(manifest["started_wall_time"]) <= frozen_time or manifest.get("scenario_returncode") != 0:
            raise ValueError("Holdout must be an independent completed run started after selection freeze")
        desired = original["data"]["controller_metadata"]["actual_pid_parameters"]
        actual = repeat["data"]["controller_metadata"]["actual_pid_parameters"]
        if set(actual) != set(desired) or any(not np.isclose(float(actual[key]), float(desired[key]), rtol=2e-6, atol=1e-7) for key in desired):
            raise ValueError("Holdout's 42 measurement-period gains differ from its assigned PID condition")
    for run in (baseline, selected, *repeats):
        guard = run["data"]["precision_audit"].get("full_experiment_ulog_guard", {})
        if guard.get("verified") is not True:
            raise ValueError("Every holdout comparison run requires the complete-experiment ULog guard")
        desired = [run["data"]["controller_metadata"][key] for key in ("task_start_sim_s", "experiment_end_sim_s")]
        if not np.allclose(guard["analysis_window_sim_s"], desired, rtol=0, atol=1e-8):
            raise ValueError("Full-experiment ULog guard does not cover appended steps")
    return dict(independent_after_selection_freeze=True, frozen_utc=frozen["created_utc"],
                selected_candidate_unchanged=frozen["selected_candidate"], reselected_using_holdout=False,
                matched_measurement_period_42_gains=True, entire_experiment_ulog_guards=True)


def summary(run):
    rows = run["data"]["position_steps_audit"]["segments"]
    all_settled = all(row["settling"]["settled_2s_observed"] for row in rows)
    entries = [row["settling"]["first_qualified_entry_time_s"] for row in rows]
    return dict(hover_rms_mm=run["static"]["rms"]*1000, hover_peak_mm=run["static"]["peak"]*1000,
        horizontal_overshoot_max_cm=max(row["overshoot_m"] for row in rows[:4])*100,
        all_6_settled=all_settled, worst_qualified_entry_s=max(entries) if all_settled else None,
        scope="Summary of original full-resolution report metrics; not recalculated from selected plot rows")


def figure(baseline, selected, repeats, summaries, output):
    fig, axes = plt.subplots(1, 3, figsize=(15., 7.))
    fig.subplots_adjust(left=.065, right=.98, bottom=.25, top=.77, wspace=.28)
    colors = (plots.COLORS[0], plots.COLORS[2])
    group_names = ["原 PID", "已冻结选中参数"]
    for condition, (screen, repeat, color) in enumerate(zip((baseline, selected), repeats, colors)):
        for run, offset, filled in ((screen, -.10, False), (repeat, .10, True)):
            values = summaries[run["name"]]
            x = condition+offset
            for key, marker in (("hover_rms_mm", "o"), ("hover_peak_mm", "D")):
                axes[0].scatter(x, values[key], s=58, marker=marker, facecolors=color if filled else "white", edgecolors=color, linewidths=1.7, zorder=4)
                axes[0].text(x, values[key]+.065, f"{values[key]:.2f}", ha="center", fontsize=8)
            axes[1].scatter(x, values["horizontal_overshoot_max_cm"], s=64, facecolors=color if filled else "white", edgecolors=color, linewidths=1.7, zorder=4)
            axes[1].text(x, values["horizontal_overshoot_max_cm"]+.08, f"{values['horizontal_overshoot_max_cm']:.2f}", ha="center", fontsize=9)
            if values["worst_qualified_entry_s"] is None:
                axes[2].scatter(x, 6.1, marker="x", color=color, s=65)
                axes[2].text(x, 6.25, "有步骤未满足", ha="center", fontsize=8, rotation=90)
            else:
                value = values["worst_qualified_entry_s"]
                axes[2].scatter(x, value, s=64, facecolors=color if filled else "white", edgecolors=color, linewidths=1.7, zorder=4)
                axes[2].text(x, value+.08, f"{value:.3f}", ha="center", fontsize=9)
    for ax in axes:
        ax.set_xticks([0, 1], group_names); ax.set_xlim(-.45, 1.45); ax.grid(axis="y", alpha=.22)
    axes[0].set(title="A  固定目标 20 s 悬停", ylabel="完整原生报告：三维位置误差（mm）",
                ylim=(0, max(row["hover_peak_mm"] for row in summaries.values())*1.24))
    axes[1].axhline(2., ls="--", color="#555555", lw=1.2, label="预声明限值 2 cm")
    axes[1].set(title="B  四个水平步骤中最大超调", ylabel="沿指令方向超调（cm）",
                ylim=(0, max(2., max(row["horizontal_overshoot_max_cm"] for row in summaries.values()))*1.25))
    axes[1].legend(frameon=False, fontsize=9)
    axes[2].set(title="C  六个步骤中最慢稳定进入时间", ylabel="从实际指令发送开始（s）", ylim=(0, 6.8))
    axes[2].text(.03, .03, "进入条件后仍需连续保持 2 s\n才能完成稳定确认。", transform=axes[2].transAxes, fontsize=9)
    handles = [plots.Line2D([], [], marker="o", color="#555555", markerfacecolor="white", lw=0, label="空心：筛选时那一轮"),
        plots.Line2D([], [], marker="o", color="#555555", lw=0, label="实心：选参冻结后的独立复测"),
        plots.Line2D([], [], marker="o", color="#555555", lw=0, label="圆点：悬停 RMS"),
        plots.Line2D([], [], marker="D", color="#555555", lw=0, label="菱形：悬停峰值")]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.5, .12), ncol=2, frameon=False, fontsize=10)
    fig.suptitle("X650：冻结选参后的独立复测", y=.96, fontsize=17)
    fig.text(.5, .86, "原 PID 与选中参数各展示筛选轮 + 独立复测，共 2 轮；复测不参与重新挑选参数。", ha="center", fontsize=11)
    fig.text(.065, .068, plots.candidate_label(baseline)+"；  "+plots.candidate_label(selected), fontsize=10)
    fig.text(.065, .025, "理想 EV 位置/速度/高度辅助仿真，机械臂保持折叠；n=2 是重复性检查，不是统计显著性或全局最优证明。", fontsize=10)
    return plots.save(fig, output, "pid_frozen_selection_holdout_zh")


def render(input_dir, output):
    folder, output = Path(input_dir), Path(output)
    if output.exists():
        raise ValueError("Output exists; preserve prior figures and choose a new directory")
    frozen_path = folder/"selection_frozen.json"
    frozen = plots.load_json(frozen_path)
    chosen = [row for row in frozen["rows"] if row["candidate"] == frozen["selected_candidate"]]
    if len(chosen) != 1:
        raise ValueError("Need one frozen selected screening run")
    gps, baseline, selected = [plots.load_run(folder/name) for name in ("gps_fixed_A1", "ev_S0", chosen[0]["label"])]
    repeats = [plots.load_run(folder/name) for name in ("ev_repeat_baseline", "ev_repeat_selected")]
    manifest_paths = [run["folder"]/"run_manifest.json" for run in repeats]
    manifests = [plots.load_json(path) for path in manifest_paths]
    plots.verify_comparison(gps, [baseline, selected, *repeats])
    verification = verify_holdout(frozen, baseline, selected, repeats, manifests)
    runs = [baseline, selected, *repeats]
    summaries = {run["name"]: summary(run) for run in runs}
    plots.configure(); output.mkdir(parents=True)
    paths = figure(baseline, selected, repeats, summaries, output)
    report = dict(kind="x650_frozen_pid_selection_holdout_plot_audit", generator_sha256=plots.sha(__file__),
        shared_plot_source_sha256=plots.sha(Path(__file__).with_name("plot_x650_precision_comparison.py")),
        selection_frozen_sha256=plots.sha(frozen_path), verification=verification,
        repeats_per_condition=2, statistical_significance_claimed=False, optimality_claimed=False,
        full_resolution_metrics_recomputed_from_plot_samples=False, metrics=summaries,
        sources={run["name"]: run["inputs_sha256"] for run in runs},
        holdout_run_manifests_sha256={run["name"]: plots.sha(path) for run, path in zip(repeats, manifest_paths)},
        output_sha256={path.name: plots.sha(path) for path in paths})
    deployment_path = repeats[1]["folder"]/"ground_wait_fix_deployment.json"
    if deployment_path.exists():
        report["selected_repeat_ground_wait_fix"] = dict(source_sha256=plots.sha(deployment_path),
            semantics="Deployment evidence for the preflight ground-readiness wait fix; measured 42 PID values are still audited against the frozen selected condition")
    (output/"holdout_plot_audit.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=plots.DEFAULT_INPUT)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    print(json.dumps(render(args.input, args.output), indent=2))


if __name__ == "__main__":
    main()
