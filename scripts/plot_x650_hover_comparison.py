#!/usr/bin/env python3
"""Four-run descriptive hover comparison; never claim a PID repair or significance."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np

from plot_x650_hover_diagnosis import load_inputs, sha


RUNS = (("baseline", "A1", "原 GPS/IMU"), ("baseline_repeat", "A2", "原配置复测"),
        ("ideal_horizontal_1", "B1", "增加理想水平辅助"), ("ideal_horizontal_2", "B2", "辅助配置复测"))


def metadata_file(directory, report, baseline_metadata):
    choices = [directory/"evidence/controller_metadata.json", directory/"controller_metadata.json"]
    if directory.name == "baseline":
        choices.append(baseline_metadata)
    expected = report["source_files"]["metadata"]["sha256"]
    return next((p for p in choices if p is not None and p.is_file() and sha(p) == expected), None)


def load_runs(root, baseline_metadata, allow_incomplete):
    loaded, absent = [], []
    for folder, label, name in RUNS:
        directory = root/folder
        if not (directory/"diagnostic.json").exists():
            absent.append(label)
            continue
        report = json.loads((directory/"diagnostic.json").read_text(encoding="utf-8"))
        path = metadata_file(directory, report, baseline_metadata)
        if path is None:
            raise ValueError(label+" lacks the exact source metadata needed for the fixed target")
        metadata = json.loads(path.read_text(encoding="utf-8"))
        if metadata.get("complete") is not True or metadata.get("stage") != "hover_folded":
            raise ValueError(label+" is not a complete hover diagnostic")
        if metadata.get("allocator_active") is not False or metadata.get("controller") != "pid_ff":
            raise ValueError(label+" does not use the common commissioning controller")
        aided = label.startswith("B")
        if aided:
            session = metadata.get("diagnostic_parameters", {})
            profile = session.get("profile", {})
            if (metadata.get("ground_truth_aided") is not True or metadata.get("aiding_confirmed") is not True
                    or metadata.get("diagnostic_only") is not True or metadata.get("eligible_as_stage_gate") is not False
                    or session.get("applied_verified") is not True or session.get("restored_verified") is not True
                    or profile.get("EKF2_EV_CTRL") != 1 or profile.get("EKF2_EV_NOISE_MD") != 1
                    or abs(profile.get("EKF2_EVP_NOISE", -1)-.01) > 1e-8):
                raise ValueError(label+" lacks confirmed/restored ideal horizontal-aiding evidence")
        elif metadata.get("ground_truth_aided") is True or metadata.get("task", {}).get("diagnostic_profile"):
            raise ValueError(label+" is not an original GPS/IMU run")
        if report["selected_initial_parameters"].get("EKF2_GPS_CTRL") != 7:
            raise ValueError(label+" does not retain the common GPS fusion configuration")
        data = load_inputs(directory, path)
        report, t, truth, estimate, setpoint, vectors, audit = data
        if abs(t[-1]-20.) > 1e-8:
            raise ValueError(label+" does not have the common complete 20s measurement window")
        loaded.append(dict(label=label, name=name, folder=folder, report=report, metadata=metadata,
                           truth=truth, estimate=estimate, setpoint=setpoint, audit=audit))
    if absent and not allow_incomplete:
        raise ValueError("Final four-run comparison needs the missing raw evidence: "+", ".join(absent))
    if not loaded:
        raise ValueError("No verified hover runs found")
    first = loaded[0]["metadata"]
    pid = first["actual_pid_parameters"]
    if not pid:
        raise ValueError("Actual PID readback is empty")
    for item in loaded:
        other = item["metadata"]
        if (other["actual_pid_parameters"] != pid or other["model_sha256"] != first["model_sha256"]
                or other["source_urdf_sha256"] != first["source_urdf_sha256"]):
            raise ValueError(item["label"]+" differs in aircraft model or actual MC/MPC parameters")
    return loaded, absent


def plot(root, baseline_metadata, output, font, allow_incomplete):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import font_manager
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    from matplotlib.patches import Patch
    loaded, absent = load_runs(Path(root).resolve(strict=True), baseline_metadata, allow_incomplete)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output/"hover_comparison_zh.png").exists() or (output/"comparison_audit.json").exists():
        raise ValueError("Comparison output exists; preserve it and use a new directory")
    font = Path(font).resolve(strict=True)
    font_manager.fontManager.addfont(str(font))
    plt.rcParams.update({"font.sans-serif": [font_manager.FontProperties(fname=str(font)).get_name()],
                         "axes.unicode_minus": False, "font.size": 11, "pdf.fonttype": 42})
    colors = {"A1": "#186aa8", "A2": "#4c9bbf", "B1": "#d2761d", "B2": "#e7a14a"}
    linestyles = {"A1": "-", "A2": "--", "B1": "-", "B2": "--"}
    mapping = {item["label"]: item for item in loaded}
    fig, (bar, xy) = plt.subplots(1, 2, figsize=(14, 8.3))
    fig.subplots_adjust(left=.075, right=.98, top=.78, bottom=.35, wspace=.23)
    state = "原始记录齐全：每组各 2 次" if not absent else "阶段预览：仅使用已归档证据；"+"、".join(absent)+" 待证据"
    fig.suptitle("悬停复测：原 GPS/IMU 与增加理想水平位置辅助", y=.965, fontsize=19)
    fig.text(.52, .906, "A：原 GPS/IMU 融合    B：增加理想水平位置辅助，仍融合 GPS", ha="center", fontsize=13)
    fig.text(.52, .858, state+"  |  各测量 20 s；PID 实参和机体模型一致", ha="center", color="#52606f", fontsize=11)
    maximum = max(item["report"]["metrics"]["truth_minus_task_target_enu"]["peak_3d_m"]*100 for item in loaded)
    for i, (_, label, _) in enumerate(RUNS):
        if label not in mapping:
            bar.text(i, maximum*.36, "待归档\n不填数值", ha="center", color="#87929a", fontsize=11)
            continue
        metric = mapping[label]["report"]["metrics"]["truth_minus_task_target_enu"]
        values = [metric["rmse_3d_m"]*100, metric["peak_3d_m"]*100]
        for delta, value, hatch, alpha in zip((-.18, .18), values, (None, "///"), (1., .40)):
            bar.bar(i+delta, value, .32, color=colors[label], alpha=alpha, edgecolor=colors[label], hatch=hatch)
            bar.text(i+delta, value+.16, f"{value:.2f}", ha="center", fontsize=10, color="#26313d")
    bar.set_xticks(range(4), ["A1\n原配置", "A2\n原配置复测", "B1\n水平辅助", "B2\n水平辅助复测"])
    bar.set_ylabel("相对固定预设世界目标的三维误差（cm）")
    bar.set_ylim(0, maximum*1.23)
    bar.set_xlim(-.6, 3.6)
    bar.set_title("A  每次实验单独报告，不做改善百分比", loc="left", pad=13)
    bar.legend(handles=[Patch(facecolor="#546375", label="3D RMSE"), Patch(facecolor="#c9ced5", hatch="///", edgecolor="#546375", label="3D 峰值")], fontsize=10)
    for item in loaded:
        label = item["label"]
        truth, setpoint = item["truth"]*100, item["setpoint"]*100
        xy.plot(truth[:, 0], truth[:, 1], color=colors[label], ls=linestyles[label], lw=1.8, label=label+" 真实轨迹")
        xy.scatter(*setpoint[0, :2], marker="*", s=120, color=colors[label], edgecolor="white", lw=.5, zorder=4)
    xy.scatter(0, 0, marker="+", s=160, color="#172333", lw=2, zorder=5)
    xy.set_aspect("equal", adjustable="datalim")
    xy.set_title("B  水平真实轨迹：都相对各自固定世界目标", loc="left", pad=13)
    xy.set_xlabel("东向 X（cm）")
    xy.set_ylabel("北向 Y（cm）")
    handles, labels = xy.get_legend_handles_labels()
    handles += [Line2D([], [], color="#172333", marker="+", ls="", ms=10), Line2D([], [], color="#778491", marker="*", ls="", ms=11)]
    labels += ["预设世界目标", "实际飞控目标（星号）"]
    # Keep the legend outside the data: the B2 upper loop otherwise sits under it.
    fig.legend(handles, labels, fontsize=9, ncol=2, loc="center", bbox_to_anchor=(.75, .265), frameon=False)
    for axis in (bar, xy):
        axis.grid(alpha=.16, axis="y" if axis is bar else "both")
        axis.set_axisbelow(True)
        axis.spines[["top", "right"]].set_visible(False)
    fig.text(.52, .204, "这是定位辅助的诊断对照，不是纯真值定位，也不是 PID 修复或 MPC 性能验证。", ha="center", color="#5b352b", fontsize=12)
    fig.text(.52, .157, "EVP_NOISE 设置为 0.01 m，不等于实际按 1 cm 噪声融合：仍有 GPS 噪声下限和 EV 偏置估计。", ha="center", fontsize=11)
    sample_note = ("每组仅两次的描述性复测；没有统计显著性结论，不把单次误差变化当作已解决问题。" if not absent
                   else f"当前仅归档 {len(loaded)} 次，计划每组两次；尚未完成全部复测，不把单次误差变化当作已解决问题。")
    fig.text(.52, .112, sample_note, ha="center", fontsize=11)
    fig.text(.52, .062, "指标来自完整 250 Hz 原生记录；轨迹为显示抽样，无原点/相位/时延拟合。图中未把不同 RMS 相加或作因果分摊。", ha="center", color="#52606f", fontsize=9)
    png, pdf = output/"hover_comparison_zh.png", output/"hover_comparison_zh.pdf"
    fig.savefig(png, dpi=180)
    fig.savefig(pdf)
    plt.close(fig)
    first = loaded[0]["metadata"]
    pid_digest = hashlib.sha256(json.dumps(first["actual_pid_parameters"], sort_keys=True, separators=(",", ":")).encode()).hexdigest()
    report = dict(kind="x650_descriptive_hover_repeat_comparison", preliminary=bool(absent),
        expected_runs=[label for _, label, _ in RUNS], absent_runs=absent,
        new_dynamics_simulated=False, pid_tuning_changed=False, repair_claimed=False,
        statistical_significance_claimed=False, percentage_improvement_claimed=False,
        common_model_sha256=first["model_sha256"], common_source_urdf_sha256=first["source_urdf_sha256"],
        common_actual_pid_parameters_sha256=pid_digest, common_actual_pid_parameter_count=len(first["actual_pid_parameters"]),
        group_definitions={"A": "Original GPS/IMU configuration", "B": "Add ideal horizontal position aiding; GPS remains fused; not pure ground-truth localization"},
        fusion_caveat="EVP_NOISE=0.01m is the requested setting, not a demonstrated 1cm fusion variance; GPS_P_NOISE=0.5m floor and EV bias estimation remain relevant",
        runs={item["label"]: dict(folder=item["folder"], full_metrics=item["report"]["metrics"],
                                  source_audit=item["audit"], aiding_applied_and_restored_verified=item["metadata"].get("diagnostic_parameters_restored") if item["label"].startswith("B") else None)
              for item in loaded}, generator_sha256=sha(__file__),
        outputs_sha256={png.name: sha(png), pdf.name: sha(pdf)})
    (output/"comparison_audit.json").write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    parser.add_argument("--baseline-metadata", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--font", type=Path, default=Path("C:/Windows/Fonts/simhei.ttf"))
    parser.add_argument("--allow-incomplete", action="store_true", help="Explicit preliminary display with missing runs unfilled")
    args = parser.parse_args()
    report = plot(args.root, args.baseline_metadata, args.output, args.font, args.allow_incomplete)
    print(json.dumps({k: v for k, v in report.items() if k != "runs"}, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
