#!/usr/bin/env python3
"""Chinese four-panel baseline hover diagnostic; never claim causal RMSE shares."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def load_inputs(directory, metadata_path):
    directory, metadata_path = Path(directory).resolve(strict=True), Path(metadata_path).resolve(strict=True)
    diagnostic_path, samples_path = directory/"diagnostic.json", directory/"plot_samples.json"
    report = json.loads(diagnostic_path.read_text(encoding="utf-8"))
    display = json.loads(samples_path.read_text(encoding="utf-8"))
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if (report.get("kind") != "x650_native_hover_control_chain_diagnostic"
            or report.get("world_chain_available") is not True or report.get("uncertainties")
            or display.get("source_sha256") != report.get("aligned_chain_sha256")
            or sha(metadata_path) != report["source_files"]["metadata"]["sha256"]):
        raise ValueError("Unverified diagnostic/world chain, display source or original target metadata")
    alignment = report["time_alignment"]
    if any(alignment.get(k) is not False for k in ("phase_fitted", "origin_fitted", "gps_delay_subtracted_from_output")):
        raise ValueError("This baseline plot requires unfitted original time and world origin")
    if report["vector_chain_closure_max_m"] > 1e-10:
        raise ValueError("Full-source diagnostic vector chain did not close")
    target = np.asarray(metadata["hover_metrics"]["fixed_target_enu_m"], float)
    rows = display["rows"]
    t = np.asarray([row["sim_s"] for row in rows], float)
    def positions(prefix):
        value = np.asarray([[row[prefix+a] for a in "xyz"] for row in rows], float)
        if value.shape != (len(t), 3) or not np.isfinite(value).all():
            raise ValueError("Nonfinite display trajectory: "+prefix)
        return value
    truth, estimate, setpoint = (positions(prefix) for prefix in
        ("truth_world_enu_", "estimate_world_enu_", "executed_setpoint_world_enu_"))
    window = np.asarray(report["analysis_window_sim_s"], float)
    if (target.shape != (3,) or not np.isfinite(target).all() or not np.isfinite(t).all()
            or np.any(np.diff(t) <= 0) or not np.allclose(t[[0, -1]], window, rtol=0, atol=1e-8)):
        raise ValueError("Display data do not cover the declared original hover window")
    total, first, second, third = truth-target, truth-estimate, estimate-setpoint, setpoint-target
    closure = float(np.linalg.norm(first+second+third-total, axis=1).max())
    if closure > 1e-10:
        raise ValueError("Display vector chain did not close")
    audit = dict(kind="x650_baseline_hover_diagnostic_plot", experiment_improved=False,
        source_window_sim_s=window.tolist(), fixed_world_target_enu_m=target.tolist(),
        inputs_sha256={"diagnostic.json": sha(diagnostic_path), "plot_samples.json": sha(samples_path),
                       "controller_metadata.json": sha(metadata_path)},
        full_aligned_chain_sha256=report["aligned_chain_sha256"],
        full_native_trace_sha256=report["source_files"]["native"]["sha256"],
        full_ulog_sha256=report["source_files"]["ulog"]["sha256"],
        display_samples=len(t), display_max_interval_s=float(np.diff(t).max()),
        full_metric_samples=report["metrics"]["truth_minus_task_target_enu"]["sample_count"],
        metric_source="Provided full 250Hz diagnostic.json; 10Hz plot samples are display decimation only, not recomputed full metrics",
        fixed_target_source="Hash-matched original hover_metrics.fixed_target_enu_m, not post-takeoff measured anchor",
        display_vector_closure_max_m=closure, generator_sha256=sha(__file__),
        interpretation="Signed vector identity only; component norms/RMSE do not add and are not causal contribution shares",
        no_phase_delay_origin_fit=True, new_dynamics_simulated=False)
    return report, t-t[0], truth-target, estimate-target, setpoint-target, (total, first, second, third), audit


def plot(directory, metadata_path, output, font_path):
    import matplotlib
    matplotlib.use("Agg")
    from matplotlib import font_manager
    import matplotlib.pyplot as plt
    from matplotlib.lines import Line2D
    font_path = Path(font_path).resolve(strict=True)
    font_manager.fontManager.addfont(str(font_path))
    font_name = font_manager.FontProperties(fname=str(font_path)).get_name()
    plt.rcParams.update({"font.family": "sans-serif", "font.sans-serif": [font_name],
                         "axes.unicode_minus": False, "font.size": 11, "pdf.fonttype": 42})
    report, t, truth, estimate, setpoint, vectors, audit = load_inputs(directory, metadata_path)
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=True)
    if (output/"hover_control_chain_zh.png").exists() or (output/"plot_audit.json").exists():
        raise ValueError("Plot output already exists; preserve evidence in a new directory")
    metrics = report["metrics"]
    full = metrics["truth_minus_task_target_enu"]
    keys = ("truth_minus_estimate_world_enu", "estimate_minus_executed_setpoint_world_enu", "executed_setpoint_minus_task_world_enu")
    rms = [metrics[key]["rmse_3d_m"]*100 for key in keys]
    black, blue, purple, orange = "#172333", "#1475b6", "#8650a3", "#d98120"
    fig, axes = plt.subplots(2, 2, figsize=(15, 10))
    fig.subplots_adjust(left=.07, right=.98, top=.83, bottom=.22, wspace=.23, hspace=.34)
    fig.suptitle("悬停误差来自哪一段？——X650 原始基线诊断", x=.52, y=.982, fontsize=19)
    fig.text(.52, .941, f"纯悬停 {t[-1]:.0f} 秒  |  相对预设世界目标：3D RMSE {full['rmse_3d_m']*100:.2f} cm，峰值 {full['peak_3d_m']*100:.2f} cm  |  诊断结果，尚未证明修复", ha="center", fontsize=12)
    for x, label, value, color in zip((.21, .53, .84), ("① 定位差：真实 - 估计", "② 闭环差：估计 - 飞控目标", "③ 目标差：飞控目标 - 预设目标"), rms, (blue, purple, orange)):
        fig.text(x, .887, f"{label}\n3D RMSE {value:.2f} cm", color=color, ha="center", va="center", fontsize=12)

    xy = axes[0, 0]
    xy.plot(truth[:, 0]*100, truth[:, 1]*100, color=blue, lw=2, label="真实轨迹（Gazebo 原生）")
    xy.plot(estimate[:, 0]*100, estimate[:, 1]*100, color=purple, lw=1.4, ls="--", label="估计轨迹（飞控认为的位置）")
    xy.plot(setpoint[:, 0]*100, setpoint[:, 1]*100, color=orange, lw=2.2)
    xy.scatter(*setpoint[0, :2]*100, color=orange, marker="*", s=190, edgecolors="white", linewidths=.6,
               label="实际飞控目标（几乎固定）", zorder=5)
    xy.scatter(0, 0, color=black, marker="+", s=160, linewidths=2.4, label="预设世界目标", zorder=5)
    xy.scatter(*truth[0, :2]*100, color=blue, s=35, zorder=4)
    xy.scatter(*truth[-1, :2]*100, color=blue, s=40, marker="s", zorder=4)
    xy.set_title("A  水平轨迹：四个位置已统一到同一世界坐标", loc="left", pad=11)
    xy.set_xlabel("东向 X（相对预设世界目标，cm）")
    xy.set_ylabel("北向 Y（相对预设世界目标，cm）")
    xy.set_aspect("equal", adjustable="datalim")
    xy.margins(.15)
    xy.legend(fontsize=9, loc="upper right", framealpha=.9)
    xy.axhline(0, color="#bec5cc", lw=.7)
    xy.axvline(0, color="#bec5cc", lw=.7)
    labels = ("总误差：真实 - 预设目标", "① 真实 - 估计", "② 估计 - 飞控目标", "③ 飞控目标 - 预设目标")
    colors = (black, blue, purple, orange)
    styles = ("-", "-", "-", "--")
    for j, axis in enumerate((axes[0, 1], axes[1, 0], axes[1, 1])):
        for values, color, style in zip(vectors, colors, styles):
            axis.plot(t, values[:, j]*100, color=color, lw=1.9 if color == black else 1.2, ls=style)
        axis.axhline(0, color="#a3adb6", lw=.7)
        axis.set_xlim(0, t[-1])
        axis.set_title(f"{'BCD'[j]}  {'东向 X / 北向 Y / 高度 Z'.split(' / ')[j]}：有符号向量误差", loc="left", pad=11)
        axis.set_ylabel("位置差（cm，保留正负方向）")
        axis.set_xlabel("测量开始后的时间（s）")
    for axis in axes.ravel():
        axis.grid(alpha=.18)
        axis.spines[["top", "right"]].set_visible(False)
    handles = [Line2D([], [], color=c, lw=2 if i == 0 else 1.4, ls=style, label=label)
               for i, (c, style, label) in enumerate(zip(colors, styles, labels))]
    fig.legend(handles=handles, loc="lower center", bbox_to_anchor=(.54, .133), ncol=2, frameon=False, fontsize=11)
    fig.text(.52, .101, "逐时刻向量关系：真实-预设目标 = (真实-估计) + (估计-飞控目标) + (飞控目标-预设目标)", ha="center", fontsize=11)
    fig.text(.52, .069, "三项都有方向和相关性：RMSE 不能相加，也不能按大小分摊原因；本图不能证明调 PID 已解决问题。", ha="center", color="#6b3929", fontsize=11)
    fig.text(.52, .038, "指标来自完整 250 Hz / 5,001 点；曲线仅按 10 Hz / 201 点抽样显示。统一世界 ENU；保留原始时间，不拟合延迟、相位或原点。", ha="center", color="#52606f", fontsize=9)
    png, pdf = output/"hover_control_chain_zh.png", output/"hover_control_chain_zh.pdf"
    fig.savefig(png, dpi=180)
    fig.savefig(pdf)
    plt.close(fig)
    audit.update(font_path=str(font_path), font_name=font_name,
                 full_metrics_cm={"total_rmse": full["rmse_3d_m"]*100, "total_peak": full["peak_3d_m"]*100,
                                  "truth_estimate_rmse": rms[0], "estimate_setpoint_rmse": rms[1], "setpoint_task_rmse": rms[2]},
                 outputs_sha256={png.name: sha(png), pdf.name: sha(pdf)})
    (output/"plot_audit.json").write_text(json.dumps(audit, indent=2, ensure_ascii=False)+"\n", encoding="utf-8")
    return audit


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--metadata", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--font", type=Path, default=Path("C:/Windows/Fonts/simhei.ttf"))
    args = parser.parse_args()
    print(json.dumps(plot(args.input_dir, args.metadata, args.output, args.font), indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
