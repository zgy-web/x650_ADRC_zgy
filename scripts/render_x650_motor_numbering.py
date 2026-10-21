#!/usr/bin/env python3
"""Draw the explicit simulation motor mapping over the actual X650 URDF mesh."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml

from render_urdf_geometry import ROOT, geometry, sha


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path, help="Original user ZIP or URDF")
    parser.add_argument("--profile", type=Path, default=ROOT / "configs/vehicles/x650_no_arm/flight_profile.yaml")
    parser.add_argument("--output", type=Path, default=ROOT / "generated/x650_local_preview/motor_numbering.png")
    args = parser.parse_args()
    triangles, colors, record = geometry(args.source.resolve(), (0., 0., -.17095298), (math.pi/2, 0., 0.), {})
    profile = yaml.safe_load(args.profile.read_text(encoding="utf-8"))
    rotor_map = {int(rotor["motor_number"]): rotor for rotor in profile["rotors"]}
    expected = {0: ("link_004", "ccw"), 1: ("link_002", "ccw"), 2: ("link_003", "cw"), 3: ("link_005", "cw")}
    if set(rotor_map) != set(expected) or any((rotor_map[i]["link"], rotor_map[i]["turning_direction"]) != item for i, item in expected.items()):
        raise ValueError("Flight profile differs from the reviewed simulation numbering")
    joint_by_link = {joint["child"]: joint for joint in record["joints"]}
    centers = {number: np.array(joint_by_link[rotor["link"]]["position_m"]) for number, rotor in rotor_map.items()}
    diagonal = float(np.linalg.norm(centers[0]-centers[1]))
    other_diagonal = float(np.linalg.norm(centers[2]-centers[3]))
    if not math.isclose(diagonal, other_diagonal, abs_tol=1e-6) or not .649 <= diagonal <= .651:
        raise ValueError("Unexpected X650 rotor-center geometry")

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib.patches import Circle, FancyArrowPatch
    from matplotlib import font_manager
    font = Path("C:/Windows/Fonts/msyh.ttc")
    if font.is_file():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams["axes.unicode_minus"] = False
    fig, ax = plt.subplots(figsize=(12, 11), facecolor="#f5f7fa")
    fig.subplots_adjust(left=.035, right=.965, top=.84, bottom=.13)
    ax.set_facecolor("white")
    ax.set_xlim(-.64, .64); ax.set_ylim(-.51, .51); ax.set_aspect("equal")
    ax.set_xticks([]); ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_color("#d5dde6")
    # Looking down +Z: +X is up and +Y is left. Sort all real facets by Z.
    uv = np.stack((-triangles[..., 1], triangles[..., 0]), axis=-1)
    order = np.argsort(triangles[..., 2].mean(axis=1), kind="stable")
    gray = .25 + .55*np.mean(colors, axis=1)
    mesh_colors = np.repeat(gray[:, None], 3, axis=1)
    ax.add_collection(PolyCollection(uv[order], facecolors=mesh_colors[order], edgecolors="none", linewidths=0, antialiaseds=False, zorder=2))
    mapping_records = []
    names = {0: "前右", 1: "后左", 2: "前左", 3: "后右"}
    for number in sorted(rotor_map):
        rotor = rotor_map[number]
        center = centers[number]
        point = np.array([-center[1], center[0]])
        direction = rotor["turning_direction"]
        color = "#14856b" if direction == "ccw" else "#296fbb"
        radius = 15*.0254/2
        ax.add_patch(Circle(point, radius, fill=False, edgecolor=color, linewidth=1.4, linestyle=(0, (4, 3)), alpha=.65, zorder=3))
        angles = np.linspace(math.radians(30 if direction == "ccw" else 150), math.radians(310 if direction == "ccw" else -130), 100)
        arc = point + .145*np.column_stack((np.cos(angles), np.sin(angles)))
        ax.plot(arc[:, 0], arc[:, 1], color=color, lw=3, zorder=4)
        ax.add_patch(FancyArrowPatch(arc[-5], arc[-1], arrowstyle="-|>", mutation_scale=19, lw=2.5, color=color, zorder=5))
        ax.plot(*point, "o", color=color, markersize=7, zorder=6)
        ax.text(point[0], point[1]-.057, f"M{number+1}", color=color, fontsize=16, weight="bold", ha="center", va="center", zorder=7, bbox={"boxstyle": "round,pad=.15", "fc": "white", "ec": "none", "alpha": .94})
        label_x = .60 if point[0] > 0 else -.60
        label_y = .43 if point[1] > 0 else -.43
        align = "right" if point[0] > 0 else "left"
        ax.annotate(f"Motor {number+1} · {names[number]}\n{rotor['link']}  |  {direction.upper()}\nPX4 output {number+1} / GZ index {number}", xy=point + [.07*np.sign(point[0]), .08*np.sign(point[1])], xytext=(label_x, label_y), fontsize=11, color=color, ha=align, va="center", linespacing=1.6, zorder=8, bbox={"boxstyle": "round,pad=.5", "fc": "white", "ec": color, "alpha": .97}, arrowprops={"arrowstyle": "-", "color": color, "lw": 1.2})
        mapping_records.append({"motor_label": f"Motor {number+1}", "px4_output": number+1, "gazebo_motor_number": number, "link": rotor["link"], "turning_direction_top_view": direction, "center_body_flu_m": center.tolist()})

    # Dimension line is between opposing motor centers, not tip-to-tip span.
    first = np.array([-centers[1][1], centers[1][0]])
    last = np.array([-centers[0][1], centers[0][0]])
    offset = np.array([-.035, .035])
    for point in (first, last):
        ax.plot([point[0], point[0]+offset[0]], [point[1], point[1]+offset[1]], color="#8d5d9d", lw=1.1, zorder=8)
    ax.annotate("", xy=last+offset, xytext=first+offset, arrowprops={"arrowstyle": "<->", "color": "#8d5d9d", "lw": 1.7}, zorder=8)
    ax.text(-.022, .058, f"名义 650 mm 对角距\nURDF 电机中心距 {diagonal*1000:.2f} mm", rotation=45, rotation_mode="anchor", fontsize=10, color="#764287", ha="center", va="center", zorder=9, bbox={"fc": "white", "ec": "none", "alpha": .94, "pad": 3})
    ax.annotate("+X  约定机头", xy=(0, .478), xytext=(0, .386), ha="center", fontsize=12, weight="bold", color="#bd4248", arrowprops={"arrowstyle": "-|>", "lw": 2, "color": "#bd4248"}, zorder=9)
    ax.annotate("+Y  左", xy=(-.155, -.447), xytext=(.015, -.447), ha="center", va="center", fontsize=11, color="#3b4859", arrowprops={"arrowstyle": "-|>", "lw": 1.6, "color": "#3b4859"}, zorder=9)
    fig.text(.045, .945, "X650 · 仿真电机编号与旋向", fontsize=23, weight="bold", color="#18334b")
    fig.text(.045, .901, "俯视 TOP VIEW  ·  从机体上方向下看（沿 −Z）  ·  +X 朝上 / +Y 朝左", fontsize=13, color="#486176")
    fig.text(.045, .865, "CCW = 逆时针   |   CW = 顺时针   |   灰色：真实 URDF 网格；虚线圆：所选 15 英寸桨扫掠盘", fontsize=11, color="#486176")
    fig.text(.045, .083, "本图规定当前仿真编号，尚未确认实物电调接线及实体机头方向。", fontsize=13, weight="bold", color="#9c4c3c")
    fig.text(.045, .048, "650 mm 指相对电机中心的对角距离，不是桨尖外包络；几何保持原 URDF 尺度，未为了标注缩放。", fontsize=10.5, color="#50677d")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, facecolor=fig.get_facecolor())
    plt.close(fig)
    manifest = {"kind": "simulation_motor_numbering_geometry_audit", "source": str(args.source.resolve()), "source_sha256": sha(args.source), "source_urdf_sha256": record["urdf_sha256"], "profile_sha256": sha(args.profile), "generator_sha256": sha(__file__), "image_sha256": sha(args.output), "motor_mapping": mapping_records, "diagonal_motor_center_distance_m": diagonal, "opposite_diagonal_motor_center_distance_m": other_diagonal, "propeller_disc_diameter_m": .381, "top_view": "Looking down along -Z; up=+X, left=+Y; CW/CCW as seen from above", "real_wiring_confirmed": False, "physical_front_confirmed": False, "flight_started": False}
    args.output.with_suffix(".json").write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"image": str(args.output), "image_sha256": sha(args.output), "diagonal_mm": diagonal*1000}, indent=2))


if __name__ == "__main__":
    main()
