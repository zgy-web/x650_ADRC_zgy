#!/usr/bin/env python3
"""Render exact STL visual geometry from an inspected URDF, without Gazebo.

Every visual is drawn at q=0 with root/joint/visual transforms and mesh scale.
This produces a geometry audit, not a flight simulation or a Gazebo screenshot.
Requires the existing numpy/matplotlib installation; no resources are fetched.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import struct
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from amsim.modeling.urdf_importer import _inspect, _origin, _rotation, _source


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def transform(xyz, rpy):
    result = np.eye(4)
    result[:3, :3] = np.asarray(_rotation(rpy))
    result[:3, 3] = xyz
    return result


def origin(node):
    value = _origin(node)
    return transform(value["xyz"], value["rpy"])


def load_stl(path):
    content = path.read_bytes()
    if len(content) >= 84:
        count = struct.unpack_from("<I", content, 80)[0]
        if len(content) == 84 + 50 * count:
            record = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
            vertices = np.frombuffer(content, dtype=record, count=count, offset=84)["vertices"].astype(np.float64)
        else:
            vertices = None
    else:
        vertices = None
    if vertices is None:
        text = content.decode("ascii")
        numbers = re.findall(r"\bvertex\s+([^\s]+)\s+([^\s]+)\s+([^\s]+)", text, flags=re.IGNORECASE)
        if not numbers or len(numbers) % 3:
            raise ValueError(f"Invalid STL triangle records: {path}")
        vertices = np.asarray(numbers, dtype=float).reshape(-1, 3, 3)
    if not len(vertices) or not np.isfinite(vertices).all():
        raise ValueError(f"Empty/non-finite STL geometry: {path}")
    return vertices


def geometry(source, root_xyz, root_rpy, packages, urdf_entry=None):
    with _source(source, urdf_entry) as (selected, boundary, entry):
        robot, report, resources = _inspect(selected, boundary, packages)
        by_uri = {mesh["source_uri"]: mesh for mesh in report["meshes"]}
        poses = {report["root_link"]: transform(root_xyz, root_rpy)}
        pending = list(robot.findall("joint"))
        while pending:
            progress = False
            for joint in pending[:]:
                parent, child = joint.find("parent").get("link"), joint.find("child").get("link")
                if parent in poses:
                    poses[child] = poses[parent] @ origin(joint)
                    pending.remove(joint)
                    progress = True
            if not progress:
                raise ValueError("Cannot resolve URDF zero-state kinematic tree")
        meshes, colors, visual_records = [], [], []
        cache = {}
        palette = [(0.12, .20, .29), (.10, .48, .73), (.10, .48, .73), (.10, .48, .73), (.10, .48, .73)]
        named_materials = {node.get("name"): node for node in robot.findall("material")}
        weighted_com = np.zeros(3)
        mass_by_link = {link["name"]: link["mass_kg"] for link in report["links"]}
        for link_index, link in enumerate(robot.findall("link")):
            link_name = link.get("name")
            link_pose = poses[link_name]
            weighted_com += mass_by_link[link_name] * (link_pose @ origin(link.find("inertial")))[:3, 3]
            for visual_index, visual in enumerate(link.findall("visual")):
                mesh = visual.find("geometry/mesh")
                if mesh is None:
                    raise ValueError("This geometry renderer currently supports STL visual meshes only")
                uri = mesh.get("filename")
                metadata = by_uri[uri]
                path = resources[metadata["asset_name"]]
                if path.suffix.lower() != ".stl":
                    raise ValueError(f"Renderer supports STL only; importer supports additional formats: {uri}")
                if metadata["sha256"] not in cache:
                    cache[metadata["sha256"]] = load_stl(path)
                scale = np.asarray([float(v) for v in mesh.get("scale", "1 1 1").split()])
                matrix = link_pose @ origin(visual)
                triangles = (cache[metadata["sha256"]] * scale) @ matrix[:3, :3].T + matrix[:3, 3]
                color = palette[min(link_index, len(palette) - 1)]
                material = visual.find("material")
                if material is not None:
                    if material.find("color") is None and material.get("name") in named_materials:
                        material = named_materials[material.get("name")]
                    if material.find("color") is not None:
                        rgba = [float(v) for v in material.find("color").get("rgba").split()]
                        color = rgba[:3]
                meshes.append(triangles)
                colors.append(np.repeat(np.asarray(color)[None, :], len(triangles), axis=0))
                visual_records.append({"link": link_name, "visual_index": visual_index, "source_uri": uri,
                    "sha256": metadata["sha256"], "triangles": len(triangles), "scale": scale.tolist(),
                    "visual_to_render_frame": matrix.tolist()})
        triangles = np.concatenate(meshes)
        colors = np.concatenate(colors)
        lower, upper = triangles.min(axis=(0, 1)), triangles.max(axis=(0, 1))
        joint_records = []
        for joint in robot.findall("joint"):
            child = joint.find("child").get("link")
            axis = np.asarray([float(v) for v in joint.find("axis").get("xyz").split()]) if joint.find("axis") is not None else None
            joint_records.append({"name": joint.get("name"), "type": joint.get("type"),
                                  "child": child, "position_m": poses[child][:3, 3].tolist(),
                                  "axis": (poses[child][:3, :3] @ axis).tolist() if axis is not None else None})
        record = {"kind": "urdf_stl_geometry_render", "source": str(source), "source_sha256": sha(source),
                  "urdf_entry": entry, "urdf_sha256": sha(selected), "root_link": report["root_link"],
                  "root_xyz_m": list(root_xyz), "root_rpy_rad": list(root_rpy), "joint_state": "q=0 for every joint",
                  "visual_count": len(visual_records), "unique_visual_meshes": len(cache),
                  "triangle_count": len(triangles), "link_count": len(report["links"]), "joint_count": len(report["joints"]),
                  "mass_from_urdf_kg": report["total_mass_kg"], "center_of_mass_m": (weighted_com / report["total_mass_kg"]).tolist(),
                  "bounds_min_m": lower.tolist(), "bounds_max_m": upper.tolist(), "dimensions_xyz_m": (upper - lower).tolist(),
                  "joints": joint_records, "visuals": visual_records,
                  "rendered_geometry": "All visual STL facets, no decimation; collision geometry is not double-drawn",
                  "colors": "URDF colors where declared; otherwise illustrative dark body and blue rotating links",
                  "gazebo_started": False, "flight_validated": False, "physical_front_confirmed": False}
        return triangles, colors, record


def render(triangles, colors, record, output, title):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PolyCollection
    from matplotlib import font_manager

    font = Path("C:/Windows/Fonts/msyh.ttc")
    if font.is_file():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams["font.family"] = font_manager.FontProperties(fname=str(font)).get_name()
    else:
        plt.rcParams["font.family"] = ["Noto Sans CJK SC", "DejaVu Sans"]
    plt.rcParams["axes.unicode_minus"] = False
    plt.rcParams["font.size"] = 11
    lower, upper = np.array(record["bounds_min_m"]), np.array(record["bounds_max_m"])
    center = (lower + upper) / 2
    span = float(np.max(upper - lower))
    normal = np.cross(triangles[:, 1] - triangles[:, 0], triangles[:, 2] - triangles[:, 0])
    normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-15)
    light = np.array([-.4, -.5, 1.]); light /= np.linalg.norm(light)
    intensity = .52 + .48 * np.abs(normal @ light)
    lit = np.clip(colors * intensity[:, None] + .08, 0, 1)
    fig = plt.figure(figsize=(18, 11), facecolor="#f4f6f8")
    grid = fig.add_gridspec(2, 3, width_ratios=(1.4, 1, 1), height_ratios=(1.05, 1), left=.025, right=.98, top=.84, bottom=.12, wspace=.12, hspace=.18)
    main = fig.add_subplot(grid[:, 0:2])
    top = fig.add_subplot(grid[0, 2])
    side = fig.add_subplot(grid[1, 2])

    def panel(ax, right, up, outward, label, perspective=False):
        right, up, outward = np.array(right, dtype=float), np.array(up, dtype=float), np.array(outward, dtype=float)
        outward /= np.linalg.norm(outward)
        right /= np.linalg.norm(right)
        up /= np.linalg.norm(up)
        basis = np.stack([right, up, outward])
        distance = span * 2.6
        def project(vertices):
            camera = (np.asarray(vertices) - center) @ basis.T
            uv = camera[..., :2]
            if perspective:
                uv = uv * (distance / (distance - camera[..., 2]))[..., None]
            return uv, camera[..., 2]
        uv, depth = project(triangles)
        order = np.argsort(depth.mean(axis=1), kind="stable")
        collection = PolyCollection(uv[order], facecolors=lit[order], edgecolors="none", linewidths=0, antialiaseds=False)
        ax.add_collection(collection)
        uv_min, uv_max = uv.min(axis=(0, 1)), uv.max(axis=(0, 1))
        extent = uv_max - uv_min
        pad = .13 * span
        ax.set_xlim(uv_min[0] - pad, uv_max[0] + pad)
        ax.set_ylim(uv_min[1] - pad, uv_max[1] + pad)
        ax.set_aspect("equal")
        ax.set_facecolor("white")
        for spine in ax.spines.values():
            spine.set_color("#d4dce4")
        ax.set_xticks([]); ax.set_yticks([])
        ax.set_title(label, loc="left", pad=11, fontweight="bold", fontsize=12, color="#1b3349")
        origin_uv, _ = project([0, 0, 0])
        length = .28 * span if perspective else .11 * span
        for vector, text, color in (([length, 0, 0], "+X 约定机头", "#d84a42"), ([0, length, 0], "+Y 左", "#23915d"), ([0, 0, length], "+Z 上", "#337bcc")):
            point, _ = project(vector)
            if np.linalg.norm(point - origin_uv) < .03 * span:
                continue
            ax.annotate("", xy=point, xytext=origin_uv, arrowprops=dict(arrowstyle="-|>", color=color, lw=2.0, mutation_scale=13), zorder=5)
            ax.annotate(text, xy=point, xytext=(5, 5), textcoords="offset points", color=color, fontsize=10, fontweight="bold", zorder=6,
                        bbox=dict(facecolor="white", alpha=.82, edgecolor="none", pad=1.4))
        if perspective:
            for index, joint in enumerate(record["joints"]):
                point, _ = project(joint["position_m"])
                ax.plot(*point, marker="o", ms=3.5, color="#f0b33d", zorder=6)
                ax.annotate(joint["child"], xy=point, xytext=(8, -13), textcoords="offset points", fontsize=9, color="#35506a", zorder=7,
                            bbox=dict(facecolor="white", alpha=.88, edgecolor="none", pad=1.4))
        return project

    outward = np.array([1., -1.2, .85]); outward /= np.linalg.norm(outward)
    right = np.cross([0, 0, 1], outward); right /= np.linalg.norm(right)
    up = np.cross(outward, right)
    panel(main, right, up, outward, "01  透视几何预览 · 全部 STL 三角面", True)
    panel(top, [0, -1, 0], [1, 0, 0], [0, 0, 1], "02  俯视 · X–Y 平面")
    panel(side, [1, 0, 0], [0, 0, 1], [0, -1, 0], "03  右侧视 · X–Z 平面")
    dimensions = record["dimensions_xyz_m"]
    fig.text(.03, .94, title, fontsize=24, fontweight="bold", color="#17324b")
    fig.text(.03, .892, f"URDF 静态几何渲染  |  {record['visual_count']} 个 visual / {record['unique_visual_meshes']} 个 STL / {record['triangle_count']:,} 个三角面  |  {record['link_count']} 个链接，{record['joint_count']} 个关节", fontsize=12, color="#4b6378")
    fig.text(.03, .075, f"外包络尺寸（X × Y × Z）：{dimensions[0]*1000:.1f} × {dimensions[1]*1000:.1f} × {dimensions[2]*1000:.1f} mm     URDF 声明总质量：{record['mass_from_urdf_kg']:.2f} kg", fontsize=13, fontweight="bold", color="#17324b")
    fig.text(.03, .038, "+X 为本次机体系的约定机头方向，仍需与实体机确认。颜色用于区分部件；本图不是 Gazebo 截图，也不是飞行或控制效果。", fontsize=11, color="#526b80")
    png = output / "geometry_multiview.png"
    fig.savefig(png, dpi=180, facecolor=fig.get_facecolor())
    plt.close(fig)
    return png


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", type=Path)
    parser.add_argument("--urdf-entry")
    parser.add_argument("--root-xyz", nargs=3, type=float, default=(0., 0., 0.))
    parser.add_argument("--root-rpy", nargs=3, type=float, default=(0., 0., 0.))
    parser.add_argument("--package", action="append", default=[], metavar="NAME=PATH")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--title", default="无人机 URDF · 坐标与外形检查")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if not all(math.isfinite(v) for v in (*args.root_xyz, *args.root_rpy)):
        parser.error("root transform must be finite")
    source = args.source.expanduser().resolve(strict=True)
    output = args.output.expanduser().resolve()
    if source.is_relative_to(output):
        parser.error("output cannot contain the original input")
    if not args.overwrite and any((output / name).exists() for name in ("geometry_multiview.png", "geometry_manifest.json")):
        parser.error("output files already exist; use a new directory or --overwrite")
    packages = {}
    for item in args.package:
        name, separator, value = item.partition("=")
        if not separator or not name or name in packages:
            parser.error("--package requires unique NAME=PATH values")
        packages[name] = Path(value).expanduser().resolve(strict=True)
    triangles, colors, record = geometry(source, args.root_xyz, args.root_rpy, packages, args.urdf_entry)
    output.mkdir(parents=True, exist_ok=True)
    png = render(triangles, colors, record, output, args.title)
    record.update({"image": str(png), "image_sha256": sha(png), "generator_sha256": sha(__file__)})
    manifest = output / "geometry_manifest.json"
    manifest.write_text(json.dumps(record, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"image": str(png), "manifest": str(manifest), "visual_count": record["visual_count"],
                      "triangles": record["triangle_count"], "dimensions_xyz_m": record["dimensions_xyz_m"],
                      "gazebo_started": False, "flight_validated": False}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
