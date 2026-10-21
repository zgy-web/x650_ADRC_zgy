#!/usr/bin/env python3
"""Replay real Gazebo X650 base poses with the complete imported visual geometry.

No dynamics are simulated here. MuJoCo is used only as an offscreen renderer.
Input poses are in the Gazebo world frame; quaternions rotate base_link to world.
The output retains real simulation time (1x) and explicitly identifies itself as
a recorded-state replay. Unlogged rotor angles are never invented or animated.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import sys
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from render_urdf_geometry import geometry, sha

BG, PANEL, FG, MUTED = "#101b2c", "#15253a", "#edf4fb", "#aebed0"
BLUE, GOLD, GREEN, RED = "#5dbaff", "#ffc15a", "#69d8b0", "#ff8a7a"
HOVER_PHASES = {"settle", "measure", "post"}
REQUIRED = ["sim_time_s", "phase", "world_x_m", "world_y_m", "world_z_m",
            "world_qx", "world_qy", "world_qz", "world_qw",
            "target_world_x_m", "target_world_y_m", "target_world_z_m"]


def load_samples(path):
    with path.open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        if not reader.fieldnames or set(REQUIRED) - set(reader.fieldnames):
            raise ValueError(f"Missing replay CSV fields: {sorted(set(REQUIRED) - set(reader.fieldnames or []))}")
        rows = list(reader)
    if len(rows) < 2:
        raise ValueError("At least two actual recorded poses are required")
    def values(names):
        return np.array([[float(row[name]) for name in names] for row in rows])
    t = values(["sim_time_s"]).ravel()
    p = values(["world_x_m", "world_y_m", "world_z_m"])
    q = values(["world_qw", "world_qx", "world_qy", "world_qz"])
    target = values(["target_world_x_m", "target_world_y_m", "target_world_z_m"])
    if not np.isfinite(np.column_stack([t, p, q])).all():
        raise ValueError("Recorded position, orientation and timestamps must be finite")
    if np.any(np.diff(t) <= 0):
        raise ValueError("Recorded simulation timestamps must be strictly increasing")
    qnorm = np.linalg.norm(q, axis=1)
    if np.any(np.abs(qnorm - 1) > .01):
        raise ValueError("Recorded orientation is not a unit quaternion")
    q /= qnorm[:, None]
    phase = np.array([row["phase"] for row in rows])
    hover = np.array([v in HOVER_PHASES for v in phase])
    if not np.isfinite(target[hover]).all():
        raise ValueError("Hover reference is missing in a hover phase")
    measured = phase == "measure"
    q0, q1, q2, q3 = q.T
    roll = np.arctan2(2*(q0*q1+q2*q3), 1-2*(q1*q1+q2*q2))
    pitch = np.arcsin(np.clip(2*(q0*q2-q3*q1), -1, 1))
    yaw = np.arctan2(2*(q0*q3+q1*q2), 1-2*(q2*q2+q3*q3))
    return {"rows": rows, "t": t, "elapsed": t-t[0], "p": p, "q": q,
            "target": target, "phase": phase, "hover": hover, "measure": measured,
            "attitude_deg": np.rad2deg(np.column_stack([roll, pitch, yaw])),
            "error": p-target, "max_sample_gap_s": float(np.diff(t).max())}


def verify_recording(path, data):
    """Require the capture's finalized checksum before publishing a replay."""
    manifest_path = path.with_name("flight_samples_manifest.json")
    if not manifest_path.is_file():
        raise ValueError("Completed flight_samples_manifest.json is required; do not render a live or unfinished capture")
    metadata = json.loads(manifest_path.read_text(encoding="utf-8"))
    if metadata.get("sha256") != sha(path):
        raise ValueError("Flight capture checksum does not match the finalized manifest")
    if metadata.get("file") != path.name:
        raise ValueError("Flight capture manifest identifies a different CSV")
    models = {row.get("model_sha256", "") for row in data["rows"]}
    if not metadata.get("model_sha256") or models != {metadata["model_sha256"]}:
        raise ValueError("Flight capture model hash differs from its manifest")
    return {"path": str(manifest_path), "sha256": sha(manifest_path), "metadata": metadata}


def sample_pose(data, elapsed, gap_limit=.25):
    """Linear translation plus shortest-path quaternion SLERP; never smooth."""
    t = data["elapsed"]
    i = min(max(0, int(np.searchsorted(t, elapsed, side="right")-1)), len(t)-1)
    if i == len(t)-1:
        return data["p"][i], data["q"][i], i, False
    dt = t[i+1]-t[i]
    if dt > gap_limit:
        return data["p"][i], data["q"][i], i, True
    fraction = float(np.clip((elapsed-t[i])/dt, 0, 1))
    pos = (1-fraction)*data["p"][i]+fraction*data["p"][i+1]
    qa, qb = data["q"][i], data["q"][i+1].copy()
    dot = float(np.dot(qa, qb))
    if dot < 0:
        qb, dot = -qb, -dot
    if dot > .9995:
        q = (1-fraction)*qa+fraction*qb
        q /= np.linalg.norm(q)
    else:
        angle = math.acos(np.clip(dot, -1, 1))
        q = (math.sin((1-fraction)*angle)*qa+math.sin(fraction*angle)*qb)/math.sin(angle)
    return pos, q, i, False


def write_binary_stl(path, triangles):
    dtype = np.dtype([("normal", "<f4", (3,)), ("vertices", "<f4", (3, 3)), ("attribute", "<u2")])
    records = np.zeros(len(triangles), dtype=dtype)
    records["vertices"] = triangles
    normal = np.cross(triangles[:, 1]-triangles[:, 0], triangles[:, 2]-triangles[:, 0])
    normal /= np.maximum(np.linalg.norm(normal, axis=1)[:, None], 1e-15)
    records["normal"] = normal
    with path.open("wb") as stream:
        stream.write(b"X650 exact transformed URDF visual; replay geometry".ljust(80, b" "))
        stream.write(struct.pack("<I", len(records)))
        stream.write(records.tobytes())


def make_scene(directory, triangles, colors, audit, data, width, height):
    """Generate render-only MJCF; one mesh per original visual, no decimation."""
    directory.mkdir(parents=True, exist_ok=True)
    root = ET.Element("mujoco", model="x650_gazebo_recorded_pose_replay")
    ET.SubElement(root, "compiler", angle="radian", inertiafromgeom="false")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(width), offheight=str(height))
    ET.SubElement(visual, "quality", shadowsize="2048", offsamples="4")
    ET.SubElement(visual, "headlight", ambient=".55 .55 .60", diffuse=".6 .6 .6", specular=".15 .15 .15")
    ET.SubElement(visual, "rgba", haze=".06 .10 .16 1")
    asset = ET.SubElement(root, "asset")
    ET.SubElement(asset, "texture", name="floor_grid", type="2d", builtin="checker",
                  rgb1=".065 .10 .15", rgb2=".10 .15 .21", width="512", height="512")
    ET.SubElement(asset, "material", name="floor", texture="floor_grid", texrepeat="20 20", reflectance="0")
    world = ET.SubElement(root, "worldbody")
    p0 = data["p"][0]
    ET.SubElement(world, "light", pos=f"{p0[0]-2} {p0[1]-3} 8", dir="0 0 -1", diffuse=".7 .7 .7")
    ET.SubElement(world, "geom", name="ground", type="plane", pos="0 0 -.003", size="20 20 .1", material="floor", contype="0", conaffinity="0")
    body = ET.SubElement(world, "body", name="x650_base", mocap="true")
    offset = 0
    for index, record in enumerate(audit["visuals"]):
        count = record["triangles"]
        name = f"urdf_visual_{index:03d}"
        path = directory / f"{name}.stl"
        write_binary_stl(path, triangles[offset:offset+count])
        # maxhullvert affects only the unused collision hull; render triangles are preserved.
        ET.SubElement(asset, "mesh", name=name, file=str(path), maxhullvert="64")
        # Lift dark CAD/default display colors so the real body remains legible
        # against the dark floor. This affects appearance, never geometry/state.
        rgba = [*np.clip(.20+.80*colors[offset], 0, 1), 1]
        ET.SubElement(body, "geom", name=name, type="mesh", mesh=name,
                      rgba=" ".join(f"{v:.8g}" for v in rgba), contype="0", conaffinity="0", group="1")
        offset += count
    if offset != len(triangles):
        raise ValueError("Visual partition does not account for all mesh triangles")
    # A small world-fixed target marker is an explanatory overlay, not a simulated object.
    valid = np.flatnonzero(data["hover"])
    if len(valid):
        target = data["target"][valid[0]]
        for axis in (0, 1):
            start, end = target.copy(), target.copy()
            start[axis] -= .12; end[axis] += .12
            ET.SubElement(world, "geom", name=f"hover_goal_{axis}", type="capsule",
                          fromto=" ".join(str(v) for v in [*start, *end]), size=".006",
                          rgba="1 .75 .35 .65", contype="0", conaffinity="0")
    path = directory / "replay_scene.xml"
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return path


def find_font(size=18, bold=False):
    from PIL import ImageFont
    names = ["/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf" if bold else "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
             "C:/Windows/Fonts/arialbd.ttf" if bold else "C:/Windows/Fonts/arial.ttf"]
    for path in names:
        if Path(path).is_file():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def scientific_figures(data, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t, pos, target, error = data["elapsed"], data["p"], data["target"], data["error"]
    mask, measured = data["hover"], data["measure"]
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True, layout="constrained")
    for axis, color in zip(range(3), ["#1478bb", "#d87c1b", "#239669"]):
        axes[0].plot(t, pos[:, axis], color=color, label=f"actual {'XYZ'[axis]}", lw=1.3)
        axes[0].plot(t, np.where(mask, target[:, axis], np.nan), color=color, ls="--", lw=1.3, label=f"hover target {'XYZ'[axis]}")
        axes[1].plot(t, np.where(mask, error[:, axis]*1000, np.nan), color=color, label=f"{'XYZ'[axis]} error")
        axes[2].plot(t, data["attitude_deg"][:, axis], color=color, label=["roll", "pitch", "yaw"][axis])
    axes[0].set_ylabel("Gazebo world position (m)")
    axes[1].set_ylabel("Hover position error (mm)")
    axes[2].set_ylabel("Attitude (deg)"); axes[2].set_xlabel("Elapsed simulation time (s)")
    for ax in axes:
        ax.grid(alpha=.2); ax.legend(ncol=3, loc="upper right", fontsize=9)
        if np.any(measured):
            ids = np.flatnonzero(measured)
            ax.axvspan(t[ids[0]], t[ids[-1]], color="#69d8b0", alpha=.12)
    fig.suptitle("X650 no-arm | PX4 cascaded position / attitude control\nGazebo recorded states; shaded window = hover measurement", fontsize=14)
    fig.savefig(output / "trajectory_and_errors.png", dpi=160)
    fig.savefig(output / "trajectory_and_errors.pdf")
    plt.close(fig)
    ids = np.flatnonzero(measured)
    metrics = {"measurement_samples": int(len(ids)), "reference_definition": "fixed world hover position; not a takeoff/landing trajectory command"}
    if len(ids):
        norm = np.linalg.norm(error[ids], axis=1)
        metrics.update({"measurement_duration_s": float(t[ids[-1]]-t[ids[0]]),
                        "position_rmse_m": float(np.sqrt(np.mean(norm**2))),
                        "position_peak_m": float(norm.max()),
                        "axis_rmse_m": np.sqrt(np.mean(error[ids]**2, axis=0)).tolist(),
                        "max_abs_roll_pitch_deg": np.max(np.abs(data["attitude_deg"][ids, :2]), axis=0).tolist()})
    (output / "replay_metrics.json").write_text(json.dumps(metrics, indent=2)+"\n", encoding="utf-8")
    return metrics


def draw_chart(draw, box, title, t, series, colors, labels, now, ymax=None, ymin=None):
    x0, y0, x1, y1 = box
    draw.rounded_rectangle(box, radius=12, fill=PANEL)
    draw.text((x0+18, y0+12), title, fill=FG, font=find_font(18, True))
    left, right, top, bottom = x0+60, x1-22, y0+55, y1-48
    finite = np.concatenate([s[np.isfinite(s)] for s in series])
    low = float(finite.min()) if ymin is None and len(finite) else (ymin or 0.)
    high = float(finite.max()) if ymax is None and len(finite) else (ymax or 1.)
    if high-low < 1e-8: high = low+1
    if ymin is None or ymax is None:
        pad = max((high-low)*.08, .001)
        if ymin is None: low -= pad
        if ymax is None: high += pad
    duration = max(t[-1], 1e-6)
    for f in (0., .5, 1.):
        y = bottom-(bottom-top)*f
        draw.line((left, y, right, y), fill="#31445b", width=1)
        draw.text((x0+7, y-8), f"{low+(high-low)*f:.2g}", fill=MUTED, font=find_font(13))
    for s, color, label_index in zip(series, colors, range(len(series))):
        points = []
        for ti, value in zip(t, s):
            if ti > now: break
            if not np.isfinite(value):
                if len(points) > 1: draw.line(points, fill=color, width=2)
                points = []; continue
            points.append((left+(right-left)*ti/duration, bottom-(bottom-top)*(value-low)/(high-low)))
        if len(points) > 1: draw.line(points, fill=color, width=2)
        draw.text((left+label_index*110, y1-29), labels[label_index], fill=color, font=find_font(13))
    xcur = left+(right-left)*now/duration
    draw.line((xcur, top, xcur, bottom), fill="#98a8b8", width=1)
    draw.text((left, bottom+5), "0", fill=MUTED, font=find_font(11))
    draw.text((right-35, bottom+5), f"{duration:.1f}s", fill=MUTED, font=find_font(11))


def render(args):
    from PIL import Image, ImageDraw
    import mujoco
    source = args.source.resolve(strict=True)
    csv_path = args.csv.resolve(strict=True)
    output = args.output.resolve()
    if output.exists() and (output / "x650_hover.mp4").exists() and not args.overwrite:
        raise ValueError("Output already exists; use a new directory or --overwrite")
    output.mkdir(parents=True, exist_ok=True)
    data = load_samples(csv_path)
    capture = verify_recording(csv_path, data)
    triangles, colors, audit = geometry(source, args.root_xyz, args.root_rpy, {})
    width, height = 1440, 900
    view_width, view_height = 900, 760
    scene = make_scene(output / "render_geometry", triangles, colors, audit, data, view_width, view_height)
    del triangles, colors
    print("Compiling render-only geometry", flush=True)
    model = mujoco.MjModel.from_xml_path(str(scene))
    state = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=view_height, width=view_width)
    cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
    cam.azimuth = 125; cam.elevation = -17
    p0 = data["p"][0]
    max_z = max(float(data["p"][:, 2].max()), float(np.nanmax(data["target"][:, 2])))
    cam.lookat[:] = [p0[0], p0[1], max_z/2]
    cam.distance = max(4.8, max_z*1.9)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    mocap = model.body_mocapid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "x650_base")]
    options = mujoco.MjvOption()
    metrics = scientific_figures(data, output)
    ffmpeg = args.ffmpeg
    if not ffmpeg:
        ffmpeg = shutil.which("ffmpeg")
        if not ffmpeg:
            installed = Path("/root/autodl-tmp/flyarm-video-venv/lib/python3.10/site-packages/imageio_ffmpeg/binaries")
            ffmpeg = next((str(p) for p in installed.glob("ffmpeg-*")), None)
    if not ffmpeg: raise ValueError("An existing ffmpeg executable is required")
    duration = float(data["elapsed"][-1])
    times = np.arange(0, duration+1e-9, 1/args.fps)
    if args.preview_only:
        measured = np.flatnonzero(data["measure"])
        times = np.array([data["elapsed"][measured[len(measured)//2]]] if len(measured) else [duration/2])
    destination = output / "x650_hover.mp4"
    log = (output / "ffmpeg.log").open("w")
    proc = None if args.preview_only else subprocess.Popen([
        ffmpeg, "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{width}x{height}",
        "-r", str(args.fps), "-i", "-", "-an", "-vcodec", "libx264", "-preset", "fast", "-crf", "20",
        "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(destination)],
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)
    snapshots = set(np.linspace(0, len(times)-1, 5).astype(int))
    altitude_target = np.where(data["hover"], data["target"][:, 2], np.nan)
    error_norm = np.where(data["hover"], np.linalg.norm(data["error"], axis=1)*1000, np.nan)
    try:
        for k, now in enumerate(times):
            pos, quat, idx, gap = sample_pose(data, now)
            state.mocap_pos[mocap] = pos; state.mocap_quat[mocap] = quat
            mujoco.mj_forward(model, state)
            renderer.update_scene(state, camera=cam, scene_option=options)
            rendered = renderer.render().copy()
            canvas = Image.new("RGB", (width, height), BG)
            canvas.paste(Image.fromarray(rendered), (0, 80))
            draw = ImageDraw.Draw(canvas)
            draw.rectangle((0, 0, width, 80), fill=BG)
            draw.text((24, 13), "X650  |  NO-ARM HOVER", font=find_font(29, True), fill=FG)
            draw.text((24, 51), "PX4 cascaded PID  /  Gazebo SITL  /  MN4014 KV330 + 15x5 CF  /  3.02 kg", font=find_font(16), fill=MUTED)
            draw.rounded_rectangle((24, 100, 320, 184), radius=10, fill=PANEL)
            draw.text((40, 111), f"{data['phase'][idx].upper()}     {now:5.2f} s", font=find_font(18, True), fill=GREEN)
            draw.text((40, 145), f"World Z: {pos[2]:.3f} m   |   1x", font=find_font(17), fill=FG)
            # Same physical pose, close camera: this is an inset, not a second run.
            if not args.no_inset:
                close = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(close)
                close.lookat[:] = pos+np.array([0, 0, -.10]); close.azimuth = 128; close.elevation = -25; close.distance = 1.45
                renderer.update_scene(state, camera=close, scene_option=options)
                inset = Image.fromarray(renderer.render().copy()).resize((342, 289))
                canvas.paste(inset, (24, 540))
                draw = ImageDraw.Draw(canvas)
                draw.rectangle((24, 540, 366, 570), fill=PANEL)
                draw.text((36, 546), "Same pose  |  body detail", font=find_font(14), fill=FG)
            draw_chart(draw, (914, 92, 1424, 322), "WORLD ALTITUDE  |  m", data["elapsed"],
                       [data["p"][:, 2], altitude_target], [BLUE, GOLD], ["actual", "hover goal"], now, ymin=0)
            draw_chart(draw, (914, 338, 1424, 568), "HOVER POSITION ERROR  |  mm", data["elapsed"],
                       [error_norm], [GREEN], ["3D norm"], now, ymin=0)
            draw_chart(draw, (914, 584, 1424, 814), "BODY ATTITUDE  |  deg", data["elapsed"],
                       [data["attitude_deg"][:, 0], data["attitude_deg"][:, 1]], [BLUE, RED], ["roll", "pitch"], now)
            draw.rectangle((0, 840, width, height), fill=BG)
            draw.text((24, 849), "GAZEBO RECORDED-STATE REPLAY  |  all 281 original visual meshes  |  no pose smoothing", font=find_font(16, True), fill=FG)
            draw.text((24, 875), "Renderer only; not a native Gazebo screen capture. Rotor angles are not logged or animated. Gold cross = hover goal.", font=find_font(13), fill=MUTED)
            if gap:
                draw.rounded_rectangle((390, 104, 868, 149), radius=8, fill="#79392c")
                draw.text((408, 116), "TELEMETRY GAP: holding last recorded pose", font=find_font(16), fill=FG)
            if k in snapshots:
                canvas.save(output / f"frame_{k:05d}.png")
            if args.preview_only:
                canvas.save(output / "preview.png")
            else:
                proc.stdin.write(canvas.tobytes())
            if k % 50 == 0:
                print(f"RENDER {k}/{len(times)} elapsed={now:.2f}s", flush=True)
    finally:
        renderer.close()
        if proc:
            proc.stdin.close(); status = proc.wait()
            log.close()
            if status: raise RuntimeError(f"ffmpeg failed with code {status}; see {output / 'ffmpeg.log'}")
        else: log.close()
    record = {"kind": "gazebo_recorded_pose_visual_replay", "native_gazebo_screen_capture": False,
              "csv": str(csv_path), "csv_sha256": sha(csv_path), "source": str(source), "source_sha256": sha(source),
              "generator_sha256": sha(__file__), "geometry_helper_sha256": sha(HERE / "render_urdf_geometry.py"),
              "model_sha256_values": sorted({r.get("model_sha256", "") for r in data["rows"]}),
              "capture_manifest": capture,
              "frames": len(times), "fps": args.fps, "video_time_scale": 1,
              "recorded_duration_s": duration, "encoded_duration_s": len(times)/args.fps if not args.preview_only else None,
              "max_sample_gap_s": data["max_sample_gap_s"],
              "translation_resampling": "piecewise linear, no smoothing; gaps >0.25s hold previous sample",
              "orientation_resampling": "shortest-path quaternion SLERP, no smoothing",
              "reference": "world-fixed hover goal, not an internal PX4 dynamic takeoff/landing setpoint",
              "motion": "only recorded world base_link pose; no invented rotor angular motion",
              "display_colors": "URDF/default visual colors brightened by 0.20 + 0.80 * RGB for dark-scene readability",
              "geometry": {k: audit[k] for k in ("visual_count", "triangle_count", "unique_visual_meshes", "root_xyz_m", "root_rpy_rad")},
              "metrics": metrics, "preview_only": args.preview_only,
              "video": str(destination) if not args.preview_only else None}
    if not args.preview_only: record["video_sha256"] = sha(destination)
    (output / "replay_manifest.json").write_text(json.dumps(record, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    print(json.dumps(record, ensure_ascii=False, indent=2), flush=True)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--source", type=Path, required=True)
    p.add_argument("--csv", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True)
    p.add_argument("--root-xyz", nargs=3, type=float, default=(0., 0., -.17095298))
    p.add_argument("--root-rpy", nargs=3, type=float, default=(math.pi/2, 0., 0.))
    p.add_argument("--fps", type=int, default=24)
    p.add_argument("--ffmpeg")
    p.add_argument("--preview-only", action="store_true")
    p.add_argument("--no-inset", action="store_true")
    p.add_argument("--overwrite", action="store_true")
    args = p.parse_args()
    if args.fps < 1 or args.fps > 60: p.error("fps must be between 1 and 60")
    render(args)


if __name__ == "__main__":
    main()
