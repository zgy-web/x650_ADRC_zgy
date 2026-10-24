#!/usr/bin/env python3
"""Render one certified isolated normal figure eight; never run new physics.

Example (new output directories only):
  MUJOCO_GL=egl /root/autodl-tmp/amsim-venv/bin/python \
    scripts/render_x650_isolated_validation.py --report AUDIT/report.json \
    --output NEW_PREVIEW --preview-only --preview-time 10

Remove --preview-only for the full 32 s, 24 fps video. --audit-only needs no
MuJoCo, EGL, Pillow or FFmpeg. The original certificate remains authoritative;
this presentation cannot qualify a stage or hardware. A failed/partial run,
hold, or fast figure eight is deliberately not supported by this entry.
"""
from __future__ import annotations

import argparse
import csv
from fractions import Fraction
import importlib.metadata
import json
import math
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "egl")
ROOT = Path(__file__).resolve().parents[1]
for directory in (ROOT, ROOT / "scripts"):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))
import numpy as np
import plot_x650_isolated_validation as accepted
import render_x650_arm_replay as replay

WIDTH, HEIGHT, PANEL_WIDTH, SCENE_HEIGHT, SCENE_Y = 1600, 1140, 800, 650, 150
FPS, DURATION, FK_TOLERANCE_M = 24, 32., 1e-4
BG, FG, MUTED, ACTUAL, TARGET = "#101b2a", "#edf4fb", "#b7c9dd", "#78a9e6", "#f3d568"
sha, read, require = accepted.sha, accepted.read, accepted.require


def identity(path):
    path = Path(path).resolve(strict=True)
    return {"path": str(path), "bytes": path.stat().st_size, "sha256": sha(path)}


def save(path, value):
    Path(path).write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def pinned_file(path, expected):
    item = identity(path)
    require(isinstance(expected, str) and item["sha256"] == expected,
            "Recorded input changed or is not bound: " + str(path))
    return item


def unchanged(inputs):
    for item in inputs.values():
        require(sha(item["path"]) == item["sha256"], "Input changed during presentation: " + item["path"])


def geometry(task, metadata):
    """Use the executed preparation, including its TS20-specific model name."""
    path = Path(task["preparation_report"]).expanduser()
    path = (path if path.is_absolute() else ROOT / path).resolve(strict=True)
    inputs = {"preparation_report": pinned_file(path, metadata["provenance"]["preparation_report_sha256"])}
    report, source = read(path), path.with_name("robot.urdf")
    require(report.get("physical_geometry_validated") is True
            and report.get("arm_self_collision_resolved") is True, "Prepared rigid geometry was not validated")
    require(report["derived_urdf_sha256"] == metadata["source_urdf_sha256"],
            "Executed and prepared URDF identities differ")
    inputs["source_urdf"] = pinned_file(source, report["derived_urdf_sha256"])
    conditional = report.get("conditional_actuator_plant")
    if conditional is not None:
        require(conditional.get("sdf_sha256") == metadata["model_sha256"]
                and conditional.get("rigid_geometry_reused_unchanged") is True,
                "Conditional actuator preparation differs from executed physics")
    robot = ET.parse(source).getroot()
    resources = report.get("mesh_resources", {})
    require(resources and {node.get("filename") for node in robot.findall(".//mesh")} == set(resources),
            "Visual resources do not match the executed preparation")
    for relative, expected in resources.items():
        local = Path(relative)
        file = (source.parent / local).resolve(strict=True)
        require(not local.is_absolute() and ".." not in local.parts and file.is_relative_to(source.parent),
                "Visual resource escapes prepared source tree")
        inputs["mesh:" + relative] = pinned_file(file, expected)
    for name, field in (("clearance_report.json", "clearance_certificate_sha256"),
                        ("arm_self_collision_report.json", "arm_self_collision_report_sha256"),
                        ("kinematics.json", "kinematics_sha256")):
        file = source.with_name(name)
        inputs[name] = pinned_file(file, report[field])
        evidence = read(file)
        if name != "kinematics.json":
            require(evidence.get("passed") is True, "Rigid geometry evidence failed: " + name)
        else:
            require(np.allclose(evidence["elbow_offset_body_m"], [0, .04, 0], rtol=0, atol=1e-14),
                    "Unexpected lateral elbow geometry")
    require(np.allclose(report["controller_model"]["elbow_offset_body_m"], [0, .04, 0], rtol=0, atol=1e-14),
            "Controller geometry differs from prepared two-link topology")
    return source, robot, report, inputs


def native_states(verified):
    """Read every certified physical sample; do not discard invalid states."""
    stamps = verified["values"][:, 0]
    first, last = float(stamps[0]), float(stamps[-1])
    rows = []
    with Path(verified["inputs"]["native_trace"]["path"]).open(newline="", encoding="utf-8") as stream:
        for index, row in enumerate(csv.DictReader(stream)):
            sim = float(row["sim_s"])
            if first <= sim <= last:
                require(all(row[key] == "1" for key in ("valid_base", "valid_joint_state", "valid_tip")),
                        "Invalid native base/joint/tip sample at row " + str(index))
                rows.append([sim, *[float(row["base_p" + axis]) for axis in "xyz"],
                             *[float(row["base_q" + axis]) for axis in "wxyz"],
                             float(row["arm_joint_1_q"]), float(row["arm_joint_2_q"]),
                             *[float(row["tip_" + axis]) for axis in "xyz"]])
    values = np.asarray(rows, float)
    require(values.shape == (len(stamps), 13) and np.isfinite(values).all(), "Incomplete/nonfinite native states")
    require(np.array_equal(values[:, 0], stamps)
            and np.array_equal(values[:, 10:13], verified["values"][:, 2:5]),
            "Render poses and certified native tip do not share exactly the same samples")
    norms = np.linalg.norm(values[:, 4:8], axis=1)
    require(np.all(abs(norms - 1.) <= .01), "Recorded quaternion is not unit length")
    return {"t": stamps.copy(), "elapsed": verified["values"][:, 1].copy(),
            "p": values[:, 1:4], "q": values[:, 4:8] / norms[:, None], "joint_q": values[:, 8:10],
            "provided_tip": values[:, 10:13], "target": verified["desired"].copy(),
            "quaternion_max_norm_correction": float(np.max(abs(norms - 1.)))}


def load_recording(report_path, expected_sha256=None):
    report_path = Path(report_path).resolve(strict=True)
    report_identity = identity(report_path)
    if expected_sha256 is not None:
        require(report_identity["sha256"] == expected_sha256, "Certificate differs from explicit SHA256")
    report = read(report_path)
    require(report.get("stage") == "normal" and report.get("kind") == "x650_isolated_normal_stage_certificate_v1",
            "Only a complete certified isolated normal figure eight is supported")
    verified = accepted.verified_data(report_path)
    require(verified["inputs"]["report"]["sha256"] == report_identity["sha256"], "Certificate changed while loading")
    run = Path(report["source_run_directory"]).resolve(strict=True)
    inputs = dict(verified["inputs"])
    for name, file in (("metadata", "controller_metadata.json"), ("native_manifest", "native_trace_manifest.json")):
        inputs[name] = accepted.checked_input(report, run / file)
    task, metadata, manifest = (read(inputs[key]["path"]) for key in ("task", "metadata", "native_manifest"))
    require(metadata.get("complete") is True and metadata.get("task") == task
            and metadata.get("stage") == "normal" and metadata.get("controller") == "mpc",
            "Recorded controller/task identity differs")
    require(manifest.get("complete") is True and manifest.get("sha256") == inputs["native_trace"]["sha256"]
            and manifest.get("source_urdf_sha256") == metadata["source_urdf_sha256"]
            and manifest.get("model_sha256") == metadata["model_sha256"], "Native recording/model identity differs")
    source, robot, geometry_report, geometry_inputs = geometry(task, metadata)
    inputs.update(geometry_inputs)
    data = native_states(verified)
    fk = replay.audit_forward_kinematics(data, robot, geometry_report, tolerance_m=FK_TOLERANCE_M)
    fk["pose_joint_timing"] = "same native Gazebo physics samples, exactly matched to certified tip timestamps"
    # audit_forward_kinematics adds data['tip']; all visual trails and metrics
    # deliberately keep using provided_tip / verified['norm'], never that FK.
    inputs.update({"renderer": identity(__file__), "verified_data_loader": identity(accepted.__file__),
                   "arm_renderer": identity(replay.__file__), "reference_implementation": identity(accepted.shared.__file__),
                   "native_metric_implementation": identity(sys.modules[accepted.shared.native_tip_metrics.__module__].__file__)})
    # These imported helper sources also participate in the rendering identity.
    for module_name in ("prepare_x650_arm", "render_urdf_geometry", "render_x650_hover"):
        inputs["dependency:" + module_name] = identity(sys.modules[module_name].__file__)
    unchanged(inputs)
    return {"verified": verified, "report": report, "data": data, "task": task, "metadata": metadata,
            "source": source, "robot": robot, "geometry_report": geometry_report, "fk": fk, "inputs": inputs}


def frame_times(duration, fps=FPS, preview_time=None):
    require(type(fps) is int and fps == FPS and math.isfinite(duration) and duration == DURATION,
            "This entry requires the complete 32 s normal stage at 24 fps")
    if preview_time is not None:
        require(math.isfinite(preview_time) and 0 <= preview_time <= duration, "Preview time is outside the task window")
        return np.array([preview_time], float)
    # 768 frames encode exactly 32 seconds; the end state at t=32 is archived
    # separately as final_state.png. Metrics include every full-window sample.
    return np.arange(round(duration * fps), dtype=float) / fps


def past_indices(times, now, maximum):
    require(type(maximum) is int and maximum >= 2, "Invalid trail sample count")
    last = int(np.searchsorted(times, now, side="right")) - 1
    if last < 0:
        return np.empty(0, dtype=int)
    return np.unique(np.linspace(0, last, min(last + 1, maximum)).astype(int))


def add_path(mujoco, scene, points, color, radius):
    """Same capsule overlay as paired replay, without its old-audit imports."""
    rgba = np.array([int(color[k:k + 2], 16) / 255 for k in (1, 3, 5)] + [1.], dtype=np.float32)
    for a, b in zip(points[:-1], points[1:]):
        if np.linalg.norm(b - a) < 1e-8:
            continue
        require(scene.ngeom < scene.maxgeom, "Render overlay capacity exceeded")
        geom = scene.geoms[scene.ngeom]
        mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3), np.eye(3).ravel(), rgba)
        mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, a, b)
        geom.emission = .35
        scene.ngeom += 1


def find_ffmpeg(explicit=None):
    if explicit:
        found = shutil.which(str(explicit))
        require(found is not None, "Explicit FFmpeg executable does not exist")
        return str(Path(found).resolve())
    found = shutil.which("ffmpeg")
    if found:
        return str(Path(found).resolve())
    root = Path("/root/autodl-tmp/flyarm-video-venv/lib")
    found = sorted(p for p in root.glob("python*/site-packages/imageio_ffmpeg/binaries/ffmpeg-*") if p.is_file())
    require(bool(found), "An existing FFmpeg executable is required; no dependency is downloaded")
    return str(found[0].resolve())


def decoded_video_contract(text, frame_count):
    """Validate full FFmpeg framemd5 output; no ffprobe installation needed."""
    dimensions, timebase, frames = {}, {}, []
    for line in text.splitlines():
        match = re.fullmatch(r"#dimensions\s+(\d+):\s+(\d+)x(\d+)", line)
        if match:
            dimensions[int(match[1])] = [int(match[2]), int(match[3])]
        match = re.fullmatch(r"#tb\s+(\d+):\s+(\d+)/(\d+)", line)
        if match:
            timebase[int(match[1])] = Fraction(int(match[2]), int(match[3]))
        if line.strip() and not line.startswith("#"):
            fields = [v.strip() for v in line.split(",")]
            require(len(fields) == 6 and re.fullmatch(r"[0-9a-f]{32}", fields[5]) is not None,
                    "Unexpected decoded-frame metadata")
            frames.append([int(value) for value in fields[:5]])
    require(dimensions == {0: [WIDTH, HEIGHT]} and set(timebase) == {0}, "Decoded video dimensions/stream differ")
    require(len(frames) == frame_count and all(row[0] == 0 and row[4] > 0 for row in frames),
            "Decoded video frame count/stream differs")
    for index, (_, dts, pts, duration, _) in enumerate(frames):
        require(pts * timebase[0] == Fraction(index, FPS) and dts == pts
                and duration * timebase[0] == Fraction(1, FPS), "Decoded video timestamps or frame rate differ")
    return {"full_decode_passed": True, "width": WIDTH, "height": HEIGHT, "fps": FPS,
            "frames": len(frames), "duration_s": len(frames) / FPS, "verification": "complete FFmpeg framemd5 decode"}


def video_decode(ffmpeg, video, frame_count):
    result = subprocess.run([ffmpeg, "-v", "error", "-xerror", "-i", str(video), "-map", "0:v:0", "-an",
                             "-f", "framemd5", "-"], capture_output=True, text=True, check=False)
    require(result.returncode == 0, "Complete video decode failed: " + result.stderr[-2000:])
    return decoded_video_contract(result.stdout, frame_count)


def draw_overlay(ImageDraw, canvas, bundle, now, quat, fonts):
    verified, data = bundle["verified"], bundle["data"]
    duration, t, error_mm = verified["duration"], data["elapsed"], verified["norm"] * 1000
    draw = ImageDraw.Draw(canvas)
    draw.text((26, 12), "NORMAL FIGURE-EIGHT | ISOLATED C++ UPPER MPC", fill=FG, font=fonts[30])
    draw.text((26, 55), "One complete certified run | Actual X650 + two-link TS20 | Precision EV", fill=MUTED, font=fonts[22])
    draw.text((26, 91), "Full target path and recorded actual tip | Same-time native error; no phase/origin alignment", fill=MUTED, font=fonts[20])
    draw.text((26, 126), "Fixed world view", fill=FG, font=fonts[20])
    draw.text((826, 126), "Arm detail (recorded-state following view)", fill=FG, font=fonts[20])
    instant = float(np.interp(now, t, error_mm))
    rms = verified["metrics"]["position_rmse_m"] * 1000
    w, x, y, z = quat
    roll = math.degrees(math.atan2(2 * (w*x + y*z), 1 - 2 * (x*x + y*y)))
    pitch = math.degrees(math.asin(np.clip(2 * (w*y - z*x), -1, 1)))
    draw.text((26, 807), f"EE error {instant:5.2f} mm | Full-window RMS {rms:.2f} mm", fill=ACTUAL, font=fonts[22])
    draw.text((826, 807), f"Actual roll {roll:+.2f} deg | pitch {pitch:+.2f} deg", fill=MUTED, font=fonts[22])
    draw.text((26, 850), "SAME-TIME 3D TIP ERROR [mm]", fill=FG, font=fonts[20])
    draw.text((910, 850), "Yellow: target    Blue: actual past tip", fill=MUTED, font=fonts[20])
    left, right, top, bottom = 95, 1545, 895, 1040
    ymax = max(10., math.ceil(float(error_mm.max()) / 5) * 5)
    for frac in np.linspace(0, 1, 5):
        yy = bottom - (bottom - top) * frac
        draw.line((left, yy, right, yy), fill="#304154")
        draw.text((28, yy - 10), f"{ymax * frac:g}", fill=MUTED, font=fonts[18])
    ids = past_indices(t, now, 2200)
    points = [(left + (right-left)*t[j]/duration, bottom - (bottom-top)*error_mm[j]/ymax) for j in ids]
    if len(points) > 1:
        draw.line(points, fill=ACTUAL, width=3)
    xx = left + (right-left) * now / duration
    draw.line((xx, top, xx, bottom), fill="#9eafc0")
    for at in range(0, 33, 4):
        draw.text((left+(right-left)*at/duration-8, bottom+7), str(at), fill=MUTED, font=fonts[18])
    params = bundle["task"]["trajectory_parameters"]
    reference = verified["reference"]
    phase = "INITIAL HOLD" if now < reference.start_s else "RETURN HOLD" if now >= reference.end_s else "TRACKING"
    draw.text((26, 1080), f"TIME {now:5.2f} s | 1x simulation time | {phase} | X +/-{params['amplitude_x']*100:g} cm, "
              f"Z +/-{params['amplitude_z']*100:g} cm | {params['period_s']:g} s/cycle", fill=MUTED, font=fonts[20])
    draw.text((26, 1114), "Recorded simulation visualization; no physics rerun or hardware footage. Unlogged rotor angles remain fixed.",
              fill=MUTED, font=fonts[18])


def close_media(proc, log, renderer):
    """Reap only our encoder; always release its log and graphics context."""
    try:
        if proc is not None and proc.poll() is None:
            if proc.stdin is not None:
                try:
                    proc.stdin.close()
                except (BrokenPipeError, OSError):
                    pass  # Encoder may have exited between poll and pipe close.
            proc.terminate()
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                proc.kill()
                proc.wait()
    finally:
        try:
            if log is not None:
                log.close()
        finally:
            renderer.close()


def render_frames(bundle, output, frames, preview, ffmpeg):
    # Delay GPU/image imports until all source data and certificate checks pass.
    import mujoco
    from PIL import Image, ImageDraw
    scene_path, scene_details = replay.make_scene(output / "render_geometry", bundle["source"], bundle["robot"], PANEL_WIDTH, SCENE_HEIGHT)
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    state = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=SCENE_HEIGHT, width=PANEL_WIDTH, max_geom=2200)
    proc, log = None, None
    try:
        mocaps = {name: model.body_mocapid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "replay_" + name)]
                  for name in [*replay.kinematic_poses(bundle["robot"]), "tip_reference"]}
        markers = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "tip_reference_" + str(i)) for i in range(3)]
        cameras = [mujoco.MjvCamera(), mujoco.MjvCamera()]
        for camera in cameras:
            mujoco.mjv_defaultCamera(camera)
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.azimuth, camera.elevation = 110., -14.
        cameras[0].distance = 2.05
        cameras[0].lookat[:] = bundle["data"]["p"][0] + [0, 0, -.22]
        cameras[1].distance = .85
        reference = bundle["verified"]["reference"]
        target_path = np.asarray([reference.sample(t)[0] for t in np.linspace(reference.start_s, reference.end_s, 400)])
        fonts = {size: replay.find_font(size) for size in (18, 20, 22, 30)}
        def frame(now):
            data = bundle["data"]
            p, quat, joints, _, gap = replay.sample_state(data, now, gap_limit=.008 + 1e-9)
            require(not gap, "Display interpolation would cross an uncertified native sample gap")
            for name, pose in replay.world_link_poses(bundle["robot"], p, quat, joints).items():
                state.mocap_pos[mocaps[name]] = pose[:3, 3]
                state.mocap_quat[mocaps[name]] = replay.rotation_quaternion(pose[:3, :3])
            state.mocap_pos[mocaps["tip_reference"]] = reference.sample(now)[0]
            model.geom_rgba[markers, 3] = .9
            mujoco.mj_forward(model, state)  # Render kinematics only; mj_step is never called.
            cameras[1].lookat[:] = replay.camera_targets(p, quat, joints, bundle["geometry_report"])[1]
            trail = data["provided_tip"][past_indices(data["elapsed"], now, 360)]
            canvas = Image.new("RGB", (WIDTH, HEIGHT), BG)
            for index, camera in enumerate(cameras):
                renderer.update_scene(state, camera=camera)
                add_path(mujoco, renderer.scene, target_path, TARGET, .0017)
                add_path(mujoco, renderer.scene, trail, ACTUAL, .0015)
                canvas.paste(Image.fromarray(renderer.render().copy()), (index * PANEL_WIDTH, SCENE_Y))
            draw_overlay(ImageDraw, canvas, bundle, now, quat, fonts)
            return canvas
        video = output / "X650_normal_isolated_CPP_MPC.mp4"
        if not preview:
            log = (output / "ffmpeg.log").open("x", encoding="utf-8")
            proc = subprocess.Popen([ffmpeg, "-n", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{WIDTH}x{HEIGHT}",
                "-r", str(FPS), "-i", "-", "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "20",
                "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(video)],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)
        snapshots = set(np.linspace(0, len(frames) - 1, 5).astype(int))
        for index, now in enumerate(frames):
            canvas = frame(float(now))
            if preview:
                canvas.save(output / "preview.png")
            else:
                if index in snapshots:
                    canvas.save(output / f"frame_{index:05d}.png")
                proc.stdin.write(np.asarray(canvas).tobytes())
            if index % 120 == 0:
                print(json.dumps({"frame": index, "total": len(frames), "task_time_s": float(now)}), flush=True)
        if preview:
            video_contract = None
        else:
            proc.stdin.close()
            require(proc.wait() == 0, "Video encoding failed; inspect ffmpeg.log")
            video_contract = video_decode(ffmpeg, video, len(frames))
            frame(DURATION).save(output / "final_state.png")
        return {"geometry": scene_details, "video": None if preview else identity(video),
                "decoded_video": video_contract, "frames": len(frames), "preview_time_s": float(frames[0]) if preview else None,
                "cameras": {"world": {"distance": 2.05, "azimuth": 110., "elevation": -14.,
                                       "lookat": cameras[0].lookat.tolist()},
                            "arm_detail": {"distance": .85, "azimuth": 110., "elevation": -14.,
                                           "lookat": "follows recorded arm geometry; no fitted transform"}}}
    finally:
        close_media(proc, log, renderer)


def render(args):
    output = args.output.resolve()
    require(not output.exists(), "Use a new output directory; existing evidence/media is never overwritten")
    bundle = load_recording(args.report, args.report_sha256)
    for protected in (Path(bundle["report"]["source_run_directory"]), Path(bundle["report"]["launch_directory"]),
                      Path(args.report).resolve().parent, bundle["source"].parent):
        require(not output.is_relative_to(protected.resolve()), "Presentation output must be outside original evidence/model folders")
    frames = frame_times(bundle["verified"]["duration"], args.fps, args.preview_time if args.preview_only else None)
    ffmpeg = find_ffmpeg(args.ffmpeg) if not (args.audit_only or args.preview_only) else None
    inputs = bundle["inputs"]
    if ffmpeg is not None:
        inputs["ffmpeg_executable"] = identity(ffmpeg)
    versions = {}
    for name in ("numpy", "scipy", "mujoco", "Pillow", "matplotlib", "PyYAML"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            versions[name] = None
    record = {"kind": "x650_isolated_normal_recorded_visualization_v1", "complete": False,
              "stage": "normal", "source_certificate_passed": True, "eligible_as_stage_gate": False,
              "hardware_qualified": False, "synthetic": False, "new_physics_run": False,
              "native_gazebo_screen_capture": False, "width": WIDTH, "height": HEIGHT, "fps": args.fps,
              "recorded_duration_s": bundle["verified"]["duration"], "video_time_scale": 1,
              "audit_only": args.audit_only, "preview_only": args.preview_only,
              "layout": "one run; fixed world view plus arm detail; native error versus task time",
              "coordinate_policy": "unchanged absolute world frame; no origin/phase fitting",
              "error_source": "independent native physical tip minus analytic reference at identical native time",
              "instantaneous_error_display": "linear interpolation of native error norm; no smoothing",
              "rms_display": "fixed complete-window time-weighted certified native RMS",
              "primary_metrics": bundle["verified"]["metrics"], "fk_audit": bundle["fk"],
              "pose_display": "linear measured position/joints and SLERP measured orientation; no dynamics integration",
              "boundary_display": {"first_sample_task_s": float(bundle["data"]["elapsed"][0]),
                                   "last_sample_task_s": float(bundle["data"]["elapsed"][-1]),
                                   "policy": "clamp display only to nearest boundary sample; no synthetic trail samples"},
              "rotor_angles": "unlogged; fixed original URDF zero angles",
              "visual_subsampling": {"target_path_points": 400, "past_actual_tip_maximum": 360,
                                     "past_error_chart_maximum": 2200, "metric_subsampling": False},
              "input_hashes": inputs, "python": sys.executable, "package_versions": versions,
              "render_backend": "MuJoCo offscreen rendering only", "mujoco_gl": os.environ.get("MUJOCO_GL")}
    output.mkdir(parents=True, exist_ok=False)
    save(output / "render_status.json", record)
    try:
        if not args.audit_only:
            record.update(render_frames(bundle, output, frames, args.preview_only, ffmpeg))
        unchanged(inputs)
        record.update(complete=True, video_produced=not (args.preview_only or args.audit_only))
        save(output / "replay_manifest.json", record)
        save(output / "render_status.json", record)
        files = {p.name: identity(p) for p in output.iterdir() if p.is_file()}
        save(output / "delivery_manifest.json", {"kind": "x650_recorded_visualization_delivery_v1", "files": files,
            "source_certificate": inputs["report"], "eligible_as_stage_gate": False})
        return record
    except BaseException as error:
        record.update(complete=False, error=repr(error))
        save(output / "render_status.json", record)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--report-sha256", help="Optional explicit SHA256 pin for the reviewed normal certificate")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--fps", type=int, choices=[FPS], default=FPS)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--audit-only", action="store_true")
    mode.add_argument("--preview-only", action="store_true")
    parser.add_argument("--preview-time", type=float, default=10.)
    parser.add_argument("--ffmpeg", help="Existing FFmpeg executable (optional)")
    result = render(parser.parse_args())
    print(json.dumps({key: result.get(key) for key in ("complete", "stage", "video", "frames", "decoded_video", "primary_metrics")}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
