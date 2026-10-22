#!/usr/bin/env python3
"""Paired X650 PID/MPC recorded-state video in the user's comparison layout.

This is visualization only. Native audits remain authoritative. Never step
physics, fit a phase/origin, replace measured joints, or invent rotor motion.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys

os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import render_x650_arm_replay as replay
import plot_x650_mixed_stage_comparison as accepted
from render_x650_hover import find_font

WIDTH, HEIGHT, PANEL_W, SCENE_H, SCENE_Y = 1600, 1320, 800, 720, 210
BG, PANEL, FG, MUTED = "#101b2a", "#15293b", "#edf4fb", "#b7c9dd"
COLORS = ("#78a9e6", "#efaa4c")
TARGET_COLOR = "#f3d568"
LABELS = ("PID cooperative baseline", "Upper MPC")


def require(condition, message):
    if not condition:
        raise ValueError(message)


def dump(path, obj):
    Path(path).write_text(json.dumps(obj, indent=2, allow_nan=False)+"\n", encoding="utf-8")


def bounded_indices(last, count=360):
    """Past samples only, including the first and current native samples."""
    require(last >= 0 and count >= 2, "Invalid trail sampling")
    return np.unique(np.linspace(0, last, min(last+1, count)).astype(int))


def fixed_error_limit(runs):
    peak = max(float(np.max(r["error_mm"])) for r in runs)
    return max(10., math.ceil(peak/10)*10)


def bind_capture(directory, source, robot, report, expected_controller):
    folder = Path(directory).resolve(strict=True)
    csv_path, manifest = folder/"native_replay.csv", folder/"native_replay_manifest.json"
    data = replay.load_samples(csv_path)
    capture = replay.verify_recording(csv_path, source, data, manifest)
    identity = replay.native_display_identity(capture)
    require(identity is not None and identity["controller"] == expected_controller,
            "Panel controller identity differs")
    meta = capture["metadata"]
    require(meta.get("all_window_samples_preserved") is True, "Full audited window required")
    for file, digest in meta["inputs_sha256"].items():
        require(replay.sha(file) == digest, "Original accepted input changed: "+file)
    run = accepted.load_run(meta["source_audit"], meta["source_run_directory"], expected_controller)
    require(run["report"]["stage"] == identity["stage"], "Capture/audit stage mismatch")
    fk = replay.audit_forward_kinematics(data, robot, report)
    physical = run["physical"]
    require(np.allclose(data["elapsed"], physical["t"], rtol=0, atol=1e-9)
            and np.allclose(data["provided_tip"], physical["tip"], rtol=0, atol=1e-12)
            and np.allclose(data["target"], run["target"], rtol=0, atol=1e-12),
            "Replay source differs from accepted native positions, reference, or time")
    # Display errors always use independent native tip, not interpolated render FK.
    error_mm = np.linalg.norm(run["error"], axis=1)*1000
    require(np.isfinite(error_mm).all() and data["max_sample_gap_s"] <= .02,
            "Missing or discontinuous native states")
    return dict(data=data, capture=capture, identity=identity, run=run, fk=fk,
                error_mm=error_mm, rms_mm=1000*run["replay"]["rmse_3d"],
                input_hashes={str(csv_path): replay.sha(csv_path), str(manifest): replay.sha(manifest),
                              **meta["inputs_sha256"]})


def rgba(hex_color):
    return np.array([int(hex_color[k:k+2], 16)/255 for k in (1, 3, 5)]+[1.], dtype=np.float32)


def add_segment(mujoco, scene, a, b, color, radius):
    if np.linalg.norm(b-a) < 1e-8:
        return
    require(scene.ngeom < scene.maxgeom, "Scene capacity exceeded")
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, np.zeros(3), np.zeros(3),
                       np.eye(3).ravel(), rgba(color))
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_CAPSULE, radius, a, b)
    geom.emission = .35
    scene.ngeom += 1


def add_path(mujoco, scene, points, color, radius):
    for a, b in zip(points[:-1], points[1:]):
        add_segment(mujoco, scene, a, b, color, radius)


def euler_degrees(quat):
    w, x, y, z = quat
    return (math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))),
            math.degrees(math.asin(np.clip(2*(w*y-z*x), -1, 1))))


def error_chart(draw, runs, now, duration, ymax):
    left, right, top, bottom = 92, WIDTH-35, 1010, 1220
    draw.text((30, 965), "SAME-TIME END-EFFECTOR ERROR  [mm]", fill=FG, font=find_font(23, True))
    for fraction in np.linspace(0, 1, 5):
        y = bottom-(bottom-top)*fraction
        draw.line((left, y, right, y), fill="#2b4058", width=1)
        draw.text((30, y-12), f"{ymax*fraction:g}", fill=MUTED, font=find_font(19))
    for t in np.linspace(0, duration, 9):
        x = left+(right-left)*t/duration
        draw.text((x-12, bottom+10), f"{t:.0f}", fill=MUTED, font=find_font(18))
    for run, color in zip(runs, COLORS):
        times = run["data"]["elapsed"]
        last = int(np.searchsorted(times, now, side="right"))-1
        ids = bounded_indices(max(0, last), 2200)
        points = [(left+(right-left)*times[i]/duration, bottom-(bottom-top)*run["error_mm"][i]/ymax)
                  for i in ids]
        if len(points) > 1:
            draw.line(points, fill=color, width=3)
    x = left+(right-left)*now/duration
    draw.line((x, top, x, bottom), fill="#9eafc0", width=1)
    draw.text((1075, 968), "PID", fill=COLORS[0], font=find_font(21, True))
    draw.text((1170, 968), "MPC", fill=COLORS[1], font=find_font(21, True))
    draw.text((1300, 970), f"Shared 0-{ymax:g} mm", fill=MUTED, font=find_font(18))


def find_ffmpeg(explicit):
    if explicit:
        return explicit
    found = shutil.which("ffmpeg")
    if found:
        return found
    root = Path("/root/autodl-tmp/flyarm-video-venv/lib")
    return next((str(p) for p in root.glob("python*/site-packages/imageio_ffmpeg/binaries/ffmpeg-*")), None)


def render(args):
    from PIL import Image, ImageDraw
    import mujoco

    source, out = args.source.resolve(strict=True), args.output.resolve()
    require(not out.exists(), "Use a new output directory; do not overwrite accepted media")
    robot, report = replay.load_composition(source)
    runs = [bind_capture(folder, source, robot, report, controller)
            for folder, controller in ((args.pid, "pid_ff"), (args.mpc, "mpc"))]
    accepted.verify_pair(*(r["run"] for r in runs))
    duration = runs[0]["data"]["elapsed"][-1]
    require(abs(duration-runs[1]["data"]["elapsed"][-1]) < 1e-8, "Pair durations differ")
    stage = runs[0]["identity"]["stage"]
    require(stage in ("small", "normal", "fast"), "Figure-eight stage required")
    params = runs[0]["run"]["task"]["trajectory_parameters"]
    out.mkdir(parents=True)
    scene_path, geometry = replay.make_scene(out/"render_geometry", source, robot, PANEL_W, SCENE_H)
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    state = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=SCENE_H, width=PANEL_W, max_geom=2200)
    mocaps = {name: model.body_mocapid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "replay_"+name)]
              for name in [*replay.kinematic_poses(robot), "tip_reference"]}
    marker_geoms = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"tip_reference_{axis}")
                    for axis in range(3)]
    camera = mujoco.MjvCamera()
    mujoco.mjv_defaultCamera(camera)
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.distance, camera.azimuth, camera.elevation = 1.75, 110., -14.
    # A single fixed world camera, derived from both initial base positions.
    # No per-panel recentering, tracking, or projection adjustment.
    camera.lookat[:] = np.mean([r["data"]["p"][0] for r in runs], axis=0)+[0, 0, -.20]
    target_paths = [r["data"]["target"][bounded_indices(len(r["data"]["t"])-1, 400)] for r in runs]
    ymax = fixed_error_limit(runs)
    frames = np.arange(0., duration+1e-9, 1/args.fps)
    if args.preview_only:
        require(0 <= args.preview_time <= duration, "Preview time outside recording")
        frames = np.array([args.preview_time])
    record = dict(kind="x650_paired_native_figure8_replay", complete=False, stage=stage,
                  source_urdf_sha256=replay.sha(source), renderer_sha256=replay.sha(__file__),
                  dependencies_sha256={str(Path(m.__file__).resolve()): replay.sha(m.__file__)
                                       for m in (replay, accepted)},
                  synthetic=False, new_dynamics_simulated=False, native_gazebo_screen_capture=False,
                  layout="left PID / right MPC; common native-error chart; full target path + past actual trail",
                  width=WIDTH, height=HEIGHT, fps=args.fps, video_time_scale=1,
                  time_alignment="same elapsed time from each predeclared measurement start; no fitted shift",
                  coordinate_policy="original absolute world positions retained; one identical fixed camera for both panels",
                  camera=dict(lookat=camera.lookat.tolist(), distance=camera.distance,
                              azimuth=camera.azimuth, elevation=camera.elevation),
                  interpolation="linear recorded position/joints + SLERP orientation; no state smoothing",
                  error_source="native tip minus analytic reference at the same native time",
                  instantaneous_error_display="linear interpolation of native error norm",
                  rmse_display="fixed full-window time-weighted accepted native RMSE",
                  target_path="entire declared native-time reference, visually subsampled only",
                  actual_trail="past native actual tip samples only, visually subsampled; no spatial offset",
                  rotor_angles="unlogged: fixed source angles", common_error_axis_mm=[0, ymax],
                  recorded_duration_s=float(duration), preview_only=args.preview_only,
                  geometry=geometry, runs=[dict(identity=r["identity"], fk_audit=r["fk"],
                                               full_window_rms_mm=r["rms_mm"],
                                               input_hashes=r["input_hashes"]) for r in runs])
    dump(out/"render_status.json", record)
    video = out/("X650_"+stage+"_PID_vs_MPC.mp4")
    ffmpeg, proc, log = find_ffmpeg(args.ffmpeg), None, None
    if not args.preview_only:
        require(ffmpeg is not None, "Use an existing ffmpeg installation")
        log = (out/"ffmpeg.log").open("x")
        proc = subprocess.Popen([ffmpeg, "-n", "-f", "rawvideo", "-pix_fmt", "rgb24",
                                 "-s", f"{WIDTH}x{HEIGHT}", "-r", str(args.fps), "-i", "-",
                                 "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "20",
                                 "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(video)],
                                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)
    snapshots = set(np.linspace(0, len(frames)-1, 5).astype(int))
    fonts = {size: find_font(size) for size in (18, 20, 22, 24)}
    try:
        for frame_index, now in enumerate(frames):
            canvas = Image.new("RGB", (WIDTH, HEIGHT), BG)
            panel_values = []
            for panel, (run, color, target_path) in enumerate(zip(runs, COLORS, target_paths)):
                data = run["data"]
                p, quat, joints, index, gap = replay.sample_state(data, now)
                require(not gap, "Frame interpolation crossed an unaccepted gap")
                for name, pose in replay.world_link_poses(robot, p, quat, joints).items():
                    state.mocap_pos[mocaps[name]] = pose[:3, 3]
                    state.mocap_quat[mocaps[name]] = replay.rotation_quaternion(pose[:3, :3])
                target = np.array([np.interp(now, data["elapsed"], data["target"][:, j]) for j in range(3)])
                state.mocap_pos[mocaps["tip_reference"]] = target
                model.geom_rgba[marker_geoms, 3] = .9
                mujoco.mj_forward(model, state)
                renderer.update_scene(state, camera=camera)
                add_path(mujoco, renderer.scene, target_path, TARGET_COLOR, .0017)
                add_path(mujoco, renderer.scene, data["provided_tip"][bounded_indices(index)], color, .0015)
                canvas.paste(Image.fromarray(renderer.render().copy()), (panel*PANEL_W, SCENE_Y))
                panel_values.append((float(np.interp(now, data["elapsed"], run["error_mm"])), euler_degrees(quat)))
            draw = ImageDraw.Draw(canvas)
            draw.rectangle((0, 0, WIDTH, 96), fill=BG)
            draw.text((30, 15), f"{stage.upper()} FIGURE-EIGHT  |  X650 WHOLE-BODY: PID / MPC",
                      fill=FG, font=find_font(29, True))
            draw.text((30, 57), "Same frozen PX4 PID / limits. Precision EV. Acceleration FF = 0; velocity FF retained.",
                      fill=MUTED, font=fonts[20])
            for panel, (run, color, values) in enumerate(zip(runs, COLORS, panel_values)):
                x = panel*PANEL_W
                instant, (roll, pitch) = values
                draw.rectangle((x+15, 108, x+PANEL_W-15, 235), fill=PANEL)
                draw.text((x+32, 119), LABELS[panel], fill=color, font=find_font(28, True))
                draw.text((x+32, 160), f"EE error  {instant:5.1f} mm   |   Full-window RMSE {run['rms_mm']:.2f} mm",
                          fill=MUTED, font=fonts[22])
                draw.text((x+32, 201), f"Actual body: roll {roll:+.2f} deg  |  pitch {pitch:+.2f} deg",
                          fill=MUTED, font=fonts[20])
                draw.text((x+30, 910), "Target path / marker", fill=TARGET_COLOR, font=fonts[20])
                draw.text((x+395, 910), "Actual tip trail", fill=color, font=fonts[20])
            draw.line((PANEL_W, 99, PANEL_W, 941), fill="#5c6e80", width=1)
            error_chart(draw, runs, now, duration, ymax)
            draw.text((30, 1264), f"TIME {now:5.2f} s  |  1x simulation time  |  "
                      f"X +/-{params['amplitude_x']*100:g} cm, Z +/-{params['amplitude_z']*100:g} cm"
                      f"  |  {params['period_s']:g} s/cycle, {params['cycles']} cycles",
                      fill=MUTED, font=fonts[22])
            draw.text((30, 1300), "Recorded simulation visualization; no physics rerun or hardware footage. No phase/origin fit.",
                      fill=MUTED, font=fonts[18])
            if frame_index in snapshots:
                canvas.save(out/f"frame_{frame_index:05d}.png")
            if args.preview_only:
                canvas.save(out/"preview.png")
            elif proc is not None:
                proc.stdin.write(np.asarray(canvas).tobytes())
            if frame_index % 120 == 0:
                print(json.dumps({"stage": stage, "frame": frame_index, "total": len(frames)}), flush=True)
        if proc is not None:
            proc.stdin.close()
            require(proc.wait() == 0, "ffmpeg encoding failed; inspect ffmpeg.log")
            checked = subprocess.run([ffmpeg, "-v", "error", "-i", str(video), "-f", "null", "-"],
                                     capture_output=True, text=True)
            require(checked.returncode == 0, "Complete decode failed: "+checked.stderr[-1500:])
        for run in runs:
            for file, digest in run["input_hashes"].items():
                require(replay.sha(file) == digest, "Input changed during rendering: "+file)
        record.update(complete=True, frames=len(frames), full_video_decode_passed=not args.preview_only,
                      video=None if args.preview_only else str(video),
                      video_sha256=None if args.preview_only else replay.sha(video),
                      encoded_duration_s=None if args.preview_only else len(frames)/args.fps)
        dump(out/"replay_manifest.json", record)
        dump(out/"render_status.json", record)
        return {k: record[k] for k in ("complete", "stage", "video", "frames", "full_video_decode_passed")}
    finally:
        renderer.close()
        if proc is not None and proc.poll() is None:
            proc.terminate()
            proc.wait()
        if log is not None:
            log.close()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for key in ("source", "pid", "mpc", "output"):
        parser.add_argument("--"+key, type=Path, required=True)
    parser.add_argument("--fps", type=int, choices=[24], default=24)
    parser.add_argument("--preview-only", action="store_true")
    parser.add_argument("--preview-time", type=float, default=10.)
    parser.add_argument("--ffmpeg")
    print(json.dumps(render(parser.parse_args()), indent=2), flush=True)


if __name__ == "__main__":
    main()
