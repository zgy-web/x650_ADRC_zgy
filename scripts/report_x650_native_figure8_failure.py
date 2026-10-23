"""Present a failed, safely recovered figure-eight without promoting it to acceptance."""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np
ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT / "scripts"), str(ROOT / "control_packages/x650_wholebody_v1")]
from simulations.x650_wholebody_mpc_v1.run_case import FigureEight
from report_x650_native_hold import identity, statistics
import render_x650_arm_replay as replay
from render_x650_hover import find_font
from render_x650_paired_figure8 import add_path, bounded_indices, euler_degrees, find_ffmpeg


def read(path):
    return json.loads(Path(path).read_text())


def save(path, obj):
    Path(path).write_text(json.dumps(obj, indent=2, allow_nan=False) + "\n")


def require(value, text):
    if not value:
        raise ValueError(text)


def load(run, launch):
    meta, cfg, stage = (read(run / n) for n in ("controller_metadata.json", "controller_task.json", "stage_result.json"))
    launched, provenance = read(launch / "launch.json"), read(launch / "platform_provenance.json")
    failure = read(run / "control_failure_snapshot.json")
    require(meta["complete"] is False and stage["passed"] is False and launched["complete"] is False,
            "This report is exclusively for a failed run")
    require(meta["landed_and_disarmed"] and meta["diagnostic_parameters_restored"]
            and meta["diagnostic_restore_pending"] is False and launched["runtime_closed"],
            "Recovery and parameter restoration must be complete")
    require(launched["run_directories"] == [str(run)] and launched["task"] == cfg == meta["task"]
            and provenance["source_unchanged"] is True, "Same-run/source binding differs")
    require(failure["event"] == "ts20_control_interval_rejected" and failure["phase"] == "measure"
            and failure["command_published"] is False, "Expected real interval-rejection evidence")
    inputs = {n: identity(run / n) for n in ("controller_metadata.json", "controller_task.json", "stage_result.json",
        "native_trace.csv", "native_trace_manifest.json", "control_trace.csv", "controller_diagnostics.jsonl",
        "control_failure_snapshot.json", "loop_timing_evidence.json", "px4.ulg")}
    for file, sha in stage["artifacts"].items():
        require(replay.sha(run / file) == sha, "Original failed-run artifact changed: " + file)
    require(read(run / "native_trace_manifest.json")["sha256"] == replay.sha(run / "native_trace.csv"),
            "Native trace hash mismatch")
    origin, stop = float(meta["task_start_sim_s"]), float(failure["t_s"])
    raw = [row for row in csv.DictReader((run / "native_trace.csv").open())
           if -1e-9 <= float(row["sim_s"]) - origin <= stop + 1e-9]
    def values(names):
        return np.array([[float(row[n]) for n in names] for row in raw])
    stamps = values(["sim_s"]).ravel()
    t = stamps - origin
    require(len(t) > 1 and abs(t[0]) < 1e-8 and stop - t[-1] <= .004 + 1e-9
            and np.all(np.diff(t) > 0) and np.diff(t).max() <= .008 + 1e-9
            and all(row["valid_tip"] == "1" for row in raw), "Incomplete pre-rejection physical interval")
    from x650_wholebody.model import ArmModel, tip_world
    model = ArmModel.from_report(cfg["preparation_report"])
    center = tip_world(np.asarray(meta["reference_base_enu_m"]), np.asarray(meta["reference_rotation"]), np.zeros(2), model)
    trajectory = FigureEight(center, **cfg["trajectory_parameters"])
    target = np.array([trajectory.sample(x)[0] for x in t])
    data = {"t": stamps, "elapsed": t, "p": values(["base_p" + a for a in "xyz"]),
            "q": values(["base_q" + a for a in "wxyz"]),
            "joint_q": values(["arm_joint_1_q", "arm_joint_2_q"]),
            "joint_dq": values(["arm_joint_1_qd", "arm_joint_2_qd"]),
            "provided_tip": values(["tip_" + a for a in "xyz"]), "target": target}
    require(all(np.isfinite(v).all() for v in data.values()), "Nonfinite physical samples")
    error = np.linalg.norm(data["provided_tip"] - target, axis=1)
    def metrics(mask):
        tm, e = t[mask], error[mask]
        return {"start_s": float(tm[0]), "end_s": float(tm[-1]), "samples": len(tm),
                "rms_m": float(np.sqrt(np.trapz(e * e, tm) / (tm[-1] - tm[0]))), "peak_m": float(e.max())}
    controls = [row for row in csv.DictReader((run / "control_trace.csv").open()) if row["phase"] == "measure"]
    ct = np.array([float(x["t_s"]) for x in controls])
    compute = np.array([float(x["compute_ms"]) for x in controls])
    diagnostics = [json.loads(line) for line in (run / "controller_diagnostics.jsonl").open() if line.strip()]
    diagnostics = [x["diagnostics"] for x in diagnostics if x.get("phase") == "measure"]
    require(len(controls) == len(diagnostics), "Returned controls and diagnostics differ")
    solve = np.array([x["solve_ms"] for x in diagnostics])
    events = read(run / "loop_timing_evidence.json")["natural_gc"]["completed_events"]
    previous = failure["loop_timing"]["previous_control_cycles"][-1]
    marks = previous["marks"]
    a, b = marks["read_returned"]["wall_monotonic_s"], marks["joint1_publish_trace_entry"]["wall_monotonic_s"]
    overlapping = [x for x in events if x["start_wall_monotonic_s"] < b and x["stop_wall_monotonic_s"] > a]
    report = {"kind": "native_figure8_failed_attempt_presentation_v1", "report_complete": True,
        "experiment_passed": False, "partial_measurement_only": True, "stage": cfg["stage"],
        "run": str(run), "launch": str(launch), "planned_duration_s": cfg["duration_s"],
        "rejected_at_s": stop, "rejected_interval_ms": 1000 * failure["dt_s"],
        "partial_metrics": metrics(np.ones(len(t), dtype=bool)), "partial_tracking_metrics": metrics(t >= trajectory.start_s),
        "completed_control_updates": len(controls),
        "returned_candidates_all_accepted": all(d["accepted_optimum"] and d["ts20_nonlinear_check_passed"]
            and not d.get("previous_plan_fallback") and not d.get("solve_budget_exceeded") for d in diagnostics),
        "timing": {"compute_wall_ms": statistics(compute), "qp_wall_ms": statistics(solve),
                   "accepted_update_interval_sim_ms": statistics(np.diff(ct) * 1000)},
        "last_returned_compute_ms": float(compute[-1]), "last_returned_qp_ms": float(solve[-1]),
        "gc_overlapping_last_compute": overlapping,
        "actual_joint_q_min_rad": data["joint_q"].min(axis=0).tolist(),
        "actual_joint_q_max_rad": data["joint_q"].max(axis=0).tolist(),
        "actual_joint_speed_peak_rad_s": np.abs(data["joint_dq"]).max(axis=0).tolist(),
        "landed_disarmed_restored": True, "runtime_closed": True, "frozen_sources_unchanged": True,
        "fast_figure8_started": False, "trajectory_parameters": cfg["trajectory_parameters"],
        "numerical_backend": cfg["numerical_backend"], "inputs": inputs,
        "time_or_position_fit": False, "new_physics_run": False,
        "limitation": "Partial interval before a real rejected update. No full-window pass or performance claim."}
    return report, data, cfg, meta, trajectory, error, ct, compute, solve


def plots(out, data, trajectory, error, ct, compute, solve, report):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = data["elapsed"]
    path = np.array([trajectory.sample(x)[0] for x in np.linspace(trajectory.start_s, trajectory.end_s, 1501)])
    fig, ax = plt.subplots(2, 2, figsize=(12, 8))
    fig.suptitle("X650 + TS20 | Frozen C++ MPC | NORMAL FIGURE-EIGHT: INCOMPLETE", color="#a32b29", fontsize=15)
    a = ax[0, 0]
    a.plot(path[:, 0], path[:, 2], "--", color="#d19724", label="Full planned path")
    a.plot(data["provided_tip"][:, 0], data["provided_tip"][:, 2], color="#147cb1", label="Actual before rejection")
    a.scatter(*data["provided_tip"][-1, [0, 2]], color="#bd3937", s=30, label="Protection stop")
    a.set(xlabel="World X [m]", ylabel="World Z [m]", title="Actual native tip; no phase or origin fit")
    a.axis("equal"); a.legend(fontsize=8)
    a = ax[0, 1]; a.plot(t, error * 1000, color="#147cb1")
    a.set(xlabel="Task time [s]", ylabel="3D tip error [mm]", title="Partial interval only: 0 to %.3f s" % t[-1])
    a = ax[1, 0]
    for i in range(2):
        a.plot(t, np.degrees(data["joint_q"][:, i]), label="Actual joint %d" % (i + 1))
    a.set(xlabel="Task time [s]", ylabel="Joint angle [deg]"); a.legend()
    a = ax[1, 1]; a.plot(ct, compute, label="Compute + dispatch (wall)", color="#7754aa")
    a.plot(ct, solve, label="QP (wall)", color="#2d9c7b")
    a.axhline(50, color="#bd3937", linestyle="--", label="50 ms limit")
    a.scatter([report["rejected_at_s"]], [report["rejected_interval_ms"]], color="#bd3937", marker="x", s=65,
              label="Rejected interval (sim clock)")
    a.set(xlabel="Task time [s]", ylabel="Time [ms]", ylim=(0, 58)); a.legend(fontsize=8)
    for a in ax.flat:
        a.grid(alpha=.2)
    fig.text(.5, .015, "Stopped at 14.34 s / planned 32 s. Safe recovery completed. Fast figure-eight was not started.", ha="center", fontsize=10)
    fig.tight_layout(rect=(0, .035, 1, .95)); fig.savefig(out / "normal_partial.png", dpi=170)
    fig.savefig(out / "normal_partial.pdf"); plt.close(fig)


def geometry(cfg, meta):
    p = Path(cfg["preparation_report"]); doc = read(p); source = p.with_name("robot.urdf")
    require(replay.sha(p) == meta["provenance"]["preparation_report_sha256"]
            and replay.sha(source) == doc["derived_urdf_sha256"] == meta["source_urdf_sha256"],
            "Render geometry differs from the executed model")
    robot = ET.parse(source).getroot()
    referenced = {n.get("filename") for n in robot.findall(".//mesh")}
    require(referenced == set(doc["mesh_resources"]), "Visual resource scope differs")
    for relative, expected in doc["mesh_resources"].items():
        file = (source.parent / relative).resolve()
        require(file.is_relative_to(source.parent) and replay.sha(file) == expected, "Mesh source changed")
    return source, robot, doc


def video(out, data, cfg, meta, trajectory, error, report, fps):
    from PIL import Image, ImageDraw
    import mujoco
    source, robot, geometry_report = geometry(cfg, meta)
    fk = replay.audit_forward_kinematics(data, robot, geometry_report)
    scene, details = replay.make_scene(out / "render_geometry", source, robot, 800, 650)
    model = mujoco.MjModel.from_xml_path(str(scene)); state = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=650, width=800, max_geom=2200)
    ids = {name: model.body_mocapid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "replay_" + name)]
           for name in [*replay.kinematic_poses(robot), "tip_reference"]}
    markers = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "tip_reference_" + str(i)) for i in range(3)]
    cameras = [mujoco.MjvCamera(), mujoco.MjvCamera()]
    for cam in cameras:
        mujoco.mjv_defaultCamera(cam); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
        cam.azimuth = 110.; cam.elevation = -14.
    cameras[0].distance = 2.05; cameras[0].lookat[:] = data["p"][0] + [0, 0, -.22]
    cameras[1].distance = .85
    target_path = np.array([trajectory.sample(x)[0] for x in np.linspace(trajectory.start_s, trajectory.end_s, 360)])
    frames = np.arange(0, data["elapsed"][-1] + 1e-9, 1 / fps)
    destination = out / "X650_normal_CPP_MPC_INCOMPLETE.mp4"
    ffmpeg = find_ffmpeg(None); require(ffmpeg is not None, "Existing ffmpeg required")
    log = (out / "ffmpeg.log").open("x")
    proc = subprocess.Popen([ffmpeg, "-n", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "1600x1140", "-r", str(fps),
        "-i", "-", "-an", "-c:v", "libx264", "-preset", "fast", "-crf", "20", "-pix_fmt", "yuv420p",
        "-movflags", "+faststart", str(destination)], stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)
    bg, fg, muted, blue, yellow = "#101b2a", "#edf4fb", "#b7c9dd", "#78a9e6", "#f3d568"
    fonts = {n: find_font(n) for n in (18, 20, 22, 26, 30)}
    ymax = max(10., np.ceil(error.max() * 1000 / 5) * 5)
    try:
        for k, now in enumerate(frames):
            p, quat, q, index, gap = replay.sample_state(data, now, gap_limit=.0081)
            require(not gap, "Unexpected native interpolation gap")
            for name, pose in replay.world_link_poses(robot, p, quat, q).items():
                state.mocap_pos[ids[name]] = pose[:3, 3]
                state.mocap_quat[ids[name]] = replay.rotation_quaternion(pose[:3, :3])
            state.mocap_pos[ids["tip_reference"]] = trajectory.sample(now)[0]
            model.geom_rgba[markers, 3] = .9
            mujoco.mj_forward(model, state)
            cameras[1].lookat[:] = replay.camera_targets(p, quat, q, geometry_report)[1]
            canvas = Image.new("RGB", (1600, 1140), bg)
            for i, cam in enumerate(cameras):
                renderer.update_scene(state, camera=cam)
                add_path(mujoco, renderer.scene, target_path, yellow, .0017)
                add_path(mujoco, renderer.scene, data["provided_tip"][bounded_indices(index)], blue, .0015)
                canvas.paste(Image.fromarray(renderer.render().copy()), (i * 800, 150))
            draw = ImageDraw.Draw(canvas)
            draw.text((26, 12), "NORMAL FIGURE-EIGHT | FROZEN C++ UPPER MPC", fill=fg, font=fonts[30])
            draw.text((26, 55), "INCOMPLETE: 52 ms update gap at 14.34 s; limit 50 ms. Fast trial NOT STARTED.", fill="#ff9a85", font=fonts[22])
            draw.text((26, 93), "Actual X650 + two-link TS20 | Same frozen PID / MPC / limits | Precision EV", fill=muted, font=fonts[20])
            draw.text((26, 126), "Fixed world view", fill=fg, font=fonts[20]); draw.text((826, 126), "Arm detail (recorded-state following view)", fill=fg, font=fonts[20])
            instant = np.interp(now, data["elapsed"], error) * 1000
            roll, pitch = euler_degrees(quat)
            draw.text((26, 807), f"EE error {instant:5.2f} mm | Partial RMS {report['partial_metrics']['rms_m']*1000:.2f} mm", fill=blue, font=fonts[22])
            draw.text((826, 807), f"Actual roll {roll:+.2f} deg | pitch {pitch:+.2f} deg", fill=muted, font=fonts[22])
            left, right, top, bottom = 95, 1545, 880, 1040
            for frac in np.linspace(0, 1, 5):
                y = bottom - (bottom - top) * frac
                draw.line((left, y, right, y), fill="#304154")
                draw.text((28, y - 10), f"{ymax*frac:g}", fill=muted, font=fonts[18])
            selected = bounded_indices(index, 1500)
            points = [(left + (right-left)*data["elapsed"][j]/report["planned_duration_s"],
                       bottom - (bottom-top)*error[j]*1000/ymax) for j in selected]
            if len(points) > 1:
                draw.line(points, fill=blue, width=3)
            xstop = left + (right-left)*report["rejected_at_s"]/report["planned_duration_s"]
            draw.line((xstop, top, xstop, bottom), fill="#ff9a85", width=2)
            draw.text((xstop+12, top+20), "No accepted recording beyond stop", fill="#ff9a85", font=fonts[20])
            for x in range(0, 33, 4):
                draw.text((left+(right-left)*x/32-8, bottom+6), str(x), fill=muted, font=fonts[18])
            draw.text((26, 850), "SAME-TIME 3D TIP ERROR [mm]", fill=fg, font=fonts[20])
            draw.text((26, 1080), f"TIME {now:5.2f} s | 1x simulation time | X +/-25 cm, Z +/-10 cm | planned 12 s/cycle, 2 cycles", fill=muted, font=fonts[20])
            draw.text((26, 1114), "Recorded simulation visualization, not live footage; no physics rerun. Yellow: target; blue: actual past tip.", fill=muted, font=fonts[18])
            if k in {0, len(frames)//2, len(frames)-1}:
                canvas.save(out / f"frame_{k:05d}.png")
            proc.stdin.write(np.asarray(canvas).tobytes())
            if k % 120 == 0:
                print(json.dumps({"frame": k, "total": len(frames)}), flush=True)
        proc.stdin.close(); require(proc.wait() == 0, "Video encoding failed")
    finally:
        if proc.poll() is None:
            proc.kill(); proc.wait()
        renderer.close(); log.close()
    result = subprocess.run([ffmpeg, "-v", "error", "-i", str(destination), "-f", "null", "-"], capture_output=True)
    require(result.returncode == 0, "Complete video decode failed")
    return {"complete": True, "video": identity(destination), "width": 1600, "height": 1140,
            "fps": fps, "frames": len(frames), "time_scale": 1, "full_decode_passed": True,
            "experiment_passed": False, "new_physics_run": False, "fk": fk, "geometry": details,
            "renderer": identity(__file__), "source_urdf": identity(source)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--run-dir", type=Path, required=True); p.add_argument("--launch-dir", type=Path, required=True)
    p.add_argument("--output", type=Path, required=True); p.add_argument("--video", action="store_true")
    p.add_argument("--fps", type=int, default=24); args = p.parse_args()
    report, data, cfg, meta, trajectory, error, ct, compute, solve = load(args.run_dir.resolve(), args.launch_dir.resolve())
    args.output.mkdir(parents=True, exist_ok=False)
    plots(args.output, data, trajectory, error, ct, compute, solve, report)
    save(args.output / "partial_report.json", report)
    if args.video:
        save(args.output / "render_status.json", video(args.output, data, cfg, meta, trajectory, error, report, args.fps))
    for value in report["inputs"].values():
        require(replay.sha(value["path"]) == value["sha256"], "Input changed during presentation")
    save(args.output / "delivery_manifest.json", {f.name: identity(f) for f in args.output.iterdir()
         if f.is_file() and f.suffix in (".json", ".png", ".pdf", ".mp4")})
    print(json.dumps({"report_complete": True, "experiment_passed": False, "output": str(args.output),
                      "partial_metrics": report["partial_metrics"]}), flush=True)


if __name__ == "__main__":
    main()
