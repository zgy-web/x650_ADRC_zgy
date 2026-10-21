#!/usr/bin/env python3
"""Render recorded X650 + two-link states; never integrate new dynamics.

The source is the prepared v2 URDF. All original 281 visual STL meshes are
retained. Independent mocap bodies display the logged base pose and measured
joint angles. Unlogged rotor angles stay at their URDF zero values.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import xml.etree.ElementTree as ET

os.environ.setdefault("MUJOCO_GL", "egl")
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from prepare_x650_arm import kinematic_poses, matrix, origin
from render_urdf_geometry import load_stl, sha
from render_x650_hover import (write_binary_stl, sample_pose, find_font, draw_chart,
                               BG, PANEL, FG, MUTED, BLUE, GOLD, GREEN, RED)

REQUIRED = ("sim_time_s", "phase", "world_x_m", "world_y_m", "world_z_m",
            "world_qx", "world_qy", "world_qz", "world_qw", "q1_rad", "q2_rad")
TIP = ("tip_world_x_m", "tip_world_y_m", "tip_world_z_m")
TARGET = ("tip_target_world_x_m", "tip_target_world_y_m", "tip_target_world_z_m")


def quaternion_rotation(wxyz):
    w, x, y, z = np.asarray(wxyz, dtype=float)
    return np.array([[1-2*(y*y+z*z), 2*(x*y-w*z), 2*(x*z+w*y)],
                     [2*(x*y+w*z), 1-2*(x*x+z*z), 2*(y*z-w*x)],
                     [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x*x+y*y)]])


def rotation_quaternion(r):
    """Active rotation matrix to wxyz, with deterministic positive w."""
    trace = float(np.trace(r))
    if trace > 0:
        s = math.sqrt(trace+1)*2
        q = np.array([s/4, (r[2, 1]-r[1, 2])/s, (r[0, 2]-r[2, 0])/s, (r[1, 0]-r[0, 1])/s])
    else:
        i = int(np.argmax(np.diag(r)))
        j, k = (i+1)%3, (i+2)%3
        s = math.sqrt(max(0., 1+r[i, i]-r[j, j]-r[k, k]))*2
        q = np.zeros(4)
        q[0] = (r[k, j]-r[j, k])/s
        q[i+1], q[j+1], q[k+1] = s/4, (r[j, i]+r[i, j])/s, (r[k, i]+r[i, k])/s
    q /= np.linalg.norm(q)
    return -q if q[0] < 0 else q


def load_composition(source):
    source = Path(source).resolve(strict=True)
    report_path = source.with_name("preparation_report.json")
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if report.get("model_name") != "x650_2link_v2" or report.get("derived_urdf_sha256") != sha(source):
        raise ValueError("Replay source is not the audited prepared X650 v2 URDF")
    if report.get("physical_geometry_validated") is not True or report.get("arm_self_collision_resolved") is not True:
        raise ValueError("Prepared model lacks completed rigid-geometry/self-collision checks")
    robot = ET.parse(source).getroot()
    resources = report.get("mesh_resources", {})
    referenced = {node.get("filename") for node in robot.findall(".//mesh")}
    if not resources or referenced != set(resources):
        raise ValueError("Prepared source mesh resource manifest is incomplete")
    for relative, digest in resources.items():
        resource = (source.parent/relative).resolve(strict=True)
        if Path(relative).is_absolute() or ".." in Path(relative).parts or not resource.is_relative_to(source.parent) or sha(resource) != digest:
            raise ValueError("Prepared visual mesh resource changed: "+relative)
    for name, field in (("clearance_report.json", "clearance_certificate_sha256"),
                        ("arm_self_collision_report.json", "arm_self_collision_report_sha256"),
                        ("kinematics.json", "kinematics_sha256")):
        file = source.with_name(name)
        if sha(file) != report.get(field):
            raise ValueError(f"Composition evidence checksum mismatch: {name}")
        evidence = json.loads(file.read_text(encoding="utf-8"))
        if name != "kinematics.json" and evidence.get("passed") is not True:
            raise ValueError(f"Composition evidence did not pass: {name}")
        if name == "kinematics.json" and not np.allclose(evidence["elbow_offset_body_m"], [0, .04, 0], rtol=0, atol=1e-14):
            raise ValueError("Kinematics evidence has a different lateral elbow offset")
    model = report["controller_model"]
    if not np.allclose(model["elbow_offset_body_m"], [0, .04, 0], rtol=0, atol=1e-14):
        raise ValueError("Replay requires the v2 lateral elbow offset")
    return robot, report


def tip_from_report(position, quaternion, joints, report):
    model = report["controller_model"]
    q1, q2 = joints
    a1, a2 = np.asarray(model["angle_offsets"])+[q1, q1+q2]
    local = (np.asarray(model["mount_body"])+np.asarray(model["elbow_offset_body_m"])
             +model["lengths"][0]*np.array([math.cos(a1), 0, -math.sin(a1)])
             +model["lengths"][1]*np.array([math.cos(a2), 0, -math.sin(a2)]))
    return position+quaternion_rotation(quaternion)@local


def world_link_poses(robot, position, quaternion, joints):
    local = kinematic_poses(robot, dict(zip(("arm_joint_1", "arm_joint_2"), joints)))
    world = matrix(position, quaternion_rotation(quaternion))
    return {name: world@pose for name, pose in local.items()}


def audit_forward_kinematics(data, robot, report, tolerance_m=1e-4):
    actual, geometry_error = [], []
    tip_local = np.array([report["controller_model"]["lengths"][1], 0., 0., 1.])
    for p, quat, q in zip(data["p"], data["q"], data["joint_q"]):
        tip = tip_from_report(p, quat, q, report)
        urdf_tip = (world_link_poses(robot, p, quat, q)["arm_link_2"]@tip_local)[:3]
        geometry_error.append(float(np.linalg.norm(tip-urdf_tip)))
        actual.append(tip)
    if max(geometry_error) > 1e-10:
        raise ValueError("Report FK disagrees with the actual URDF tree")
    data["tip"] = np.asarray(actual)
    record = {"samples": len(actual), "report_vs_urdf_maximum_error_m": max(geometry_error),
              "provided_tip_fields": data["provided_tip"] is not None,
              "tolerance_m": tolerance_m, "all_samples_audited": True}
    ages = data.get("joint_age_s")
    record["pose_joint_timing"] = ("separately timestamped ROS samples; FK is a state reconstruction, not independent same-step tip truth"
                                   if ages is not None else "timestamps provided by capture; no separate joint timestamp column")
    if ages is not None:
        record["maximum_absolute_joint_age_s"] = float(np.max(np.abs(ages)))
    if data["provided_tip"] is not None:
        error = np.linalg.norm(data["provided_tip"]-data["tip"], axis=1)
        record.update(recorded_vs_fk_maximum_error_m=float(error.max()),
                      recorded_vs_fk_rmse_m=float(np.sqrt(np.mean(error**2))),
                      worst_sample_index=int(error.argmax()))
        if error.max() > tolerance_m:
            raise ValueError(f"Logged tip disagrees with base/joint FK: maximum {error.max():.9g} m at sample {error.argmax()}")
    record["passed"] = True
    return record


def load_samples(path):
    with Path(path).open(newline="", encoding="utf-8-sig") as stream:
        reader = csv.DictReader(stream)
        names = set(reader.fieldnames or [])
        missing = set(REQUIRED)-names
        if missing:
            raise ValueError(f"Missing recorded-state CSV fields: {sorted(missing)}")
        rows = list(reader)
    if len(rows) < 2:
        raise ValueError("At least two recorded states are required")
    def array(fields):
        return np.asarray([[float(row[name]) for name in fields] for row in rows])
    def optional(fields):
        present = names & set(fields)
        if present and present != set(fields):
            raise ValueError(f"Incomplete optional vector fields: {sorted(set(fields)-names)}")
        if not present:
            return None
        # The ROS recorder leaves desired references empty before takeoff and
        # after landing. Only optional references accept blank -> NaN; actual
        # logged tip, base pose and joint measurements remain strict numbers.
        if fields == TARGET:
            return np.asarray([[float(row[name]) if row[name].strip() else np.nan for name in fields] for row in rows])
        return array(fields)
    t = array(["sim_time_s"]).ravel()
    p = array(["world_x_m", "world_y_m", "world_z_m"])
    quat = array(["world_qw", "world_qx", "world_qy", "world_qz"])
    joints = array(["q1_rad", "q2_rad"])
    provided_tip, target = optional(TIP), optional(TARGET)
    if not np.isfinite(np.column_stack([t, p, quat, joints])).all():
        raise ValueError("Actual timestamps, poses and measured joint states must be finite")
    if np.any(np.diff(t) <= 0):
        raise ValueError("Simulation timestamps must be strictly increasing")
    norm = np.linalg.norm(quat, axis=1)
    if np.any(np.abs(norm-1) > .01):
        raise ValueError("Recorded body-to-world orientation must be a unit quaternion")
    quat /= norm[:, None]
    if provided_tip is not None and not np.isfinite(provided_tip).all():
        raise ValueError("Provided actual tip must be finite in every sample")
    # Reference vectors may be absent during takeoff/landing, represented by NaN.
    if target is not None and (np.isinf(target).any() or np.any(np.isfinite(target).sum(axis=1)%3)):
        raise ValueError("Reference must contain complete finite XYZ vectors or three NaNs")
    phase = np.asarray([row["phase"] for row in rows])
    ages = array(["joint_age_s"]).ravel() if "joint_age_s" in names else None
    if ages is not None and not np.isfinite(ages).all():
        raise ValueError("Joint timestamp ages must be finite")
    return {"rows": rows, "t": t, "elapsed": t-t[0], "p": p, "q": quat, "joint_q": joints, "joint_age_s": ages,
            "phase": phase, "provided_tip": provided_tip, "target": target,
            "measure": np.isin(phase, ["measure", "track", "tracking", "figure8"]),
            "max_sample_gap_s": float(np.diff(t).max())}


def verify_recording(csv_path, source, data, manifest_path=None):
    manifest_path = Path(manifest_path) if manifest_path else Path(csv_path).with_name("flight_samples_manifest.json")
    metadata = json.loads(manifest_path.read_text(encoding="utf-8"))
    if metadata.get("complete") is not True:
        raise ValueError("A complete, finalized recording manifest is required")
    if metadata.get("file") != Path(csv_path).name or metadata.get("sha256") != sha(csv_path):
        raise ValueError("Recording manifest file/checksum mismatch")
    if metadata.get("source_urdf_sha256") != sha(source):
        raise ValueError("Recording manifest does not identify this prepared URDF")
    model_hash = metadata.get("model_sha256")
    row_hashes = {row.get("model_sha256") for row in data["rows"]}
    if not model_hash or (row_hashes != {None} and row_hashes != {model_hash}):
        raise ValueError("Recorded model identity differs from finalized capture")
    if metadata.get("synthetic") is True:
        raise ValueError("Synthetic captures require --synthetic-test and cannot produce experiment video")
    if any("SYNTHETIC" in str(phase).upper() for phase in data["phase"]):
        raise ValueError("Synthetic test phases cannot produce experiment video")
    return {"path": str(manifest_path.resolve()), "sha256": sha(manifest_path), "metadata": metadata,
            "model_identity_source": "finalized manifest" if row_hashes == {None} else "manifest and every CSV row"}


def sample_state(data, elapsed, gap_limit=.25):
    p, quat, index, gap = sample_pose(data, elapsed, gap_limit)
    if gap or index == len(data["elapsed"])-1:
        joints = data["joint_q"][index]
    else:
        dt = data["elapsed"][index+1]-data["elapsed"][index]
        fraction = float(np.clip((elapsed-data["elapsed"][index])/dt, 0, 1))
        joints = (1-fraction)*data["joint_q"][index]+fraction*data["joint_q"][index+1]
    return p, quat, joints, index, gap


def camera_targets(position, quaternion, joints, report):
    """Follow recorded body/arm state, independently of scene model extent."""
    model = report["controller_model"]
    rotate = quaternion_rotation(quaternion)
    a1 = model["angle_offsets"][0]+joints[0]
    mount = np.asarray(model["mount_body"])
    elbow = mount+model["lengths"][0]*np.array([math.cos(a1), 0., -math.sin(a1)])
    tip = rotate.T@(tip_from_report(position, quaternion, joints, report)-position)
    arm_center = np.mean([mount, elbow+np.asarray(model["elbow_offset_body_m"]), tip], axis=0)
    return position+rotate@np.array([0., 0., -.12]), position+rotate@arm_center


def joint_probe_context(data, csv_path, capture, metadata_path=None):
    from analyze_x650_joint_probe import reference
    path = Path(metadata_path) if metadata_path else Path(csv_path).with_name("controller_metadata.json")
    metadata = json.loads(path.read_text(encoding="utf-8"))
    if metadata.get("stage") != "joint_probe" or metadata.get("complete") is not True:
        raise ValueError("Joint-probe display requires completed joint_probe metadata")
    for key in ("model_sha256", "source_urdf_sha256"):
        if metadata.get(key) != capture["metadata"].get(key):
            raise ValueError("Joint-probe metadata has a different recorded model identity")
    start, duration = float(metadata["task_start_sim_s"]), float(metadata["requested_duration_s"])
    if not np.isfinite([start, duration]).all() or duration < 16.:
        raise ValueError("Joint-probe metadata has an invalid reference window")
    mask = (data["phase"] == "measure") & (data["t"] >= start-1e-9) & (data["t"] <= start+duration+1e-9)
    if not mask.any() or data["t"][mask][0] > start+.1 or data["t"][mask][-1] < start+duration-.1:
        raise ValueError("Recorded samples do not cover the joint-probe reference window")
    desired = np.full_like(data["joint_q"], np.nan)
    desired[mask] = reference(data["t"][mask]-start, duration)[0]
    anchor = np.asarray(metadata["reference_base_enu_m"], float)
    if anchor.shape != (3,) or not np.isfinite(anchor).all():
        raise ValueError("Joint-probe body anchor is invalid")
    body_error = np.full_like(data["p"], np.nan)
    body_error[mask] = data["p"][mask]-anchor
    q = data["q"]
    tilt = np.degrees(np.arccos(np.clip(1-2*(q[:, 1]**2+q[:, 2]**2), -1., 1.)))
    return dict(q_reference=desired, q_error=data["joint_q"]-desired, body_error=body_error,
                tilt_deg=tilt, reference_mask=mask,
                metadata={"path": str(path.resolve()), "sha256": sha(path), "task_start_sim_s": start,
                          "duration_s": duration, "reference_outside_measure": "absent, never invented",
                          "reference_source": "declared joint_probe sin^3 position law; actual actuator command is velocity",
                          "primary_control_metric": False, "measurement_source": "separately timestamped recorded ROS base/joint states"})


def make_scene(directory, source, robot, width=900, height=760):
    """Render-only MJCF: exact local visual geometry, one mocap per URDF link."""
    directory, source = Path(directory), Path(source).resolve(strict=True)
    directory.mkdir(parents=True, exist_ok=True)
    root = ET.Element("mujoco", model="x650_two_link_recorded_state_replay")
    ET.SubElement(root, "compiler", angle="radian", inertiafromgeom="false")
    # Clip distances are fractions of model extent. A hidden marker at z=-100
    # used to enlarge automatic extent and clip the complete near arm view.
    ET.SubElement(root, "statistic", extent="2", center="0 0 0")
    visual = ET.SubElement(root, "visual")
    ET.SubElement(visual, "global", offwidth=str(width), offheight=str(height))
    ET.SubElement(visual, "quality", shadowsize="2048", offsamples="4")
    ET.SubElement(visual, "map", znear=".002", zfar="30")
    ET.SubElement(visual, "headlight", ambient=".55 .55 .60", diffuse=".6 .6 .6", specular=".15 .15 .15")
    asset, world = ET.SubElement(root, "asset"), ET.SubElement(root, "worldbody")
    ET.SubElement(world, "light", pos="0 -3 8", dir="0 0 -1", diffuse=".7 .7 .7")
    ET.SubElement(world, "geom", name="ground", type="plane", pos="0 0 -.003", size="20 20 .1",
                  rgba=".10 .15 .21 1", contype="0", conaffinity="0")
    records, mesh_count, triangle_count, primitive_count = [], 0, 0, 0
    named_materials = {node.get("name"): node for node in robot.findall("material")}
    for link in robot.findall("link"):
        lname = link.get("name")
        body = ET.SubElement(world, "body", name="replay_"+lname, mocap="true")
        for index, node in enumerate(link.findall("visual")):
            name = f"visual_{lname}_{index:03d}"
            rgba = np.array([.12, .2, .29, 1.]) if lname == "base_link" else np.array([.10, .48, .73, 1.])
            material = node.find("material")
            if material is not None:
                if material.find("color") is None:
                    material = named_materials.get(material.get("name"), material)
                color = material.find("color")
                if color is not None:
                    rgba = np.array([float(v) for v in color.get("rgba").split()])
            rgba[:3] = np.clip(.20+.80*rgba[:3], 0, 1)
            xyz, orient = origin(node)
            attributes = {"name": name, "rgba": " ".join(f"{v:.9g}" for v in rgba),
                          "contype": "0", "conaffinity": "0", "group": "1"}
            mesh, box = node.find("geometry/mesh"), node.find("geometry/box")
            if mesh is not None:
                relative = Path(mesh.get("filename"))
                file = (source.parent/relative).resolve(strict=True)
                if relative.is_absolute() or ".." in relative.parts or not file.is_relative_to(source.parent):
                    raise ValueError("Visual resource escapes prepared source tree")
                triangles = load_stl(file)*np.asarray([float(v) for v in mesh.get("scale", "1 1 1").split()])
                triangles = triangles@orient.T+xyz
                path = directory/(name+".stl")
                write_binary_stl(path, triangles)
                ET.SubElement(asset, "mesh", name=name, file=str(path.resolve()), maxhullvert="64")
                ET.SubElement(body, "geom", type="mesh", mesh=name, **attributes)
                mesh_count += 1
                triangle_count += len(triangles)
                records.append({"link": lname, "visual": index, "mesh_sha256": sha(file),
                                "triangles": len(triangles), "baked_visual_to_link": matrix(xyz, orient).tolist()})
            elif box is not None:
                size = np.asarray([float(v) for v in box.get("size").split()])/2
                ET.SubElement(body, "geom", type="box", size=" ".join(map(str, size)), pos=" ".join(map(str, xyz)),
                              quat=" ".join(map(str, rotation_quaternion(orient))), **attributes)
                primitive_count += 1
            else:
                raise ValueError("Replay supports the audited STL meshes and arm boxes only")
    marker = ET.SubElement(world, "body", name="replay_tip_reference", mocap="true", pos="0 0 0")
    for axis in range(3):
        a, b = np.zeros(3), np.zeros(3)
        a[axis], b[axis] = -.045, .045
        ET.SubElement(marker, "geom", name=f"tip_reference_{axis}", type="capsule", fromto=" ".join(map(str, [*a, *b])), size=".003",
                      rgba="1 .75 .35 0", contype="0", conaffinity="0")
    path = directory/"replay_scene.xml"
    ET.ElementTree(root).write(path, encoding="utf-8", xml_declaration=True)
    return path, {"visual_mesh_count": mesh_count, "original_triangle_count": triangle_count,
                  "primitive_visual_count": primitive_count, "mocap_link_count": len(robot.findall("link")),
                  "visuals": records, "mesh_decimation": False, "unlogged_rotor_angles": "fixed at source q=0",
                  "new_dynamics_simulated": False, "statistic_extent_m": 2,
                  "near_clip_m": .004, "far_clip_m": 60., "reference_hidden_by_alpha": True}


def joint_figures(data, context, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(5, 1, figsize=(12, 12), sharex=True, layout="constrained")
    t = data["elapsed"]
    for j, color in enumerate((BLUE, RED)):
        axes[j].plot(t, np.degrees(context["q_reference"][:, j]), "--", color="#444444", label=f"J{j+1} reference")
        axes[j].plot(t, np.degrees(data["joint_q"][:, j]), color=color, label=f"J{j+1} recorded actual")
        axes[j].set_ylabel(f"J{j+1} angle (deg)")
        axes[2].plot(t, np.degrees(context["q_error"][:, j]), color=color, label=f"J{j+1} error")
    for j, color in enumerate((BLUE, GREEN, RED)):
        axes[3].plot(t, context["body_error"][:, j]*100, color=color, label=f"{'XYZ'[j]} to fixed anchor")
    axes[4].plot(t, context["tilt_deg"], color=BLUE, label="Body tilt")
    for axis, label in zip(axes[2:], ("Joint error (deg)", "Body displacement (cm)", "Tilt (deg)")):
        axis.set_ylabel(label)
    axes[-1].set_xlabel("Elapsed recorded simulation time (s)")
    for axis in axes:
        axis.grid(alpha=.2)
        axis.legend(ncol=3, fontsize=8)
    fig.suptitle("Joint-probe replay diagnostics | recorded ROS states\nReference exists only during measure; independent native physics audit supplies primary metrics")
    fig.savefig(output/"trajectory_and_errors.png", dpi=160)
    fig.savefig(output/"trajectory_and_errors.pdf")
    plt.close(fig)
    return dict(primary_control_metric=False, source="Recorded ROS base/joint states, potentially asynchronous",
                display_mode="joint_probe", reference_source=context["metadata"])


def figures(data, output):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t, target = data["elapsed"], data["target"]
    fig, axes = plt.subplots(3, 1, figsize=(12, 10), sharex=True, layout="constrained")
    for axis, color in zip(range(3), [BLUE, GOLD, GREEN]):
        axes[0].plot(t, data["tip"][:, axis], color=color, label=f"actual {'XYZ'[axis]}")
        if target is not None:
            axes[0].plot(t, target[:, axis], color=color, ls="--", label=f"reference {'XYZ'[axis]}")
            axes[1].plot(t, (data["tip"][:, axis]-target[:, axis])*1000, color=color, label=f"{'XYZ'[axis]} error")
    for axis, color in zip(range(2), [BLUE, RED]):
        axes[2].plot(t, np.degrees(data["joint_q"][:, axis]), color=color, label=f"q{axis+1}")
    axes[0].set_ylabel("World endpoint position (m)")
    axes[1].set_ylabel("Endpoint error (mm)")
    axes[2].set_ylabel("Measured joint angle (deg)"); axes[2].set_xlabel("Elapsed simulation time (s)")
    for axis in axes:
        axis.grid(alpha=.2)
        handles, labels = axis.get_legend_handles_labels()
        if handles: axis.legend(ncol=3, fontsize=9)
    fig.suptitle("X650 + original two-link arm | recorded Gazebo states, FK audited")
    fig.savefig(output/"trajectory_and_errors.png", dpi=160)
    fig.savefig(output/"trajectory_and_errors.pdf")
    plt.close(fig)
    metrics = {"samples": len(t), "duration_s": float(t[-1]), "new_dynamics_simulated": False,
               "endpoint_source": "FK of recorded base pose and measured joints; not independent native tip measurement",
               "primary_control_metric": False}
    if target is not None:
        mask = np.isfinite(target).all(1)&data["measure"]
        metrics["tracking_samples"] = int(mask.sum())
        if mask.any():
            error = data["tip"][mask]-target[mask]
            metrics.update(endpoint_rmse_m=float(np.sqrt(np.mean(np.sum(error**2, axis=1)))),
                           endpoint_maximum_error_m=float(np.linalg.norm(error, axis=1).max()),
                           axis_rmse_m=np.sqrt(np.mean(error**2, axis=0)).tolist())
    return metrics


def render(args):
    source, csv_path, output = args.source.resolve(strict=True), args.csv.resolve(strict=True), args.output.resolve()
    if args.synthetic_test and not (args.preview_only or args.audit_only):
        raise ValueError("Synthetic data may only be audited or previewed, never exported as experiment video")
    if output.exists() and any((output/name).exists() for name in ("replay_manifest.json", "x650_arm.mp4", "preview.png")) and not args.overwrite:
        raise ValueError("Replay output exists; use a new directory")
    robot, report = load_composition(source)
    data = load_samples(csv_path)
    capture = ({"synthetic_test": True, "experiment_result": False} if args.synthetic_test
               else verify_recording(csv_path, source, data, args.capture_manifest))
    if args.joint_probe and args.synthetic_test:
        raise ValueError("Joint-probe mode requires real completed recording metadata")
    joint_context = joint_probe_context(data, csv_path, capture, args.controller_metadata) if args.joint_probe else None
    fk_audit = audit_forward_kinematics(data, robot, report, args.tip_tolerance_m)
    output.mkdir(parents=True, exist_ok=True)
    record = {"kind": "gazebo_recorded_x650_arm_replay", "synthetic_test": args.synthetic_test,
              "experiment_result": not args.synthetic_test, "native_gazebo_screen_capture": False,
              "new_dynamics_simulated": False, "csv_sha256": sha(csv_path), "source_urdf_sha256": sha(source),
              "generator_sha256": sha(__file__), "capture": capture, "fk_audit": fk_audit,
              "max_sample_gap_s": data["max_sample_gap_s"], "video_time_scale": 1,
              "base_interpolation": "piecewise linear translation + quaternion SLERP; no smoothing",
              "joint_interpolation": "piecewise linear measured joint angles; no smoothing",
              "gap_policy": "gaps >0.25s hold previous base pose and both joints", "rotor_angles": "unlogged: never animated"}
    record["display_mode"] = "joint_probe" if joint_context is not None else "endpoint"
    if joint_context is not None: record["joint_reference"] = joint_context["metadata"]
    (output/"fk_audit.json").write_text(json.dumps(fk_audit, indent=2)+"\n", encoding="utf-8")
    if args.audit_only:
        record["audit_only"] = True
        (output/"replay_manifest.json").write_text(json.dumps(record, indent=2)+"\n", encoding="utf-8")
        return record
    from PIL import Image, ImageDraw
    import mujoco
    scene, audit = make_scene(output/"render_geometry", source, robot)
    record["geometry"] = audit
    model = mujoco.MjModel.from_xml_path(str(scene))
    metrics = joint_figures(data, joint_context, output) if joint_context is not None else figures(data, output)
    state, renderer = mujoco.MjData(model), mujoco.Renderer(model, height=760, width=900)
    mocaps = {name: model.body_mocapid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "replay_"+name)]
              for name in [*kinematic_poses(robot), "tip_reference"]}
    cam = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(cam)
    cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    cam.distance, cam.azimuth, cam.elevation = 2.3, 115, -14
    close = mujoco.MjvCamera(); mujoco.mjv_defaultCamera(close)
    close.type = mujoco.mjtCamera.mjCAMERA_FREE
    close.distance, close.azimuth, close.elevation = .80, 90, -8
    marker_geoms = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"tip_reference_{axis}") for axis in range(3)]
    record["camera"] = {"main": "recorded body-following; distance 2.3m", "detail": "actual shoulder/elbow/tip centre; distance 0.80m",
                        "main_azimuth_deg": 115, "detail_azimuth_deg": 90, "near_clip_m": .004,
                        "mj_forward_before_update_scene": True, "body_pose_used_for_both_views": True}
    times = np.arange(0, data["elapsed"][-1]+1e-9, 1/args.fps)
    if args.preview_only:
        if args.preview_time_s is not None:
            if not 0 <= args.preview_time_s <= data["elapsed"][-1]:
                raise ValueError("Preview time is outside the recorded interval")
            preview_time = args.preview_time_s
        elif joint_context is not None:
            values = np.where(joint_context["reference_mask"], np.max(abs(data["joint_q"]), axis=1), -1.)
            preview_time = float(data["elapsed"][int(np.argmax(values))])
        else: preview_time = float(data["elapsed"][-1]/2)
        times = np.array([preview_time])
    video, proc, log = output/"x650_arm.mp4", None, None
    if not args.preview_only:
        ffmpeg = args.ffmpeg or shutil.which("ffmpeg")
        if not ffmpeg:
            bins = Path("/root/autodl-tmp/flyarm-video-venv/lib/python3.10/site-packages/imageio_ffmpeg/binaries")
            ffmpeg = next((str(p) for p in bins.glob("ffmpeg-*")), None)
        if not ffmpeg: raise ValueError("An existing ffmpeg executable is required")
        log = (output/"ffmpeg.log").open("w")
        proc = subprocess.Popen([ffmpeg, "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "1440x900",
                                 "-r", str(args.fps), "-i", "-", "-an", "-vcodec", "libx264", "-preset", "fast",
                                 "-crf", "20", "-pix_fmt", "yuv420p", "-movflags", "+faststart", str(video)],
                                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)
    target = data["target"]
    error_norm = np.full(len(data["t"]), np.nan) if target is None else np.linalg.norm(data["tip"]-target, axis=1)*1000
    snapshots = set(np.linspace(0, len(times)-1, 5).astype(int))
    try:
        for k, now in enumerate(times):
            p, quat, joints, index, gap = sample_state(data, now)
            for name, pose in world_link_poses(robot, p, quat, joints).items():
                state.mocap_pos[mocaps[name]] = pose[:3, 3]
                state.mocap_quat[mocaps[name]] = rotation_quaternion(pose[:3, :3])
            marker_visible = target is not None and np.isfinite(target[index]).all() and joint_context is None
            state.mocap_pos[mocaps["tip_reference"]] = target[index] if marker_visible else p
            model.geom_rgba[marker_geoms, 3] = .75 if marker_visible else 0.
            mujoco.mj_forward(model, state)
            cam.lookat[:], close.lookat[:] = camera_targets(p, quat, joints, report)
            renderer.update_scene(state, camera=cam)
            canvas = Image.new("RGB", (1440, 900), BG)
            # Keep the full-width central camera image above the arm inset.
            # Crop symmetrically, never stretch the aircraft aspect ratio.
            canvas.paste(Image.fromarray(renderer.render().copy()).crop((0, 130, 900, 630)), (0, 80))
            renderer.update_scene(state, camera=close)
            canvas.paste(Image.fromarray(renderer.render().copy()).resize((410, 346)), (24, 484))
            draw = ImageDraw.Draw(canvas)
            draw.rectangle((0, 0, 1440, 80), fill=BG)
            draw.text((24, 13), "X650 + TWO-LINK ARM  |  RECORDED STATE", font=find_font(27, True), fill=FG)
            draw.text((24, 51), "Gazebo SITL  /  3.495 kg  /  +Y 40 mm elbow offset  /  base + measured joint replay", font=find_font(16), fill=MUTED)
            draw.rounded_rectangle((24, 100, 365, 184), radius=10, fill=PANEL)
            draw.text((40, 111), f"{data['phase'][index].upper()}    {now:5.2f} s  |  1x", font=find_font(18, True), fill=GREEN)
            draw.text((40, 145), f"q1 {math.degrees(joints[0]):.1f} deg   q2 {math.degrees(joints[1]):.1f} deg", font=find_font(16), fill=FG)
            draw.rectangle((24, 484, 434, 514), fill=PANEL)
            draw.text((36, 491), "Same recorded state | shoulder + elbow detail", font=find_font(14), fill=FG)
            if joint_context is None:
                draw_chart(draw, (914, 92, 1424, 322), "ENDPOINT WORLD X / Z  |  m", data["elapsed"],
                           [data["tip"][:, 0], data["tip"][:, 2]], [BLUE, GOLD], ["X actual", "Z actual"], now)
                draw_chart(draw, (914, 338, 1424, 568), "ENDPOINT TRACKING ERROR  |  mm", data["elapsed"],
                           [error_norm], [GREEN], ["3D norm"], now, ymin=0)
                draw_chart(draw, (914, 584, 1424, 814), "MEASURED JOINT ANGLES  |  deg", data["elapsed"],
                           [np.degrees(data["joint_q"][:, 0]), np.degrees(data["joint_q"][:, 1])], [BLUE, RED], ["q1", "q2"], now)
            else:
                refs = np.degrees(joint_context["q_reference"])
                draw_chart(draw, (914, 92, 1424, 322), "JOINT ACTUAL / REFERENCE | deg", data["elapsed"],
                           [refs[:, 0], refs[:, 1], np.degrees(data["joint_q"][:, 0]), np.degrees(data["joint_q"][:, 1])],
                           [GOLD, GREEN, BLUE, RED], ["J1 ref", "J2 ref", "J1 actual", "J2 actual"], now)
                draw_chart(draw, (914, 338, 1424, 568), "JOINT TRACKING ERROR | deg", data["elapsed"],
                           [np.degrees(joint_context["q_error"][:, j]) for j in range(2)], [BLUE, RED], ["J1 error", "J2 error"], now)
                draw_chart(draw, (914, 584, 1424, 814), "BODY TO FIXED HOVER ANCHOR | cm", data["elapsed"],
                           [joint_context["body_error"][:, j]*100 for j in range(3)], [BLUE, GREEN, RED], ["X", "Y", "Z"], now)
                draw.rounded_rectangle((24, 191, 365, 232), radius=8, fill=PANEL)
                draw.text((36, 202), f"Body tilt {joint_context['tilt_deg'][index]:.3f} deg | joint velocity actuator", font=find_font(13), fill=FG)
            draw.rectangle((0, 840, 1440, 900), fill=BG)
            draw.text((24, 849), "GAZEBO RECORDED-STATE REPLAY  |  all original visual meshes  |  sample-by-sample FK verified", font=find_font(16, True), fill=FG)
            footer = ("Joint probe only; references exist in measure phase. Primary metrics use independent native physics; this view uses recorded ROS states."
                      if joint_context is not None else "Renderer only; no new dynamics. Unlogged rotor angles stay fixed. Gold marker follows recorded endpoint reference.")
            draw.text((24, 875), footer, font=find_font(13), fill=MUTED)
            if gap or args.synthetic_test:
                draw.rounded_rectangle((390, 104, 890, 151), radius=8, fill="#79392c")
                text = "SYNTHETIC CODE TEST - NOT AN EXPERIMENT" if args.synthetic_test else "TELEMETRY GAP: holding previous state"
                draw.text((400, 118), text, font=find_font(16, True), fill=FG)
            if k in snapshots: canvas.save(output/f"frame_{k:05d}.png")
            if args.preview_only: canvas.save(output/"preview.png")
            else: proc.stdin.write(canvas.tobytes())
            if k%50 == 0: print(f"RENDER {k}/{len(times)} elapsed={now:.2f}s", flush=True)
    finally:
        renderer.close()
        if proc:
            proc.stdin.close()
            status = proc.wait()
            log.close()
            if status: raise RuntimeError(f"ffmpeg failed with code {status}")
    if not args.preview_only:
        checked = subprocess.run([ffmpeg, "-v", "error", "-i", str(video), "-f", "null", "-"], capture_output=True, text=True)
        if checked.returncode: raise RuntimeError("Encoded video failed complete ffmpeg decode: "+checked.stderr[-2000:])
        record.update(video=str(video), video_sha256=sha(video), complete_video_decode_passed=True)
    record.update(metrics=metrics, preview_only=args.preview_only, frames=len(times), fps=args.fps,
                  recorded_duration_s=float(data["elapsed"][-1]), encoded_duration_s=None if args.preview_only else len(times)/args.fps)
    (output/"replay_manifest.json").write_text(json.dumps(record, indent=2)+"\n", encoding="utf-8")
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--capture-manifest", type=Path)
    parser.add_argument("--joint-probe", action="store_true", help="Display actual joint references/errors and body stability, not endpoint tracking")
    parser.add_argument("--controller-metadata", type=Path)
    parser.add_argument("--preview-time-s", type=float, help="Elapsed recorded time of preview; default joint mode picks actual maximum excursion")
    parser.add_argument("--fps", type=int, default=24)
    parser.add_argument("--tip-tolerance-m", type=float, default=1e-4)
    parser.add_argument("--ffmpeg")
    parser.add_argument("--preview-only", action="store_true")
    parser.add_argument("--audit-only", action="store_true")
    parser.add_argument("--synthetic-test", action="store_true")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args()
    if not 1 <= args.fps <= 60: parser.error("fps must be in [1,60]")
    if not math.isfinite(args.tip_tolerance_m) or not 0 < args.tip_tolerance_m <= .01: parser.error("tip tolerance must be in (0,0.01] m")
    print(json.dumps(render(args), indent=2), flush=True)


if __name__ == "__main__":
    main()
