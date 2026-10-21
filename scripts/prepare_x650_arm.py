#!/usr/bin/env python3
"""Compose the audited X650 with the original two-link arm, without starting simulation.

The X650 input must be the output of prepare_x650_flight.py.  The arm is read
from the previously flown x500_underfold SDF, including inertias, limits and
zero offsets.  The mounting adapter is an explicit, unmeasured 25 g model.
No source CAD, old model, or existing output directory is modified.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
import itertools
import json
import math
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT / "src"))
from amsim.modeling.urdf_importer import import_urdf
from amsim.modeling.urdf_flight import bind_flight_profile
from prepare_x650_flight import origin, rotation, set_origin, properties, set_properties, parallel_axis
from render_urdf_geometry import load_stl

MODEL_NAME = "x650_2link_v2"
MOUNT = np.array([0., 0., -.18])
ELBOW_OFFSET = np.array([0., .04, 0.])
ADAPTER_MASS = .025
PLATE_BOTTOM = -.1492
ARM_NAMES = ("arm_link_1", "arm_link_2")
JOINT_NAMES = ("arm_joint_1", "arm_joint_2")


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def fmt(values):
    return " ".join(f"{float(value):.17g}" for value in values)


def matrix(xyz, orient):
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = orient, xyz
    return result


def kinematic_poses(robot, q=None):
    q = q or {}
    poses = {"base_link": np.eye(4)}
    pending = list(robot.findall("joint"))
    while pending:
        progress = False
        for joint in pending[:]:
            parent = joint.find("parent").get("link")
            child = joint.find("child").get("link")
            if parent not in poses:
                continue
            pose = matrix(*origin(joint))
            angle = float(q.get(joint.get("name"), 0.))
            if angle:
                axis = np.asarray([float(v) for v in joint.find("axis").get("xyz").split()])
                axis /= np.linalg.norm(axis)
                cross = np.array([[0., -axis[2], axis[1]], [axis[2], 0., -axis[0]], [-axis[1], axis[0], 0.]])
                turn = np.eye(4)
                turn[:3, :3] = np.eye(3) + math.sin(angle)*cross + (1-math.cos(angle))*(cross@cross)
                pose = pose @ turn
            poses[child] = poses[parent] @ pose
            pending.remove(joint)
            progress = True
        if not progress:
            raise ValueError("Unresolved or cyclic URDF tree")
    return poses


def aggregate(robot, q=None):
    poses = kinematic_poses(robot, q)
    values = []
    for link in robot.findall("link"):
        pose = poses[link.get("name")]
        values.append(properties(link, pose[:3, 3], pose[:3, :3]))
    mass = sum(value[0] for value in values)
    com = sum(m*c for m, c, _ in values)/mass
    inertia = sum(i + m*parallel_axis(c-com) for m, c, i in values)
    return mass, com, inertia


def original_arm(path):
    """Read the original flight asset, not an editable/example approximation."""
    model = ET.parse(path).getroot().find("model")
    specs = []
    old_mount = None
    previous_angle = 0.
    for lname, jname in zip(ARM_NAMES, JOINT_NAMES):
        link = model.find(f"link[@name='{lname}']")
        joint = model.find(f"joint[@name='{jname}']")
        if link is None or joint is None:
            raise ValueError("Original two-link flight asset is incomplete")
        pose = np.asarray([float(v) for v in link.findtext("pose").split()])
        if old_mount is None:
            old_mount = pose[:3]
        if abs(pose[3])+abs(pose[5]) > 1e-10:
            raise ValueError("Original arm is no longer planar about +Y")
        axis = np.asarray([float(v) for v in joint.findtext("axis/xyz").split()])
        np.testing.assert_allclose(axis, [0, 1, 0], atol=1e-12)
        size = np.asarray([float(v) for v in link.findtext("collision/geometry/box/size").split()])
        vsize = np.asarray([float(v) for v in link.findtext("visual/geometry/box/size").split()])
        np.testing.assert_allclose(size, vsize, atol=1e-12)
        mass = float(link.findtext("inertial/mass"))
        ipose = np.asarray([float(v) for v in link.findtext("inertial/pose").split()])
        np.testing.assert_allclose(ipose, [size[0]/2, 0, 0, 0, 0, 0], atol=1e-12)
        values = {key: float(link.findtext("inertial/inertia/"+key)) for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
        inertia = np.array([[values["ixx"], values["ixy"], values["ixz"]],
                            [values["ixy"], values["iyy"], values["iyz"]],
                            [values["ixz"], values["iyz"], values["izz"]]])
        specs.append({"link": lname, "joint": jname, "size": size, "mass": mass, "inertia": inertia,
                      "initial_angle": pose[4]-previous_angle,
                      "q_limits": [float(joint.findtext("axis/limit/"+key)) for key in ("lower", "upper")],
                      "effort": float(joint.findtext("axis/limit/effort")),
                      "velocity": float(joint.findtext("axis/limit/velocity")),
                      "damping": float(joint.findtext("axis/dynamics/damping", "0")),
                      "friction": float(joint.findtext("axis/dynamics/friction", "0")),
                      "rgba": link.findtext("visual/material/diffuse")})
        previous_angle = pose[4]
    # Canonical 150deg offset avoids the SDF's 2.99e-12rad decimal truncation.
    # Both offsets are preserved to 1e-10rad; all other physical values copied.
    if abs(specs[0]["initial_angle"]-math.radians(150)) > 1e-10 or abs(specs[1]["initial_angle"]+math.radians(150)) > 1e-10:
        raise ValueError("Original arm's folded convention changed; explicit review needed")
    specs[0]["initial_angle"], specs[1]["initial_angle"] = math.radians(150), -math.radians(150)
    return specs, old_mount


def box_nodes(link, name, size, xyz, rgba):
    for tag in ("visual", "collision"):
        node = ET.SubElement(link, tag, name=f"{name}_{tag}")
        set_origin(node, xyz)
        ET.SubElement(ET.SubElement(node, "geometry"), "box", size=fmt(size))
        if tag == "visual":
            ET.SubElement(ET.SubElement(node, "material", name=name+"_material"), "color", rgba=rgba)


def add_arm(robot, specs):
    base = robot.find("link[@name='base_link']")
    mass, com, inertia = properties(base)
    height = PLATE_BOTTOM-MOUNT[2]
    adapter_size = np.array([.025, .025, height])
    adapter_com = np.array([0., 0., (PLATE_BOTTOM+MOUNT[2])/2])
    adapter_inertia = ADAPTER_MASS/12*np.diag([adapter_size[1]**2+height**2,
                                             adapter_size[0]**2+height**2,
                                             adapter_size[0]**2+adapter_size[1]**2])
    new_mass = mass+ADAPTER_MASS
    new_com = (mass*com+ADAPTER_MASS*adapter_com)/new_mass
    new_inertia = inertia+mass*parallel_axis(com-new_com)+adapter_inertia+ADAPTER_MASS*parallel_axis(adapter_com-new_com)
    set_properties(base, new_mass, new_com, new_inertia)
    box_nodes(base, "arm_mount_adapter", adapter_size, adapter_com, ".42 .42 .42 1")
    previous_length = 0.
    for index, spec in enumerate(specs):
        link = ET.SubElement(robot, "link", name=spec["link"])
        set_properties(link, spec["mass"], np.array([spec["size"][0]/2, 0., 0.]), spec["inertia"])
        box_nodes(link, spec["link"], spec["size"], np.array([spec["size"][0]/2, 0., 0.]), spec["rgba"])
        joint = ET.SubElement(robot, "joint", name=spec["joint"], type="revolute")
        ET.SubElement(joint, "parent", link="base_link" if index == 0 else specs[index-1]["link"])
        ET.SubElement(joint, "child", link=spec["link"])
        set_origin(joint, MOUNT if index == 0 else np.array([previous_length, 0., 0.])+ELBOW_OFFSET,
                   rotation([0., spec["initial_angle"], 0.]))
        ET.SubElement(joint, "axis", xyz="0 1 0")
        ET.SubElement(joint, "limit", lower=f"{spec['q_limits'][0]:.17g}", upper=f"{spec['q_limits'][1]:.17g}",
                      effort=f"{spec['effort']:.17g}", velocity=f"{spec['velocity']:.17g}")
        ET.SubElement(joint, "dynamics", damping=f"{spec['damping']:.17g}", friction=f"{spec['friction']:.17g}")
        previous_length = spec["size"][0]
    return {"mass_kg": ADAPTER_MASS, "size_m": adapter_size.tolist(), "com_flu_m": adapter_com.tolist(),
            "inertia_at_com_kg_m2": adapter_inertia.tolist(),
            "source": "25 g mass inherited from original root_mount_adapter; resized to X650 bottom plate gap, unmeasured assumption",
            "intentional_contacts": ["top face touches X650 bottom plate at z=-0.1492 m", "proximal arm_joint_1 hinge interface"]}


def collision_triangles(robot, source):
    poses = kinematic_poses(robot)
    triangles, records = [], []
    for link in robot.findall("link"):
        for index, collision in enumerate(link.findall("collision")):
            mesh = collision.find("geometry/mesh")
            if mesh is None:
                raise ValueError("Audited X650 input collision geometry must remain STL meshes")
            relative = Path(mesh.get("filename"))
            if relative.is_absolute() or ".." in relative.parts:
                raise ValueError("Mesh paths must be self-contained relative resources")
            file = (source.parent/relative).resolve(strict=True)
            if not file.is_relative_to(source.parent):
                raise ValueError("Mesh path escapes source directory")
            values = load_stl(file)*np.asarray([float(v) for v in mesh.get("scale", "1 1 1").split()])
            pose = poses[link.get("name")] @ matrix(*origin(collision))
            values = values @ pose[:3, :3].T+pose[:3, 3]
            triangles.append(values)
            records.append({"link": link.get("name"), "collision_index": index, "mesh": relative.as_posix(),
                            "sha256": sha(file), "triangle_count": len(values)})
    return np.concatenate(triangles), records


def arm_corners(specs, q):
    result = []
    xyz = MOUNT.copy()
    angle = 0.
    for index, (spec, qi) in enumerate(zip(specs, q)):
        if index == 1:
            xyz += ELBOW_OFFSET
        angle += spec["initial_angle"]+qi
        r = rotation([0, angle, 0])
        length, width, height = spec["size"]
        local = np.asarray(list(itertools.product([0., length], [-width/2, width/2], [-height/2, height/2])))
        result.append(local @ r.T+xyz)
        xyz = xyz+r@np.array([length, 0., 0.])
    return np.stack(result)


def _cross2(a, b):
    return a[0]*b[1]-a[1]*b[0]


def elbow_intersection(specs, q2):
    """Exact convex polygon intersection in the link-1 XZ plane at the elbow."""
    l1, _, h1 = specs[0]["size"]
    l2, _, h2 = specs[1]["size"]
    first = np.array([[-l1, -h1/2], [0., -h1/2], [0., h1/2], [-l1, h1/2]])
    second = np.array([[0., -h2/2], [l2, -h2/2], [l2, h2/2], [0., h2/2]])
    angle = specs[1]["initial_angle"]+q2
    r = np.array([[math.cos(angle), math.sin(angle)], [-math.sin(angle), math.cos(angle)]])
    polygon = second@r.T
    for a, b in zip(first, np.roll(first, -1, axis=0)):
        clipped = []
        if not len(polygon):
            break
        for x, y in zip(polygon, np.roll(polygon, -1, axis=0)):
            dx, dy = _cross2(b-a, x-a), _cross2(b-a, y-a)
            if dx >= -1e-14:
                clipped.append(x)
            if (dx < 0 < dy) or (dy < 0 < dx):
                clipped.append(x+(y-x)*dx/(dx-dy))
        polygon = np.asarray(clipped)
    if not len(polygon):
        return {"maximum_overlap_radius_from_elbow_m": 0., "overlap_xz_area_m2": 0., "vertices_xz_m": []}
    area = abs(sum(_cross2(a, b) for a, b in zip(polygon, np.roll(polygon, -1, axis=0))))/2
    return {"maximum_overlap_radius_from_elbow_m": float(np.linalg.norm(polygon, axis=1).max()),
            "overlap_xz_area_m2": area, "vertices_xz_m": polygon.tolist()}


def self_collision_audit(specs):
    samples = []
    for q2 in [specs[1]["q_limits"][0], 0., specs[1]["q_limits"][1]]:
        overlap = elbow_intersection(specs, q2)
        samples.append({"gazebo_q2_rad": q2, "physical_relative_angle_deg": math.degrees(specs[1]["initial_angle"]+q2), **overlap})
    thresholds = []
    for radius in (.03, .04, .08):
        lo, hi = specs[1]["q_limits"]
        for _ in range(60):
            middle = (lo+hi)/2
            if elbow_intersection(specs, middle)["maximum_overlap_radius_from_elbow_m"] > radius:
                lo = middle
            else:
                hi = middle
        thresholds.append({"assumed_hinge_exclusion_radius_m": radius, "diagnostic_q2_lower_rad": hi,
                           "physical_relative_angle_deg": math.degrees(specs[1]["initial_angle"]+hi),
                           "q0_inside_exclusion_radius": samples[1]["maximum_overlap_radius_from_elbow_m"] <= radius,
                           "usable_without_confirmed_hinge_geometry": False})
    minimum_offset = float((specs[0]["size"][1]+specs[1]["size"][1])/2)
    original = {"kind": "original_coplanar_two_link_self_intersection", "passed": False,
            "method": "Exact 2D convex polygon clipping of solid rectangular links; Y intervals overlap in the original model",
            "samples": samples, "q0_has_solid_overlap_beyond_30mm_hinge_neighborhood": samples[1]["maximum_overlap_radius_from_elbow_m"] > .03,
            "hinge_exclusion_diagnostics": thresholds,
            "minimum_lateral_link2_offset_for_disjoint_y_intervals_m": minimum_offset,
            "candidate_lateral_offset_m": .04, "candidate_y_surface_clearance_m": .04-minimum_offset,
            "candidate_changes": "L2 joint frame shifted +body-Y by 40 mm; lengths, masses, local inertias, angles, and joint axes unchanged; endpoint acquires 40 mm Y offset. Requires explicit composition choice and matching controller model.",
            "conclusion": "Original solid box links self-intersect over substantial length at the folded pose and near their lower q2 limit. Adjacent-link exclusion does not establish physical feasibility.",
            "source_model_modified": False, "current_composed_model_self_collision_resolved": False}
    y1 = [MOUNT[1]-specs[0]["size"][1]/2, MOUNT[1]+specs[0]["size"][1]/2]
    y2 = [MOUNT[1]+ELBOW_OFFSET[1]-specs[1]["size"][1]/2, MOUNT[1]+ELBOW_OFFSET[1]+specs[1]["size"][1]/2]
    gap = float(y2[0]-y1[1])
    return {"kind": "laterally_separated_original_two_link_arm", "passed": gap > 0,
            "method": "Disjoint Y slabs for the complete solid link boxes, invariant under every +Y joint rotation",
            "elbow_offset_body_m": ELBOW_OFFSET.tolist(), "link_1_y_interval_m": list(map(float, y1)),
            "link_2_y_interval_m": list(map(float, y2)), "certified_surface_clearance_m": gap,
            "covers_continuous_entire_physical_joint_ranges": True,
            "dimensions_masses_local_inertias_joint_axes_and_angles_preserved": True,
            "idealized_joint_axis_connection": "The elbow axis runs along +Y through both link ends; axle, bearings and servo housing are not separately modeled or assigned extra mass.",
            "original_coplanar_audit": original,
            "source_asset_modified": False, "current_composed_model_self_collision_resolved": gap > 0}


def triangle_y_slab_floor(triangles, lower, upper):
    """Exact minimum Z of triangle/slab intersections, including cut edges."""
    selected = triangles[(triangles[:, :, 1].min(1) <= upper) & (triangles[:, :, 1].max(1) >= lower)]
    if not len(selected):
        raise ValueError("No CAD triangles in arm Y slab")
    inside = (selected[:, :, 1] >= lower) & (selected[:, :, 1] <= upper)
    candidates = [selected[:, :, 2][inside]]
    for boundary in (lower, upper):
        for ia, ib in ((0, 1), (1, 2), (2, 0)):
            a, b = selected[:, ia, :], selected[:, ib, :]
            crossing = ((a[:, 1]-boundary)*(b[:, 1]-boundary) < 0)
            a, b = a[crossing], b[crossing]
            candidates.append(a[:, 2]+(b[:, 2]-a[:, 2])*(boundary-a[:, 1])/(b[:, 1]-a[:, 1]))
    values = np.concatenate(candidates)
    if not len(values):
        raise ValueError("Triangle/slab clipping found no vertices")
    return float(values.min()), len(selected)


def clearance_audit(triangles, specs, grid_step_deg=.5):
    """Certify the continuous limit rectangle using a separating Z plane.

    Each link stays in a constant Y strip. Original CAD triangles are clipped
    exactly to each strip, retaining inside vertices and edge intersections.
    Joint-grid corner extrema are enlarged by analytic angular displacement
    bounds for the entire nearest-grid cell, covering every continuous q.
    This is stronger than checking only q=0 or sampled collision points.
    """
    if grid_step_deg <= 0 or grid_step_deg > 2:
        raise ValueError("Collision certificate grid step must be in (0, 2] degrees")
    y_centers = [MOUNT[1], MOUNT[1]+ELBOW_OFFSET[1]]
    y_intervals = [[float(center-spec["size"][1]/2), float(center+spec["size"][1]/2)] for center, spec in zip(y_centers, specs)]
    floors_and_counts = [triangle_y_slab_floor(triangles, *interval) for interval in y_intervals]
    body_floors = np.asarray([value[0] for value in floors_and_counts])
    if abs(body_floors[0]-PLATE_BOTTOM) > 1e-8:
        raise ValueError(f"X650 center-strip floor changed: {body_floors[0]}; mounting review required")
    grids = []
    half_steps = []
    for spec in specs:
        lo, hi = spec["q_limits"]
        count = int(math.ceil((hi-lo)/math.radians(grid_step_deg)))
        grid = np.linspace(lo, hi, count+1)
        grids.append(grid)
        half_steps.append((hi-lo)/(2*count))
    # Radius from each proximal joint to any point of the rectangular link.
    radii = [float(np.linalg.norm([spec["size"][0], spec["size"][2]/2])) for spec in specs]
    l1 = specs[0]["size"][0]
    # |d(point)/dq1| <= L1 + R2 for link2; R1 for link1.
    displacements = [radii[0]*half_steps[0], (l1+radii[1])*half_steps[0]+radii[1]*half_steps[1]]
    upper = np.full(2, -np.inf)
    lower = np.full(2, np.inf)
    min_q, max_q = [None]*2, [None]*2
    for q1 in grids[0]:
        for q2 in grids[1]:
            corners = arm_corners(specs, [q1, q2])
            lo, hi = corners[:, :, 2].min(1), corners[:, :, 2].max(1)
            for index in range(2):
                if hi[index] > upper[index]:
                    upper[index], max_q[index] = hi[index], [q1, q2]
                if lo[index] < lower[index]:
                    lower[index], min_q[index] = lo[index], [q1, q2]
    certified_upper = upper+displacements
    certified_lower = lower-displacements
    per_link_gap = body_floors-certified_upper
    gap = float(min(per_link_gap))
    zero_corners = arm_corners(specs, [0., 0.])
    ground_z = float(triangles[:, :, 2].min())
    ground_clearance = float(zero_corners[:, :, 2].min()-ground_z)
    folded_joint_corner_z = float(zero_corners[:, :, 2].min())
    # Ground clearance when the CAD just touches a horizontal plane; rotate all
    # actual STL vertices, not a surrogate body box. Chunk over attitude grid.
    vertices = triangles.reshape(-1, 3)
    # 3D extrema are attained on the convex hull. A 26-direction support subset
    # is insufficient, so use all unique vertices if scipy is unavailable.
    try:
        from scipy.spatial import ConvexHull
        unique = np.unique(vertices, axis=0)
        vertices = unique[ConvexHull(unique).vertices]
        support = "exact convex hull of every original collision STL vertex"
    except ImportError:
        support = "every original collision STL vertex"
    grounded = {}
    for angle_deg in (2., 5.):
        worst = (math.inf, None)
        for roll in np.linspace(-angle_deg, angle_deg, 21):
            for pitch in np.linspace(-angle_deg, angle_deg, 21):
                r = rotation(np.radians([roll, pitch, 0.]))
                body_min = float((vertices@r[2, :]).min())
                arm_min = float((zero_corners.reshape(-1, 3)@r[2, :]).min())
                distance = arm_min-body_min
                if distance < worst[0]:
                    worst = (distance, [roll, pitch])
        grounded[str(angle_deg)] = {"minimum_sampled_clearance_m": worst[0], "worst_roll_pitch_deg": worst[1],
                                   "angle_grid_points_per_axis": 21, "continuous_attitude_certificate": False}
    return {"kind": "continuous_arm_limit_clearance_certificate", "passed": gap > 0 and ground_clearance > 0,
            "method": "Exact CAD triangle clipping to each link's invariant Y slab + exact box corner extrema on complete joint grid + analytic nearest-grid-cell displacement bound",
            "original_collision_triangle_count": len(triangles), "per_link_slab_triangle_count": [value[1] for value in floors_and_counts],
            "link_y_intervals_m": y_intervals, "per_link_cad_minimum_z_m": body_floors.tolist(),
            "joint_grid_step_requested_deg": grid_step_deg, "joint_grid_shape": [len(grid) for grid in grids],
            "joint_grid_evaluations": len(grids[0])*len(grids[1]), "q_limits_rad": [spec["q_limits"] for spec in specs],
            "sampled_link_maximum_z_m": upper.tolist(), "sampled_link_minimum_z_m": lower.tolist(),
            "per_link_continuous_displacement_bound_m": displacements,
            "certified_link_maximum_z_m": certified_upper.tolist(), "certified_link_minimum_z_m": certified_lower.tolist(),
            "certified_original_airframe_arm_clearance_m": gap,
            "per_link_certified_original_airframe_clearance_m": per_link_gap.tolist(),
            "sampled_q_at_link_maximum_z_rad": max_q, "sampled_q_at_link_minimum_z_rad": min_q,
            "scope": "Entire original X650 collision meshes, including body, motors, landing gear and q=0 prop geometry; rotating prop disk separately bounded below",
            "rotor_disk": {"plane_z_m": 0., "radius_m": .1905, "assumed_half_thickness_m": .002,
                           "certified_vertical_arm_gap_m": -.002-float(max(certified_upper)),
                           "method": "full swept cylinder separated vertically, independent of spin angle"},
            "landing": {"original_ground_plane_z_m": ground_z, "folded_arm_lowest_z_m": folded_joint_corner_z,
                        "level_q0_clearance_m": ground_clearance, "folded_q_rad": [0., 0.],
                        "sampled_roll_pitch_clearances": grounded, "body_support_geometry": support,
                        "all_joint_positions_ground_safe": bool(float(min(certified_lower)) >= ground_z),
                        "required_minimum_root_height_above_ground_for_full_joint_range_m": -float(min(certified_lower)),
                        "operation": "Keep q=[0,0] for takeoff/landing; move arm only after airborne. Native contact/settling validation pending.",
                        "native_settling_validated": False},
            "excluded_intentional_interfaces": ["arm_joint_1 with mounting adapter"],
            "physical_limits_changed": False,
            "limitations": ["Rigid CAD and ideal rectangular links; no cable, servo housing, flexure or dimensional tolerance model.",
                            "25 g mounting adapter and its resized geometry are explicit simulation assumptions.",
                            "Grounded tilt results are sampled; continuous joint certificate concerns airframe and rotors, not ground during arm deployment."]}


def prepare(source, output, airframe_id, arm_source=None, grid_step_deg=.5):
    source = Path(source).resolve(strict=True)
    output = Path(output).resolve()
    arm_source = Path(arm_source or PROJECT/"assets/models/x500_underfold/model.sdf").resolve(strict=True)
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}")
    if output.is_relative_to(source.parent) or source.is_relative_to(output):
        raise ValueError("Output must be independent of input source")
    if type(airframe_id) is not int or airframe_id <= 0:
        raise ValueError("Explicit positive PX4 airframe ID required")
    previous = json.loads((source.parent/"preparation_report.json").read_text(encoding="utf-8"))
    if previous.get("model_name") != "x650_mn4014_v1" or previous.get("derived_urdf_sha256") != sha(source):
        raise ValueError("X650 source does not match its audited preparation report")
    robot = ET.parse(source).getroot()
    if set(ARM_NAMES) & {link.get("name") for link in robot.findall("link")}:
        raise ValueError("X650 input already contains an arm")
    initial_mass, initial_com, initial_inertia = aggregate(robot)
    if abs(initial_mass-3.02) > 1e-10:
        raise ValueError("Expected the user-confirmed 3.02 kg no-arm X650")
    triangles, mesh_records = collision_triangles(robot, source)
    specs, old_mount = original_arm(arm_source)
    audit = clearance_audit(triangles, specs, grid_step_deg)
    self_audit = self_collision_audit(specs)
    audit["arm_self_collision"] = {key: self_audit[key] for key in ("passed", "method", "certified_surface_clearance_m", "elbow_offset_body_m")}
    if not audit["passed"] or not self_audit["passed"]:
        raise ValueError("Full-range airframe clearance or folded ground clearance is not positive")
    adapter = add_arm(robot, specs)
    mass, com, inertia = aggregate(robot)
    rigid_robot = copy.deepcopy(robot)
    for node in list(rigid_robot):
        if (node.tag == "link" and node.get("name") in ARM_NAMES) or (node.tag == "joint" and node.get("name") in JOINT_NAMES):
            rigid_robot.remove(node)
    rigid_mass, rigid_com, rigid_inertia = aggregate(rigid_robot)
    profile = yaml.safe_load((source.parent/"flight_profile.yaml").read_text(encoding="utf-8"))
    profile["px4_sys_autostart"] = airframe_id
    profile["controlled_joints"] = list(JOINT_NAMES)
    profile["notes"] += " Original x500_underfold two-link arm (0.45 kg), mount FLU [0,0,-0.18] m, L2 shifted +Y 0.04 m to remove original solid-box self-intersection, explicit unmeasured 0.025 kg adapter. Original X650 landing gear retained; arm stays folded during takeoff/landing."
    poses = kinematic_poses(robot)
    motor_mapping = []
    for rotor in profile["rotors"]:
        xyz = poses[rotor["link"]][:3, 3]
        motor_mapping.append({"px4_motor": rotor["motor_number"]+1, "motor_number": rotor["motor_number"],
                              "link": rotor["link"], "turning_direction": rotor["turning_direction"],
                              "position_flu_m": xyz.tolist(), "position_com_frd_m": ((xyz-com)*[1,-1,-1]).tolist(),
                              "allocation_reference": "folded q=[0,0]; articulated COM variation must be handled by controller"})
    # Record the COM/inertia variation across all physical limit corners and q0.
    samples = []
    for q in [[0., 0.], *itertools.product(*(spec["q_limits"] for spec in specs))]:
        _, c, i = aggregate(robot, dict(zip(JOINT_NAMES, q)))
        samples.append({"q_rad": list(q), "com_flu_m": c.tolist(), "inertia_at_com_flu_kg_m2": i.tolist()})
    robot.set("name", MODEL_NAME)
    resources = {}
    for mesh in robot.findall(".//mesh"):
        relative = Path(mesh.get("filename"))
        original = (source.parent/relative).resolve(strict=True)
        if relative.is_absolute() or ".." in relative.parts or not original.is_relative_to(source.parent):
            raise ValueError("Unsafe mesh resource")
        resources[relative] = original
    output.mkdir(parents=True)
    for relative, original in resources.items():
        destination = output/relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(original, destination)
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(output/"robot.urdf", encoding="utf-8", xml_declaration=True)
    (output/"flight_profile.yaml").write_text(yaml.safe_dump(profile, sort_keys=False), encoding="utf-8")
    kinematics = {"body_frame": "FLU", "world_frame": "ENU", "mount_xyz_m": MOUNT.tolist(),
                  "elbow_offset_body_m": ELBOW_OFFSET.tolist(),
                  "mount_rpy_rad": [0., 0., 0.], "joint_names": list(JOINT_NAMES), "joint_axes_local": [[0., 1., 0.]]*2,
                  "lengths_m": [float(spec["size"][0]) for spec in specs],
                  "initial_relative_angles_rad": [spec["initial_angle"] for spec in specs],
                  "gazebo_zero_is_folded_pose": True, "joint_limits_rad": [spec["q_limits"] for spec in specs],
                  "joint_velocity_limits_rad_s": [spec["velocity"] for spec in specs],
                  "tip_in_last_link_m": [float(specs[-1]["size"][0]), 0., 0.],
                  "takeoff_landing_q_rad": [0., 0.]}
    (output/"kinematics.json").write_text(json.dumps(kinematics, indent=2)+"\n", encoding="utf-8")
    (output/"clearance_report.json").write_text(json.dumps(audit, indent=2)+"\n", encoding="utf-8")
    (output/"arm_self_collision_report.json").write_text(json.dumps(self_audit, indent=2)+"\n", encoding="utf-8")
    thrust_max = float(profile["rotors"][0]["motor_constant"])*float(profile["rotors"][0]["max_rot_velocity"])**2
    report = {"schema_version": 1, "model_name": MODEL_NAME, "kind": "x650_original_two_link_composition",
              "source_urdf_sha256": sha(source), "source_preparation_sha256": sha(source.parent/"preparation_report.json"),
              "source_original_cad_sha256": previous["source_urdf_sha256"], "original_arm_asset_sha256": sha(arm_source),
              "derived_urdf_sha256": sha(output/"robot.urdf"), "source_unchanged": sha(source)==previous["derived_urdf_sha256"],
              "flight_profile_sha256": sha(output/"flight_profile.yaml"), "generator_sha256": sha(__file__),
              "base_airframe_mass_kg": initial_mass, "two_link_mass_kg": sum(spec["mass"] for spec in specs),
              "adapter": adapter, "total_mass_kg": mass, "com_flu_q0_m": com.tolist(),
              "inertia_at_com_flu_q0_kg_m2": inertia.tolist(), "articulated_mass_properties_examples": samples,
              "motor_mapping_q0": motor_mapping, "px4_sys_autostart": airframe_id,
              "hover_prediction": {"equal_share_thrust_per_rotor_n": mass*9.80665/4,
                                   "normalized_thrust": mass*9.80665/(4*thrust_max),
                                   "model_maximum_thrust_to_weight": 4*thrust_max/(mass*9.80665)},
              "arm_source": "assets/models/x500_underfold/model.sdf; original flown two-link asset, no editable-example values substituted",
              "old_mount_xyz_m": old_mount.tolist(), "new_mount_xyz_m": MOUNT.tolist(),
              "joint_state_offset_convention_preserved": True, "copied_old_landing_gear_extensions": False,
              "mesh_resources_unchanged": all(sha(output/path)==sha(file) for path, file in resources.items()),
              "mesh_resources": {path.as_posix(): sha(file) for path, file in resources.items()},
              "original_collision_meshes": mesh_records,
              "clearance_certificate_sha256": sha(output/"clearance_report.json"),
              "kinematics_sha256": sha(output/"kinematics.json"), "flight_validated": False,
              "native_ground_contact_validated": False,
              "scope": "Composition, articulated inertia and continuous rigid-geometry clearance checked. Native Gazebo contact settling and all flight/controller checks remain separate prerequisites."}
    report["rigid_airframe_including_adapter_and_rotors"] = {
        "mass_kg": rigid_mass, "com_body_m": rigid_com.tolist(),
        "inertia_at_com_body_kg_m2": rigid_inertia.tolist(),
        "rotor_spin_assumption": "locked q=0 mass properties; balanced rotor inertia is axisymmetric"}
    report["controller_model"] = {
        "profile_name": MODEL_NAME, "body_frame": "FLU", "world_frame": "ENU",
        "mount_body": MOUNT.tolist(), "lengths": [float(spec["size"][0]) for spec in specs],
        "elbow_offset_body_m": ELBOW_OFFSET.tolist(),
        "masses": [spec["mass"] for spec in specs],
        "com_distances": [float(spec["size"][0]/2) for spec in specs],
        "angle_offsets": [specs[0]["initial_angle"], sum(spec["initial_angle"] for spec in specs)],
        "joint_zero_offsets_rad": [spec["initial_angle"] for spec in specs],
        "inertia_diagonal": [np.diag(spec["inertia"]).tolist() for spec in specs],
        "inertia_com_kg_m2": [spec["inertia"].tolist() for spec in specs],
        "total_mass": mass, "base_mass": rigid_mass, "base_com_body": rigid_com.tolist(),
        "base_inertia_at_com_body_kg_m2": rigid_inertia.tolist(),
        "joint_lower": [spec["q_limits"][0] for spec in specs],
        "joint_upper": [spec["q_limits"][1] for spec in specs],
        "joint_velocity_limits": [spec["velocity"] for spec in specs],
        "joint_effort_limits": [spec["effort"] for spec in specs],
        "length_units": "m", "mass_units": "kg", "angle_units": "rad",
        "angle_convention": "absolute link angles = angle_offsets + [q1,q1+q2]; +Y rotates +X toward -Z"}
    report["airframe_arm_geometry_and_inertia_validated"] = True
    report["rigid_geometry_and_inertia_validated"] = True
    report["physical_geometry_validated"] = True
    report["arm_self_collision_resolved"] = True
    report["arm_self_collision_report_sha256"] = sha(output/"arm_self_collision_report.json")
    report["composition_adjustment"] = "Second link joint shifted +body-Y 40 mm; original coplanar box-arm intersection documented in arm_self_collision_report.json. Lengths, masses, local inertias, axes and q convention retained."
    (output/"preparation_report.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    return report


def render_preview(source, output):
    """Draw every original visual STL plus the three added primitive boxes."""
    from render_urdf_geometry import geometry, render
    triangles, colors, record = geometry(Path(source), [0, 0, 0], [0, 0, 0], {})
    robot = ET.parse(output/"robot.urdf").getroot()
    poses = kinematic_poses(robot)
    meshes, shades = [triangles], [colors]
    faces = np.asarray([(0, 1, 3), (0, 3, 2), (4, 6, 7), (4, 7, 5),
                        (0, 4, 5), (0, 5, 1), (2, 3, 7), (2, 7, 6),
                        (0, 2, 6), (0, 6, 4), (1, 5, 7), (1, 7, 3)])
    for link in robot.findall("link"):
        for visual in link.findall("visual"):
            box = visual.find("geometry/box")
            if box is None:
                continue
            size = np.asarray([float(v) for v in box.get("size").split()])
            corners = np.asarray(list(itertools.product([-1, 1], repeat=3)))*size/2
            pose = poses[link.get("name")] @ matrix(*origin(visual))
            points = corners @ pose[:3, :3].T+pose[:3, 3]
            meshes.append(points[faces])
            rgba = np.asarray([float(v) for v in visual.find("material/color").get("rgba").split()])
            shades.append(np.repeat(rgba[None, :3], len(faces), axis=0))
    triangles, colors = np.concatenate(meshes), np.concatenate(shades)
    lower, upper = triangles.min((0, 1)), triangles.max((0, 1))
    mass, _, _ = aggregate(robot)
    record.update({"kind": "x650_two_link_composition_static_preview", "visual_count": record["visual_count"]+3,
                   "triangle_count": len(triangles), "link_count": len(robot.findall("link")), "joint_count": len(robot.findall("joint")),
                   "mass_from_urdf_kg": mass, "bounds_min_m": lower.tolist(), "bounds_max_m": upper.tolist(),
                   "dimensions_xyz_m": (upper-lower).tolist(), "joint_state": "q=[0,0], folded original arm convention",
                   "source_urdf_sha256": sha(output/"robot.urdf"), "gazebo_started": False, "flight_validated": False})
    for jname in JOINT_NAMES:
        joint = robot.find(f"joint[@name='{jname}']")
        child = joint.find("child").get("link")
        record["joints"].append({"name": jname, "child": child, "position_m": poses[child][:3, 3].tolist()})
    image = render(triangles, colors, record, output, "X650 + 原二连杆机械臂 · 折叠安装检查")
    record.update(image_sha256=sha(image), generator_sha256=sha(__file__))
    (output/"geometry_manifest.json").write_text(json.dumps(record, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
    return image


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--airframe-id", type=int, required=True)
    parser.add_argument("--arm-source", type=Path)
    parser.add_argument("--grid-step-deg", type=float, default=.5)
    parser.add_argument("--native-import", action="store_true")
    parser.add_argument("--models-root", type=Path)
    parser.add_argument("--preview", action="store_true")
    args = parser.parse_args()
    report = prepare(args.source, args.output, args.airframe_id, args.arm_source, args.grid_step_deg)
    result = {key: report[key] for key in ("model_name", "total_mass_kg", "com_flu_q0_m", "inertia_at_com_flu_q0_kg_m2", "hover_prediction", "motor_mapping_q0")}
    result["output"] = str(args.output.resolve())
    if args.preview:
        result["geometry_preview"] = str(render_preview(args.source, args.output))
    if args.native_import:
        imported = import_urdf(args.output/"robot.urdf", MODEL_NAME, output_root=args.models_root)
        flight = bind_flight_profile(imported.directory, args.output/"flight_profile.yaml", output_root=args.models_root)
        result.update(imported_directory=str(imported.directory), flight_directory=str(flight.output_dir),
                      flight_model=flight.name, flight_sha256=flight.sha256)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
