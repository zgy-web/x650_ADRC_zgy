#!/usr/bin/env python3
"""Build the traceable X650 MN4014 commissioning model; never launch PX4.

The original user CAD is immutable. Propulsion uses the user's selected
MN4014 KV330 / 15*5CF / 22.2 V table. Unmeasured dynamics remain estimates.
"""
from __future__ import annotations

import argparse
import copy
import hashlib
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
from amsim.modeling.urdf_importer import _reframe, import_urdf
from amsim.modeling.urdf_flight import bind_flight_profile

SOURCE_SHA = "643f5c1d8a57eeffa822b044cba7b9ec6b1a0d72186b8e524fb0d081be3943bb"
MODEL_NAME = "x650_mn4014_v1"
MOTOR_URL = "https://store.tmotor.com/product/mn4014-kv330-motor-navigator-type.html"
PROP_URL = "https://store.tmotor.com/product/polish-carbon-fiber-15x5-prop.html"
GZ_URL = "https://raw.githubusercontent.com/gazebosim/gz-sim/gz-sim8/src/systems/multicopter_motor_model/MulticopterMotorModel.cc"
# Simulation numbering authorized by the user; this does not claim real wiring.
MAPPING = [("link_004", "front_right", "ccw"), ("link_002", "rear_left", "ccw"),
           ("link_003", "front_left", "cw"), ("link_005", "rear_right", "cw")]
PROP_PARTS = {"link_002": "part_278", "link_003": "part_279", "link_004": "part_277", "link_005": "part_273"}


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def rotation(rpy):
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                     [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]])


def angles(r):
    p = math.asin(np.clip(-r[2, 0], -1, 1))
    if abs(math.cos(p)) > 1e-10:
        return [math.atan2(r[2, 1], r[2, 2]), p, math.atan2(r[1, 0], r[0, 0])]
    return [math.atan2(-r[1, 2], r[1, 1]), p, 0.]


def origin(node):
    elem = node.find("origin")
    if elem is None:
        return np.zeros(3), np.eye(3)
    return np.array([float(x) for x in elem.get("xyz", "0 0 0").split()]), rotation([float(x) for x in elem.get("rpy", "0 0 0").split()])


def set_origin(node, xyz, r=np.eye(3)):
    elem = node.find("origin")
    if elem is None:
        elem = ET.SubElement(node, "origin")
    elem.set("xyz", " ".join(format(float(x), ".17g") for x in xyz))
    elem.set("rpy", " ".join(format(float(x), ".17g") for x in angles(r)))


def parallel_axis(x):
    return float(x @ x) * np.eye(3) - np.outer(x, x)


def properties(link, translation=np.zeros(3), orientation=np.eye(3)):
    inertial = link.find("inertial")
    mass = float(inertial.find("mass").get("value"))
    xyz, r = origin(inertial)
    i = inertial.find("inertia").attrib
    tensor = np.array([[float(i["ixx"]), float(i["ixy"]), float(i["ixz"])],
                       [float(i["ixy"]), float(i["iyy"]), float(i["iyz"])],
                       [float(i["ixz"]), float(i["iyz"]), float(i["izz"])]])
    total_r = orientation @ r
    return mass, translation + orientation @ xyz, total_r @ tensor @ total_r.T


def set_properties(link, mass, com, inertia):
    eig = np.linalg.eigvalsh(inertia)
    if min(eig) <= 0 or 2 * max(eig) > sum(eig) + 1e-12:
        raise ValueError(f"Nonphysical derived inertia for {link.get('name')}: {eig}")
    old = link.find("inertial")
    if old is not None:
        link.remove(old)
    node = ET.SubElement(link, "inertial")
    set_origin(node, com)
    ET.SubElement(node, "mass", value=format(mass, ".17g"))
    ET.SubElement(node, "inertia", **{k: format(float(v), ".17g") for k, v in
                                    zip(("ixx", "iyy", "izz", "ixy", "ixz", "iyz"),
                                        (inertia[0,0], inertia[1,1], inertia[2,2], inertia[0,1], inertia[0,2], inertia[1,2]))})


def aggregate(robot):
    joints = {j.find("child").get("link"): j for j in robot.findall("joint")}
    values = []
    for link in robot.findall("link"):
        pose = origin(joints[link.get("name")]) if link.get("name") in joints else (np.zeros(3), np.eye(3))
        values.append(properties(link, *pose))
    mass = sum(v[0] for v in values)
    com = sum(m*c for m, c, i in values) / mass
    inertia = sum(i + m*parallel_axis(c-com) for m, c, i in values)
    return mass, com, inertia


def fit_propulsion():
    rpm = np.array([3900., 4600., 5100., 5600., 6000.])
    thrust_gf = np.array([830., 1150., 1430., 1690., 1920.])
    thrust = thrust_gf * .00980665
    omega = rpm * 2*math.pi/60
    x = omega**2
    kt = float((x @ thrust)/(x @ x))
    predicted = kt*x
    hover_n = 3.02*9.80665/4
    hover_rpm = math.sqrt(hover_n/kt)*60/(2*math.pi)
    return {
        "source": {"type": "user_selected_manufacturer_table", "url": MOTOR_URL,
                   "matched_official_table": True, "motor": "MN4014 KV330", "propeller": "T-MOTOR 15*5CF",
                   "test_voltage_v": 22.2, "reported_battery": "6S 30000 mAh", "esc_model": None},
        "samples": [{"rpm": float(r), "thrust_gf": float(t), "current_a": a, "throttle_percent": u}
                    for r,t,a,u in zip(rpm, thrust_gf, [3.6,5.9,7.8,10.1,11.9], [50,65,75,85,100])],
        "fit": {"method": "unweighted least squares through origin: T=kT*omega^2",
                "motor_constant": kt, "units": "N/(rad/s)^2", "rmse_n": float(np.sqrt(np.mean((predicted-thrust)**2))),
                "max_abs_relative_error_percent": float(np.max(np.abs((predicted-thrust)/thrust))*100),
                "predicted_thrust_n": predicted.tolist(), "residual_n": (predicted-thrust).tolist()},
        "limits": {"maximum_rpm": 6000., "maximum_rad_s": float(omega[-1]),
                   "measured_maximum_thrust_n": float(thrust[-1]), "model_maximum_thrust_n": float(predicted[-1]),
                   "note": "Maximum is the highest selected KV330/15*5CF/22.2V table point, not KV400's 6700 rpm."},
        "hover_prediction": {"all_up_mass_kg": 3.02, "average_thrust_per_rotor_n": hover_n,
                             "rpm": hover_rpm, "normalized_rpm": hover_rpm/6000,
                             "normalized_thrust": hover_n/float(predicted[-1]),
                             "below_lowest_measured_rpm": True,
                             "extrapolation_fraction_below_minimum": 1-hover_rpm/3900,
                             "model_maximum_thrust_to_weight": 4*float(predicted[-1])/(3.02*9.80665)},
        "preliminary_dynamics": {"moment_constant_m": .016, "time_constant_up_s": .0125,
                                 "time_constant_down_s": .025, "rotor_drag_coefficient": .0001,
                                 "rolling_moment_coefficient": 1e-6, "rotor_velocity_slowdown_sim": 10.,
                                 "source": GZ_URL, "status": "Gazebo 8 reference defaults; NOT measured MN4014 parameters"},
        "limitations": ["Hover RPM is extrapolated below the lowest measured 3900 rpm / 830 gf sample.",
                        "Fixed 22.2 V curve; battery voltage sag, discharge and endurance are not modeled.",
                        "Torque ratio, rotor response and aerodynamic drag use explicitly labeled reference estimates.",
                        "Manufacturer throttle is not equated with PX4 normalized thrust or speed.",
                        "The user confirmed 3.02 kg; no additional battery mass is added to the URDF."]}


def profile(airframe_id, fit):
    result = {"schema_version": 1, "type": "px4_multicopter", "base_link": "base_link", "body_frame": "FLU",
              "px4_sys_autostart": airframe_id, "controlled_joints": [],
              "notes": "X650 commissioning model: user-selected MN4014 KV330, T-MOTOR 15*5CF, 22.2 V table; "
                       "kT fitted through origin, 6000 rpm cap. Simulation motor mapping authorized by user, "
                       "not claimed as real wiring. Gazebo8 reference KM/tau/drag estimates; see preparation_report.json. "
                       + MOTOR_URL, "rotors": []}
    for number, (link, location, direction) in enumerate(MAPPING):
        result["rotors"].append({"motor_number": number, "joint": link+"_joint", "link": link,
                                 "turning_direction": direction, "motor_constant": fit["fit"]["motor_constant"],
                                 "moment_constant": .016, "max_rot_velocity": fit["limits"]["maximum_rad_s"],
                                 "time_constant_up": .0125, "time_constant_down": .025,
                                 "rotor_drag_coefficient": .0001, "rolling_moment_coefficient": 1e-6,
                                 "rotor_velocity_slowdown_sim": 10., "allow_continuous_conversion": False,
                                 "notes": f"Simulation M{number+1}, {location}, {direction} viewed from above."})
    return result


def prepare(source: Path, output: Path, airframe_id: int):
    source = source.resolve(strict=True)
    output = output.resolve()
    if sha(source) != SOURCE_SHA:
        raise ValueError("Input does not match the audited original X650 robot.urdf SHA256")
    if output.exists():
        raise FileExistsError(f"Refusing to overwrite {output}")
    if source.is_relative_to(output) or output.is_relative_to(source.parent):
        raise ValueError("Derived output must be outside the original source directory")
    if airframe_id <= 0:
        raise ValueError("Explicit positive airframe ID required")
    robot = ET.parse(source).getroot()
    _reframe(robot, "base_link", (0,0,-.17095298), (math.pi/2,0,0))
    old_mass, old_com, old_inertia = aggregate(robot)
    links = {link.get("name"):link for link in robot.findall("link")}
    base = links["base_link"]
    fit = fit_propulsion()
    rotor_mass, radius, thickness = .021, .381/2, .001
    rotor_inertia = np.diag([rotor_mass*(3*radius**2+thickness**2)/12]*2 + [rotor_mass*radius**2/2])
    rotor_positions = []
    relocation = []
    for name, label, direction in MAPPING:
        joint = robot.find(f"joint[@name='{name}_joint']")
        xyz, r = origin(joint)
        rotor_positions.append(xyz)
        link = links[name]
        for tag in ("visual", "collision"):
            for node in list(link.findall(tag)):
                mesh = node.find("geometry/mesh")
                is_prop = PROP_PARTS[name] in mesh.get("filename", "")
                p, orient = origin(node)
                # Canonicalize the joint rotation while preserving all mesh poses.
                set_origin(node, r@p if is_prop else xyz+r@p, r@orient)
                if not is_prop:
                    link.remove(node)
                    base.append(node)
                    relocation.append({"from_link": name, "to_link": "base_link", "tag": tag, "mesh": mesh.get("filename")})
        set_origin(joint, xyz)
        joint.set("type", "continuous")
        joint.find("axis").set("xyz", "0 0 1")
        limit = joint.find("limit")
        for key in ("lower", "upper"):
            limit.attrib.pop(key, None)
        limit.set("velocity", format(fit["limits"]["maximum_rad_s"], ".17g"))
        # Gazebo's velocity-controlled actuator supplies the prescribed spin;
        # this inherited effort bound is not a claim about motor shaft torque.
        limit.set("effort", "10")
        set_properties(link, rotor_mass, np.zeros(3), rotor_inertia)
    base_mass = old_mass - len(rotor_positions)*rotor_mass
    base_com = (old_mass*old_com - rotor_mass*sum(rotor_positions))/base_mass
    origin_inertia = old_inertia + old_mass*parallel_axis(old_com)
    base_inertia = (origin_inertia - sum(rotor_inertia+rotor_mass*parallel_axis(p) for p in rotor_positions)
                    - base_mass*parallel_axis(base_com))
    set_properties(base, base_mass, base_com, base_inertia)
    mass, com, inertia = aggregate(robot)
    if abs(mass-old_mass)>1e-12 or np.max(abs(com-old_com))>1e-12 or np.max(abs(inertia-old_inertia))>1e-12:
        raise AssertionError("Mass/COM/inertia conservation failed")
    # Copy only referenced mesh resources, avoiding writes to the original CAD.
    meshes = {}
    for mesh in robot.findall(".//mesh"):
        rel = Path(mesh.get("filename"))
        if rel.is_absolute() or ".." in rel.parts:
            raise ValueError(f"Unexpected mesh path {rel}")
        mesh_source = (source.parent/rel).resolve(strict=True)
        if not mesh_source.is_relative_to(source.parent):
            raise ValueError("Mesh escapes source tree")
        meshes[rel] = mesh_source
    output.mkdir(parents=True)
    for relative, mesh_source in meshes.items():
        target = output/relative
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(mesh_source, target)
    robot.set("name", MODEL_NAME)
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(output/"robot.urdf", encoding="utf-8", xml_declaration=True)
    (output/"flight_profile.yaml").write_text(yaml.safe_dump(profile(airframe_id, fit), sort_keys=False), encoding="utf-8")
    report = {"schema_version": 1, "model_name": MODEL_NAME, "source_urdf_sha256": SOURCE_SHA,
              "derived_urdf_sha256": sha(output/"robot.urdf"), "source_unchanged": sha(source)==SOURCE_SHA,
              "fit": fit, "px4_sys_autostart": airframe_id,
              "rotor_inertial_approximation": {"type": "balanced axisymmetric thin disk, mass-equivalent body redistribution",
                  "rotating_mass_kg": rotor_mass, "reference_mass_source": PROP_URL,
                  "reference_mass_note": "Official P15*5 single propeller mass 21 +/-1.5g; installed version not weighed. Bell inertia is not independently modeled.",
                  "disk_radius_m": radius, "disk_thickness_m": thickness,
                  "spin_inertia_kg_m2": rotor_inertia.tolist(), "motor_visuals_fixed_to_body": relocation,
                  "base_mass_kg": base_mass, "base_com_flu_m": base_com.tolist(),
                  "base_inertia_kg_m2": base_inertia.tolist(),
                  "scope": "Preserves original total mass, COM and q=0 inertia. Removes CAD rotor imbalance and angle-dependent tensor; not a calibrated gyroscopic model. Slowdown is a numerical simulation convention."},
              "conservation": {"total_mass_kg": mass, "com_flu_m": com.tolist(), "inertia_at_com_flu_kg_m2": inertia.tolist(),
                  "mass_error_kg": mass-old_mass, "max_com_error_m": float(np.max(abs(com-old_com))),
                  "max_inertia_error_kg_m2": float(np.max(abs(inertia-old_inertia)))},
              "motor_mapping": [{"motor_number": n, "px4_motor": n+1, "link": link,
                  "position": label, "turning_direction_above": direction,
                  "position_flu_m": pos.tolist(), "position_com_frd_m": ((pos-com)*[1,-1,-1]).tolist(),
                  "px4_km_sign": 1 if direction=="ccw" else -1}
                  for n, ((link,label,direction),pos) in enumerate(zip(MAPPING,rotor_positions))],
              "flight_validated": False,
              "remaining_assumptions": ["Physical motor wiring and forward direction are not claimed verified.",
                  "Effort=10Nm is inherited CAD numerical bound, not measured motor torque.",
                  "No voltage sag, motor thermal limits, battery endurance or calibrated motor-bell gyroscopic dynamics."]}
    (output/"preparation_report.json").write_text(json.dumps(report, indent=2)+"\n", encoding="utf-8")
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--airframe-id", type=int, required=True)
    parser.add_argument("--native-import", action="store_true")
    parser.add_argument("--models-root", type=Path)
    args = parser.parse_args()
    report = prepare(args.source, args.output, args.airframe_id)
    result = {"prepared": str(args.output.resolve()), "model_name": MODEL_NAME,
              "fit": report["fit"]["fit"], "hover_prediction": report["fit"]["hover_prediction"],
              "conservation": report["conservation"]}
    if args.native_import:
        imported = import_urdf(args.output/"robot.urdf", MODEL_NAME, output_root=args.models_root)
        flight = bind_flight_profile(imported.directory, args.output/"flight_profile.yaml", output_root=args.models_root)
        result.update(imported_directory=str(imported.directory), flight_directory=str(flight.output_dir),
                      flight_model=flight.name, flight_sha256=flight.sha256)
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
