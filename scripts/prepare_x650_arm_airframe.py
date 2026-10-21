"""Derive an independent PX4 airframe from an audited X650+arm composition.

No runtime parameters, PX4 processes or existing no-arm files are changed.
The allocation origin is the articulated vehicle COM at folded q=[0,0].
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
TEMPLATE = ROOT / "configs/vehicles/x650_no_arm/4999_gz_x650_mn4014_v1"


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def derive(report_path: Path):
    report_path = Path(report_path).resolve(strict=True)
    folder = report_path.parent
    report = json.loads(report_path.read_text(encoding="utf-8"))
    if (report.get("kind") != "x650_original_two_link_composition"
            or report.get("rigid_geometry_and_inertia_validated") is not True):
        raise ValueError("An audited X650 two-link composition report is required")
    if sha(folder / "robot.urdf") != report.get("derived_urdf_sha256"):
        raise ValueError("Composition URDF hash differs from its report")
    if sha(folder / "clearance_report.json") != report.get("clearance_certificate_sha256"):
        raise ValueError("Clearance certificate differs from composition report")
    clearance = json.loads((folder / "clearance_report.json").read_text(encoding="utf-8"))
    if (clearance.get("passed") is not True
            or report.get("physical_geometry_validated") is not True
            or report.get("arm_self_collision_resolved") is not True
            or clearance.get("arm_self_collision", {}).get("passed") is not True):
        raise ValueError("Composition must pass airframe, rotor and two-link self-collision checks")
    self_collision_path = folder / "arm_self_collision_report.json"
    if sha(self_collision_path) != report.get("arm_self_collision_report_sha256"):
        raise ValueError("Independent self-collision certificate differs from composition report")
    self_collision = json.loads(self_collision_path.read_text(encoding="utf-8"))
    if self_collision.get("passed") is not True:
        raise ValueError("Independent two-link self-collision certificate did not pass")
    profile_path = folder / "flight_profile.yaml"
    if sha(profile_path) != report.get("flight_profile_sha256"):
        raise ValueError("Flight profile hash differs from the composition report")
    profile = yaml.safe_load(profile_path.read_text(encoding="utf-8"))
    autostart = profile.get("px4_sys_autostart")
    if autostart != 4998 or report.get("px4_sys_autostart") != autostart:
        raise ValueError("This separate X650 two-link airframe must use ID 4998")
    if profile.get("controlled_joints") != ["arm_joint_1", "arm_joint_2"]:
        raise ValueError("The original two controlled joints are required")
    name = "amsim_urdf_" + str(report["model_name"]) + "_flight"
    if not re.fullmatch(r"amsim_urdf_x650_2link_[A-Za-z0-9_]+_flight", name):
        raise ValueError("Unexpected composition model name")
    mass = float(report["total_mass_kg"])
    if not math.isfinite(mass) or mass <= 3.02:
        raise ValueError("Explicit total mass must include X650 and the two-link arm")
    component_mass = (float(report["base_airframe_mass_kg"])+float(report["two_link_mass_kg"])
                      +float(report["adapter"]["mass_kg"]))
    if not math.isclose(component_mass, mass, rel_tol=1e-10, abs_tol=1e-10):
        raise ValueError("Total mass differs from the explicit airframe/arm/adapter budget")
    com = np.asarray(report["com_flu_q0_m"], dtype=float)
    if com.shape != (3,) or not np.isfinite(com).all():
        raise ValueError("Invalid folded vehicle COM")
    mapping = sorted(report["motor_mapping_q0"], key=lambda r: r["motor_number"])
    rotors = sorted(profile["rotors"], key=lambda r: r["motor_number"])
    if len(mapping) != 4 or len(rotors) != 4:
        raise ValueError("Four explicit rotor mappings are required")
    replacements = {}
    columns = []
    maxima = []
    axis = np.array([0., 0., -1.])
    for i, (mapped, rotor) in enumerate(zip(mapping, rotors)):
        if (mapped["motor_number"] != i or rotor["motor_number"] != i
                or mapped["link"] != rotor["link"]
                or mapped["turning_direction"] != rotor["turning_direction"]
                or rotor["turning_direction"] not in {"cw", "ccw"}):
            raise ValueError("Motor numbering differs between profile and audited geometry")
        pos = (np.asarray(mapped["position_flu_m"], float)-com)*[1., -1., -1.]
        if not np.allclose(pos, mapped["position_com_frd_m"], atol=1e-10, rtol=0):
            raise ValueError("Motor FRD positions do not match the whole-vehicle COM")
        maximum = min(628, math.floor(float(rotor["max_rot_velocity"])))
        if maximum <= 0:
            raise ValueError("Rotor maximum speed must be positive")
        ct = float(rotor["motor_constant"])*maximum**2
        raw_km = float(rotor["moment_constant"])
        km = raw_km*(1 if rotor["turning_direction"] == "ccw" else -1)
        if not (math.isfinite(ct) and ct > 0 and math.isfinite(km) and raw_km > 0):
            raise ValueError("Invalid thrust/torque coefficients")
        maxima.append(ct)
        for key, value in zip(("PX", "PY", "PZ"), pos):
            replacements[f"CA_ROTOR{i}_{key}"] = float(value)
        replacements[f"CA_ROTOR{i}_CT"] = ct
        replacements[f"CA_ROTOR{i}_KM"] = km
        replacements[f"SIM_GZ_EC_MAX{i+1}"] = maximum
        columns.append(np.r_[ct*np.cross(pos, axis)-ct*km*axis, -ct])
    matrix = np.column_stack(columns)
    mg = mass*9.80665
    fractions = np.linalg.solve(matrix, [0., 0., 0., -mg])
    retained = matrix.copy()
    retained[np.max(np.abs(retained), axis=1) <= .05] = 0.
    if np.linalg.matrix_rank(retained) != 4 or np.any(fractions <= 0) or np.any(fractions >= 1):
        raise ValueError("Folded static trim is infeasible or loses control authority")
    if not np.allclose(maxima, maxima[0], atol=1e-10, rtol=0):
        raise ValueError("This hover initializer assumes four identical propulsion units")
    replacements["MPC_THR_HOVER"] = mg/sum(maxima)
    text = TEMPLATE.read_text(encoding="utf-8").replace("\r\n", "\n")
    text = text.replace("# AMSIM X650 commissioning v1;", "# AMSIM X650 original two-link commissioning;")
    text = text.replace("amsim_urdf_x650_mn4014_v1_flight", name)
    for parameter, value in replacements.items():
        line = f"param set {parameter} {value:.12g}"
        text, count = re.subn(r"^param set "+re.escape(parameter)+r" [^\n]+", line, text, flags=re.MULTILINE)
        if count != 1:
            raise ValueError(f"Template parameter must occur exactly once: {parameter}")
    text = text.replace("# Positions are FRD relative to the whole-vehicle COM (metres).",
                        "# Positions are FRD relative to folded q=[0,0] whole-vehicle COM.\n"
                        "# Arm motion changes actual COM; this is a fixed nominal allocation geometry.\n"
                        "# New X650+arm PID tuning and native ground/flight validation remain separate.")
    metadata = {"kind": "x650_two_link_airframe_derivation", "airframe_id": autostart,
                "model": name, "composition_report_sha256": sha(report_path),
                "flight_profile_sha256": sha(profile_path), "template_sha256": sha(TEMPLATE),
                "arm_self_collision_report_sha256": sha(self_collision_path),
                "airframe_sha256": hashlib.sha256(text.encode()).hexdigest(),
                "total_mass_kg": mass, "com_flu_q0_m": com.tolist(),
                "hover_thrust_fraction": replacements["MPC_THR_HOVER"],
                "static_trim_motor_thrust_fractions": fractions.tolist(),
                "static_trim_residual": (matrix@fractions-[0, 0, 0, -mg]).tolist(),
                "effectiveness_rank_after_threshold": int(np.linalg.matrix_rank(retained)),
                "native_ground_contact_validated": report.get("native_ground_contact_validated", False),
                "flight_started": False, "flight_validated": False,
                "note": "Configuration derivation does not certify arm self-collision, native settling, PID tuning or flight."}
    return text, metadata


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True, help="New output directory, never overwritten")
    args = parser.parse_args()
    text, metadata = derive(args.report)
    args.output.mkdir(parents=True, exist_ok=False)
    filename = "4998_gz_"+metadata["model"].removeprefix("amsim_urdf_").removesuffix("_flight")
    (args.output/filename).write_bytes(text.encode())
    (args.output/"airframe_derivation.json").write_text(json.dumps(metadata, indent=2)+"\n", encoding="utf-8")
    print(json.dumps({"output": str(args.output), "airframe": filename, **metadata}, indent=2))


if __name__ == "__main__":
    main()
