#!/usr/bin/env python3
"""Import the assembled CAD URDF at q=0 as a static Gazebo visual model.

This deliberately does not approximate mimic joints with independent dynamics.
"""
import argparse
import hashlib
import json
from pathlib import Path
import shutil
import sys
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
from amsim.modeling.urdf_preview import build_preview, check_preview


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("package", type=Path)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    package = args.package.resolve(strict=True)
    source = package / "urdf/uav_arm_urdf.urdf"
    robot = ET.parse(source).getroot()
    joints = robot.findall("joint")
    parents = {j.find("child").get("link"): j for j in joints}
    # Only the zero pose is supported. Nonzero mimic offsets would require FK.
    for joint in joints:
        mimic = joint.find("mimic")
        if mimic is not None and float(mimic.get("offset", "0")) != 0:
            raise ValueError("Nonzero mimic offsets require explicit pose conversion")
    name = "amsim_x650_cad_arm_static_" + sha(source)[:10]
    dest = ROOT / "generated/imported_models" / name
    if dest.exists():
        raise FileExistsError(f"Preserving existing import: {dest}")
    (dest / "meshes").mkdir(parents=True)
    (dest / "source").mkdir()
    shutil.copy2(source, dest / "source/original.urdf")
    sdf = ET.Element("sdf", version="1.9")
    model = ET.SubElement(sdf, "model", name=name)
    ET.SubElement(model, "static").text = "true"
    assets = {}
    for original in robot.findall("link"):
        link = ET.SubElement(model, "link", name=original.get("name"))
        joint = parents.get(original.get("name"))
        if joint is not None:
            origin = joint.find("origin")
            pose = ET.SubElement(link, "pose", relative_to=joint.find("parent").get("link"))
            pose.text = origin.get("xyz", "0 0 0") + " " + origin.get("rpy", "0 0 0")
        for index, visual in enumerate(original.findall("visual")):
            output = ET.SubElement(link, "visual", name=f"visual_{index}")
            origin = visual.find("origin")
            if origin is not None:
                ET.SubElement(output, "pose").text = origin.get("xyz", "0 0 0") + " " + origin.get("rpy", "0 0 0")
            mesh = visual.find("geometry/mesh")
            prefix = "package://uav_arm_urdf/"
            uri = mesh.get("filename")
            if not uri.startswith(prefix):
                raise ValueError(f"Unexpected mesh URI: {uri}")
            asset = (package / uri[len(prefix):]).resolve(strict=True)
            if not asset.is_relative_to(package):
                raise ValueError("Mesh outside package")
            shutil.copy2(asset, dest / "meshes" / asset.name)
            assets[str(asset)] = sha(asset)
            geometry = ET.SubElement(output, "geometry")
            target_mesh = ET.SubElement(geometry, "mesh")
            ET.SubElement(target_mesh, "uri").text = f"model://{name}/meshes/{asset.name}"
            ET.SubElement(target_mesh, "scale").text = mesh.get("scale", "1 1 1")
            color = visual.find("material/color")
            if color is not None:
                material = ET.SubElement(output, "material")
                for tag in ("ambient", "diffuse"):
                    ET.SubElement(material, tag).text = color.get("rgba")
    ET.indent(sdf)
    ET.ElementTree(sdf).write(dest / "model.sdf", encoding="utf-8", xml_declaration=True)
    config = ET.Element("model")
    ET.SubElement(config, "name").text = name
    ET.SubElement(config, "version").text = "1.0"
    ET.SubElement(config, "sdf", version="1.9").text = "model.sdf"
    ET.SubElement(config, "description").text = "Static CAD zero-pose preview only; no flight or arm dynamics."
    ET.ElementTree(config).write(dest / "model.config", encoding="utf-8", xml_declaration=True)
    report = dict(kind="static_cad_preview", source=str(source), source_sha256=sha(source),
                  assets=assets, links=len(robot.findall("link")), joints=len(joints),
                  source_mass_kg=sum(float(x.get("value")) for x in robot.findall("link/inertial/mass")),
                  mimic_joints=[j.get("name") for j in joints if j.find("mimic") is not None],
                  missing_inertial=[l.get("name") for l in robot.findall("link") if l.find("inertial") is None],
                  dynamic_ready=False, flight_ready=False,
                  limitations="q=0 visuals only; no collisions, actuators, or inertial simulation")
    (dest / "cad_import_report.json").write_text(json.dumps(report, indent=2) + "\n")
    world = build_preview(dest, height=0.8, static=True)
    print(json.dumps(dict(model=str(dest), world=str(world)), indent=2), flush=True)
    if args.check:
        result = check_preview(world, iterations=100)
        print(json.dumps(result, indent=2))
        if not result["ok"]:
            raise SystemExit(1)


if __name__ == "__main__":
    main()
