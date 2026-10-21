"""Import user URDF assets without inventing flight dynamics or changing the source.

Inspection is portable; conversion requires the server's native ``gz sdf`` tool.
Only self-contained STL/DAE/OBJ geometry is supported. Textures and externally
referenced materials are rejected explicitly, rather than silently lost.
"""
from __future__ import annotations

from contextlib import contextmanager
from collections import Counter
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import stat
import subprocess
import tempfile
import xml.etree.ElementTree as ET
import zipfile

from ..paths import PROJECT_ROOT

MAX_ARCHIVE_BYTES = 256 * 1024 * 1024
MAX_EXTRACTED_BYTES = 512 * 1024 * 1024
MAX_FILE_BYTES = 128 * 1024 * 1024
MAX_ARCHIVE_FILES = 10000
CONVERSION_TIMEOUT_S = 120


class UrdfImportError(ValueError):
    """The source cannot be imported without an unsupported or ambiguous change."""


@dataclass(frozen=True)
class ImportedModel:
    model_name: str
    directory: Path
    urdf_path: Path
    sdf_path: Path
    manifest_path: Path
    sha256: str
    root_link: str
    joints: list[str]
    total_mass_kg: float


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _xml(path: Path) -> ET.Element:
    if path.stat().st_size > MAX_FILE_BYTES:
        raise UrdfImportError(f"XML file exceeds size limit: {path.name}")
    raw = path.read_bytes()
    markup = raw.replace(b"\x00",b"").upper()  # Also catches UTF-16/32 declarations.
    if b"<!DOCTYPE" in markup or b"<!ENTITY" in markup:
        raise UrdfImportError("XML document types and entities are not supported")
    try:
        return ET.fromstring(raw)
    except ET.ParseError as exc:
        raise UrdfImportError(f"Invalid XML in {path.name}: {exc}") from exc


def _safe_member(name: str) -> PurePosixPath:
    # Backslashes are rejected, including Windows traversal and UNC names.
    p = PurePosixPath(name)
    if not name or "\\" in name or ":" in name or p.is_absolute() or any(
        part in ("..", "") for part in p.parts
    ):
        raise UrdfImportError(f"Unsafe archive path: {name!r}")
    return p


@contextmanager
def _source(source: Path, urdf_entry: str | None):
    source = Path(source).expanduser().resolve(strict=True)
    if not source.is_file():
        raise UrdfImportError("source must be a .urdf file or .zip archive")
    if source.suffix.lower() == ".urdf":
        if urdf_entry is not None:
            raise UrdfImportError("urdf_entry applies only to ZIP archives")
        yield source, source.parent, source.name
        return
    if source.suffix.lower() != ".zip":
        raise UrdfImportError("source must be a .urdf file or .zip archive")
    if source.stat().st_size > MAX_ARCHIVE_BYTES:
        raise UrdfImportError("ZIP exceeds compressed size limit")
    with tempfile.TemporaryDirectory(prefix="amsim-urdf-source-") as tmp:
        unpacked = Path(tmp)
        try:
            with zipfile.ZipFile(source) as archive:
                members = archive.infolist()
                if len(members) > MAX_ARCHIVE_FILES:
                    raise UrdfImportError("ZIP exceeds file count limit")
                seen: set[str] = set()
                size = 0
                for member in members:
                    rel = _safe_member(member.filename)
                    key = rel.as_posix().casefold()
                    if key in seen:
                        raise UrdfImportError(f"Duplicate archive path: {rel}")
                    seen.add(key)
                    mode = member.external_attr >> 16
                    if stat.S_ISLNK(mode) or (stat.S_IFMT(mode) not in (0, stat.S_IFREG, stat.S_IFDIR)):
                        raise UrdfImportError(f"ZIP special files are not supported: {rel}")
                    if member.flag_bits & 1:
                        raise UrdfImportError("Encrypted ZIP archives are not supported")
                    size += member.file_size
                    if member.file_size > MAX_FILE_BYTES or size > MAX_EXTRACTED_BYTES:
                        raise UrdfImportError("ZIP exceeds uncompressed size limit")
                    destination = unpacked.joinpath(*rel.parts)
                    if member.is_dir():
                        destination.mkdir(parents=True, exist_ok=True)
                    else:
                        destination.parent.mkdir(parents=True, exist_ok=True)
                        # Stream and enforce the limit as well as checking ZIP metadata.
                        with archive.open(member) as inp, destination.open("xb") as out:
                            copied = 0
                            while chunk := inp.read(1024 * 1024):
                                copied += len(chunk)
                                if copied > member.file_size or copied > MAX_FILE_BYTES:
                                    raise UrdfImportError("ZIP entry exceeds declared size")
                                out.write(chunk)
            candidates = sorted(p for p in unpacked.rglob("*") if p.is_file() and p.suffix.lower() == ".urdf")
            if urdf_entry is None:
                if len(candidates) != 1:
                    names = [p.relative_to(unpacked).as_posix() for p in candidates]
                    raise UrdfImportError(f"ZIP contains {len(candidates)} URDF files; select urdf_entry: {names}")
                selected = candidates[0]
            else:
                selected = unpacked.joinpath(*_safe_member(urdf_entry).parts)
                if not selected.is_file() or selected.suffix.lower() != ".urdf":
                    raise UrdfImportError(f"URDF archive entry not found: {urdf_entry}")
            yield selected, unpacked, selected.relative_to(unpacked).as_posix()
        except (zipfile.BadZipFile, RuntimeError) as exc:
            raise UrdfImportError(f"Cannot read ZIP: {exc}") from exc


def _numbers(value, count: int, label: str) -> list[float]:
    try:
        result = [float(x) for x in (value.split() if isinstance(value, str) else value)]
    except (ValueError, TypeError) as exc:
        raise UrdfImportError(f"{label} must contain {count} finite numbers") from exc
    if len(result) != count or not all(math.isfinite(x) for x in result):
        raise UrdfImportError(f"{label} must contain {count} finite numbers")
    return result


def _number(value, label: str) -> float:
    return _numbers([value], 1, label)[0]


def _origin(parent: ET.Element) -> dict:
    node = parent.find("origin")
    return {
        "xyz": _numbers(node.get("xyz", "0 0 0") if node is not None else "0 0 0", 3, "origin xyz"),
        "rpy": _numbers(node.get("rpy", "0 0 0") if node is not None else "0 0 0", 3, "origin rpy"),
    }


def _det(a):
    return (a[0][0] * (a[1][1]*a[2][2]-a[1][2]*a[2][1])
            - a[0][1] * (a[1][0]*a[2][2]-a[1][2]*a[2][0])
            + a[0][2] * (a[1][0]*a[2][1]-a[1][1]*a[2][0]))


def _inertia(node: ET.Element, name: str) -> dict:
    mass_node, inertia_node = node.find("mass"), node.find("inertia")
    if mass_node is None or inertia_node is None:
        raise UrdfImportError(f"{name}: inertial requires mass and inertia")
    mass = _number(mass_node.get("value"), f"{name} mass")
    if mass <= 0:
        raise UrdfImportError(f"{name}: mass must be positive")
    values = {key: _number(inertia_node.get(key), f"{name} {key}")
              for key in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")}
    a = [[values["ixx"], values["ixy"], values["ixz"]],
         [values["ixy"], values["iyy"], values["iyz"]],
         [values["ixz"], values["iyz"], values["izz"]]]
    scale = max(abs(x) for row in a for x in row)
    if not scale:
        raise UrdfImportError(f"{name}: inertia must be positive definite")
    a = [[v/scale for v in row] for row in a]
    if a[0][0] <= 0 or a[0][0]*a[1][1]-a[0][1]**2 <= 0 or _det(a) <= 0:
        raise UrdfImportError(f"{name}: inertia must be positive definite")
    half_trace = sum(a[i][i] for i in range(3))/2
    b = [[(half_trace if i == j else 0)-a[i][j] for j in range(3)] for i in range(3)]
    minors = [b[i][i] for i in range(3)] + [b[i][i]*b[j][j]-b[i][j]**2 for i,j in ((0,1),(0,2),(1,2))] + [_det(b)]
    if min(minors) < -1e-10:
        raise UrdfImportError(f"{name}: principal inertia triangle inequality fails")
    return {"mass_kg": mass, "origin": _origin(node), "tensor": values}


def _mesh_file(uri: str, urdf: Path, boundary: Path, packages: dict[str, Path]) -> Path:
    if uri.startswith("package://"):
        parts = PurePosixPath(uri[len("package://"):]).parts
        if len(parts) < 2 or parts[0] not in packages or ".." in parts:
            raise UrdfImportError(f"Provide an explicit package_paths mapping for mesh {uri}")
        root = Path(packages[parts[0]]).expanduser().resolve(strict=True)
        candidate = root.joinpath(*parts[1:]).resolve(strict=True)
        if not candidate.is_relative_to(root):
            raise UrdfImportError(f"Package mesh escapes package root: {uri}")
    else:
        if not uri or ":" in uri or "\\" in uri or PurePosixPath(uri).is_absolute():
            raise UrdfImportError(f"Use relative or explicitly mapped package:// mesh URIs: {uri}")
        candidate = (urdf.parent / uri).resolve(strict=True)
        if not candidate.is_relative_to(boundary):
            raise UrdfImportError(f"Relative mesh escapes source directory; use package_paths: {uri}")
    if not candidate.is_file() or candidate.stat().st_size > MAX_FILE_BYTES:
        raise UrdfImportError(f"Mesh missing or exceeds size limit: {uri}")
    suffix = candidate.suffix.lower()
    if suffix not in (".stl", ".dae", ".obj"):
        raise UrdfImportError(f"Supported mesh formats are STL, DAE and OBJ: {uri}")
    if suffix == ".obj":
        for line in candidate.read_text(encoding="utf-8", errors="strict").splitlines():
            words = line.split()
            if words and words[0].lower() in ("mtllib", "usemtl"):
                raise UrdfImportError(f"OBJ external materials are unsupported; export untextured geometry: {uri}")
    elif suffix == ".dae":
        dae = _xml(candidate)
        for node in dae.iter():
            if node.tag.rsplit("}",1)[-1] == "image" or any(
                key in ("url", "source") and value and not value.startswith("#")
                for key,value in node.attrib.items()
            ):
                raise UrdfImportError(f"DAE external images/references are unsupported: {uri}")
    return candidate


def _inspect(urdf: Path, boundary: Path, packages: dict[str, Path]):
    robot = _xml(urdf)
    if robot.tag != "robot":
        raise UrdfImportError("Expected a URDF <robot> root")
    unsupported = {node.tag for node in robot if node.tag not in ("link", "joint", "material")}
    if unsupported:
        raise UrdfImportError(f"Unsupported URDF extensions (use a separate simulator profile): {sorted(unsupported)}")
    if robot.findall(".//texture"):
        raise UrdfImportError("URDF textures are unsupported; export untextured geometry")
    links, joints, warnings = [], [], []
    link_names: set[str] = set()
    for node in robot.findall("link"):
        name = node.get("name", "")
        if not name or name in link_names:
            raise UrdfImportError(f"Missing or duplicate link name: {name!r}")
        link_names.add(name)
        inertial = node.find("inertial")
        if inertial is None:
            raise UrdfImportError(f"{name}: missing inertial; dynamic imports require explicit positive mass/inertia")
        data = _inertia(inertial, name)
        links.append({"name":name, **data, "visual_count":len(node.findall("visual")), "collision_count":len(node.findall("collision"))})
        for geometry in node.findall("visual") + node.findall("collision"):
            _origin(geometry)
            shape = geometry.find("geometry")
            if shape is None or len(shape) != 1 or shape[0].tag not in ("mesh", "box", "sphere", "cylinder"):
                raise UrdfImportError(f"{name}: unsupported or ambiguous geometry")
            primitive = shape[0]
            attributes = {"box": [("size",3)], "sphere": [("radius",1)], "cylinder": [("radius",1),("length",1)]}.get(primitive.tag, [])
            for key,count in attributes:
                vals = _numbers(primitive.get(key), count, f"{name} {primitive.tag} {key}")
                if min(vals) <= 0:
                    raise UrdfImportError(f"{name}: geometry dimensions must be positive")
    names, children = set(), set()
    for node in robot.findall("joint"):
        name, kind = node.get("name", ""), node.get("type", "")
        if not name or name in names:
            raise UrdfImportError(f"Missing or duplicate joint name: {name!r}")
        names.add(name)
        if kind not in ("fixed", "continuous", "revolute", "prismatic") or node.find("mimic") is not None:
            raise UrdfImportError(f"{name}: supported joints are fixed/continuous/revolute/prismatic without mimic")
        parent_node, child_node = node.find("parent"), node.find("child")
        parent = parent_node.get("link") if parent_node is not None else None
        child = child_node.get("link") if child_node is not None else None
        if parent not in link_names or child not in link_names or parent == child or child in children:
            raise UrdfImportError(f"{name}: invalid parent/child or multiple parents")
        children.add(child)
        axis_node, limit_node = node.find("axis"), node.find("limit")
        axis = _numbers(axis_node.get("xyz", "1 0 0") if axis_node is not None else "1 0 0", 3, f"{name} axis") if kind != "fixed" else None
        if axis is not None and abs(sum(x*x for x in axis)-1) > 1e-6:
            raise UrdfImportError(f"{name}: joint axis must be a unit vector")
        limits = {}
        if kind != "fixed":
            if limit_node is None:
                raise UrdfImportError(f"{name}: moving joint requires explicit effort/velocity limits")
            limits = {key:_number(value, f"{name} limit {key}") for key,value in limit_node.attrib.items()}
            if any(limits.get(key,0) <= 0 for key in ("effort", "velocity")):
                raise UrdfImportError(f"{name}: effort/velocity limits must be positive")
            if kind in ("revolute", "prismatic") and ("lower" not in limits or "upper" not in limits or limits["lower"] > limits["upper"]):
                raise UrdfImportError(f"{name}: requires ordered lower/upper limits")
        valid = kind not in ("revolute", "prismatic") or limits["lower"] <= 0 <= limits["upper"]
        if not valid:
            warnings.append(f"{name}: initial q=0 is outside joint limits; imported as declared, not flight-ready")
        joints.append({"name":name,"type":kind,"parent":parent,"child":child,"origin":_origin(node),"axis":axis,"limits":limits,"initial_position":0.0,"initial_position_within_limits":valid})
    roots = link_names-children
    if len(roots) != 1:
        raise UrdfImportError(f"Expected one root link, found {sorted(roots)}")
    root = next(iter(roots))
    visited, pending = set(), [root]
    while pending:
        name = pending.pop()
        if name in visited:
            raise UrdfImportError("URDF joint graph contains a cycle")
        visited.add(name)
        pending.extend(j["child"] for j in joints if j["parent"] == name)
    if visited != link_names or len(joints) != len(links)-1:
        raise UrdfImportError("URDF is disconnected or contains a cycle")
    resources, meshes = {}, []
    for node in robot.findall(".//mesh"):
        uri = node.get("filename", "")
        try:
            file = _mesh_file(uri, urdf, boundary, packages)
        except FileNotFoundError as exc:
            raise UrdfImportError(f"Mesh or package path not found: {uri}") from exc
        scale = _numbers(node.get("scale", "1 1 1"), 3, f"mesh scale {uri}")
        if min(scale) <= 0:
            raise UrdfImportError(f"Mesh scale must be positive: {uri}")
        digest = _sha(file)
        asset_name = digest + file.suffix.lower()
        resources[asset_name] = file
        meshes.append({"source_uri":uri,"sha256":digest,"source_relative_path":file.relative_to(boundary).as_posix() if file.is_relative_to(boundary) else uri,"asset_name":asset_name,"scale":scale})
    if len(robot.findall(".//collision/geometry/mesh")) > 50:
        warnings.append("Many mesh collision objects; contact performance has not been validated")
    warnings.append("URDF geometry/inertia do not define rotor aerodynamics, PX4 outputs, sensors or flight-controller tuning")
    return robot, {"root_link":root,"links":links,"joints":joints,"total_mass_kg":sum(l["mass_kg"] for l in links),"meshes":meshes,"warnings":warnings,"flight_ready":False}, resources


def _rotation(rpy):
    r,p,y = rpy
    cr,sr,cp,sp,cy,sy = math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    return [[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr], [sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr], [-sp,cp*sr,cp*cr]]


def _multiply(a,b):
    return [[sum(a[i][k]*b[k][j] for k in range(3)) for j in range(3)] for i in range(3)]


def _reframe(robot: ET.Element, root: str, xyz, rpy):
    rotation = _rotation(rpy)
    base = next(l for l in robot.findall("link") if l.get("name") == root)
    targets = base.findall("visual") + base.findall("collision") + base.findall("inertial")
    targets += [j for j in robot.findall("joint") if j.find("parent").get("link") == root]
    for target in targets:
        old = _origin(target)
        translated = [xyz[i]+sum(rotation[i][k]*old["xyz"][k] for k in range(3)) for i in range(3)]
        a = _multiply(rotation, _rotation(old["rpy"]))
        pitch = math.asin(max(-1., min(1., -a[2][0])))
        if abs(math.cos(pitch)) > 1e-10:
            angles = [math.atan2(a[2][1],a[2][2]),pitch,math.atan2(a[1][0],a[0][0])]
        else:
            angles = [math.atan2(-a[1][2],a[1][1]),pitch,0.]
        origin = target.find("origin")
        if origin is None:
            origin = ET.SubElement(target, "origin")
        origin.set("xyz", " ".join(format(x,".17g") for x in translated))
        origin.set("rpy", " ".join(format(x,".17g") for x in angles))


def _frame_report(robot: ET.Element, report: dict):
    """Report transformed local origins and derived zero-configuration root axes."""
    links = {n.get("name"):n for n in robot.findall("link")}
    joints = {n.get("name"):n for n in robot.findall("joint")}
    for link in report["links"]:
        link["original_origin"] = link["origin"]
        link["origin"] = _origin(links[link["name"]].find("inertial"))
    pending = list(report["joints"])
    poses = {report["root_link"]:(_rotation((0,0,0)),[0.,0.,0.])}
    while pending:
        for joint in pending[:]:
            if joint["parent"] not in poses:
                continue
            joint["original_origin"] = joint["origin"]
            joint["origin"] = _origin(joints[joint["name"]])
            pr,pt = poses[joint["parent"]]
            origin = joint["origin"]
            rotation = _multiply(pr,_rotation(origin["rpy"]))
            position = [pt[i]+sum(pr[i][k]*origin["xyz"][k] for k in range(3)) for i in range(3)]
            poses[joint["child"]] = (rotation,position)
            joint["pose_in_root_at_zero"] = {"xyz":position,"rotation":rotation}
            joint["axis_in_root_at_zero"] = ([sum(rotation[i][k]*joint["axis"][k] for k in range(3)) for i in range(3)] if joint["axis"] else None)
            pending.remove(joint)


def inspect_urdf(source: Path, urdf_entry: str | None = None, *, root_xyz=(0,0,0), root_rpy=(0,0,0), package_paths: dict[str, Path] | None = None) -> dict:
    """Return a serializable source audit, with no persistent writes or conversion."""
    source = Path(source).expanduser().resolve(strict=True)
    xyz, rpy = _numbers(root_xyz,3,"root_xyz"), _numbers(root_rpy,3,"root_rpy")
    with _source(source, urdf_entry) as (selected,boundary,entry):
        robot, report, _ = _inspect(selected,boundary,package_paths or {})
        if any(xyz) or any(rpy):
            _reframe(robot,report["root_link"],xyz,rpy)
        _frame_report(robot,report)
        report.update({"schema_version":1,"kind":"urdf_inspection","source":{"path":str(source),"sha256":_sha(source),"urdf_entry":entry,"urdf_sha256":_sha(selected)},"root_transform":{"xyz":xyz,"rpy":rpy},"converted":False})
        return report


def _convert(urdf: Path, sdf: Path, model_name: str) -> dict:
    env = os.environ.copy()
    # The staged directory is already named after the model. Supply its parent
    # to native URI resolvers without changing the calling process environment.
    for key in ("GZ_SIM_RESOURCE_PATH", "SDF_PATH"):
        existing = env.get(key, "")
        env[key] = str(urdf.parent.parent) + (os.pathsep+existing if existing else "")
    try:
        result = subprocess.run(["gz","sdf","-p",str(urdf)],capture_output=True,text=True,env=env,timeout=CONVERSION_TIMEOUT_S,check=False)
    except FileNotFoundError as exc:
        raise UrdfImportError("Native 'gz sdf' is required for conversion; use inspect_urdf on this host and import on the Gazebo server") from exc
    except subprocess.TimeoutExpired as exc:
        raise UrdfImportError("gz sdf conversion timed out") from exc
    if result.returncode:
        raise UrdfImportError("gz sdf conversion failed: " + (result.stderr+result.stdout)[-5000:])
    try:
        root = ET.fromstring(result.stdout)
    except ET.ParseError as exc:
        raise UrdfImportError("gz sdf did not return valid SDF XML: " + result.stderr[-2000:]) from exc
    models = root.findall("model")
    if root.tag != "sdf" or len(models) != 1:
        raise UrdfImportError("Expected exactly one converted SDF model")
    models[0].set("name",model_name)
    ET.indent(root,space="  ")
    ET.ElementTree(root).write(sdf,encoding="utf-8",xml_declaration=True)
    try:
        validated = subprocess.run(["gz","sdf","-k",str(sdf)],capture_output=True,text=True,env=env,timeout=CONVERSION_TIMEOUT_S,check=False)
    except subprocess.TimeoutExpired as exc:
        raise UrdfImportError("gz sdf validation timed out") from exc
    if validated.returncode or "Valid." not in validated.stdout+validated.stderr:
        raise UrdfImportError("Converted SDF validation failed: " + (validated.stdout+validated.stderr)[-5000:])
    return {"tool":"gz sdf -p / gz sdf -k","validated":True,"stderr":result.stderr[-5000:],"validation_output":(validated.stdout+validated.stderr)[-5000:]}


def _model_name(name: str) -> str:
    cleaned = re.sub(r"[^a-zA-Z0-9_]+","_",str(name)).strip("_").lower()
    if not cleaned:
        raise UrdfImportError("Provide a model name containing ASCII letters or digits")
    if cleaned.startswith("amsim_urdf_"):
        return cleaned
    return "amsim_urdf_" + cleaned


def _result(directory: Path, report: dict) -> ImportedModel:
    return ImportedModel(report["model_name"],directory,directory/"model.urdf",directory/"model.sdf",directory/"import_manifest.json",_sha(directory/"model.sdf"),report["root_link"],[j["name"] for j in report["joints"] if j["type"]!="fixed"],report["total_mass_kg"])


def import_urdf(source: Path, name: str, urdf_entry: str | None = None, output_root: Path | None = None, root_xyz=(0,0,0), root_rpy=(0,0,0), package_paths: dict[str, Path] | None = None) -> ImportedModel:
    """Convert and atomically publish a portable model; never overwrite an import.

    ``root_xyz/root_rpy`` are a coordinate change on the root link contents and
    direct child joint frames (Rz Ry Rx), not a simulator spawn pose. Repeating an
    identical import returns the verified existing artifact.
    """
    source = Path(source).expanduser().resolve(strict=True)
    xyz, rpy = _numbers(root_xyz,3,"root_xyz"), _numbers(root_rpy,3,"root_rpy")
    model_name = _model_name(name)
    parent = Path(output_root or PROJECT_ROOT/"generated"/"imported_models").expanduser().resolve()
    destination = parent/model_name
    if source.is_relative_to(destination):
        raise UrdfImportError("Output directory cannot contain the input source")
    with _source(source,urdf_entry) as (selected,boundary,entry):
        robot, report, resources = _inspect(selected,boundary,package_paths or {})
        identity = {"source_sha256":_sha(source),"urdf_entry":entry,"urdf_sha256":_sha(selected),"root_transform":{"xyz":xyz,"rpy":rpy},"mesh_sha256":sorted(resources)}
        fingerprint = hashlib.sha256(json.dumps(identity,sort_keys=True).encode()).hexdigest()
        if destination.exists():
            if destination.is_symlink():
                raise UrdfImportError("Refusing symbolic-link output directory")
            manifest = destination/"import_manifest.json"
            if not manifest.is_file():
                raise UrdfImportError(f"Refusing to overwrite existing directory: {destination}")
            previous = json.loads(manifest.read_text(encoding="utf-8"))
            if previous.get("import_fingerprint") != fingerprint or previous.get("model_name") != model_name:
                raise UrdfImportError(f"Import name already belongs to a different source/configuration: {destination}")
            if not {"model.sdf","model.urdf","model.config","source/original.urdf"}.issubset(previous.get("artifacts",{})):
                raise UrdfImportError("Existing import manifest is incomplete")
            for rel,digest in previous.get("artifacts",{}).items():
                file = destination.joinpath(*_safe_member(rel).parts)
                if file.is_symlink() or not file.resolve().is_relative_to(destination) or not file.is_file() or _sha(file) != digest:
                    raise UrdfImportError(f"Existing import has been modified: {rel}")
            return _result(destination,previous)
        parent.mkdir(parents=True,exist_ok=True)
        with tempfile.TemporaryDirectory(prefix=".urdf-import-",dir=parent) as temporary:
            stage = Path(temporary)/model_name
            (stage/"source"/"resources").mkdir(parents=True)
            (stage/"meshes").mkdir()
            shutil.copyfile(selected,stage/"source"/"original.urdf")
            for asset,file in resources.items():
                shutil.copyfile(file,stage/"meshes"/asset)
                shutil.copyfile(file,stage/"source"/"resources"/asset)
            robot.set("name",model_name)
            # An identity transform does not rewrite source numeric formatting.
            if any(xyz) or any(rpy):
                _reframe(robot,report["root_link"],xyz,rpy)
            _frame_report(robot,report)
            for node,mesh in zip(robot.findall(".//mesh"),report["meshes"]):
                uri = f"model://{model_name}/meshes/{mesh['asset_name']}"
                node.set("filename",uri)
                mesh["output_uri"] = uri
                mesh["source_snapshot"] = f"source/resources/{mesh['asset_name']}"
            ET.indent(robot,space="  ")
            ET.ElementTree(robot).write(stage/"model.urdf",encoding="utf-8",xml_declaration=True)
            conversion = _convert(stage/"model.urdf",stage/"model.sdf",model_name)
            sdf_root = _xml(stage/"model.sdf")
            expected = Counter(mesh["output_uri"] for mesh in report["meshes"])
            actual = Counter(node.text for node in sdf_root.findall(".//mesh/uri"))
            if actual != expected:
                raise UrdfImportError("Converted SDF dropped mesh geometry or has non-portable/unexpected mesh references")
            sdf_links = sdf_root.findall("model/link")
            if report["root_link"] not in {link.get("name") for link in sdf_links}:
                raise UrdfImportError("Converted SDF lost the canonical root link")
            converted_mass = sum(float(link.findtext("inertial/mass", "0")) for link in sdf_links)
            if not math.isclose(converted_mass,report["total_mass_kg"],rel_tol=1e-8,abs_tol=1e-10):
                raise UrdfImportError("Converted SDF mass differs from the URDF source")
            moving = {j["name"] for j in report["joints"] if j["type"] != "fixed"}
            if not moving.issubset({j.get("name") for j in sdf_root.findall("model/joint")}):
                raise UrdfImportError("Converted SDF lost movable joints")
            config = ET.Element("model")
            ET.SubElement(config,"name").text=model_name
            ET.SubElement(config,"version").text="1.0"
            ET.SubElement(config,"sdf",version=sdf_root.get("version","1.9")).text="model.sdf"
            ET.SubElement(config,"description").text="Imported URDF geometry and inertia; no flight dynamics are inferred."
            ET.indent(config,space="  ")
            ET.ElementTree(config).write(stage/"model.config",encoding="utf-8",xml_declaration=True)
            report.update({"schema_version":1,"kind":"urdf_import","model_name":model_name,"source":{"path":str(source),"sha256":identity["source_sha256"],"urdf_entry":entry,"urdf_sha256":identity["urdf_sha256"],"snapshot":"source/original.urdf"},"root_transform":identity["root_transform"],"import_fingerprint":fingerprint,"converted":True,"conversion":conversion,"artifacts":{p.relative_to(stage).as_posix():_sha(p) for p in sorted(stage.rglob("*")) if p.is_file()}})
            (stage/"import_manifest.json").write_text(json.dumps(report,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
            # rename is atomic on the same filesystem and fails if another import won.
            stage.rename(destination)
        return _result(destination,report)
