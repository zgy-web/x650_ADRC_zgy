"""Standalone, bounded Gazebo previews of imported URDF models; never starts PX4."""
from __future__ import annotations

from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import signal
import subprocess
import time
import uuid
import xml.etree.ElementTree as ET

from ..paths import PROJECT_ROOT


class UrdfPreviewError(ValueError):
    pass


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_xml(root: ET.Element, path: Path):
    ET.indent(root,space="  ")
    ET.ElementTree(root).write(path,encoding="utf-8",xml_declaration=True)


def _put(parent: ET.Element, tag: str, value, **attrs):
    node = ET.SubElement(parent,tag,**attrs)
    node.text = str(value)
    return node


def preview_environment(model_dir: Path) -> dict[str,str]:
    """An explicit resource path for callers opening the original imported model."""
    parent = str(Path(model_dir).resolve().parent)
    existing = os.environ.get("GZ_SIM_RESOURCE_PATH","")
    paths = [parent]+[p for p in existing.split(os.pathsep) if p and p != parent]
    return {"GZ_SIM_RESOURCE_PATH":os.pathsep.join(paths)}


def build_preview(model_dir: Path, output_dir: Path | None = None, height: float = .5, static: bool = False) -> Path:
    """Create a standalone world and URI-only model copy, preserving source physics.

    Absolute mesh paths in the preview copy avoid shell resource-path setup.
    Use ``gui_command`` from the generated manifest so interactive Gazebo also
    runs in this preview's isolated transport partition. The portable imported
    model remains unchanged.
    ``static=True`` is a visual preview only; the default remains dynamic.
    """
    model_dir = Path(model_dir).expanduser().resolve(strict=True)
    source_sdf = model_dir/"model.sdf"
    if not source_sdf.is_file():
        raise UrdfPreviewError(f"No model.sdf in {model_dir}")
    try:
        height = float(height)
    except (TypeError,ValueError) as exc:
        raise UrdfPreviewError("height must be a finite non-negative number in metres") from exc
    if not math.isfinite(height) or height < 0 or not isinstance(static,bool):
        raise UrdfPreviewError("height must be finite/non-negative and static must be boolean")
    document = ET.parse(source_sdf).getroot()
    models = document.findall("model")
    if document.tag != "sdf" or len(models) != 1:
        raise UrdfPreviewError("Preview requires one standalone SDF model")
    model_name = models[0].get("name","")
    if not model_name:
        raise UrdfPreviewError("Imported SDF model has no name")
    if document.findall(".//include"):
        raise UrdfPreviewError("Nested external SDF includes are unsupported in URDF previews")
    resources = {}
    for node in document.findall(".//mesh/uri"):
        uri = (node.text or "").strip()
        prefix = f"model://{model_name}/"
        if not uri.startswith(prefix):
            raise UrdfPreviewError(f"Expected a portable imported model mesh URI, got {uri!r}")
        relative = PurePosixPath(uri[len(prefix):])
        if relative.is_absolute() or ".." in relative.parts or "\\" in uri or ":" in str(relative):
            raise UrdfPreviewError(f"Mesh path escapes imported model: {uri}")
        file = model_dir.joinpath(*relative.parts).resolve(strict=True)
        if not file.is_relative_to(model_dir) or not file.is_file():
            raise UrdfPreviewError(f"Mesh is outside imported model: {uri}")
        resources[str(file)] = _sha(file)
        node.text = file.as_uri()
    partition = "amsim-urdf-preview-"+uuid.uuid4().hex
    token = partition.rsplit("-",1)[-1][:12]
    output = (Path(output_dir).expanduser().resolve() if output_dir else
              PROJECT_ROOT/"runs"/f"urdf_preview_{datetime.now(timezone.utc):%Y%m%d_%H%M%S}_{token}")
    if output == model_dir:
        raise UrdfPreviewError("Use a separate preview output directory")
    output.mkdir(parents=True,exist_ok=True)
    world_path = output/"world.sdf"
    preview_sdf = output/"preview_model.sdf"
    metadata_path = output/"preview_manifest.json"
    if any(p.exists() for p in (world_path,preview_sdf,metadata_path)):
        raise UrdfPreviewError(f"Refusing to overwrite an existing preview: {output}")
    _write_xml(document,preview_sdf)
    sdf = ET.Element("sdf",version="1.9")
    world = ET.SubElement(sdf,"world",name=f"urdf_preview_{token}")
    physics = ET.SubElement(world,"physics",name="preview_physics",type="ignored")
    _put(physics,"max_step_size","0.001")
    _put(physics,"real_time_factor","1")
    _put(world,"gravity","0 0 -9.80665")
    for library,system in (("physics","Physics"),("user-commands","UserCommands"),("scene-broadcaster","SceneBroadcaster")):
        ET.SubElement(world,"plugin",filename=f"gz-sim-{library}-system",name=f"gz::sim::systems::{system}")
    scene = ET.SubElement(world,"scene")
    _put(scene,"ambient","0.4 0.4 0.4 1")
    _put(scene,"background","0.75 0.78 0.82 1")
    _put(scene,"shadows","true")
    light = ET.SubElement(world,"light",name="sun",type="directional")
    _put(light,"pose","0 0 10 0 0 0")
    _put(light,"cast_shadows","true")
    _put(light,"diffuse","0.9 0.9 0.9 1")
    _put(light,"specular","0.2 0.2 0.2 1")
    _put(light,"direction","-0.5 0.2 -1")
    ground = ET.SubElement(world,"model",name="ground_plane")
    _put(ground,"static","true")
    link = ET.SubElement(ground,"link",name="ground_link")
    for kind in ("collision","visual"):
        item = ET.SubElement(link,kind,name=f"ground_{kind}")
        plane = ET.SubElement(ET.SubElement(item,"geometry"),"plane")
        _put(plane,"normal","0 0 1")
        _put(plane,"size","20 20")
        if kind == "visual":
            material = ET.SubElement(item,"material")
            _put(material,"ambient","0.55 0.55 0.55 1")
            _put(material,"diffuse","0.65 0.65 0.65 1")
    include = ET.SubElement(world,"include")
    _put(include,"uri",preview_sdf.as_uri())
    _put(include,"name",model_name)
    _put(include,"pose",f"0 0 {height:.17g} 0 0 0")
    _put(include,"static","true" if static else "false")
    _write_xml(sdf,world_path)
    metadata = {
        "schema_version":1,"kind":"urdf_preview","model_name":model_name,
        "model_dir":str(model_dir),"source_sdf":str(source_sdf),"source_sdf_sha256":_sha(source_sdf),
        "world":str(world_path),"world_sha256":_sha(world_path),
        "preview_model":str(preview_sdf),"preview_model_sha256":_sha(preview_sdf),
        "mesh_files":resources,"height_m":height,"static":static,"dynamic":not static,
        "physics_changed":static,"model_parameters_changed":False,"uri_changes_only_in_preview_copy":True,
        "px4_started":False,"flight_validated":False,
        "partition":partition,
        "gui_command":["env",f"GZ_PARTITION={partition}","gz","sim",str(world_path)],
        "warning":"Unpowered import/load preview. A dynamic model falls under gravity; this does not validate flight or controller tuning.",
    }
    metadata_path.write_text(json.dumps(metadata,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    return world_path


def _stop_own_process(process):
    """Stop only the process/session created by this check, never named processes."""
    if process.poll() is not None:
        return
    if os.name == "posix":
        try:
            os.killpg(process.pid,signal.SIGTERM)
        except ProcessLookupError:
            return
    else:
        process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        if os.name == "posix":
            try:
                os.killpg(process.pid,signal.SIGKILL)
            except ProcessLookupError:
                return
        else:
            process.kill()
        process.wait(timeout=3)


def check_preview(world_path: Path, iterations: int = 100, timeout_s: float = 60) -> dict:
    """Run a bounded Gazebo-only load check in an isolated transport partition.

    Success means native SDF validation and a zero-exit bounded server run without
    reported loader errors. It makes no claim about flight/contact stability.
    """
    world_path = Path(world_path).expanduser().resolve(strict=True)
    if isinstance(iterations,bool) or not isinstance(iterations,int) or not 1 <= iterations <= 100000:
        raise UrdfPreviewError("iterations must be an integer between 1 and 100000")
    try:
        timeout_s = float(timeout_s)
    except (TypeError,ValueError) as exc:
        raise UrdfPreviewError("timeout_s must be between 0 and 300 seconds") from exc
    if not math.isfinite(timeout_s) or not 0 < timeout_s <= 300:
        raise UrdfPreviewError("timeout_s must be between 0 and 300 seconds")
    metadata_path = world_path.parent/"preview_manifest.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    if metadata.get("kind") != "urdf_preview" or Path(metadata.get("world","")).resolve() != world_path:
        raise UrdfPreviewError("Not a world generated by build_preview")
    hashes = {str(world_path):metadata["world_sha256"],metadata["source_sdf"]:metadata["source_sdf_sha256"],metadata["preview_model"]:metadata["preview_model_sha256"],**metadata["mesh_files"]}
    for filename,digest in hashes.items():
        file = Path(filename)
        if not file.is_file() or _sha(file) != digest:
            raise UrdfPreviewError(f"Preview source/artifact changed: {filename}")
    partition = metadata.get("partition","")
    if not isinstance(partition,str) or re.fullmatch(r"amsim-urdf-preview-[0-9a-f]{32}",partition) is None:
        raise UrdfPreviewError("Missing or invalid isolated partition; rebuild this preview")
    env = os.environ.copy()
    env.update(preview_environment(Path(metadata["model_dir"])))
    env["GZ_PARTITION"] = partition
    command = ["gz","sim","-s","-r","-v","3","--iterations",str(iterations),str(world_path)]
    log_path = world_path.parent/"gazebo_check.log"
    result_path = world_path.parent/"preview_check.json"
    if log_path.exists() or result_path.exists():
        raise UrdfPreviewError("Preview was already checked; build a new output directory for another check")
    result = {"schema_version":1,"kind":"urdf_preview_check","ok":False,"world":str(world_path),
              "iterations":iterations,"timeout_s":timeout_s,"partition":partition,"command":command,
              "log":str(log_path),"result":str(result_path),"dynamic":metadata["dynamic"],
              "px4_started":False,"flight_validated":False,"timed_out":False,"returncode":None}
    started = time.monotonic()
    process = None
    try:
        validation = subprocess.run(["gz","sdf","-k",str(world_path)],capture_output=True,text=True,env=env,timeout=min(float(timeout_s),30),check=False)
        result["sdf_validation"] = {"returncode":validation.returncode,"output":(validation.stdout+validation.stderr)[-10000:]}
        if validation.returncode or "Valid." not in validation.stdout+validation.stderr:
            result["error"] = "Native Gazebo SDF world validation failed"
        else:
            with log_path.open("xb") as log:
                process = subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,env=env,start_new_session=(os.name=="posix"))
                try:
                    result["returncode"] = process.wait(timeout=float(timeout_s))
                except subprocess.TimeoutExpired:
                    result["timed_out"] = True
                    result["error"] = "Gazebo preview exceeded its wall-clock timeout"
                    _stop_own_process(process)
                    result["returncode"] = process.returncode
            with log_path.open("rb") as log:
                size = log_path.stat().st_size
                if size > 2*1024*1024:
                    log.seek(-2*1024*1024,os.SEEK_END)
                output = log.read().decode("utf-8",errors="replace")
            errors = [line for line in output.splitlines() if re.search(r"\[(?:Err|Error)\]|Unable to find|Unable to load|Failed to load|Error Code",line,re.IGNORECASE)]
            result["loader_errors"] = errors[-40:]
            result["log_tail_scanned_bytes"] = min(size,2*1024*1024)
            result["log_size_bytes"] = size
            result["ok"] = result["returncode"] == 0 and not result["timed_out"] and not errors and size <= 2*1024*1024
            if not result["ok"] and "error" not in result:
                result["error"] = "Gazebo returned nonzero, reported loader errors, or exceeded the checked log-size bound"
    except FileNotFoundError:
        result["error"] = "Native gz is unavailable; build/inspection remains available, but run this check on the Gazebo server"
    except subprocess.TimeoutExpired:
        result["timed_out"] = True
        result["error"] = "Native SDF validation timed out"
    finally:
        if process is not None:
            _stop_own_process(process)
        result["elapsed_wall_s"] = time.monotonic()-started
        result_path.write_text(json.dumps(result,indent=2,ensure_ascii=False)+"\n",encoding="utf-8")
    return result


def run_preview_check(model_dir: Path, output_dir: Path | None = None, height: float = .5, static: bool = False, iterations: int = 100, timeout_s: float = 60) -> dict:
    return check_preview(build_preview(model_dir,output_dir,height,static),iterations,timeout_s)
