from __future__ import annotations

import hashlib
import json
import math
import re
import shutil
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path, PurePosixPath

import yaml

from ..paths import PROJECT_ROOT, px4_dir
from .generator import build, list_configs
from ..runtime.model_metadata import normalize_model

REVIEWED_NAME = "x500_underfold"
REVIEWED_SHA = "8c86eb60a3e8ed494ec54d08d2ad6a04eb44a5f5e3f8e4ec5bc2dcc5ccc226c8"

@dataclass(frozen=True)
class ModelSpec:
    ref: str
    model_name: str
    sdf_path: Path
    sha256: str
    joints: list[str]
    source: str
    config_path: Path | None = None
    total_arm_mass_kg: float | None = None
    base_link: str = "base_link"
    manifest_path: Path | None = None
    total_mass_kg: float | None = None
    flight_ready: bool = True
    px4_sys_autostart: int = 4001
    initial_joint_positions: dict[str, float] = field(default_factory=dict)


IMPORTED_SOURCES = {"urdf_import", "urdf_flight"}
INSTALL_MARKER = ".amsim_install.json"


def _artifact_path(directory: Path, relative: str) -> Path:
    if not isinstance(relative, str) or not relative or "\\" in relative or ":" in relative:
        raise ValueError(f"Invalid manifest artifact path: {relative!r}")
    path = PurePosixPath(relative)
    if path.is_absolute() or path.as_posix() != relative or any(part in (".", "..") for part in path.parts):
        raise ValueError(f"Invalid manifest artifact path: {relative!r}")
    candidate = directory.joinpath(*path.parts)
    if candidate.is_symlink() or not candidate.resolve().is_relative_to(directory):
        raise ValueError(f"Manifest artifact escapes its model directory: {relative}")
    return candidate


def _imported_spec(path: Path, ref: str) -> ModelSpec:
    if path.is_symlink() or path.parent.is_symlink():
        raise ValueError("Imported model path cannot be a symbolic link")
    path = path.resolve()
    directory = path.parent
    if path.name != "import_manifest.json" or not path.is_file():
        raise FileNotFoundError(f"Expected imported model manifest: {path}")
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("Imported model manifest must be an object")
    kind = data.get("kind")
    if kind not in IMPORTED_SOURCES or data.get("schema_version") != 1:
        raise ValueError("Unsupported imported model manifest schema")
    schema = data.get("schema")
    if schema is not None and schema != {"kind": kind, "version": 1}:
        raise ValueError("Imported model manifest has conflicting schema declarations")
    name = data.get("model_name")
    if not isinstance(name, str) or not re.fullmatch(r"amsim_urdf_[A-Za-z0-9_]+", name):
        raise ValueError("Imported models must use the amsim_urdf_ namespace")
    if directory.name != name:
        raise ValueError("Imported model directory and manifest model name disagree")
    ready = kind == "urdf_flight"
    if data.get("flight_ready") is not ready:
        raise ValueError("Imported model kind and flight_ready disagree")
    artifacts = data.get("artifacts")
    required = {"model.sdf", "model.urdf", "model.config", "source/original.urdf"}
    if ready:
        required.update({"flight_profile.yaml", "source_import_manifest.json"})
    if not isinstance(artifacts, dict) or not required.issubset(artifacts):
        raise ValueError("Imported model artifact manifest is incomplete")
    actual_files = set()
    for entry in directory.rglob("*"):
        if entry.is_symlink() or not entry.resolve().is_relative_to(directory):
            raise ValueError(f"Imported model contains a symbolic link: {entry}")
        if entry.is_file() and entry.relative_to(directory).as_posix() != INSTALL_MARKER:
            actual_files.add(entry.relative_to(directory).as_posix())
    if actual_files != set(artifacts) | {"import_manifest.json"}:
        raise ValueError("Imported model contains missing or untracked artifact files")
    for relative, digest in artifacts.items():
        entry = _artifact_path(directory, relative)
        if not isinstance(digest, str) or not re.fullmatch(r"[0-9a-f]{64}", digest):
            raise ValueError(f"Invalid manifest artifact SHA256: {relative}")
        if not entry.is_file() or _sha(entry) != digest:
            raise ValueError(f"Imported model artifact missing or hash mismatch: {relative}")
    sdf = directory / "model.sdf"
    model_sha = artifacts["model.sdf"]
    for field_name in ("model_sha256", "sdf_sha256"):
        if field_name in data and data[field_name] != model_sha:
            raise ValueError("Imported model SDF hash declarations disagree")
    root = ET.parse(sdf).getroot()
    models = root.findall("model")
    if root.tag != "sdf" or len(models) != 1 or models[0].get("name") != name:
        raise ValueError("Imported SDF model identity disagrees with manifest")
    model = models[0]
    if ET.parse(directory / "model.config").getroot().findtext("name") != name:
        raise ValueError("Imported model.config identity disagrees with manifest")
    mass = sum(float(link.findtext("inertial/mass", "0")) for link in model.findall("link"))
    if not math.isfinite(mass) or mass <= 0:
        raise ValueError("Imported SDF total mass must be positive and finite")
    declared_mass = data.get("total_mass_kg", mass)
    if not isinstance(declared_mass, (int, float)) or isinstance(declared_mass, bool) or not math.isclose(declared_mass, mass, rel_tol=1e-8, abs_tol=1e-10):
        raise ValueError("Imported model total mass disagrees with SDF")
    base_link = data.get("base_link", data.get("root_link"))
    if base_link not in {link.get("name") for link in model.findall("link")}:
        raise ValueError("Imported model base link is absent from SDF")
    joints = data.get("controlled_joints", []) if ready else []
    if ready and data.get("joints") != joints:
        raise ValueError("Flight manifest controlled joint declarations disagree")
    autostart = data.get("px4_sys_autostart") if ready else 4001
    if ready and (type(autostart) is not int or autostart <= 0):
        raise ValueError("Flight manifest requires a positive explicit PX4 airframe ID")
    normalized = normalize_model({
        "model": name, "model_sha256": model_sha, "joint_names": joints,
        "source": kind, "base_link": base_link, "px4_sys_autostart": autostart,
        "flight_ready": ready, "initial_joint_positions": data.get("initial_joint_positions", {}),
    }) if ready else {"joint_names": [], "initial_joint_positions": {}}
    if ready:
        profile = yaml.safe_load((directory / "flight_profile.yaml").read_text(encoding="utf-8"))
        if not isinstance(profile, dict) or profile.get("px4_sys_autostart") != autostart or profile.get("controlled_joints") != joints or profile.get("base_link") != base_link:
            raise ValueError("Flight profile and manifest runtime settings disagree")
        if data.get("profile_sha256") != artifacts["flight_profile.yaml"] or data.get("source_manifest_sha256") != artifacts["source_import_manifest.json"]:
            raise ValueError("Flight profile/source manifest hashes disagree")
        sdf_joints = {joint.get("name") for joint in model.findall("joint")}
        if not set(joints).issubset(sdf_joints):
            raise ValueError("Controlled joints are absent from flight SDF")
        rotor_names = {rotor.get("joint") for rotor in data.get("rotors", [])}
        if set(joints) & rotor_names:
            raise ValueError("Rotor joints cannot be controlled by ArmAPI")
    return ModelSpec(
        ref=ref, model_name=name, sdf_path=sdf, sha256=model_sha,
        joints=normalized["joint_names"], source=kind, base_link=base_link,
        manifest_path=path, total_mass_kg=mass, flight_ready=ready,
        px4_sys_autostart=autostart,
        initial_joint_positions=normalized["initial_joint_positions"],
    )


def _import_manifest_by_ref(ref: str) -> Path | None:
    candidate = Path(ref).expanduser()
    if not candidate.is_absolute():
        candidate = PROJECT_ROOT / candidate
    if candidate.is_dir():
        return candidate / "import_manifest.json"
    if candidate.suffix.lower() == ".json":
        return candidate
    imported = PROJECT_ROOT / "generated" / "imported_models" / ref / "import_manifest.json"
    # Names cannot turn the managed lookup into an arbitrary filesystem search.
    if re.fullmatch(r"amsim_urdf_[A-Za-z0-9_]+", ref) and imported.is_file():
        return imported
    return None

def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()

def _config_by_ref(ref: str) -> Path:
    p = Path(ref)
    if p.suffix in (".yaml", ".yml"):
        if not p.is_absolute():
            p = PROJECT_ROOT / p
        if not p.is_file():
            raise FileNotFoundError(f"model config not found: {p}")
        return p.resolve()

    root = PROJECT_ROOT / "configs" / "models"
    matches = []
    for path in root.glob("*.yaml"):
        data = yaml.safe_load(path.read_text()) or {}
        if ref in {path.stem, str(data.get("name", "")), str(data.get("model_name", ""))}:
            matches.append(path)
    if len(matches) != 1:
        raise KeyError(
            f"Model '{ref}' resolved to {len(matches)} config files: "
            + ", ".join(str(x) for x in matches)
        )
    return matches[0].resolve()

def resolve(ref: str | None) -> ModelSpec:
    if not ref or ref == REVIEWED_NAME:
        sdf = PROJECT_ROOT / "assets" / "models" / REVIEWED_NAME / "model.sdf"
        actual = _sha(sdf)
        if actual != REVIEWED_SHA:
            raise RuntimeError(
                f"reviewed model asset changed: {actual} expected={REVIEWED_SHA}"
            )
        return ModelSpec(
            ref=REVIEWED_NAME,
            model_name=REVIEWED_NAME,
            sdf_path=sdf,
            sha256=actual,
            joints=["arm_joint_1", "arm_joint_2"],
            source="reviewed",
        )

    manifest = _import_manifest_by_ref(ref)
    if manifest is not None:
        return _imported_spec(manifest, ref)
    path = _config_by_ref(ref)
    built = build(path)
    return ModelSpec(
        ref=ref,
        model_name=built.name,
        sdf_path=built.sdf_path,
        sha256=built.sha256,
        joints=built.joints,
        source="generated",
        config_path=path,
        total_arm_mass_kg=built.total_arm_mass_kg,
        manifest_path=built.output_dir / "model_manifest.json",
    )


def ensure_flight_ready(spec: ModelSpec) -> None:
    if not spec.flight_ready or spec.source == "urdf_import":
        raise RuntimeError(
            "This URDF import has no flight profile. Use model-preview, then bind an "
            "explicit PX4 flight profile before start/run/install."
        )


def validate_px4_airframe(spec: ModelSpec) -> Path | None:
    """Require a built airframe for new URDF bindings; retain legacy startup."""
    ensure_flight_ready(spec)
    if spec.source != "urdf_flight":
        return None
    if type(spec.px4_sys_autostart) is not int or spec.px4_sys_autostart <= 0:
        raise ValueError("PX4 airframe must be a positive integer")
    folder = px4_dir() / "build/px4_sitl_default/etc/init.d-posix/airframes"
    matches = sorted(path for path in folder.glob(f"{spec.px4_sys_autostart}_*") if path.is_file())
    if len(matches) != 1:
        raise RuntimeError(
            f"PX4 airframe {spec.px4_sys_autostart} resolved to {len(matches)} built runtime scripts in {folder}. "
            "Configure and build that airframe before starting this model."
        )
    return matches[0]

def validate_sdf(spec: ModelSpec) -> tuple[bool, str]:
    p = subprocess.run(
        ["gz", "sdf", "-k", str(spec.sdf_path)],
        text=True,
        capture_output=True,
        timeout=60,
    )
    text = (p.stdout + p.stderr).strip()
    return p.returncode == 0 and "Valid." in text, text

def install(spec: ModelSpec, *, require_sitl_airframe: bool = True) -> Path:
    ensure_flight_ready(spec)
    if require_sitl_airframe:
        validate_px4_airframe(spec)
    models = (px4_dir() / "Tools" / "simulation" / "gz" / "models").resolve()
    target = models / spec.model_name
    source_dir = spec.sdf_path.parent.resolve()
    if target.is_symlink() or target.resolve().parent != models:
        raise RuntimeError(f"Model install target escapes its configured directory: {target}")

    if spec.source == "reviewed":
        installed_sdf = target / "model.sdf"
        if not installed_sdf.is_file():
            raise RuntimeError(f"reviewed model is not installed in PX4 tree: {target}")
        actual = _sha(installed_sdf)
        if actual != spec.sha256:
            raise RuntimeError(
                f"installed reviewed model hash differs: {actual} expected={spec.sha256}"
            )
        return target

    if not re.fullmatch(r"(?:x500_amsim_|amsim_urdf_)[A-Za-z0-9_]+", spec.model_name):
        raise RuntimeError(f"Refusing to install a model outside the AMSIM namespace: {spec.model_name}")
    if source_dir == target.resolve():
        raise RuntimeError("Model source and installation directory must differ")
    if spec.source in IMPORTED_SOURCES:
        checked = _imported_spec(spec.manifest_path, spec.ref)
        if checked.sha256 != spec.sha256 or checked != spec:
            raise RuntimeError("Imported model changed after resolution")
    for entry in source_dir.rglob("*"):
        if entry.is_symlink():
            raise RuntimeError(f"Refusing to install a model with symbolic links: {entry}")
    ok, detail = validate_sdf(spec)
    if not ok:
        raise RuntimeError("generated SDF validation failed:\n" + detail[-5000:])

    if target.exists():
        for entry in target.rglob("*"):
            if entry.is_symlink() or not entry.resolve().is_relative_to(target.resolve()):
                raise RuntimeError(f"Refusing to replace an installed model with symbolic links: {entry}")
        marker_path = target / INSTALL_MARKER
        if marker_path.is_file() and not marker_path.is_symlink():
            marker = json.loads(marker_path.read_text(encoding="utf-8"))
            if marker.get("owner") != "amsim.modeling.manager" or marker.get("model_name") != spec.model_name or marker.get("schema_version") != 1 or marker.get("source") != spec.source:
                raise RuntimeError(f"Refusing to replace a model without matching AMSIM ownership: {target}")
            old_manifest = target / ("import_manifest.json" if spec.source in IMPORTED_SOURCES else "model_manifest.json")
            if marker.get("model_sha256") != _sha(target / "model.sdf") or not old_manifest.is_file() or marker.get("manifest_sha256") != _sha(old_manifest):
                raise RuntimeError(f"Installed model was modified after AMSIM installation: {target}")
            if spec.source in IMPORTED_SOURCES:
                _imported_spec(target / "import_manifest.json", spec.ref)
        elif spec.source == "generated" and target.name.startswith("x500_amsim_"):
            # Permit existing platform-generated models from before ownership markers.
            old_manifest = target / "model_manifest.json"
            old = json.loads(old_manifest.read_text(encoding="utf-8")) if old_manifest.is_file() and not old_manifest.is_symlink() else {}
            if old.get("model_name") != spec.model_name or old.get("model_sha256") != _sha(target / "model.sdf"):
                raise RuntimeError(f"Existing generated model ownership or hash differs: {target}")
        else:
            raise RuntimeError(f"Refusing to replace an unowned model directory: {target}")
    models.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".amsim-install-", dir=models) as temporary:
        staging_root = Path(temporary).resolve()
        if staging_root.parent != models:
            raise RuntimeError("Model installation staging directory escaped model root")
        staged = staging_root / spec.model_name
        shutil.copytree(source_dir, staged)
        (staged / INSTALL_MARKER).write_text(json.dumps({
            "owner": "amsim.modeling.manager", "schema_version": 1,
            "model_name": spec.model_name, "source": spec.source,
            "model_sha256": spec.sha256,
            "manifest_sha256": _sha(spec.manifest_path) if spec.manifest_path else None,
        }, indent=2) + "\n", encoding="utf-8")
        if _sha(staged / "model.sdf") != spec.sha256:
            raise RuntimeError("Staged model hash differs from source")
        if spec.source in IMPORTED_SOURCES:
            _imported_spec(staged / "import_manifest.json", spec.ref)
        previous = staging_root / "previous"
        if target.exists():
            target.rename(previous)
        try:
            staged.rename(target)
        except BaseException:
            if previous.exists() and not target.exists():
                previous.rename(target)
            raise

    actual = _sha(target / "model.sdf")
    if actual != spec.sha256:
        raise RuntimeError("installed generated model hash mismatch")
    return target

def env_for(spec: ModelSpec, *, require_sitl_airframe: bool = True) -> dict[str, str]:
    ensure_flight_ready(spec)
    if require_sitl_airframe:
        validate_px4_airframe(spec)
    return {
        "AMSIM_MODEL_NAME": spec.model_name,
        "AMSIM_MODEL_SHA256": spec.sha256,
        "AMSIM_MODEL_JOINTS": json.dumps(spec.joints),
        "AMSIM_MODEL_SOURCE": spec.source,
        "AMSIM_MODEL_BASE_LINK": spec.base_link,
        "AMSIM_PX4_SYS_AUTOSTART": str(spec.px4_sys_autostart),
        "AMSIM_MODEL_INITIAL_JOINT_POSITIONS": json.dumps(spec.initial_joint_positions),
        "AMSIM_MODEL_FLIGHT_READY": "1",
    }

def inventory() -> list[dict]:
    rows = [{
        "ref": REVIEWED_NAME,
        "model_name": REVIEWED_NAME,
        "source": "reviewed",
        "links": 2,
        "config": None,
        "flight_ready": True,
    }]
    for row in list_configs():
        rows.append({
            "ref": row["name"],
            "model_name": row["model_name"],
            "source": "generated",
            "links": row["links"],
            "config": row["config"],
            "flight_ready": True,
        })
    imports = PROJECT_ROOT / "generated" / "imported_models"
    for path in sorted(imports.glob("*/import_manifest.json")):
        try:
            spec = _imported_spec(path, path.parent.name)
            rows.append({
                "ref": spec.model_name, "model_name": spec.model_name,
                "source": spec.source, "manifest": str(path), "config": None,
                "flight_ready": spec.flight_ready, "total_mass_kg": spec.total_mass_kg,
                "controlled_joints": spec.joints,
            })
        except (OSError, ValueError, RuntimeError) as exc:
            rows.append({"ref": path.parent.name, "source": "invalid_import", "manifest": str(path), "flight_ready": False, "error": str(exc)})
    return rows
