from __future__ import annotations

import hashlib
import json
import shutil
import subprocess
from dataclasses import dataclass
from pathlib import Path

import yaml

from ..paths import PROJECT_ROOT, px4_dir
from .generator import build, list_configs

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
    )

def validate_sdf(spec: ModelSpec) -> tuple[bool, str]:
    p = subprocess.run(
        ["gz", "sdf", "-k", str(spec.sdf_path)],
        text=True,
        capture_output=True,
    )
    text = (p.stdout + p.stderr).strip()
    return p.returncode == 0 and "Valid." in text, text

def install(spec: ModelSpec) -> Path:
    target = px4_dir() / "Tools" / "simulation" / "gz" / "models" / spec.model_name
    source_dir = spec.sdf_path.parent

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

    ok, detail = validate_sdf(spec)
    if not ok:
        raise RuntimeError("generated SDF validation failed:\n" + detail[-5000:])

    tmp = target.with_name(target.name + ".amsim-tmp")
    if tmp.exists():
        shutil.rmtree(tmp)
    shutil.copytree(source_dir, tmp)

    if target.exists():
        # Generated models are owned by this platform namespace only.
        if not target.name.startswith("x500_amsim_"):
            raise RuntimeError(f"refusing to replace non-AMSIM model: {target}")
        shutil.rmtree(target)
    tmp.rename(target)

    actual = _sha(target / "model.sdf")
    if actual != spec.sha256:
        raise RuntimeError("installed generated model hash mismatch")
    return target

def env_for(spec: ModelSpec) -> dict[str, str]:
    return {
        "AMSIM_MODEL_NAME": spec.model_name,
        "AMSIM_MODEL_SHA256": spec.sha256,
        "AMSIM_MODEL_JOINTS": json.dumps(spec.joints),
        "AMSIM_MODEL_SOURCE": spec.source,
    }

def inventory() -> list[dict]:
    rows = [{
        "ref": REVIEWED_NAME,
        "model_name": REVIEWED_NAME,
        "source": "reviewed",
        "links": 2,
        "config": None,
    }]
    for row in list_configs():
        rows.append({
            "ref": row["name"],
            "model_name": row["model_name"],
            "source": "generated",
            "links": row["links"],
            "config": row["config"],
        })
    return rows
