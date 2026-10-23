"""One-time installed-source provenance, before ROS interfaces are created."""
import hashlib
import json
import os
from pathlib import Path
import sys


def _identity(path):
    path = Path(path).resolve(strict=True)
    content = path.read_bytes()
    return {"path": str(path), "sha256": hashlib.sha256(content).hexdigest(), "bytes": len(content)}


def write_startup_metadata(path, engine):
    path = Path(path)
    if not path.is_absolute():
        raise ValueError("startup_metadata_path must be absolute")
    metadata = engine.metadata()
    root = Path(metadata["repo_root"]).resolve(strict=True)
    sources = {}
    for name, module in sorted(list(sys.modules.items())):
        filename = getattr(module, "__file__", None)
        if filename is None:
            continue
        source = Path(filename).resolve()
        project = False
        try:
            source.relative_to(root)
            project = True
        except ValueError:
            pass
        if (source.suffix == ".py" and source.is_file()
                and (project or name.startswith(("flyarm_mpc_reference.", "x650_wholebody.")))):
            sources[name] = _identity(source)
    predictor = engine.adapter.allocator.actuator_predictor
    declared_native = predictor.metadata()["native_accelerator"]
    if declared_native is None or predictor._native is None:
        raise ValueError("Reference startup requires the explicit loaded native predictor")
    actual_native = _identity(predictor._native._name)
    if (Path(declared_native["path"]).resolve(strict=True) != Path(actual_native["path"])
            or declared_native["sha256"] != actual_native["sha256"]):
        raise ValueError("Actually loaded native predictor differs from its identity")
    optimization = getattr(engine.adapter.allocator, "native_optimization", None)
    actual_optimization = None
    if optimization is not None:
        actual_optimization = _identity(optimization.lib._name)
        if (Path(optimization.receipt["path"]).resolve(strict=True) != Path(actual_optimization["path"])
                or optimization.receipt["sha256"] != actual_optimization["sha256"]):
            raise ValueError("Actually loaded native optimization differs from its identity")
    dependencies = {}
    for name in ("numpy", "scipy"):
        module = sys.modules.get(name)
        if module is None or not getattr(module, "__file__", None):
            raise ValueError("Reference dependency was not loaded before ROS startup: " + name)
        dependencies[name] = {"version": getattr(module, "__version__", None), **_identity(module.__file__)}
    document = {"schema_version": 1, "complete": True,
        "scope": "offline/shadow Python reference process; source bytes observed after actual imports",
        "created_before_ros_publishers_and_subscriptions": True,
        "pid": os.getpid(), "pgid": os.getpgid(0) if hasattr(os, "getpgid") else None,
        "python": {"executable": sys.executable, "version": sys.version},
        "dependencies": dependencies, "engine": metadata,
        "loaded_module_sources": sources, "actually_loaded_native_library": actual_native,
        "actually_loaded_native_optimization": actual_optimization,
        "native_numerical_kernels": actual_optimization is not None,
        "native_mpc": False, "actuator_dispatch": False}
    # Never replace a previous startup report or a failed attempt's evidence.
    with path.open("x", encoding="utf-8") as stream:
        json.dump(document, stream, indent=2, allow_nan=False)
        stream.write("\n")
    return document
