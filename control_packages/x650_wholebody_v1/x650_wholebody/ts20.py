"""Offline TS20-50 reference bridge and hash-pinned external model loader.

This module neither talks to hardware nor changes the accepted MPC model.  The
upstream numerical implementation remains an external local dependency.  Its
command-normalized coefficients are not physical joint inertias or shaft Kt.
"""
from __future__ import annotations

import copy
import hashlib
import importlib.abc
import importlib.util
import json
import math
from pathlib import Path, PurePosixPath
import re
import sys
import uuid

import numpy as np


MANIFEST_SCHEMA = "ts20_external_source_v1"
MODEL_DIRECTORY = "models/ts20_50"
POSITION_STEP_RAD = 2 * math.pi / (2 ** 24)
VELOCITY_STEP_RAD_S = 1 / 512
TORQUE_STEP_NM = 1 / 2048
_REQUIRED = {f"{MODEL_DIRECTORY}/{name}" for name in (
    "__init__.py", "runtime.py", "mechanics.py", "observers.py", "motion.json",
    "reported_outputs.json", "thermal.json", "conditional_candidates.json",
    "provenance.json", "README.md", "requirements.txt", "example.py")}


def _array(value, name):
    raw = np.asarray(value)
    if raw.dtype.kind not in "fiu":
        raise ValueError(f"{name} must contain finite real numbers, not booleans")
    result = np.asarray(raw, dtype=float)
    if not np.isfinite(result).all():
        raise ValueError(f"{name} must contain finite real numbers")
    return result


def _scalar(value, name):
    result = _array(value, name)
    if result.ndim:
        raise ValueError(f"{name} must be a scalar")
    return float(result)


def quantize_half_away(value, step):
    """C ``round`` convention, expressed in physical units; no clipping."""
    values = _array(value, "quantization input")
    step = _scalar(step, "quantization step")
    if step <= 0:
        raise ValueError("quantization step must be positive")
    scaled = values / step
    # Restrict to exactly representable integer counts, not a hardware limit.
    if not np.isfinite(scaled).all() or np.any(np.abs(scaled) > 2 ** 52 - 1):
        raise ValueError("quantization input exceeds exact integer precision")
    return np.copysign(np.floor(np.abs(scaled) + .5), scaled) * step


def build_mit_commands(times, velocity_commands, q0, *, bus_period_s=.005,
                       kp=6., kd=3276 / 32768, qlimits=None, vlimits=None):
    """Sample a two-joint reference on a fixed, relative-time bus grid.

    ``times`` is strictly increasing, ``velocity_commands`` is N-by-2, and q0
    is the reference position at times[0].  Original velocities hold on
    [times[i], times[i+1]); their exact integral defines q_ref at bus times.
    Bus dq_ref is the original velocity active at the send instant and holds
    until the next send.  It is not the derivative of the held bus q_ref.

    qlimits=(lower[2], upper[2]); vlimits is a positive symmetric bound [2].
    Both the unquantized reference and encoded values must satisfy bounds.
    The final reference time need not be a send instant; no short packet tail
    is appended.  Inputs do not supply real-device gain encoding scales.
    """
    t = _array(times, "times")
    velocity = _array(velocity_commands, "velocity_commands")
    initial = _array(q0, "q0")
    period = _scalar(bus_period_s, "bus_period_s")
    p, d = _scalar(kp, "kp"), _scalar(kd, "kd")
    if (t.ndim != 1 or len(t) < 2 or np.any(np.diff(t) <= 0)
            or velocity.shape != (len(t), 2) or initial.shape != (2,)):
        raise ValueError("increasing N times, N-by-2 velocity and q0[2] required")
    if not .001 <= period <= .01 or p < 0 or d < 0:
        raise ValueError("bus period must be 1-10 ms and gains nonnegative")
    duration = float(t[-1] - t[0])
    if not math.isfinite(duration) or duration < period or duration > 120:
        raise ValueError("reference duration must cover a bus interval and be <=120 s")
    # Subtracting a nonzero time origin can make an exact grid end appear a
    # few ulps short (e.g. 1.015 - 1.0).  The final <= comparison still rejects
    # actual grid points beyond the reference end; no fractional tail is made.
    count = int(math.floor(duration / period + 1e-10)) + 1
    if count > 120001:
        raise ValueError("bus sample budget exceeded")
    bus_t = t[0] + np.arange(count) * period
    bus_t = bus_t[bus_t <= t[-1]]
    if len(bus_t) < 2 or np.any(np.diff(bus_t) <= 0):
        raise ValueError("time origin prevents representable bus intervals")
    with np.errstate(over="ignore", invalid="ignore"):
        knots_q = np.vstack((initial, initial + np.cumsum(
            np.diff(t)[:, None] * velocity[:-1], axis=0)))
    if not np.isfinite(knots_q).all():
        raise ValueError("integrated reference is nonfinite")
    source_index = np.searchsorted(t, bus_t, side="right") - 1
    reference_q = knots_q[source_index] + (bus_t - t[source_index])[:, None] * velocity[source_index]
    q_encoded = quantize_half_away(reference_q, POSITION_STEP_RAD)
    v_encoded = quantize_half_away(velocity[source_index], VELOCITY_STEP_RAD_S)
    # Encoding bounds are protocol representation only, never operating limits.
    for values, step, lo, hi, label in (
            (q_encoded, POSITION_STEP_RAD, -(2 ** 31), 2 ** 31 - 1, "position"),
            (v_encoded, VELOCITY_STEP_RAD_S, -(2 ** 15), 2 ** 15 - 1, "velocity")):
        counts = np.rint(values / step)
        if np.any((counts < lo) | (counts > hi)):
            raise ValueError(f"{label} exceeds protocol encoding range")
    q_bounds = None
    if qlimits is not None:
        q_bounds = _array(qlimits, "qlimits")
        if q_bounds.shape != (2, 2) or np.any(q_bounds[0] >= q_bounds[1]):
            raise ValueError("qlimits must be (lower[2], upper[2])")
        # Extrema of each linear reference segment occur at its end points.
        if (np.any(knots_q < q_bounds[0]) or np.any(knots_q > q_bounds[1])
                or np.any(q_encoded < q_bounds[0]) or np.any(q_encoded > q_bounds[1])):
            raise ValueError("position reference or quantized command exceeds qlimits; no clipping")
    v_bounds = None
    if vlimits is not None:
        v_bounds = _array(vlimits, "vlimits")
        if v_bounds.shape != (2,) or np.any(v_bounds <= 0):
            raise ValueError("vlimits must be positive symmetric bounds [2]")
        if np.any(np.abs(velocity) > v_bounds) or np.any(np.abs(v_encoded) > v_bounds):
            raise ValueError("velocity reference or quantized command exceeds vlimits; no clipping")
    zero = np.zeros_like(q_encoded)
    command = {"q_ref": q_encoded, "dq_ref": v_encoded,
               "tau_ff": quantize_half_away(zero, TORQUE_STEP_NM),
               "kp": np.full_like(zero, p), "kd": np.full_like(zero, d)}
    return {"input_time_s": bus_t, "command": command, "metadata": {
        "motor": "TS20-50", "shape": "all command fields are N_bus-by-2",
        "bus_period_s": period, "reference_end_time_s": float(t[-1]),
        "last_send_time_s": float(bus_t[-1]),
        "integrated_reference_end": knots_q[-1].tolist(),
        "reference_integration": "exact original ZOH velocity integral from explicit q0",
        "bus_hold": "q_ref and sampled dq_ref are held independently between bus sends",
        "gain_assumption": "provided historical effective gains assumed; device encoding scales unknown",
        "gain_encoding_applied": False, "hardware_accessed": False,
        "physical_inertia_identified": False, "assembly_load_qualified": False,
        "supply_24v_qualified": False, "nominal_identification_supply_v": 20.,
        "source_initial_dq_assumed": False, "tau_ff_policy": "zero nominal command",
        "quantization": {"rounding": "half away from zero", "position_step_rad": POSITION_STEP_RAD,
                         "velocity_step_rad_s": VELOCITY_STEP_RAD_S, "torque_step_nm": TORQUE_STEP_NM},
        "qlimits_lower_upper": None if q_bounds is None else q_bounds.tolist(),
        "symmetric_vlimits": None if v_bounds is None else v_bounds.tolist(),
        "bounds_clipped": False}}


class _PinnedLoader(importlib.abc.Loader):
    def __init__(self, source, path):
        self.source, self.path = source, path

    def create_module(self, spec):
        return None

    def exec_module(self, module):
        # Compile the bytes that were hashed, never execute an unchecked pyc.
        module.__file__ = str(self.path)
        exec(compile(self.source, str(self.path), "exec"), module.__dict__)


class _PinnedFinder(importlib.abc.MetaPathFinder):
    def __init__(self, alias, root, sources):
        self.alias, self.root, self.sources = alias, root, sources

    def find_spec(self, fullname, path=None, target=None):
        if fullname == self.alias:
            relative, package = f"{MODEL_DIRECTORY}/__init__.py", True
        elif fullname.startswith(self.alias + "."):
            child = fullname[len(self.alias) + 1:].replace(".", "/")
            relative, package = f"{MODEL_DIRECTORY}/{child}.py", False
        else:
            return None
        if relative not in self.sources:
            raise ImportError("Unpinned TS20 submodule requested")
        return importlib.util.spec_from_loader(
            fullname, _PinnedLoader(self.sources[relative], self.root / relative),
            origin=str(self.root / relative), is_package=package)


class FrozenTs20Model:
    """Load only a caller-supplied, SHA-256-pinned local TS20-50 source tree.

    Manifest keys: schema, motor, upstream_commit, files. files maps every .py,
    .json, README.md and requirements.txt in models/ts20_50 to its SHA-256.
    upstream_commit records provenance; actual identity is enforced by bytes,
    without requiring git, network, serial drivers, or device access.
    """
    def __init__(self, source_dir, manifest):
        self.root = Path(source_dir).resolve(strict=True)
        self.manifest = copy.deepcopy(manifest)
        if (not isinstance(manifest, dict) or manifest.get("schema") != MANIFEST_SCHEMA
                or manifest.get("motor") != "TS20-50"
                or not isinstance(manifest.get("upstream_commit"), str)
                or re.fullmatch(r"[0-9a-f]{40}", manifest["upstream_commit"]) is None
                or not isinstance(manifest.get("files"), dict)):
            raise ValueError("TS20-50 manifest schema, source revision and files required")
        if not _REQUIRED.issubset(self.manifest["files"]):
            raise ValueError("TS20 manifest omits required source files")
        sources = self._verified_sources()
        model = json.loads(sources[f"{MODEL_DIRECTORY}/motion.json"])
        if (model.get("motor") != "TS20-50" or model.get("schema") != "ts20_public_motion_v1"
                or model.get("status") != "frozen_conditional_interface_model"):
            raise ValueError("External model identity is not frozen TS20-50")
        self._motion_metadata = copy.deepcopy(model)
        self._alias = "_x650_ts20_" + uuid.uuid4().hex
        finder = _PinnedFinder(self._alias, self.root, sources)
        sys.meta_path.insert(0, finder)
        try:
            spec = finder.find_spec(self._alias)
            module = importlib.util.module_from_spec(spec)
            sys.modules[self._alias] = module
            spec.loader.exec_module(module)
            self._simulate = module.simulate_motion
        except BaseException:
            for name in list(sys.modules):
                if name == self._alias or name.startswith(self._alias + "."):
                    del sys.modules[name]
            raise
        finally:
            sys.meta_path.remove(finder)

    def _verified_sources(self):
        directory = self.root / MODEL_DIRECTORY
        actual = {p.relative_to(self.root).as_posix() for p in directory.rglob("*")
                  if p.is_file() and (p.suffix in {".py", ".json"}
                                      or p.name in {"README.md", "requirements.txt"})}
        if actual != set(self.manifest["files"]):
            raise ValueError("TS20 manifest does not match the complete numerical source tree")
        content = {}
        for relative, expected in self.manifest["files"].items():
            if (not isinstance(relative, str) or "\\" in relative
                    or not relative.startswith(MODEL_DIRECTORY + "/")
                    or any(part in {"..", "."} for part in PurePosixPath(relative).parts)
                    or not isinstance(expected, str)
                    or re.fullmatch(r"[0-9a-f]{64}", expected) is None):
                raise ValueError("Invalid source path or SHA-256 in TS20 manifest")
            path = (self.root / relative).resolve(strict=True)
            if not path.is_relative_to(self.root) or not path.is_file():
                raise ValueError("TS20 source path escapes its dependency root")
            data = path.read_bytes()
            if hashlib.sha256(data).hexdigest() != expected:
                raise ValueError(f"TS20 source identity mismatch: {relative}")
            content[relative] = data
        return content

    def metadata(self):
        return {"source_manifest": copy.deepcopy(self.manifest),
                "motion_model": copy.deepcopy(self._motion_metadata),
                "external_local_dependency": True, "hardware_accessed": False,
                "interface_model_not_physical_joint_inertia": True,
                "supply_24v_qualified": False, "aerial_arm_load_qualified": False,
                "raw_identification_timeseries_published": False}

    def simulate_motion(self, trial, *, max_seconds=30., max_rhs=1000000):
        """Run the external pure numerical reference with explicit initial state."""
        self._verified_sources()
        return self._simulate(trial, max_seconds=max_seconds, max_rhs=max_rhs)
