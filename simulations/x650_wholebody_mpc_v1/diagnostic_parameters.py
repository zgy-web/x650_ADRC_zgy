"""Reversible ground-only parameter override for diagnostics or declared research.

Uses the owned VehicleAPI MAVLink link and the existing raw PARAM_VALUE decoder.
The existing local PX4 client synchronously saves the restored parameter store;
MAV_CMD_PREFLIGHT_STORAGE's early ACCEPTED is not persistence completion.
Pinned PX4 PARAM_SET copies the float wire slot bytes to its typed parameter;
INT32 therefore needs bit encoding, not the numerical float(value) convention.
Research use binds a reviewed operating condition before the first write; it
does not turn this parameter transaction into a flight-stage certificate.
"""
from __future__ import annotations
from copy import deepcopy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import struct
import subprocess
import time
import uuid

PROFILE = {"EKF2_EV_CTRL": 1, "EKF2_EV_NOISE_MD": 1, "EKF2_EVP_NOISE": .01}
# A separate simulation diagnostic: EV position XYZ and body-frame 3D velocity,
# with no GNSS/barometer fusion. Yaw/attitude remain the ordinary IMU/mag EKF.
# Requires source-backed *raw EV* world/local coordinates, not GPS datum mapping.
PRECISION_PROFILE = {"EKF2_EV_CTRL": 7, "EKF2_EV_NOISE_MD": 1, "EKF2_EVP_NOISE": .01,
                     "EKF2_EVV_NOISE": .01, "EKF2_GPS_CTRL": 0, "EKF2_BARO_CTRL": 0,
                     "EKF2_HGT_REF": 3}
TYPES = {"EKF2_EV_CTRL": 6, "EKF2_EV_NOISE_MD": 6, "EKF2_EVP_NOISE": 9,
         "EKF2_EVV_NOISE": 9, "EKF2_GPS_CTRL": 6, "EKF2_BARO_CTRL": 6, "EKF2_HGT_REF": 6}
PROTECTED = ("EKF2_GPS_CTRL", "EKF2_HGT_REF")
# The runtime observer measures base_link, so existing sensor/delay offsets must
# already match that measurement. Do not mutate reboot-required delay or silently
# reinterpret a different point. All are typed REAL32 parameters in pinned PX4.
PRECISION_REQUIRED = {name: 0. for name in ("EKF2_EV_DELAY", "EKF2_EV_POS_X", "EKF2_EV_POS_Y",
    "EKF2_EV_POS_Z", "EKF2_IMU_POS_X", "EKF2_IMU_POS_Y", "EKF2_IMU_POS_Z")}
PID_PARAMETERS = ("MPC_XY_P", "MPC_Z_P", "MPC_XY_VEL_P_ACC", "MPC_Z_VEL_P_ACC")
PID_CANDIDATES = {
    "baseline": dict(zip(PID_PARAMETERS, (.95, 1., 1.8, 4.))),
    "velocity_low": dict(zip(PID_PARAMETERS, (.95, 1., 1.5, 3.2))),
    "velocity_high": dict(zip(PID_PARAMETERS, (.95, 1., 2.2, 4.8))),
    **{f"position_{position}_velocity_{velocity}": dict(zip(PID_PARAMETERS, (*pos, *vel)))
       for position, pos in (("low", (.75, .8)), ("high", (1.2, 1.2)))
       for velocity, vel in (("baseline", (1.8, 4.)), ("low", (1.5, 3.2)), ("high", (2.2, 4.8)))},
}
ROOT = Path(__file__).resolve().parents[2]
PENDING_JOURNAL = ROOT/".amsim/x650_diagnostic_restore_pending.json"


def assert_no_pending_journal(path=None):
    path = Path(path or PENDING_JOURNAL)
    if path.exists():
        raise RuntimeError("Pending X650 diagnostic parameter restoration blocks new flights: "+str(path))


def validate_profile(profile, stage="hover_folded", controller="pid_ff"):
    if profile is None:
        return None
    if stage != "hover_folded" or controller != "pid_ff":
        raise ValueError("External-position diagnostic is limited to hover_folded / pid_ff")
    if isinstance(profile, dict):
        for candidate in (PROFILE, PRECISION_PROFILE):
            if set(profile) == set(candidate) and all(not isinstance(profile[name], bool)
                    and isinstance(profile[name], (float, int)) and math.isfinite(profile[name])
                    and profile[name] == expected for name, expected in candidate.items()):
                return dict(candidate)
    raise ValueError("Diagnostic profile must exactly match one reviewed fixed EKF2 EV profile")


def profile_name(profile):
    normalized = validate_profile(profile)
    if normalized is None:
        return None
    return "ev_precision_xyz_velocity_v1" if normalized == PRECISION_PROFILE else "ev_horizontal_gnss_v1"


def validate_pid_candidate(candidate, stage="hover_folded", controller="pid_ff"):
    """Normalize a finite reviewed catalog; no free gain or limit changes."""
    if candidate is None:
        return None
    if stage != "hover_folded" or controller != "pid_ff":
        raise ValueError("PID tuning candidate is limited to hover_folded / pid_ff")
    if isinstance(candidate, str):
        if candidate in PID_CANDIDATES:
            return dict(PID_CANDIDATES[candidate])
    elif isinstance(candidate, dict):
        for values in PID_CANDIDATES.values():
            if set(candidate) == set(values) and all(not isinstance(candidate[name], bool)
                    and isinstance(candidate[name], (float, int)) and math.isfinite(candidate[name])
                    and candidate[name] == value for name, value in values.items()):
                return dict(values)
    raise ValueError("PID tuning candidate must exactly match one reviewed fixed four-parameter catalog entry")


def pid_candidate_name(candidate):
    values = validate_pid_candidate(candidate)
    return None if values is None else next(name for name, expected in PID_CANDIDATES.items() if values == expected)


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    temporary.replace(path)


def load_parameter_evidence():
    path = ROOT/"simulations/x650_hover_v1/parameter_evidence.py"
    spec = importlib.util.spec_from_file_location("x650_diagnostic_raw_parameter_reader", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def load_reader():
    return load_parameter_evidence().read_parameters


def expected_record(value, kind):
    raw = struct.pack("<i" if kind == 6 else "<f", value)
    return {"value": struct.unpack("<i" if kind == 6 else "<f", raw)[0],
            "param_type": kind, "raw_value_hex": raw.hex()}


def wire_value(record):
    raw = bytes.fromhex(record["raw_value_hex"])
    if len(raw) != 4 or record["param_type"] not in (6, 9):
        raise ValueError("Unsupported diagnostic parameter wire record")
    value = struct.unpack("<f", raw)[0]
    if not math.isfinite(value):
        raise ValueError("Cannot guarantee NaN-bit-preserving PARAM_SET through float encoder")
    return value


def same_record(a, b):
    return (a.get("param_type"), a.get("raw_value_hex")) == (b.get("param_type"), b.get("raw_value_hex"))


class DiagnosticParameterSession:
    def __init__(self, ctx, profile, folder, *, protected_pid_records, reader=None, journal_path=None,
                 storage_saver=None, pid_candidate=None, purpose="diagnostic", operating_condition=None):
        if purpose not in ("diagnostic", "research"):
            raise ValueError("Parameter session purpose must be diagnostic or research")
        if purpose == "diagnostic" and operating_condition is not None:
            raise ValueError("Diagnostic sessions cannot claim a research operating condition")
        self.purpose = purpose
        self.operating_condition = None
        self.operating_condition_sha256 = None
        if purpose == "research":
            # Imported here because the condition contract imports the fixed
            # parameter catalogs from this module. The diagnostic path remains
            # independent of the new commissioning protocol.
            from research_condition import canonical_hash, validate_operating_condition
            self.operating_condition = validate_operating_condition(operating_condition)
            self.operating_condition_sha256 = canonical_hash(self.operating_condition)
        self.ctx, self.profile, self.folder = ctx, validate_profile(profile), Path(folder)
        self.pid_candidate = validate_pid_candidate(pid_candidate)
        if purpose == "research" and (self.profile != PRECISION_PROFILE
                or self.pid_candidate != PID_CANDIDATES["velocity_high"]):
            raise ValueError("Research sessions require the precision EV profile and selected velocity_high PID")
        if self.profile is None and self.pid_candidate is None:
            raise ValueError("A parameter session requires an explicit diagnostic profile or PID tuning candidate")
        self.estimator_parameters = self.profile or {}
        self.profile_id = profile_name(self.profile)
        self.pid_candidate_id = pid_candidate_name(self.pid_candidate)
        self.precision = self.profile == PRECISION_PROFILE
        self.prerequisites = dict(PRECISION_REQUIRED) if self.precision else {}
        self.reader = reader or load_reader()
        self.storage_saver = storage_saver
        self.journal_path = Path(journal_path or PENDING_JOURNAL)
        assert_no_pending_journal(self.journal_path)
        self.pid_records = deepcopy(protected_pid_records)
        if not self.pid_records or any(not name.startswith(("MC_", "MPC_")) or "raw_value_hex" not in value or value.get("param_type") not in (6, 9)
                                       for name, value in self.pid_records.items()):
            raise ValueError("Typed canonical MC/MPC parameter readback is required for diagnostic protection")
        if self.pid_candidate is not None:
            required = set(load_parameter_evidence().CORE_CONTROL_PARAMETERS)
            if not required <= set(self.pid_records):
                raise ValueError("PID tuning requires all 42 canonical control parameter originals: "+", ".join(sorted(required-set(self.pid_records))))
            if any(self.pid_records[name]["param_type"] != 9 for name in self.pid_candidate):
                raise ValueError("Reviewed PID tuning parameters must have REAL32 original readback")
        if purpose == "research":
            required = set(load_parameter_evidence().CORE_CONTROL_PARAMETERS)
            if set(self.pid_records) != required:
                raise ValueError("Research sessions require exactly 42 canonical control originals")
            # Compare the declared condition to the values that the existing
            # typed transaction will actually apply, including float32 wire
            # quantization. Hashing unquantized catalog literals would bind a
            # different controller state from its real PX4 readback.
            expected_pid = {name: float(record["value"]) for name, record in self.pid_records.items()}
            expected_pid.update({name: float(expected_record(value, 9)["value"])
                                 for name, value in self.pid_candidate.items()})
            if canonical_hash(expected_pid) != self.operating_condition["pid_parameters_sha256"]:
                raise ValueError("Research operating condition differs from the effective 42 control parameters")
            if ctx.model.get("model_sha256") != self.operating_condition["model_sha256"]:
                raise ValueError("Research operating condition differs from the physical model SHA-256")
        self.session_id = str(uuid.uuid4())
        self.land_receipt = ctx.node.received.get("land")
        self.land_observed_sim_s = None
        self.original, self.touched = None, False
        self.document = {"kind": ("x650_ground_truth_aided_research_parameter_session" if purpose == "research"
                                  else "x650_ground_truth_aided_diagnostic_parameter_session"), "schema_version": 1,
            "purpose": purpose, "operating_condition": deepcopy(self.operating_condition),
            "operating_condition_sha256": self.operating_condition_sha256,
            "diagnostic_only": purpose == "diagnostic", "eligible_as_stage_gate": False,
            "ground_truth_aided": purpose == "research", "ideal_data": self.profile is not None,
            "aiding_requested": self.profile is not None, "aiding_confirmed": False,
            "formal_performance_claim": False, "profile": self.profile, "profile_name": self.profile_id,
            "simulation_ground_truth_aided": self.profile is not None, "formal_gps_performance_claim": False,
            "hardware_performance_claim": False,
            "pid_tuning_candidate": self.pid_candidate, "pid_tuning_candidate_name": self.pid_candidate_id,
            "canonical_control_originals_count": len(self.pid_records), "mutable_parameters": self.mutable_parameters,
            "tuning_claim": "Finite local candidate comparison only; no claim of global or fully tuned optimum",
            "fixed_world_reference_required": self.precision,
            "protected_parameters": self.protected_names, "read_only_prerequisites": self.prerequisites,
            "required_fusion_flags_before_takeoff": ({"cs_ev_pos": True, "cs_ev_vel": True, "cs_ev_hgt": True,
                "cs_gnss_pos": False, "cs_gnss_vel": False, "cs_gps_hgt": False, "cs_baro_hgt": False}
                if self.precision else ({"cs_ev_pos": True} if self.profile is not None else {})),
            "origin_semantics": ("Raw EV NED observation has [world_y, world_x, -world_z]; with zero EV/IMU offsets, "
                "no GNSS position fusion resets EKF local XY to [world_y, world_x]. EV height reference resets global altitude "
                "to world_z, retaining an existing origin altitude: local_z=ref_alt-world_z. Require live frame bridge checks; "
                "old GPS/world geodetic mapping is not a valid precision-mode target conversion."
                if self.precision else ("GNSS-referenced horizontal EV assistance with estimated EV bias and GNSS noise floor"
                    if self.profile is not None else "Canonical estimator unchanged; GPS-mode PID candidate diagnostic")),
            "applied_verified": False, "restored_verified": False, "write_attempts": [],
            "transport": "Existing owned VehicleAPI.link only; raw-byte PX4 PARAM_SET / PARAM_VALUE",
            "restore_journal_path": str(self.journal_path.resolve()),
            "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
        self.save()

    @property
    def names(self):
        return list(dict.fromkeys([*self.mutable_parameters, *self.protected_names]))

    @property
    def mutable_parameters(self):
        return {**self.estimator_parameters, **(self.pid_candidate or {})}

    @property
    def protected_names(self):
        estimator_protection = tuple(TYPES) if self.pid_candidate is not None else PROTECTED
        return list(dict.fromkeys([*(name for name in estimator_protection if name not in self.estimator_parameters),
                                   *self.prerequisites, *(name for name in self.pid_records if name not in (self.pid_candidate or {}))]))

    def journal(self, status):
        if self.journal_path.exists():
            previous = json.loads(self.journal_path.read_text(encoding="utf-8"))
            if previous.get("session_id") != self.session_id:
                raise RuntimeError("Diagnostic restoration journal belongs to another session")
        self.journal_path.parent.mkdir(parents=True, exist_ok=True)
        atomic_json(self.journal_path, {"kind": "x650_diagnostic_restore_pending", "session_id": self.session_id,
            "purpose": self.purpose, "operating_condition": deepcopy(self.operating_condition),
            "operating_condition_sha256": self.operating_condition_sha256,
            "status": status, "run_directory": str(self.folder.resolve()), "model": deepcopy(self.ctx.model),
            "allowlist": self.mutable_parameters, "profile_name": self.profile_id,
            "pid_tuning_candidate_name": self.pid_candidate_id, "ground_truth_aided": self.profile is not None,
            "original": self.original, "touched": self.touched,
            "diagnostic_only": self.purpose == "diagnostic", "restored_verified": False, "updated_wall_time": time.time()})

    def save(self):
        atomic_json(self.folder/"diagnostic_parameter_session.json", self.document)

    def require_grounded(self, *, new_reports=False):
        node, vehicle = self.ctx.node, self.ctx.vehicle
        before = {key: node.received.get(key, 0.) for key in ("status", "land")}
        def confirmed():
            self.observe_ground_messages(check=not new_reports)
            status, land = node.data.get("status"), node.data.get("land")
            if new_reports:
                # The first pump need not deliver both topics. Wait for fresh
                # receipts; an already reported unsafe state still fails now.
                if ((status is not None and int(status.arming_state) == 2)
                        or (land is not None and not bool(land.landed))):
                    raise RuntimeError("Diagnostic parameter writes are forbidden while armed or airborne")
                if (status is None or land is None
                        or time.monotonic()-node.received.get("status", 0.) > 3.
                        or self.land_observed_sim_s is None
                        or not 0. <= float(node.sim_time_s())-self.land_observed_sim_s <= 3.+1e-9):
                    return False
                return all(node.received.get(key, 0.) > before[key] for key in before)
            if status is None or land is None or time.monotonic()-node.received.get("status", 0.) > 3.:
                raise RuntimeError("Fresh disarmed/landed state is required for diagnostic parameter writes")
            if int(status.arming_state) == 2 or not bool(land.landed):
                raise RuntimeError("Diagnostic parameter writes are forbidden while armed or airborne")
            return not new_reports or all(node.received.get(key, 0.) > before[key] for key in before)
        if new_reports:
            vehicle.wait(confirmed, 180., "new disarmed and landed reports before diagnostic parameter transaction")
        else:
            confirmed()

    def observe_ground_messages(self, *, check=True):
        node = self.ctx.node
        sim_s, receipt = float(node.sim_time_s()), node.received.get("land")
        if receipt is not None and (self.land_receipt is None or receipt > self.land_receipt):
            self.land_receipt, self.land_observed_sim_s = receipt, sim_s
        if check and (self.land_observed_sim_s is None or not 0. <= sim_s-self.land_observed_sim_s <= 3.+1e-9):
            raise RuntimeError("Diagnostic transaction land receipts stalled for more than 3 simulation seconds")

    def read(self, names):
        vehicle = self.ctx.vehicle
        original_pump = vehicle._pump
        def pump():
            original_pump()
            self.observe_ground_messages()
        vehicle._pump = pump
        try:
            values, errors, attempts = self.reader(vehicle, names)
        finally:
            vehicle._pump = original_pump
        if errors or any(name not in values for name in names):
            raise RuntimeError("Diagnostic parameter readback incomplete: "+json.dumps({"errors": errors, "missing": sorted(set(names)-set(values))}))
        for name in names:
            value = values[name]
            if value.get("param_type") not in (6, 9) or "raw_value_hex" not in value:
                raise RuntimeError("Typed raw parameter readback is missing: "+name)
            try:
                raw = bytes.fromhex(value["raw_value_hex"])
                decoded = struct.unpack("<i" if value["param_type"] == 6 else "<f", raw)[0]
            except (ValueError, struct.error) as exc:
                raise RuntimeError("Invalid raw diagnostic parameter record: "+name) from exc
            if decoded != value["value"] or not math.isfinite(decoded):
                raise RuntimeError("Raw/value diagnostic parameter mismatch: "+name)
        return values

    def _save_px4_storage(self):
        """Pinned `param save` exits only after param_save_default(true) returns.

        Commander acknowledges MAV_CMD_PREFLIGHT_STORAGE before its worker
        starts saving. That ACK cannot clear the persistent flight interlock.
        """
        from amsim.paths import px4_dir
        px4 = Path(px4_dir()).resolve()
        executable = px4/"build/px4_sitl_default/bin/px4-param"
        if not executable.is_file():
            raise RuntimeError("Existing synchronous PX4 parameter client missing: "+str(executable))
        log_path = self.folder/"diagnostic_parameter_storage_save.log"
        record = {"args": [str(executable), "save"], "cwd": str(px4),
                  "log": log_path.name, "started_wall_time": time.time(), "timeout_s": 60.,
                  "semantics": "Pinned param.cpp save without path calls do_save_default -> param_save_default(true); exit 0 means synchronous success",
                  "synchronous_default_save": True}
        self.document["restore_storage_save"] = record
        self.save()
        with log_path.open("w", encoding="utf-8") as log:
            process = subprocess.Popen(record["args"], cwd=px4, stdout=log, stderr=subprocess.STDOUT)
            try:
                deadline = time.monotonic()+record["timeout_s"]
                while process.poll() is None:
                    self.ctx.vehicle._pump()
                    self.observe_ground_messages()
                    self.require_grounded()
                    if log_path.stat().st_size > 2_000_000:
                        raise RuntimeError("PX4 restored parameter save exceeded bounded output capture")
                    if time.monotonic() > deadline:
                        raise RuntimeError("PX4 restored parameter save timed out")
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=3.)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=3.)
                record.update(returncode=process.returncode, completed_wall_time=time.time())
                self.save()
        record["log_sha256"] = hashlib.sha256(log_path.read_bytes()).hexdigest()
        self.save()
        if process.returncode != 0:
            raise RuntimeError("PX4 synchronous restored parameter save failed; inspect "+str(log_path))
        return record

    def save_restored_parameters(self):
        """Require synchronous CLI success plus exact typed readback before clear."""
        self.require_grounded()
        self.journal("restored_runtime_verified_storage_save_pending")
        saved = self.storage_saver(self) if self.storage_saver is not None else self._save_px4_storage()
        if saved.get("returncode") != 0 or saved.get("synchronous_default_save") is not True:
            raise RuntimeError("Synchronous PX4 default parameter storage save was not confirmed")
        self.document["restore_storage_save"] = saved
        self.require_grounded()
        after_save = self.read(self.names)
        if any(not same_record(after_save[name], self.original[name]) for name in self.names):
            raise RuntimeError("Parameter values changed during restored-storage save")
        self.document["restore_post_save_readback"] = after_save
        self.document["restore_persistent_storage_verified"] = True
        self.save()

    def write_verified(self, name, desired, phase):
        expected_type = 9 if name in (self.pid_candidate or {}) else TYPES.get(name)
        if name not in self.mutable_parameters or desired["param_type"] != expected_type:
            raise ValueError("Write outside diagnostic parameter/type allowlist")
        last = None
        for attempt in range(1, 4):
            self.require_grounded()
            self.touched = True  # Even a lost ACK can leave a real parameter change.
            entry = {"phase": phase, "name": name, "attempt": attempt,
                     "desired": deepcopy(desired), "started_wall_time": time.time()}
            self.document["write_attempts"].append(entry)
            self.save()
            self.journal(phase+"_write_pending")
            try:
                self.ctx.vehicle.link.mav.param_set_send(*self.ctx.vehicle.target, name.encode("ascii"),
                    wire_value(desired), desired["param_type"])
                actual = self.read([name])[name]
                entry["readback"] = actual
                if not same_record(actual, desired):
                    raise RuntimeError("Diagnostic parameter raw readback differs: "+name)
                entry["verified"] = True
                self.save()
                return actual
            except Exception as exc:
                last = exc
                entry.update(verified=False, error=repr(exc))
                self.save()
        raise RuntimeError("Diagnostic parameter write/readback failed: "+name) from last

    def apply(self):
        self.require_grounded(new_reports=True)
        self.original = self.read(self.names)
        for name in self.mutable_parameters:
            expected_type = 9 if name in (self.pid_candidate or {}) else TYPES[name]
            if self.original[name]["param_type"] != expected_type:
                raise RuntimeError("Original diagnostic parameter type differs: "+name)
            wire_value(self.original[name])  # Prove original can be restored before any write.
        for name, expected in self.prerequisites.items():
            if not same_record(self.original[name], expected_record(expected, 9)):
                raise RuntimeError("Precision EV diagnostic requires reviewed zero sensor/delay offset: "+name)
        for name, expected in self.pid_records.items():
            if not same_record(self.original[name], expected):
                raise RuntimeError("Diagnostic PID protection differs from canonical before snapshot: "+name)
        self.document["before"] = deepcopy(self.original)
        self.save()
        atomic_json(self.folder/"diagnostic_parameters_before.json", self.original)
        self.journal("original_saved_before_any_write")
        try:
            # Configure noise/reference first, remove competing GNSS/barometer
            # sources while disarmed, then enable the reviewed EV mask last.
            order = ("EKF2_EVP_NOISE", "EKF2_EVV_NOISE", "EKF2_EV_NOISE_MD", "EKF2_HGT_REF",
                     "EKF2_GPS_CTRL", "EKF2_BARO_CTRL", "EKF2_EV_CTRL") if self.precision else (
                     ("EKF2_EVP_NOISE", "EKF2_EV_NOISE_MD", "EKF2_EV_CTRL") if self.profile is not None else ())
            order = (*order, *(self.pid_candidate or {}))
            for name in order:
                kind = 9 if name in (self.pid_candidate or {}) else TYPES[name]
                self.write_verified(name, expected_record(self.mutable_parameters[name], kind), "apply")
            current = self.read(self.names)
            for name in self.mutable_parameters:
                kind = 9 if name in (self.pid_candidate or {}) else TYPES[name]
                if not same_record(current[name], expected_record(self.mutable_parameters[name], kind)):
                    raise RuntimeError("Diagnostic profile final verification differs: "+name)
            for name in self.protected_names:
                if not same_record(current[name], self.original[name]):
                    raise RuntimeError("Protected diagnostic/PID parameter changed: "+name)
            self.document.update(applied_verified=True, ground_truth_aided=self.profile is not None, applied=current)
            atomic_json(self.folder/"diagnostic_parameters_applied.json", current)
            self.save()
            self.journal("applied_verified_restore_required")
        except BaseException as exc:
            self.document["apply_error"] = repr(exc)
            self.save()
            raise

    def restore(self):
        if not self.touched and not self.journal_path.exists():
            self.document.update(restoration_needed=False, restored_verified=True)
            self.save()
            return
        try:
            self.require_grounded(new_reports=True)
            failures = {}
            # Precision mode restores ordinary sources/reference before removing
            # EV; legacy mode keeps its original disable-first behavior.
            # Both remain ground-only and attempt every restore after a failure.
            order = ("EKF2_HGT_REF", "EKF2_GPS_CTRL", "EKF2_BARO_CTRL", "EKF2_EVV_NOISE",
                     "EKF2_EV_NOISE_MD", "EKF2_EVP_NOISE", "EKF2_EV_CTRL") if self.precision else (
                     ("EKF2_EV_CTRL", "EKF2_EV_NOISE_MD", "EKF2_EVP_NOISE") if self.profile is not None else ())
            order = (*(self.pid_candidate or {}), *order)
            for name in order:
                try:
                    self.write_verified(name, self.original[name], "restore")
                except Exception as exc:
                    failures[name] = repr(exc)
            current = self.read(self.names)
            mismatches = [name for name in self.names if not same_record(current[name], self.original[name])]
            self.document.update(restored=current, restore_failures=failures, restore_mismatches=mismatches,
                                 runtime_restored_verified=not failures and not mismatches,
                                 restored_verified=not failures and not mismatches)
            atomic_json(self.folder/"diagnostic_parameters_restored.json", current)
            self.save()
            if not self.document["restored_verified"]:
                raise RuntimeError("Diagnostic parameters were not fully restored and verified")
            self.save_restored_parameters()
            journal = json.loads(self.journal_path.read_text(encoding="utf-8"))
            if journal.get("session_id") != self.session_id:
                raise RuntimeError("Refusing to archive another diagnostic session's journal")
            journal.update(status="restored_readback_and_synchronous_storage_save_verified", restored_verified=True, restored=current,
                           storage_save=self.document["restore_storage_save"])
            atomic_json(self.folder/"diagnostic_restore_journal_completed.json", journal)
            self.journal_path.unlink()  # Single owned journal, only after exact restoration verification.
        except BaseException as exc:
            self.document.update(restored_verified=False, restore_error=repr(exc))
            self.save()
            raise
