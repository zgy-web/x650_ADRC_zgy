"""Native X650/two-link commissioning and PID-FF / upper-MPC experiments.

Actual flights execute only through the existing server ScenarioContext.
Parameter capture is read-only. Independent tip metrics come exclusively from
the native Gazebo observer; controller FK values are explicitly diagnostic.
"""
from __future__ import annotations
import csv
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
import traceback
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
PACKAGE = ROOT/"control_packages/x650_wholebody_v1"
for folder in (HERE, PACKAGE):
    if str(folder) not in sys.path:
        sys.path.insert(0, str(folder))

from recording import (JOINTS, FlightRecorder, NativeRecorder, atomic_json, json_value,
                       native_tip_metrics, native_ground_metrics, native_hover_metrics,
                       native_joint_probe_metrics, sha, state_from_node)
from x650_wholebody.api import ControlCommand, EndpointTarget
from x650_wholebody.adapter import ControllerAdapter
from x650_wholebody.baseline import BaselineController
from x650_wholebody.model import ArmModel, tip_world
from diagnostic_parameters import DiagnosticParameterSession, validate_profile, validate_pid_candidate, assert_no_pending_journal, PRECISION_PROFILE
from response_protocol import validate_response_protocol, protocol_velocity, protocol_annotation
from execution_trace import ExecutionTrace, callback_clock_snapshot, evidence_value

STAGES = {"hover_folded", "joint_probe", "response_probe", "tip_hold", "small", "normal", "fast"}


def load_diagnostic_odometry():
    from diagnostic_odometry import DiagnosticOdometry
    return DiagnosticOdometry


def load_parameter_evidence():
    path = ROOT/"simulations/x650_hover_v1/parameter_evidence.py"
    spec = importlib.util.spec_from_file_location("x650_arm_readonly_parameters", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.ParameterEvidence


def resolved(path):
    path = Path(path).expanduser()
    return path.resolve() if path.is_absolute() else (ROOT/path).resolve()


def actual_pid_parameters(evidence):
    source = evidence.snapshots["before"]["parameters"]
    result = {name: value["value"] for name, value in source.items()
              if name.startswith(("MC_", "MPC_"))}
    if not all(name in result and np.isfinite(result[name]) and result[name] > 0 for name in ("MPC_XY_P", "MPC_Z_P")):
        raise RuntimeError("Actual PX4 position-gain readback is incomplete")
    return result


def verify_pid_profile(path, actual):
    if path is None:
        return {"source": "actual PX4 readback; commissioning only", "tuning_claim": False}
    path = resolved(path)
    document = json.loads(path.read_text(encoding="utf-8"))
    parameters = document.get("parameters", document.get("pid_gains"))
    if not isinstance(parameters, dict) or not parameters:
        raise ValueError("PID profile must contain explicit parameters / pid_gains")
    mismatches = {name: {"expected": value, "actual": actual.get(name)} for name, value in parameters.items()
                  if name not in actual or not np.isfinite(value)
                  or not math.isclose(float(actual[name]), float(value), rel_tol=2e-6, abs_tol=1e-7)}
    if mismatches:
        raise RuntimeError("Actual PID does not match selected profile: "+json.dumps(mismatches))
    return {"file": str(path), "sha256": sha(path), "matched_parameter_count": len(parameters), "parameter_writes": False}


def research_annotations(cfg, actual_pid=None):
    """Research flights remain provisional until the finalized ULog is audited."""
    from research_condition import condition_for_task, is_upper_research
    condition = condition_for_task(cfg)
    if condition is None:
        return {}
    digest = hashlib.sha256(json.dumps(condition, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    if actual_pid is not None:
        actual_digest = hashlib.sha256(json.dumps({k: float(v) for k, v in actual_pid.items()},
            sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
        if actual_digest != condition["pid_parameters_sha256"]:
            raise RuntimeError("Research condition differs from all 42 actual PID parameters")
    mixed = cfg.get("mixed_mpc_protocol") is not None
    extra = ({"mixed_mpc_protocol": cfg["mixed_mpc_protocol"],
              "mixed_mpc_protocol_sha256": hashlib.sha256(json.dumps(cfg["mixed_mpc_protocol"], sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest(),
              "acceleration_feedforward_policy": "zero"} if mixed else {})
    return dict(research_profile=cfg["research_profile"], operating_condition=condition,
        operating_condition_sha256=digest, pid_parameters_sha256=condition["pid_parameters_sha256"],
        diagnostic_only=False, ground_truth_aided=True, eligible_as_stage_gate=False,
        acceptance_pending="full_ulog_audit", formal_gps_performance_claim=False,
        formal_hardware_performance_claim=False,
        control_feedback_contract=condition.get("feedback_contract", "PX4 EKF position loop; fixed raw-EV world target; measured joint position/velocity"),
        feedback_contract=condition.get("feedback_contract"),
        upper_command_interface=condition.get("control_interface"),
        upper_allocator_active=is_upper_research(cfg) and (cfg["stage"] == "tip_hold" or mixed),
        truth_used_as_upper_feedback=False if is_upper_research(cfg) else None,
        **protocol_annotation(cfg), **extra)


def formal_gate_evidence(cfg, model_sha256, source_urdf_sha256):
    """Reject caller booleans; require hash-bound completed physical evidence.

    task.preflight performs platform/input validation too. This second check
    runs inside the actual flight process so a launch-only approval cannot
    replace evidence for the model that is currently running.
    """
    from task import validate_stage_gates
    return validate_stage_gates(cfg, {"model_sha256": model_sha256, "source_urdf_sha256": source_urdf_sha256})


def joint_probe_reference(t, duration):
    """Small individual then combined q excursions, C2 at segment boundaries."""
    active = duration-4.
    if active < 12.:
        raise ValueError("Joint probe needs at least 16 simulation seconds")
    segment = active/3.
    if t <= 2. or t >= duration-2.:
        return np.zeros(2), np.zeros(2)
    index = min(2, int((t-2.)/segment))
    s = t-2.-index*segment
    angle, rate = 2*np.pi*s/segment, 2*np.pi/segment
    amplitude = np.array([.08, 0.] if index == 0 else [0., .08] if index == 1 else [.06, -.06])
    return amplitude*np.sin(angle)**3, amplitude*3*np.sin(angle)**2*np.cos(angle)*rate


def response_probe_velocity(t, duration):
    """Recorded commanded +/- velocity pulses, one body translation axis at a time.

    These are control-reference excitations, not external physical disturbances.
    The final interval is reserved for returning to the hover anchor.
    """
    if duration < 18.:
        raise ValueError("Response probe needs at least 18 simulation seconds")
    out = np.zeros(3)
    if 2. <= t < 14.:
        axis, section = int((t-2.)//4), (t-2.) % 4.
        if section < 1.:
            out[axis] = .08 if axis < 2 else .05
        elif 2. <= section < 3.:
            out[axis] = -.08 if axis < 2 else -.05
    return out


class FigureEight:
    """The archived C2 phase law with explicit new-model world target centre."""
    def __init__(self, center, *, amplitude_x, amplitude_z, period_s, cycles, start_s=2., ramp_s=2.):
        self.center = np.asarray(center, float)
        self.amplitude_x, self.amplitude_z = float(amplitude_x), float(amplitude_z)
        self.period_s, self.cycles, self.start_s, self.ramp_s = float(period_s), int(cycles), float(start_s), float(ramp_s)
        self.duration_s = self.cycles*self.period_s+self.ramp_s
        self.end_s = self.start_s+self.duration_s
        if (self.center.shape != (3,) or not np.isfinite(self.center).all()
                or not all(np.isfinite(v) for v in (self.amplitude_x, self.amplitude_z, self.period_s, self.start_s, self.ramp_s))
                or self.period_s <= 0 or self.cycles < 1 or self.start_s < 0 or self.ramp_s <= 0
                or self.ramp_s > self.cycles*self.period_s):
            raise ValueError("Invalid figure-eight reference")

    def sample(self, t):
        s, omega, total = float(t)-self.start_s, 2*np.pi/self.period_s, 2*np.pi*self.cycles
        def ramp(u):
            return 2.5*u**4-3*u**5+u**6, 10*u**3-15*u**4+6*u**5, 30*u*u*(1-u)**2
        if s <= 0:
            phi, speed, accel = 0., 0., 0.
        elif s >= self.duration_s:
            phi, speed, accel = total, 0., 0.
        elif s < self.ramp_s:
            a, b, c = ramp(s/self.ramp_s)
            phi, speed, accel = omega*self.ramp_s*a, omega*b, omega*c/self.ramp_s
        elif s > self.duration_s-self.ramp_s:
            a, b, c = ramp((self.duration_s-s)/self.ramp_s)
            phi, speed, accel = total-omega*self.ramp_s*a, omega*b, -omega*c/self.ramp_s
        else:
            phi, speed, accel = omega*(s-.5*self.ramp_s), omega, 0.
        a, b = self.amplitude_x, self.amplitude_z
        p = self.center+[a*np.sin(phi), 0., b*np.sin(2*phi)]
        tangent = np.array([a*np.cos(phi), 0., 2*b*np.cos(2*phi)])
        curvature = np.array([-a*np.sin(phi), 0., -4*b*np.sin(2*phi)])
        return p, tangent*speed, curvature*speed**2+tangent*accel


class ConstantTarget:
    def __init__(self, center):
        self.center = np.asarray(center, float)
    def sample(self, t):
        return self.center.copy(), np.zeros(3), np.zeros(3)


def build_allocator(cfg, model, pid, trajectory, *, accepted_response=None):
    name = cfg["controller"]
    if cfg.get("mixed_mpc_protocol") is not None:
        from accepted_response_model import load_accepted_response_model
        from mixed_mpc_protocol import controller_config, baseline_config
        binding = load_accepted_response_model(cfg) if accepted_response is None else accepted_response
        # Full evidence hashing occurs before takeoff. During flight only
        # recheck the small selected model, preserving the Offboard send loop.
        if sha(resolved(cfg["response_model"])) != binding["binding"]["model"]["sha256"]:
            raise RuntimeError("Accepted response model changed after runtime admission")
        actual = {k: float(v) for k, v in pid.items() if k.startswith(("MC_", "MPC_"))}
        from research_condition import canonical_hash
        if canonical_hash(actual) != binding["model"]["pid_parameters_sha256"]:
            raise RuntimeError("Mixed response model differs from actual flight PID readback")
        if name == "pid_ff":
            return BaselineController(model, pid, config=baseline_config(cfg["mixed_mpc_protocol"]))
        from x650_wholebody.mpc import MpcController
        return MpcController(model, pid, trajectory=trajectory,
                             config=controller_config(cfg["mixed_mpc_protocol"], binding["model"]))
    if name in {"pid_ff", "pidff", "pid"}:
        return BaselineController(model, pid)
    if name != "mpc":
        raise ValueError("Controller must explicitly be pid_ff or mpc")
    from x650_wholebody.mpc import MpcController
    response = cfg.get("response_model")
    if response is None:
        raise RuntimeError("MPC requires an explicit native-identified response-model JSON")
    response = json.loads(resolved(response).read_text(encoding="utf-8"))
    if response.get("identified") is not True or response.get("passed") is not True:
        raise RuntimeError("MPC response model has not passed independent identification")
    actual = {k: float(v) for k, v in pid.items() if k.startswith(("MC_", "MPC_"))}
    digest = hashlib.sha256(json.dumps(actual, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()
    if response.get("pid_parameters_sha256") != digest:
        raise RuntimeError("MPC response model does not match the current PX4 PID readback")
    config = dict(cfg.get("mpc_config", {}))
    config.update(base_time_constant_s=response["base_time_constant_s"], response_identified=True, failure_policy="raise")
    return MpcController(model, pid, trajectory=trajectory, config=config)


def active_runtime_instance(root, model_sha256):
    state = json.loads((root/".amsim/runtime.json").read_text(encoding="utf-8"))
    runtime = json.loads(Path(state["runtime_meta"]).read_text(encoding="utf-8"))
    if runtime["model_sha256"] != model_sha256 or not isinstance(runtime.get("instance"), str):
        raise RuntimeError("Native recorder runtime instance belongs to another model")
    return runtime["instance"]


def require_periodic_telemetry(node, *, status_watchdog=None, sim_s=None):
    """Keep fast wall-time guards; upper status has its own source-rate guard."""
    now = time.monotonic()
    topics = (("truth", 2.), ("joints", 2.), ("position", 2.))
    if status_watchdog is None:
        topics += (("status", 3.),)  # Preserve the legacy commissioning contract.
    for key, timeout in topics:
        if now-node.received.get(key, 0.) > timeout:
            raise RuntimeError("Stale flight telemetry: "+key)
    if status_watchdog is not None:
        status_watchdog.check(sim_s, wall_now=now)


def receipt_time(node, key):
    """ApiNode._receive records this monotonic clock at the actual callback."""
    stamp = node.received.get(key)
    if stamp is None or not math.isfinite(float(stamp)) or float(stamp) <= 0:
        raise RuntimeError("Missing/invalid telemetry receipt: "+key)
    return float(stamp)


def px4_timestamp_diagnostic(node, key):
    """DDS-converted PX4 epoch values are logged, never used as a clock."""
    value = getattr(node.data.get(key), "timestamp", None)
    try:
        return float(value) if value is not None and math.isfinite(float(value)) else None
    except (TypeError, ValueError, OverflowError):
        return None


def estimated_ground_probe(node, recorder_s):
    """Check inputs on the ground without declaring them upper-control eligible.

    Pinned PX4 EKF/ekf.h::isYawFinalAlignComplete requires in-flight magnetic
    alignment plus one second when mag fusion is active. NAV takeoff retains
    PX4's own checks; the strict estimated adapter is bound only afterwards.
    """
    from estimated_feedback import TOPICS, EV_FLAGS
    if node.data["status"].arming_state == 2 or not node.data["land"].landed:
        raise RuntimeError("Estimated ground probe requires disarmed landed state")
    now, wall = node.estimator_clock_s, time.monotonic()
    if now is None or abs(recorder_s-now) > .05+1e-9 or node.estimator_feedback_error:
        raise RuntimeError("Invalid estimated-feedback ground clock")
    receipts = {}
    for key in (*TOPICS, "clock"):
        if key not in node.data or node.received_sim.get(key) is None:
            raise RuntimeError("Missing ground estimator stream: "+key)
        age, wall_age = now-node.received_sim[key], wall-node.received[key]
        limit = 1.25 if key == "estimator_flags" else .25 if key == "joints" else .10
        wall_limit = 10. if key == "estimator_flags" else 4.
        if not -1e-9 <= age <= limit or not -1e-6 <= wall_age <= wall_limit:
            raise RuntimeError("Stale ground estimator stream: "+key)
        receipts[key] = dict(sequence=node.callback_sequences[key],sim_age_s=age,wall_age_s=wall_age)
    p, flags = node.data["position"], node.data["estimator_flags"]
    for name in ("xy_valid", "z_valid", "v_xy_valid", "v_z_valid"):
        if not getattr(p,name):
            raise RuntimeError("Invalid ground position/velocity: "+name)
    for name, required in {**EV_FLAGS,"cs_tilt_align":True,"cs_yaw_align":True,
                           "cs_inertial_dead_reckoning":False,"cs_wind_dead_reckoning":False}.items():
        if bool(getattr(flags,name)) != required:
            raise RuntimeError("Invalid initial estimator alignment/fusion: "+name)
    q = np.asarray(node.data["attitude"].q,float)
    numbers = [p.x,p.y,p.z,p.vx,p.vy,p.vz,*q,*node.data["angular_velocity"].xyz]
    if q.shape!=(4,) or not np.isfinite(numbers).all() or abs(np.linalg.norm(q)-1)>1e-3:
        raise RuntimeError("Invalid initial attitude/position/rate data")
    return dict(ground_streams_verified=True,control_eligible=False,callbacks=receipts,
        heading_good_for_control=bool(p.heading_good_for_control),
        final_heading_gate="required after NAV takeoff and before any upper-control output",
        source_contract="pinned PX4 EKF/ekf.h::isYawFinalAlignComplete; in-flight mag alignment required")


class LandReceiptWatchdog:
    """Observe callback progress against Gazebo simulation time, not DDS epochs.

    PX4 LandDetector publishes on changes or every 1 HRT second. uXRCE adds
    its *current* time-sync offset when each outgoing message is serialized;
    different topics can therefore carry different offsets in a slow SITL.
    A receipt is observed by every pump, including ground/non-strict pumps.
    The cached receipt present at construction cannot establish liveness.
    This checks topic arrival progress, not source-to-subscriber transport age.
    """
    def __init__(self, node, timeout_sim_s=3.):
        self.timeout_sim_s = float(timeout_sim_s)
        if not math.isfinite(self.timeout_sim_s) or self.timeout_sim_s <= 0:
            raise ValueError("Land receipt timeout must be positive")
        self.receipt = node.received.get("land")
        self.observed_sim_s, self.last_sim_s, self.sequence = None, None, 0

    def observe(self, node, sim_s):
        sim_s = float(sim_s)
        if not math.isfinite(sim_s) or (self.last_sim_s is not None and sim_s < self.last_sim_s-1e-9):
            raise RuntimeError("Land watchdog simulation clock regressed/invalid")
        self.last_sim_s = sim_s
        current = node.received.get("land")
        if current is None:
            return
        current = receipt_time(node, "land")
        if self.receipt is not None and current < self.receipt:
            raise RuntimeError("Land callback receipt monotonic time regressed")
        if self.receipt is None or current > self.receipt:
            if node.data.get("land") is None:
                raise RuntimeError("Land receipt has no corresponding message")
            self.receipt, self.observed_sim_s = current, sim_s
            self.sequence += 1

    def check(self, sim_s):
        if self.observed_sim_s is None:
            raise RuntimeError("No new land callback observed since control-loop initialization")
        age = float(sim_s)-self.observed_sim_s
        if not math.isfinite(age) or age < -1e-9 or age > self.timeout_sim_s+1e-9:
            raise RuntimeError(f"Stale land receipt: {age:.4f} simulation seconds without a new callback")
        return max(0., age)

    def diagnostics(self, node, sim_s):
        land, position = px4_timestamp_diagnostic(node, "land"), px4_timestamp_diagnostic(node, "position")
        return {"land_timestamp_us": land, "position_timestamp_us": position,
                "status_timestamp_us": px4_timestamp_diagnostic(node, "status"),
                "land_epoch_delta_diagnostic_s": None if land is None or position is None else (position-land)*1e-6,
                "land_received_wall_monotonic_s": node.received.get("land"),
                "status_received_wall_monotonic_s": node.received.get("status"),
                "position_received_wall_monotonic_s": node.received.get("position"),
                "land_receipt_sequence": self.sequence, "land_last_observed_sim_s": self.observed_sim_s,
                "land_receipt_age_sim_s": None if self.observed_sim_s is None else float(sim_s)-self.observed_sim_s}


class StatusReceiptWatchdog:
    """Upper-research status liveness at the pinned source's simulation rate.

    Commander publishes vehicle_status on change or every 500 ms of HRT;
    the DDS 5 Hz rate_limit is a ceiling, not a wall-clock frequency promise.
    Three source periods bound callback progress to 1.5 simulation seconds.
    An independent 10 wall-second bound also rejects a stopped status stream.
    Receipt tokens, never DDS epoch timestamps, establish callback progress.
    """
    SOURCE_PERIOD_SIM_S = .5
    DDS_RATE_LIMIT_HZ = 5.
    TIMEOUT_SIM_S = 1.5
    TIMEOUT_WALL_S = 10.

    def __init__(self, node):
        self.receipt = node.received.get("status")
        if self.receipt is not None:
            self.receipt = self._receipt(self.receipt)
        self.observed_sim_s, self.last_sim_s, self.sequence = None, None, 0

    @staticmethod
    def _receipt(value):
        if isinstance(value, (bool, np.bool_)):
            raise RuntimeError("Invalid status callback receipt")
        try:
            value = float(value)
        except (TypeError, ValueError) as exc:
            raise RuntimeError("Missing/invalid status callback receipt") from exc
        if not math.isfinite(value) or value <= 0:
            raise RuntimeError("Missing/invalid status callback receipt")
        return value

    @classmethod
    def contract(cls):
        return {"source_period_sim_s": cls.SOURCE_PERIOD_SIM_S,
                "dds_rate_limit_hz": cls.DDS_RATE_LIMIT_HZ,
                "timeout_sim_s": cls.TIMEOUT_SIM_S, "timeout_wall_s": cls.TIMEOUT_WALL_S,
                "timeout_basis": "Three 2 Hz source periods; separate 10 s wall bound as for low-rate estimator flags",
                "source_contract": "PX4 d6f12ad1c4f70ad3230afd7d86e971421e02fef4 Commander.cpp: 2 Hz or status change; DDS vehicle_status ceiling 5 Hz",
                "receipt_semantics": "New actual ApiNode callback token observed at every pump; initial cache is not liveness; DDS timestamps diagnostic only",
                "clock_scope": "Recorder simulation time for safety supervision only; not upper-control state feedback",
                "transport_age_claimed": False}

    def observe(self, node, sim_s):
        if isinstance(sim_s, (bool, np.bool_)):
            raise RuntimeError("Status watchdog simulation clock regressed/invalid")
        try:
            sim_s = float(sim_s)
        except (TypeError, ValueError) as exc:
            raise RuntimeError("Status watchdog simulation clock regressed/invalid") from exc
        if (not math.isfinite(sim_s) or sim_s < 0
                or (self.last_sim_s is not None and sim_s < self.last_sim_s-1e-9)):
            raise RuntimeError("Status watchdog simulation clock regressed/invalid")
        self.last_sim_s = sim_s
        current = node.received.get("status")
        if current is None:
            if self.receipt is not None:
                raise RuntimeError("Status callback receipt disappeared")
            return
        current = self._receipt(current)
        if node.data.get("status") is None:
            raise RuntimeError("Status callback receipt has no corresponding message")
        if self.receipt is not None and current < self.receipt:
            raise RuntimeError("Status callback receipt monotonic time regressed")
        if self.receipt is None or current > self.receipt:
            self.receipt, self.observed_sim_s = current, sim_s
            self.sequence += 1

    def check(self, sim_s, *, wall_now=None):
        if self.observed_sim_s is None:
            raise RuntimeError("No new status callback observed since control-loop initialization")
        if isinstance(sim_s, (bool, np.bool_)):
            raise RuntimeError("Invalid status watchdog simulation time")
        try:
            age = float(sim_s)-self.observed_sim_s
        except (TypeError, ValueError) as exc:
            raise RuntimeError("Invalid status watchdog simulation time") from exc
        if not math.isfinite(age) or age < -1e-9 or age > self.TIMEOUT_SIM_S+1e-9:
            raise RuntimeError(f"Stale status receipt: {age:.4f} simulation seconds without a new callback")
        wall_now = time.monotonic() if wall_now is None else wall_now
        wall_age = self._receipt(wall_now)-self.receipt
        if wall_age < -1e-6 or wall_age > self.TIMEOUT_WALL_S:
            raise RuntimeError(f"Stale status receipt: {wall_age:.4f} wall seconds or future callback")
        return max(0., age)

    def diagnostics(self, sim_s, *, wall_now=None):
        wall_now = time.monotonic() if wall_now is None else wall_now
        return {"status_receipt_sequence": self.sequence,
                "status_last_observed_sim_s": self.observed_sim_s,
                "status_receipt_age_sim_s": None if self.observed_sim_s is None else float(sim_s)-self.observed_sim_s,
                "status_receipt_age_wall_s": None if self.receipt is None else float(wall_now)-self.receipt,
                "status_source_period_sim_s": self.SOURCE_PERIOD_SIM_S,
                "status_dds_rate_limit_hz": self.DDS_RATE_LIMIT_HZ,
                "status_receipt_timeout_sim_s": self.TIMEOUT_SIM_S,
                "status_receipt_timeout_wall_s": self.TIMEOUT_WALL_S}


def transition_marker(node):
    land_receipt, status_receipt = receipt_time(node, "land"), receipt_time(node, "status")
    if node.data.get("land") is None or node.data.get("status") is None:
        raise RuntimeError("Missing land/status state at command boundary")
    return {"command_boundary_wall_monotonic_s": time.monotonic(),
            "land_received_wall_monotonic_s": land_receipt, "status_received_wall_monotonic_s": status_receipt,
            "land_timestamp_us": px4_timestamp_diagnostic(node, "land"),
            "status_timestamp_us": px4_timestamp_diagnostic(node, "status"),
            "landed_before": bool(node.data["land"].landed),
            "boundary_semantics": "immediately before VehicleAPI takeoff/land operation; raw DDS epoch stamps diagnostic only"}


def perform_nav_operation(ctx, nav_command, operation):
    """Mark the actual NAV send without changing the shared VehicleAPI.

    A bad marker never suppresses a NAV send, particularly during recovery.
    The public operation completes normally before its marker error is returned
    to the caller for failed certification. The original send method is restored
    even when the operation itself raises. The latest retry defines the boundary.
    """
    original_send = ctx.vehicle._send_command
    marker, marker_error = None, None
    def send(command, params):
        nonlocal marker, marker_error
        candidate = None
        if command == nav_command:
            try:
                candidate = transition_marker(ctx.node)
            except BaseException as exc:
                marker_error = marker_error or exc
        original_send(command, params)
        if command == nav_command:
            marker = candidate
            if marker is not None:
                marker.update(nav_command=int(command), nav_send_completed=True,
                    boundary_semantics="immediately before actual NAV command_long_send; raw DDS epoch stamps diagnostic only")
    ctx.vehicle._send_command = send
    try:
        operation()
    finally:
        ctx.vehicle._send_command = original_send
    return marker, marker_error


def confirm_land_state_after_command(ctx, marker, *, landed, watchdog, sim_clock, timeout_s=180., status_watchdog=None):
    """Require both callbacks after the actual NAV boundary, matching state."""
    accepted = {}
    def confirmed():
        sim = sim_clock()
        require_periodic_telemetry(ctx.node, status_watchdog=status_watchdog, sim_s=sim)
        age = watchdog.check(sim)
        status = ctx.node.data["status"]
        land_receipt, status_receipt = receipt_time(ctx.node, "land"), receipt_time(ctx.node, "status")
        boundary = marker["command_boundary_wall_monotonic_s"]
        if (land_receipt <= max(boundary, marker["land_received_wall_monotonic_s"])
                or status_receipt <= max(boundary, marker["status_received_wall_monotonic_s"])
                or bool(ctx.node.data["land"].landed) != landed
                or (int(status.arming_state) == 2) == landed):
            return False
        accepted.update(watchdog.diagnostics(ctx.node, sim_clock()),
                        land_received_wall_monotonic_s=land_receipt, status_received_wall_monotonic_s=status_receipt,
                        landed=landed, armed=int(status.arming_state) == 2, land_receipt_age_sim_s=age,
                        requires_new_report_after_command=True,
                        new_report_semantics="actual ApiNode callback reception after actual NAV command send boundary")
        if status_watchdog is not None:
            accepted.update(status_watchdog.diagnostics(sim))
        return True
    ctx.vehicle.wait(confirmed, timeout_s, "post-command "+("landed/disarmed" if landed else "airborne/armed")+" state transition")
    return {"before": marker, "after": accepted}


class ControlLoop:
    """Pump the existing ROS/MAVLink connection, then command measured joints."""
    def __init__(self, ctx, cfg, model, recorder, pid, out, *, extension=None):
        self.ctx, self.cfg, self.model, self.recorder, self.pid = ctx, cfg, model, recorder, pid
        self.extension = extension
        self.original_pump = ctx.vehicle._pump
        self.last, self.next_control, self.last_truth, self.last_joint = None, -math.inf, None, None
        self.land_watchdog = LandReceiptWatchdog(ctx.node)
        self.task_start, self.controller, self.trajectory = None, None, None
        self.anchor, self.yaw_ned = None, None
        self.fixed_reference = None
        from research_condition import is_upper_research
        self.upper_research = is_upper_research(cfg)
        self.status_watchdog = StatusReceiptWatchdog(ctx.node) if self.upper_research else None
        self.response_protocol = validate_response_protocol(cfg.get("response_probe_protocol"))
        self.feedback_adapter, self.feedback_diagnostics, self.evaluation_state = None, None, None
        self.step_references = {}
        self.step_send_events, self.last_step_index = [], None
        self.offboard, self.send_reference, self.recovery, self.strict = False, False, False, False
        self.rows, self.max_joint_cmd = 0, np.zeros(2)
        self.control = (out/"control_trace.csv").open("w", newline="", encoding="utf-8")
        self.writer = None
        self.diagnostic = (out/"controller_diagnostics.jsonl").open("w", encoding="utf-8")
        self.feedback_stream = (out/"feedback_trace.jsonl").open("w", encoding="utf-8") if self.upper_research else None
        self.execution_trace = ExecutionTrace(out)
        self.sent_command_history = []

    def disable_upper_feedback(self):
        """End the upper experiment before recovery; never substitute truth into it."""
        self.send_reference, self.controller = False, None
        self.feedback_adapter, self.feedback_diagnostics = None, None

    def control_now(self):
        """One explicit clock for upper scheduling, snapshots and send traces."""
        if self.feedback_adapter is not None:
            value = self.ctx.node.estimator_clock_s
            if value is None or not math.isfinite(value):
                raise RuntimeError("Missing estimated-feedback control clock")
            return float(value)
        return self.recorder.now()

    def read(self):
        state, joint_stamp = state_from_node(self.ctx.node)
        sim = self.recorder.now()
        self.land_watchdog.observe(self.ctx.node, sim)
        if self.status_watchdog is not None:
            self.status_watchdog.observe(self.ctx.node, sim)
        if self.strict:
            self.recorder.check()
            require_periodic_telemetry(self.ctx.node, status_watchdog=self.status_watchdog, sim_s=sim)
            self.land_watchdog.check(sim)
            if not -.001 <= sim-state.stamp_s <= .25 or not -.001 <= sim-joint_stamp <= .25:
                raise RuntimeError("Truth or joint measurement age exceeds 0.25 simulation seconds")
            if self.last_truth is not None and state.stamp_s < self.last_truth-1e-9:
                raise RuntimeError("Truth measurement time regressed")
            if self.last_joint is not None and joint_stamp < self.last_joint-1e-9:
                raise RuntimeError("Joint measurement time regressed")
            if np.any(state.q < np.array(self.model.joint_lower)-.01) or np.any(state.q > np.array(self.model.joint_upper)+.01):
                raise RuntimeError("Physical joint envelope exceeded")
            tilt = math.acos(float(np.clip(state.rotation[2, 2], -1., 1.)))
            if tilt > math.radians(45.):
                raise RuntimeError("Aircraft tilt exceeded 45 degrees")
            status = self.ctx.node.data["status"]
            if status.failsafe or (self.offboard and status.nav_state != 14):
                raise RuntimeError("PX4 failsafe or unexpected offboard exit")
            if self.anchor is not None and np.linalg.norm(state.position-self.anchor) > 1.5:
                raise RuntimeError("Task aircraft position exceeded 1.5 m hover envelope")
        self.last_truth, self.last_joint = state.stamp_s, joint_stamp
        self.evaluation_state = state
        if self.feedback_adapter is not None:
            clock_s = self.ctx.node.estimator_clock_s
            if clock_s is None or abs(sim-clock_s) > .05+1e-9:
                raise RuntimeError("Recorder and estimated-feedback /clock progress differ by over 0.05 s")
            state, joint_stamp, self.feedback_diagnostics = self.feedback_adapter.read(self.ctx.node, clock_s)
            sim = clock_s
        if self.extension is not None:
            state, joint_stamp = self.extension.after_state_read(self, state, joint_stamp, sim)
        return sim, state, joint_stamp

    def pump(self):
        from pymavlink import mavutil
        self.original_pump()
        for _ in range(32 if self.upper_research else 6):
            self.ctx.node.spin_once(timeout_sec=0.)
        sim = self.recorder.now()
        if self.extension is not None:
            self.extension.on_pump(self, sim)
        self.land_watchdog.observe(self.ctx.node, sim)
        if self.status_watchdog is not None:
            self.status_watchdog.observe(self.ctx.node, sim)
            if self.strict:
                self.status_watchdog.check(sim)
        if self.control_now() < self.next_control-1e-9:
            return
        sim, measured, joint_stamp = self.read()
        dt = self.cfg["control_period_s"] if self.last is None else sim-self.last
        if dt <= 0:
            return
        if self.strict and dt > .05+1e-10:
            raise RuntimeError(f"Control interval {dt:.4f}s exceeds 0.05s")
        begin = time.perf_counter()
        compute_before = callback_clock_snapshot(self.ctx.node)
        t = -1. if self.task_start is None else sim-self.task_start
        q_ref, qd_ref = np.zeros(2), np.zeros(2)
        command = None
        if self.task_start is not None and self.cfg["stage"] == "joint_probe" and not self.recovery:
            q_ref, qd_ref = joint_probe_reference(t, self.cfg["duration_s"])
        joint = np.clip(qd_ref+3.*(q_ref-measured.q), [-.15, -.15], [.15, .15])
        if self.controller is not None and self.task_start is not None and not self.recovery:
            desired = EndpointTarget(*self.trajectory.sample(t))
            self.recorder.tip_target = desired.position.copy()
            command = self.execution_trace.controller_step(self.controller, t, dt, measured, desired,
                feedback=self.feedback_diagnostics, phase=self.recorder.phase,
                clock_snapshot=lambda: callback_clock_snapshot(self.ctx.node))
            joint = command.joint_velocity
        elif self.send_reference and self.anchor is not None and (self.fixed_reference is None or self.upper_research):
            if self.upper_research and self.feedback_adapter is None:
                raise RuntimeError("Upper PVA output requires validated PX4 estimated feedback")
            velocity = np.clip(1.5*(self.anchor-measured.position), -.12, .12)
            if self.task_start is not None and self.cfg["stage"] == "response_probe" and not self.recovery:
                if self.response_protocol is not None:
                    if t < self.response_protocol["return_anchor_start_s"]:
                        velocity = np.asarray(protocol_velocity(t, self.response_protocol), float)
                elif t < 14.:
                    velocity = response_probe_velocity(t, self.cfg["duration_s"])
            command = ControlCommand.from_executed_velocity(measured, velocity, joint,
                nominal_position=self.anchor, acceleration_feedforward=np.zeros(3),
                position_gains=[self.pid["MPC_XY_P"], self.pid["MPC_XY_P"], self.pid["MPC_Z_P"]],
                diagnostics={"response_excitation": "commanded velocity pulse; no external force"})
        compute_after = callback_clock_snapshot(self.ctx.node)
        publication_context = {"sim_s": sim, "t_s": t, "phase": self.recorder.phase,
            "control_dt_s": dt, "measured_joint_stamp_s": joint_stamp,
            "compute_before": compute_before, "compute_after": compute_after}
        if np.any(abs(joint) > np.array([.45, .35])+1e-8):
            raise RuntimeError("Executed joint command exceeds common speed limits")
        if self.extension is not None:
            publication_context["pre_publish_guard"] = self.extension.before_publish(self, begin, sim, command)
        for name, value in zip(JOINTS, joint):
            self.execution_trace.publish("joint:"+name, {"joint": name, "velocity_rad_s": float(value)},
                lambda name=name, value=value: self.ctx.arm.set_velocity(name, float(value)),
                clock_snapshot=lambda: callback_clock_snapshot(self.ctx.node), context=publication_context)
        self.max_joint_cmd = np.maximum(self.max_joint_cmd, abs(joint))
        if self.send_reference and self.fixed_reference is not None and not self.upper_research:
            reference = self.fixed_reference
            step_index = None
            if self.step_references and self.task_start is not None and not self.recovery:
                from position_step_probe import segment_at
                segment = segment_at(t)
                if segment is not None:
                    reference = self.step_references[segment["segment_index"]]
                    step_index = segment["segment_index"]
            reference.check(self.ctx.node.data["position"])
            fixed_sp = reference.setpoint()
            self.recorder.target_body = np.asarray(reference.document["target_world_enu_m"], float)
            self.execution_trace.publish("px4_pva", fixed_sp,
                lambda: self.ctx.vehicle.link.mav.set_position_target_local_ned_send(int(sim*1000)&0xffffffff,
                    *self.ctx.vehicle.target, fixed_sp["coordinate_frame"], fixed_sp["type_mask"],
                    *fixed_sp["position"], *fixed_sp["velocity"], *fixed_sp["acceleration"], fixed_sp["yaw"], fixed_sp["yaw_rate"]),
                clock_snapshot=lambda: callback_clock_snapshot(self.ctx.node), context=publication_context)
            if step_index != self.last_step_index:
                self.step_send_events.append({"segment_index": step_index, "first_send_sim_s": sim,
                    "elapsed_s": t, "position_target_local_ned_m": fixed_sp["position"],
                    "target_world_enu_m": reference.document["target_world_enu_m"]})
                self.last_step_index = step_index
        else:
            fixed_sp = None
        sp = None
        if command is not None:
            if fixed_sp is not None:
                raise RuntimeError("Fixed-position and upper-controller outputs must be mutually exclusive")
            if self.upper_research:
                from estimated_feedback import encode_world_command
                if self.feedback_adapter is None:
                    raise RuntimeError("Upper command has no validated feedback snapshot")
                sp = encode_world_command(command, self.fixed_reference)
            else:
                estimate = self.ctx.node.data["position"]
                sp = command.px4_setpoint([estimate.x, estimate.y, estimate.z], [estimate.vx, estimate.vy, estimate.vz])
            self.execution_trace.publish("px4_pva", {"coordinate_frame": mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                "type_mask": sp.type_mask, "position": sp.position, "velocity": sp.velocity,
                "acceleration": sp.acceleration, "yaw": self.yaw_ned, "yaw_rate": 0.},
                lambda: self.ctx.vehicle.link.mav.set_position_target_local_ned_send(int(sim*1000)&0xffffffff,
                    *self.ctx.vehicle.target, mavutil.mavlink.MAV_FRAME_LOCAL_NED, sp.type_mask,
                    *sp.position, *sp.velocity, *sp.acceleration, self.yaw_ned, 0.),
                clock_snapshot=lambda: callback_clock_snapshot(self.ctx.node), context=publication_context)
            # Append only after both joint publishes and the PVA send returned.
            # Partial publication exceptions use ExecutionTrace and recovery.
            self.sent_command_history.append((float(sim), np.r_[command.base_velocity_target, joint].copy()))
            self.sent_command_history = [(when, value) for when, value in self.sent_command_history
                                         if when >= sim-.75]
        truth = self.evaluation_state
        row = dict(sim_s=sim, t_s=t, phase=self.recorder.phase, truth_stamp_s=truth.stamp_s,
                   truth_age_s=sim-truth.stamp_s, joint_stamp_s=joint_stamp, joint_age_s=sim-joint_stamp,
                   control_dt_s=dt, q1=measured.q[0], q2=measured.q[1], qd1=measured.dq[0], qd2=measured.dq[1],
                   qd1_cmd=float(joint[0]), qd2_cmd=float(joint[1]),
                   **self.land_watchdog.diagnostics(self.ctx.node, sim))
        for boundary, observed in (("before", compute_before), ("after", compute_after)):
            for field in ("wall_monotonic_s", "processed_estimator_clock_s", "joint_header_stamp_s"):
                row["compute_"+boundary+"_"+field] = observed[field]
        for prefix, values in (("base_p", truth.position), ("base_v", truth.velocity), ("body_w", truth.omega_body),
                               ("base_velocity_target_", np.zeros(3) if command is None else command.base_velocity_target),
                               ("command_acceleration_", np.zeros(3) if command is None else command.acceleration_feedforward)):
            row.update({prefix+axis: float(value) for axis, value in zip("xyz", values)})
        row["compute_ms"] = (time.perf_counter()-begin)*1000.
        row["fixed_world_position_sent"] = fixed_sp is not None
        for axis, value in zip("ned", [math.nan]*3 if fixed_sp is None else fixed_sp["position"]):
            row["fixed_position_target_"+axis] = value
        if self.upper_research:
            feedback = self.feedback_diagnostics
            row.update(self.status_watchdog.diagnostics(self.recorder.now()))
            row.update(pva_sent=sp is not None, sent_coordinate_frame=1 if sp is not None else None,
                       sent_type_mask=None if sp is None else sp.type_mask,
                       sent_yaw_ned=None if sp is None else self.yaw_ned,
                       feedback_state_stamp_s=None if feedback is None else measured.stamp_s,
                       feedback_snapshot_sequence=None if feedback is None else feedback["callbacks"]["position"]["sequence"])
            for field in ("position", "velocity", "acceleration"):
                for axis, value in zip("ned", [math.nan]*3 if sp is None else getattr(sp, field)):
                    row["sent_"+field+"_target_"+axis] = value
            for prefix, key in (("feedback_position_", "position_ned_m"), ("feedback_velocity_", "velocity_ned_m_s")):
                values = [math.nan]*3 if feedback is None else feedback["px4_snapshot"][key]
                row.update({prefix+axis: float(value) for axis, value in zip("ned", values)})
            for prefix, values in (("feedback_p", measured.position), ("feedback_v", measured.velocity), ("feedback_omega_", measured.omega_body)):
                row.update({prefix+axis: float(value) if feedback is not None else math.nan for axis, value in zip("xyz", values)})
            if self.cfg.get("mixed_mpc_protocol") is not None or self.extension is not None:
                acceleration = measured.acceleration_enu if feedback is not None else [math.nan]*3
                row.update({"feedback_a"+axis: float(value) for axis, value in zip("xyz", acceleration)})
                source = {} if feedback is None else feedback["acceleration_source"]
                row.update(feedback_acceleration_position_stamp_s=source.get("received_sim_s"),
                           feedback_acceleration_position_sequence=source.get("callback_sequence"))
            self.feedback_stream.write(json.dumps(json_value({"sim_s": sim, "t_s": t,
                "phase": self.recorder.phase, "feedback": feedback}), allow_nan=False)+"\n")
        if self.writer is None:
            self.writer = csv.DictWriter(self.control, fieldnames=list(row))
            self.writer.writeheader()
        self.writer.writerow(row)
        if command is not None:
            diagnostic_row = {"sim_s": sim, "t_s": t, "diagnostics": command.diagnostics}
            if self.cfg.get("mixed_mpc_protocol") is not None or self.extension is not None:
                diagnostic_row["phase"] = self.recorder.phase
            self.diagnostic.write(json.dumps(json_value(diagnostic_row), allow_nan=False)+"\n")
        if self.rows % 50 == 0:
            self.control.flush()
            self.diagnostic.flush()
            if self.feedback_stream is not None:
                self.feedback_stream.flush()
        self.rows += 1
        self.last, self.next_control = sim, sim+self.cfg["control_period_s"]

    def close(self, *, experiment_complete=None):
        self.ctx.vehicle._pump = self.original_pump
        self.control.close()
        self.diagnostic.close()
        if self.feedback_stream is not None:
            self.feedback_stream.close()
        return self.execution_trace.close(experiment_complete=experiment_complete)


def wait_sim(ctx, loop, duration, description):
    start = loop.control_now()
    ctx.vehicle.wait(lambda: loop.control_now()-start >= duration, max(45., duration*12.), description)


def activate_prediction_chain(ctx, loop, cfg, meta, reference_rotation):
    """Hand over immediately after an actual prestream publication completes.

    The preparation/launch evidence already exists. Keep activation metadata in
    memory for the existing completion/exception writers: synchronous metadata
    writes or console flushing here could interrupt even the prestream. The
    scheduler and actual previous send time remain intact, and an overlong
    interval still fails the ordinary 50 ms guard.
    """
    if (loop.task_start is not None or loop.controller is None or loop.recovery
            or not loop.send_reference or not loop.sent_command_history):
        raise RuntimeError("Prediction-chain activation requires an inactive prepared controller and actual prestream history")
    meta.update(phase="activation_prepared", reference_base_enu_m=loop.anchor,
                reference_rotation=reference_rotation, requested_duration_s=cfg["duration_s"],
                activation_metadata_persistence="Existing preparation/launch evidence retained; activation held in memory until completion or exception, with no synchronous handoff metadata or console writes")
    previous_send = loop.sent_command_history[-1][0]
    # A single pump can consume callbacks without reaching next_control. Wait
    # for a new successfully published joint/PVA batch, not merely new /clock.
    ctx.vehicle.wait(lambda: bool(loop.sent_command_history)
        and loop.sent_command_history[-1][0] > previous_send, 8., "fresh actual prestream command before controller activation")
    history = loop.sent_command_history
    start = float(history[-1][0])
    if (loop.last is None or abs(loop.last-start) > 1e-9
            or abs(loop.control_now()-start) > 1e-9
            or abs(loop.next_control-start-cfg["control_period_s"]) > 1e-9):
        raise RuntimeError("Prediction-chain activation lost the actual send/scheduler boundary")
    if cfg["controller"] == "mpc":
        loop.controller.allocator.prime_command_history(history, start)
    elif cfg["controller"] == "pid_ff":
        loop.controller.allocator.previous_command = history[-1][1].copy()
    else:
        raise RuntimeError("Unknown prediction-chain comparison controller")
    meta["activation_command_history"] = [
        {"control_time_s": when, "task_time_s": when-start, "command": value.tolist()}
        for when, value in history]
    loop.task_start = start
    loop.recorder.phase = "measure"
    meta.update(phase="measure", task_start_sim_s=start)
    return start


def restore_folded(ctx, loop):
    loop.recovery, loop.controller, loop.task_start = True, None, None
    loop.recorder.phase = "restore_folded"
    stable = [None]
    def folded():
        _, state, _ = loop.read()
        good = np.max(abs(state.q)) < .006 and np.max(abs(state.dq)) < .02
        stable[0] = (loop.recorder.now() if stable[0] is None else stable[0]) if good else None
        return stable[0] is not None and loop.recorder.now()-stable[0] > .3
    ctx.vehicle.wait(folded, 90., "folded q0 before landing")


def ground_gate(ctx, loop, native, cfg, model_sha256, source_sha256, out):
    """Verify native ground stability before any arming/takeoff call."""
    loop.recorder.phase = "ground_ready"
    begin = loop.recorder.now()
    wait_sim(ctx, loop, 5.5, "independent 5 s ground stability capture")
    interval = (begin, begin+5.)
    manifest = native.freeze(interval[1], model_sha256=model_sha256, source_urdf_sha256=source_sha256,
                             complete=True, task_window=interval, filename="native_ground.csv")
    metrics = native_ground_metrics(out/"native_ground.csv", interval)
    evidence = dict(schema_version=1, kind="x650_native_stage_evidence", stage="native_ground", complete=True,
                    simulation_backend="gazebo_px4", model_sha256=model_sha256, source_urdf_sha256=source_sha256,
                    preparation_report_sha256=sha(resolved(cfg["preparation_report"])),
                    interval_sim_s=interval, **metrics,
                    artifacts={"native_ground.csv": manifest["sha256"],
                               "native_ground_manifest.json": sha(out/"native_ground_manifest.json")})
    if any(cfg.get(k) is not None for k in ("diagnostic_profile", "pid_tuning_candidate", "precision_test_profile")):
        evidence.update(kind="x650_diagnostic_ground_observation", diagnostic_only=True,
                        ground_truth_aided=cfg.get("diagnostic_profile") is not None, eligible_as_stage_gate=False, formal_performance_claim=False)
    if cfg.get("research_profile") is not None:
        evidence.update(research_annotations(cfg, loop.pid))
    atomic_json(out/"native_ground.json", evidence)
    if not evidence["passed"]:
        raise RuntimeError("Independent native ground-stability gate failed; takeoff refused")
    return {"path": str(out/"native_ground.json"), "sha256": sha(out/"native_ground.json"), **metrics}


def execute(cfg, model_spec, provenance, out, *, extension=None):
    from amsim import ScenarioContext
    from amsim.modeling.manager import validate_px4_airframe
    from pymavlink import mavutil
    out = Path(out)
    if extension is not None:
        extension.validate_runtime(cfg, model_spec, provenance, out)
    assert_no_pending_journal()
    diagnostic_profile = validate_profile(cfg.get("diagnostic_profile"), cfg["stage"], cfg["controller"])
    pid_candidate = validate_pid_candidate(cfg.get("pid_tuning_candidate"), cfg["stage"], cfg["controller"])
    diagnostic_run = diagnostic_profile is not None or pid_candidate is not None or cfg.get("precision_test_profile") is not None
    research_run = cfg.get("research_profile") is not None
    from research_condition import is_upper_research
    upper_research = is_upper_research(cfg)
    research = research_annotations(cfg)
    if research_run:
        from research_condition import research_runtime_parameters
        diagnostic_profile, pid_candidate = research_runtime_parameters(cfg)
        if diagnostic_run:
            raise ValueError("Research and diagnostic experiment purposes cannot be combined")
        if provenance.get("operating_condition_sha256") != research["operating_condition_sha256"]:
            raise RuntimeError("Research operating condition changed after preflight")
    if (pid_candidate is not None or diagnostic_profile == PRECISION_PROFILE) and cfg.get("hover_reference_profile") is None:
        raise ValueError("Precision localization and PID tuning require explicit fixed world position control")
    if cfg["stage"] not in STAGES:
        raise NotImplementedError("Unsupported X650 stage: "+str(cfg["stage"]))
    if cfg["stage"] in {"hover_folded", "joint_probe", "response_probe"} and cfg["controller"] != "pid_ff":
        raise ValueError("Native commissioning probes use PX4 PID; select pid_ff to avoid labelling them MPC")
    model = ArmModel.from_report(resolved(cfg["preparation_report"]))
    report = json.loads(resolved(cfg["preparation_report"]).read_text(encoding="utf-8"))
    model_sha, source_sha = model_spec.sha256, report["derived_urdf_sha256"]
    gates = (formal_gate_evidence(cfg, model_sha, source_sha) if extension is None else
             extension.stage_gates(cfg, model_sha, source_sha))
    accepted_response = None
    if cfg.get("mixed_mpc_protocol") is not None:
        from accepted_response_model import load_accepted_response_model
        accepted_response = load_accepted_response_model(cfg)
        if accepted_response["binding"] != provenance.get("accepted_response_binding"):
            raise RuntimeError("Accepted response binding changed before runtime admission")
        if cfg["controller"] == "mpc":
            # Load SciPy before takeoff, without an unreported solve warm-up.
            # solve_ms covers optimization; compute_ms also includes assembly.
            from x650_wholebody.mpc import MpcController
    meta = dict(schema_version=1, complete=False, stage=cfg["stage"], controller=cfg["controller"],
                task=cfg, model_sha256=model_sha, source_urdf_sha256=source_sha, physical_profile=model.metadata(),
                provenance=provenance, stage_gates=gates, native_tip="Independent native Gazebo plugin only",
                phase="preflight", parameter_writes=False, landed_and_disarmed=False,
                allocator_active=cfg["stage"] in {"tip_hold", "small", "normal", "fast"},
                external_force_disturbance=False, response_identification_claimed=False,
                diagnostic_only=diagnostic_run, ground_truth_aided=False,
                aiding_requested=diagnostic_profile is not None, aiding_confirmed=False,
                formal_performance_claim=False if diagnostic_run else None,
                eligible_as_stage_gate=not diagnostic_run,
                telemetry_freshness={"continuous_topic_wall_timeout_s": {"truth": 2., "joints": 2., "position": 2., "status": 3.},
                    "land_watchdog_timeout_sim_s": 3.,
                    "land_semantics": "new ApiNode land callback token observed at every pump, bound to recorder simulation clock; cached initial message cannot establish liveness",
                    "transport_age_claimed": False,
                    "px4_timestamps": "DDS time-sync adjusted epoch; diagnostic only, may jump or regress in non-real-time SITL",
                    "transition_semantics": "new land and status callbacks after actual NAV send plus expected airborne/armed or landed/disarmed state"})
    if upper_research:
        meta["telemetry_freshness"]["continuous_topic_wall_timeout_s"].pop("status")
        meta["telemetry_freshness"]["status_receipt_watchdog"] = StatusReceiptWatchdog.contract()
    meta.update(research)
    atomic_json(out/"controller_task.json", cfg)
    atomic_json(out/"controller_metadata.json", meta)
    error, loop, capture, native, evidence, diagnostic, diagnostic_observer = None, None, None, None, None, None, None
    attempted_takeoff, task_window, target = False, None, None
    with ScenarioContext() as ctx:
        try:
            if upper_research:
                ctx.node.enable_estimator_feedback()
            if (ctx.model["model_sha256"] != model_sha or ctx.model["model"] != model_spec.model_name
                    or ctx.model["joint_names"] != list(JOINTS)):
                raise RuntimeError("Active model/joint contract differs from selected composed X650")
            ctx.vehicle.wait_ready()
            initial = ctx.observe(ground_truth=True)
            if initial.vehicle.armed or not initial.vehicle.landed:
                raise RuntimeError("Fresh disarmed, landed simulation is required")
            evidence = load_parameter_evidence()(out, validate_px4_airframe(model_spec),
                resolved(cfg["airframe_source"]), model_spec.px4_sys_autostart, model_sha)
            evidence.capture(ctx.vehicle, "before", enforce_contract=True)
            pid = actual_pid_parameters(evidence)
            if not research_run:
                meta["pid_profile"] = verify_pid_profile(cfg.get("pid_profile"), pid)
            meta["actual_pid_parameters"] = pid
            if diagnostic_profile is not None:
                observer_kwargs = {} if not research_run else {"purpose": "research", "operating_condition": research["operating_condition"]}
                diagnostic_observer = load_diagnostic_odometry()(ROOT, active_runtime_instance(ROOT, model_sha), out, **observer_kwargs)
                diagnostic_observer.load(ctx.vehicle._pump)
                meta["diagnostic_odometry"] = diagnostic_observer.document
            if diagnostic_profile is not None or pid_candidate is not None:
                records = evidence.snapshots["before"]["parameters"]
                extra = {} if pid_candidate is None else {"pid_candidate": pid_candidate}
                if research_run:
                    extra.update(purpose="research", operating_condition=research["operating_condition"])
                diagnostic = DiagnosticParameterSession(ctx, diagnostic_profile, out,
                    protected_pid_records={name: record for name, record in records.items() if name.startswith(("MC_", "MPC_"))}, **extra)
                diagnostic.apply()
                pid = {name: record["value"] for name, record in diagnostic.document["applied"].items() if name.startswith(("MC_", "MPC_"))}
                meta.update(ground_truth_aided=diagnostic_profile is not None, parameter_writes=True,
                    actual_pid_parameters=pid, pid_tuning_candidate=pid_candidate,
                    parameter_write_scope="Exact reviewed estimator/PID profile; other actual MC/MPC gains protected; restore original typed values after landing",
                    diagnostic_parameters=diagnostic.document)
                atomic_json(out/"controller_metadata.json", meta)
            if research_run:
                meta["pid_profile"] = verify_pid_profile(cfg["pid_profile"], pid)
                meta.update(research_annotations(cfg, pid))
                atomic_json(out/"controller_metadata.json", meta)
            state, _ = state_from_node(ctx.node)
            if np.max(abs(state.q)) > .01:
                raise RuntimeError("Initial arm must be folded at measured q0")
            target = state.position+np.array([0., 0., cfg["hover_height_m"]])
            capture = FlightRecorder(out, model_sha256=model_sha, source_urdf_sha256=source_sha)
            capture.target_body = target.copy()
            capture.attach(ctx)
            loop_kwargs = {} if extension is None else {"extension": extension}
            loop = ControlLoop(ctx, cfg, model, capture, pid, out, **loop_kwargs)
            ctx.vehicle._pump = loop.pump
            if extension is not None:
                extension.on_loop_ready(ctx, loop, meta, out)
            native = NativeRecorder(out, resolved(cfg["native_recorder_library"]), active_runtime_instance(ROOT, model_sha))
            native.load(loop.pump)
            meta["native_ground"] = ground_gate(ctx, loop, native, cfg, model_sha, source_sha, out)
            if diagnostic_observer is not None:
                required_flags = diagnostic.document.get("required_fusion_flags_before_takeoff")
                meta["diagnostic_odometry"] = diagnostic_observer.confirm_horizontal_fusion(loop.pump, required_flags=required_flags)
                if meta["diagnostic_odometry"].get("aiding_confirmed") is not True:
                    raise RuntimeError("Horizontal external-position fusion was not confirmed; diagnostic takeoff refused")
                meta["aiding_confirmed"] = True
                diagnostic.document["aiding_confirmed"] = True
                diagnostic.save()
                if diagnostic_profile == PRECISION_PROFILE:
                    # Aiding can reset altitude while retaining the old map origin.
                    # Require new global messages and stable, valid local-origin
                    # state before NAV_TAKEOFF computes its relative altitude.
                    from hover_reference import position_snapshot
                    ctx.vehicle.global_position = None
                    ctx.vehicle._command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                        [mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000])
                    stable = [None, None]
                    def switched_estimator_ready():
                        snapshot = position_snapshot(ctx.node.data["position"])
                        identity = tuple(snapshot[k] for k in ("ref_lat", "ref_lon", "ref_alt", "xy_reset_counter", "z_reset_counter", "heading_reset_counter"))
                        if identity != stable[0]:
                            stable[:] = [identity, capture.now()]
                        return (capture.now()-stable[1] >= 2. and ctx.vehicle.global_position is not None
                                and time.monotonic()-ctx.vehicle.global_time < 2.)
                    ctx.vehicle.wait(switched_estimator_ready, 60., "fresh EV-frame global data and stable local origin")
                    meta["precision_pre_takeoff_reference"] = position_snapshot(ctx.node.data["position"])
                    _, ground_state, _ = loop.read()
                    p = ctx.node.data["position"]
                    reconstructed = np.array([p.y, p.x, p.ref_alt-p.z], dtype=float)
                    ground_error = reconstructed-ground_state.position
                    meta["precision_ground_frame_check"] = {"estimated_world_enu_m": reconstructed,
                        "native_world_enu_m": ground_state.position, "difference_m": ground_error,
                        "used_for_setpoint_calibration": False, "tolerance_m": .10}
                    if not np.isfinite(ground_error).all() or np.linalg.norm(ground_error) > .10:
                        raise RuntimeError("Raw EV source-backed world/local coordinate check failed before takeoff")
            if upper_research:
                from estimated_feedback import Px4EstimatedFeedback
                from hover_reference import FixedWorldHoverReference
                from estimated_feedback import TOPICS
                ctx.vehicle.wait(lambda: ctx.node.estimator_clock_s is not None and all(
                    ctx.node.received_sim.get(key) is not None for key in TOPICS), 15., "all PX4 estimated-feedback topics")
                meta["estimated_feedback_ground_probe"] = estimated_ground_probe(ctx.node, capture.now())
                meta["hover_reference_role"] = "Frozen datum and predeclared target; upper v2 sends dynamic world PVA"
            loop.land_watchdog.check(capture.now())
            loop.strict = True
            capture.phase, meta["phase"] = "takeoff", "takeoff"
            atomic_json(out/"controller_metadata.json", meta)
            attempted_takeoff = True
            takeoff_marker, marker_error = perform_nav_operation(ctx, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                lambda: ctx.vehicle.takeoff(cfg["hover_height_m"], stable_s=3.))
            if marker_error is not None:
                raise marker_error
            if takeoff_marker is None:
                raise RuntimeError("No actual NAV_TAKEOFF send observed; takeoff cannot be certified")
            meta["takeoff_state_transition"] = confirm_land_state_after_command(ctx, takeoff_marker, landed=False,
                watchdog=loop.land_watchdog, sim_clock=capture.now, status_watchdog=loop.status_watchdog)
            capture.phase = "settle"
            wait_sim(ctx, loop, 5., "post-takeoff settle")
            if upper_research:
                ctx.vehicle.wait(lambda: bool(ctx.node.data["position"].heading_good_for_control),
                                 45., "final in-flight heading alignment before upper control")
            _, anchor_state, _ = loop.read()
            loop.anchor = anchor_state.position.copy()
            loop.yaw_ned = float(ctx.node.data["position"].heading)
            fixed_profile = cfg.get("hover_reference_profile")
            if fixed_profile is not None:
                from hover_reference import FixedWorldHoverReference
                from diagnostic_odometry import active_world
                from xml.etree import ElementTree as ET
                world_path = Path(fixed_profile["world_sdf"])
                if sha(world_path) != provenance["hover_world_sdf_sha256"]:
                    raise RuntimeError("Fixed hover datum changed after preflight")
                declared_world = ET.parse(world_path).getroot().find("world")
                actual_world, _ = active_world(ROOT, active_runtime_instance(ROOT, model_sha))
                if declared_world is None or declared_world.get("name") != actual_world:
                    raise RuntimeError("Fixed hover datum world differs from actual runtime world")
                loop.fixed_reference = FixedWorldHoverReference.bind(position=ctx.node.data["position"],
                    target_world_enu_m=target, world_sdf=fixed_profile["world_sdf"], yaw_ned=loop.yaw_ned,
                    frame_mode="raw_ev_world" if diagnostic_profile == PRECISION_PROFILE else "global_wgs84")
                loop.anchor = target.copy()
                meta["fixed_hover_reference"] = loop.fixed_reference.document
                atomic_json(out/"hover_reference.json", loop.fixed_reference.document)
                if upper_research:
                    feedback_kwargs = {"require_acceleration": True} if cfg.get("mixed_mpc_protocol") is not None else {}
                    if "prediction_chain" in (cfg.get("mixed_mpc_protocol") or {}).get("mpc_controller_config", {}):
                        feedback_kwargs["require_prediction_timing"] = True
                    if extension is not None:
                        feedback_kwargs.update(extension.feedback_kwargs(cfg))
                    loop.feedback_adapter = Px4EstimatedFeedback(loop.fixed_reference, ctx.node, **feedback_kwargs)
                    _, anchor_state, _ = loop.read()
                    loop.last, loop.next_control = None, loop.control_now()
                    meta["estimated_feedback"] = loop.feedback_adapter.document
                    atomic_json(out/"estimated_feedback.json", loop.feedback_adapter.document)
                if cfg.get("precision_test_profile") is not None:
                    from position_step_probe import probe_metadata
                    probe = probe_metadata()
                    for segment in probe["segments"]:
                        step_ref = FixedWorldHoverReference.bind(position=ctx.node.data["position"],
                            target_world_enu_m=target+np.asarray(segment["target_offset_world_enu_m"]),
                            world_sdf=fixed_profile["world_sdf"], yaw_ned=loop.yaw_ned,
                            frame_mode=loop.fixed_reference.document["frame_mode"])
                        loop.step_references[segment["segment_index"]] = step_ref
                        segment["reference"] = step_ref.document
                    meta["position_step_probe"] = probe
                    atomic_json(out/"position_step_probe.json", probe)
            if cfg["stage"] not in {"hover_folded", "joint_probe"} or fixed_profile is not None:
                # Continue position holding while prestreaming; no allocator
                # history is advanced before its declared task time origin.
                loop.send_reference = True
                prestream = time.monotonic()
                while time.monotonic()-prestream < 1.25:
                    loop.pump()
                ctx.vehicle._command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, [1, 6, 0])
                # Enable reference sending during the transition, but delay the
                # strict nav-state check until PX4 confirms offboard.
                ctx.vehicle.wait(lambda: ctx.node.data["status"].nav_state == 14, 8., "offboard confirmation")
                loop.offboard = True
                wait_sim(ctx, loop, 3., "offboard settle")
                if cfg["stage"] not in {"response_probe", "hover_folded", "joint_probe"}:
                    center = tip_world(loop.anchor, anchor_state.rotation, np.zeros(2), model)
                    trajectory = ConstantTarget(center) if cfg["stage"] == "tip_hold" else FigureEight(center, **cfg["trajectory_parameters"])
                    allocator_kwargs = {"accepted_response": accepted_response} if accepted_response is not None else {}
                    allocator = (build_allocator(cfg, model, pid, trajectory, **allocator_kwargs) if extension is None else
                                 extension.build_allocator(cfg, model, pid, trajectory, loop))
                    adapter_kwargs = {"acceleration_feedforward_policy": "zero"} if cfg.get("mixed_mpc_protocol") is not None else {}
                    if extension is not None:
                        adapter_kwargs.update(extension.adapter_kwargs(cfg))
                    loop.controller = ControllerAdapter(allocator, model, pid, **adapter_kwargs)
                    loop.controller.reset(loop.anchor, anchor_state.rotation, np.zeros(2))
                    loop.trajectory = trajectory
                    meta["controller_details"] = loop.controller.metadata()
                    if cfg.get("mixed_mpc_protocol") is not None or extension is not None:
                        # ROS callbacks run before the first control step.
                        # Declare the existing t=0 target before marking any
                        # sample as measure, rather than leaving it blank.
                        capture.tip_target = trajectory.sample(0.)[0].copy()
            if extension is not None:
                start = extension.activate(ctx, loop, cfg, meta, anchor_state.rotation)
            elif "prediction_chain" in (cfg.get("mixed_mpc_protocol") or {}).get("mpc_controller_config", {}):
                start = activate_prediction_chain(ctx, loop, cfg, meta, anchor_state.rotation)
            else:
                capture.phase = "measure"
                start = loop.control_now()
                loop.task_start = start
                loop.last, loop.next_control = None, start
                meta.update(phase="measure", task_start_sim_s=start, reference_base_enu_m=loop.anchor,
                            reference_rotation=anchor_state.rotation, requested_duration_s=cfg["duration_s"])
                atomic_json(out/"controller_metadata.json", meta)
                print("X650_STAGE_START", cfg["stage"], cfg["controller"], flush=True)
            wait_sim(ctx, loop, cfg["duration_s"], "X650 "+cfg["stage"])
            task_window = (start, loop.control_now())
            meta["task_end_sim_s"] = task_window[1]
            if cfg.get("precision_test_profile") is not None:
                from position_step_probe import TOTAL_DURATION_S
                meta["primary_hover_window_sim_s"] = list(task_window)
                capture.phase = "position_steps"
                wait_sim(ctx, loop, max(0., start+TOTAL_DURATION_S-capture.now()), "six declared position steps")
                meta["experiment_end_sim_s"] = capture.now()
                meta["position_step_send_events"] = loop.step_send_events
            meta["max_joint_command_rad_s"] = loop.max_joint_cmd
            restore_folded(ctx, loop)
            meta["folded_before_landing"] = True
        except BaseException as exc:
            error = exc
            meta.update(error=repr(exc), traceback=traceback.format_exc(), phase="recovery")
            if hasattr(exc, "diagnostics"):
                # An infeasible solve may contain inf/NaN. Preserve it as an
                # explicit marker so JSON logging cannot hide the real error.
                meta["failure_diagnostics"] = evidence_value(exc.diagnostics)
            if loop is not None:
                meta["telemetry_receipt_state_at_failure"] = loop.land_watchdog.diagnostics(ctx.node, capture.now())
                if loop.execution_trace.first_failure is not None:
                    meta["failure_control_snapshot"] = loop.execution_trace.first_failure
                if loop.execution_trace.logging_error is not None:
                    meta["execution_trace_logging_error"] = loop.execution_trace.logging_error
            atomic_json(out/"controller_metadata.json", meta)
        finally:
            if attempted_takeoff:
                try:
                    if loop is not None and not meta.get("folded_before_landing"):
                        loop.strict = False
                        if upper_research:
                            loop.disable_upper_feedback()
                            meta["upper_feedback_disabled_for_failed_recovery"] = True
                        restore_folded(ctx, loop)
                        meta["folded_before_landing"] = True
                except BaseException as exc:
                    meta["fold_recovery_error"] = repr(exc)
                    if error is None:
                        error = exc
                try:
                    if capture is not None:
                        capture.phase = "landing"
                    # NAV_LAND leaves offboard. Stop publishing offboard refs,
                    # but keep measured-q home holding through the same pump.
                    if loop is not None:
                        loop.offboard, loop.send_reference, loop.strict = False, False, False
                        if upper_research:
                            loop.disable_upper_feedback()
                    landing_marker = None
                    meta["landing_command_attempted"] = True
                    landing_marker, marker_error = perform_nav_operation(ctx, mavutil.mavlink.MAV_CMD_NAV_LAND,
                        ctx.vehicle.land)
                    meta["landing_api_returned"] = True
                    if marker_error is not None:
                        meta["landing_marker_error"] = repr(marker_error)
                        raise marker_error
                    if landing_marker is None:
                        raise RuntimeError("NAV_LAND operation returned, but no actual command send marker permits landing certification")
                    meta["landing_state_transition"] = confirm_land_state_after_command(ctx, landing_marker, landed=True,
                        watchdog=loop.land_watchdog, sim_clock=capture.now, status_watchdog=loop.status_watchdog)
                    meta["landed_and_disarmed"] = True
                    if capture is not None:
                        capture.phase = "landed"
                    if loop is not None:
                        wait_sim(ctx, loop, 1., "landed capture")
                except BaseException as exc:
                    meta["landing_error"] = repr(exc)
                    if error is None:
                        error = exc
            if diagnostic is not None:
                try:
                    if attempted_takeoff and not meta["landed_and_disarmed"]:
                        raise RuntimeError("Diagnostic restoration blocked: landing/disarm has not been confirmed; pending journal retained")
                    diagnostic.restore()
                    meta["diagnostic_parameters_restored"] = diagnostic.document["restored_verified"]
                except BaseException as exc:
                    meta["diagnostic_restore_error"] = repr(exc)
                    if error is None:
                        error = exc
                meta["diagnostic_parameters"] = diagnostic.document
                meta["parameter_writes"] = diagnostic.touched
                meta["diagnostic_restore_pending"] = diagnostic.journal_path.exists()
            if diagnostic_observer is not None:
                meta["diagnostic_odometry"] = diagnostic_observer.document
            if evidence is not None and "before" in evidence.snapshots and meta["landed_and_disarmed"]:
                try:
                    evidence.capture(ctx.vehicle, "after", enforce_contract=True)
                    meta["parameter_evidence"] = evidence.reports
                except BaseException as exc:
                    meta["parameter_after_error"] = repr(exc)
                    if error is None:
                        error = exc
            if native is not None:
                try:
                    meta["native_recording"] = native.freeze(capture.now(), model_sha256=model_sha,
                        source_urdf_sha256=source_sha, complete=error is None and meta["landed_and_disarmed"],
                        task_window=(task_window[0], meta["experiment_end_sim_s"]) if task_window is not None and "experiment_end_sim_s" in meta else task_window)
                except BaseException as exc:
                    meta["native_recording_error"] = repr(exc)
                    if error is None:
                        error = exc
            if loop is not None:
                if loop.step_references:
                    meta["position_step_send_events"] = loop.step_send_events
                try:
                    meta["control_execution_trace"] = loop.close(experiment_complete=error is None and meta["landed_and_disarmed"])
                except BaseException as exc:
                    meta["execution_trace_close_error"] = repr(exc)
                    if error is None:
                        error = exc
            if capture is not None:
                try:
                    capture.check()
                    if cfg["stage"] == "hover_folded" and task_window is not None:
                        meta["hover_metrics"] = native_hover_metrics(out/"native_trace.csv", target, task_window,
                                                                    expected_duration_s=cfg["duration_s"])
                        if not meta["hover_metrics"]["accepted"]:
                            raise RuntimeError("Folded hover commissioning envelope did not pass")
                    if cfg["stage"] == "joint_probe" and task_window is not None:
                        meta["joint_probe_metrics"] = native_joint_probe_metrics(out, joint_probe_reference,
                            task_window[0], cfg["duration_s"], loop.anchor)
                        if not meta["joint_probe_metrics"]["passed"]:
                            raise RuntimeError("Independent native joint sign/excitation/tracking/aircraft checks failed")
                    if loop is not None and loop.trajectory is not None and task_window is not None:
                        interval = (0., cfg["duration_s"]) if cfg["stage"] == "tip_hold" else (loop.trajectory.start_s, loop.trajectory.end_s)
                        meta["native_tip_metrics"] = native_tip_metrics(out, loop.trajectory.sample, task_window[0], interval)
                except BaseException as exc:
                    meta["analysis_error"] = repr(exc)
                    if error is None:
                        error = exc
                meta["flight_recording"] = capture.close(complete=error is None and meta["landed_and_disarmed"])
            meta.update(complete=error is None and meta["landed_and_disarmed"], phase="complete" if error is None else "failed")
            atomic_json(out/"controller_metadata.json", meta)
            artifacts = {p.name: sha(p) for p in (
                out/"flight_samples.csv", out/"flight_samples_manifest.json", out/"native_trace.csv",
                out/"hover_reference.json",
                out/"estimated_feedback.json", out/"feedback_trace.jsonl",
                out/"control_execution_trace.jsonl", out/"control_execution_trace_manifest.json",
                out/"control_failure_snapshot.json",
                out/"position_step_probe.json",
                out/"native_trace_manifest.json", out/"control_trace.csv", out/"px4_parameters_before.json",
                out/"px4_parameters_after.json", out/"diagnostic_parameter_session.json",
                out/"diagnostic_parameters_before.json", out/"diagnostic_parameters_applied.json",
                out/"diagnostic_parameters_restored.json", out/"diagnostic_restore_journal_completed.json") if p.is_file()}
            if diagnostic_run or research_run:
                for path in [*out.glob("diagnostic_odometry*"), out/"diagnostic_parameter_storage_save.log"]:
                    if path.is_file():
                        artifacts[path.name] = sha(path)
            stage_evidence = {"success": meta["complete"], "complete": meta["complete"],
                "passed": meta["complete"], "stage": cfg["stage"], "kind": "x650_native_stage_evidence",
                "simulation_backend": "gazebo_px4",
                "model_sha256": model_sha, "source_urdf_sha256": source_sha,
                "landed_and_disarmed": meta["landed_and_disarmed"], "metadata": "controller_metadata.json",
                "artifacts": artifacts}
            if diagnostic_run:
                stage_evidence.update(kind="x650_diagnostic_run_result", diagnostic_only=True, ground_truth_aided=meta["ground_truth_aided"],
                    aiding_confirmed=meta["aiding_confirmed"],
                    eligible_as_stage_gate=False, formal_performance_claim=False,
                    diagnostic_parameters_restored=meta.get("diagnostic_parameters_restored", False))
            if research_run:
                stage_evidence.update(research)
                stage_evidence.update(aiding_confirmed=meta["aiding_confirmed"],
                    diagnostic_parameters_restored=meta.get("diagnostic_parameters_restored", False))
            if extension is not None:
                extension.finalize(meta, stage_evidence, out)
                atomic_json(out/"controller_metadata.json", meta)
            atomic_json(out/"stage_result.json", stage_evidence)
            atomic_json(out/("diagnostic_result.json" if diagnostic_run else cfg["stage"]+".json"), stage_evidence)
    if error is not None:
        raise error
    print("X650_STAGE_COMPLETE", cfg["stage"], str(out), flush=True)
    return meta


def scenario():
    from task import preflight, selected_task_config
    cfg = selected_task_config()
    model, provenance = preflight(cfg)
    out = Path(os.environ["AMSIM_RUN_DIR"]).resolve()
    return execute(cfg, model, provenance, out)


if __name__ == "__main__":
    scenario()
