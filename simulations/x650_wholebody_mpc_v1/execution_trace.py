"""Controller-attempt and publication timing evidence, never application ACKs."""
from __future__ import annotations

import hashlib
import json
import math
from pathlib import Path
import time
import traceback


def evidence_value(value):
    """Preserve nonfinite failures explicitly without emitting invalid JSON."""
    if isinstance(value, dict):
        return {str(key): evidence_value(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [evidence_value(item) for item in value]
    if hasattr(value, "tolist"):
        return evidence_value(value.tolist())
    if isinstance(value, float) and not math.isfinite(value):
        return {"nonfinite": "nan" if math.isnan(value) else "+inf" if value > 0 else "-inf"}
    if value is None or isinstance(value, (str, int, float, bool)):
        return value
    if hasattr(value, "item"):
        return evidence_value(value.item())
    return {"unsupported_type": type(value).__module__+"."+type(value).__qualname__, "repr": repr(value)}


def callback_clock_snapshot(node):
    """Read already processed callbacks only; do not spin or sample Gazebo."""
    message = getattr(node, "data", {}).get("joints")
    header = getattr(message, "header", None)
    stamp = getattr(header, "stamp", None)
    sec, nanosec = getattr(stamp, "sec", None), getattr(stamp, "nanosec", None)
    joint_stamp = None if sec is None or nanosec is None else sec+nanosec*1e-9
    return evidence_value({
        "wall_monotonic_s": time.monotonic(),
        "processed_estimator_clock_s": getattr(node, "estimator_clock_s", None),
        "joint_header": {"sec": sec, "nanosec": nanosec},
        "joint_header_stamp_s": joint_stamp,
        "joint_callback_received_wall_monotonic_s": getattr(node, "received", {}).get("joints"),
        "joint_callback_received_sim_s": getattr(node, "received_sim", {}).get("joints"),
        "joint_callback_sequence": getattr(node, "callback_sequences", {}).get("joints"),
    })


def state_snapshot(state):
    return evidence_value({name: getattr(state, name, None) for name in
        ("position", "rotation", "velocity", "omega_body", "q", "dq", "stamp_s", "acceleration_enu",
         "control_time_s", "joint_stamp_s")})


def target_snapshot(target):
    return evidence_value({name: getattr(target, name, None) for name in ("position", "velocity", "acceleration")})


def exception_snapshot(error):
    return evidence_value({"type": type(error).__module__+"."+type(error).__qualname__,
        "repr": repr(error), "message": str(error), "traceback": traceback.format_exc(),
        "diagnostics": getattr(error, "diagnostics", None)})


def observed_clock(clock_snapshot):
    try:
        return evidence_value(clock_snapshot())
    except BaseException as error:
        return {"clock_observation_error": repr(error)}


class ExecutionTrace:
    """Append evidence without changing commands or swallowing control errors.

    A successful publication means only that the local publish/send call
    returned. Processed callback clocks are held observations, not an estimate
    of when Gazebo received/applied the command. That requires separate evidence.
    """
    def __init__(self, folder):
        self.folder = Path(folder)
        self.path = self.folder/"control_execution_trace.jsonl"
        self.stream = self.path.open("x", encoding="utf-8")
        self.sequence = 0
        self.written_events = 0
        self.last_sent = {}
        self.failure_count = 0
        self.first_failure = None
        self.logging_error = None
        self.manifest = None

    def _append(self, event, *, flush=False):
        self.sequence += 1
        event = evidence_value({"schema_version": 1, "sequence": self.sequence, **event})
        try:
            self.stream.write(json.dumps(event, allow_nan=False, separators=(",", ":"))+"\n")
            self.written_events += 1
            if flush or self.sequence % 50 == 0:
                self.stream.flush()
        except BaseException as error:
            self.logging_error = {"type": type(error).__name__, "repr": repr(error)}
            raise
        return event

    def _failed(self, event, original_error):
        """A recorder failure must not replace the controller's original error."""
        self.failure_count += 1
        event = evidence_value({**event, "exception": exception_snapshot(original_error),
                                "previous_publications": self.last_sent})
        if self.first_failure is None:
            self.first_failure = event
        try:
            saved = self._append(event, flush=True)
            snapshot = self.folder/"control_failure_snapshot.json"
            if self.failure_count == 1:
                with snapshot.open("x", encoding="utf-8") as stream:
                    json.dump(saved, stream, indent=2, allow_nan=False)
                    stream.write("\n")
        except BaseException as logging_error:
            self.logging_error = evidence_value({"type": type(logging_error).__name__, "repr": repr(logging_error)})

    def controller_step(self, controller, t, dt, state, target, *, feedback, phase, clock_snapshot):
        # Copy the inputs before invoking user/controller code, so mutations in
        # an unsuccessful implementation cannot rewrite the failure snapshot.
        before = observed_clock(clock_snapshot)
        context = evidence_value({"t_s": t, "dt_s": dt, "phase": phase,
            "state": state_snapshot(state), "target": target_snapshot(target),
            "feedback_snapshot": feedback, "compute_before": before})
        try:
            result = controller.step(t, dt, state, target)
        except BaseException as error:
            self._failed({"event": "controller_step_exception", **context,
                          "compute_after": observed_clock(clock_snapshot), "command_generated": False,
                          "command_published": False}, error)
            raise
        self._append({"event": "controller_step_returned", **context,
                      "compute_after": observed_clock(clock_snapshot), "command_published": False})
        return result

    def publish(self, channel, payload, operation, *, clock_snapshot, context):
        before = observed_clock(clock_snapshot)
        payload = evidence_value(payload)
        try:
            result = operation()
        except BaseException as error:
            self._failed({"event": "publication_exception", "channel": channel, "payload": payload,
                "context": context, "publish_before": before, "publish_after": observed_clock(clock_snapshot),
                "local_call_returned": False, "downstream_application_confirmed": False}, error)
            raise
        publication = evidence_value({"channel": channel, "payload": payload,
            "context": context, "publish_before": before, "publish_after": observed_clock(clock_snapshot),
            "local_call_returned": True, "downstream_application_confirmed": False})
        # Track the actual last local return per actuator, including a partial
        # pair if a later joint publication raises. Do not claim a batch ACK.
        self.last_sent[channel] = publication
        self._append({"event": "publication_returned", **publication})
        return result

    def close(self, *, experiment_complete=None):
        if self.manifest is not None:
            return self.manifest
        self.stream.flush()
        self.stream.close()
        self.manifest = {
            "kind": "x650_control_execution_trace", "schema_version": 1,
            "file": self.path.name, "sha256": hashlib.sha256(self.path.read_bytes()).hexdigest(),
            "event_count": self.written_events, "event_sequence_attempts": self.sequence,
            "failure_count": self.failure_count,
            "recording_closed": True, "recording_error": self.logging_error,
            "experiment_complete": experiment_complete, "eligible_as_stage_gate": False,
            "source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            "clock_semantics": "monotonic local wall clock plus latest already processed /clock and JointState header; no extra spin",
            "publication_semantics": "local ROS publish or MAVLink send returned; neither Gazebo receipt nor application is acknowledged",
            "last_publications": evidence_value(self.last_sent),
        }
        snapshot = self.folder/"control_failure_snapshot.json"
        if snapshot.is_file():
            self.manifest["first_failure_snapshot"] = {"file": snapshot.name,
                "sha256": hashlib.sha256(snapshot.read_bytes()).hexdigest()}
        path = self.folder/"control_execution_trace_manifest.json"
        with path.open("x", encoding="utf-8") as stream:
            json.dump(self.manifest, stream, indent=2, allow_nan=False)
            stream.write("\n")
        return self.manifest
