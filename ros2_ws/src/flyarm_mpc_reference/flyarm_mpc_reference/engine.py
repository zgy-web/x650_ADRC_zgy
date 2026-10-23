"""Pure typed-message adapter around the existing reference controller.

No ROS import, CSV read, complete diagnostic JSON, retry or actuator dispatch is
performed here. The numeric solver is still the existing Python/SciPy MPC.
"""
from __future__ import annotations

import math
import time
import numpy as np


def finite_scalar(value, name):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, float, np.integer, np.floating)):
        raise ValueError(name + " must be numeric")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(name + " must be finite")
    return value


def uint(value, name, *, minimum=0, maximum=2**64-1):
    if isinstance(value, (bool, np.bool_)) or not isinstance(value, (int, np.integer)):
        raise ValueError(name + " must be an integer")
    value = int(value)
    if not minimum <= value <= maximum:
        raise ValueError(name + " is out of range")
    return value


def vector(value, count, name):
    array = np.asarray(value)
    if array.shape != (count,) or array.dtype.kind not in "iuf" or not np.isfinite(array).all():
        raise ValueError(name + " must be a finite fixed-size numeric vector")
    result = np.array(array, dtype=float, copy=True)
    if not np.isfinite(result).all():
        raise ValueError(name + " cannot be represented by the float64 wire contract")
    return result


def hash_bytes(value):
    raw = list(value)
    if len(raw) != 32:
        raise ValueError("configuration_sha256 must contain 32 bytes")
    return bytes(uint(item, "configuration byte", maximum=255) for item in raw)


class SnapshotContext:
    """One already validated in-memory observation, not a second source read."""
    def __init__(self):
        self.value, self.state, self.task_time = None, None, None

    def bind(self, state, task_time, value):
        self.state, self.task_time, self.value = state, task_time, value

    def __call__(self, state, task_time):
        if self.state is not state or task_time != self.task_time or self.value is None:
            raise ValueError("Reference context differs from the bound snapshot")
        return self.value

    def clear(self):
        self.value, self.state, self.task_time = None, None, None


class ReferenceEngine:
    """Fail-closed sequential worker with an explicit 50 ms outer deadline."""
    def __init__(self, adapter, context, *, task_origin, position_gains, target_position,
                 session_id, epoch, configuration_sha256, state_factory, target_factory,
                 clock_ns=time.monotonic_ns, metadata=None):
        self.adapter, self.context = adapter, context
        self.task_origin = finite_scalar(task_origin, "task origin")
        self.position_gains = vector(position_gains, 3, "position gains")
        self.target_position = vector(target_position, 3, "constant hold target")
        self.session_id, self.epoch = uint(session_id, "session", minimum=1), uint(epoch, "epoch", minimum=1)
        self.configuration_sha256 = hash_bytes(configuration_sha256)
        self.state_factory, self.target_factory, self.clock_ns = state_factory, target_factory, clock_ns
        self.last_control_time, self.last_sequence = self.task_origin, 0
        self.faulted, self.busy, self.last_error = False, False, None
        self._metadata = dict(metadata or {})
        self.requests, self.accepted = 0, 0
        self.simulation_live, self.live_started, self.trajectory = False, False, None

    def begin_live(self, start):
        """One-shot prewarmed handoff. Numerics/reset/history methods unchanged."""
        if not self.simulation_live or self.live_started or self.requests:
            raise ValueError("Live handoff is one-shot and must precede all solves")
        if (int(start.session_id)!=self.session_id or int(start.epoch)!=self.epoch
                or hash_bytes(start.configuration_sha256)!=self.configuration_sha256):
            raise ValueError("Live handoff identity differs")
        count=uint(start.history_count,"history_count",minimum=1,maximum=64)
        times=vector(start.history_time_s,64,"history times")[:count]
        commands=vector(start.history_command,320,"history commands").reshape(64,5)[:count]
        origin=finite_scalar(start.origin_s,"origin")
        if abs(times[-1]-origin)>1e-9 or np.any(np.diff(times)<=0):
            raise ValueError("Live origin must be the final actual prestream send")
        target = vector(start.target_position_enu,3,"target")
        if getattr(self,'live_reference_deferred',False):
            # All executable references come from this one actual handoff.
            # The pre-arm constructor template has never been stepped.
            allocator=self.adapter.allocator
            allocator.trajectory=type(allocator.trajectory)(target.copy())
            self.target_position=target.copy()
            self.live_reference_deferred=False
        elif not np.array_equal(target,self.target_position):
            raise ValueError("Live target changed after warm construction")
        self.adapter.reset(vector(start.anchor_position_enu,3,"anchor"),
            vector(start.anchor_rotation_row_major,9,"rotation").reshape(3,3),vector(start.anchor_q,2,"q"))
        allocator=self.adapter.allocator
        allocator.prime_command_history(list(zip(times,commands)),origin)
        self.task_origin=self.last_control_time=origin
        self.live_started=True

    def metadata(self):
        native = self._metadata.get("numerical_backend",{}).get("kind")=="cpp_numerics_v1"
        return {**self._metadata, "implementation": "frozen_cpp_numerics_with_python_adapter" if native else "existing_python_scipy_reference_mpc",
                "native_mpc": native,
                "replay_only": not self.simulation_live, "actuator_dispatch": False,
                "requests": self.requests, "accepted": self.accepted,
                "faulted": self.faulted, "last_error": self.last_error,
                "outer_deadline_s": .05, "configuration_sha256": self.configuration_sha256.hex()}

    def _history(self, snapshot, prefix, count_name, width):
        count = uint(getattr(snapshot, count_name), count_name, minimum=1, maximum=64)
        values = vector(getattr(snapshot, prefix), 64*width, prefix).reshape(64, width)
        if np.any(values[count:] != 0.):
            raise ValueError(prefix + " unused entries must be zero")
        return count, values[:count]

    def _decode(self, snapshot, begin):
        from_snapshot = lambda name: finite_scalar(getattr(snapshot, name), name)
        if (uint(snapshot.schema_version, "schema", minimum=1, maximum=1) != 1
                or uint(snapshot.session_id, "session", minimum=1) != self.session_id
                or uint(snapshot.epoch, "epoch", minimum=1) != self.epoch
                or snapshot.replay_only is not (not self.simulation_live)
                or hash_bytes(snapshot.configuration_sha256) != self.configuration_sha256):
            raise ValueError("Snapshot session/schema/configuration or offline-only identity differs")
        if self.simulation_live and not self.live_started:
            raise ValueError("Live worker has not received its one-shot handoff")
        sequence = uint(snapshot.sequence, "sequence", minimum=1)
        if sequence != self.last_sequence+1:
            raise ValueError("Snapshot sequence is not the next sequential request")
        issued = uint(snapshot.issued_monotonic_ns, "issued timestamp", minimum=1)
        if issued > begin:
            raise ValueError("Snapshot issue timestamp is in the future")
        if begin-issued > 50_000_000:
            raise TimeoutError("Snapshot exceeded the unchanged 50 ms age limit")
        now, task, dt = (from_snapshot(name) for name in ("control_time_s", "task_time_s", "dt_s"))
        state_time, joint_time = from_snapshot("state_time_s"), from_snapshot("joint_time_s")
        if (now < 0 or task < 0 or not 0 < dt <= .05+1e-10
                or abs(now-self.task_origin-task) > 1e-9
                or abs(now-self.last_control_time-dt) > 1e-9
                or not 0 <= now-state_time <= .25+1e-10
                or not 0 <= now-joint_time <= .025+1e-10):
            raise ValueError("Snapshot control/source clocks or actual control interval differ")
        gains = vector(snapshot.position_gains, 3, "position_gains")
        if not np.array_equal(gains, self.position_gains):
            raise ValueError("Snapshot position gains differ from frozen PID")
        p, v, w, a = (vector(getattr(snapshot, name), 3, name) for name in
                       ("position_enu", "velocity_enu", "omega_body_flu", "acceleration_enu"))
        rotation = vector(snapshot.rotation_row_major, 9, "rotation").reshape(3, 3)
        q, dq = vector(snapshot.q, 2, "q"), vector(snapshot.dq, 2, "dq")
        state = self.state_factory(p, rotation, v, w, q, dq, state_time,
            acceleration_enu=a, control_time_s=now, joint_stamp_s=joint_time)
        target_values = [vector(getattr(snapshot, name), 3, name) for name in
                         ("target_position_enu", "target_velocity_enu", "target_acceleration_enu")]
        expected = (self.target_position,np.zeros(3),np.zeros(3)) if self.trajectory is None else self.trajectory.sample(task)
        if any(not np.array_equal(a,b) for a,b in zip(target_values,expected)):
            raise ValueError("Phase 1 frozen tip-hold target changed")
        target = self.target_factory(*target_values)
        reference_time = from_snapshot("reference_time_s")
        if abs(reference_time-(task-(now-joint_time))) > 1e-9:
            raise ValueError("Reference anchor is not the measured joint source time")
        reference = vector(snapshot.reference_r, 2, "reference_r")
        bus_origin = from_snapshot("bus_origin_s")
        commands, velocities = self._history(snapshot, "command_velocity", "command_count", 2)
        command_times = vector(snapshot.command_time_s, 64, "command times")
        packets = uint(snapshot.packet_count, "packet_count", minimum=1, maximum=64)
        packet_times = vector(snapshot.packet_send_time_s, 64, "packet times")
        if (np.any(command_times[commands:] != 0.) or np.any(packet_times[packets:] != 0.)
                or np.any(np.diff(command_times[:commands]) <= 0.)
                or np.any(np.diff(packet_times[:packets]) <= 0.)
                or command_times[commands-1] > task+1e-10 or packet_times[packets-1] > task+1e-10):
            raise ValueError("History times are unsorted/future or have nonzero unused entries")
        fields = {}
        for name in ("q_ref", "dq_ref", "tau_ff", "kp", "kd"):
            count, values = self._history(snapshot, "packet_"+name, "packet_count", 2)
            fields[name] = values
        context = {"reference_state": {"time_s": reference_time, "r": reference},
            "bus_origin_s": bus_origin,
            "command_history": [(float(when), value.copy()) for when, value in zip(command_times[:commands], velocities)],
            "packet_history": [{"send_time_s": float(packet_times[index]),
                **{name: values[index].copy() for name, values in fields.items()}} for index in range(packets)]}
        return sequence, now, task, dt, state, target, context

    def _empty_proposal(self, snapshot, factory, begin):
        proposal = factory()
        proposal.schema_version, proposal.session_id, proposal.epoch = 1, self.session_id, self.epoch
        try:
            sequence = uint(snapshot.sequence, "sequence", minimum=1)
        except Exception:
            sequence = 0
        proposal.sequence = proposal.snapshot_sequence = sequence
        proposal.replay_only = not self.simulation_live
        proposal.configuration_sha256 = list(self.configuration_sha256)
        proposal.compute_start_monotonic_ns, proposal.compute_end_monotonic_ns = begin, begin
        proposal.status, proposal.accepted_optimum, proposal.nonlinear_check_passed = 1, False, False
        proposal.control_time_s = proposal.task_time_s = 0.
        for name, width in (("position_reference_enu", 3), ("velocity_feedforward_enu", 3),
                            ("acceleration_feedforward_enu", 3), ("base_velocity_target", 3), ("joint_velocity", 2)):
            setattr(proposal, name, [0.]*width)
        proposal.primal_violation, proposal.kkt_residual, proposal.subproblems = 0., 0., 0
        return proposal

    def process(self, snapshot, proposal_factory):
        begin = self.clock_ns()
        proposal = self._empty_proposal(snapshot, proposal_factory, begin)
        self.requests += 1
        try:
            if self.faulted or self.busy:
                raise ValueError("Reference worker is faulted/busy; an explicit new process/epoch is required")
            decoded = self._decode(snapshot, begin)
            sequence, now, task, dt, state, target, context = decoded
            self.last_sequence = sequence
            proposal.control_time_s, proposal.task_time_s = now, task
            self.busy = True
            self.context.bind(state, task, context)
            try:
                command = self.adapter.step(task, dt, state, target)
            except Exception as error:
                proposal.status = 2
                raise RuntimeError("Existing reference controller rejected: " + str(error)) from error
            proposal.status = 2
            diagnostic = command.diagnostics
            if (diagnostic.get("accepted_optimum") is not True
                    or diagnostic.get("ts20_nonlinear_check_passed") is not True
                    or diagnostic.get("previous_plan_fallback") is not False):
                proposal.status = 2
                raise RuntimeError("Missing accepted optimum/full nonlinear check, or a forbidden fallback")
            primal = finite_scalar(diagnostic["primal_violation"], "primal_violation")
            # The original _quality returns the infinity norm of stationarity,
            # not a field called kkt_residual. Preserve that actual meaning on
            # the compact wire field instead of fabricating a new metric.
            kkt = finite_scalar(diagnostic["kkt_stationarity_inf"], "kkt_stationarity_inf")
            subproblems = uint(len(diagnostic["ts20_relinearization_attempts"]), "subproblems", minimum=1, maximum=3)
            for field, attribute, width in (("position_reference_enu", "position_reference", 3),
                    ("velocity_feedforward_enu", "velocity_feedforward", 3),
                    ("acceleration_feedforward_enu", "acceleration_feedforward", 3),
                    ("base_velocity_target", "base_velocity_target", 3), ("joint_velocity", "joint_velocity", 2)):
                setattr(proposal, field, vector(getattr(command, attribute), width, field).tolist())
            proposal.primal_violation, proposal.kkt_residual, proposal.subproblems = primal, kkt, subproblems
            proposal.status, proposal.accepted_optimum, proposal.nonlinear_check_passed = 0, True, True
        except Exception as error:
            self.faulted = True
            self.last_error = (type(error).__name__+": "+str(error))[:512]
            if isinstance(error, TimeoutError):
                proposal.status = 3
            proposal.accepted_optimum = proposal.nonlinear_check_passed = False
            for name in ("position_reference_enu", "velocity_feedforward_enu", "acceleration_feedforward_enu", "base_velocity_target"):
                setattr(proposal, name, [0.]*3)
            proposal.joint_velocity = [0.]*2
        finally:
            self.context.clear()
            self.busy = False
            proposal.compute_end_monotonic_ns = self.clock_ns()
        if not 0 <= proposal.compute_end_monotonic_ns-begin <= 50_000_000:
            previous_error = self.last_error
            self.faulted = True
            self.last_error = ("Complete reference conversion/controller/proposal exceeded 50 ms"
                               + ("; "+previous_error if previous_error else ""))[:512]
            proposal.status, proposal.accepted_optimum, proposal.nonlinear_check_passed = 3, False, False
            for name in ("position_reference_enu", "velocity_feedforward_enu", "acceleration_feedforward_enu", "base_velocity_target"):
                setattr(proposal, name, [0.]*3)
            proposal.joint_velocity = [0.]*2
        elif proposal.status == 0:
            self.last_control_time, self.accepted = proposal.control_time_s, self.accepted+1
        return proposal
