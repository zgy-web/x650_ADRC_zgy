"""Opt-in TS20 pump evidence without changing scheduling or dispatch policy.

Three due/read cycles and 64 completed pumps of scalar counters are retained.
Natural GC observation has a separate 64-event bound, plus three generation
summaries retaining their wall and CPU peak events after ring eviction.
Ordinary calls add no files, flushes, callback draining or collection-policy
changes. A rejected control interval is copied after rejection, before the
existing recovery path.
"""
from collections import deque
import hashlib
import gc
import json
from pathlib import Path
import time
import threading

from simulations.x650_wholebody_mpc_v1.execution_trace import evidence_value, state_snapshot


class _TimedStream:
    def __init__(self, observer, name, stream):
        self.observer, self.name, self.stream = observer, name, stream

    def write(self, content):
        self.observer.mark(self.name + "_write_begin")
        self.observer.count(self.name + "_write_calls")
        self.observer.count(self.name + "_characters", len(content))
        try:
            return self.stream.write(content)
        finally:
            self.observer.mark(self.name + "_write_exit")

    def flush(self):
        # Delegate exactly the flushes already requested by the shared loop.
        self.observer.mark(self.name + "_flush_begin")
        self.observer.count(self.name + "_flush_calls")
        try:
            return self.stream.flush()
        finally:
            self.observer.mark(self.name + "_flush_exit")

    def __getattr__(self, name):
        return getattr(self.stream, name)


class LoopTimingObserver:
    """Explicitly installed only by the TS20 scenario extension, before arming."""
    def __init__(self, loop, output):
        if loop.writer is not None:
            raise RuntimeError("Loop observer must attach before the CSV writer is created")
        self.loop, self.output = loop, Path(output)
        self.ring = deque(maxlen=3)
        self.recent_pumps = deque(maxlen=64)
        self.current, self.read_result, self.failure = None, None, None
        self.pump_calls, self.read_calls, self.closed = 0, 0, False
        self.observation_errors, self.gc_callback_errors, self.gc_unpaired_stops = 0, 0, 0
        self.gc_events, self.gc_active = deque(maxlen=64), {}
        self.gc_summary = [{"completed": 0, "max_wall_s": 0., "max_thread_cpu_s": 0.,
                            "max_wall_event": None, "max_thread_cpu_event": None}
                           for _ in range(3)]
        self._gc_callback, self._gc_registered = self.on_gc, False
        self.original = {"pump": loop.pump, "original_pump": loop.original_pump,
                         "read": loop.read, "publish": loop.execution_trace.publish}
        self.streams = {name: getattr(loop, name) for name in ("control", "diagnostic", "feedback_stream")}
        loop.pump, loop.original_pump, loop.read = self.pump, self.original_pump, self.read
        loop.execution_trace.publish = self.publish
        for name, stream in self.streams.items():
            if stream is not None:
                setattr(loop, name, _TimedStream(self, name, stream))
        # on_loop_ready runs after the shared lifecycle's first assignment.
        loop.ctx.vehicle._pump = loop.pump
        gc.callbacks.append(self._gc_callback)
        self._gc_registered = True

    def on_gc(self, phase, info):
        """Observe natural collections only; never inspect node or heap objects.

        Duration includes any intervening callbacks as well as collection. It
        establishes an observed interval, not exclusive collector CPU cost.
        """
        if not self._gc_registered:
            return
        try:
            generation = info["generation"]
            if type(generation) is not int or not 0 <= generation < 3:
                self.gc_callback_errors += 1
                return
            stamp = (time.monotonic(), time.perf_counter(), time.thread_time(), threading.get_ident())
            if phase == "start":
                self.gc_active[generation] = stamp
            elif phase == "stop":
                start = self.gc_active.pop(generation, None)
                if start is None:
                    self.gc_unpaired_stops += 1
                    return
                wall, cpu = stamp[1]-start[1], stamp[2]-start[2]
                event = {"generation": generation,
                    "start_wall_monotonic_s": start[0], "stop_wall_monotonic_s": stamp[0],
                    "start_perf_counter_s": start[1], "stop_perf_counter_s": stamp[1],
                    "start_thread_cpu_s": start[2], "stop_thread_cpu_s": stamp[2],
                    "start_thread_ident": start[3], "stop_thread_ident": stamp[3],
                    "wall_s": wall, "thread_cpu_s": cpu,
                    "collected": info.get("collected"), "uncollectable": info.get("uncollectable")}
                self.gc_events.append(event)
                summary = self.gc_summary[generation]
                summary["completed"] += 1
                # The two maxima can belong to different collections. Keep
                # their full timestamps in fixed slots, never an event list.
                if summary["max_wall_event"] is None or wall > summary["max_wall_s"]:
                    summary["max_wall_s"], summary["max_wall_event"] = wall, event
                if summary["max_thread_cpu_event"] is None or cpu > summary["max_thread_cpu_s"]:
                    summary["max_thread_cpu_s"], summary["max_thread_cpu_event"] = cpu, event
        except BaseException:
            # A diagnostics callback must never replace a live safety error.
            self.gc_callback_errors += 1

    def _remove_gc_callback(self):
        if self._gc_registered:
            self._gc_registered = False
            for index, callback in enumerate(gc.callbacks):
                if callback is self._gc_callback:
                    del gc.callbacks[index]
                    break

    def gc_snapshot(self):
        return {"completed_events": list(self.gc_events),
                "per_generation": [{**value,
                    "max_wall_event": None if value["max_wall_event"] is None else dict(value["max_wall_event"]),
                    "max_thread_cpu_event": (None if value["max_thread_cpu_event"] is None
                                             else dict(value["max_thread_cpu_event"]))}
                    for value in self.gc_summary],
                "active_generations": sorted(self.gc_active),
                "unpaired_stops": self.gc_unpaired_stops,
                "callback_errors": self.gc_callback_errors,
                "interval_semantics": "natural collection plus intervening callbacks; not exclusive collector cost"}

    def sample(self):
        node = self.loop.ctx.node
        sequences = getattr(node, "callback_sequences", {})
        return {"wall_monotonic_s": time.monotonic(), "thread_cpu_s": time.thread_time(),
                "processed_clock_s": getattr(node, "estimator_clock_s", None),
                "clock_callback_sequence": sequences.get("clock"),
                "position_callback_sequence": sequences.get("position"),
                "joint_callback_sequence": sequences.get("joints")}

    def mark(self, name):
        if self.current is not None:
            try:
                self.current["marks"][name] = self.sample()
            except BaseException:
                self.observation_errors += 1

    def count(self, name, value=1):
        if self.current is not None:
            counts = self.current["counts"]
            counts[name] = counts.get(name, 0) + value

    def after_drain(self, sim):
        # Called by the existing extension.on_pump hook, after the shared
        # original_pump + 32/6 spin_once drain and recorder.now() call.
        self.mark("after_shared_drain")
        if self.current is not None:
            self.current["recorder_sim_after_drain_s"] = sim

    def original_pump(self):
        self.mark("original_pump_entry")
        try:
            return self.original["original_pump"]()
        finally:
            self.mark("original_pump_exit")

    def read(self):
        self.mark("read_entry")
        result = self.original["read"]()
        self.read_result = result  # actual immutable state; not a later sample
        self.read_calls += 1
        self.mark("read_returned")
        if self.current is not None:
            self.current.update(read_sim_s=result[0], read_state_stamp_s=result[1].stamp_s,
                                read_joint_stamp_s=result[2])
        return result

    def publish(self, channel, payload, operation, **kwargs):
        label = {"joint:arm_joint_1": "joint1", "joint:arm_joint_2": "joint2",
                 "px4_pva": "pva"}.get(channel, "other")
        self.count("publication_calls")
        self.mark(label + "_publish_trace_entry")
        result = self.original["publish"](channel, payload, operation, **kwargs)
        self.mark(label + "_publish_trace_returned")
        if self.current is not None:
            sent = self.loop.execution_trace.last_sent.get(channel)
            if sent is not None:
                # Existing ExecutionTrace samples this wall boundary after the
                # actual local send returns, before its JSON _append. Do not
                # pair it with the observer's later thread CPU timestamp.
                wall = sent.get("publish_after", {}).get("wall_monotonic_s")
                self.current[label + "_local_send_returned_wall_s"] = wall
        return result

    def _record_interval_rejection(self, error):
        loop = self.loop
        if self.read_result is None or self.current is None:
            return
        sim, state, joint_stamp = self.read_result
        dt = loop.cfg["control_period_s"] if loop.last is None else sim-loop.last
        if (not loop.strict or dt <= .05 + 1e-10
                or not str(error).startswith("Control interval ")
                or self.current["counts"].get("publication_calls", 0) != 0):
            return
        self.mark("interval_rejection_observed")
        observed = evidence_value({
            "kind": "ts20_control_interval_rejection_timing_v1",
            "last_control_s": loop.last, "next_control_s": loop.next_control,
            "current_control_s": sim, "current_dt_s": dt,
            "allocator_invoked": False, "command_dispatched": False,
            "previous_control_cycles": list(self.ring), "rejected_cycle": self.current,
            "recent_completed_pumps": list(self.recent_pumps), "recent_pump_capacity": 64,
            "state": state_snapshot(state), "joint_stamp_s": joint_stamp,
            "feedback_snapshot": loop.feedback_diagnostics,
            "natural_gc": self.gc_snapshot(),
            "semantics": ("Actual read return at the unchanged shared dt guard. "
                          "publish_trace_returned includes ExecutionTrace JSON/write; "
                          "local_send_returned_wall_s has no paired observer CPU sample. "
                          "Gaps before stream writes combine conversion/encoding and intervening code.")})
        self.failure = observed
        previous = getattr(error, "diagnostics", None)
        diagnostics = dict(previous) if isinstance(previous, dict) else {}
        diagnostics["ts20_loop_timing"] = observed
        error.diagnostics = diagnostics
        loop.execution_trace._failed({
            "event": "ts20_control_interval_rejected", "sim_s": sim,
            "t_s": -1. if loop.task_start is None else sim-loop.task_start,
            "dt_s": dt, "phase": loop.recorder.phase,
            "allocator_invoked": False, "command_generated": False,
            "command_published": False, "state": state_snapshot(state),
            "feedback_snapshot": loop.feedback_diagnostics,
            "loop_timing": observed}, error)

    def pump(self):
        loop = self.loop
        self.pump_calls += 1
        self.read_result = None
        self.current = {"pump_sequence": self.pump_calls, "marks": {}, "counts": {},
                        "last_control_at_entry_s": loop.last,
                        "next_control_at_entry_s": loop.next_control,
                        "phase": loop.recorder.phase, "recovery": loop.recovery}
        self.mark("pump_entry")
        try:
            return self.original["pump"]()
        except BaseException as error:
            try:
                self._record_interval_rejection(error)
            except BaseException as recording_error:
                # Preserve the existing safety exception and recovery path.
                error.ts20_loop_timing_recording_error = repr(recording_error)
            raise
        finally:
            self.mark("pump_exit")
            self.recent_pumps.append(self.current)
            if self.read_result is not None:
                self.ring.append(self.current)
            self.current, self.read_result = None, None

    def metadata(self):
        return {"enabled": True, "kind": "ts20_opt_in_loop_scalar_timing_v1",
                "retained_control_cycles": 3, "normal_path_extra_file_writes": 0,
                "retained_completed_pumps": 64,
                "normal_path_extra_flushes": 0, "gc_policy_changed": False,
                "natural_gc_callback_registered": self._gc_registered,
                "max_completed_gc_events": 64, "active_collection_requested": False,
                "retained_gc_generation_summaries": 3, "peak_event_slots_per_generation": 2,
                "scheduler_or_deadline_changed": False,
                "json_conversion_and_encoding_separately_measured": False}

    def close(self):
        if self.closed:
            return self.manifest
        loop = self.loop
        try:
            loop.pump, loop.original_pump, loop.read = (self.original[name] for name in
                                                       ("pump", "original_pump", "read"))
            loop.execution_trace.publish = self.original["publish"]
            for name, stream in self.streams.items():
                setattr(loop, name, stream)
            loop.ctx.vehicle._pump = self.original["original_pump"]
        finally:
            # Removal precedes any postflight JSON/file operation. Only our
            # exact callback object is removed; enabled/thresholds are untouched.
            self._remove_gc_callback()
        document = {**self.metadata(), "closed": True, "pump_calls": self.pump_calls,
                    "read_calls": self.read_calls, "last_control_cycles": list(self.ring),
                    "recent_completed_pumps": list(self.recent_pumps),
                    "interval_failure": self.failure,
                    "natural_gc": self.gc_snapshot(), "observation_errors": self.observation_errors,
                    "observer_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
        path = self.output / "loop_timing_evidence.json"
        with path.open("x", encoding="utf-8") as stream:
            json.dump(evidence_value(document), stream, indent=2, allow_nan=False)
            stream.write("\n")
        self.manifest = {"file": path.name, "sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                         "complete": True, "interval_failure_recorded": self.failure is not None,
                         "observations_valid": (self.observation_errors == 0 and self.gc_callback_errors == 0
                                                and self.gc_unpaired_stops == 0 and not self.gc_active)}
        self.closed = True
        return self.manifest
