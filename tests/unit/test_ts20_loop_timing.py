"""Bounded timing evidence preserves existing dispatch/scheduler semantics."""
from io import StringIO
import ast
import gc
import json
from pathlib import Path
import shutil
from types import SimpleNamespace
import sys
import time
import unittest
from unittest.mock import patch
from uuid import uuid4

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_ts20_hold_v1.loop_timing import LoopTimingObserver
from simulations.x650_wholebody_mpc_v1.execution_trace import ExecutionTrace


class Stream(StringIO):
    def __init__(self):
        super().__init__()
        self.flush_calls = 0

    def flush(self):
        self.flush_calls += 1
        return super().flush()


class LoopTimingTests(unittest.TestCase):
    def setUp(self):
        parent = (ROOT.parent / "tmp").resolve()
        parent.mkdir(exist_ok=True)
        self.output = parent / ("loop_timing_test_" + uuid4().hex)
        self.output.mkdir()
        def cleanup():
            self.output.resolve().relative_to(parent)
            shutil.rmtree(self.output)
        self.addCleanup(cleanup)

    def make_loop(self, clocks):
        clock_values = iter(clocks)
        node = SimpleNamespace(estimator_clock_s=0., callback_sequences={"clock": 0, "position": 0, "joints": 0})
        counts = {"original_pump": 0, "read": 0, "allocator": 0, "send": 0}
        state = SimpleNamespace(position=[1., 2., 3.], rotation=[[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]],
            velocity=[0., 0., 0.], omega_body=[0., 0., 0.], q=[.01, -.02], dq=[.03, -.04],
            stamp_s=0., acceleration_enu=[0., 0., 0.], control_time_s=0., joint_stamp_s=0.)
        loop = SimpleNamespace(writer=None, cfg={"control_period_s": .02}, last=0., next_control=.02,
            rows=0, strict=True, recovery=False, task_start=0., recorder=SimpleNamespace(phase="measure"),
            ctx=SimpleNamespace(node=node, vehicle=SimpleNamespace()), control=Stream(), diagnostic=Stream(),
            feedback_stream=Stream(), feedback_diagnostics={"snapshot": "actual read", "q": [.01, -.02]})
        trace = ExecutionTrace(self.output)
        loop.execution_trace = trace
        self.addCleanup(lambda: trace.close(experiment_complete=False))
        def original_pump():
            counts["original_pump"] += 1
            node.estimator_clock_s = next(clock_values)
            for key in node.callback_sequences:
                node.callback_sequences[key] += 1
        def read():
            counts["read"] += 1
            state.stamp_s = state.control_time_s = state.joint_stamp_s = node.estimator_clock_s
            return node.estimator_clock_s, state, node.estimator_clock_s
        def pump():
            loop.original_pump()
            loop.observer.after_drain(node.estimator_clock_s)
            if node.estimator_clock_s < loop.next_control-1e-9:
                return
            sim, _, _ = loop.read()
            dt = .02 if loop.last is None else sim-loop.last
            if loop.strict and dt > .05+1e-10:
                raise RuntimeError(f"Control interval {dt:.4f}s exceeds 0.05s")
            counts["allocator"] += 1
            def send():
                counts["send"] += 1
                return "sent"
            loop.execution_trace.publish("px4_pva", {"position": [1., 2., 3.]}, send,
                clock_snapshot=lambda: {"wall_monotonic_s": time.monotonic()}, context={"sim_s": sim})
            loop.feedback_stream.write("feedback\n")
            loop.control.write("control\n")
            loop.diagnostic.write("diagnostic\n")
            if loop.rows % 50 == 0:
                loop.control.flush()
                loop.diagnostic.flush()
                loop.feedback_stream.flush()
            loop.rows += 1
            loop.last, loop.next_control = sim, sim+.02
        loop.pump, loop.original_pump, loop.read = pump, original_pump, read
        loop.ctx.vehicle._pump = loop.pump
        original = {"pump": pump, "original_pump": original_pump, "read": read, "publish": trace.publish,
                    "control": loop.control, "diagnostic": loop.diagnostic, "feedback_stream": loop.feedback_stream}
        observer = LoopTimingObserver(loop, self.output)
        self.addCleanup(observer.close)
        loop.observer = observer
        return loop, observer, counts, state, original

    def test_one_call_delegation_same_dispatch_and_only_existing_flushes(self):
        loop, observer, counts, _, original = self.make_loop([.02, .04, .06, .08])
        for _ in range(4):
            loop.ctx.vehicle._pump()
        self.assertEqual(counts, {"original_pump": 4, "read": 4, "allocator": 4, "send": 4})
        self.assertEqual(loop.rows, 4)
        self.assertAlmostEqual(loop.last, .08)
        self.assertAlmostEqual(loop.next_control, .10)
        for name, content in (("control", "control\n"), ("diagnostic", "diagnostic\n"),
                              ("feedback_stream", "feedback\n")):
            self.assertEqual(original[name].getvalue(), content * 4)
            self.assertEqual(original[name].flush_calls, 1)
        self.assertEqual(len(observer.ring), 3)
        last = observer.ring[-1]
        self.assertIn("after_shared_drain", last["marks"])
        self.assertIn("read_returned", last["marks"])
        self.assertLessEqual(last["pva_local_send_returned_wall_s"],
                             last["marks"]["pva_publish_trace_returned"]["wall_monotonic_s"])
        self.assertNotIn("pva_local_send_returned_thread_cpu_s", last)
        observer.close()
        for name in ("pump", "original_pump", "read"):
            self.assertIs(getattr(loop, name), original[name])
        self.assertIs(loop.ctx.vehicle._pump, original["original_pump"])

    def test_rejected_dt_captures_exact_read_and_feedback_without_invoking_allocator(self):
        loop, observer, counts, state, _ = self.make_loop([.02, .04, .06, .116])
        for _ in range(3):
            loop.pump()
        with self.assertRaisesRegex(RuntimeError, "0.0560s") as raised:
            loop.pump()
        self.assertEqual(counts, {"original_pump": 4, "read": 4, "allocator": 3, "send": 3})
        self.assertAlmostEqual(loop.last, .06)
        self.assertAlmostEqual(loop.next_control, .08)
        captured = raised.exception.diagnostics["ts20_loop_timing"]
        self.assertFalse(captured["allocator_invoked"])
        self.assertFalse(captured["command_dispatched"])
        self.assertAlmostEqual(captured["current_dt_s"], .056)
        self.assertEqual(captured["state"]["control_time_s"], .116)
        self.assertEqual(captured["rejected_cycle"]["marks"]["read_returned"]["clock_callback_sequence"], 4)
        self.assertEqual(len(captured["previous_control_cycles"]), 3)
        state.position[0] = 99.
        loop.feedback_diagnostics["q"][0] = 98.
        self.assertEqual(captured["state"]["position"], [1., 2., 3.])
        self.assertEqual(captured["feedback_snapshot"]["q"], [.01, -.02])
        event = json.loads((self.output / "control_failure_snapshot.json").read_text())
        self.assertEqual(event["event"], "ts20_control_interval_rejected")
        self.assertFalse(event["allocator_invoked"])
        self.assertFalse(event["command_published"])
        manifest = observer.close()
        self.assertTrue(manifest["interval_failure_recorded"])

    def test_non_due_pumps_do_not_replace_the_control_ring(self):
        loop, observer, counts, _, _ = self.make_loop([.02 + index*.00001 for index in range(70)])
        for _ in range(70):
            loop.pump()
        self.assertEqual(counts["original_pump"], 70)
        self.assertEqual(counts["read"], 1)
        self.assertEqual(len(observer.ring), 1)
        self.assertEqual(observer.ring[0]["pump_sequence"], 1)
        self.assertEqual(len(observer.recent_pumps), 64)
        self.assertEqual(observer.recent_pumps[0]["pump_sequence"], 7)
        self.assertEqual(observer.recent_pumps[-1]["pump_sequence"], 70)
        self.assertNotIn("state", observer.ring[0])
        self.assertFalse((self.output / "loop_timing_evidence.json").exists())
        observer.close()

    def test_observer_close_error_does_not_replace_original_error_or_block_actuator_seal(self):
        self.exercise_finalize_failure(OSError("observer disk error"))

    def test_incomplete_observer_manifest_rejects_gate_and_still_seals_actuator(self):
        self.exercise_finalize_failure(None)

    def test_complete_file_with_invalid_observations_cannot_receive_gate(self):
        self.exercise_finalize_failure(None, {"complete": True, "observations_valid": False})

    def exercise_finalize_failure(self, close_error, manifest=None):
        # Execute the exact production method body, isolating only unrelated
        # ROS/PX4 import dependencies from this postflight control-flow test.
        source = ROOT / "simulations/x650_ts20_hold_v1/run_case.py"
        tree = ast.parse(source.read_text(encoding="utf-8"))
        cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "Ts20Extension")
        method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "finalize")
        calls = []
        def close_observer():
            calls.append("observer_close")
            if close_error is not None:
                raise close_error
            return {"complete": False} if manifest is None else manifest
        def close_runtime():
            calls.append("runtime_close")
            return {"closed": True}
        def freeze(source, destination, key, end):
            calls.append(destination.name)
            destination.write_text("actual frozen fixture\n")
            return {"file": destination.name, "samples": 2}
        def write_json(path, value):
            path.write_text(json.dumps(value))
        namespace = {"Path": Path, "task": SimpleNamespace(sha=lambda path: "fixture_sha"),
                     "shared": SimpleNamespace(atomic_json=write_json)}
        exec(compile(ast.fix_missing_locations(ast.Module(body=[method], type_ignores=[])), str(source), "exec"), namespace)
        extension = SimpleNamespace(provenance={"ts20_operating_condition": {},
            "ts20_operating_condition_sha256": "fixture"}, loop_timing=SimpleNamespace(close=close_observer),
            runtime=SimpleNamespace(close=close_runtime), log_dir=self.output, _freeze_csv=freeze)
        meta = {"complete": True, "error": "original control failure", "native_recording": {"time_last_s": 2.}}
        stage = {"success": True, "complete": True, "passed": True, "artifacts": {}}
        namespace["finalize"](extension, meta, stage, self.output)
        self.assertEqual(meta["error"], "original control failure")
        self.assertIn("ts20_loop_timing_evidence_error", meta)
        self.assertFalse(meta["ts20_loop_timing_evidence"]["complete"])
        self.assertFalse(meta["complete"])
        self.assertFalse(stage["passed"])
        self.assertEqual(calls, ["observer_close", "runtime_close", "ts20_state.csv", "ts20_packets.csv"])
        self.assertTrue(meta["ts20_actuator"]["closed"])
        self.assertIn("ts20_recording_manifest.json", stage["artifacts"])
        self.assertFalse(json.loads((self.output / "ts20_recording_manifest.json").read_text())["complete"])

    def test_unrelated_exception_is_not_relabelled_as_dt_rejection(self):
        loop, observer, _, _, _ = self.make_loop([.02])
        original = RuntimeError("original callback failure")
        def fail():
            raise original
        observer.original["read"] = fail
        with self.assertRaisesRegex(RuntimeError, "original callback failure") as raised:
            loop.pump()
        self.assertIs(raised.exception, original)
        self.assertIsNone(observer.failure)
        self.assertEqual(loop.execution_trace.failure_count, 0)
        observer.close()

    def test_natural_gc_observer_is_bounded_and_removes_only_its_callback(self):
        sentinel = lambda phase, info: None
        gc.callbacks.append(sentinel)
        def remove_sentinel():
            for index, callback in enumerate(gc.callbacks):
                if callback is sentinel:
                    del gc.callbacks[index]
                    break
        self.addCleanup(remove_sentinel)
        callbacks_before = tuple(gc.callbacks)
        thresholds, enabled = gc.get_threshold(), gc.isenabled()
        _, observer, _, _, _ = self.make_loop([.02])
        self.assertEqual(sum(callback is observer._gc_callback for callback in gc.callbacks), 1)
        # Synthetic event delivery tests the observer only; no collection is
        # requested and no claim is made about collector performance.
        for index in range(70):
            generation = index % 3
            observer.on_gc("start", {"generation": generation})
            observer.on_gc("stop", {"generation": generation, "collected": 2, "uncollectable": 0})
        snapshot = observer.gc_snapshot()
        self.assertEqual(len(snapshot["completed_events"]), 64)
        self.assertGreaterEqual(sum(item["completed"] for item in snapshot["per_generation"]), 70)
        event = snapshot["completed_events"][-1]
        self.assertIn("start_perf_counter_s", event)
        self.assertIn("stop_thread_cpu_s", event)
        self.assertEqual(event["collected"], 2)
        observer.close()
        self.assertEqual(tuple(gc.callbacks), callbacks_before)
        self.assertEqual(gc.get_threshold(), thresholds)
        self.assertEqual(gc.isenabled(), enabled)

    def test_observation_faults_cannot_mask_original_guard_rejection(self):
        loop, observer, _, _, _ = self.make_loop([.02, .076])
        loop.pump()
        with patch.object(observer, "sample", side_effect=RuntimeError("observer clock error")), \
                patch("simulations.x650_ts20_hold_v1.loop_timing.time.perf_counter",
                      side_effect=RuntimeError("gc observation error")):
            observer.on_gc("start", {"generation": 0})
            with self.assertRaisesRegex(RuntimeError, "Control interval 0.0560s") as raised:
                loop.pump()
        self.assertFalse(raised.exception.diagnostics["ts20_loop_timing"]["allocator_invoked"])
        self.assertGreater(observer.gc_callback_errors, 0)
        self.assertGreater(observer.observation_errors, 0)
        observer.close()

    def test_postflight_file_error_still_removes_callback_and_restores_methods(self):
        loop, observer, _, _, original = self.make_loop([.02])
        with patch("pathlib.Path.open", side_effect=OSError("postflight file error")):
            with self.assertRaisesRegex(OSError, "postflight file error"):
                observer.close()
        self.assertFalse(any(callback is observer._gc_callback for callback in gc.callbacks))
        self.assertIs(loop.pump, original["pump"])
        self.assertIs(loop.ctx.vehicle._pump, original["original_pump"])
        observer.close()


if __name__ == "__main__":
    unittest.main()
