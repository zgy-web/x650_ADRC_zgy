"""Transactional evidence tests; no plant, solver timing or flight acceptance."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from x650_wholebody.mpc import MpcController, MpcSolveError
from x650_wholebody.ts20_mpc import Ts20MpcController


MEMORY = ("_plan", "_plan_t", "_plan_h", "previous_command", "last_t", "_previous_stamp",
          "step_count", "_joint_history", "_chain_clock_origin")


def fixture():
    # Bypass constructor/model setup: only the wrapper transaction is tested.
    controller = object.__new__(Ts20MpcController)
    controller._plan = np.arange(10, dtype=float).reshape(2, 5)/100
    controller._plan_t, controller._plan_h = 12., np.array([.02, .04])
    controller.previous_command = controller._plan[0].copy()
    controller.last_t, controller._previous_stamp = 12., 51.99
    controller.step_count = 7
    controller._joint_history = [(11.98, np.array([.01, -.02]))]
    controller._chain_clock_origin = 40.
    controller._frozen_context, controller._linearization_plan = None, None
    context = {
        "reference_state": {"time_s": 12.032, "r": np.array([.02, -.03])},
        "packet_history": [{"send_time_s": 12.03,
                            "q_ref": np.array([.0201, -.0299]),
                            "dq_ref": np.array([.03, -.04]),
                            "tau_ff": np.zeros(2), "kp": np.full(2, 6.),
                            "kd": np.full(2, .25)}],
        "command_history": [(12.028, np.array([.03, -.04]))],
        "bus_origin_s": -40.,
    }
    controller.context_provider = lambda state, t: context
    return controller, context


def counters(before=(2, 3, 4), after=(3, 3, 5)):
    return [[{"collections": value} for value in values] for values in (before, after)]


class Ts20MpcFailureEvidenceTests(unittest.TestCase):
    def assert_nested_equal(self, actual, expected):
        if isinstance(expected, np.ndarray):
            np.testing.assert_array_equal(actual, expected)
        elif isinstance(expected, dict):
            self.assertEqual(set(actual), set(expected))
            for name in expected:
                self.assert_nested_equal(actual[name], expected[name])
        elif isinstance(expected, (tuple, list)):
            self.assertEqual(len(actual), len(expected))
            for left, right in zip(actual, expected):
                self.assert_nested_equal(left, right)
        else:
            self.assertEqual(actual, expected)

    def test_outer_deadline_preserves_passed_inner_result_and_exact_replay_copy(self):
        controller, context = fixture()
        original = {name: deepcopy(getattr(controller, name)) for name in MEMORY}
        frozen = deepcopy(context)
        inner = {"accepted_optimum": True, "compute_ms": 8.,
                 "ts20_nonlinear_check_passed": True,
                 "ts20_candidate_q_linearization_error_rad": .0002,
                 "ts20_candidate_dq_linearization_error_rad_s": .001,
                 "predicted_commands": np.full((2, 5), .03),
                 "predicted_states": np.ones((2, 12))}
        expected_inner = deepcopy(inner)

        def returned(_self, *_):
            controller._plan[:] = 99.
            controller.previous_command[:] = 98.
            controller.step_count += 1
            controller._joint_history.append((12.04, np.ones(2)))
            return {"diagnostics": inner}

        with patch.object(MpcController, "step", returned), \
                patch("x650_wholebody.ts20_mpc.perf_counter", side_effect=[0., .053]), \
                patch("x650_wholebody.ts20_mpc.thread_time", side_effect=[1., 1.001, 1.009, 1.010]), \
                patch("x650_wholebody.ts20_mpc.gc.get_stats", side_effect=counters()):
            with self.assertRaisesRegex(MpcSolveError, "deadline exceeded") as raised:
                controller.step(12.04, .04, object(), object())
        diagnostic = raised.exception.diagnostics
        self.assertFalse(diagnostic["accepted_optimum"])
        self.assertFalse(diagnostic["ts20_command_accepted"])
        self.assertTrue(diagnostic["ts20_nonlinear_check_passed"])
        self.assertAlmostEqual(diagnostic["ts20_total_compute_ms"], 53.)
        replay = diagnostic["ts20_failure_replay"]
        self.assertTrue(replay["copied_after_final_rejection"])
        self.assertFalse(replay["command_dispatched"])
        self.assert_nested_equal(replay["frozen_context"], frozen)
        self.assert_nested_equal(replay["memory_before"], original)
        self.assert_nested_equal(replay["inner_allocator_diagnostics"], expected_inner)
        for name in MEMORY:
            self.assert_nested_equal(getattr(controller, name), original[name])
        self.assertIsNone(controller._frozen_context)
        self.assertIsNone(controller._linearization_plan)
        # Evidence owns its arrays independently of the provider, rejected
        # allocator output and restored live controller memory.
        context["packet_history"][0]["q_ref"][:] = -99.
        inner["predicted_commands"][:] = -98.
        controller.previous_command[:] = -97.
        self.assert_nested_equal(replay["frozen_context"], frozen)
        self.assert_nested_equal(replay["memory_before"], original)
        self.assert_nested_equal(replay["inner_allocator_diagnostics"], expected_inner)

    def test_unsuccessful_inner_result_remains_rejected_and_rolls_back(self):
        controller, context = fixture()
        original = {name: deepcopy(getattr(controller, name)) for name in MEMORY}
        frozen = deepcopy(context)

        def rejected(_self, *_):
            controller.previous_command[:] = 9.
            raise MpcSolveError("original nonlinear rejection", {
                "accepted_optimum": False, "compute_ms": 9.,
                "ts20_nonlinear_check_passed": False,
                "ts20_candidate_actual_bounds_passed": False})

        with patch.object(MpcController, "step", rejected), \
                patch("x650_wholebody.ts20_mpc.perf_counter", side_effect=[0., .012]), \
                patch("x650_wholebody.ts20_mpc.thread_time", side_effect=[1., 1.001, 1.009, 1.010]), \
                patch("x650_wholebody.ts20_mpc.gc.get_stats", side_effect=counters()):
            with self.assertRaisesRegex(MpcSolveError, "original nonlinear rejection") as raised:
                controller.step(12.04, .04, object(), object())
        diagnostic = raised.exception.diagnostics
        self.assertFalse(diagnostic["accepted_optimum"])
        self.assertFalse(diagnostic["ts20_nonlinear_check_passed"])
        self.assertEqual(len(diagnostic["ts20_relinearization_attempts"]), 1)
        self.assertFalse(diagnostic["ts20_relinearization_attempts"][0]["passed"])
        replay = diagnostic["ts20_failure_replay"]
        self.assertIsNone(replay["inner_allocator_diagnostics"])
        self.assert_nested_equal(replay["frozen_context"], frozen)
        self.assert_nested_equal(replay["memory_before"], original)
        for name in MEMORY:
            self.assert_nested_equal(getattr(controller, name), original[name])

    def test_success_observes_counts_and_thread_cpu_without_copying_replay_or_gc_policy(self):
        controller, _ = fixture()
        inner = {"accepted_optimum": True, "compute_ms": 8.,
                 "ts20_nonlinear_check_passed": True,
                 "ts20_candidate_q_linearization_error_rad": .0002,
                 "ts20_candidate_dq_linearization_error_rad_s": .001}
        callback_ids_before = tuple(id(value) for value in __import__("gc").callbacks)
        order = []

        def stats():
            order.append("gc")
            return next(stats.values)
        stats.values = iter(counters())

        def clock():
            order.append("wall")
            return next(clock.values)
        clock.values = iter([0., .020])

        def returned(_self, *_):
            controller.step_count += 1
            return {"diagnostics": inner}

        with patch.object(MpcController, "step", returned), \
                patch("x650_wholebody.ts20_mpc.perf_counter", side_effect=clock), \
                patch("x650_wholebody.ts20_mpc.thread_time", side_effect=[1., 1.001, 1.009, 1.010]), \
                patch("x650_wholebody.ts20_mpc.gc.get_stats", side_effect=stats), \
                patch("x650_wholebody.ts20_mpc.gc.collect", side_effect=AssertionError("must not collect")), \
                patch("x650_wholebody.ts20_mpc.gc.disable", side_effect=AssertionError("must not disable")), \
                patch("x650_wholebody.ts20_mpc.gc.get_objects", side_effect=AssertionError("must not enumerate")):
            out = controller.step(12.04, .04, object(), object())
        self.assertEqual(order, ["wall", "gc", "gc", "wall"])
        self.assertEqual(controller.step_count, 8)
        self.assertTrue(out["diagnostics"]["accepted_optimum"])
        self.assertNotIn("ts20_failure_replay", out["diagnostics"])
        observed = out["diagnostics"]["ts20_timing_observation"]
        self.assertEqual(observed["gc_collection_count_delta"], [1, 0, 1])
        self.assertFalse(observed["gc_policy_changed"])
        self.assertEqual(observed["inner_allocator_compute_ms"], 8.)
        self.assertAlmostEqual(observed["inner_allocator_thread_cpu_ms"], 8.)
        self.assertAlmostEqual(observed["outer_thread_cpu_ms"], 10.)
        self.assertAlmostEqual(observed["outer_compute_ms"], 20.)
        self.assertEqual(tuple(id(value) for value in __import__("gc").callbacks), callback_ids_before)


if __name__ == "__main__":
    unittest.main()
