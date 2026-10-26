"""Counterexamples for independent replay acceptance; no ROS/solver is mocked as a pass."""
from __future__ import annotations
from copy import deepcopy
import importlib.util
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location("audit_recovery", ROOT/"scripts/audit_x650_recovery_replay.py")
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def fixture():
    return {"config": {"nodes": 2, "node_dt": .04, "max_dt": .05,
        "feasibility_tolerance": 2e-7, "velocity_limit": [.55, .35, .45, .45, .35],
        "acceleration_limit": [.45, .30, .45, .60, .60], "q_lower": [-1., -1.],
        "q_upper": [1., 1.], "kp": [6., 6.], "kd": [.25, .25], "bus_period": .005},
        "memory": {"step_count": 0, "anchor": {"position": [0.]*3, "rotation": [1., 0., 0., 0., 1., 0., 0., 0., 1.], "q": [0., 0.]},
                   "previous_command": [0.]*5, "last_t": 0., "plan": [], "previous_stamp": 0.},
        "input": {"t": 0., "dt": .02, "state": {"control_time": 0., "joint_stamp": 0., "stamp": 0.},
                  "context": {"reference_time": -.005, "reference": [0., 0.],
                     "commands": [{"time": -.005, "velocity": [0., 0.]}],
                     "packets": [{"send_time": -.005, "q_ref": [0., 0.], "dq_ref": [0., 0.],
                                  "tau_ff": [0., 0.], "kp": [6., 6.], "kd": [.25, .25]}]}}}


def accepted(f):
    rows = []
    for i in range(2):
        for j in range(2):
            rows.append([1. if k == 5*i+3+j else 0. for k in range(10)])
    rows += [[1. if k == j else 0. for k in range(10)] for j in range(5)]
    after = deepcopy(f["memory"])
    after.update(step_count=1, last_t=0., previous_command=[0.]*5, plan=[0.]*10)
    d = {"accepted": True, "nonlinear_passed": True, "actual_bounds": True,
         "previous_plan_fallback": False, "compute_ms": 3., "subproblems": 1,
         "q_linearization_error": 0., "dq_linearization_error": 0.,
         "qp_rows": 9, "qp_columns": 10, "C": sum(rows, []),
         "lower": [-.02]*4+[-1.]*5, "upper": [.02]*4+[1.]*5,
         "solution": [0.]*10, "plan": [0.]*10, "candidate_envelope": [0.]*8,
         "candidate_packets": deepcopy(f["input"]["context"]["packets"]),
         "query_times": [.02, .06], "candidate_states": [0.]*8,
         "predicted_states": [0.]*24, "linearized_states": [0.]*24, "recovery_diagnostics_available": True,
         "recovery_triggered": False}
    for i in range(13):
        packet = deepcopy(f["input"]["context"]["packets"][-1])
        packet["send_time"] = i*.005
        d["candidate_packets"].append(packet)
    return {"kind": "x650_cpp_recovery_raw_replay_v1", "schema_version": 1, "complete": True,
            "input_sha256": "f"*64, "loaded_code": {k: {"sha256": "a"*64} for k in ("core", "optimization", "predictor")},
            "scope": {"offline_fixed_snapshot": True, "actuator_io": False,
                      "closed_loop_or_physical_validation": False, "injected_delay": False,
                      "original_core_deadline_ms": 50., "initial_trust_radius": .02},
            "memory_before": deepcopy(f["memory"]), "memory_after": after, "memory_unchanged": False,
            "memory_commit_valid": True, "returned": True, "core_error": False,
            "unexpected_exception": False, "error": None, "core_step_wall_ms": 4.,
            "output": {k: [0.]*n for k, n in audit.OUTPUT_DIMENSIONS.items()}, "diagnostics": d}


def rejected(f, error="QP infeasible"):
    out = accepted(f)
    out.update(memory_after=deepcopy(f["memory"]), memory_unchanged=True,
               memory_commit_valid=False, returned=False, core_error=True, output=None, error=error)
    return out


def recovered(f):
    out = accepted(f)
    out["diagnostics"].update(recovery_triggered=True, recovery_reason="QP infeasible",
        recovery_trust_radius=.02, recovery_stopping_violation=0., subproblems=2,
        recovery_attempts=[{"centre_kind": "slew_limited_brake", "rejection": "",
            "nonlinear_passed": True, "original_constraints_passed": True,
            "trusted": True, "projection_attempted": False, "projection_solved": False,
            "original_primal_violation": 0., "stopping_violation": 0., "elapsed_ms": 2.,
            "centre_plan": [0.]*10, "envelope": [0.]*8, "substeps": 60,
            "node_times": [.02, .06], "node_states": [0.]*8, "node_references": [0.]*4}])
    return out


class RecoveryReplayAuditTests(unittest.TestCase):
    def setUp(self):
        self.f = fixture()
        self.libraries = {k: "a"*64 for k in ("core", "optimization", "predictor")}

    def run_audit(self, old, new, expectation):
        return audit.audit(self.f, old, new, "f"*64, self.libraries, self.libraries, expectation)

    def test_normal_result_and_memory_must_be_numerically_unchanged(self):
        old = accepted(self.f)
        result = self.run_audit(old, deepcopy(old), "unchanged")
        self.assertTrue(result["passed"])
        self.assertEqual(max(result["maximum_absolute_errors"].values()), 0.)

    def test_normal_output_difference_is_not_hidden_by_accepted_flag(self):
        old, new = accepted(self.f), accepted(self.f)
        new["output"]["tip_actual"][0] = .001
        with self.assertRaisesRegex(ValueError, "Normal result changed"):
            self.run_audit(old, new, "unchanged")

    def test_raw_original_failure_and_verified_recovery(self):
        result = self.run_audit(rejected(self.f), recovered(self.f), "recovered")
        self.assertTrue(result["passed"])
        self.assertEqual(len(result["trusted_centres"]), 1)
        self.assertFalse(result["physical_validation_claimed"])
        self.assertTrue(result["new"]["linearization_discrepancy_independently_recomputed"])

    def test_linearization_error_recomputed_from_pre_replacement_native_capture(self):
        new = recovered(self.f)
        new["diagnostics"]["linearized_states"][6] = .001
        with self.assertRaisesRegex(ValueError, "discrepancy differs from native evidence"):
            self.run_audit(rejected(self.f), new, "recovered")
        new["diagnostics"]["q_linearization_error"] = .001
        self.assertTrue(self.run_audit(rejected(self.f), new, "recovered")["passed"])
        del new["diagnostics"]["linearized_states"]
        with self.assertRaisesRegex(ValueError, "lacks original linearization capture"):
            self.run_audit(rejected(self.f), new, "recovered")

    def test_old_abi_without_linearization_capture_remains_readable(self):
        old = accepted(self.f)
        del old["diagnostics"]["linearized_states"]
        result = self.run_audit(old, accepted(self.f), "unchanged")
        self.assertFalse(result["old"]["linearization_discrepancy_independently_recomputed"])
        self.assertTrue(result["new"]["linearization_discrepancy_independently_recomputed"])

    def test_reported_success_does_not_hide_original_qp_violation(self):
        new = recovered(self.f)
        new["diagnostics"]["lower"][-1] = .1
        with self.assertRaisesRegex(ValueError, "original assembled QP row"):
            self.run_audit(rejected(self.f), new, "recovered")

    def test_trust_radius_must_be_verified_in_actual_matrix(self):
        new = recovered(self.f)
        new["diagnostics"]["upper"][0] = .20
        with self.assertRaisesRegex(ValueError, "0.02 trust"):
            self.run_audit(rejected(self.f), new, "recovered")

    def test_cannot_reset_actual_reference_or_packet_history(self):
        new = recovered(self.f)
        new["diagnostics"]["candidate_packets"][0]["q_ref"][0] = .01
        with self.assertRaisesRegex(ValueError, "actual/in-flight MIT packet"):
            self.run_audit(rejected(self.f), new, "recovered")
        new = recovered(self.f)
        new["diagnostics"]["recovery_attempts"][0]["node_references"][0] = .01
        with self.assertRaisesRegex(ValueError, "reference was reset"):
            self.run_audit(rejected(self.f), new, "recovered")

    def test_future_packet_requires_real_reference_integration_and_full_horizon(self):
        new = recovered(self.f)
        new["diagnostics"]["candidate_packets"][2]["q_ref"][0] = .01
        with self.assertRaisesRegex(ValueError, "integrated from actual reference"):
            self.run_audit(rejected(self.f), new, "recovered")
        new = recovered(self.f)
        new["diagnostics"]["candidate_packets"] = new["diagnostics"]["candidate_packets"][:2]
        with self.assertRaisesRegex(ValueError, "cover the original horizon"):
            self.run_audit(rejected(self.f), new, "recovered")

    def test_trusted_centre_reserve_is_independent_of_reported_pass(self):
        new = recovered(self.f)
        new["diagnostics"]["recovery_attempts"][0]["node_states"][0] = 2.
        with self.assertRaisesRegex(ValueError, "centre violates original nonlinear stopping"):
            self.run_audit(rejected(self.f), new, "recovered")

    def test_final_nonlinear_reserve_uses_actual_not_linearized_states(self):
        new = recovered(self.f)
        # Small linearization error is within old 6 mrad / 0.10 rad/s gates,
        # but near a boundary it can violate the original nonlinear reserve.
        self.f["config"]["q_upper"][0] = .04
        new["diagnostics"]["candidate_states"][2] = .09
        new["diagnostics"]["dq_linearization_error"] = .09
        with self.assertRaisesRegex(ValueError, "candidate violates original stopping"):
            self.run_audit(rejected(self.f), new, "recovered")

    def test_rejected_call_cannot_commit_even_if_step_count_is_same(self):
        new = rejected(self.f)
        new["memory_after"]["previous_stamp"] = .01
        new["memory_unchanged"] = False
        with self.assertRaisesRegex(ValueError, "Rejected/timeout call committed"):
            self.run_audit(rejected(self.f), new, "rejected")

    def test_timeout_must_reject_and_rollback_full_memory(self):
        result = self.run_audit(rejected(self.f), rejected(self.f, "core deadline exceeded"), "rejected")
        self.assertTrue(result["new"]["memory_rollback_verified"])
        new = recovered(self.f)
        new["core_step_wall_ms"] = 51.
        with self.assertRaisesRegex(ValueError, "50 ms"):
            self.run_audit(rejected(self.f), new, "recovered")

    def test_wrong_input_or_loaded_library_is_rejected(self):
        new = recovered(self.f)
        new["input_sha256"] = "b"*64
        with self.assertRaisesRegex(ValueError, "raw fixture"):
            self.run_audit(rejected(self.f), new, "recovered")
        new = recovered(self.f)
        new["loaded_code"]["core"]["sha256"] = "b"*64
        with self.assertRaisesRegex(ValueError, "Loaded library differs"):
            self.run_audit(rejected(self.f), new, "recovered")

    def test_nan_cannot_pass_finite_output_check(self):
        new = recovered(self.f)
        new["output"]["tip_actual"][0] = float("nan")
        with self.assertRaisesRegex(ValueError, "finite number"):
            self.run_audit(rejected(self.f), new, "recovered")


if __name__ == "__main__":
    unittest.main()
