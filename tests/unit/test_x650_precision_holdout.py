"""Synthetic audit contracts only; no generated flight or performance evidence."""
import copy
import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location("holdout_plot", ROOT/"scripts/plot_x650_precision_holdout.py")
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class PrecisionHoldoutTests(unittest.TestCase):
    def fixture(self):
        frozen = dict(selection_frozen_before_holdout=True, holdout_runs_used=[], global_optimality_claimed=False,
            created_utc="2026-10-21T08:08:49+00:00", selected_candidate="velocity_high",
            selected_parameters={"MPC_XY_VEL_P_ACC": 2.2}, rows=[dict(label="ev_S2", candidate="velocity_high")])
        def run(name, gain):
            params = {"OTHER_"+str(i): float(i) for i in range(41)}
            params["MPC_XY_VEL_P_ACC"] = gain
            return dict(name=name, data=dict(controller_metadata=dict(actual_pid_parameters=params,
                task_start_sim_s=10., experiment_end_sim_s=66.),
                precision_audit=dict(full_experiment_ulog_guard=dict(verified=True, analysis_window_sim_s=[10., 66.]))))
        baseline, selected = run("ev_S0", 1.8), run("ev_S2", 2.2)
        repeats = [run("ev_repeat_baseline", 1.8), run("ev_repeat_selected", 2.2)]
        manifests = [dict(started_wall_time=1792571000., scenario_returncode=0) for _ in repeats]
        return frozen, baseline, selected, repeats, manifests

    def test_independent_matching_runs_are_audited_without_reselection(self):
        result = audit.verify_holdout(*self.fixture())
        self.assertTrue(result["independent_after_selection_freeze"])
        self.assertFalse(result["reselected_using_holdout"])

    def test_restored_or_other_gain_values_cannot_replace_assigned_condition(self):
        args = self.fixture()
        args[3][1]["data"]["controller_metadata"]["actual_pid_parameters"]["MPC_XY_VEL_P_ACC"] = 1.8
        with self.assertRaisesRegex(ValueError, "42 measurement-period gains"):
            audit.verify_holdout(*args)

    def test_early_or_failed_run_is_not_independent_holdout(self):
        args = self.fixture()
        args[4][0]["started_wall_time"] = 0.
        with self.assertRaisesRegex(ValueError, "started after"):
            audit.verify_holdout(*args)
        args = self.fixture()
        args[4][1]["scenario_returncode"] = 1
        with self.assertRaisesRegex(ValueError, "completed run"):
            audit.verify_holdout(*args)

    def test_primary_hover_guard_does_not_certify_step_window(self):
        args = self.fixture()
        args[3][1]["data"]["precision_audit"]["full_experiment_ulog_guard"]["analysis_window_sim_s"] = [10., 30.]
        with self.assertRaisesRegex(ValueError, "appended steps"):
            audit.verify_holdout(*args)


if __name__ == "__main__":
    unittest.main()
