"""Status source-rate guards; synthetic callbacks never certify a native flight."""
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parents[1]
ROOT = HERE.parents[1]
for directory in (HERE, ROOT/"control_packages/x650_wholebody_v1"):
    sys.path.insert(0, str(directory))
import run_case as runner
from test_recording import ros_message


def node():
    truth = ros_message(1., yaw=0.)
    return NS(data={"truth": truth, "joints": NS(header=truth.header),
                   "position": NS(timestamp=1000000), "land": NS(timestamp=1000000, landed=True),
                   "status": NS(timestamp=1000000, arming_state=1, failsafe=False, nav_state=5)},
              received={key: 100. for key in ("truth", "joints", "position", "land", "status")},
              spin_once=lambda **kwargs: None,
              joint_values=lambda: {name: NS(position_rad=0., velocity_rad_s=0.) for name in runner.JOINTS})


class StatusReceiptGuardTests(unittest.TestCase):
    def live(self):
        value = node()
        watcher = runner.StatusReceiptWatchdog(value)
        value.received["status"] = 100.1
        watcher.observe(value, 1.)
        return value, watcher

    def test_slow_simulation_source_period_does_not_become_wall_staleness(self):
        value, watcher = self.live()
        value.received.update(truth=103.4, joints=103.4, position=103.4)
        watcher.observe(value, 1.504)
        with patch.object(runner.time, "monotonic", return_value=103.4):
            runner.require_periodic_telemetry(value, status_watchdog=watcher, sim_s=1.504)
            with self.assertRaisesRegex(RuntimeError, "Stale flight telemetry: status"):
                runner.require_periodic_telemetry(value)  # legacy contract remains unchanged
        diagnostic = watcher.diagnostics(1.504, wall_now=103.4)
        self.assertAlmostEqual(diagnostic["status_receipt_age_sim_s"], .504)
        self.assertAlmostEqual(diagnostic["status_receipt_age_wall_s"], 3.3)

    def test_initial_cached_message_and_duplicate_receipt_are_not_new_callbacks(self):
        value = node()
        watcher = runner.StatusReceiptWatchdog(value)
        watcher.observe(value, 1.)
        with self.assertRaisesRegex(RuntimeError, "No new status callback"):
            watcher.check(1., wall_now=100.)
        value.received["status"] = 100.1
        watcher.observe(value, 1.)
        watcher.observe(value, 2.5)
        self.assertEqual(watcher.sequence, 1)
        self.assertEqual(watcher.check(2.5, wall_now=102.), 1.5)
        with self.assertRaisesRegex(RuntimeError, "simulation seconds"):
            watcher.check(2.501, wall_now=102.)

    def test_independent_wall_guard_rejects_stall_even_when_simulation_pauses(self):
        _, watcher = self.live()
        self.assertEqual(watcher.check(1., wall_now=110.1), 0.)
        with self.assertRaisesRegex(RuntimeError, "wall seconds"):
            watcher.check(1., wall_now=110.101)
        with self.assertRaisesRegex(RuntimeError, "future callback"):
            watcher.check(1., wall_now=100.)

    def test_missing_nonfinite_and_regressing_receipts_fail_closed(self):
        for receipt in (None, float("nan"), float("inf"), True, -1., 0., 99.):
            value, watcher = self.live()
            value.received["status"] = receipt
            with self.subTest(receipt=receipt), self.assertRaises(RuntimeError):
                watcher.observe(value, 1.1)
        value, watcher = self.live()
        value.data.pop("status")
        with self.assertRaisesRegex(RuntimeError, "no corresponding message"):
            watcher.observe(value, 1.1)

    def test_bad_simulation_clock_fails_but_dds_epoch_changes_are_diagnostic_only(self):
        value, watcher = self.live()
        for epoch in (1792554749151522, 2, 0):
            value.data["status"].timestamp = epoch
            watcher.observe(value, 1.)
            watcher.check(1., wall_now=101.)
        self.assertEqual(watcher.sequence, 1)
        for clock in (float("nan"), float("inf"), True, None, -.1, .99):
            with self.subTest(clock=clock), self.assertRaises(RuntimeError):
                watcher.observe(value, clock)

    def test_fast_topic_two_second_wall_guards_are_unchanged(self):
        for key in ("truth", "joints", "position"):
            value, watcher = self.live()
            value.received.update(truth=104., joints=104., position=104.)
            value.received[key] = 101.9
            with patch.object(runner.time, "monotonic", return_value=104.):
                with self.subTest(key=key), self.assertRaisesRegex(RuntimeError, "Stale flight telemetry: "+key):
                    runner.require_periodic_telemetry(value, status_watchdog=watcher, sim_s=1.5)

    def test_upper_pump_observes_and_checks_status_before_next_control_deadline(self):
        with tempfile.TemporaryDirectory() as tmp, patch.object(runner.time, "monotonic", return_value=101.):
            value = node()
            clock = [1.]
            ctx = NS(node=value, vehicle=NS(_pump=lambda: None))
            capture = NS(now=lambda: clock[0], check=lambda: None)
            cfg = {"research_profile": {"name": "precision_ev_upper_v2"}}
            loop = runner.ControlLoop(ctx, cfg, NS(joint_lower=[-1., -.5], joint_upper=[1., 1.]), capture, {}, Path(tmp))
            try:
                self.assertIsInstance(loop.status_watchdog, runner.StatusReceiptWatchdog)
                loop.next_control = 100.
                value.received["status"] = 100.1
                with patch.dict(sys.modules, {"pymavlink": NS(mavutil=NS())}):
                    loop.pump()
                    self.assertEqual(loop.status_watchdog.sequence, 1)
                    self.assertEqual(loop.rows, 0)
                    loop.strict, clock[0] = True, 2.501
                    with self.assertRaisesRegex(RuntimeError, "Stale status receipt"):
                        loop.pump()
            finally:
                loop.close()

    def test_status_guard_does_not_relax_failsafe_or_offboard_state(self):
        for failsafe, nav_state in ((True, 14), (False, 5)):
            with tempfile.TemporaryDirectory() as tmp, patch.object(runner.time, "monotonic", return_value=101.):
                value = node()
                ctx = NS(node=value, vehicle=NS(_pump=lambda: None))
                capture = NS(now=lambda: 1., check=lambda: None)
                cfg = {"control_period_s": .02, "research_profile": {"name": "precision_ev_upper_v2"}}
                loop = runner.ControlLoop(ctx, cfg, NS(joint_lower=[-1., -.5], joint_upper=[1., 1.]), capture, {}, Path(tmp))
                try:
                    value.received.update(status=100.1, land=100.1)
                    loop.read()
                    loop.strict, loop.offboard = True, True
                    value.data["status"].failsafe = failsafe
                    value.data["status"].nav_state = nav_state
                    with self.subTest(failsafe=failsafe, nav_state=nav_state), self.assertRaisesRegex(RuntimeError, "failsafe or unexpected offboard"):
                        loop.read()
                finally:
                    loop.close()

    def test_nav_confirmation_requires_new_status_and_land_with_correct_states(self):
        with patch.object(runner.time, "monotonic", return_value=101.):
            value, status = self.live()
            land = runner.LandReceiptWatchdog(value)
            value.received["land"] = 100.1
            land.observe(value, 1.)
            marker = runner.transition_marker(value)
            calls = []
            def wait(predicate, timeout, description):
                calls.append(predicate())  # initial cached landed+disarmed are insufficient
                value.received["status"] = 101.1
                status.observe(value, 1.1)
                with patch.object(runner.time, "monotonic", return_value=101.2):
                    calls.append(predicate())  # new status alone is insufficient
                    value.received["land"] = 101.15
                    land.observe(value, 1.1)
                    value.data["status"].arming_state = 2
                    calls.append(predicate())  # wrong arm state
                    value.data["status"].arming_state = 1
                    calls.append(predicate())
            result = runner.confirm_land_state_after_command(NS(node=value, vehicle=NS(wait=wait)), marker,
                landed=True, watchdog=land, status_watchdog=status, sim_clock=lambda: 1.1)
            self.assertEqual(calls, [False, False, False, True])
            self.assertTrue(result["after"]["requires_new_report_after_command"])
            self.assertIn("status_receipt_age_sim_s", result["after"])

    def test_contract_records_source_rate_basis_without_changing_land_guard(self):
        contract = runner.StatusReceiptWatchdog.contract()
        self.assertEqual(contract["source_period_sim_s"], .5)
        self.assertEqual(contract["dds_rate_limit_hz"], 5.)
        self.assertEqual(contract["timeout_sim_s"], 3*contract["source_period_sim_s"])
        self.assertEqual(contract["timeout_wall_s"], 10.)
        self.assertEqual(runner.LandReceiptWatchdog(node()).timeout_sim_s, 3.)


if __name__ == "__main__":
    unittest.main()
