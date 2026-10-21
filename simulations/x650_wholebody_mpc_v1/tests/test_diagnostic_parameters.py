"""Typed wire transactions and crash-persistent interlock, without a vehicle."""
from copy import deepcopy
import json
from pathlib import Path
import struct
import sys
import tempfile
import time
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
from diagnostic_parameters import (PROFILE, PRECISION_PROFILE, PRECISION_REQUIRED, TYPES, DiagnosticParameterSession,
    PID_CANDIDATES, PID_PARAMETERS, assert_no_pending_journal, expected_record, validate_profile, profile_name,
    validate_pid_candidate, pid_candidate_name, load_parameter_evidence)


class ParameterFixture:
    def __init__(self, precision=False, full_pid=False):
        self.values = {name: expected_record(value, kind) for name, value, kind in (
            ("EKF2_EV_CTRL", 0, 6), ("EKF2_EV_NOISE_MD", 0, 6), ("EKF2_EVP_NOISE", .1, 9),
            ("EKF2_GPS_CTRL", 7, 6), ("EKF2_HGT_REF", 1, 6),
            ("MPC_XY_P", 1.5, 9), ("MPC_Z_P", 1., 9), ("MC_ROLL_P", 6.5, 9))}
        if precision or full_pid:
            self.values.update({"EKF2_BARO_CTRL": expected_record(1, 6), "EKF2_EVV_NOISE": expected_record(.1, 9)})
        if precision:
            self.values.update({name: expected_record(value, 9) for name, value in PRECISION_REQUIRED.items()})
        if full_pid:
            for name in load_parameter_evidence().CORE_CONTROL_PARAMETERS:
                self.values.setdefault(name, expected_record(1, 6) if name in ("MC_AIRMODE", "MPC_USE_HTE") else expected_record(.1, 9))
            self.values.update({name: expected_record(value, 9) for name, value in PID_CANDIDATES["baseline"].items()})
        self.original = deepcopy(self.values)
        self.writes, self.ignore, self.journal = [], None, None
        self.save_failure = False
        self.sim = 1.
        node = NS(data={"status": NS(arming_state=1), "land": NS(landed=True)},
                  received={"status": time.monotonic(), "land": time.monotonic()}, sim_time_s=lambda: self.sim)
        def wait(predicate, timeout, description):
            for _ in range(4):
                self.sim += .004
                node.received.update(status=max(time.monotonic(), node.received["status"]+.0001),
                                     land=max(time.monotonic(), node.received["land"]+.0001))
                if predicate():
                    return
            raise RuntimeError("unit wait did not converge")
        self.ctx = NS(node=node, model={"model": "unit_x650", "model_sha256": "a"*64},
            vehicle=NS(wait=wait, _pump=lambda: None, ack={}, target=(1, 1), link=NS(mav=NS(param_set_send=self.send))))
        def no_storage_ack(command, params):
            raise AssertionError("MAV_CMD_PREFLIGHT_STORAGE ACK must never certify persistence")
        self.ctx.vehicle._send_command = no_storage_ack

    def send(self, sysid, compid, name, wire, kind):
        name = name.decode()
        self.writes.append(name)
        if self.journal is not None:
            assert self.journal.exists(), "journal must precede first real write"
        if self.ignore == name:
            return
        raw = struct.pack("<f", wire)
        self.values[name] = {"param_type": kind, "raw_value_hex": raw.hex(),
            "value": struct.unpack("<i" if kind == 6 else "<f", raw)[0]}

    def read(self, vehicle, names):
        return {name: deepcopy(self.values[name]) for name in names}, {}, {name: 1 for name in names}

    def session(self, folder, profile=PROFILE, pid_candidate=None):
        self.journal = folder/"runtime/restore_pending.json"
        def saver(session):
            if self.save_failure:
                raise RuntimeError("synchronous storage save failed")
            return {"returncode": 0, "synchronous_default_save": True, "test_fixture": True}
        return DiagnosticParameterSession(self.ctx, profile, folder, reader=self.read,
            protected_pid_records={name: record for name, record in self.values.items() if name.startswith(("MC_", "MPC_"))},
            journal_path=self.journal, storage_saver=saver, pid_candidate=pid_candidate)


class DiagnosticParameterTests(unittest.TestCase):
    def test_new_reports_waits_through_initial_missing_or_stale_before_any_write(self):
        for initial in ("missing", "stale"):
            with self.subTest(initial=initial), tempfile.TemporaryDirectory() as tmp, patch(
                    "diagnostic_parameters.time.monotonic", return_value=100.):
                fixture = ParameterFixture()
                node = fixture.ctx.node
                node.received.update(status=90., land=90.)
                if initial == "missing":
                    node.data.clear()
                session = fixture.session(Path(tmp))
                calls = []
                def wait(predicate, timeout, description):
                    self.assertFalse(predicate())  # No pump yet: formerly raised.
                    self.assertEqual(fixture.writes, [])
                    node.data["status"] = NS(arming_state=1)
                    node.received["status"] = 100.
                    self.assertFalse(predicate())  # A new status alone is insufficient.
                    self.assertEqual(fixture.writes, [])
                    node.data["land"] = NS(landed=True)
                    node.received["land"] = 100.
                    self.assertTrue(predicate())
                    calls.append(description)
                fixture.ctx.vehicle.wait = wait
                session.apply()
                self.assertEqual(len(calls), 1)
                self.assertTrue(session.document["applied_verified"])
                self.assertEqual(set(fixture.writes), set(PROFILE))

    def test_new_reports_forever_stale_times_out_without_any_parameter_write(self):
        with tempfile.TemporaryDirectory() as tmp, patch("diagnostic_parameters.time.monotonic", return_value=100.):
            fixture = ParameterFixture()
            fixture.ctx.node.received.update(status=90., land=90.)
            session = fixture.session(Path(tmp))
            checks = []
            def wait(predicate, timeout, description):
                self.assertEqual(timeout, 180.)
                for _ in range(4):
                    checks.append(predicate())
                raise RuntimeError("Timeout: " + description)
            fixture.ctx.vehicle.wait = wait
            with self.assertRaisesRegex(RuntimeError, "Timeout: new disarmed"):
                session.apply()
            self.assertEqual(checks, [False] * 4)
            self.assertEqual(fixture.writes, [])
            self.assertIsNone(session.original)
            self.assertFalse(fixture.journal.exists())

    def test_new_reports_armed_or_airborne_refuses_immediately_even_with_other_topic_missing(self):
        for unsafe in ("armed", "airborne"):
            with self.subTest(unsafe=unsafe), tempfile.TemporaryDirectory() as tmp:
                fixture = ParameterFixture()
                node = fixture.ctx.node
                node.received.update(status=1., land=1.)
                node.data = {"status": NS(arming_state=2)} if unsafe == "armed" else {"land": NS(landed=False)}
                session = fixture.session(Path(tmp))
                def wait(predicate, timeout, description):
                    predicate()
                    self.fail("Unsafe state must be rejected before another pump")
                fixture.ctx.vehicle.wait = wait
                with self.assertRaisesRegex(RuntimeError, "armed or airborne"):
                    session.apply()
                self.assertEqual(fixture.writes, [])
                self.assertFalse(fixture.journal.exists())

    def test_new_reports_waits_again_when_land_ages_out_while_status_is_missing(self):
        with tempfile.TemporaryDirectory() as tmp, patch("diagnostic_parameters.time.monotonic", return_value=100.):
            fixture = ParameterFixture()
            node = fixture.ctx.node
            node.received.update(status=90., land=90.)
            session = fixture.session(Path(tmp))
            def wait(predicate, timeout, description):
                self.assertFalse(predicate())
                node.received["land"] = 91.
                self.assertFalse(predicate())  # Land is new, status remains stale.
                fixture.sim += 4.
                node.received["status"] = 100.
                self.assertFalse(predicate())  # Both advanced, but land is now too old.
                self.assertEqual(fixture.writes, [])
                node.received["land"] = 100.
                self.assertTrue(predicate())
            fixture.ctx.vehicle.wait = wait
            session.apply()
            self.assertTrue(session.document["applied_verified"])

    def test_strict_ground_check_still_raises_without_waiting_for_stale_states(self):
        for stale in ("status", "land", "missing"):
            with self.subTest(stale=stale), tempfile.TemporaryDirectory() as tmp, patch(
                    "diagnostic_parameters.time.monotonic", return_value=100.):
                fixture = ParameterFixture()
                session = fixture.session(Path(tmp))
                session.land_observed_sim_s = fixture.sim
                if stale == "status":
                    fixture.ctx.node.received["status"] = 90.
                elif stale == "land":
                    fixture.sim += 4.
                else:
                    fixture.ctx.node.data.pop("status")
                fixture.ctx.vehicle.wait = lambda *args: self.fail("Strict check must not wait")
                with self.assertRaisesRegex(RuntimeError, "Fresh disarmed|land receipts stalled"):
                    session.require_grounded()
                self.assertEqual(fixture.writes, [])

    def test_pid_candidate_catalog_has_nine_fixed_combinations_and_no_free_values(self):
        self.assertEqual(len(PID_CANDIDATES), 9)
        self.assertIsNone(validate_pid_candidate(None))
        for name, values in PID_CANDIDATES.items():
            self.assertEqual(validate_pid_candidate(name), values)
            self.assertEqual(pid_candidate_name(values), name)
            self.assertEqual(set(values), set(PID_PARAMETERS))
        for bad in ("best", {**PID_CANDIDATES["baseline"], "MPC_XY_P": 1.01},
                    {**PID_CANDIDATES["baseline"], "MPC_TILTMAX_AIR": 90},
                    {**PID_CANDIDATES["baseline"], "MPC_Z_P": True}):
            with self.assertRaises(ValueError):
                validate_pid_candidate(bad)
        with self.assertRaises(ValueError):
            validate_pid_candidate("baseline", "normal")

    def test_gps_pid_candidate_changes_only_four_and_restores_all_42_without_ev_writes(self):
        fixture = ParameterFixture(full_pid=True)
        with tempfile.TemporaryDirectory() as tmp:
            session = fixture.session(Path(tmp), None, "velocity_low")
            session.apply()
            self.assertFalse(session.document["ground_truth_aided"])
            self.assertFalse(session.document["simulation_ground_truth_aided"])
            self.assertFalse(session.document["aiding_requested"])
            self.assertEqual(session.document["required_fusion_flags_before_takeoff"], {})
            self.assertEqual(session.document["canonical_control_originals_count"], 42)
            self.assertEqual(set(fixture.writes), set(PID_PARAMETERS))
            for name in set(session.pid_records)-set(PID_PARAMETERS):
                self.assertEqual(fixture.values[name], fixture.original[name])
            for name, value in PID_CANDIDATES["velocity_low"].items():
                self.assertEqual(fixture.values[name], expected_record(value, 9))
            session.restore()
            self.assertEqual(fixture.values, fixture.original)
            self.assertEqual(set(fixture.writes), set(PID_PARAMETERS))
            self.assertFalse(fixture.journal.exists())

    def test_one_journal_can_restore_precision_and_pid_candidate_together(self):
        fixture = ParameterFixture(precision=True, full_pid=True)
        with tempfile.TemporaryDirectory() as tmp:
            session = fixture.session(Path(tmp), PRECISION_PROFILE, "position_high_velocity_low")
            session.apply()
            self.assertEqual(set(fixture.writes), set(PRECISION_PROFILE)|set(PID_PARAMETERS))
            journal = json.loads(fixture.journal.read_text())
            self.assertEqual(set(journal["allowlist"]), set(PRECISION_PROFILE)|set(PID_PARAMETERS))
            self.assertEqual(journal["pid_tuning_candidate_name"], "position_high_velocity_low")
            session.restore()
            self.assertEqual(fixture.values, fixture.original)
            self.assertFalse(fixture.journal.exists())

    def test_pid_candidate_requires_full_originals_and_protects_the_other_38(self):
        fixture = ParameterFixture()
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError, "all 42"):
                fixture.session(Path(tmp), None, "baseline")
        fixture = ParameterFixture(full_pid=True)
        with tempfile.TemporaryDirectory() as tmp:
            session = fixture.session(Path(tmp), None, "velocity_high")
            session.apply()
            fixture.values["MPC_XY_VEL_I_ACC"] = expected_record(5., 9)
            with self.assertRaisesRegex(RuntimeError, "not fully restored"):
                session.restore()
            self.assertNotIn("MPC_XY_VEL_I_ACC", fixture.writes)
            self.assertTrue(fixture.journal.exists())

    def test_only_reviewed_hover_profile_is_accepted(self):
        self.assertIsNone(validate_profile(None, "fast", "mpc"))
        for bad in ({**PROFILE, "MPC_XY_P": 9.}, {**PROFILE, "EKF2_EV_CTRL": 7}, {**PROFILE, "EKF2_EVP_NOISE": .02}, {**PROFILE, "EKF2_EV_CTRL": True}):
            with self.assertRaises(ValueError):
                validate_profile(bad)
        with self.assertRaises(ValueError):
            validate_profile(PROFILE, "joint_probe", "pid_ff")
        self.assertEqual(validate_profile(PRECISION_PROFILE), PRECISION_PROFILE)
        self.assertEqual(profile_name(PRECISION_PROFILE), "ev_precision_xyz_velocity_v1")
        self.assertEqual(profile_name(PROFILE), "ev_horizontal_gnss_v1")
        for bad in ({**PRECISION_PROFILE, "EKF2_EV_CTRL": 15}, {**PRECISION_PROFILE, "MPC_XY_P": 2.},
                    {k: v for k, v in PRECISION_PROFILE.items() if k != "EKF2_BARO_CTRL"}):
            with self.assertRaises(ValueError):
                validate_profile(bad)

    def test_precision_is_independent_ev_xyz_velocity_and_exactly_restores_all_sources(self):
        fixture = ParameterFixture(precision=True)
        with tempfile.TemporaryDirectory() as tmp:
            session = fixture.session(Path(tmp), PRECISION_PROFILE)
            session.apply()
            self.assertEqual(set(fixture.writes), set(PRECISION_PROFILE))
            self.assertEqual(fixture.writes[-1], "EKF2_EV_CTRL")
            for name, value in PRECISION_PROFILE.items():
                self.assertEqual(fixture.values[name], expected_record(value, TYPES[name]))
            self.assertNotIn("EKF2_GPS_CTRL", session.document["protected_parameters"])
            self.assertTrue(session.document["fixed_world_reference_required"])
            self.assertFalse(session.document["formal_gps_performance_claim"])
            self.assertEqual(session.document["required_fusion_flags_before_takeoff"], {
                "cs_ev_pos": True, "cs_ev_vel": True, "cs_ev_hgt": True, "cs_gnss_pos": False,
                "cs_gnss_vel": False, "cs_gps_hgt": False, "cs_baro_hgt": False})
            self.assertIn("local_z=ref_alt-world_z", session.document["origin_semantics"])
            session.restore()
            self.assertEqual(fixture.values, fixture.original)
            self.assertFalse(fixture.journal.exists())
            self.assertEqual(fixture.writes[-7:-4], ["EKF2_HGT_REF", "EKF2_GPS_CTRL", "EKF2_BARO_CTRL"])

    def test_precision_wrong_existing_sensor_delay_or_offset_refuses_before_any_write(self):
        for parameter in PRECISION_REQUIRED:
            fixture = ParameterFixture(precision=True)
            fixture.values[parameter] = expected_record(.01, 9)
            with tempfile.TemporaryDirectory() as tmp:
                session = fixture.session(Path(tmp), PRECISION_PROFILE)
                with self.assertRaisesRegex(RuntimeError, "zero sensor/delay offset"):
                    session.apply()
                self.assertEqual(fixture.writes, [])
                self.assertFalse(fixture.journal.exists())

    def test_precision_gnss_restore_failure_preserves_journal(self):
        fixture = ParameterFixture(precision=True)
        with tempfile.TemporaryDirectory() as tmp:
            session = fixture.session(Path(tmp), PRECISION_PROFILE)
            session.apply()
            fixture.ignore = "EKF2_GPS_CTRL"
            with self.assertRaisesRegex(RuntimeError, "not fully restored"):
                session.restore()
            self.assertTrue(fixture.journal.exists())
            self.assertEqual(fixture.values["EKF2_GPS_CTRL"]["value"], 0)
            self.assertFalse(session.document["restored_verified"])

    def test_apply_bit_encodes_int32_and_restores_exact_values_with_journal(self):
        fixture = ParameterFixture()
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            session = fixture.session(folder)
            session.apply()
            self.assertTrue(session.document["applied_verified"])
            self.assertEqual(fixture.values["EKF2_EV_CTRL"]["value"], 1)
            self.assertEqual(fixture.values["EKF2_EV_CTRL"]["raw_value_hex"], "01000000")
            self.assertTrue(fixture.journal.exists())
            with self.assertRaisesRegex(RuntimeError, "blocks new flights"):
                assert_no_pending_journal(fixture.journal)
            session.restore()
            self.assertTrue(session.document["restored_verified"])
            self.assertEqual(fixture.values, fixture.original)
            self.assertEqual(set(fixture.writes), set(PROFILE))
            self.assertFalse(fixture.journal.exists())
            self.assertTrue((folder/"diagnostic_restore_journal_completed.json").is_file())

    def test_restore_failure_leaves_persistent_flight_interlock(self):
        fixture = ParameterFixture()
        with tempfile.TemporaryDirectory() as tmp:
            session = fixture.session(Path(tmp))
            session.apply()
            fixture.ignore = "EKF2_EV_CTRL"
            with self.assertRaisesRegex(RuntimeError, "not fully restored"):
                session.restore()
            self.assertFalse(session.document["restored_verified"])
            self.assertTrue(fixture.journal.exists())
            with self.assertRaisesRegex(RuntimeError, "blocks new flights"):
                fixture.session(Path(tmp))

    def test_synchronous_storage_failure_keeps_interlock_despite_exact_runtime_restore(self):
        fixture = ParameterFixture()
        with tempfile.TemporaryDirectory() as tmp:
            session = fixture.session(Path(tmp))
            session.apply()
            fixture.save_failure = True
            with self.assertRaisesRegex(RuntimeError, "synchronous storage save failed"):
                session.restore()
            self.assertEqual(fixture.values, fixture.original)
            self.assertTrue(session.document["runtime_restored_verified"])
            self.assertFalse(session.document["restored_verified"])
            self.assertTrue(fixture.journal.exists())
            self.assertFalse((Path(tmp)/"diagnostic_restore_journal_completed.json").exists())

    def test_armed_restore_refuses_writes_and_preserves_journal(self):
        fixture = ParameterFixture()
        with tempfile.TemporaryDirectory() as tmp:
            session = fixture.session(Path(tmp))
            session.apply()
            fixture.ctx.node.data["status"].arming_state = 2
            count = len(fixture.writes)
            with self.assertRaisesRegex(RuntimeError, "armed or airborne"):
                session.restore()
            self.assertEqual(len(fixture.writes), count)
            self.assertTrue(fixture.journal.exists())

    def test_protected_pid_change_fails_application_and_cannot_be_silently_restored(self):
        fixture = ParameterFixture()
        with tempfile.TemporaryDirectory() as tmp:
            session = fixture.session(Path(tmp))
            session.apply()
            fixture.values["MPC_XY_P"] = expected_record(2., 9)
            with self.assertRaisesRegex(RuntimeError, "not fully restored"):
                session.restore()
            self.assertNotIn("MPC_XY_P", fixture.writes)
            self.assertIn("MPC_XY_P", session.document["restore_mismatches"])
            self.assertTrue(fixture.journal.exists())


if __name__ == "__main__":
    unittest.main()
