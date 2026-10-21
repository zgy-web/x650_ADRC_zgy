"""New research sessions bind provenance before any runtime parameter write."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import tempfile
import unittest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(Path(__file__).resolve().parent))
from diagnostic_parameters import (DiagnosticParameterSession, PID_CANDIDATES,
    PRECISION_PROFILE, PROFILE, expected_record, validate_profile, validate_pid_candidate)
from diagnostic_odometry import DiagnosticOdometry
from research_condition import RESEARCH_PROFILE_NAME, canonical_hash
from hover_reference import PINNED_PX4
from test_diagnostic_parameters import ParameterFixture


def operating_condition(fixture):
    values = {name: float(record["value"]) for name, record in fixture.values.items()
              if name.startswith(("MC_", "MPC_"))}
    values.update({name: float(expected_record(value, 9)["value"])
                   for name, value in PID_CANDIDATES["velocity_high"].items()})
    return {"kind": "x650_research_operating_condition", "schema_version": 1,
        "research_profile": RESEARCH_PROFILE_NAME, "scope": "simulation_truth_aided_ev_commissioning",
        "model_sha256": fixture.ctx.model["model_sha256"], "estimator_parameters": dict(PRECISION_PROFILE),
        "observer": {"rate_hz": 50., "gaussian_noise": 0., "measurement_point": "base_link"},
        "coordinate_contract": "raw_ev_world", "px4_source_contract_commit": PINNED_PX4,
        "control_interface": "fixed_world_position_v1", "pid_parameters_sha256": canonical_hash(values),
        "world_sdf_sha256": "b"*64, "ground_truth_aided": True,
        "gps_performance_claim": False, "hardware_performance_claim": False}


def session_for(fixture, folder, *, profile=PRECISION_PROFILE, pid_candidate="velocity_high", **kwargs):
    fixture.journal = folder/"runtime/restore_pending.json"
    return DiagnosticParameterSession(fixture.ctx, profile, folder, reader=fixture.read,
        protected_pid_records={name: record for name, record in fixture.values.items()
                               if name.startswith(("MC_", "MPC_"))},
        journal_path=fixture.journal, pid_candidate=pid_candidate,
        storage_saver=lambda session: {"returncode": 0, "synchronous_default_save": True, "test_fixture": True},
        **kwargs)


def observer_for(folder, **kwargs):
    listener = folder/"px4/build/px4_sitl_default/bin/px4-listener"
    listener.parent.mkdir(parents=True)
    listener.write_text("mock executable", encoding="utf-8")
    (folder/".amsim").mkdir()
    metadata = folder/"runtime_meta.json"
    metadata.write_text(json.dumps({"instance": "unit_x650_0", "world": "unit_world"}), encoding="utf-8")
    (folder/".amsim/runtime.json").write_text(json.dumps({"runtime_meta": str(metadata)}), encoding="utf-8")
    return DiagnosticOdometry(folder, "unit_x650_0", folder, px4_directory=folder/"px4", **kwargs)


class ResearchRuntimePurposeTests(unittest.TestCase):
    def test_research_apply_restore_has_same_transaction_and_bound_condition(self):
        fixture = ParameterFixture(precision=True, full_pid=True)
        condition = operating_condition(fixture)
        digest = canonical_hash(condition)
        with tempfile.TemporaryDirectory() as tmp:
            folder = Path(tmp)
            session = session_for(fixture, folder, purpose="research", operating_condition=condition)
            document = json.loads((folder/"diagnostic_parameter_session.json").read_text())
            self.assertEqual(document["kind"], "x650_ground_truth_aided_research_parameter_session")
            self.assertEqual(document["operating_condition_sha256"], digest)
            self.assertEqual(document["operating_condition"], condition)
            self.assertTrue(document["ground_truth_aided"])
            self.assertTrue(document["ideal_data"])
            self.assertFalse(document["diagnostic_only"])
            self.assertFalse(document["aiding_confirmed"])
            self.assertFalse(document["eligible_as_stage_gate"])
            self.assertFalse(document["formal_gps_performance_claim"])
            self.assertFalse(document["hardware_performance_claim"])
            condition["observer"]["rate_hz"] = 999.  # Caller mutation cannot rewrite provenance.
            self.assertEqual(session.operating_condition["observer"]["rate_hz"], 50.)
            session.apply()
            journal = json.loads(fixture.journal.read_text())
            self.assertEqual(journal["operating_condition_sha256"], digest)
            self.assertFalse(journal["diagnostic_only"])
            self.assertEqual(journal["purpose"], "research")
            self.assertEqual(set(fixture.writes), set(PRECISION_PROFILE)|set(PID_CANDIDATES["velocity_high"]))
            session.restore()
            self.assertEqual(fixture.values, fixture.original)
            self.assertFalse(fixture.journal.exists())
            self.assertTrue(session.document["restored_verified"])
            self.assertFalse(session.document["eligible_as_stage_gate"])

    def test_research_requires_precision_profile_selected_pid_and_exact_condition(self):
        fixture = ParameterFixture(precision=True, full_pid=True)
        valid = operating_condition(fixture)
        requests = [
            {"operating_condition": None},
            {"profile": PROFILE, "operating_condition": valid},
            {"pid_candidate": "baseline", "operating_condition": valid},
            {"operating_condition": {**valid, "pid_parameters_sha256": "c"*64}},
            {"operating_condition": {**valid, "model_sha256": "c"*64}},
            {"operating_condition": {**valid, "observer": {**valid["observer"], "rate_hz": 20.}}},
            {"operating_condition": {**valid, "hardware_performance_claim": True}},
        ]
        for request in requests:
            with self.subTest(request=request), tempfile.TemporaryDirectory() as tmp:
                folder = Path(tmp)
                with self.assertRaises(ValueError):
                    session_for(fixture, folder, purpose="research", **request)
                self.assertEqual(fixture.writes, [])
                self.assertFalse((folder/"diagnostic_parameter_session.json").exists())
                self.assertFalse(fixture.journal.exists())

    def test_digest_binds_float32_wire_values_not_unquantized_catalog(self):
        fixture = ParameterFixture(precision=True, full_pid=True)
        condition = operating_condition(fixture)
        values = {name: float(record["value"]) for name, record in fixture.values.items()
                  if name.startswith(("MC_", "MPC_"))}
        values.update(PID_CANDIDATES["velocity_high"])
        condition["pid_parameters_sha256"] = canonical_hash(values)
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaisesRegex(ValueError, "effective 42"):
                session_for(fixture, Path(tmp), purpose="research", operating_condition=condition)

    def test_default_diagnostic_cannot_be_relabelled_by_passing_condition(self):
        fixture = ParameterFixture(precision=True, full_pid=True)
        with tempfile.TemporaryDirectory() as tmp:
            session = session_for(fixture, Path(tmp))
            self.assertTrue(session.document["diagnostic_only"])
            self.assertFalse(session.document["ground_truth_aided"])
            self.assertEqual(session.document["purpose"], "diagnostic")
            self.assertIsNone(session.document["operating_condition"])
            self.assertIsNone(session.document["operating_condition_sha256"])
        for request in ({"operating_condition": operating_condition(fixture)}, {"purpose": "promote"}):
            with self.subTest(request=request), tempfile.TemporaryDirectory() as tmp:
                with self.assertRaises(ValueError):
                    session_for(fixture, Path(tmp), **request)
        with self.assertRaises(ValueError):
            validate_profile(PRECISION_PROFILE, stage="joint_probe")
        with self.assertRaises(ValueError):
            validate_pid_candidate("velocity_high", stage="joint_probe")

    def test_research_does_not_bypass_armed_write_refusal(self):
        fixture = ParameterFixture(precision=True, full_pid=True)
        with tempfile.TemporaryDirectory() as tmp:
            session = session_for(fixture, Path(tmp), purpose="research", operating_condition=operating_condition(fixture))
            fixture.ctx.node.data["status"].arming_state = 2
            with self.assertRaisesRegex(RuntimeError, "armed or airborne"):
                session.apply()
            self.assertEqual(fixture.writes, [])
            self.assertFalse(fixture.journal.exists())

    def test_research_observer_is_labelled_at_creation_without_certifying_gate(self):
        condition = operating_condition(ParameterFixture(precision=True, full_pid=True))
        expected = deepcopy(condition)
        with tempfile.TemporaryDirectory() as tmp:
            observer = observer_for(Path(tmp), purpose="research", operating_condition=condition)
            condition["observer"]["rate_hz"] = 123.
            document = observer.document
            self.assertEqual(document["kind"], "x650_ideal_external_position_research_observer")
            self.assertEqual(document["operating_condition"], expected)
            self.assertEqual(document["operating_condition_sha256"], canonical_hash(expected))
            self.assertEqual(document["requested_rate_hz"], expected["observer"]["rate_hz"])
            self.assertEqual(document["gaussian_noise"], expected["observer"]["gaussian_noise"])
            self.assertTrue(document["ideal_data"])
            self.assertTrue(document["ground_truth_aided"])
            for name in ("diagnostic_only", "eligible_as_stage_gate", "formal_performance_claim",
                         "formal_gps_performance_claim", "hardware_performance_claim", "aiding_confirmed", "observer_loaded"):
                self.assertFalse(document[name], name)
            self.assertEqual(document, json.loads((Path(tmp)/"diagnostic_odometry.json").read_text()))

    def test_observer_rejects_unreviewed_purpose_or_condition_before_runtime_access(self):
        condition = operating_condition(ParameterFixture(precision=True, full_pid=True))
        for kwargs in ({"purpose": "research"}, {"purpose": "research", "operating_condition": {}},
                       {"purpose": "research", "operating_condition": {**condition, "ground_truth_aided": 1}},
                       {"operating_condition": condition}, {"purpose": "promote"}):
            with self.subTest(kwargs=kwargs), tempfile.TemporaryDirectory() as tmp:
                with self.assertRaises(ValueError):
                    DiagnosticOdometry(tmp, "unit_x650_0", tmp, px4_directory=tmp, **kwargs)
                self.assertFalse((Path(tmp)/"diagnostic_odometry.json").exists())


if __name__ == "__main__":
    unittest.main()
