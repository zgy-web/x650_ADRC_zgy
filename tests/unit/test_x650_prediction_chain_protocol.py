"""Admission tests; synthetic inputs are never flight evidence."""
from copy import deepcopy
import unittest
import test_x650_mixed_mpc_protocol as existing
import mixed_mpc_protocol as protocol


class PredictionChainProtocolTests(existing.MixedMpcProtocolTests):
    def test_old_protocol_bytes_remain_frozen(self):
        self.assertEqual(protocol.protocol_hash(protocol.build_mixed_mpc_protocol()),
                         "52f4758648836eb6654554d85201fb447132869b53392bb23ed90a08c89920d9")

    def test_new_candidate_preserves_gains_limits_acceptance_and_physical_condition(self):
        old, new = protocol.build_mixed_mpc_protocol(), protocol.build_prediction_chain_protocol()
        self.assertNotEqual(protocol.protocol_hash(old), protocol.protocol_hash(new))
        for key in ("common_controller_config", "acceptance", "research_profile", "feedback_contract"):
            self.assertEqual(new[key], old[key])
        config = deepcopy(new["mpc_controller_config"])
        config.pop("prediction_chain")
        self.assertEqual(config, old["mpc_controller_config"])
        cfg = self.cfg()
        before = existing.research.condition_for_task(cfg)
        cfg["mixed_mpc_protocol"] = new
        self.assertEqual(existing.task.validate_task_config(cfg), cfg)
        self.assertEqual(existing.research.condition_for_task(cfg), before)
        for key, number in (("joint_delay_s", .020), ("joint_delay_guard_s", .030),
                            ("gravity_m_s2", 10.), ("attitude_residual_decay_s", .2)):
            changed = deepcopy(new)
            changed["mpc_controller_config"]["prediction_chain"][key] = number
            with self.subTest(key=key), self.assertRaises(ValueError):
                protocol.validate_mixed_mpc_protocol(changed)

    def test_old_baseline_cannot_unlock_new_mpc(self):
        cfg = self.cfg(controller="mpc")
        provenance = self.gate_bundle(cfg)
        cfg["mixed_mpc_protocol"] = protocol.build_prediction_chain_protocol()
        with self.assertRaisesRegex(ValueError, "protocol|Mixed"):
            existing.task.validate_stage_gates(cfg, provenance)


if __name__ == "__main__":
    unittest.main()
