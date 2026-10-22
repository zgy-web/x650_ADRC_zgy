"""Audit propagation budgets/initialization, using an independent linear oracle."""
from pathlib import Path
import sys
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from simulations.x650_ts20_hold_v1.audit import free_forecast_in_blocks, packet_state_binding


class LinearPredictor:
    def __init__(self):
        self.calls = []

    def rollout(self, start, q, dq, packets, query):
        if query[-1] - start > 2.:
            raise ValueError("Two-second model budget")
        self.calls.append((start, q.copy(), dq.copy(), query.copy()))
        elapsed = query - start
        return {"time_s": query.copy(), "q": q + elapsed[:, None] * dq,
                "dq": np.tile(dq, (len(query), 1))}


class Ts20AuditPredictionTests(unittest.TestCase):
    def test_long_trial_carries_prediction_only_between_blocks(self):
        predictor = LinearPredictor()
        t = 42. + np.arange(7501) * .004
        q0, v0 = np.array([-.25, .35]), np.array([.01, -.007])
        q, v = free_forecast_in_blocks(predictor, t, q0, v0, [])
        np.testing.assert_allclose(q, q0 + (t-t[0])[:, None] * v0, atol=2e-14)
        np.testing.assert_allclose(v, np.tile(v0, (len(t), 1)), atol=0)
        self.assertGreater(len(predictor.calls), 10)
        for at, initial, _, query in predictor.calls:
            self.assertLessEqual(query[-1] - at, 1.90000001)
            np.testing.assert_allclose(initial, q0 + (at-t[0])*v0, atol=2e-14)

    def test_invalid_time_or_an_excessive_gap_cannot_be_silently_filled(self):
        for t in ([0.], [0., 0.], [0., np.nan], [0., 3.]):
            with self.subTest(t=t), self.assertRaises(ValueError):
                free_forecast_in_blocks(LinearPredictor(), t, [0., 0.], [0., 0.], [])

    def test_static_motion_cannot_hide_a_truncated_actual_packet_stream(self):
        packets = [{"sequence": i, "send_s": i*.005, "apply_s": i*.005+.003346379092576424,
                    "qref1": 0., "qref2": 0., "dqref1": 0., "dqref2": 0.} for i in range(4)]
        states = []
        for at, i in ((.014, 2), (.019, 3)):
            packet = packets[i]
            states.append({"sim_s": at, "last_send_sequence": i, "active_sequence": i,
                "last_send_s": packet["send_s"], "last_send_apply_s": packet["apply_s"],
                "active_send_s": packet["send_s"], "active_apply_s": packet["apply_s"],
                **{name+str(j): 0. for name in ("last_qref", "last_dqref", "active_qref", "active_dqref") for j in (1, 2)}})
        self.assertTrue(packet_state_binding(states, packets)["verified"])
        with self.assertRaisesRegex(ValueError, "absent"):
            packet_state_binding(states, packets[:-1])


if __name__ == "__main__":
    unittest.main()
