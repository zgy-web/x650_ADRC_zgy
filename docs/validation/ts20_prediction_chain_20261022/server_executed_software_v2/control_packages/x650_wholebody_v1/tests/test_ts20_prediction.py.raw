"""Independent numerical/clock contracts, not hardware or 24 V validation."""
from copy import deepcopy
import hashlib
import math
import os
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from x650_wholebody.ts20 import POSITION_STEP_RAD, VELOCITY_STEP_RAD_S
from x650_wholebody.ts20_prediction import Ts20PredictionModel


def metadata(friction=True):
    return {"schema": "ts20_public_motion_v1", "motor": "TS20-50",
            "status": "frozen_conditional_interface_model",
            "fixed_parameters": {"J_total_over_ka": .0007447642530597398,
                "b_over_ka": .004194907941904997,
                "Fc_over_ka": .03470285366117535 if friction else 0.,
                "offset_over_ka": .0004632811241788992 if friction else 0.,
                "combined_interface_delay_s": .003346379092576424},
            "friction_parameters": {"static_threshold": .075 if friction else 0.,
                "delta_static": .04 if friction else 0., "stribeck_velocity": .2}}


def packet(t, q=(0., 0.), v=(0., 0.), torque=(0., 0.)):
    return {"send_time_s": t, "q_ref": np.array(q), "dq_ref": np.array(v),
            "tau_ff": np.array(torque), "kp": np.full(2, 6.),
            "kd": np.full(2, 3276/32768)}


def state(q=(0., 0.), dq=(0., 0.), now=10., stamp=10.):
    return SimpleNamespace(q=np.array(q), dq=np.array(dq), control_time_s=now, joint_stamp_s=stamp)


def prediction(model=None, *, t=0., value=None, nominal=None, history=None,
               reference_state=None, packet_history=None, nodes=None):
    model = model or Ts20PredictionModel(metadata())
    nodes = np.array([.04, .08, .12]) if nodes is None else np.asarray(nodes)
    h = np.diff(np.r_[0., nodes])
    nominal = np.zeros((len(nodes), 5)) if nominal is None else nominal
    return model.predict(t, nodes, h, value or state(), history or [(-.01, np.zeros(2))],
                         [.45, .35], nominal,
                         reference_state=reference_state or {"time_s": -.01, "r": np.zeros(2)},
                         packet_history=packet_history or [packet(-.01), packet(-.005)], bus_origin_s=0.)


class Ts20NominalTests(unittest.TestCase):
    def test_static_zero_command_does_not_invent_motion(self):
        model = Ts20PredictionModel(metadata())
        result = model.rollout(0., [0., 0.], [0., 0.], [packet(-.005)], [0., .02, .1])
        np.testing.assert_array_equal(result["q"], 0.)
        np.testing.assert_array_equal(result["dq"], 0.)
        self.assertFalse(result["metadata"]["supply_24v_qualified"])

    def test_independent_damped_oscillator_closed_form(self):
        model = Ts20PredictionModel(metadata(False))
        times = np.array([0., .005, .02, .07, .2])
        result = model.rollout(0., [0., 0.], [0., 0.], [packet(-.005, q=(.1, -.2))], times)
        alpha = (model.b+3276/32768)/(2*model.J)
        omega = math.sqrt(6/model.J-alpha**2)
        decay = np.exp(-alpha*times)
        ratio = 1-decay*(np.cos(omega*times)+alpha/omega*np.sin(omega*times))
        rate = decay*(6/model.J)/omega*np.sin(omega*times)
        np.testing.assert_allclose(result["q"], ratio[:, None]*[.1, -.2], atol=2e-13, rtol=0)
        np.testing.assert_allclose(result["dq"], rate[:, None]*[.1, -.2], atol=2e-11, rtol=0)

    def test_combined_delay_applied_once_to_actual_send(self):
        model = Ts20PredictionModel(metadata(False))
        delay = model.delay_s
        result = model.rollout(0., [0., 0.], [0., 0.],
            [packet(-.005), packet(0., q=(.05, -.05))], [0., delay/2, delay, delay+.01])
        np.testing.assert_array_equal(result["q"][:3], 0.)
        self.assertGreater(result["q"][-1, 0], 0.)
        np.testing.assert_allclose(result["q"][-1, 0], -result["q"][-1, 1], atol=1e-15)

    def test_event_brake_and_reversal(self):
        model = Ts20PredictionModel(metadata())
        result = model.rollout(0., [0., 0.], [.15, -.15],
            [packet(-.005, q=(-.02, .02))], [0., .02, .1])
        self.assertGreater(sum(e["zero_events"] for e in result["events"]), 0)
        self.assertLess(result["q"][-1, 0], 0.)
        self.assertGreater(result["q"][-1, 1], 0.)

    def test_streaming_already_delayed_packet_agrees_with_rollout(self):
        model = Ts20PredictionModel(metadata())
        cmd = packet(-.005, q=(.025, -.03), v=(.1, -.1))
        value = model.advance([0., 0.], [0., 0.], {k: v for k, v in cmd.items() if k != "send_time_s"}, .1)
        run = model.rollout(0., [0., 0.], [0., 0.], [cmd], [.1])
        np.testing.assert_array_equal(value["q"], run["q"][0])
        np.testing.assert_array_equal(value["dq"], run["dq"][0])

    def test_actual_grid_coverage_and_gain_rejections(self):
        model = Ts20PredictionModel(metadata())
        cases = [[packet(0.)], [packet(-.005), packet(-.005)],
                 [packet(-.005), packet(.001)], [dict(packet(-.005), kp=[5., 6.])]]
        for packets in cases:
            with self.subTest(packets=packets), self.assertRaises(ValueError):
                model.rollout(0., [0., 0.], [0., 0.], packets, [0., .1])

    def test_envelope_catches_between_query_position_and_velocity_extrema(self):
        model = Ts20PredictionModel(metadata(False))
        result = model.rollout(0., [0., 0.], [0., 0.], [packet(-.005, q=(.1, -.1))], [0., .2])
        envelope = result["envelope"]
        alpha = (model.b+3276/32768)/(2*model.J)
        omega = math.sqrt(6/model.J-alpha**2)
        q_peak = .1*(1+math.exp(-alpha*math.pi/omega))
        t_v_peak = math.atan(omega/alpha)/omega
        v_peak = .1*math.exp(-alpha*t_v_peak)*(6/model.J)/omega*math.sin(omega*t_v_peak)
        self.assertTrue(envelope["collected"])
        self.assertAlmostEqual(envelope["q_upper"][0], q_peak, places=11)
        self.assertAlmostEqual(envelope["dq_upper"][0], v_peak, places=10)
        self.assertGreater(envelope["q_upper"][0], np.max(result["q"][:, 0]))
        self.assertGreater(envelope["dq_upper"][0], np.max(result["dq"][:, 0]))
        self.assertEqual({event["kind"] for event in envelope["events"]},
                         {"velocity_zero", "acceleration_zero"})

    def test_native_loader_rejects_unpinned_or_mismatched_library(self):
        model = Ts20PredictionModel(metadata())
        for digest in ("bad", "f"*64):
            with self.assertRaises(ValueError):
                model.enable_native(__file__, expected_sha256=digest)

    def test_effective_gain_metadata_is_explicit_and_default_remains_historical(self):
        historical = Ts20PredictionModel(metadata())
        damped = Ts20PredictionModel(metadata(), kp=6., kd=.25)
        self.assertEqual(historical.metadata()["effective_mit_kd"], [3276/32768]*2)
        self.assertEqual(damped.metadata()["effective_mit_kp"], [6.,6.])
        self.assertEqual(damped.metadata()["effective_mit_kd"], [.25,.25])
        self.assertEqual(historical.J, damped.J)
        self.assertEqual(historical.Fs, damped.Fs)
        self.assertEqual(historical.delay_s, damped.delay_s)
        p=packet(-.005, q=(.05,-.05));p["kd"][:]=.25
        result=damped.rollout(0.,[0.,0.],[0.,0.],[p],[0.,.1])
        self.assertGreater(result["q"][-1,0],0.)
        with self.assertRaisesRegex(ValueError, "gains differ"):
            damped.rollout(0.,[0.,0.],[0.,0.],[packet(-.005)],[0.,.1])


class Ts20PredictionContracts(unittest.TestCase):
    def test_nominal_interpolation_and_nonzero_breakaway_sensitivity(self):
        result = prediction()
        z = result["nominal_z"]
        for i, (_, q, dq, Q, D) in enumerate(result["events"]):
            np.testing.assert_allclose(q+Q@z, result["nominal_q"][i], atol=1e-15)
            np.testing.assert_allclose(dq+D@z, result["nominal_dq"][i], atol=1e-15)
            np.testing.assert_array_equal(Q[:, np.tile([True, True, True, False, False], 3)], 0.)
        np.testing.assert_array_equal(result["nominal_q"], 0.)
        self.assertGreater(np.linalg.norm(result["nodes"][-1][3]), 0.)
        self.assertTrue(result["metadata"]["nonlinear_candidate_check_required"])
        self.assertEqual(result["metadata"]["trust_radius_normalized"], .15)

    def test_reference_integrates_sent_history_not_measured_position(self):
        model = Ts20PredictionModel(metadata())
        anchor = {"time_s": -.01, "r": np.array([.02, -.01])}
        history = [(-.01, np.array([.2, -.1])), (.007, np.array([-.1, .3]))]
        packets = [packet(-.01, q=(.02, -.01)), packet(-.005, q=(.021, -.0105)),
                   packet(0., q=(.022, -.011)), packet(.005, q=(.023, -.0115)),
                   packet(.01, q=(.0231, -.0108)), packet(.015, q=(.0226, -.0093))]
        result = prediction(model, t=.02, value=state(q=(.019, -.02)), history=history,
                            reference_state=anchor, packet_history=packets)
        # Piecewise hand integral .017*[.2,-.1]+.013*[-.1,.3].
        np.testing.assert_allclose(result["r_current"], [.0221, -.0078], atol=1e-16)
        np.testing.assert_array_equal(anchor["r"], [.02, -.01])
        self.assertFalse(np.array_equal(result["r_current"], [.019, -.02]))

    def test_off_phase_now_does_not_shift_bus_grid(self):
        result = prediction(t=.002)
        future = [p["send_time_s"] for p in result["nominal_packets"] if p["send_time_s"] >= .002]
        self.assertAlmostEqual(future[0], .005)
        np.testing.assert_allclose(np.diff(future), .005, atol=1e-15)

    def test_already_sent_current_packet_cannot_be_rewritten(self):
        nominal = np.zeros((3, 5)); nominal[:, 3] = .1
        result = prediction(nominal=nominal, packet_history=[packet(-.005), packet(0.)])
        self.assertEqual(result["nominal_packets"][1]["send_time_s"], 0.)
        self.assertEqual(result["nominal_packets"][2]["send_time_s"], .005)
        np.testing.assert_array_equal(result["nominal_packets"][1]["dq_ref"], 0.)

    def test_source_age_uses_pending_packets_without_future_feedback(self):
        model = Ts20PredictionModel(metadata(False))
        packets = [packet(-.005), packet(0., q=(.02, -.03))]
        result = prediction(model, t=.01, value=state(now=10.01, stamp=10.),
                            packet_history=packets)
        independent = model.rollout(0., [0., 0.], [0., 0.], packets, [.01])
        np.testing.assert_allclose(result["q_current"], independent["q"][0], atol=1e-14)
        np.testing.assert_allclose(result["dq_current"], independent["dq"][0], atol=1e-12)
        self.assertAlmostEqual(result["measurement_age_s"], .01)

    def test_reference_nodes_exact_piecewise_integral(self):
        nominal = np.zeros((3, 5)); nominal[:, 3:] = [[.1, -.2], [-.2, .1], [.3, 0.]]
        result = prediction(nominal=nominal)
        for i, (_, r, R) in enumerate(result["reference_nodes"]):
            expected = np.sum(.04*nominal[:i+1, 3:], axis=0)
            np.testing.assert_allclose(r+R@result["nominal_z"], expected, atol=1e-16)

    def test_candidate_check_is_free_prediction_with_frozen_measurement(self):
        model = Ts20PredictionModel(metadata())
        nominal = np.zeros((3, 5))
        prepared = prediction(model, nominal=nominal)
        candidate = nominal.copy(); candidate[:, 3:] = [.1, -.15]
        checked = model.candidate_rollout(prepared, candidate)
        # Independent full re-linearization at the candidate must reproduce
        # the same free nominal q/dq; the cheap check skips only Q/D assembly.
        fresh = prediction(model, nominal=candidate)
        np.testing.assert_array_equal(checked["q"], fresh["nominal_q"])
        np.testing.assert_array_equal(checked["dq"], fresh["nominal_dq"])
        np.testing.assert_array_equal(prepared["_context"]["initial_q"], [0., 0.])
        np.testing.assert_allclose(checked["reference_nodes"][-1], [.012, -.018], atol=1e-16)
        self.assertTrue(checked["envelope"]["collected"])

    def test_moving_nominal_does_not_overwrite_measurement_snapshot(self):
        nominal = np.zeros((3, 5)); nominal[:, 3:] = [.1, -.15]
        model = Ts20PredictionModel(metadata())
        prepared = prediction(model, nominal=nominal)
        checked = model.candidate_rollout(prepared, nominal)
        np.testing.assert_array_equal(prepared["_context"]["initial_q"], [0., 0.])
        np.testing.assert_array_equal(checked["q"], prepared["nominal_q"])

    def test_sliding_surrogate_matches_unquantized_linear_oracle(self):
        model = Ts20PredictionModel(metadata(False))
        nominal = np.zeros((3, 5)); nominal[:, 3:] = [[.2, -.1], [.1, -.2], [.15, -.15]]
        result = prediction(model, nominal=nominal)
        # Choose a perturbation large enough to exceed one wire velocity LSB.
        delta = np.zeros(15); delta[3] = .02
        changed = nominal.copy(); changed[0, 3] += delta[3]*.45
        check = prediction(model, nominal=changed)
        for base, perturbed in zip(result["nodes"], check["nodes"]):
            predicted_q = base[1]+base[3]@(result["nominal_z"]+delta)
            actual_q = perturbed[1]+perturbed[3]@check["nominal_z"]
            # Quantizer was not differentiated; its known half-LSB effect is
            # bounded here independently rather than pretending exact smoothness.
            np.testing.assert_allclose(predicted_q, actual_q, atol=4e-5, rtol=0)

    def test_stale_future_history_and_missing_reference_rejections(self):
        for changes in ({"value": state(now=10., stamp=10.001)},
                        {"value": state(now=10., stamp=9.7)},
                        {"history": [(0.001, np.zeros(2))]},
                        {"reference_state": {"time_s": .001, "r": np.zeros(2)}},
                        {"packet_history": [packet(-.005), packet(.005)]}):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                prediction(**changes)

    def test_source_model_and_numerical_budget_rejections(self):
        bad = metadata(); bad["motor"] = "TS20-100"
        for model, options in ((bad, {}), (metadata(), {"max_step_s": .002}),
                                (metadata(), {"trust_radius_normalized": 1.}),
                                (metadata(), {"kp": True})):
            with self.assertRaises(ValueError):
                Ts20PredictionModel(model, **options)


@unittest.skipUnless(os.environ.get("TS20_TEST_NATIVE_LIB"), "explicit native library not supplied")
class OptionalNativeNumericsTests(unittest.TestCase):
    def test_native_reference_and_full_extrema_agree(self):
        reference, native = Ts20PredictionModel(metadata()), Ts20PredictionModel(metadata())
        path = Path(os.environ["TS20_TEST_NATIVE_LIB"])
        native.enable_native(path, expected_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        packets = [packet(-.005)]
        for i, t in enumerate(np.arange(0., .605, .005)):
            packets.append(packet(float(t), q=(.03*math.sin(17*t), -.025*math.sin(13*t)),
                                  v=(.4*math.cos(17*t), -.3*math.cos(13*t))))
        times = np.linspace(0., .6, 101)
        expected = reference.rollout(0., [0., 0.], [0., 0.], packets, times)
        actual = native.rollout(0., [0., 0.], [0., 0.], packets, times)
        np.testing.assert_allclose(actual["q"], expected["q"], atol=2e-12, rtol=0)
        np.testing.assert_allclose(actual["dq"], expected["dq"], atol=2e-10, rtol=0)
        for key in ("q_lower", "q_upper", "dq_lower", "dq_upper"):
            np.testing.assert_allclose(actual["envelope"][key], expected["envelope"][key], atol=2e-10, rtol=0)


if __name__ == "__main__":
    unittest.main()

