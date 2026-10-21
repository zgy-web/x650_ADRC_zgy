"""Synthetic frame/freshness contracts; no ROS server or flight certificate."""
from pathlib import Path
import sys
import types
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for path in (ROOT, ROOT/"control_packages/x650_wholebody_v1"):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
from simulations.x650_wholebody_mpc_v1.estimated_feedback import (
    Px4EstimatedFeedback, encode_world_command, PINNED_PX4, TOPICS, EV_FLAGS)
from x650_wholebody.api import ControlCommand
from x650_wholebody.adapter import ControllerAdapter
from x650_wholebody.model import rotation_matrix

NS = types.SimpleNamespace


def fixture():
    position = NS(x=3., y=2., z=486.5, vx=.2, vy=.3, vz=-.4,
        timestamp=500000, timestamp_sample=499000,
        ref_lat=47., ref_lon=8., ref_alt=488., ref_timestamp=100,
        xy_valid=True, z_valid=True, v_xy_valid=True, v_z_valid=True,
        xy_global=False, z_global=True, heading_good_for_control=True, dead_reckoning=False,
        xy_reset_counter=3, z_reset_counter=2, heading_reset_counter=1,
        vxy_reset_counter=4, vz_reset_counter=2)
    reference = {"frame_mode": "raw_ev_world", "target_frozen": True,
        "fit_estimator_translation": False, "px4_source_contract_commit": PINNED_PX4,
        "px4_reference_at_bind": {name: getattr(position, name) for name in (
            "ref_lat", "ref_lon", "ref_alt", "ref_timestamp", "xy_global", "z_global",
            "xy_reset_counter", "z_reset_counter", "heading_reset_counter")}}
    node = NS(estimator_feedback_enabled=True, estimator_feedback_error=None,
        estimator_clock_s=10., received_sim={}, received={}, callback_sequences={}, data={})
    node.data = {"position": position,
        "attitude": NS(q=[np.sqrt(.5), 0., 0., np.sqrt(.5)], quat_reset_counter=2,
                       timestamp=900000, timestamp_sample=899000),
        "angular_velocity": NS(xyz=[.1,.2,.3], timestamp=1200000, timestamp_sample=1199000),
        "estimator_flags": NS(**EV_FLAGS, cs_inertial_dead_reckoning=False,
                              cs_wind_dead_reckoning=False, timestamp=3000000, timestamp_sample=2999000),
        "joints": NS(name=["arm_joint_2", "arm_joint_1"], position=[.4,.3], velocity=[.2,.1],
                     header=NS(stamp=NS(sec=10,nanosec=0))),
        "clock": NS(clock=NS(sec=10,nanosec=0))}
    for key in (*TOPICS, "clock"):
        node.received_sim[key] = 10.
        node.received[key] = 100.
        node.callback_sequences[key] = 1
    return node, reference


def advance(node, sim_s, *, angular=True):
    node.estimator_clock_s = sim_s
    for key in (*TOPICS, "clock"):
        if key == "angular_velocity" and not angular:
            continue
        node.received_sim[key] = sim_s
        node.received[key] = 100.+sim_s-10.
        node.callback_sequences[key] += 1
    seconds = int(sim_s)
    node.data["joints"].header.stamp = NS(sec=seconds, nanosec=round((sim_s-seconds)*1e9))


class EstimatedFeedbackTests(unittest.TestCase):
    def test_frame_and_nonzero_altitude_origin_without_truth(self):
        node, reference = fixture()
        adapter = Px4EstimatedFeedback(reference, node)
        state, stamp, diagnostic = adapter.read(node, 10., wall_now=100.)
        np.testing.assert_allclose(state.position, [2.,3.,1.5])
        np.testing.assert_allclose(state.velocity, [.3,.2,.4])
        np.testing.assert_allclose(state.rotation, np.eye(3), atol=1e-15)
        np.testing.assert_allclose(state.omega_body, [.1,-.2,-.3])
        np.testing.assert_allclose(state.q, [.3,.4])
        np.testing.assert_allclose(state.dq, [.1,.2])
        self.assertEqual(stamp, 10.)
        self.assertEqual(state.stamp_s, 10.)
        self.assertEqual(diagnostic["px4_snapshot"]["position_ned_m"], [3.,2.,486.5])
        self.assertFalse(diagnostic["truth_feedback"])
        node.data["truth"] = object()  # Not even a usable odometry object.
        node.sim_time_s = lambda: self.fail("Truth time must never be used")
        state2, _, _ = adapter.read(node, 10., wall_now=100.)
        np.testing.assert_array_equal(state2.position, state.position)

    def test_full_attitude_basis_and_flu_rates(self):
        node, reference = fixture()
        q = np.array([.4, .2, -.3, .5]); q /= np.linalg.norm(q)
        node.data["attitude"].q = q
        state, _, _ = Px4EstimatedFeedback(reference).read(node, 10., wall_now=100.)
        C = np.array([[0,1,0],[1,0,0],[0,0,-1]])
        D = np.diag([1,-1,-1])
        np.testing.assert_allclose(state.rotation, C@rotation_matrix(q)@D)
        np.testing.assert_allclose(state.rotation @ state.omega_body,
                                   C@rotation_matrix(q)@np.array([.1,.2,.3]))

    def test_encoder_has_no_live_estimator_error_bridge(self):
        node, reference = fixture()
        state, _, _ = Px4EstimatedFeedback(reference).read(node, 10., wall_now=100.)
        gains = np.array([1.2,1.2,1.5])
        executed = np.array([.2,-.3,.1])
        command = ControlCommand.from_executed_velocity(state, executed, [.1,.2],
            nominal_position=[2.1,3.2,1.6], acceleration_feedforward=[.3,.4,.5], position_gains=gains)
        encoded = encode_world_command(command, reference)
        np.testing.assert_allclose(encoded.position, [3.2,2.1,486.4])
        np.testing.assert_allclose(encoded.acceleration, [.4,.3,-.5])
        ned_gains = np.array([1.2,1.2,1.5])
        reconstructed = ned_gains*(encoded.position-np.array([3.,2.,486.5]))+encoded.velocity
        np.testing.assert_allclose(reconstructed, [executed[1],executed[0],-executed[2]], atol=1e-13)
        node.data["position"].x = 1000.
        np.testing.assert_array_equal(encode_world_command(command, reference).position, encoded.position)

    def test_reference_validation_and_copy(self):
        for key, value in (("frame_mode", "global_wgs84"), ("px4_source_contract_commit", "wrong"),
                           ("target_frozen", False), ("fit_estimator_translation", True)):
            node, reference = fixture(); reference[key] = value
            with self.subTest(key=key), self.assertRaises(ValueError):
                Px4EstimatedFeedback(reference)
        node, reference = fixture()
        adapter = Px4EstimatedFeedback(reference)
        reference["px4_reference_at_bind"]["ref_alt"] = 999.
        self.assertEqual(adapter.read(node,10.,wall_now=100.)[0].position[2],1.5)

    def test_each_origin_global_validity_and_reset_change_is_rejected(self):
        for key in ("ref_lat", "ref_lon", "ref_alt", "xy_global", "z_global", "xy_reset_counter",
                    "z_reset_counter", "heading_reset_counter", "vxy_reset_counter", "vz_reset_counter", "quat_reset_counter"):
            node, reference = fixture(); adapter = Px4EstimatedFeedback(reference, node)
            message = node.data["attitude" if key == "quat_reset_counter" else "position"]
            old = getattr(message, key); setattr(message, key, not old if isinstance(old,bool) else old+1)
            with self.subTest(key=key), self.assertRaises(RuntimeError):
                adapter.read(node,10.,wall_now=100.)

    def test_validity_heading_deadreckoning_and_ev_flags_are_required(self):
        cases = [("position", key, False) for key in ("xy_valid","z_valid","v_xy_valid","v_z_valid","heading_good_for_control")]
        cases += [("position","dead_reckoning",True)]
        cases += [("estimator_flags", key, not value) for key,value in EV_FLAGS.items()]
        cases += [("estimator_flags", key, True) for key in ("cs_inertial_dead_reckoning","cs_wind_dead_reckoning")]
        for topic,key,value in cases:
            node, reference=fixture(); setattr(node.data[topic],key,value)
            with self.subTest(key=key),self.assertRaises(RuntimeError):
                Px4EstimatedFeedback(reference).read(node,10.,wall_now=100.)

    def test_missing_stream_or_receipt_metadata_never_falls_back(self):
        for key in TOPICS:
            node,reference=fixture(); del node.data[key]
            with self.subTest(key=key),self.assertRaisesRegex(RuntimeError,"Missing"):
                Px4EstimatedFeedback(reference).read(node,10.,wall_now=100.)
        for map_name in ("received","received_sim","callback_sequences"):
            node,reference=fixture(); del getattr(node,map_name)["angular_velocity"]
            with self.subTest(map=map_name), self.assertRaises(ValueError):
                Px4EstimatedFeedback(reference).read(node,10.,wall_now=100.)

    def test_each_stream_receipt_age_limit_and_wall_stall(self):
        for key, age in (("position",.101),("attitude",.101),("angular_velocity",.101),
                         ("estimator_flags",1.251),("joints",.251)):
            node,reference=fixture(); node.received_sim[key]-=age
            with self.subTest(key=key),self.assertRaisesRegex(RuntimeError,"stale"):
                Px4EstimatedFeedback(reference).read(node,10.,wall_now=100.)
        node,reference=fixture()
        with self.assertRaisesRegex(RuntimeError,"wall"):
            Px4EstimatedFeedback(reference).read(node,10.,wall_now=104.001)

    def test_flags_source_period_and_slow_sitl_have_separate_bounded_ages(self):
        node,reference=fixture()
        # Fixed-source flags can legitimately be held for more than one second
        # and >4 wall seconds, unlike the fast pose/rate inputs.
        node.received_sim["estimator_flags"]=8.75
        node.received["estimator_flags"]=90.
        adapter=Px4EstimatedFeedback(reference)
        state,_,diagnostic=adapter.read(node,10.,wall_now=100.)
        self.assertEqual(state.stamp_s,10.)
        self.assertEqual(diagnostic["callbacks"]["estimator_flags"]["sim_age_s"],1.25)
        self.assertEqual(adapter.document["max_age_s"]["flags"],1.25)
        self.assertEqual(adapter.document["max_age_s"]["flags_wall"],10.)
        self.assertEqual(adapter.document["max_age_s"]["wall"],4.)
        for field,value in (("received_sim",8.749),("received",89.999)):
            stale,ref=fixture();getattr(stale,field)["estimator_flags"]=value
            with self.subTest(field=field),self.assertRaisesRegex(RuntimeError,"stale"):
                Px4EstimatedFeedback(ref).read(stale,10.,wall_now=100.)
        for key in ("position","attitude","angular_velocity","joints","clock"):
            stale,ref=fixture();stale.received[key]=95.999
            with self.subTest(topic=key),self.assertRaisesRegex(RuntimeError,"wall"):
                Px4EstimatedFeedback(ref).read(stale,10.,wall_now=100.)

    def test_flags_wall_override_is_validated_and_does_not_disable_fusion_guard(self):
        for limit in (0.,-1.,float("nan")):
            node,reference=fixture()
            with self.subTest(limit=limit),self.assertRaises(ValueError):
                Px4EstimatedFeedback(reference,max_flags_wall_age_s=limit)
        node,reference=fixture();node.received["estimator_flags"]=95.
        with self.assertRaisesRegex(RuntimeError,"wall"):
            Px4EstimatedFeedback(reference,max_flags_wall_age_s=4.).read(node,10.,wall_now=100.)
        node,reference=fixture();node.received["estimator_flags"]=95.
        node.received_sim["estimator_flags"]=9.
        node.data["estimator_flags"].cs_ev_pos=False
        with self.assertRaisesRegex(RuntimeError,"fusion flags"):
            Px4EstimatedFeedback(reference).read(node,10.,wall_now=100.)

    def test_clock_mismatch_future_receipt_regression_and_poison(self):
        node,reference=fixture(); adapter=Px4EstimatedFeedback(reference)
        adapter.read(node,10.,wall_now=100.)
        with self.assertRaisesRegex(RuntimeError,"current estimator"):
            adapter.read(node,10.01,wall_now=100.)
        node.estimator_clock_s=9.99
        with self.assertRaisesRegex(RuntimeError,"regressed"):
            adapter.read(node,9.99,wall_now=100.)
        node.estimator_clock_s=10.;node.estimator_feedback_error="ROS /clock regressed"
        with self.assertRaisesRegex(RuntimeError,"clock error"):
            adapter.read(node,10.,wall_now=100.)

    def test_callback_sequence_and_receipt_regressions(self):
        for mutation in ("sequence", "sim", "wall", "held_receipt"):
            node,reference=fixture();adapter=Px4EstimatedFeedback(reference)
            node.callback_sequences["angular_velocity"]=3
            adapter.read(node,10.,wall_now=100.)
            if mutation=="sequence":node.callback_sequences["angular_velocity"]=2
            if mutation=="sim":node.received_sim["angular_velocity"]=9.99
            if mutation=="wall":node.received["angular_velocity"]=99.99
            if mutation=="held_receipt":node.received["angular_velocity"]=100.01
            with self.subTest(mutation=mutation),self.assertRaises(RuntimeError):
                adapter.read(node,10.,wall_now=100.02)

    def test_old_omega_does_not_get_a_new_stamp_from_fresh_position(self):
        node,reference=fixture();feedback=Px4EstimatedFeedback(reference)
        state,_,_=feedback.read(node,10.,wall_now=100.)
        motion=ControllerAdapter.__new__(ControllerAdapter)
        motion._previous_stamp=None;motion._previous_omega=None;motion._alpha_body=np.zeros(3)
        motion.angular_acceleration_filter_tau_s=.12
        np.testing.assert_array_equal(motion.observe(state),np.zeros(3))
        advance(node,10.02,angular=False);node.data["position"].vx=.8
        state,_,diagnostic=feedback.read(node,10.02,wall_now=100.02)
        self.assertEqual(state.stamp_s,10.)
        self.assertFalse(diagnostic["callbacks"]["angular_velocity"]["new_callback"])
        np.testing.assert_array_equal(motion.observe(state),np.zeros(3))
        advance(node,10.04);node.data["angular_velocity"].xyz=[.14,.2,.3]
        state,_,_=feedback.read(node,10.04,wall_now=100.04)
        np.testing.assert_allclose(motion.observe(state),[.25,0,0],atol=1e-12)
        # A further angular callback with the SAME /clock receipt gets merged:
        # ControllerAdapter does not substitute a different omega at old time.
        node.callback_sequences["angular_velocity"]+=1
        node.data["angular_velocity"].xyz=[.16,.2,.3]
        state,_,_=feedback.read(node,10.04,wall_now=100.04)
        self.assertEqual(state.stamp_s,10.04)
        np.testing.assert_allclose(motion.observe(state),[.25,0,0],atol=1e-12)
        advance(node,10.06);node.data["angular_velocity"].xyz=[.18,.2,.3]
        state,_,_=feedback.read(node,10.06,wall_now=100.06)
        np.testing.assert_allclose(motion.observe(state),[.5,0,0],atol=1e-12)

    def test_raw_dds_epochs_are_diagnostic_not_a_shared_clock(self):
        node,reference=fixture();adapter=Px4EstimatedFeedback(reference)
        adapter.read(node,10.,wall_now=100.)
        advance(node,10.02)
        node.data["attitude"].timestamp=2;node.data["attitude"].timestamp_sample=1
        state,_,d=adapter.read(node,10.02,wall_now=100.02)
        self.assertEqual(state.stamp_s,10.02)
        self.assertEqual(d["px4_raw_timestamps"]["attitude"]["timestamp"],2)

    def test_malformed_numeric_joint_and_quaternion_rejected(self):
        for mutation in ("quaternion","joint_nan","joint_missing","joint_time","angular_nan"):
            node,reference=fixture()
            if mutation=="quaternion":node.data["attitude"].q=[2.,0,0,0]
            if mutation=="joint_nan":node.data["joints"].position[0]=float("nan")
            if mutation=="joint_missing":node.data["joints"].velocity=[]
            if mutation=="joint_time":node.data["joints"].header.stamp.sec=9
            if mutation=="angular_nan":node.data["angular_velocity"].xyz=[0,float("nan"),0]
            with self.subTest(mutation=mutation),self.assertRaises((RuntimeError,ValueError)):
                Px4EstimatedFeedback(reference).read(node,10.,wall_now=100.)




if __name__ == "__main__":
    unittest.main()
