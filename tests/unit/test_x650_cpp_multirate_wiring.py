"""Exercise actual launch argument/config exporters without a ROS graph."""
import importlib.util
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('x650_adapter_harness', ROOT/'tests/unit/test_x650_cpp_live_adapter.py')
harness = importlib.util.module_from_spec(spec)
spec.loader.exec_module(harness)


class MultirateWiringTests(unittest.TestCase):
    def setUp(self):
        self.ns = harness.load()

    def test_named_budget_and_legacy_default_reject_unknown(self):
        select = self.ns['timing_profile_parameters']
        self.assertEqual(select({})['name'], 'legacy_v1')
        new = select({'timing_profile': 'mpc25_dispatch50_v1'})
        self.assertEqual((new['nominal_solve_period_ns'], new['output_period_ns'], new['command_lease_ns'],
                          new['control_dt_max_s']), (40_000_000, 20_000_000, 160_000_000, .1))
        self.assertEqual((new['solve_roundtrip_max_ns'], new['source_wall_age_max_ns'],
                          new['consumption_progress_max_ns']), (50_000_000, 25_000_000, 40_000_000))
        self.assertFalse(new['hard_realtime_qualified'])
        for unknown in (None, '', False, 'mpc25_dispatch25_v1'):
            with self.assertRaises(ValueError):
                select({'timing_profile': unknown})

    def test_profile_reaches_both_gateways_and_worker_without_changing_consumption(self):
        item = self.ns['CppLiveExtension']()
        item.protocol = dict(execution_revision=2, consumption_policy='latest_value_v1',
                             consumption_progress_budget_ms=40, timing_profile='mpc25_dispatch50_v1')
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for package, executable in [('flyarm_control_gateway', 'state_ingress'),
                    ('flyarm_control_gateway', 'live_dispatcher'), ('flyarm_mpc_cpp', 'isolated_worker')]:
                path = root/'lib'/package/executable
                path.parent.mkdir(parents=True, exist_ok=True)
                path.touch()
                item.prefix = lambda _: directory
                original = dict(simulation_live=True)
                command = item._ros(package, executable, original)
                self.assertIn('timing_profile:=mpc25_dispatch50_v1', command)
                self.assertEqual(original, dict(simulation_live=True))
                if executable == 'live_dispatcher':
                    self.assertIn('consumption_progress_budget_ms:=40', command)
                else:
                    self.assertFalse(any('consumption_' in value for value in command))
                with self.assertRaisesRegex(ValueError, 'Conflicting'):
                    item._ros(package, executable, dict(timing_profile='legacy_v1'))
            item.protocol.pop('timing_profile')
            legacy = item._ros('flyarm_control_gateway', 'state_ingress', {})
            self.assertFalse(any('timing_profile' in value for value in legacy))
            with self.assertRaisesRegex(ValueError, 'Conflicting'):
                item._ros('flyarm_control_gateway', 'state_ingress', dict(timing_profile='mpc25_dispatch50_v1'))

    def test_core_export_only_changes_explicit_time_guards(self):
        cfg = dict(mpc_config=dict(nodes=15, max_iterations=60, feasibility_tolerance=2e-7,
            kkt_tolerance=.002, arm_share=.5, joint_position_gain=1., node_dt_s=.04, max_dt_s=.05,
            velocity_limit=[.55, .35, .45, .45, .35], acceleration_limit=[1.]*5,
            q_lower=[-.75, -.35], q_upper=[.45, .85], tip_position_weight=400., tip_velocity_weight=4.,
            base_position_weight=.25, joint_posture_weight=.02, input_nominal_weight=.01, input_delta_weight=.01))
        model = NS(joint_lower=[-1., -1.], joint_upper=[1., 1.], lengths=[.2, .2],
                   angle_offsets=[0., 0.], mount_body=[0., 0., -.1], elbow_offset_body_m=[0., 0., 0.])
        predictor = NS(J=.01, b=.01, Fc=.01, offset=0., delay_s=.01, Fs=.02, delta=.01, vs=.01,
                       kp=[6., 6.], kd=[.25, .25], bus_period_s=.005, max_step_s=.0001, trust_radius_normalized=.02)
        prior = dict(parameters=dict(x=dict(wn_rad_s=2., zeta=1.), y=dict(wn_rad_s=2., zeta=1.), z=dict(tau_s=.2)))
        self.ns['shared'] = NS(json_value=lambda value: value)
        export = self.ns['expected_core_config']
        args = (cfg, model, {'MPC_XY_P': 1., 'MPC_Z_P': 1.}, predictor, prior)
        legacy = export(*args)
        new = export(*args, timing_profile='mpc25_dispatch50_v1')
        self.assertNotIn('command_hold_s', legacy)
        self.assertEqual((new['command_hold_s'], new['max_dt']), (.16, .1))
        compared = dict(new)
        compared.pop('command_hold_s')
        compared['max_dt'] = legacy['max_dt']
        self.assertEqual(compared, legacy)
        self.assertEqual(new['bus_period'], .005)
        with self.assertRaises(ValueError):
            export(*args, timing_profile='')


if __name__ == '__main__':
    unittest.main()
