"""No evidence hashing inside an active Offboard allocation initialization."""
import json
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
for directory in (ROOT/'simulations/x650_wholebody_mpc_v1', ROOT/'control_packages/x650_wholebody_v1',
                  ROOT/'control_packages/x650_wholebody_v1/tests'):
    sys.path.insert(0, str(directory))
import run_case
import accepted_response_model as accepted
from mixed_mpc_protocol import build_mixed_mpc_protocol
from research_condition import canonical_hash
from test_model import example_model as ArmModel


class MixedRuntimeWiringTests(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.path = Path(tmp.name)/'response_model.json'
        self.pid = {'MPC_XY_P': .95, 'MPC_Z_P': 1.}
        self.model = {
            'kind': accepted.MODEL_KIND, 'schema_version': 1,
            **{key: True for key in ('identified', 'passed', 'response_identified',
                'eligible_as_stage_gate', 'research_acceptance_verified', 'ground_truth_aided')},
            'diagnostic_only': False, 'model_structure': accepted.STRUCTURE,
            'state_order': ['vx', 'ax', 'vy', 'ay', 'vz'], 'gain': [1., 1., 1.],
            'bias_m_s': [0., 0., 0.], 'command_delay_s': 0.,
            'acceleration_feedforward_contract': accepted.FF_CONTRACT,
            'joint_response_identified': False, 'full_body_dynamics_identified': False,
            'parameters': {'x': {'wn_rad_s': 2.8, 'zeta': .7},
                           'y': {'wn_rad_s': 2.9, 'zeta': .7}, 'z': {'tau_s': .2}},
            'pid_parameters_sha256': canonical_hash(self.pid),
        }
        self.path.write_text(json.dumps(self.model))
        self.checked = {'model': self.model, 'binding': {'model': {'path': str(self.path), 'sha256': run_case.sha(self.path)}}}
        self.cfg = {'controller': 'mpc', 'response_model': str(self.path),
                    'mixed_mpc_protocol': build_mixed_mpc_protocol()}

    def test_already_admitted_model_does_not_reread_large_evidence_in_flight(self):
        with patch.object(accepted, 'load_accepted_response_model', side_effect=AssertionError('large evidence IO in flight')):
            controller = run_case.build_allocator(self.cfg, ArmModel(), self.pid, run_case.ConstantTarget([0., 0., 1.]),
                                                 accepted_response=self.checked)
        self.assertTrue(controller.mixed_response)
        self.assertEqual(controller.state_size, 12)
        self.assertEqual(controller.response_model['parameters'], self.model['parameters'])

    def test_model_bytes_changed_after_preflight_are_rejected(self):
        self.path.write_text(self.path.read_text()+'\n')
        with self.assertRaisesRegex(RuntimeError, 'changed after runtime admission'):
            run_case.build_allocator(self.cfg, ArmModel(), self.pid, None, accepted_response=self.checked)

    def test_actual_pid_change_cannot_reuse_response_model(self):
        with self.assertRaisesRegex(RuntimeError, 'actual flight PID'):
            run_case.build_allocator(self.cfg, ArmModel(), {**self.pid, 'MPC_XY_P': 1.}, None,
                                     accepted_response=self.checked)

    def test_pid_and_mpc_use_identical_declared_common_constraints(self):
        mpc = run_case.build_allocator(self.cfg, ArmModel(), self.pid, None, accepted_response=self.checked)
        baseline = run_case.build_allocator({**self.cfg, 'controller': 'pid_ff'}, ArmModel(), self.pid, None,
                                           accepted_response=self.checked)
        for key in ('velocity_limit', 'acceleration_limit', 'q_lower', 'q_upper'):
            self.assertEqual(getattr(mpc, key).tolist(), getattr(baseline, key).tolist())


if __name__ == '__main__':
    unittest.main()
