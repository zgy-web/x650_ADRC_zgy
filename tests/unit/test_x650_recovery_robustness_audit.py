"""Counterexamples for new-flight acceptance with compact ABI 2 diagnostics."""
from copy import deepcopy
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('recovery_robust_audit',
    ROOT/'scripts/audit_x650_recovery_robustness.py')
audit = importlib.util.module_from_spec(SPEC); SPEC.loader.exec_module(audit)


def fixture():
    config = dict(nodes=2,node_dt=.04,max_dt=.05,feasibility_tolerance=2e-7,
        velocity_limit=[.55,.35,.45,.45,.35],acceleration_limit=[.45,.30,.45,.60,.60],
        q_lower=[-1.,-1.],q_upper=[1.,1.],trust_radius=.15)
    snapshot = SimpleNamespace(sequence=1,task_time_s=0.,dt_s=.02,control_time_s=0.,joint_time_s=0.,
        reference_time_s=-.005,reference_r=[0.,0.],command_time_s=[-.005],
        command_velocity=[0.,0.],command_count=1)
    centre = dict(centre_kind='slew_limited_brake',rejection='',nonlinear_passed=True,
        original_constraints_passed=True,trusted=True,projection_attempted=False,projection_solved=False,
        original_primal_violation=0.,stopping_violation=0.,elapsed_ms=2.,centre_plan=[0.]*10,
        node_times=[.02,.06],node_states=[0.]*8,node_references=[0.]*4,envelope=[0.]*8,substeps=60)
    diagnostic = dict(recovery_triggered=True,recovery_reason='QP infeasible',recovery_trust_radius=.02,
        recovery_stopping_violation=0.,recovery_attempts=[centre],subproblems=2,compute_ms=4.,
        predicted_states=[0.]*24)
    call = (snapshot,None,dict(result=dict(command=[0.]*5,diagnostics=diagnostic)))
    return config,dict(history_count=1,history_command=[0.]*5),call


class RuntimeRecoveryEvidenceTests(unittest.TestCase):
    def setUp(self): self.config,self.start,self.call=fixture()

    def check(self): return audit.check_runtime_recoveries([self.call],self.start,self.config)

    @property
    def diagnostic(self): return self.call[2]['result']['diagnostics']

    def test_actual_centre_reference_envelope_and_nonlinear_reserve_are_checked(self):
        result=self.check()
        self.assertTrue(result['verified']);self.assertEqual(result['recovery_calls'],1)
        self.assertEqual(result['projection_solves'],0)
        self.assertIn('not recaptured/reconstructed',result['compact_capture_scope'])

    def test_no_recovery_is_an_observation_not_a_required_fabricated_event(self):
        self.diagnostic.update(recovery_triggered=False,recovery_attempts=[],recovery_reason='',
            recovery_trust_radius=0.,recovery_stopping_violation=0.,subproblems=1)
        result=self.check()
        self.assertEqual(result['recovery_calls'],0);self.assertEqual(result['normal_calls'],1)
        self.assertIsNone(result['recovery_core_ms'])

    def test_changed_radius_or_fake_projection_is_rejected(self):
        self.diagnostic['recovery_trust_radius']=.15
        with self.assertRaisesRegex(ValueError,'radius'): self.check()
        self.diagnostic['recovery_trust_radius']=.02
        self.diagnostic['recovery_attempts'][0]['projection_solved']=True
        with self.assertRaisesRegex(ValueError,'without an actual attempt'): self.check()

    def test_centre_cannot_reset_reference_or_ignore_slew(self):
        centre=self.diagnostic['recovery_attempts'][0]
        centre['node_references'][0]=.01
        with self.assertRaisesRegex(ValueError,'reference was reset'): self.check()
        centre['node_references'][0]=0.
        centre['centre_plan'][3]=.1
        with self.assertRaises(ValueError): self.check()

    def test_actual_final_nonlinear_reserve_cannot_be_replaced_by_pass_flag(self):
        self.config['q_upper'][0]=.04
        self.diagnostic['predicted_states'][8]=.09
        with self.assertRaisesRegex(ValueError,'Final actual nonlinear candidate'): self.check()

    def test_trusted_centre_actual_stopping_reserve_and_envelope_are_checked(self):
        centre=self.diagnostic['recovery_attempts'][0]
        centre['node_states'][0]=2.
        with self.assertRaisesRegex(ValueError,'Trusted centre violates'): self.check()
        centre['node_states'][0]=0.
        centre['envelope'][2]=2.
        with self.assertRaisesRegex(ValueError,'nonlinear envelope violates'): self.check()

    def test_horizon_budget_and_untriggered_extra_evidence_are_rejected(self):
        centre=self.diagnostic['recovery_attempts'][0]
        centre['node_times'][1]=.08
        with self.assertRaisesRegex(ValueError,'horizon'): self.check()
        centre['node_times'][1]=.06
        self.diagnostic['subproblems']=4
        with self.assertRaisesRegex(ValueError,'budget'): self.check()
        self.diagnostic.update(subproblems=2,recovery_triggered=False)
        with self.assertRaisesRegex(ValueError,'untriggered'): self.check()

    def test_centre_and_full_call_share_original_deadline(self):
        self.diagnostic['recovery_attempts'][0]['elapsed_ms']=4.1
        with self.assertRaisesRegex(ValueError,'clock'): self.check()
        self.diagnostic.update(compute_ms=51.)
        with self.assertRaisesRegex(ValueError,'deadline'): self.check()

    def test_projection_requires_actual_attempt_and_success_in_correct_order(self):
        first=self.diagnostic['recovery_attempts'][0]
        second=deepcopy(first)
        first.update(trusted=False,rejection='centre original-row check failed')
        second.update(centre_kind='original_constraints_brake_projection',projection_attempted=True,
                      projection_solved=True)
        self.diagnostic['recovery_attempts'].append(second)
        self.assertEqual(self.check()['projection_solves'],1)
        second['projection_solved']=False
        with self.assertRaisesRegex(ValueError,'lacks an actual solved'): self.check()


class IncompletePhysicalExperimentTests(unittest.TestCase):
    def test_partial_run_preserves_original_failure_but_never_becomes_certificate(self):
        with tempfile.TemporaryDirectory() as name:
            root=Path(name);launch,run,out=(root/n for n in ('launch','run','out'))
            for folder in (launch,run,out):folder.mkdir()
            path=run/'controller_metadata.json'
            path.write_text(json.dumps(dict(complete=False,error='QP infeasible at 7.544 s',
                landed_and_disarmed=True,diagnostic_parameters_restored=True)))
            before=path.read_bytes()
            result=audit.audit(launch,run,out)
            self.assertFalse(result['passed']);self.assertFalse(result['certificate'])
            self.assertFalse(result['integrity_and_execution_passed']);self.assertFalse(result['physical_acceptance'])
            self.assertFalse(result['hardware_qualified'])
            self.assertIn(str(path),result['artifacts'])
            self.assertEqual(result['failure_context']['controller_metadata.json']['error'],'QP infeasible at 7.544 s')
            self.assertEqual(before,path.read_bytes())

    def test_native_rejection_diagnostics_are_preserved_without_success_relabelling(self):
        with tempfile.TemporaryDirectory() as name:
            run=Path(name);folder=run/'isolated_live/native_records';folder.mkdir(parents=True)
            (folder/'000379.json').write_text(json.dumps(dict(sequence=379,error='no trusted recovery centre',
                rejected_diagnostics=dict(accepted=False,recovery_triggered=True,recovery_attempts=[]))))
            context=audit.failure_context(run)
            self.assertFalse(context['physical_acceptance']);self.assertFalse(context['full_window_validated'])
            self.assertEqual(context['last_native_record']['sequence'],379)
            self.assertFalse(context['last_native_record']['diagnostics']['accepted'])

    def test_prefix_metrics_exclude_takeover_and_landing_after_original_snapshot(self):
        # The decoder stands in for a typed CDR dependency only; no flight,
        # source, worker or certificate checker is replaced with a success.
        with tempfile.TemporaryDirectory() as name:
            run=Path(name);folder=run/'isolated_live/native_records';folder.mkdir(parents=True)
            (run/'isolated_live/live_start.json').write_text(json.dumps(dict(origin_s=10.,
                target_position_enu=[0.,0.,0.])))
            cdr=folder/'000379.snapshot.cdr';cdr.write_bytes(b'synthetic typed snapshot input')
            (folder/'000379.json').write_text(json.dumps(dict(sequence=379,error='QP infeasible',
                snapshot_cdr=audit.identity(cdr))))
            fields=['sim_s','tip_x','tip_y','tip_z','valid_base','valid_joint_state','valid_tip']
            with (run/'native_trace.csv').open('w',newline='') as stream:
                writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader()
                for tick in range(6001):
                    t=tick*.004
                    writer.writerow(dict(sim_s=10.+t,tip_x=100. if t>7.544+1e-9 else 0.,tip_y=0.,tip_z=0.,
                        valid_base=1,valid_joint_state=1,valid_tip=1))
            snapshot=SimpleNamespace(sequence=379,control_time_s=17.544)
            modules={'rclpy':SimpleNamespace(),
                'rclpy.serialization':SimpleNamespace(deserialize_message=lambda raw,kind:snapshot),
                'flyarm_control_interfaces':SimpleNamespace(),
                'flyarm_control_interfaces.msg':SimpleNamespace(ControlSnapshot=object)}
            with patch.dict('sys.modules',modules): context=audit.failure_context(run)
            metric=context['native_prefix_error']
            self.assertNotIn('read_error',metric)
            self.assertAlmostEqual(metric['last_task_s'],7.544)
            self.assertLess(metric['position_max_m'],1.)
            self.assertTrue(metric['time_weighted']);self.assertFalse(metric['full_24s_observed'])


if __name__=='__main__':unittest.main()
