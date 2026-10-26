"""Compact ABI-3 flight audit counterexamples; no ROS, solver or actuator I/O."""
from copy import deepcopy
import importlib.util
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from uuid import uuid4

import numpy as np

ROOT=Path(__file__).resolve().parents[2]
SPEC=importlib.util.spec_from_file_location('interval_flight_audit',ROOT/'scripts/audit_x650_interval_robustness.py')
audit=importlib.util.module_from_spec(SPEC);SPEC.loader.exec_module(audit)
from tests.unit.test_x650_interval_constraints_audit import accepted_fixture,add_cut


def fixture(with_cut=False):
    f,raw,_=accepted_fixture();c=f['config'];c['integration_step']=.001
    if with_cut:add_cut(f,raw)
    d=raw['diagnostics'];ctx=f['input']['context'];state=f['input']['state']
    d.setdefault('interval_centre_plan',[]);d.setdefault('interval_canonical_boundaries',[])
    d.setdefault('interval_canonical_states',[]);d.setdefault('interval_canonical_envelope',[0.]*8)
    d.setdefault('interval_canonical_substeps',0)
    s=SimpleNamespace(sequence=1,task_time_s=f['input']['t'],dt_s=f['input']['dt'],
        control_time_s=state['control_time'],joint_time_s=state['joint_stamp'],
        q=state['q'],dq=state['dq'],reference_time_s=ctx['reference_time'],reference_r=ctx['reference'],
        bus_origin_s=ctx['bus_origin'],command_count=len(ctx['commands']),
        command_time_s=[a['time'] for a in ctx['commands']],command_velocity=[v for a in ctx['commands'] for v in a['velocity']],
        packet_count=len(ctx['packets']),packet_send_time_s=[a['send_time'] for a in ctx['packets']])
    for name in ('q_ref','dq_ref','tau_ff','kp','kd'):
        setattr(s,'packet_'+name,[v for p in ctx['packets'] for v in p[name]])
    if with_cut:
        count=int(sum(max(1,int(np.ceil(dt/.001-1e-12))) for dt in np.diff(d['candidate_boundary_times']) if dt>0))
        d['candidate_substeps']=d['interval_canonical_substeps']=count
        for repair in d['interval_repairs']:repair['canonical_substeps']=count
    start=dict(history_count=1,history_command=[0.]*5)
    return c,start,(s,None,dict(result=dict(command=[0.]*5,diagnostics=d)))


class IntervalFlightAuditTests(unittest.TestCase):
    def setUp(self):self.config,self.start,self.call=fixture(True)
    @property
    def d(self):return self.call[2]['result']['diagnostics']
    def check(self):return audit.check_runtime_intervals([self.call],self.start,self.config)

    def test_actual_cut_metadata_and_packet_encoding_are_checked_without_full_qp_claim(self):
        result=self.check();self.assertTrue(result['verified']);self.assertEqual(result['interval_calls'],1)
        self.assertIn('not recaptured or rerun',result['compact_capture_scope'])
        self.assertTrue(result['no_slack_variables'])

    def test_no_interval_event_is_valid_without_fabricating_recovery(self):
        c,start,call=fixture(False);result=audit.check_runtime_intervals([call],start,c)
        self.assertEqual(result['interval_calls'],0)
        call[2]['result']['diagnostics']['interval_rows']=1
        with self.assertRaisesRegex(ValueError,'shape'):audit.check_runtime_intervals([call],start,c)

    def test_feasible_constant_zero_rows_may_be_omitted_without_claiming_full_matrix_proof(self):
        c,start,call=fixture(False);call[2]['result']['diagnostics']['qp_rows']=224
        result=audit.check_runtime_intervals([call],start,c)
        self.assertEqual(result['original_base_rows']['observed_min'],224)
        self.assertEqual(result['original_base_rows']['allowed_min'],9*15+5)
        self.assertFalse(result['original_base_rows']['full_base_matrix_independently_verified'])
        self.d['qp_rows']=225;self.d['interval_constraints'][0]['qp_row']=224
        result=self.check();self.assertEqual(result['original_base_rows']['observed_max'],224)

    def test_dynamic_base_count_bounds_and_integer_shape_are_required(self):
        for rows in (9*15+4,15*15+6,230.,True):
            c,start,call=fixture(False);call[2]['result']['diagnostics']['qp_rows']=rows
            with self.subTest(rows=rows),self.assertRaises(ValueError):audit.check_runtime_intervals([call],start,c)
        for columns in (74,76,75.,True):
            c,start,call=fixture(False);call[2]['result']['diagnostics']['qp_columns']=columns
            with self.subTest(columns=columns),self.assertRaises(ValueError):audit.check_runtime_intervals([call],start,c)

    def test_appended_cut_index_uses_actual_dynamic_base_not_fixed230(self):
        self.d['qp_rows']=225
        with self.assertRaisesRegex(ValueError,'cut identity or QP row'):self.check()
        self.d['interval_constraints'][0]['qp_row']=224
        self.check()
        self.d['interval_constraints'][0]['gradient'].append(0.)
        with self.assertRaisesRegex(ValueError,'shape'):self.check()

    def test_final_native_envelope_and_original_padding_cannot_be_waived(self):
        self.d['candidate_envelope'][4]=-.42001
        with self.assertRaisesRegex(ValueError,'nonlinear envelope'):self.check()

    def test_budget_and_no_slack_dimensions_remain_original(self):
        for field,value in [('compute_ms',50.001),('subproblems',4),('qp_columns',76),('qp_rows',232)]:
            c,start,call=fixture(True);call[2]['result']['diagnostics'][field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):audit.check_runtime_intervals([call],start,c)

    def test_original_violation_trigger_and_source_identity_are_required(self):
        self.d['interval_repairs'][0]['trigger_points'][0]['value']=-.41
        with self.assertRaisesRegex(ValueError,'hard native violation trigger'):self.check()

    def test_changed_gradient_encoding_or_anchor_are_rejected(self):
        for field in ('gradient','encoding_lower_remainder','affine_offset','qp_row'):
            c,start,call=fixture(True);cut=call[2]['result']['diagnostics']['interval_constraints'][0]
            if field=='gradient':cut[field][3]+=.001
            elif field=='qp_row':cut[field]+=1
            else:cut[field]+=.001
            with self.subTest(field=field),self.assertRaises((ValueError,RuntimeError)):
                audit.check_runtime_intervals([call],start,c)

    def test_changed_risk_encoding_and_grid_are_rejected(self):
        self.d['interval_repairs'][0]['risk_encoding_lower'][0]-=.001
        with self.assertRaisesRegex(ValueError,'encoding range'):self.check()
        self.d['interval_repairs'][0]['risk_encoding_lower'][0]+=.001
        self.d['interval_repairs'][0]['canonical_substeps']+=1
        with self.assertRaisesRegex(ValueError,'canonical substep'):self.check()

    def test_actual_history_changes_cannot_be_replaced_by_current_state(self):
        self.call[0].reference_r=[.1,0.]
        with self.assertRaises((ValueError,RuntimeError)):self.check()

    def test_incomplete_physical_run_cannot_issue_acceptance(self):
        base=ROOT/'tmp'/('interval_flight_audit_'+uuid4().hex);run=base/'run';launch=base/'launch';out=base/'audit'
        for path in (run,launch,out):path.mkdir(parents=True,exist_ok=True)
        (run/'controller_metadata.json').write_text(json.dumps(dict(complete=False,error='incomplete original 24 s')))
        report=audit.audit(launch,run,out)
        self.assertFalse(report['passed']);self.assertFalse(report['physical_acceptance'])
        self.assertFalse(report['integrity_and_execution_passed']);self.assertFalse(report['overall_disturbance_test_passed'])
        self.assertEqual(report['kind'],'x650_cpp_interval_robustness_stage_certificate_v1')
        self.assertIn('Incomplete flight lifecycle',report['error'])


if __name__=='__main__':unittest.main()
