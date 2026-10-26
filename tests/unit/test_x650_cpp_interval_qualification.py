"""Focused synthetic ABI-3 admission counterexamples; no ROS or flight."""
from copy import deepcopy
from pathlib import Path
import tempfile
import unittest

from simulations.x650_cpp_interval_v1 import qualification as q


def numeric_summary():
    return dict(kind='x650_interval_hard_constraint_software_v1', passed=True,
        closed_loop=False, actuator_io=False, hardware_qualified=False, slack_variables=0,
        inner_velocity_limits=[.42, .32], expected_libraries=q.EXPECTED_LIBRARIES.copy(),
        calls=429, passed_calls=429, core_ms_max=28.1)


class NumericAdmissionTests(unittest.TestCase):
    def test_frozen_summary_is_software_only(self):
        q.check_numeric_summary(numeric_summary())

    def test_wrong_library_relaxed_margin_slack_or_late_replay_rejects(self):
        for key, value in [('passed', False), ('closed_loop', True), ('slack_variables', 1),
                           ('inner_velocity_limits', [.45, .35]), ('passed_calls', 428),
                           ('core_ms_max', 50.0001), ('hardware_qualified', True)]:
            with self.subTest(key=key):
                changed=numeric_summary();changed[key]=value
                with self.assertRaises(ValueError):q.check_numeric_summary(changed)
        changed=numeric_summary();changed['expected_libraries']['optimization']='0'*64
        with self.assertRaises(ValueError):q.check_numeric_summary(changed)

    def test_critical_headers_and_old_protocol_helper_are_explicit(self):
        self.assertIn('control_packages/x650_wholebody_v1/native/canonical_interval.hpp',q.EXECUTION_SOURCES)
        self.assertIn('control_packages/x650_wholebody_v1/native/joint_safety.hpp',q.EXECUTION_SOURCES)
        protocol=dict(cpu_roles={'worker':[2,4], 'source':[0]},
            worker_thread_roles={'executor':[2], 'auxiliary':[4]}, cpu_scope='owned only',
            output_period_s=.02, core_compute_max_s=.05, consumption_progress_budget_ms=40)
        q.validate_protocol(protocol,protocol)
        changed=deepcopy(protocol);changed['core_compute_max_s']=.06
        with self.assertRaises(ValueError):q.validate_protocol(changed,protocol)

    def test_partial_or_counterfactual_regression_is_not_admission(self):
        report=dict(kind='x650_interval_fixed_original_regression_v1',complete=True,passed=True,
            closed_loop=False,actuator_io=False,delay_injection=False,expected_libraries=q.EXPECTED_LIBRARIES,
            sequences=list(range(1,430)),passed_cases=429,rows=[{}]*429)
        for key,value in [('complete',False),('closed_loop',True),('passed_cases',428),
                          ('sequences',list(range(1,429))),('delay_injection',True)]:
            changed=deepcopy(report);changed[key]=value
            with self.assertRaisesRegex(ValueError,'429 fixed-original'):
                q.verify_regression(changed,{},None)


class Graph:
    """In-memory document fixture for sequence logic; not a file-integrity mock."""
    def __init__(self,root):self.root=root;self.documents={}
    def put(self,name,value):
        path=str(self.root/name);self.documents[path]=value
        return dict(path=path,sha256='a'*64)
    def document(self,binding):return self.documents[binding['path']]
    def artifact(self,binding):return Path(binding['path'])


def worker_graph(root):
    evidence=Graph(root)
    task_path=root/'task.json';task_path.write_bytes(b'{"same_task":true}\n')
    task=dict(path=str(task_path),sha256=q.sha(task_path))
    worker=dict(path=str(root/'isolated_worker'),sha256='b'*64)
    libraries={k:dict(path=str(root/(k+'.so')),sha256=v) for k,v in q.EXPECTED_LIBRARIES.items()}
    startup=dict(config={'nodes':15},trajectory={'kind':'fast_figure8'},task_config=task,
        **{k+'_library':libraries[k] for k in libraries})
    startup_id=evidence.put('startup.json',deepcopy(startup))
    actual_startup=evidence.put('worker_startup.json',dict(core_abi=3,diagnostic_mode='compact',
        python_in_worker=False,startup_controller_steps=0,startup_manifest=deepcopy(startup),
        **{'loaded_'+role+'_path':receipt['path'] for role,receipt in libraries.items()}))
    sources=[dict(path=str(root/f'source_{i}.json'),sha256='c'*64,sequence=i) for i in range(1,430)]
    recording=evidence.put('recording.json',dict(kind='x650_original_cpp_recording_export_v1',
        originals_verified_unchanged=True,config=startup['config'],trajectory=startup['trajectory'],rows=sources))
    calls=[]
    for i,source in enumerate(sources,1):
        call=evidence.put(f'oracle/{i}.json',dict(kind='x650_cpp_interval_sequence_oracle_call_v1',
            sequence=i,returned=True,source_row=source,memory_before=dict(step_count=i-1),memory_after=dict(step_count=i)))
        call['sequence']=i;calls.append(call)
    oracle_value=dict(kind='x650_cpp_interval_sequence_oracle_v1',core_abi=3,complete=True,passed=True,
        row_count=429,step_count_after=429,memory_sequentially_reconstructed_in_cpp=True,
        original_proposals_used_as_output_oracle=False,recorded_states_and_actual_history_preserved=True,
        physical_validation=False,counterfactual_closed_loop_claimed=False,recording=recording,
        rows=calls,loaded_code=libraries)
    oracle=evidence.put('oracle.json',oracle_value)
    rows=[dict(sequence=i,passed=True,native_callback_ms=30.,absolute_errors={'commands':0.},
        full_native_output_absolute_errors={'all_fields':0.},diagnostic_absolute_error=0.) for i in range(1,430)]
    wires=[dict(sequence=i,snapshot_all_fields_exact=True,proposal_all_fields_exact=True) for i in range(1,430)]
    probe=dict(kind='x650_cpp_interval_actual_worker_probe_v1',core_abi=3,complete=True,passed=True,
        no_actuator_publishers=True,physical_acceptance=False,physical_validation=False,
        counterfactual_closed_loop_claimed=False,injected_delay=False,test_sequence_complete=True,
        rows_count=429,child_exit=0,
        worker=worker,startup=startup_id,worker_startup=actual_startup,libraries=libraries,oracle=oracle,recording=recording,
        final=dict(requests=429,accepted=429,faulted=False),
        recorder=dict(complete=True,dropped=0,written=429,offered=429),
        records=[{}]*429,wire_crosscheck=wires,rows=rows)
    return evidence,startup,probe,worker,{worker['path']:worker['sha256']},oracle_value


class WorkerAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.temporary=tempfile.TemporaryDirectory();self.addCleanup(self.temporary.cleanup)
        self.graph,self.startup,self.probe,self.worker,self.binaries,self.oracle=worker_graph(Path(self.temporary.name))
    def verify(self):return q.verify_worker(self.probe,self.startup,self.graph,self.binaries,self.worker)

    def test_actual_new_worker_with_own_sequential_memory_admits_software_only(self):
        result=self.verify();self.assertTrue(result['verified']);self.assertFalse(result['physical_acceptance'])

    def test_old_abi_original_output_oracle_or_missing_prefix_rejects(self):
        self.probe['core_abi']=2
        with self.assertRaises(ValueError):self.verify()
        self.probe['core_abi']=3;self.oracle['original_proposals_used_as_output_oracle']=True
        with self.assertRaisesRegex(ValueError,'oracle'):self.verify()
        self.oracle['original_proposals_used_as_output_oracle']=False;self.oracle['rows'].pop()
        with self.assertRaises(ValueError):self.verify()

    def test_sequential_memory_cannot_be_replaced_by_original_fixture_memory(self):
        binding=self.oracle['rows'][10]
        self.graph.documents[binding['path']]['memory_before']={'step_count':10,'original_fixture_only':True}
        with self.assertRaisesRegex(ValueError,'memory was not advanced'):self.verify()

    def test_late_callback_wire_change_or_diagnostic_difference_rejects(self):
        self.probe['rows'][-1]['native_callback_ms']=50.001
        with self.assertRaisesRegex(ValueError,'deadline'):self.verify()
        self.probe['rows'][-1]['native_callback_ms']=30.
        self.probe['wire_crosscheck'][-1]['snapshot_all_fields_exact']=False
        with self.assertRaisesRegex(ValueError,'wire identity'):self.verify()
        self.probe['wire_crosscheck'][-1]['snapshot_all_fields_exact']=True
        self.probe['rows'][-1]['diagnostic_absolute_error']=1e-5
        with self.assertRaisesRegex(ValueError,'diagnostic'):self.verify()

    def test_installed_worker_library_and_recording_task_are_bound(self):
        self.binaries.clear()
        with self.assertRaisesRegex(ValueError,'Installed'):self.verify()
        self.binaries[self.worker['path']]=self.worker['sha256']
        self.probe['libraries']['predictor']['sha256']='0'*64
        with self.assertRaisesRegex(ValueError,'library'):self.verify()


if __name__=='__main__':unittest.main()
