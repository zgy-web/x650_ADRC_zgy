"""Probe policy tests: old rejected output/memory must never become oracle."""
from copy import deepcopy
from pathlib import Path
import sys
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'scripts'))
import probe_x650_interval_worker as probe


def fixture():
    rows=[dict(sequence=i) for i in range(1,430)]
    recording=dict(kind='x650_original_cpp_recording_export_v1',originals_verified_unchanged=True,
        config={'declared':'original'},trajectory={'kind':'fast_figure8'},rows=deepcopy(rows),
        live_start={'configuration_sha256':[0]*32})
    startup=dict(config=recording['config'],trajectory=recording['trajectory'],task_config={'sha256':'00'*32})
    oracle=dict(kind='x650_cpp_interval_sequence_oracle_v1',core_abi=3,passed=True,complete=True,
        row_count=429,step_count_after=429,memory_sequentially_reconstructed_in_cpp=True,
        original_proposals_used_as_output_oracle=False,recorded_states_and_actual_history_preserved=True,rows=rows)
    return recording,startup,oracle


class IntervalWorkerProbeTests(unittest.TestCase):
    def test_new_sequential_oracle_admitted(self):probe.validate_oracle(*fixture())

    def test_old_fixed_memory_or_original_output_oracle_rejected(self):
        for field,value in (('core_abi',2),('memory_sequentially_reconstructed_in_cpp',False),
            ('original_proposals_used_as_output_oracle',True),('step_count_after',378),
            ('passed',False),('complete',False)):
            r,s,o=fixture();o[field]=value
            with self.subTest(field=field),self.assertRaises(ValueError):probe.validate_oracle(r,s,o)

    def test_incomplete_or_permuted_snapshot_prefix_rejected(self):
        for change in ('short','permuted'):
            r,s,o=fixture()
            if change=='short':o['rows'].pop()
            else:o['rows'][400],o['rows'][401]=o['rows'][401],o['rows'][400]
            with self.subTest(change=change),self.assertRaises(ValueError):probe.validate_oracle(r,s,o)

    def test_parameter_or_task_rebinding_rejected(self):
        r,s,o=fixture();s['config']={'declared':'changed'}
        with self.assertRaises(ValueError):probe.validate_oracle(r,s,o)
        r,s,o=fixture();s['task_config']['sha256']='11'*32
        with self.assertRaises(ValueError):probe.validate_oracle(r,s,o)

    def test_only_independent_elapsed_measurements_ignored(self):
        wanted=dict(interval_repairs=[dict(elapsed_ms=12.,points=[dict(time=.0525,value=.422)])],compute_ms=20.,accepted=True)
        actual=deepcopy(wanted);actual['compute_ms']=22.;actual['interval_repairs'][0]['elapsed_ms']=14.
        self.assertEqual(probe.structural_error(actual,wanted),0.)
        actual['interval_repairs'][0]['points'][0]['value']=.425
        self.assertGreater(probe.structural_error(actual,wanted),1e-8)

    def test_missing_nonfinite_or_false_branch_cannot_match(self):
        for actual in ({},{'accepted':1},{'accepted':False}):
            with self.subTest(actual=actual),self.assertRaises(ValueError):probe.structural_error(actual,{'accepted':True})
        for value in (float('nan'),float('inf')):
            with self.assertRaises(ValueError):probe.structural_error([value],[.42])


if __name__=='__main__':unittest.main()
