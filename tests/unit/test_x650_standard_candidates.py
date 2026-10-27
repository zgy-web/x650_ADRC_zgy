"""Schema/provenance tests only; never launches solvers or physical processes."""
from __future__ import annotations

from copy import deepcopy
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('event_candidate_audit', ROOT/'scripts/audit_x650_standard_candidates.py')
audit = importlib.util.module_from_spec(spec)
spec.loader.exec_module(audit)


class StandardCandidateAuditTests(unittest.TestCase):
    def fixture(self, root):
        config = {'nodes': 15}
        selection_path = root/'selection.json'
        cases = [dict(source='synthetic_schema_test', source_sha256='a'*64, source_index=i,
            case=dict(id='schema_'+str(i), input=dict(t=i*.04, dt=.04),
                memory=dict(step_count=i), oracle={})) for i in range(16)]
        audit.write(selection_path, dict(kind='standard_nmpc_bound_fixture_selection_v1', config=config, cases=cases))
        selection = audit.identity(selection_path)
        row = dict(index=0, selection=selection, source=cases[0]['source'],
            source_sha256=cases[0]['source_sha256'], source_index=0,
            case_sha256=audit.canonical_hash(cases[0]['case']), case_id='schema_0',
            passed=False, graph_sha256='b'*64, binding_sha256='c'*64)
        case_dir = root/'case_000'
        case_dir.mkdir()
        numeric_path = case_dir/'numeric_case.json'
        audit.write(numeric_path, dict(kind='x650_standard_nmpc_numeric_case_v1', N=15, nu=5,
            graph_sha256=row['graph_sha256'], binding_sha256=row['binding_sha256'],
            config_sha256=audit.canonical_hash(config),
            source_identity={name: row[name] for name in ('source', 'source_sha256', 'source_index', 'case_sha256')}))
        row['numeric_case'] = audit.identity(numeric_path)
        result = dict(kind='x650_standard_nmpc_solver_result_v1', backend='ipopt_reference',
            graph_sha256=row['graph_sha256'], binding_sha256=row['binding_sha256'],
            raw_status='Not_Succeeded', solver_success=False, solve_elapsed_ms=1., objective=2.,
            numerically_feasible=False, first_order_kkt_passed=False, residuals={'primal_violation': .5},
            multiple_shooting_dynamics_residual_inf=0., rollout_disagreement_inf=0., details={},
            plan=[[0.]*5 for _ in range(15)], plan_flat=[0.]*75,
            accepted_for_publication=False, native_event_validation_required=True)
        result_path = case_dir/'ipopt.json'
        audit.write(result_path, result)
        row['ipopt'] = {key: result[key] for key in audit.COMPACT_FIELDS}
        report = dict(kind='x650_standard_nmpc_offline_qualification_v1', complete=True,
            passed=False, mode='all', selected_count=16, physical_acceptance=False,
            online_acceptance=False, accepted_for_publication=False,
            configuration_sha256=audit.canonical_hash(config), selections=[selection],
            source_artifacts=[], cases=[row])
        report_path = root/'report.json'
        audit.write(report_path, report)
        audit.write(root/'delivery_manifest.json', dict(kind='x650_standard_nmpc_offline_manifest_v1',
            files=[audit.identity(p) for p in (report_path, numeric_path, result_path)]))
        return report_path

    def test_plan_requires_complete_finite_consistent_pair(self):
        value = dict(plan=[[0.]*5 for _ in range(15)], plan_flat=[0.]*75)
        self.assertEqual(audit.physical_plan(value), [0.]*75)
        for bad in (float('nan'), float('inf'), True):
            changed = deepcopy(value)
            changed['plan'][0][0] = changed['plan_flat'][0] = bad
            self.assertIsNone(audit.physical_plan(changed))
        changed = deepcopy(value)
        changed['plan_flat'][3] = .001
        self.assertIsNone(audit.physical_plan(changed))
        changed['plan_flat'].pop()
        self.assertIsNone(audit.physical_plan(changed))

    def test_failed_solver_still_audited_and_all_missing_explicit(self):
        with tempfile.TemporaryDirectory() as directory:
            values = audit.load_bound_report(self.fixture(Path(directory)))
            path, _, _, selected, rows, delivered, _, reader = values
            records, inputs = audit.collect_candidates(path, rows, selected, delivered, reader)
            self.assertEqual(len(records), 48)
            self.assertEqual(len(inputs), 1)
            self.assertFalse(records[0]['solver_numerical_passed'])
            self.assertEqual(sum(v['status']=='missing_solver_result' for v in records), 2)
            self.assertEqual(sum(v['status']=='missing_qualification_case' for v in records), 45)
            row = inputs[0]
            checked = dict(id=row['id'], plan=row['plan'], passed=True,
                task_time_s=row['input']['t'], dt_s=row['input']['dt'],
                committed_step_count=row['memory']['step_count'])
            checked.update({name: True for name in ('input_valid', 'command_bounds_passed',
                'reference_bounds_passed', 'native_checked', 'native_bounds_passed',
                'stopping_passed', 'pending_guard_passed', 'hold_passed')})
            event = dict(kind='x650_standard_common_event_replay_v1', count=1,
                accepted_count=1, records=[checked])
            audit.merge_event(records, inputs, event)
            self.assertTrue(records[0]['common_event_passed'])
            self.assertFalse(records[0]['numerical_and_common_passed'])
            self.assertFalse(records[0]['accepted_for_publication'])
            summary = audit.summarize(records)
            self.assertEqual(summary['ipopt']['common_event_passed'], 1)
            self.assertEqual(summary['ipopt']['numerical_and_common_passed'], 0)
            changed = deepcopy(event)
            changed['records'][0]['id'] = 'reused_other_source'
            with self.assertRaises(ValueError):
                audit.merge_event(records, inputs, changed)

    def test_changed_selection_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            report = self.fixture(root)
            selection = audit.read(root/'selection.json')
            selection['cases'][0]['case']['memory']['step_count'] += 1
            (root/'selection.json').write_text(json.dumps(selection))
            with self.assertRaisesRegex(ValueError, 'selection SHA'):
                audit.load_bound_report(report)

    def test_case_hash_mismatch_rejected_even_with_new_report_binding(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            report_path = self.fixture(root)
            report = audit.read(report_path)
            report['cases'][0]['case_sha256'] = 'f'*64
            report_path.write_text(json.dumps(report))
            manifest = audit.read(root/'delivery_manifest.json')
            manifest['files'][0] = audit.identity(report_path)
            (root/'delivery_manifest.json').write_text(json.dumps(manifest))
            with self.assertRaisesRegex(ValueError, 'Case provenance differs'):
                audit.load_bound_report(report_path)

    def test_in_progress_qualification_not_used(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            report_path = self.fixture(root)
            report = audit.read(report_path)
            report['complete'] = False
            report_path.write_text(json.dumps(report))
            with self.assertRaisesRegex(ValueError, 'incomplete'):
                audit.load_bound_report(report_path)


if __name__ == '__main__':
    unittest.main()
