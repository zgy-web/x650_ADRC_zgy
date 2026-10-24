"""Full new placement evidence and strict backwards-compatible mask checks."""
from copy import deepcopy
import ast
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'ros2_ws/src/flyarm_mpc_reference'))
from flyarm_mpc_reference.worker_affinity import WorkerAffinity, verify_execution_resource_evidence

MASK = [87, 89, 91, 97]
ROLES = {'executor': [87], 'auxiliary': [89, 91, 97]}


class ResourceAuditTests(unittest.TestCase):
    def setUp(self):
        (ROOT/'tmp').mkdir(exist_ok=True)
        self.temp = tempfile.TemporaryDirectory(dir=ROOT/'tmp', prefix='worker-affinity-')
        self.addCleanup(self.temp.cleanup); self.out = Path(self.temp.name)
        self.resources = {'roles': {'source': [17], 'worker': MASK, 'dispatcher': [93],
            'ingress': [94], 'consumption': [95]}, 'background': [0, 1]}
        self.protocol = {'cpu_roles': self.resources['roles'], 'worker_thread_roles': ROLES}
        self.observed = {role: {'5': cpus} for role, cpus in self.resources['roles'].items()}
        self.observed.update(recorder={'7': [0, 1]}, source_recorder={'8': [0, 1]})
        rows = {'10': {'start_ticks': 100, 'cpus': list(MASK), 'comm': 'python'},
                '11': {'start_ticks': 101, 'cpus': list(MASK), 'comm': 'dds'}}
        def setter(tid, cpus): rows[str(tid)]['cpus'] = sorted(cpus)
        times = iter([10, 20, 40, 50])
        helper = WorkerAffinity(MASK, ROLES, pid=10, snapshot=lambda pid: deepcopy(rows),
                                setter=setter, clock_ns=lambda: next(times))
        helper.bind_before_ready(); helper.check('ready')
        helper.write(self.out/'worker_affinity_ready.json')
        self.save('worker_affinity_at_handoff.json', {'kind': 'worker_thread_affinity_handoff_v1',
            'worker_pid': 10, 'monotonic_ns': 30, 'threads': rows})
        self.observed['worker'] = {tid: r['cpus'] for tid, r in rows.items()}
        helper.check('live_start'); helper.check('final'); helper.write(self.out/'worker_affinity_final.json')
        self.save('worker_timing.json', {'columns': ['sequence', 'callback_entry_ns','callback_exit_ns','callback_native_tid_begin',
            'callback_actual_cpu_begin', 'callback_native_tid_end', 'callback_actual_cpu_end',
            'executor_affinity_checked'], 'rows': [[1, 41, 49, 10, 87, 10, 87, 3]], 'complete': True, 'dropped': 0})

    def test_callbacks_outside_observed_lifecycle_are_rejected(self):
        original=self.load('worker_timing.json')
        for index,value in ((1,39),(2,51),(2,40)):
            doc=deepcopy(original);doc['rows'][0][index]=value
            self.save('worker_timing.json',doc)
            with self.assertRaisesRegex(ValueError,'outside placement stage window'):self.verify()

    def save(self, name, value): (self.out/name).write_text(json.dumps(value))
    def load(self, name): return json.loads((self.out/name).read_text())
    def verify(self): return verify_execution_resource_evidence(self.out, self.resources, self.protocol, self.observed)

    def test_complete_split_placement_passes(self):
        result = self.verify()
        self.assertTrue(result['verified']); self.assertEqual(result['executor_cpu'], 87)

    def test_legacy_missing_declaration_uses_exact_full_worker_mask(self):
        self.protocol.pop('worker_thread_roles')
        self.observed['worker'] = {'10': MASK, '11': MASK}
        for path in self.out.iterdir(): path.unlink()
        self.assertEqual(self.verify()['thread_policy'], 'legacy_all_threads_full_worker_mask')
        self.observed['worker']['11'] = [89, 91, 97]
        with self.assertRaises(ValueError): self.verify()

    def test_nonworker_resource_checks_are_not_relaxed(self):
        self.observed['recorder']['7'] = [0, 1, 87]
        with self.assertRaisesRegex(ValueError, 'recorder'): self.verify()

    def test_unknown_handoff_thread_and_reused_final_tid_fail(self):
        for filename, change in [('worker_affinity_at_handoff.json', 'unknown'),
                                 ('worker_affinity_final.json', 'reuse')]:
            original = self.load(filename); document = deepcopy(original)
            if change == 'unknown': document['threads']['12'] = {'start_ticks': 150, 'cpus': ROLES['auxiliary'], 'comm': 'late'}
            else: document['phases']['final']['threads']['11']['start_ticks'] += 1
            self.save(filename, document)
            with self.subTest(change=change), self.assertRaises(ValueError): self.verify()
            self.save(filename, original)

    def test_incomplete_or_wrong_callback_observations_fail(self):
        original = self.load('worker_timing.json')
        for column, value in [('callback_actual_cpu_begin', 97), ('callback_actual_cpu_end', 89),
                              ('callback_native_tid_end', 11), ('executor_affinity_checked', 1)]:
            document = deepcopy(original)
            document['rows'][0][document['columns'].index(column)] = value
            self.save('worker_timing.json', document)
            with self.subTest(column=column), self.assertRaises(ValueError): self.verify()
        self.save('worker_timing.json', {**original, 'dropped': 1})
        with self.assertRaises(ValueError): self.verify()

    def test_raw_mask_identity_and_clock_mismatch_fail(self):
        handoff = self.load('worker_affinity_at_handoff.json')
        for field, value in [('worker_pid', 99), ('monotonic_ns', 45)]:
            self.save('worker_affinity_at_handoff.json', {**handoff, field: value})
            with self.subTest(field=field), self.assertRaises(ValueError): self.verify()
        self.save('worker_affinity_at_handoff.json', handoff)
        self.observed['worker']['11'] = [89]
        with self.assertRaises(ValueError): self.verify()

    def test_ready_evidence_cannot_change_between_stages(self):
        document = self.load('worker_affinity_final.json')
        document['phases']['ready']['monotonic_ns'] += 1
        self.save('worker_affinity_final.json', document)
        with self.assertRaisesRegex(ValueError, 'startup evidence changed'): self.verify()

    def test_declared_ros_json_parameters_remain_strings_after_yaml_parsing(self):
        import yaml
        source = (ROOT/'simulations/x650_isolated_live_v1/run_case.py').read_text()
        tree = ast.parse(source)
        declaration = next(n for n in ast.walk(tree) if isinstance(n, ast.Assign)
                           and any(isinstance(t, ast.Name) and t.id == 'worker_placement' for t in n.targets)
                           and isinstance(n.value, ast.Dict) and n.value.keys)
        from types import SimpleNamespace
        ns = {'json': json, 'self': SimpleNamespace(resources=self.resources, worker_thread_roles=ROLES)}
        exec(compile(ast.Module(body=[declaration], type_ignores=[]), '<actual-worker-cli-values>', 'exec'), ns)
        for name, expected in [('worker_resource_mask_json', MASK), ('worker_thread_roles_json', ROLES)]:
            parsed = yaml.safe_load(ns['worker_placement'][name])
            self.assertIsInstance(parsed, str)
            self.assertEqual(json.loads(parsed), expected)


if __name__ == '__main__': unittest.main()
