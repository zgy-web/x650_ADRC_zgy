"""New scenario bindings cannot retune the already accepted native backend."""
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT),str(ROOT/'src'),str(ROOT/'scripts')]
import qualify_x650_multirate_robustness as q


class StartupBindingTests(unittest.TestCase):
    def fixture(self):
        original=dict(kind='cpp_full_pipeline_v1',simulation_only=True,timing_profile=q.profile.NAME,
            config=dict(nodes=15,node_dt=.04,max_dt=.1,command_hold_s=.16,trust_radius=.02),
            task_config=dict(path='/original/task.json',sha256='old_task'),trajectory=dict(kind='hold'),
            core_library=dict(path='/installed/libcore.so',sha256='original_core'),
            source_artifacts=[dict(path='/old.py',sha256='old')],unrecognized_preserved_field=[1,2,3])
        task=dict(path='/new/task.json',sha256='new_task')
        sources=[dict(path='/new.py',sha256='new',bytes=123,relative_path='new.py')]
        return original,task,dict(kind='figure8',period_s=8.),sources

    def test_task_and_sources_change_but_all_numeric_and_binary_fields_are_preserved(self):
        args=self.fixture();before=deepcopy(args)
        result=q.inherit_startup(*args)
        original,task,trajectory,sources=args
        self.assertEqual(args,before)
        self.assertEqual(result['config'],original['config'])
        self.assertEqual(result['core_library'],original['core_library'])
        self.assertEqual(result['unrecognized_preserved_field'],[1,2,3])
        self.assertEqual(result['task_config'],task)
        self.assertEqual(result['trajectory'],trajectory)
        self.assertEqual(result['source_artifacts'],[dict(path='/new.py',sha256='new')])
        result['config']['trust_radius']=999
        self.assertEqual(original['config']['trust_radius'],.02)

    def test_nonaccepted_profile_or_non_simulation_startup_rejects(self):
        for key,value in [('kind','legacy'),('simulation_only',False),('timing_profile','other')]:
            args=self.fixture();args[0][key]=value
            with self.subTest(key=key),self.assertRaises(ValueError):q.inherit_startup(*args)

    def test_task_binding_excludes_byte_count_without_losing_real_hash(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'task.json';path.write_text('{"original":true}')
            result=q.binding(path);identity=q.identity(path)
            self.assertEqual(set(result),{'path','sha256'})
            self.assertEqual(result['sha256'],identity['sha256'])
            self.assertEqual(result['path'],identity['path'])


if __name__=='__main__':unittest.main()
