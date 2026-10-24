"""Qualifier fixture roots must die by refcount, without explicit collection."""
import ast
import gc
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest
import weakref

ROOT=Path(__file__).resolve().parents[3]
sys.path.insert(0,str(ROOT/'scripts'))
from validate_x650_native_numerics import frozen_constructor


class FixtureObject:
    pass


REFERENCES=[]


def setup():
    values=tuple(FixtureObject() for _ in range(4))
    REFERENCES.extend(weakref.ref(value) for value in values)
    return values


SOURCE='''from test_frozen_fixture_lifecycle import setup
fixtures=[{'mode':'stuck'}, {'mode':'moving'}]
for spec in fixtures:
 c,p,s,target=setup()
 r=[.0124,-.0124]; u=[.05,-.04]
 packet=lambda t: {'send_time_s':t, 'q_ref':r.copy(), 'dq_ref':u.copy()}
 packets=[packet(-.025),packet(-.020)]
 def context(state,t):
  return {'reference_state':{'time_s':0.,'r':r.copy()},'command_history':[(-.025,u.copy())],'packet_history':packets,'bus_origin_s':0.}
 c.context_provider=context
 if spec.get('raise'): raise RuntimeError('synthetic setup exception')
 shifted=s
 a=0
'''


class FrozenFixtureLifecycleTests(unittest.TestCase):
    def setUp(self):
        REFERENCES.clear()
        self.policy=(gc.isenabled(),gc.get_threshold(),gc.get_freeze_count())
        self.tmp=tempfile.TemporaryDirectory(dir=ROOT/'tmp')
        self.addCleanup(self.tmp.cleanup)

    def tearDown(self):
        self.assertEqual((gc.isenabled(),gc.get_threshold(),gc.get_freeze_count()),self.policy)

    def builder(self,source=SOURCE):
        path=Path(self.tmp.name)/'fixture.py';path.write_text(source)
        return frozen_constructor(path,hashlib.sha256(path.read_bytes()).hexdigest())

    def test_returned_context_keeps_original_code_globals_values_and_releases(self):
        build=self.builder();spec=build()[0]
        # Execute the same setup AST without trimming, as the independent oracle.
        tree=ast.parse(SOURCE);loop=tree.body[-1]
        old={'spec':spec};exec(compile(ast.Module(body=tree.body[:-1],type_ignores=[]),'fixture','exec'),old)
        exec(compile(ast.Module(body=loop.body[:-1],type_ignores=[]),'fixture','exec'),old)
        expected=old['context'](None,0.)
        expected_code=old['context'].__code__.co_code
        old.clear();REFERENCES.clear()
        c,p,s,target=build(spec);context=c.context_provider;ns=context.__globals__
        self.assertEqual(context.__code__.co_code,expected_code)
        self.assertEqual(context(None,0.),expected)
        self.assertIs(context(None,0.)['packet_history'],ns['packets'])
        self.assertTrue({'r','u','packets'}<=set(ns))
        self.assertFalse({'c','p','s','target','shifted','context','packet'}&set(ns))
        del context,ns,c,p,s,target
        self.assertTrue(all(reference() is None for reference in REFERENCES))

    def test_fixture_list_is_unchanged(self):
        self.assertEqual(self.builder()(),[{'mode':'stuck'},{'mode':'moving'}])

    def test_setup_exception_releases_partial_objects(self):
        with self.assertRaisesRegex(RuntimeError,'synthetic setup exception'):
            self.builder()({'raise':True})
        self.assertTrue(all(reference() is None for reference in REFERENCES))

    def test_removed_global_dependency_fails_closed_and_releases(self):
        source=SOURCE.replace("return {'reference_state'", "return {'controller':c,'reference_state'")
        with self.assertRaisesRegex(ValueError,'depends on constructor roots'):
            self.builder(source)({})
        self.assertTrue(all(reference() is None for reference in REFERENCES))

    def test_dynamic_globals_dependency_fails_closed_and_releases(self):
        source=SOURCE.replace("return {'reference_state'", "return {'mapping':globals(),'reference_state'")
        with self.assertRaisesRegex(ValueError,'dynamic globals'):
            self.builder(source)({})
        self.assertTrue(all(reference() is None for reference in REFERENCES))

    def test_frozen_script_hash_is_still_required(self):
        path=Path(self.tmp.name)/'fixture.py';path.write_text(SOURCE)
        with self.assertRaisesRegex(ValueError,'hash differs'):frozen_constructor(path,'0'*64)


if __name__=='__main__':unittest.main()
