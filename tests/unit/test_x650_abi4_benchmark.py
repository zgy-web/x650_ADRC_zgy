"""ABI 4 timing/evidence gates; no SSH, simulation or actuator access."""
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'scripts'))
sys.path.insert(0, str(Path(__file__).parent))
import prepare_x650_abi4_benchmark as preparation
import compare_x650_paired_core_benchmark as paired
import test_x650_paired_core_benchmark as fixtures
from prepare_x650_recorded_core_benchmark import frozen_target
from prepare_x650_orin_benchmark import package_sources

save = fixtures.save


class Abi4ProfileTests(unittest.TestCase):
    def test_custom_source_package_rejects_escape_duplicate_and_unbound_frozen_members(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for names in (('../outside.cpp',), ('C:/outside.cpp',), ('/outside.cpp',), ('same.cpp', 'same.cpp')):
                with self.subTest(names=names), self.assertRaises(ValueError):
                    package_sources(root, root/'unsafe.tar.gz', source_paths=names)
            with self.assertRaisesRegex(ValueError, 'frozen-source'):
                package_sources(root, root/'unsafe.tar.gz', frozen_archive=root/'unread.zip', source_paths=('extra.cpp',))

    def fixture(self, directory):
        root = fixtures.PairedCoreTests().fixture(directory)
        path = root/'input.json'
        bundle = paired.load_json(path)
        bundle['config'].update(max_dt=.1, command_hold_s=.16, nodes=15, node_dt=.04, bus_period=.005)
        bundle['provenance'].update(expected_core_abi=4, expected_optimization_abi=2)
        save(path, bundle)
        input_id = paired.identity(path)
        for side in ('server', 'nx'):
            path = root/side/'native_stdout.json'
            result = paired.load_json(path)
            result.update(core_abi=4, optimization_abi=2, provenance=bundle['provenance'], input_sha256=input_id['sha256'])
            result['scope'].update(control_update_target_ms=40, continuous_dispatch_period_ms=20,
                configured_max_model_dt_s=.1, configured_command_hold_s=.16,
                twenty_ms_sample_flag_is_dispatch_reference_only=True)
            for row in [result['cold_first_call'], *result['warmup_samples'], *result['samples']]:
                row['diagnostics'].update(hold_required=True, hold_checked=True, hold_passed=True,
                    hold_duration_s=.16, hold_substeps=160, interval_rows=0, interval_repair_count=0,
                    interval_added_points=0)
            save(path, result)
            tpath = root/side/'transport_report.json'
            transport = paired.load_json(tpath)
            transport.update(input=input_id, stdout=paired.identity(path))
            save(tpath, transport)
        return root

    def compare(self, root, profile='abi4_mpc25_dispatch50'):
        return paired.compare(root/'input.json', root/'server', root/'nx', root/'preparation.json',
            root/'server_build.json', root/'nx_build.json', timing_profile=profile)

    def test_profile_passes_but_legacy_default_rejects_extended_dt(self):
        with tempfile.TemporaryDirectory() as directory:
            root = self.fixture(directory)
            report = self.compare(root)
            self.assertTrue(report['pass'])
            self.assertEqual(report['measured']['deadlines']['nx']['over_40ms'], 0)
            with self.assertRaisesRegex(ValueError, 'dt gate'):
                self.compare(root, 'legacy20')

    def test_wrong_abi_wrong_cadence_and_failed_hold_are_rejected(self):
        for mutation in (
            lambda result: result.update(core_abi=3),
            lambda result: result['scope'].update(control_update_target_ms=20),
            lambda result: result['samples'][0]['diagnostics'].update(hold_passed=False),
        ):
            with self.subTest(mutation=mutation), tempfile.TemporaryDirectory() as directory:
                root = self.fixture(directory)
                fixtures.PairedCoreTests().change_result(root, mutation)
                with self.assertRaises(ValueError):
                    self.compare(root)


class ActualArchiveTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        receipt = ROOT/'docs/validation/cpp_multirate_20261026/multirate_evidence_receipt.json'
        if not receipt.exists():
            raise unittest.SkipTest('Actual ABI 4 evidence not available')
        evidence = preparation.Evidence(receipt)
        try:
            cls.cfg, cls.cases, cls.provenance = preparation.recover(evidence, 'wrench')
        finally:
            evidence.close()

    def test_failed_physics_is_preserved_and_all_recovery_calls_included(self):
        self.assertEqual(len(self.cases), 570)
        self.assertFalse(self.provenance['physical_audit']['physical_acceptance'])
        self.assertTrue(self.provenance['physical_audit']['complete'])
        self.assertEqual([r['original_sequence'] for r in self.provenance['cases']
            if r['historical_subproblems'] > 1], [351, 354, 358, 364])
        self.assertEqual([c['memory']['step_count'] for c in self.cases], list(range(570)))

    def test_recovered_context_and_acceleration_oracle_are_consecutive(self):
        for previous, case in zip(self.cases, self.cases[1:]):
            self.assertEqual(case['memory']['previous_command'], previous['oracle']['command'])
            self.assertAlmostEqual(case['input']['t']-previous['input']['t'], case['input']['dt'], places=9)
            self.assertEqual(paired.derived_acceleration_check(case['oracle'], case), [])
        self.assertEqual(self.cfg['command_hold_s'], .16)

    def test_hold_preview_is_stationary_and_does_not_alias_center(self):
        center = [1., 2., 3.]
        target = frozen_target(100., center, 'hold')
        self.assertEqual(target, dict(position=center, velocity=[0.]*3, acceleration=[0.]*3))
        target['position'][0] = 99.
        self.assertEqual(center, [1., 2., 3.])


if __name__ == '__main__':
    unittest.main()
