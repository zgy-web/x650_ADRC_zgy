import importlib.util
from pathlib import Path
import tempfile
import unittest

BASE = Path(__file__).resolve().parents[2]
SPEC = importlib.util.spec_from_file_location('robustness_verifier', BASE/'scripts/verify_x650_robustness_model.py')
v = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(v)
FIXTURE = importlib.util.spec_from_file_location('robustness_model_fixture', Path(__file__).with_name('test_x650_robustness_models.py'))
f = importlib.util.module_from_spec(FIXTURE)
FIXTURE.loader.exec_module(f)


class RobustnessVerifier(unittest.TestCase):
    def report(self, scale=.85):
        original = f.fixture()
        actual, ledger = f.module.scale_tree(original, scale=scale, model_name='amsim_urdf_new')
        report = {'scale':scale, 'mass_bias_fraction':scale-1., 'per_link':ledger,
                  'nominal_total_mass_kg':sum(x['nominal']['mass'] for x in ledger),
                  'total_mass_kg':sum(x['actual']['mass'] for x in ledger)}
        return report, original, actual

    def test_ledger_recomputed_from_sdf(self):
        for scale in (.85,1.15):
            report,original,actual = self.report(scale)
            nominal,physical = v.verify_ledger(report,original,actual)
            self.assertAlmostEqual(physical,nominal*scale)

    def test_false_ledger_or_aggregate_or_bias_rejected(self):
        for field in ('ledger','aggregate','bias','duplicate'):
            report,original,actual = self.report()
            if field=='ledger':report['per_link'][0]['actual']['mass']+=.001
            if field=='aggregate':report['total_mass_kg']+=.001
            if field=='bias':report['mass_bias_fraction']=.15
            if field=='duplicate':report['per_link'][-1]=report['per_link'][0]
            with self.assertRaises(ValueError):v.verify_ledger(report,original,actual)

    def test_actual_actuator_or_pose_tamper_rejected(self):
        for path in ('model/plugin/bus_period_s','model/link/inertial/pose'):
            report,original,actual=self.report()
            actual.find(path).text='0.01'
            with self.assertRaises(ValueError):v.verify_ledger(report,original,actual)

    def test_bound_identity_rejects_changed_bytes(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'nominal.json';path.write_text('nominal')
            bound=v.identity(path)
            self.assertEqual(v.bound_identity(bound,'nominal'),path.resolve())
            path.write_text('changed')
            with self.assertRaises(ValueError):v.bound_identity(bound,'nominal')


if __name__=='__main__':
    unittest.main()
