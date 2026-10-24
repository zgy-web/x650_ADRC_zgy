"""Synthetic original-record presentation checks; no ROS, solver or physics."""
from contextlib import contextmanager
import csv
import importlib.util
import json
from pathlib import Path
import unittest
from unittest.mock import patch
import uuid

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('robust_plots', ROOT / 'scripts/plot_x650_cpp_robustness.py')
plot = importlib.util.module_from_spec(spec); spec.loader.exec_module(plot)


@contextmanager
def space():
    path = ROOT / ('.tmp_robust_plots_' + uuid.uuid4().hex); path.mkdir()
    try: yield path
    finally:
        for child in sorted(path.rglob('*'), key=lambda p: len(p.parts), reverse=True):
            if child.is_file(): child.unlink()
            elif child.is_dir(): child.rmdir()
        path.rmdir()


def put(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value), encoding='utf-8')


def fixture(out, case_id='mass085_fast'):
    run = out / case_id; run.mkdir()
    scale = {'mass085_fast': .85, 'mass115_fast': 1.15, 'wrench_fast': 1.}[case_id]
    schedule = [dict(start_s=7., end_s=7.6, force_world_n=[1.5,0.,0.], torque_world_nm=[0.,0.,0.]),
        dict(start_s=13., end_s=13.6, force_world_n=[-1.5,0.,0.], torque_world_nm=[0.,0.,0.]),
        dict(start_s=20.5, end_s=20.9, force_world_n=[0.,0.,0.], torque_world_nm=[0.,.04,0.])]
    robust = dict(case_id=case_id, kind='external_wrench' if case_id=='wrench_fast' else 'uniform_rigid_mismatch',
        mass_inertia_scale=scale, recovery_threshold_m=.01, recovery_hold_s=1.,
        schedule=schedule if case_id=='wrench_fast' else [])
    params = dict(amplitude_x=.25, amplitude_z=.1, period_s=8., cycles=2, start_s=2., ramp_s=2.)
    put(run / 'controller_task.json', dict(stage='fast', duration_s=24., trajectory_parameters=params, robustness=robust))
    center = np.array([.4,.04,1.2])
    put(run / 'isolated_live/live_start.json', dict(origin_s=100., target_position_enu=center.tolist()))
    reference = plot.original.shared.FigureEight(center, **params)
    with (run / 'native_trace.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.writer(stream); writer.writerow(['sim_s','valid_tip','tip_x','tip_y','tip_z'])
        for at in np.linspace(0.,24.,3001):
            sim = 100.+float(at); t = sim-100.
            actual = reference.sample(t)[0] + [.002,0.,.001]
            writer.writerow([sim,1,*actual])
    endpoint = plot.original.shared.native_tip_metrics(run, reference.sample, 100., (0.,24.))
    performance = dict(case_id=case_id, same_time=True, no_phase_alignment=True, delay_injected=False,
        performance_passed=True if case_id=='wrench_fast' else None)
    if case_id=='wrench_fast':
        performance['events'] = [dict(start_s=e['start_s'], end_s=e['end_s'], threshold_m=.01, hold_s=1.,
            recovered=True, recovery_after_end_s=0., confirmed_at_task_s=e['end_s']+1.) for e in schedule]
        (run / 'physical_disturbance').mkdir()
        with (run / 'physical_disturbance/applied_wrench.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream); writer.writerow(['sim_s','t_s','fx_enqueued_N','ty_com_enqueued_Nm'])
            for at in np.linspace(0.,24.,6001):
                sim=100.+float(at);t=sim-100.;f=tau=0.
                for e in schedule:
                    if e['start_s']<=t<e['end_s']:f+=e['force_world_n'][0];tau+=e['torque_world_nm'][1]
                writer.writerow([sim,t,f,tau])
    report = dict(kind=plot.KIND, stage='fast', passed=True, complete=True, certificate=True,
        physical_acceptance=True, full_window_ulog_verified=True, parameter_restore_verified=True,
        native_ground_verified=True, actuator_log_verified=True, source_run_directory=str(run),
        window_sim_s=[100.,124.], endpoint=endpoint, robustness=robust,
        physical_model=dict(verified=True, scale=scale, total_mass_kg=3.495*scale,
            controller_preparation_unchanged=True, controller_pid_values_unchanged=True),
        robustness_performance=performance, external_wrench=dict(verified=True, enabled=case_id=='wrench_fast'),
        artifacts={str(p):plot.sha(p) for p in run.rglob('*') if p.is_file()})
    path=out/(case_id+'_report.json');put(path,report)
    return path,report,run


class OriginalRecordChecks(unittest.TestCase):
    def test_original_native_metrics_and_untouched_nominal_scope(self):
        with space() as out:
            path, cert, run = fixture(out)
            with patch('scipy.optimize.minimize', side_effect=AssertionError('No controller during presentation')):
                value = plot.load_case(path)
            self.assertEqual(value['case_id'], 'mass085_fast')
            self.assertEqual(value['data']['metrics'], cert['endpoint'])
            self.assertEqual(value['physical']['total_mass_kg'], 3.495*.85)
            self.assertEqual(len(value['data']['norm']), 3001)
            np.testing.assert_allclose(value['data']['norm'], np.sqrt(.002**2+.001**2), atol=1e-15)
            (run / 'native_trace.csv').write_text('changed')
            with self.assertRaisesRegex(ValueError,'has changed'):
                plot.load_case(path)

    def test_wrong_kind_partial_and_model_condition_never_get_a_curve(self):
        with space() as out:
            path,cert,_ = fixture(out)
            for field,value in [('kind','x650_cpp_fast_stage_certificate_v1'),('passed',False),('complete',False)]:
                changed=dict(cert);changed[field]=value;put(path,changed)
                with self.assertRaisesRegex(ValueError,'Complete passed'):
                    plot.load_case(path)
            changed=dict(cert);changed['physical_model']=dict(cert['physical_model'],scale=1.15);put(path,changed)
            with self.assertRaisesRegex(ValueError,'Physical model'):
                plot.load_case(path)

    def test_wrench_uses_original_enqueued_trace_and_actual_recovery_markers(self):
        with space() as out:
            path,cert,run=fixture(out,'wrench_fast');value=plot.load_case(path)
            self.assertEqual(value['wrench'].shape,(6001,4))
            self.assertEqual(value['wrench'][:,2].max(),1.5)
            self.assertEqual(value['wrench'][:,3].max(),.04)
            cert['robustness_performance']['events'][0]['recovered']=False
            put(path,cert)
            with self.assertRaisesRegex(ValueError,'fabricated marker'):
                plot.load_case(path)

    def test_source_time_or_reference_metric_change_is_rejected(self):
        with space() as out:
            path,cert,_=fixture(out)
            cert['window_sim_s'][1]=123.99;put(path,cert)
            with self.assertRaisesRegex(ValueError,'window differs'):
                plot.load_case(path)


class Delivery(unittest.TestCase):
    def test_constant_error_is_not_clipped_at_auto_upper_axis_limit(self):
        self.assertGreater(plot.error_limit_mm([.002236]*3),2.236)
        self.assertEqual(plot.error_limit_mm([.002236]*3,recovery_band=True),12.)

    def test_new_per_case_actual_title_png_pdf_csv_manifest(self):
        with space() as out:
            path,_,_=fixture(out);value=plot.load_case(path)
            manifest=plot.render_case(value,out/'figures')
            self.assertIn('mass085_fast',manifest['display_title'])
            self.assertIn('actual mass 2.97075 kg',manifest['condition'])
            self.assertEqual(set(manifest['outputs']),{'endpoint_validation.png','endpoint_validation.pdf','endpoint_chart_data.csv'})
            self.assertTrue(all(Path(i['path']).stat().st_size>0 for i in manifest['outputs'].values()))
            with self.assertRaisesRegex(ValueError,'already exists'):
                plot.render_case(value,out/'figures')

    def test_failed_certificate_stays_failed_in_batch_without_substitution(self):
        with space() as out:
            paths=[]
            for case in plot.CASES:
                path,cert,_=fixture(out,case)
                if case=='mass115_fast':cert['passed']=False;put(path,cert)
                paths.append(path)
            with patch.object(plot,'render_case',return_value={}) as render, \
                    patch.object(plot,'render_wrench') as wrench, patch.object(plot,'batch_summary'):
                result=plot.run(paths,out/'batch')
            self.assertFalse(result['complete'])
            self.assertEqual(render.call_count,2);self.assertEqual(wrench.call_count,1)
            failed=next(x for x in result['cases'] if x['case_id']=='mass115_fast')
            self.assertFalse(failed['verified']);self.assertNotIn('metrics',failed)
            self.assertTrue((out/'batch/presentation_summary.json').is_file())

    def test_duplicate_cases_rejected_before_output_write(self):
        with space() as out:
            path,_,_=fixture(out)
            with self.assertRaisesRegex(ValueError,'Repeated'):
                plot.run([path,path,path],out/'batch')
            self.assertFalse((out/'batch').exists())


class FailedLabels(unittest.TestCase):
    def fixture(self,out):
        path,cert,run=fixture(out,'wrench_fast')
        task_hash=plot.sha(run/'controller_task.json')
        start=plot.read(run/'isolated_live/live_start.json')
        start.update(session_id=12,epoch=1,configuration_sha256=list(bytes.fromhex(task_hash)))
        put(run/'isolated_live/live_start.json',start)
        raw=run/'isolated_live/native_records/000379.snapshot.cdr'
        raw.parent.mkdir();raw.write_bytes(b'synthetic rejected CDR, not a physical acceptance proof')
        put(raw.with_name('000379.json'),dict(kind='cpp_full_pipeline_call_v1',sequence=379,
            error='QP infeasible',snapshot_cdr=plot.identity(raw)))
        cert.pop('robustness');cert.update(passed=False,complete=False,physical_acceptance=False,
            artifacts={str(p):plot.sha(p) for p in run.rglob('*') if p.is_file()})
        put(path,cert)
        decoded=dict(session_id=12,epoch=1,sequence=379,configuration_sha256=start['configuration_sha256'],
            control_time_s=107.544,task_time_s=7.544)
        return path,cert,run,decoded

    def test_early_failed_report_gets_bound_actual_case_reason_and_snapshot_time(self):
        with space() as out:
            path,cert,run,decoded=self.fixture(out)
            with patch.object(plot,'decode_failure_snapshot',return_value=decoded):
                value=plot.failure_context(cert,plot.identity(path))
            self.assertEqual(value['case_id'],'wrench_fast')
            self.assertEqual(value['failure_reason'],'QP infeasible')
            self.assertAlmostEqual(value['failure_task_s'],7.544)
            self.assertIn('rejected_snapshot',value['failure_proof_inputs'])
            self.assertNotIn('metrics',value)

    def test_without_decoder_never_infers_a_failure_time(self):
        with space() as out:
            path,cert,run,decoded=self.fixture(out)
            with patch.object(plot,'decode_failure_snapshot',side_effect=ImportError('No ROS')):
                value=plot.failure_context(cert,plot.identity(path))
            self.assertEqual(value['case_id'],'wrench_fast')
            self.assertEqual(value['failure_reason'],'QP infeasible')
            self.assertIsNone(value['failure_task_s'])

    def test_changed_raw_task_or_wrong_snapshot_clock_rejects_label_proof(self):
        with space() as out:
            path,cert,run,decoded=self.fixture(out)
            decoded['task_time_s']=7.6
            with patch.object(plot,'decode_failure_snapshot',return_value=decoded):
                with self.assertRaisesRegex(ValueError,'clock'):
                    plot.failure_context(cert,plot.identity(path))
            (run/'controller_task.json').write_text('{}')
            with self.assertRaisesRegex(ValueError,'has changed'):
                plot.failure_context(cert,plot.identity(path))


if __name__=='__main__':unittest.main()
