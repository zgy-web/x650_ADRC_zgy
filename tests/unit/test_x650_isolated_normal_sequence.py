"""Driver-only sequencing tests with a fake launcher; never start ROS or flight."""
import importlib.util
import json
from pathlib import Path
import shutil
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import uuid

ROOT=Path(__file__).resolve().parents[2]
HERE=ROOT/'tmp'
spec=importlib.util.spec_from_file_location('run_x650_isolated_normal_sequence',ROOT/'scripts/run_x650_isolated_normal_sequence.py')
driver=importlib.util.module_from_spec(spec);spec.loader.exec_module(driver)


class ProgressTests(unittest.TestCase):
    def test_only_explicit_integer_simulation_revision_at_least_nine_is_supported(self):
        for value in (9,10,100):
            driver.require_protocol({'execution_revision':value,'simulation_only':True})
        for value in (8,0,-1,True,9.,'9',None):
            with self.assertRaises(ValueError):
                driver.require_protocol({'execution_revision':value,'simulation_only':True})
        with self.assertRaises(ValueError):
            driver.require_protocol({'execution_revision':9,'simulation_only':False})

    def directory(self):
        root=(HERE/('driver_fixture_'+uuid.uuid4().hex)).resolve()
        root.relative_to(HERE);root.mkdir()
        self.addCleanup(lambda:shutil.rmtree(root))
        return root

    def test_wrong_or_incomplete_normal_report_never_opens_plot_gate(self):
        for report in ({'kind':'x650_isolated_normal_stage_certificate_v1','stage':'normal','passed':True},
                       {'kind':'x650_isolated_live_stage_certificate_v1','stage':'tip_hold','passed':True}):
            with self.assertRaises(ValueError):
                driver.require_normal_certificate(report,Path('/launch'),Path('/run'))

    def test_launch_identity_must_be_unique_not_latest(self):
        root=self.directory();log=root/'log.txt'
        log.write_text('no marker\n')
        with self.assertRaisesRegex(ValueError,'exactly one'):driver.exact_launch(log,root)
        log.write_text('ISOLATED_LAUNCH /one\nISOLATED_LAUNCH /two\n')
        with self.assertRaisesRegex(ValueError,'exactly one'):driver.exact_launch(log,root)

    def test_failed_normal_runs_audit_then_stops_without_plots_or_retry(self):
        root=self.directory()
        def write(relative,value):
            path=root/relative;path.parent.mkdir(parents=True,exist_ok=True)
            path.write_text(json.dumps(value));return path
        baseline=write('inputs/hold_task.json',{'stage':'tip_hold','numerical_backend':{'library_sha256':'f'*64}})
        report=write('inputs/hold_report.json',{'fixture_only':True})
        proof=write('inputs/transport.json',{'fixture_only':True})
        protocol=write('configs/experiments/x650_isolated_live_v1.json',{
            'execution_revision':9,'simulation_only':True,'frozen_hold_task_sha256':driver.sha(baseline),
            'frozen_numerical_library_sha256':'f'*64})
        for name in ('simulations/x650_isolated_figure8_v1/launch.py','scripts/audit_x650_isolated_figure8.py',
                     'scripts/audit_x650_isolated_live.py','scripts/plot_x650_isolated_validation.py'):
            write(name,{'fixture_only':True})
        output=root/'artifacts/driver'
        argv=['progress','--repo',str(root),'--hold-task',str(baseline),'--hold-task-sha256',driver.sha(baseline),
              '--hold-report',str(report),'--hold-report-sha256',driver.sha(report),
              '--transport-proof',str(proof),'--transport-proof-sha256',driver.sha(proof),
              '--protocol-sha256',driver.sha(protocol),'--output',str(output),'--execute']
        cfg={'stage':'normal','figure8_protocol':{'fast_admitted':False}}
        fake=NS(ROOT=root,build_task_config=lambda *a:cfg,validate_task_config=lambda c:c,
                validate_live_hold=lambda c:{'verified':True})
        package=NS(task=fake)
        calls=[]
        def popen(command,**kwargs):
            calls.append(command)
            if '--dry-run' in command:return NS(pid=123,wait=lambda:0)
            if command[1].endswith('/launch.py') or command[1].endswith('\\launch.py'):
                launch=root/'runs/x650_isolated_figure8_launch_fixture';launch.mkdir(parents=True)
                run=root/'runs/x650_isolated_figure8_v1_fixture';run.mkdir()
                task_path=Path(command[command.index('--task')+1])
                (launch/'task.json').write_bytes(task_path.read_bytes())
                (launch/'launch.json').write_text(json.dumps({'execution_scenario':'x650_isolated_figure8_v1',
                    'task_sha256':driver.sha(task_path),'task':driver.read(task_path),
                    'run_directories':[str(run)],'complete':False}))
                kwargs['stdout'].write('ISOLATED_LAUNCH '+str(launch)+'\n');kwargs['stdout'].flush()
                return NS(pid=124,wait=lambda:1)
            if 'audit_x650_isolated_figure8.py' in command[1]:
                audit=Path(command[command.index('--output')+1]);audit.mkdir()
                (audit/'report.json').write_text(json.dumps({'passed':False,'complete':False,'error':'fixture failure'}))
                return NS(pid=125,wait=lambda:1)
            self.fail('Failure progression unexpectedly launched '+repr(command))
        modules={'simulations.x650_isolated_figure8_v1':package,
                 'simulations.x650_isolated_figure8_v1.task':fake}
        with patch.object(sys,'argv',argv),patch.dict(sys.modules,modules),patch.object(driver.subprocess,'Popen',popen):
            self.assertEqual(driver.main(),1)
        state=driver.read(output/'status.json')
        self.assertFalse(state['normal_passed']);self.assertFalse(state['fast_started'])
        self.assertEqual(state['phase'],'stopped_no_progression')
        self.assertEqual(len(calls),3)
        self.assertFalse((output/'normal_plots').exists())


if __name__=='__main__':unittest.main()
