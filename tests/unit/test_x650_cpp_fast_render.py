"""Presentation contracts for full fast records; synthetic data never grants a flight gate."""
import csv
import importlib.util
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np
ROOT=Path(__file__).resolve().parents[2]
sys.path[:0]=[str(ROOT/'scripts'),str(ROOT/'tests/unit')]
import test_x650_isolated_validation_render as original
import render_x650_cpp_fast as fast
video=fast.visual

class FastPresentationTests(unittest.TestCase):
    directory=original.IsolatedVideoTests.directory
    def fixture(self):
        root,report=original.IsolatedVideoTests.fixture(self)
        run=root/'run';task=video.read(run/'controller_task.json')
        task.update(stage='fast',scenario='x650_cpp_fast_v1',duration_s=24.)
        task['trajectory_parameters']['period_s']=8.
        original.write_json(run/'controller_task.json',task)
        metadata=video.read(run/'controller_metadata.json');metadata.update(task=task,stage='fast')
        original.write_json(run/'controller_metadata.json',metadata)
        robot=video.ET.parse(root/'prepared_ts20/robot.urdf').getroot()
        geometry=video.read(root/'prepared_ts20/preparation_report.json')
        center=np.array([.4,.04,1.2])
        ref=video.accepted.shared.FigureEight(center,**task['trajectory_parameters'])
        path=run/'native_trace.csv'
        with path.open(newline='') as stream: rows=list(csv.DictReader(stream));fields=list(rows[0])
        for row in rows:
            t=.75*(float(row['sim_s'])-100.);row['sim_s']=str(100.+t)
            q=np.array([float(row['arm_joint_1_q']),float(row['arm_joint_2_q'])])
            relative=video.replay.tip_from_report(np.zeros(3),[1,0,0,0],q,geometry)
            tip=ref.sample(t)[0]+[.002,0,.00005];base=tip-relative-[0,0,.00005]
            for axis,index in zip('xyz',range(3)):
                row['base_p'+axis]=str(base[index]);row['tip_'+axis]=str(tip[index])
        with path.open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=fields);writer.writeheader();writer.writerows(rows)
        native=video.read(run/'native_trace_manifest.json');native['sha256']=video.sha(path)
        original.write_json(run/'native_trace_manifest.json',native)
        cert=video.read(report);cert.update(kind='x650_cpp_fast_stage_certificate_v1',stage='fast',physical_acceptance=True,window_sim_s=[100.,124.])
        cert['endpoint']=video.accepted.shared.native_tip_metrics(run,ref.sample,100.,(0.,24.))
        cert['artifacts']={str(p):video.sha(p) for p in run.rglob('*') if p.is_file()}
        original.write_json(report,cert)
        return root,report

    def test_fast_certificate_and_complete_native_replay_are_bound(self):
        root,report=self.fixture();bundle=fast.load_recording(report,video.sha(report))
        self.assertEqual(bundle['verified']['duration'],24.)
        self.assertEqual(bundle['verified']['stage'],'fast')
        self.assertEqual(bundle['expected_duration_s'],24.)
        self.assertAlmostEqual(bundle['verified']['metrics']['position_rmse_m'],np.hypot(.002,.00005),places=11)
        self.assertIn('FAST',bundle['display_title'])
        with self.assertRaises(ValueError):video.load_recording(report)
        with self.assertRaises(ValueError):video.accepted.verified_data(report)

    def test_duration_is_explicit_and_video_covers_exactly_24_seconds(self):
        with self.assertRaises(ValueError):video.frame_times(24.)
        frames=video.frame_times(24.,expected_duration=24.)
        self.assertEqual(len(frames),576);self.assertEqual(frames[-1],575/24)
        text='#tb 0: 1/24\n#dimensions 0: 1600x1140\n'+'\n'.join(f"0, {i}, {i}, 1, 2736000, {'a'*32}" for i in range(576))
        self.assertEqual(video.decoded_video_contract(text,576)['duration_s'],24.)
        for duration in (20.,23.99,30.,32.):
            with self.assertRaises(ValueError):video.frame_times(duration,expected_duration=24.)

    def test_audit_only_never_runs_new_physics_or_a_media_process(self):
        root,report=self.fixture();args=SimpleNamespace(report=report,report_sha256=video.sha(report),output=root/'fast_display',audit_only=True,preview_only=False,preview_time=10.,fps=24,ffmpeg=None)
        before={str(p):video.sha(p) for p in root.rglob('*') if p.is_file()}
        with patch.object(video,'render_frames',side_effect=AssertionError('No media runtime')),patch.object(video.subprocess,'Popen',side_effect=AssertionError('No subprocess')):
            result=video.render(args,loader=fast.load_recording)
        self.assertTrue(result['complete']);self.assertEqual(result['stage'],'fast')
        self.assertFalse(result['eligible_as_stage_gate']);self.assertFalse(result['new_physics_run'])
        self.assertEqual(before,{p:video.sha(p) for p in before})

if __name__=='__main__':unittest.main()
