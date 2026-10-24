#!/usr/bin/env python3
"""Present a certified full C++ fast run using original native physics poses."""
from pathlib import Path
import argparse
import json
import sys

import render_x650_isolated_validation as visual


def load_recording(report_path, expected_sha256=None):
    report_path = Path(report_path).resolve(strict=True)
    identity = visual.identity(report_path)
    if expected_sha256 is not None:
        visual.require(identity['sha256'] == expected_sha256, 'Explicit certificate SHA differs')
    report = visual.read(report_path)
    visual.require(report.get('kind') == 'x650_cpp_fast_stage_certificate_v1'
                   and report.get('stage') == 'fast'
                   and report.get('physical_acceptance') is True,
                   'Complete C++ fast physical certificate required')
    verified = visual.accepted.verified_data(report_path, certificate_kinds={
        'fast': 'x650_cpp_fast_stage_certificate_v1'}, stage_durations={'fast': 24.})
    visual.require(verified['inputs']['report']['sha256'] == identity['sha256'],
                   'Certificate changed while loading')
    run = Path(report['source_run_directory']).resolve(strict=True)
    inputs = dict(verified['inputs'])
    for name, filename in [('metadata', 'controller_metadata.json'),
                           ('native_manifest', 'native_trace_manifest.json')]:
        inputs[name] = visual.accepted.checked_input(report, run / filename)
    task, metadata, manifest = [visual.read(inputs[name]['path'])
                                for name in ('task', 'metadata', 'native_manifest')]
    from simulations.x650_cpp_fast_v1.task import trajectory
    expected = trajectory()
    visual.require(task.get('duration_s') == expected['duration_s']
                   and task.get('trajectory_parameters') == {k:v for k,v in expected.items()
                       if k not in {'kind','duration_s'}}, 'Frozen fast reference differs')
    visual.require(metadata.get('complete') is True and metadata.get('task') == task
                   and metadata.get('stage') == 'fast' and metadata.get('controller') == 'mpc',
                   'Recorded C++ fast task differs')
    visual.require(manifest.get('complete') is True
                   and manifest.get('sha256') == inputs['native_trace']['sha256']
                   and manifest.get('source_urdf_sha256') == metadata['source_urdf_sha256']
                   and manifest.get('model_sha256') == metadata['model_sha256'],
                   'Native physical recording/model binding differs')
    source, robot, geometry_report, geometry_inputs = visual.geometry(task, metadata)
    inputs.update(geometry_inputs)
    data = visual.native_states(verified)
    fk = visual.replay.audit_forward_kinematics(data, robot, geometry_report,
                                               tolerance_m=visual.FK_TOLERANCE_M)
    fk['pose_joint_timing'] = 'same native physics samples matched to certified tip timestamps'
    inputs.update(renderer=visual.identity(__file__), visual_helper=visual.identity(visual.__file__),
                  plot_helper=visual.identity(visual.accepted.__file__),
                  arm_renderer=visual.identity(visual.replay.__file__),
                  reference_implementation=visual.identity(visual.accepted.shared.__file__))
    for module in ('prepare_x650_arm', 'render_urdf_geometry', 'render_x650_hover'):
        inputs['dependency:' + module] = visual.identity(sys.modules[module].__file__)
    visual.unchanged(inputs)
    return dict(verified=verified, report=report, data=data, task=task, metadata=metadata,
                source=source, robot=robot, geometry_report=geometry_report, fk=fk, inputs=inputs,
                display_title='FAST FIGURE-EIGHT | FULL C++ COORDINATED MPC',
                video_filename='X650_fast_CPP_MPC.mp4',
                visualization_kind='x650_cpp_fast_recorded_visualization_v1', expected_duration_s=24.)


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report', type=Path, required=True)
    p.add_argument('--report-sha256')
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--fps', type=int, choices=[visual.FPS], default=visual.FPS)
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--audit-only', action='store_true')
    mode.add_argument('--preview-only', action='store_true')
    mode.add_argument('--plots-only', action='store_true')
    p.add_argument('--preview-time', type=float, default=10.)
    p.add_argument('--ffmpeg')
    p.add_argument('--quality', choices=('original', 'software_fast_v1'), default='original',
                   help='Presentation only: software_fast_v1 keeps all meshes but disables shadows, reflections and MSAA')
    args = p.parse_args()
    if args.plots_only:
        bundle = load_recording(args.report, args.report_sha256)
        result = visual.accepted.render(bundle['verified'], args.output)
        visual.unchanged(bundle['inputs'])
        print(json.dumps(result, indent=2))
        return 0
    def selected_recording(report, digest):
        bundle = load_recording(report, digest)
        if args.quality == 'software_fast_v1':
            bundle['render_quality'] = {'profile': 'software_fast_v1', 'shadows': False,
                                        'reflections': False, 'offscreen_samples': 0,
                                        'mesh_decimation': False}
        return bundle
    result = visual.render(args, loader=selected_recording)
    print(json.dumps({k: result.get(k) for k in ('complete', 'video', 'decoded_video', 'primary_metrics')}, indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
