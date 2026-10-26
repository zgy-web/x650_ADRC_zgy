#!/usr/bin/env python3
"""Present the complete multirate wrench recording without granting acceptance.

One run, two camera views, 24 seconds at 1x. Reuses original URDF visual meshes
and same-step measured body/joint poses; no physics integration or time fitting.
"""
from __future__ import annotations

import argparse
import csv
import importlib.metadata
import json
import math
import os
from pathlib import Path
import sys

import numpy as np

import plot_x650_recovery_disturbance as recorded
import render_x650_isolated_validation as visual

require, identity, read, save = visual.require, visual.identity, visual.read, visual.save
RED, GREEN = '#ff9691', '#80d6ac'


def check_report(report, metrics):
    require(report.get('kind') == 'x650_cpp_multirate_wrench_followup_report_v1'
            and report.get('timing_profile') == 'mpc25_dispatch50_v1'
            and report.get('complete') is True
            and report.get('integrity_and_execution_passed') is True,
            'A complete independently audited multirate wrench recording is required')
    expected = report['endpoint']
    require(expected['sample_count'] == metrics['sample_count'] and all(
        np.allclose(expected[k], metrics[k], rtol=1e-10, atol=1e-12)
        for k in ('position_rmse_m', 'position_max_m', 'axis_rmse_m', 'duration_s')),
        'Video metrics differ from the exact complete audit, including failed performance')
    require(report.get('passed') is False and report.get('physical_acceptance') is False
            and report.get('recovery_performance_passed') is True
            and metrics['position_max_m'] > .1,
            'This presentation is specific to the completed run with failed 100 mm peak criterion')


def load_recording(launch, audit, expected_sha):
    require(identity(audit)['sha256'] == expected_sha, 'Explicit audit SHA differs')
    original = recorded.load(Path(launch).resolve(), Path(audit).resolve())
    require(original['inputs']['audit']['sha256'] == expected_sha, 'Audit changed while loading')
    require(original['full'] and original['cutoff'] == 24., 'Full original 24 second window required')
    check_report(original['audit'], original['metrics'])
    metadata = read(original['inputs']['metadata']['path'])
    native_manifest = read(original['inputs']['native_manifest']['path'])
    require(native_manifest.get('complete') is True
            and native_manifest['source_urdf_sha256'] == metadata['source_urdf_sha256']
            and native_manifest['model_sha256'] == metadata['model_sha256'],
            'Executed model and native recording identities differ')
    start = read(original['inputs']['live_start']['path'])
    reference = recorded.shared.FigureEight(np.asarray(start['target_position_enu']),
                                            **original['task']['trajectory_parameters'])
    verified = dict(values=np.column_stack((original['native'][:, 0], original['t'],
                                            original['native'][:, 1:4])),
                    desired=original['desired'], norm=original['norm'], duration=24.,
                    reference=reference, inputs={'native_trace': original['inputs']['native_trace']})
    data = visual.native_states(verified)
    source, robot, geometry_report, geometry_inputs = visual.geometry(original['task'], metadata)
    fk = visual.replay.audit_forward_kinematics(data, robot, geometry_report,
                                               tolerance_m=visual.FK_TOLERANCE_M)
    inputs = {**original['inputs'], **geometry_inputs}
    wrench_path = original['run']/'physical_disturbance/applied_wrench.csv'
    inputs['applied_wrench'] = identity(wrench_path)
    require(original['audit']['artifacts'].get(str(wrench_path)) == inputs['applied_wrench']['sha256'],
            'Applied wrench log is not bound by the original audit')
    with wrench_path.open(newline='', encoding='utf-8') as stream:
        rows = [row for row in csv.DictReader(stream)
                if row['in_task'] == '1' and -1e-9 <= float(row['t_s']) <= 24.+1e-9]
    require(rows and all(row['valid_enqueued'] == '1' for row in rows), 'Invalid applied wrench recording')
    wrench = np.array([[float(row[k]) for k in
                       ('t_s', 'fx_enqueued_N', 'ty_com_enqueued_Nm')] for row in rows])
    require(np.isfinite(wrench).all() and np.all(np.diff(wrench[:, 0]) > 0)
            and np.max(np.diff(wrench[:, 0])) <= .004+1e-8,
            'Invalid or gapped physical wrench timeline')
    for name, module in [('renderer', sys.modules[__name__]), ('recorded_loader', recorded),
                         ('visual_helper', visual), ('arm_renderer', visual.replay),
                         ('reference_implementation', recorded.shared)]:
        inputs[name] = identity(module.__file__)
    for name in ('prepare_x650_arm', 'render_urdf_geometry', 'render_x650_hover',
                 'plot_x650_cpp_robustness', 'plot_x650_isolated_validation'):
        inputs['dependency:'+name] = identity(sys.modules[name].__file__)
    visual.unchanged(inputs)
    return dict(original=original, report=original['audit'], verified=verified, data=data,
                source=source, robot=robot, geometry_report=geometry_report, fk=fk,
                inputs=inputs, wrench=wrench, task=original['task'],
                video_filename='X650_MPC25Hz_24s_disturbance.mp4',
                camera_distances=(1.75, 1.10),
                render_quality={'profile': 'software_fast_v1', 'shadows': False,
                                'reflections': False, 'offscreen_samples': 0, 'mesh_decimation': False})


def wrench_at(values, now):
    index = int(np.searchsorted(values[:, 0], now, side='right'))-1
    return (0., 0.) if index < 0 else tuple(float(x) for x in values[index, 1:3])


def draw_overlay(ImageDraw, canvas, bundle, now, quat, fonts):
    draw = ImageDraw.Draw(canvas)
    original = bundle['original']
    t, errors = original['t'], original['norm']*1000
    draw.text((26, 12), '24 s DISTURBANCE TEST | C++ COORDINATED MPC', fill=visual.FG, font=fonts[30])
    draw.text((26, 55), 'MPC nominal 25 Hz | Command output 50 Hz | SAME RUN, TWO VIEWS | 1x replay',
              fill=visual.MUTED, font=fonts[22])
    peak = original['metrics']['position_max_m']*1000
    draw.text((26, 91), f'Execution COMPLETE | Peak {peak:.2f} mm > 100 mm: FAILED | Recovery within 10 mm for 1 s: 3/3',
              fill=RED, font=fonts[20])
    draw.text((26, 126), 'Fixed world view | Yellow: target; blue: actual past tip', fill=visual.FG, font=fonts[20])
    draw.text((826, 126), 'Arm detail | Same recorded body and joint states', fill=visual.FG, font=fonts[20])
    force, torque = wrench_at(bundle['wrench'], now)
    active = abs(force) > 1e-8 or abs(torque) > 1e-8
    if active:
        event = f'EXTERNAL INPUT ACTIVE | World X force {force:+.2f} N | World Y free torque {torque:+.3f} N m'
    elif now < 7.:
        event = 'NO EXTERNAL INPUT | P1: +1.5 N at 7 s | P2: -1.5 N at 13 s | P3: +0.04 N m at 20.5 s'
    else:
        event = 'EXTERNAL INPUT OFF | Recorded response and recovery | No control interruption in this run'
    draw.rectangle((18, 754, 1582, 796), fill='#543c20' if active else '#193044')
    draw.text((30, 761), event, fill=visual.TARGET if active else visual.FG, font=fonts[20])
    w, x, y, z = quat
    roll = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
    pitch = math.degrees(math.asin(float(np.clip(2*(w*y-z*x), -1, 1))))
    draw.text((26, 807), f'EE error {np.interp(now,t,errors):6.2f} mm | Full 24 s RMS {original["metrics"]["position_rmse_m"]*1000:.2f} mm',
              fill=visual.ACTUAL, font=fonts[22])
    draw.text((826, 807), f'Actual body roll {roll:+.2f} deg | pitch {pitch:+.2f} deg', fill=visual.MUTED, font=fonts[22])
    draw.text((26, 850), 'SAME-TIME 3D END-EFFECTOR ERROR [mm]', fill=visual.FG, font=fonts[20])
    draw.text((945, 850), 'Red: 100 mm peak limit | Green: 10 mm', fill=visual.MUTED, font=fonts[20])
    left, right, top, bottom, ymax = 95, 1545, 895, 1040, 200.
    xx = lambda value: left+(right-left)*value/24.
    yy = lambda value: bottom-(bottom-top)*value/ymax
    for i, event in enumerate(original['task']['robustness']['schedule'], 1):
        draw.rectangle((xx(event['start_s']), top, xx(event['end_s']), bottom), fill='#362f46')
        draw.text((xx(event['start_s'])-3, top-22), 'P'+str(i), fill=visual.TARGET, font=fonts[18])
    for value in (0, 50, 100, 150, 200):
        draw.line((left, yy(value), right, yy(value)), fill=RED if value == 100 else '#304154')
        draw.text((30, yy(value)-10), str(value), fill=visual.MUTED, font=fonts[18])
    draw.line((left, yy(10), right, yy(10)), fill=GREEN)
    ids = visual.past_indices(t, now, 6001)
    points = [(xx(t[j]), yy(errors[j])) for j in ids]
    if len(points) > 1:
        draw.line(points, fill=visual.ACTUAL, width=3)
    draw.line((xx(now), top, xx(now), bottom), fill='#b3c2d2', width=2)
    for value in range(0, 25, 4):
        draw.text((xx(value)-8, bottom+7), str(value), fill=visual.MUTED, font=fonts[18])
    ref = bundle['verified']['reference']
    phase = 'INITIAL HOLD' if now < ref.start_s else 'RETURN HOLD' if now >= ref.end_s else 'FIGURE-EIGHT TRACKING'
    draw.text((26, 1080), f'TASK TIME {now:6.2f} / 24.00 s | {phase} | 8 s/cycle | X +/-25 cm, Z +/-10 cm',
              fill=visual.MUTED, font=fonts[20])
    draw.text((26, 1114), 'Recorded-state visualization; no physics rerun. Input labels: physics-enqueued command, not force sensor.',
              fill=visual.MUTED, font=fonts[18])


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--launch', type=Path, required=True)
    parser.add_argument('--audit', type=Path, required=True)
    parser.add_argument('--audit-sha256', required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ffmpeg')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--audit-only', action='store_true')
    mode.add_argument('--preview-only', action='store_true')
    parser.add_argument('--preview-time', type=float, default=9.212)
    args = parser.parse_args()
    bundle = load_recording(args.launch, args.audit, args.audit_sha256)
    output = args.output.resolve()
    require(not output.exists(), 'A new output directory is required')
    for protected in (args.launch, args.audit.parent, bundle['original']['run'], bundle['source'].parent):
        require(not output.is_relative_to(Path(protected).resolve()), 'Output overlaps source evidence/model directory')
    frames = visual.frame_times(24., preview_time=args.preview_time if args.preview_only else None,
                                expected_duration=24.)
    ffmpeg = None if args.audit_only or args.preview_only else visual.find_ffmpeg(args.ffmpeg)
    if ffmpeg:
        bundle['inputs']['ffmpeg'] = identity(ffmpeg)
    record = dict(kind='x650_multirate_wrench_recorded_visualization_v1', presentation_complete=False,
        source_execution_complete=True, source_physical_acceptance=False, source_peak_criterion_passed=False,
        source_recovery_criterion_passed=True, eligible_as_stage_gate=False, hardware_qualified=False,
        new_physics_run=False, native_gazebo_screen_capture=False, same_run_two_views=True,
        original_run=str(bundle['original']['run']), full_recorded_task_window_s=[0.,24.],
        metrics=bundle['original']['metrics'], native_sample_count=len(bundle['original']['t']),
        fps=visual.FPS, width=visual.WIDTH, height=visual.HEIGHT, frames=len(frames),
        video_time_scale=1., video_duration_s=None if args.audit_only or args.preview_only else 24.,
        timeline_policy='24 s at 1x; 576 frames at source times 0..23.958333; exact 24 s final state separately saved',
        pose_interpolation='same-step measured position/joints linear; quaternion SLERP; no physics integration',
        rotor_angles='not logged; fixed original URDF zero angles', original_mesh_geometry=True,
        no_mesh_decimation=True, no_time_or_origin_fitting=True, metric_subsampling=False,
        force_labels='latest original physics-enqueued world-X force and world-Y free couple; not force sensor',
        fk_audit=bundle['fk'], input_hashes=bundle['inputs'],
        audit_only=args.audit_only, preview_only=args.preview_only, python=sys.executable,
        mujoco_gl=os.environ.get('MUJOCO_GL'),
        package_versions={name:importlib.metadata.version(name) for name in ('numpy','mujoco','Pillow')})
    output.mkdir(parents=True, exist_ok=False)
    save(output/'render_status.json', record)
    try:
        with (output/'display_frame_clock.csv').open('x', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(['frame','video_pts_s','source_task_s'])
            writer.writerows((i,i/visual.FPS,float(t)) for i,t in enumerate(frames))
        if not args.audit_only:
            record.update(visual.render_frames(bundle, output, frames, args.preview_only, ffmpeg, overlay=draw_overlay))
        visual.unchanged(bundle['inputs'])
        record.update(presentation_complete=True, video_produced=not(args.audit_only or args.preview_only))
        save(output/'replay_manifest.json', record)
        save(output/'render_status.json', record)
        save(output/'delivery_manifest.json', dict(kind='x650_multirate_video_delivery_v1',
            files={p.name:identity(p) for p in output.iterdir() if p.is_file()}, source_physical_acceptance=False))
        print(json.dumps({k:record.get(k) for k in ('presentation_complete','video','decoded_video','metrics')},indent=2))
    except BaseException as error:
        record.update(error=repr(error)); save(output/'render_status.json', record); raise
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
