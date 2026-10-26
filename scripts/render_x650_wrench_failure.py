#!/usr/bin/env python3
"""Replay the original failed wrench prefix; never integrate new physics.

The last native state is the rejected ControlSnapshot boundary, not NAV_LAND.
Presentation slows the first pulse to 0.25x and explicitly freezes the final
state. Both viewports show the same experiment. Successful-stage renderers and
their certificate requirements remain unchanged.
"""
from __future__ import annotations

import argparse
import csv
import importlib.metadata
import json
import math
import os
from pathlib import Path
import subprocess
import sys

os.environ.setdefault('MUJOCO_GL', 'egl')
HERE = Path(__file__).resolve().parent
for directory in (HERE.parent, HERE):
    if str(directory) not in sys.path:
        sys.path.insert(0, str(directory))
import numpy as np
import plot_x650_wrench_failure as failure
import render_x650_isolated_validation as visual

FPS, WIDTH, HEIGHT = visual.FPS, visual.WIDTH, visual.HEIGHT
require, identity, save = visual.require, visual.identity, visual.save
RED, GOLD = '#ff8d88', visual.TARGET


def load_recording(run, analysis_path, analysis_sha=None):
    if analysis_sha is not None:
        require(identity(analysis_path)['sha256'] == analysis_sha, 'Failure analysis SHA changed')
    original = failure.load_prefix(run, analysis_path)
    metadata = failure.read(original['inputs']['metadata']['path'])
    require(metadata['task'] == original['task'] and metadata['complete'] is False,
            'Metadata does not describe this failed task')
    # Always reconstruct the reference from original LiveStart, never a measured
    # endpoint or a later snapshot reference that might already be moving.
    reference = failure.shared.FigureEight(np.asarray(
        failure.read(original['inputs']['start']['path'])['target_position_enu']),
        **original['task']['trajectory_parameters'])
    verified = {'values': np.column_stack((original['native'][:, 0], original['t'],
                    original['native'][:, 1:4])), 'desired': original['desired'],
                'inputs': {'native_trace': original['inputs']['native']}}
    data = visual.native_states(verified)
    source, robot, geometry_report, geometry_inputs = visual.geometry(original['task'], metadata)
    fk = visual.replay.audit_forward_kinematics(data, robot, geometry_report,
                                               tolerance_m=visual.FK_TOLERANCE_M)
    inputs = {**original['inputs'], **geometry_inputs,
              'renderer': identity(__file__), 'visual_helper': identity(visual.__file__),
              'prefix_loader': identity(failure.__file__),
              'arm_renderer': identity(visual.replay.__file__),
              'reference_implementation': identity(failure.shared.__file__)}
    for name in ('prepare_x650_arm', 'render_urdf_geometry', 'render_x650_hover',
                 'plot_x650_cpp_robustness', 'plot_x650_isolated_validation'):
        inputs['dependency:' + name] = identity(sys.modules[name].__file__)
    visual.unchanged(inputs)
    return dict(original=original, data=data, source=source, robot=robot,
                geometry_report=geometry_report, fk=fk, inputs=inputs, reference=reference)


def frame_clock(cutoff, slow_start=7., speed=.25, hold_s=3.):
    require(math.isfinite(cutoff) and slow_start < cutoff and 0 < speed <= 1
            and hold_s > 0, 'Invalid failed-prefix display timeline')
    lead = np.arange(math.ceil(slow_start * FPS), dtype=float) / FPS
    lead = lead[lead < slow_start]
    tail = slow_start + np.arange(math.ceil((cutoff-slow_start)*FPS/speed)) * speed/FPS
    tail = tail[tail < cutoff]
    stamps = np.r_[lead, tail, np.full(round(hold_s*FPS), cutoff)]
    phases = ['REPLAY 1x']*len(lead) + ['SLOW MOTION 0.25x']*len(tail) + ['FINAL STATE PAUSED']*round(hold_s*FPS)
    require(len(stamps) == len(phases) and np.all(np.diff(stamps) >= 0)
            and stamps[0] == 0 and stamps.max() == cutoff,
            'Frame timeline leaves the controlled prefix')
    return stamps, phases


def force_at(original, now):
    values = original['wrench']
    index = int(np.searchsorted(values[:, 1], now, side='right')) - 1
    return 0. if index < 0 else float(values[index, 2])


def draw_overlay(ImageDraw, canvas, bundle, now, quat, joints, phase, fonts):
    original, data = bundle['original'], bundle['data']
    cutoff, t = original['cutoff_task_s'], original['t']
    draw = ImageDraw.Draw(canvas)
    draw.text((26, 12), 'FAILED DISTURBANCE TEST | C++ COORDINATED MPC', fill=visual.FG, font=fonts[30])
    draw.text((26, 55), 'Original 2026-10-24 recording | SAME RUN, TWO VIEWS | No physics rerun',
              fill=visual.MUTED, font=fonts[22])
    draw.text((26, 91), 'Planned 24 s fast figure-eight; control aborted at 7.544 s | Full-run acceptance: FAILED',
              fill=RED, font=fonts[20])
    draw.text((26, 126), 'Fixed world view | Yellow: planned path; blue: measured trail', fill=visual.FG, font=fonts[20])
    draw.text((826, 126), 'Arm detail | Same recorded body and actual joint states', fill=visual.FG, font=fonts[20])
    instantaneous = float(np.interp(now, t, original['norm']*1000))
    force = force_at(original, now)
    paused = phase == 'FINAL STATE PAUSED'
    event = 'QP INFEASIBLE | sequence 379 | FINAL MEASURED STATE' if paused else (
        f'EXTERNAL FORCE: +{force:.1f} N world X | pulse in progress' if force > 0 else
        'NO ADDED FORCE | first pulse scheduled at task 7.000 s')
    draw.rectangle((18, 754, 1582, 796), fill='#5b232b' if paused else '#473f21' if force else '#193044')
    draw.text((30, 761), event, fill=RED if paused else GOLD if force else visual.FG, font=fonts[22])
    w, x, y, z = quat
    roll = math.degrees(math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)))
    pitch = math.degrees(math.asin(float(np.clip(2*(w*y-z*x), -1, 1))))
    draw.text((26, 807), f'EE error {instantaneous:.2f} mm | FAILED-prefix RMS {original["metrics"]["tip_3d_rmse_m"]*1000:.2f} mm',
              fill=visual.ACTUAL, font=fonts[22])
    draw.text((826, 807), f'Body roll {roll:+.2f} deg | pitch {pitch:+.2f} deg | q [{joints[0]:+.3f}, {joints[1]:+.3f}] rad',
              fill=visual.MUTED, font=fonts[20])
    draw.text((26, 850), 'SAME-TIME 3D TIP ERROR [mm] | ONLY 0-7.544 s', fill=visual.FG, font=fonts[20])
    draw.text((960, 850), 'No complete 24 s RMS or recovery claim', fill=RED, font=fonts[20])
    left, right, top, bottom = 95, 1545, 895, 1040
    ymax = 60.
    for value in range(0, 61, 15):
        yy = bottom-(bottom-top)*value/ymax
        draw.line((left, yy, right, yy), fill='#304154')
        draw.text((30, yy-10), str(value), fill=visual.MUTED, font=fonts[18])
    ids = visual.past_indices(t, now, 2200)
    points = [(left+(right-left)*t[j]/cutoff, bottom-(bottom-top)*original['norm'][j]*1000/ymax) for j in ids]
    if len(points) > 1:
        draw.line(points, fill=visual.ACTUAL, width=3)
    onset = left+(right-left)*7./cutoff
    draw.line((onset, top, onset, bottom), fill=GOLD, width=2)
    draw.text((onset-145, top-24), 'Force starts 7.000 s', fill=GOLD, font=fonts[18])
    cursor = left+(right-left)*now/cutoff
    draw.line((cursor, top, cursor, bottom), fill=RED if paused else '#9eafc0', width=2)
    for value in (0, 1, 2, 3, 4, 5, 6, 7):
        draw.text((left+(right-left)*value/cutoff-7, bottom+7), str(value), fill=visual.MUTED, font=fonts[18])
    draw.text((right-55, bottom+7), '7.544', fill=RED, font=fonts[18])
    draw.text((26, 1080), f'TASK TIME {now:6.3f} s | {phase} | First pulse not completed; recovery NOT evaluated',
              fill=RED if paused else visual.MUTED, font=fonts[20])
    draw.text((26, 1114), 'Recorded-state visualization; original URDF meshes. Force arrow is illustrative; unlogged rotors stay fixed.',
              fill=visual.MUTED, font=fonts[18])


def draw_force(mujoco, scene, position, force):
    if force <= 0:
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(geom, mujoco.mjtGeom.mjGEOM_ARROW, np.zeros(3), np.zeros(3),
                      np.eye(3).ravel(), np.array([1., .38, .24, 1.], dtype=np.float32))
    # A fixed display scale, not a reconstructed force measurement.
    mujoco.mjv_connector(geom, mujoco.mjtGeom.mjGEOM_ARROW, .012,
                        position-np.array([.40, 0., -.10]), position+np.array([.04, 0., .10]))
    geom.emission = .5
    scene.ngeom += 1


def render_frames(bundle, output, stamps, phases, ffmpeg, *, preview=False):
    import mujoco
    from PIL import Image, ImageDraw
    replay = visual.replay
    scene_path, geometry = replay.make_scene(output/'render_geometry', bundle['source'],
                                            bundle['robot'], visual.PANEL_WIDTH, visual.SCENE_HEIGHT)
    model = mujoco.MjModel.from_xml_path(str(scene_path))
    model.vis.quality.offsamples = 0
    state = mujoco.MjData(model)
    renderer = mujoco.Renderer(model, height=visual.SCENE_HEIGHT, width=visual.PANEL_WIDTH, max_geom=2200)
    proc, log = None, None
    try:
        mocaps = {name: model.body_mocapid[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'replay_'+name)]
                  for name in [*replay.kinematic_poses(bundle['robot']), 'tip_reference']}
        markers = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, 'tip_reference_'+str(i)) for i in range(3)]
        cameras = [mujoco.MjvCamera(), mujoco.MjvCamera()]
        for camera in cameras:
            mujoco.mjv_defaultCamera(camera)
            camera.type = mujoco.mjtCamera.mjCAMERA_FREE
            camera.azimuth, camera.elevation = 110., -14.
        cameras[0].distance, cameras[1].distance = 2.05, .85
        cameras[0].lookat[:] = bundle['data']['p'][0]+[0, 0, -.22]
        ref = bundle['reference']
        target_path = np.asarray([ref.sample(t)[0] for t in np.linspace(ref.start_s, ref.end_s, 400)])
        fonts = {size: replay.find_font(size) for size in (18, 20, 22, 30)}
        video = output/'X650_wrench_failed_original.mp4'
        if not preview:
            log = (output/'ffmpeg.log').open('x', encoding='utf-8')
            proc = subprocess.Popen([ffmpeg, '-n', '-f', 'rawvideo', '-pix_fmt', 'rgb24', '-s', f'{WIDTH}x{HEIGHT}',
                '-r', str(FPS), '-i', '-', '-an', '-c:v', 'libx264', '-preset', 'fast', '-crf', '20',
                '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(video)],
                stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=log)
        last_key, canvas = None, None
        for index, (now, phase) in enumerate(zip(stamps, phases)):
            key = (float(now), phase)
            if key != last_key:
                data = bundle['data']
                p, quat, joints, _, gap = replay.sample_state(data, float(now), gap_limit=.004+1e-8)
                require(not gap, 'Native pose interpolation crosses missing physical samples')
                for name, pose in replay.world_link_poses(bundle['robot'], p, quat, joints).items():
                    state.mocap_pos[mocaps[name]] = pose[:3, 3]
                    state.mocap_quat[mocaps[name]] = replay.rotation_quaternion(pose[:3, :3])
                state.mocap_pos[mocaps['tip_reference']] = ref.sample(float(now))[0]
                model.geom_rgba[markers, 3] = .9
                mujoco.mj_forward(model, state)  # Rendering kinematics only; never mj_step.
                cameras[1].lookat[:] = replay.camera_targets(p, quat, joints, bundle['geometry_report'])[1]
                trail = data['provided_tip'][visual.past_indices(data['elapsed'], float(now), 360)]
                canvas = Image.new('RGB', (WIDTH, HEIGHT), visual.BG)
                for view, camera in enumerate(cameras):
                    renderer.update_scene(state, camera=camera)
                    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
                    renderer.scene.flags[mujoco.mjtRndFlag.mjRND_REFLECTION] = False
                    visual.add_path(mujoco, renderer.scene, target_path, visual.TARGET, .0017)
                    visual.add_path(mujoco, renderer.scene, trail, visual.ACTUAL, .0015)
                    draw_force(mujoco, renderer.scene, p, force_at(bundle['original'], float(now)))
                    canvas.paste(Image.fromarray(renderer.render().copy()), (view*visual.PANEL_WIDTH, visual.SCENE_Y))
                draw_overlay(ImageDraw, canvas, bundle, float(now), quat, joints, phase, fonts)
                last_key = key
            if preview:
                canvas.save(output/'preview.png')
            else:
                if index in {0, 167, 168, len(stamps)-73, len(stamps)-72, len(stamps)-1}:
                    canvas.save(output/f'frame_{index:05d}.png')
                proc.stdin.write(np.asarray(canvas).tobytes())
            if index % 48 == 0 or index == len(stamps)-1:
                print(json.dumps({'frame': index, 'total': len(stamps), 'task_s': float(now), 'phase': phase}), flush=True)
        if preview:
            decoded = None
        else:
            proc.stdin.close()
            require(proc.wait() == 0, 'Video encoder failed; see ffmpeg.log')
            decoded = visual.video_decode(ffmpeg, video, len(stamps))
            canvas.save(output/'final_state.png')
        return dict(video=None if preview else identity(video), decoded_video=decoded,
                    geometry=geometry, effective_offscreen_samples=int(model.vis.quality.offsamples))
    finally:
        visual.close_media(proc, log, renderer)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True)
    parser.add_argument('--failure-analysis', type=Path, required=True)
    parser.add_argument('--failure-analysis-sha256')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--ffmpeg')
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument('--audit-only', action='store_true')
    mode.add_argument('--preview-only', action='store_true')
    parser.add_argument('--preview-time', type=float)
    args = parser.parse_args()
    bundle = load_recording(args.run, args.failure_analysis, args.failure_analysis_sha256)
    output = args.output.resolve()
    require(not output.exists(), 'New presentation output directory required')
    for protected in (Path(args.run), Path(args.failure_analysis).parent, bundle['source'].parent):
        require(not output.is_relative_to(protected.resolve()), 'Output may not enter original evidence/model folders')
    original = bundle['original']
    stamps, phases = frame_clock(original['cutoff_task_s'])
    if args.preview_only:
        now = original['cutoff_task_s'] if args.preview_time is None else args.preview_time
        require(0 <= now <= original['cutoff_task_s'], 'Preview lies outside failed prefix')
        stamps = np.array([now]); phases = ['FINAL STATE PAUSED' if now == original['cutoff_task_s']
            else 'SLOW MOTION 0.25x' if now >= 7 else 'REPLAY 1x']
    ffmpeg = None if args.audit_only or args.preview_only else visual.find_ffmpeg(args.ffmpeg)
    if ffmpeg:
        bundle['inputs']['ffmpeg'] = identity(ffmpeg)
    output.mkdir(parents=True, exist_ok=False)
    record = dict(kind='x650_failed_wrench_recorded_visualization_v1', presentation_complete=False,
        source_run_failed=True, physical_acceptance=False, full_experiment_complete=False,
        eligible_as_stage_gate=False, hardware_qualified=False, new_physics_run=False,
        native_gazebo_screen_capture=False, original_run=str(original['run']),
        controlled_prefix_task_s=[float(original['t'][0]), original['cutoff_task_s']],
        failed_sequence=original['sequence'], failure_reason='QP infeasible', recovery_performance_evaluated=False,
        prefix_only_metrics=original['metrics'], native_sample_count=len(original['t']),
        fps=FPS, width=WIDTH, height=HEIGHT, frames=len(stamps),
        video_duration_s=None if args.audit_only or args.preview_only else len(stamps)/FPS,
        timeline_policy='0-7 s at1x; pulse interval at0.25x; exact rejected final state paused3s. No source sample after7.544s.',
        same_run_two_views=True, original_mesh_geometry=True, no_mesh_decimation=True,
        force_overlay='Recorded ECS enqueued world-X command; arrow illustrative, not a force sensor',
        pose_interpolation='Measured same-step position/joints linear; quaternion SLERP; no dynamics integration',
        fk_audit=bundle['fk'], input_hashes=bundle['inputs'],
        audit_only=args.audit_only, preview_only=args.preview_only,
        python=sys.executable, mujoco_gl=os.environ.get('MUJOCO_GL'),
        package_versions={p: importlib.metadata.version(p) for p in ('numpy','mujoco','Pillow')})
    save(output/'render_status.json', record)
    try:
        with (output/'display_frame_clock.csv').open('x', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream); writer.writerow(['frame','video_pts_s','source_task_s','display_phase'])
            writer.writerows((i,i/FPS,float(t),phase) for i,(t,phase) in enumerate(zip(stamps,phases)))
        if not args.audit_only:
            record.update(render_frames(bundle, output, stamps, phases, ffmpeg, preview=args.preview_only))
        visual.unchanged(bundle['inputs'])
        record.update(presentation_complete=True, video_produced=not (args.audit_only or args.preview_only))
        save(output/'replay_manifest.json', record); save(output/'render_status.json', record)
        save(output/'delivery_manifest.json', dict(kind='x650_failed_visualization_delivery_v1',
            physical_acceptance=False, files={p.name:identity(p) for p in output.iterdir() if p.is_file()}))
        print(json.dumps({k:record.get(k) for k in ('presentation_complete','video','frames','decoded_video','prefix_only_metrics')}, indent=2))
    except BaseException as error:
        record.update(error=repr(error)); save(output/'render_status.json', record); raise
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
