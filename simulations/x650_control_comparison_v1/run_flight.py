"""Shared PX4 velocity-only experiment. Truth is evaluation-only, clock is ROS /clock."""
from __future__ import annotations

import csv
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(ROOT / 'simulations/x650_hover_v1'))
from parameter_evidence import ParameterEvidence
from run_case import FlightTelemetry, euler
from amsim import ScenarioContext
from amsim.modeling.manager import resolve, validate_px4_airframe
from amsim.runtime.process import status as runtime_status
from geometry_msgs.msg import Vector3
from rosgraph_msgs.msg import Clock
from rclpy.qos import qos_profile_sensor_data
from pymavlink import mavutil
from controllers import make_controller
from protocol import CASES, ALGORITHMS, reference, identification_command, limit_command, enu_ned
from analysis import summarize_trace, LIMITS
from processes import stop_process_group
from clock_observation import ClockObservation

JOINTS = ['upper_arm_joint', 'fore_crank_joint']


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


class Wrench:
    """One persistent world-frame wrench, cleared before landing and on failure.

    Publish success means transport accepted the request, not a measured force.
    The log deliberately labels it as requested force.
    """
    def __init__(self, world, instance):
        from gz.transport13 import Node
        from gz.msgs10.entity_wrench_pb2 import EntityWrench
        from gz.msgs10.entity_pb2 import Entity
        self.node = Node()
        self.EntityWrench, self.Entity = EntityWrench, Entity
        self.entity = Entity(name=f'{instance}::base_link', type=Entity.LINK)
        self.force_pub = self.node.advertise(f'/world/{world}/wrench/persistent', EntityWrench)
        self.clear_pub = self.node.advertise(f'/world/{world}/wrench/clear', Entity)
        self.active = False

    def ready(self):
        return self.force_pub.has_connections() and self.clear_pub.has_connections()

    def set_active(self, value):
        if value == self.active:
            return
        if value:
            msg = self.EntityWrench()
            msg.entity.CopyFrom(self.entity)
            msg.wrench.force.x = 1.
            if not self.force_pub.publish(msg):
                raise RuntimeError('Force publication failed')
            self.active = True
        else:
            self.clear()

    def clear(self):
        if self.active:
            if not self.clear_pub.publish(self.entity):
                raise RuntimeError('Force clear publication failed')
            self.active = False


def main():
    cfg = yaml.safe_load(Path(os.environ['AMSIM_SCENARIO_CONFIG']).read_text())
    folder = Path(os.environ['AMSIM_RUN_DIR'])
    algorithm = os.environ.get('X650_COMPARE_ALGORITHM', 'pid')
    case = os.environ.get('X650_COMPARE_CASE', 'hover')
    if algorithm not in ALGORITHMS or case not in CASES:
        raise ValueError('Unknown algorithm or case')
    config_path = Path(os.environ.get('X650_COMPARE_CONFIG', HERE / 'config.json'))
    response_path = Path(os.environ.get('X650_COMPARE_MODEL', HERE / 'response_model.json'))
    config = json.loads(config_path.read_text())
    if config['acceptance'] != LIMITS:
        raise ValueError('Comparison acceptance limits are fixed by the common analysis protocol')
    if config['limits'] != {'speed': .4, 'slew': 1., 'joint_speed': .15}:
        raise ValueError('Common actuator limits are fixed for this comparison protocol')
    response = json.loads(response_path.read_text())
    model = resolve(cfg['model'])
    result = dict(success=False, execution_success=False, accuracy_accepted=False,
                  algorithm=algorithm, case=case, model=model.model_name, model_sha256=model.sha256,
                  config_sha256=sha(config_path), response_model_sha256=sha(response_path),
                  acceptance=config['acceptance'], requested_duration_s=CASES[case],
                  host_logical_cpus=os.cpu_count(), process_affinity_cpus=len(os.sched_getaffinity(0)),
                  control='External position controller -> shared PX4 velocity/attitude/rate loops',
                  force_semantics='Requested world-frame base_link force; transport publication is not force measurement')
    for path in HERE.glob('*.py'):
        shutil.copy2(path, folder / path.name)
    shutil.copy2(config_path, folder / 'controller_config.json')
    shutil.copy2(response_path, folder / 'response_model.json')
    if model.model_name != config['model'] or model.sha256 != config['model_sha256']:
        raise RuntimeError('Frozen comparison model identity mismatch')
    if response.get('identified') and (response.get('model_sha256') != model.sha256 or not response.get('validation_accepted')):
        raise RuntimeError('Response model belongs to another plant or did not pass validation')
    manifest = json.loads(model.manifest_path.read_text())
    if sha(manifest['world_installed']) != manifest['artifacts']['world.sdf']:
        raise RuntimeError('World differs from model manifest')
    world = ET.parse(manifest['world_installed']).getroot().find('world').get('name')
    controls, events, truth_rows = [], [], []
    closure = [None, 0.]
    clock = ClockObservation()
    telemetry = params = wrench = None
    attempted = False
    bridge_log = (folder / 'closure_bridge.log').open('w')
    bridge = subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/x650_cad_arm/closure_metrics@geometry_msgs/msg/Vector3[gz.msgs.Vector3d'],
        stdout=bridge_log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        with ScenarioContext() as ctx:
            original_pump = ctx.vehicle._pump
            def event(name, **extra):
                entry = dict(event=name, sim_time_s=clock.stamp, **extra)
                events.append(entry)
                print(json.dumps(entry), flush=True)
            def receive_clock(msg):
                value = msg.clock.sec + msg.clock.nanosec * 1e-9
                if clock.stamp is not None and value < clock.stamp:
                    result['clock_error'] = 'Simulation clock regressed'
                clock.receive(value, time.monotonic())
            ctx.node.create_subscription(Clock, '/clock', receive_clock, qos_profile_sensor_data)
            ctx.node.create_subscription(Vector3, '/x650_cad_arm/closure_metrics',
                lambda msg: closure.__setitem__(slice(None), [msg, time.monotonic()]), 10)
            try:
                ctx.vehicle.wait_ready()
                ctx.vehicle.wait(lambda: closure[0] is not None and clock.stamp is not None, 15, 'clock and closure')
                if ctx.model['model_sha256'] != model.sha256 or ctx.arm.joints != JOINTS:
                    raise RuntimeError('Active model/joint mismatch')
                if case == 'force':
                    runtime = json.loads(Path(runtime_status()['runtime_meta']).read_text())
                    wrench = Wrench(world, runtime['instance'])
                    ctx.vehicle.wait(wrench.ready, 10, 'Gazebo force and clear subscribers')
                airframe = validate_px4_airframe(model)
                params = ParameterEvidence(folder, airframe, model.sdf_path.parent / airframe.name,
                                           model.px4_sys_autostart, model.sha256)
                params.capture(ctx.vehicle, 'before', enforce_contract=True)
                initial = ctx.observe(ground_truth=True)
                if initial.vehicle.armed or not initial.vehicle.landed:
                    raise RuntimeError('Fresh grounded state required')
                height = cfg['vehicle']['takeoff_height_m']
                g = initial.ground_truth
                truth_anchor = np.array([g.x, g.y, g.z + height])
                est = ctx.node.data['position']
                estimate_anchor = enu_ned([est.x, est.y, est.z - height])
                resets = (est.xy_reset_counter, est.z_reset_counter)
                yaw = euler(g.qx, g.qy, g.qz, g.qw)[2]
                telemetry = FlightTelemetry(folder, 50, model.sha256, truth_anchor, yaw, truth_rows)
                telemetry.attach(ctx)
                telemetry.phase = 'takeoff'
                attempted = True
                event('takeoff_begin')
                def takeoff_pump(ros_timeout_s=.02):
                    original_pump(ros_timeout_s)
                    g = ctx.node.observation(include_ground_truth=True).ground_truth
                    angles = euler(g.qx, g.qy, g.qz, g.qw)
                    if max(abs(angles[0]), abs(angles[1])) > math.radians(35) or g.z > truth_anchor[2] + 1.5:
                        raise RuntimeError('Takeoff envelope exceeded')
                ctx.vehicle._pump = takeoff_pump
                ctx.vehicle.takeoff(height, stable_s=2.5)
                ctx.vehicle._pump = original_pump
                ctx.wait(2.)
                controller = make_controller(algorithm, config, response)
                est = ctx.node.data['position']
                controller.reset(enu_ned([est.x, est.y, est.z]), enu_ned([est.vx, est.vy, est.vz]), np.zeros(3))
                yaw_ned = float(est.heading)
                start, last, last_send = [None], [None], [time.monotonic()]
                sent = np.zeros(3)
                def pump(ros_timeout_s=.02):
                    nonlocal sent
                    original_pump(ros_timeout_s)
                    sim = clock.stamp
                    dt = .02 if last[0] is None else sim - last[0]
                    now = time.monotonic()
                    timing_error = result.get('clock_error')
                    if now - clock.received_at > 1.:
                        timing_error = 'Simulation clock stale'
                    elif now - last_send[0] > .5:
                        timing_error = 'Offboard wall send gap exceeded .5s'
                    elif dt > .15:
                        timing_error = f'Control timestep exceeded: {dt}'
                    if timing_error:
                        result['timing_fault'] = dict(reason=timing_error, sim_time_s=sim,
                            sim_dt_s=dt, wall_since_last_send_s=now-last_send[0], **clock.snapshot(now))
                        raise RuntimeError(timing_error)
                    if dt < .019999:
                        return
                    last[0] = sim
                    t = -1. if start[0] is None else sim - start[0]
                    est, js = ctx.node.data['position'], ctx.arm.state()
                    if (est.xy_reset_counter, est.z_reset_counter) != resets:
                        raise RuntimeError('Estimator reset')
                    if not (est.xy_valid and est.z_valid and est.v_xy_valid and est.v_z_valid):
                        raise RuntimeError('Invalid estimate')
                    for key in ('position', 'truth', 'joints'):
                        if time.monotonic() - ctx.node.received.get(key, 0) > 1.:
                            raise RuntimeError('Stale ' + key)
                    if bridge.poll() is not None or time.monotonic() - closure[1] > 1.:
                        raise RuntimeError('Clock/closure telemetry stale')
                    if not np.all(np.isfinite([closure[0].x, closure[0].y])) or closure[0].x > .0005 or closure[0].y > .005:
                        raise RuntimeError('Closed-chain error exceeded limit')
                    offset, q_target, force = reference(case, t)
                    pos = enu_ned([est.x, est.y, est.z])
                    vel = enu_ned([est.vx, est.vy, est.vz])
                    target = estimate_anchor + offset
                    if not np.all(np.isfinite(np.r_[pos, vel])):
                        raise RuntimeError('Nonfinite feedback')
                    begun = time.perf_counter()
                    try:
                        raw = (np.zeros(3) if start[0] is None else
                               identification_command(t, pos, target) if case == 'identify' else
                               controller.step(dt, pos, vel, target))
                    except Exception:
                        result['controller_failure'] = dict(t=t, diagnostics=controller.diagnostics,
                            compute_ms=(time.perf_counter()-begun)*1000)
                        raise
                    compute_ms = (time.perf_counter() - begun) * 1000
                    cmd, limited = limit_command(raw, sent, dt, config['limits']['speed'], config['limits']['slew'])
                    # A solver failure is fatal; do not silently substitute another controller.
                    if case != 'identify' and compute_ms > 20.:
                        result['deadline_failure_ms'] = compute_ms
                        result['deadline_failure_t'] = t
                        result['controller_failure'] = dict(t=t, compute_ms=compute_ms, diagnostics=controller.diagnostics)
                        raise RuntimeError(f'{algorithm} exceeded 20ms deadline: {compute_ms:.3f}')
                    q = np.array([js[j].position_rad for j in JOINTS])
                    qvel = np.clip(1.5 * (q_target - q), -config['limits']['joint_speed'], config['limits']['joint_speed'])
                    if not np.all(np.isfinite(qvel)):
                        raise RuntimeError('Nonfinite arm command')
                    vn = enu_ned(cmd)
                    ctx.vehicle.link.mav.set_position_target_local_ned_send(int(sim * 1000) & 0xffffffff,
                        *ctx.vehicle.target, mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                        1|2|4|64|128|256|2048, 0, 0, 0, *vn, 0, 0, 0, yaw_ned, 0)
                    for joint, speed in zip(JOINTS, qvel):
                        ctx.arm.set_velocity(joint, float(speed))
                    sent = cmd.copy()
                    if start[0] is not None:
                        controller.accept_applied(sent, dt)
                    sent_at = time.monotonic()
                    wall_send_interval = sent_at - last_send[0]
                    last_send[0] = sent_at
                    clock_timing = clock.snapshot(sent_at)
                    clock.reset_window()
                    if wrench and bool(force[0]) != wrench.active:
                        wrench.set_active(bool(force[0]))
                        event('force_request_on' if force[0] else 'force_request_clear')
                    obs = ctx.node.observation(include_ground_truth=True)
                    if obs.vehicle.failsafe or not obs.vehicle.armed:
                        raise RuntimeError('Unexpected flight state')
                    if t >= 0 and ctx.node.data['status'].nav_state != 14:
                        raise RuntimeError('Lost Offboard')
                    g = obs.ground_truth
                    roll, pitch, _ = euler(g.qx, g.qy, g.qz, g.qw)
                    tilt = math.acos(float(np.clip(math.cos(roll) * math.cos(pitch), -1, 1)))
                    if tilt > math.radians(40) or np.linalg.norm(np.array([g.x, g.y, g.z]) - truth_anchor) > 2.:
                        raise RuntimeError('Trial envelope exceeded')
                    msg = ctx.node.data['truth']
                    truth_stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    truth_t = -1 if start[0] is None else truth_stamp - start[0]
                    true_target = truth_anchor + reference(case, truth_t)[0]
                    telemetry.target = true_target
                    telemetry.check()
                    row = dict(t=t, sim_time_s=sim, dt=dt, px4_stamp_us=int(est.timestamp), truth_stamp_s=truth_stamp,
                        px4_sample_stamp_us=int(getattr(est, 'timestamp_sample', est.timestamp)),
                        position_receipt_age_s=sent_at-ctx.node.received['position'],
                        joint_receipt_age_s=sent_at-ctx.node.received['joints'],
                        truth_receipt_age_s=sent_at-ctx.node.received['truth'],
                        wall_send_interval_s=wall_send_interval, host_loadavg_1m=os.getloadavg()[0],
                        **clock_timing,
                        compute_ms=compute_ms, limited=limited, roll_deg=math.degrees(roll),
                        pitch_deg=math.degrees(pitch), tilt_deg=math.degrees(tilt),
                        q0=q[0], q1=q[1], q0_target=q_target[0], q1_target=q_target[1],
                        dq0=js[JOINTS[0]].velocity_rad_s, dq1=js[JOINTS[1]].velocity_rad_s,
                        dq0_cmd=qvel[0], dq1_cmd=qvel[1], closure_m=closure[0].x, axis_rad=closure[0].y,
                        diagnostics=json.dumps(controller.diagnostics))
                    for i, axis in enumerate('xyz'):
                        row.update({f'truth_{axis}': [g.x,g.y,g.z][i], f'target_{axis}': true_target[i],
                            f'command_target_{axis}': truth_anchor[i]+offset[i], f'est_{axis}':pos[i],
                            f'controller_target_enu_{axis}':target[i],
                            f'v{axis}':vel[i], f'v{axis}_raw':float(raw[i]), f'v{axis}_cmd':cmd[i], f'force_{axis}':force[i]})
                    controls.append(row)
                ctx.vehicle._pump = pump
                pre = time.monotonic()
                while time.monotonic() - pre < 1.25:
                    pump()
                ctx.vehicle._command(mavutil.mavlink.MAV_CMD_DO_SET_MODE, [1,6,0])
                ctx.vehicle.wait(lambda: ctx.node.data['status'].nav_state == 14, 10, 'Offboard mode')
                est = ctx.node.data['position']
                controller.reset(enu_ned([est.x,est.y,est.z]), enu_ned([est.vx,est.vy,est.vz]), sent)
                start[0] = clock.stamp
                clock.reset_window()
                telemetry.phase = 'measure'
                event('measurement_begin', algorithm=algorithm, case=case)
                ctx.vehicle.wait(lambda: clock.stamp - start[0] >= CASES[case], max(120, CASES[case]*5), 'comparison trial')
                if wrench:
                    wrench.clear()
                ctx.vehicle._pump = original_pump
                ctx.arm.stop()
                telemetry.phase = 'landing'
                event('landing_begin')
                ctx.vehicle.land()
                result['landing_confirmed'] = True
                event('landed_and_disarmed')
                telemetry.phase = 'landed'
                ctx.wait(.5)
                params.capture(ctx.vehicle, 'after', enforce_contract=False)
                if not params.reports['after'].get('parameters_unchanged', False):
                    raise RuntimeError('PX4 parameters changed during trial')
                result['execution_success'] = result['success'] = True
            except (Exception, KeyboardInterrupt) as exc:
                result['error'] = f'{type(exc).__name__}: {exc}'
                event('trial_failed', error=result['error'])
                if wrench:
                    try:
                        wrench.clear()
                    except Exception as error:
                        result['force_clear_error'] = str(error)
                ctx.vehicle._pump = original_pump
                ctx.arm.stop()
                if attempted:
                    try:
                        if telemetry:
                            telemetry.phase = 'recovery_landing'
                        ctx.vehicle.land()
                        result['recovery_landed'] = True
                    except Exception as error:
                        result['recovery_error'] = str(error)
            finally:
                ctx.vehicle._pump = original_pump
                if telemetry:
                    telemetry.detach()
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = f'{type(exc).__name__}: {exc}'
    finally:
        if wrench:
            try:
                wrench.clear()
            except Exception as error:
                result['force_clear_error'] = str(error)
        stop_process_group(bridge)
        bridge_log.close()
        if telemetry:
            result['telemetry'] = telemetry.close()
        if params:
            result['parameters'] = params.reports
        if controls:
            with (folder / 'control_trace.csv').open('w') as stream:
                writer = csv.DictWriter(stream, fieldnames=list(controls[0]))
                writer.writeheader()
                writer.writerows(controls)
            try:
                result['metrics'] = summarize_trace(controls, case) if case != 'identify' else {}
                metrics = result['metrics']
                result['accuracy_accepted'] = bool(result['execution_success'] and metrics.get('accuracy_accepted', False))
                if case == 'arm':
                    measured = [r for r in controls if r['t'] >= 0]
                    settled = [r for r in measured if 11 <= r['t'] <= 12.5]
                    errors = [max((abs(r[f'q{i}']-r[f'q{i}_target']) for r in settled), default=1e9) for i in range(2)]
                    motion = [max(r[f'q{i}'] for r in measured)-min(r[f'q{i}'] for r in measured) for i in range(2)] if measured else [0.,0.]
                    result['arm_motion_verified'] = bool(max(errors)<.03 and min(motion)>.07)
                    result['arm_settled_error_rad'], result['arm_range_rad'] = errors, motion
                    if not result['arm_motion_verified']:
                        result['success'] = result['execution_success'] = result['accuracy_accepted'] = False
                        result['error'] = result.get('error', 'Prescribed arm motion was not achieved')
            except Exception as error:
                result['analysis_error'] = str(error)
                result['success'] = False
        result['events'] = events
        (folder / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False)+'\n')
    print(json.dumps(result, indent=2), flush=True)
    return 0 if result['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
