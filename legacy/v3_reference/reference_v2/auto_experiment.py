"""SITL-only automatic baseline flight. ROS telemetry + local MAVLink GCS."""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.executors import SingleThreadedExecutor
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleLandDetected
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from pymavlink import mavutil
from bounded_arm_excitation import Experiment
from ground_stow import GroundStow, validate_model
from root_trial import velocity as root_velocity, reference as root_reference

PREFIX = '/model/x500_arm2dof_0/joint/'
TOPICS = ['/arm/joint_states', PREFIX+'arm_joint_1/cmd_vel', PREFIX+'arm_joint_2/cmd_vel',
          '/fmu/out/vehicle_local_position_v1', '/fmu/out/vehicle_attitude',
          '/fmu/out/vehicle_odometry', '/fmu/out/vehicle_status_v1',
          '/fmu/out/vehicle_control_mode', '/fmu/out/vehicle_land_detected',
          '/ground_truth/odometry', '/clock']


class Telemetry(Node):
    def __init__(self):
        super().__init__('aerial_experiment_supervisor')
        self.data = {}
        self.received = {}
        self.subs = []
        for key, typ, topic in (
                ('position', VehicleLocalPosition, TOPICS[3]),
                ('status', VehicleStatus, TOPICS[6]),
                ('land', VehicleLandDetected, TOPICS[8]),
                ('joints', JointState, TOPICS[0]),
                ('truth', Odometry, '/ground_truth/odometry')):
            self.subs.append(self.create_subscription(typ, topic,
                lambda msg, key=key: self.receive(key, msg), qos_profile_sensor_data))
        self.pubs = [self.create_publisher(Float64, PREFIX+n+'/cmd_vel', 10)
                     for n in ('arm_joint_1', 'arm_joint_2')]

    def receive(self, key, msg):
        old = self.data.get(key)
        stamp = lambda m: (m.header.stamp.sec, m.header.stamp.nanosec) if key in ('joints', 'truth') else m.timestamp
        if old is None or stamp(old) != stamp(msg):
            self.received[key] = time.monotonic()
        self.data[key] = msg

    def fresh(self, require_truth=True):
        keys = ('position', 'status', 'land', 'joints') + (('truth',) if require_truth else ())
        return all(time.monotonic()-self.received.get(k, 0) < (3 if k in ('status','land') else 2) for k in keys)

    def zero(self):
        for pub in self.pubs:
            pub.publish(Float64(data=0.0))


class Run:
    def __init__(self, folder):
        # Exclusive probe avoids silently sharing QGC's UDP port.
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
            probe.bind(('0.0.0.0', 14550))
        self.link = mavutil.mavlink_connection('udpin:127.0.0.1:14550', source_system=255,
                                             source_component=190)
        self.target = None
        self.heartbeat_time = 0
        self.last_heartbeat = 0
        self.global_position = None
        self.global_time = 0
        self.ack = {}
        self.node = Telemetry()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.arm = None
        self.stow = None
        self.stowed_guard = False
        self.folder = folder
        self.recorder = None
        self.attempted_arm = False
        self.airborne = False
        self.monitor_flight = False
        self.origin = None
        self.reset_counters = None
        self.events = []
        self.last_sim_stamp = None
        self.root_trial = False
        self.motion_active = False

    def event(self, text):
        print(text, flush=True)
        truth = self.node.data.get('truth')
        sim_time = (truth.header.stamp.sec+truth.header.stamp.nanosec/1e9) if truth else None
        self.events.append({'wall_time': time.time(), 'sim_time_latest_truth': sim_time, 'event': text})

    def pump(self):
        self.executor.spin_once(timeout_sec=0.02)
        if 'truth' in self.node.data:
            self.sim_seconds()
        now = time.monotonic()
        for _ in range(100):
            msg = self.link.recv_match(blocking=False)
            if msg is None:
                break
            if msg.get_type() == 'HEARTBEAT' and msg.autopilot == mavutil.mavlink.MAV_AUTOPILOT_PX4:
                identity = (msg.get_srcSystem(), msg.get_srcComponent())
                if self.target is not None and self.target != identity:
                    raise RuntimeError('Multiple PX4 systems detected.')
                self.target = identity
                self.last_heartbeat = now
            if self.target and (msg.get_srcSystem(), msg.get_srcComponent()) == self.target:
                if msg.get_type() == 'COMMAND_ACK':
                    self.ack[msg.command] = (now, msg.result)
                elif msg.get_type() == 'GLOBAL_POSITION_INT':
                    self.global_position, self.global_time = msg, now
                elif msg.get_type() == 'STATUSTEXT' and msg.severity <= 4:
                    self.event('PX4: '+msg.text)
        if self.target and now-self.heartbeat_time >= 0.5:
            self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
            self.heartbeat_time = now
        if self.recorder and self.recorder.poll() is not None:
            raise RuntimeError('Recorder exited unexpectedly; inspect recorder.log.')
        if self.monitor_flight:
            if not self.node.fresh() or now-self.last_heartbeat > 3:
                raise RuntimeError('Flight telemetry stale.')
            p = self.node.data['position']
            st = self.node.data['status']
            if not all(math.isfinite(x) for x in (p.x, p.y, p.z, p.vx, p.vy, p.vz)):
                raise RuntimeError('Nonfinite position or velocity.')
            if self.reset_counters != (p.xy_reset_counter, p.z_reset_counter):
                raise RuntimeError('Estimator position reference reset during experiment.')
            if not (p.xy_valid and p.z_valid and p.v_xy_valid and p.v_z_valid):
                raise RuntimeError('Invalid position estimate.')
            if st.failsafe:
                raise RuntimeError('PX4 failsafe active.')
            if self.root_trial and self.airborne:
                quat = self.node.data['truth'].pose.pose.orientation
                vals = (quat.w, quat.x, quat.y, quat.z)
                if not all(math.isfinite(v) for v in vals):
                    raise RuntimeError('Nonfinite truth attitude.')
                tilt = math.acos(max(-1., min(1., 1-2*(quat.x**2+quat.y**2))))
                if tilt > math.radians(10):
                    raise RuntimeError('True tilt exceeds 10 degrees.')
                if abs(p.z-(self.origin[2]-2.5)) > .5 or math.hypot(p.x-self.origin[0],p.y-self.origin[1]) > .5:
                    raise RuntimeError('Small-motion flight envelope exceeded (0.5 m).')
            if self.stowed_guard:
                self.check_stowed_joints()
                if any(self.node.count_publishers(PREFIX+n+'/cmd_vel') != 1 for n in ('arm_joint_1','arm_joint_2')):
                    raise RuntimeError('Competing arm command publisher.')
            if self.origin and (math.hypot(p.x-self.origin[0], p.y-self.origin[1]) > 2 or -p.z+self.origin[2] > 4):
                raise RuntimeError('Experiment position envelope exceeded.')
            if self.airborne and (self.node.data['land'].landed or st.arming_state != VehicleStatus.ARMING_STATE_ARMED):
                raise RuntimeError('Unexpected landing or disarm.')
        if not self.motion_active and self.stow is None and (self.arm is None or self.arm.finished):
            self.node.zero()

    def check_stowed_joints(self):
        joints=self.node.data['joints']
        for name,target in [('arm_joint_1',-16),('arm_joint_2',-85)]:
            i=joints.name.index(name);q=joints.position[i];v=joints.velocity[i]
            lower, upper = ((-18.5,-15.5) if self.motion_active and name=='arm_joint_1' else (target-.5,target+.5))
            if not math.isfinite(q) or not math.isfinite(v) or not math.radians(lower)<=q<=math.radians(upper) or abs(v)>(.04 if self.motion_active else .10):
                raise RuntimeError('Stowed joint angle/velocity outside holding limits: '+name)

    def move_root(self):
        self.motion_active = True
        samples = []
        self.event('ROOT MOTION: -16 -> -18 -> -16 deg; elbow held at -85 deg')
        start = self.sim_seconds()
        prev = None
        u = 0.
        deadline = time.monotonic()+180
        try:
            while time.monotonic()<deadline:
                self.pump()
                j=self.node.data['joints']
                stamp=j.header.stamp.sec+j.header.stamp.nanosec*1e-9
                if stamp==prev: continue
                if prev is None:
                    prev=stamp
                    start=stamp
                    continue
                dt=stamp-prev;prev=stamp
                if dt<=0 or dt>.2: raise RuntimeError('Root-motion joint feedback gap/reset')
                i=j.name.index('arm_joint_1');e=j.name.index('arm_joint_2')
                q=j.position[i];v=j.velocity[i];t=stamp-start
                u=root_velocity(t,q,u,dt)
                self.node.pubs[0].publish(Float64(data=u))
                self.node.pubs[1].publish(Float64(data=0.))
                samples.append([stamp,root_reference(t)[0],q,v,u,j.position[e]])
                if t>=12 and abs(q-math.radians(-16))<math.radians(.1) and abs(v)<.003 and abs(u)<.001:
                    self.event('ROOT MOTION COMPLETE: returned to -16 deg')
                    return
                if t>18: raise RuntimeError('Root motion failed to settle')
            raise RuntimeError('Root motion wall timeout')
        finally:
            self.motion_active=False
            self.node.zero()
            (self.folder/'root_motion.json').write_text(json.dumps({'columns':['sim_s','q_ref_rad','q_rad','v_rad_s','cmd_rad_s','elbow_rad'],'samples':samples}))

    def prepare_stow(self):
        self.stow=GroundStow(publishers=self.node.pubs)
        self.executor.add_node(self.stow)
        try:
            self.wait(lambda:self.stow.done,185,'Ground stow before arming: elbow -85 deg, root -16 deg')
            (self.folder/'ground_stow.json').write_text(json.dumps({'success':self.stow.success,'reason':self.stow.reason,'stop_state':self.stow.stop_info,'samples':self.stow.records}))
            if not self.stow.success:raise RuntimeError(self.stow.reason)
        finally:
            self.stow.publish([0.,0.])
            self.executor.remove_node(self.stow)
            self.stow.destroy_node();self.stow=None
        self.check_stowed_joints()
        self.stowed_guard=True
        self.event('Ground stow complete; zero-velocity holding with joint monitoring')

    def wait(self, predicate, timeout, description):
        self.event(description)
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            self.pump()
            if predicate():
                return
        raise RuntimeError('Timeout: '+description)

    def command(self, command, params):
        self.ack.pop(command, None)
        self.link.mav.command_long_send(*self.target, command, 0, *(params+[0.0]*7)[:7])
        self.wait(lambda: command in self.ack, 8, f'Awaiting command {command} acknowledgement')
        result = self.ack[command][1]
        if result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError(f'Command {command} result={result}; not accepted. No force arming.')

    def hover_ok(self, target_z):
        p = self.node.data['position']
        return abs(p.z-target_z) < 0.30 and math.sqrt(p.vx**2+p.vy**2+p.vz**2) < 0.25

    def sim_seconds(self):
        # Gazebo odometry header is in simulation time; PX4 ROS timestamps can
        # be timesync-adjusted. Never mix those clocks for experiment duration.
        stamp=self.node.data['truth'].header.stamp
        value=stamp.sec+stamp.nanosec*1e-9
        if not math.isfinite(value):raise RuntimeError('Invalid simulation time')
        if self.last_sim_stamp is not None and value < self.last_sim_stamp-1e-9:
            raise RuntimeError('Simulation clock moved backwards')
        self.last_sim_stamp=value
        return value

    def hold(self, seconds, target_z):
        start = self.sim_seconds()
        self.wait(lambda: self.sim_seconds()-start >= seconds,
                  seconds*10+30, f'Hold {seconds} simulation seconds')
        self.event(f'Hold finished: requested={seconds}s, actual_sim={self.sim_seconds()-start:.3f}s')
        if not self.hover_ok(target_z):
            raise RuntimeError('Hover drifted outside starting tolerance.')

    def land(self):
        self.monitor_flight = False
        if self.arm:
            self.arm.stop('ABORT: landing requested')
        self.node.zero()
        self.command(mavutil.mavlink.MAV_CMD_NAV_LAND, [0, 0, 0, math.nan, math.nan, math.nan, math.nan])
        self.wait(lambda: self.node.fresh(require_truth=False) and self.node.data['land'].landed,
                  180, 'Waiting for confirmed landing')
        self.wait(lambda: self.node.fresh(require_truth=False) and self.node.data['status'].arming_state != VehicleStatus.ARMING_STATE_ARMED,
                  30, 'Waiting for automatic disarm')

    def execute(self, check_only=False, stationary=False, stowed=False, root_trial=False):
        self.root_trial=root_trial
        self.wait(lambda: self.target is not None, 45, 'Waiting for PX4 MAVLink heartbeat')
        self.wait(lambda: self.node.fresh(), 45, 'Waiting for PX4 and joint feedback')
        self.wait(lambda: self.node.data['status'].pre_flight_checks_pass,
                  60, 'Waiting for normal PX4 preflight checks (GCS heartbeat is active)')
        if self.node.data['status'].arming_state == VehicleStatus.ARMING_STATE_ARMED or not self.node.data['land'].landed:
            raise RuntimeError('Start requires disarmed aircraft on ground.')
        joints = self.node.data['joints']
        for name in ('arm_joint_1', 'arm_joint_2'):
            i = joints.name.index(name)
            if not math.isfinite(joints.position[i]) or not math.isfinite(joints.velocity[i]) or abs(joints.position[i]) > (math.radians(.4) if stowed else 1) or abs(joints.velocity[i]) > (0.01 if stowed else 0.02):
                raise RuntimeError('Arm initial pose is outside limits or moving.')
        self.wait(lambda: all(p.get_subscription_count() > 0 for p in self.node.pubs),
                  10, 'Checking ROS-to-Gazebo command subscribers')
        if check_only:
            self.event('CHECK PASSED: no arming or flight commands sent.')
            return
        self.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                     [mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 200000])
        self.wait(lambda: self.global_position is not None and time.monotonic()-self.global_time < 2,
                  15, 'Waiting for global altitude reference')
        p = self.node.data['position']
        self.origin = (p.x, p.y, p.z)
        self.reset_counters = (p.xy_reset_counter, p.z_reset_counter)
        target_z = p.z-2.5
        altitude_amsl = self.global_position.alt/1000+2.5
        with open(self.folder/'recorder.log', 'w') as output:
            self.recorder = subprocess.Popen(['ros2', 'bag', 'record', '-o', str(self.folder/'bag'), *TOPICS],
                stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        def recording():
            log = (self.folder/'recorder.log').read_text(errors='replace')
            return all(any('Subscribed to topic' in line and topic in line for line in log.splitlines()) for topic in TOPICS)
        self.wait(recording, 30, 'Waiting for recorder subscriptions to all experiment topics')
        if stowed:
            self.prepare_stow()
            self.wait(lambda:self.node.fresh() and self.node.data['status'].pre_flight_checks_pass and self.node.data['status'].arming_state==VehicleStatus.ARMING_STATE_DISARMED and self.node.data['land'].landed and time.monotonic()-self.global_time<2,
                      30,'Rechecking ground state and flight readiness after stow')
            self.check_stowed_joints()
            p=self.node.data['position']
            self.origin=(p.x,p.y,p.z)
            self.reset_counters=(p.xy_reset_counter,p.z_reset_counter)
            target_z=p.z-2.5
            altitude_amsl=self.global_position.alt/1000+2.5
        self.event(f'Takeoff reference: origin={self.origin}, target_z={target_z}, altitude_amsl={altitude_amsl}')
        self.monitor_flight = True
        self.attempted_arm = True
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, [1, 0])
        self.wait(lambda: self.node.data['status'].arming_state == VehicleStatus.ARMING_STATE_ARMED, 8, 'Confirming armed state')
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, [0, 0, 0, math.nan, math.nan, math.nan, altitude_amsl])
        stable_since = [None]
        def stable():
            stamp = self.sim_seconds()
            if self.hover_ok(target_z) and not self.node.data['land'].landed:
                if stable_since[0] is None:
                    stable_since[0] = stamp
                return stamp-stable_since[0] >= 3
            stable_since[0] = None
            return False
        self.wait(stable, 180, 'Waiting for 2.5 m hover stable for 3 simulation seconds')
        self.airborne = True
        if root_trial:
            self.hold(20,target_z)
            self.move_root()
            self.hold(20,target_z)
            self.land()
            self.event('COMPLETE: small root motion, landing and disarm confirmed.')
            return
        self.hold(10, target_z)
        if stationary:
            self.event('Stationary-arm control interval begins')
            self.hold(20, target_z)
        else:
            self.arm = Experiment()
            self.executor.add_node(self.arm)
            self.wait(lambda: self.arm.finished, 300, 'Executing bounded arm trajectory')
            if not self.arm.success:
                raise RuntimeError(self.arm.reason)
        self.hold(10, target_z)
        self.land()
        self.event('COMPLETE: flight, '+('stationary arm' if stationary else 'arm trajectory')+', landing and disarm confirmed.')

    def close(self):
        if self.recorder:
            try:
                os.killpg(self.recorder.pid, signal.SIGINT)
            except ProcessLookupError:
                pass
            try:
                self.recorder.wait(timeout=20)
            except subprocess.TimeoutExpired:
                self.event('Recorder did not finish: inspect bag before reuse.')
        self.node.zero()
        self.executor.shutdown()
        if self.arm:
            self.arm.destroy_node()
        self.node.destroy_node()
        self.link.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--check', action='store_true', help='Readiness check only; no flight')
    parser.add_argument('--stationary', action='store_true', help='Keep arm velocity zero during the 20 s test interval')
    parser.add_argument('--stowed', action='store_true', help='Fresh centered model: auto ground stow, then stationary flight ONLY')
    parser.add_argument('--root-small', action='store_true', help='SITL: stow, hover, root -16/-18/-16 degrees, land')
    args = parser.parse_args()
    if args.root_small: args.stowed=True
    if args.stowed:args.stationary=True
    folder = Path(os.environ.get('RESULTS_DIR', str(Path.home()/'px4_baseline_bags')))/('auto_'+time.strftime('%Y%m%d_%H%M%S')+f'_{os.getpid()}')
    folder.mkdir(parents=True)
    rclpy.init()
    # Keep ROS alive during Ctrl+C so zero/land requests can still be sent.
    signal.signal(signal.SIGINT, signal.default_int_handler)
    run = None
    result = {'success': False, 'check_only': args.check, 'stationary_arm': args.stationary, 'stowed_takeoff':args.stowed,
              'actuator_model': 'unchanged direct velocity control'}
    result['root_small']=args.root_small
    if args.root_small: result['stationary_arm']=False
    model = Path(os.environ.get('PX4_DIR', str(Path.home()/'PX4-Autopilot')))/'Tools/simulation/gz/models/x500_arm2dof/model.sdf'
    if model.is_file():
        import hashlib
        raw = model.read_bytes()
        (folder/'model_snapshot.sdf').write_bytes(raw)
        result['model_sha256'] = hashlib.sha256(raw).hexdigest()
    try:
        if args.stowed:validate_model(model)
        if args.root_small and result.get('model_sha256') != '709130aca20327c2c1c3e5eb065a0c2b9a2b330c5265bb07d5dda90b266eb30a':
            raise RuntimeError('Small-motion experiment requires the reviewed model hash.')
        run = Run(folder)
        run.execute(args.check, args.stationary, args.stowed, args.root_small)
        result['success'] = True
    except (Exception, KeyboardInterrupt) as exc:
        result['error'] = str(exc) or 'Interrupted by user'
        print('EXPERIMENT STOPPED: '+result['error'], flush=True)
        if run and run.attempted_arm:
            # Recorder failure must not prevent recovery flight commands.
            recorder = run.recorder
            run.recorder = None
            try:
                run.land()
                result['recovery_landed'] = True
            except (Exception, KeyboardInterrupt) as recovery:
                result['recovery_error'] = str(recovery)
                print('Landing NOT confirmed. Keep simulator running; inspect flight state.', flush=True)
            finally:
                run.recorder = recorder
    finally:
        if run:
            try:
                run.close()
            except Exception as cleanup:
                result['cleanup_error'] = str(cleanup)
                result['success'] = False
            finally:
                result['events'] = run.events
        (folder/'result.json').write_text(json.dumps(result, indent=2))
        print(f'Results: {folder}', flush=True)
        if rclpy.ok():
            rclpy.shutdown()
    if result['success'] and not args.check:
        print('Creating offline replay (no flight commands)...',flush=True)
        replay=subprocess.run(['/usr/bin/python3',str(Path(__file__).with_name('make_replay.py')),str(folder)])
        if replay.returncode:
            print('Replay failed; flight data preserved. Rerun make_replay.py on this results directory.')
    raise SystemExit(0 if result['success'] else 1)


if __name__ == '__main__':
    main()
