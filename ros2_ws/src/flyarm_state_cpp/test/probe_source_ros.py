"""Real ROS lifecycle probe with synthetic raw streams, no actuator or simulator.

Use an isolated ROS_DOMAIN_ID. This probes the actual compiled state_source and
message types; it is not physical control or estimator accuracy evidence.
"""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import struct
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude, VehicleAngularVelocity, EstimatorStatusFlags
from flyarm_control_interfaces.msg import ControlSnapshot, LiveStart, RuntimeStop, SourceBinding, SourceBindingAck


def identity(path):
    raw = path.read_bytes()
    return {'path': str(path.resolve()), 'bytes': len(raw), 'sha256': hashlib.sha256(raw).hexdigest()}


def packet(seq):
    at = (seq-1)*.005
    return {'sequence': seq, 'send_s': at, 'apply_s': at+.005,
            **{k+str(j): v for j in (1, 2) for k, v in [('qref', j*.1), ('dqref', j*.1+.2), ('tau', 0.), ('kp', 6.), ('kd', .25)]}}


def joint(seq):
    p, a = packet(seq), packet(seq-1)
    row = {'sim_s': p['send_s'], 'interval_begin_s': p['send_s']-.005,
           'last_send_sequence': seq, 'active_sequence': seq-1,
           'last_send_s': p['send_s'], 'last_send_apply_s': p['apply_s'],
           'active_send_s': a['send_s'], 'active_apply_s': a['apply_s']}
    for j in (1, 2):
        row.update({f'upper_sequence{j}': 1, f'u{j}': 0., f'receipt{j}_s': .985,
                    f'q{j}_physical': j*.1, f'q{j}_internal': j*.1, f'dq{j}_internal': j*.1+.2, f'r{j}': j*.1})
        for prefix, raw in [('last_', p), ('active_', a)]:
            for key in ('qref', 'dqref'):
                row[prefix+key+str(j)] = raw[key+str(j)]
    return row


def write_rows(path, rows, append=False):
    with path.open('a' if append else 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(rows[0]))
        if not append:
            writer.writeheader()
        writer.writerows(rows)


def run_case(exe, folder, case, session):
    folder.mkdir()
    state, packets = folder/'state.csv', folder/'packets.csv'
    write_rows(packets, [packet(s) for s in range(198, 202)])
    write_rows(state, [joint(s) for s in range(199, 202)])
    node = rclpy.create_node('source_software_probe_'+case)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    pubs = {name: node.create_publisher(cls, topic, qos) for name, cls, topic in [
        ('position', VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1'),
        ('attitude', VehicleAttitude, '/fmu/out/vehicle_attitude'),
        ('angular', VehicleAngularVelocity, '/fmu/out/vehicle_angular_velocity'),
        ('flags', EstimatorStatusFlags, '/fmu/out/estimator_status_flags'),
        ('clock', Clock, '/clock')]}
    binding = node.create_publisher(SourceBinding, '/flyarm/isolated/v1/source_binding', 1)
    start = node.create_publisher(LiveStart, '/flyarm/isolated/v1/live_start', 1)
    stop = node.create_publisher(RuntimeStop, '/flyarm/isolated/v1/stop', 1)
    snapshots, acks, stops = [], [], []
    subs = [node.create_subscription(ControlSnapshot, '/flyarm/isolated/v1/source', snapshots.append, qos),
            node.create_subscription(SourceBindingAck, '/flyarm/isolated/v1/source_binding_ack', acks.append, 1),
            node.create_subscription(RuntimeStop, '/flyarm/isolated/v1/stop', stops.append, 1)]
    params = dict(simulation_live=True, expected_session_id=session, expected_epoch=1, configuration_sha256='1'*64,
                  state_csv=str(state), packet_csv=str(packets), ready_file=str(folder/'ready'),
                  evidence_directory=str(folder/'evidence'), position_gains=[1., 1., 1.], bus_period_s=.005,
                  delay_s=.005, trajectory_kind='hold')
    command = [str(exe), '--ros-args']
    for name, value in params.items():
        command += ['-p', name+':='+json.dumps(value)]
    log = (folder/'node.log').open('w')
    process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)

    def publish_raw(at=1.):
        stamp = int(round(at*1e9))
        clock = Clock();clock.clock.sec, clock.clock.nanosec = divmod(stamp, 1_000_000_000);pubs['clock'].publish(clock)
        p = VehicleLocalPosition();p.timestamp = 1000000;p.timestamp_sample = 1000000;p.ref_timestamp = 100
        p.ref_lat=47.;p.ref_lon=8.;p.ref_alt=500.;p.xy_global=True;p.z_global=True
        p.xy_valid=p.z_valid=p.v_xy_valid=p.v_z_valid=p.heading_good_for_control=True
        p.x=1.;p.y=2.;p.z=3.;p.vx=4.;p.vy=5.;p.vz=6.;p.ax=7.;p.ay=8.;p.az=9.
        a=VehicleAttitude();a.timestamp=1000000;a.q=[1.,0.,0.,0.]
        w=VehicleAngularVelocity();w.timestamp=1000000;w.xyz=[1.,2.,3.]
        f=EstimatorStatusFlags();f.timestamp=1000000;f.cs_ev_pos=f.cs_ev_vel=f.cs_ev_hgt=True
        for key, value in [('position',p),('attitude',a),('angular',w),('flags',f)]:pubs[key].publish(value)

    def wait(predicate, timeout=5., at=1., raw=True):
        deadline=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>deadline:
                raise TimeoutError((case, process.poll(), (folder/'node.log').read_text()))
            if raw:
                publish_raw(at)
            rclpy.spin_once(node, timeout_sec=.005)

    def pause(seconds=.04, at=1.):
        deadline=time.monotonic()+seconds
        wait(lambda: time.monotonic()>=deadline, timeout=seconds+1, at=at)

    try:
        wait(lambda:(folder/'ready').exists() and start.get_subscription_count()>0 and binding.get_subscription_count()>0)
        # Endpoint matching may precede node-graph discovery in Fast DDS.
        wait(lambda: ('isolated_raw_state_source','/') in node.get_node_names_and_namespaces())
        publisher_topics = sorted(name for name, _ in node.get_publisher_names_and_types_by_node('isolated_raw_state_source', '/'))
        assert set(publisher_topics) <= {'/flyarm/isolated/v1/source', '/flyarm/isolated/v1/source_binding_ack',
                                         '/flyarm/isolated/v1/stop', '/parameter_events', '/rosout'}
        assert {'/flyarm/isolated/v1/source', '/flyarm/isolated/v1/source_binding_ack', '/flyarm/isolated/v1/stop'} <= set(publisher_topics)
        pause();assert not snapshots
        b=SourceBinding();b.session_id=session;b.epoch=1;b.configuration_sha256=[17]*32;b.binding_sequence=1
        b.px4_source_contract_commit='d6f12ad1c4f70ad3230afd7d86e971421e02fef4';b.frame_mode='raw_ev_world';b.target_frozen=True
        b.ref_lat=47.;b.ref_lon=8.;b.ref_alt=500.;b.ref_timestamp=100;b.xy_global=True;b.z_global=True
        b.position_gains=[1.,1.,1.];b.bus_period_s=.005;b.delay_s=.005
        live=LiveStart();live.session_id=session;live.epoch=1;live.configuration_sha256=[17]*32;live.origin_s=1.
        live.anchor_rotation_row_major=[1.,0.,0.,0.,1.,0.,0.,0.,1.];live.anchor_q=[.1,.2];live.target_position_enu=[0.,0.,2.]
        live.history_count=1;live.history_time_s[0]=1.;live.ref_alt=500.
        if case=='unbound_start':
            live.issued_monotonic_ns=time.monotonic_ns();start.publish(live)
        else:
            if case=='bad_binding_hash':b.configuration_sha256[0]=0
            b.issued_monotonic_ns=time.monotonic_ns();binding.publish(b)
            wait(lambda: bool(acks) or process.poll() is not None)
            assert acks and acks[-1].binding_issued_monotonic_ns==b.issued_monotonic_ns
            if case=='bad_binding_hash':assert not acks[-1].accepted
            else:
                assert acks[-1].accepted and acks[-1].rejection_code==0
                pause();assert not snapshots
                if case=='duplicate_binding':
                    b.issued_monotonic_ns=time.monotonic_ns();binding.publish(b)
                else:
                    live.issued_monotonic_ns=time.monotonic_ns();start.publish(live)
                    pause(.02)
                    write_rows(packets,[packet(202)],append=True);write_rows(state,[joint(202)],append=True)
                    wait(lambda: bool(snapshots), at=1.006)
                    s=snapshots[0]
                    assert s.replay_only is False and s.session_id==session and s.epoch==1
                    assert list(s.position_enu)==[2.,1.,497.] and list(s.acceleration_enu)==[8.,7.,-9.]
                    assert list(s.q)==[.1,.2] and s.joint_time_s==packet(202)['send_s'] and s.control_time_s==1.006
                    m=RuntimeStop();m.session_id=session;m.epoch=1;stop.publish(m)
        wait(lambda: process.poll() is not None, raw=False)
        expected=0 if case=='normal' else 1
        assert process.returncode==expected,(case,process.returncode)
        final=json.loads((folder/'evidence/source_final.json').read_text())
        assert bool(final['failure']) is (expected==1)
        if expected==1:assert final['snapshots']==0 and not snapshots
        else:assert final['ready'] and final['bound'] and final['started'] and final['stopped'] and final['snapshots']>=1
        wire=(folder/'evidence/source_wire.cdr').read_bytes()
        assert wire[:8]==b'FSCdr001'
        at=8;original_handoffs=[];wire_records=0
        while at<len(wire):
            kind,wall,sequence,sim,size=struct.unpack_from('<IQQdI',wire,at);at+=32
            raw=wire[at:at+size];assert len(raw)==size;at+=size;wire_records+=1
            if kind in (6,9,10):
                cls={6:LiveStart,9:SourceBinding,10:SourceBindingAck}[kind]
                original_handoffs.append((kind,deserialize_message(raw,cls)))
        assert at==len(wire) and wire_records==final['wire_records']
        if case=='bad_binding_hash':
            assert any(kind==9 and message.configuration_sha256[0]==0 for kind,message in original_handoffs)
            assert any(kind==10 and not message.accepted for kind,message in original_handoffs)
        if case=='duplicate_binding':
            assert sum(kind==9 for kind,_ in original_handoffs)==2
            assert any(kind==10 and not message.accepted for kind,message in original_handoffs)
        if case=='unbound_start':assert sum(kind==6 for kind,_ in original_handoffs)==1
        return dict(case=case,passed=True,exit=process.returncode,snapshots=len(snapshots),acks=len(acks),
                    actual_node_publishers=publisher_topics,no_actuator_publishers=True,
                    original_rejected_handoff_cdr_verified=True,
                    final=identity(folder/'evidence/source_final.json'),wire=identity(folder/'evidence/source_wire.cdr'),
                    scope='real ROS transport of synthetic raw inputs; no actuator node or simulator')
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=5)
            except subprocess.TimeoutExpired:process.kill();process.wait()
        log.close();node.destroy_node()


def main():
    p=argparse.ArgumentParser();p.add_argument('--executable',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    args=p.parse_args();assert os.environ.get('ROS_DOMAIN_ID')=='197','Private software domain 197 required'
    args.output.mkdir(parents=True,exist_ok=False);rclpy.init()
    try:cases=[run_case(args.executable,args.output/name,name,1000+i) for i,name in enumerate(('normal','bad_binding_hash','duplicate_binding','unbound_start'))]
    finally:rclpy.shutdown()
    report=dict(passed=all(c['passed'] for c in cases),cases=cases,executable=identity(args.executable),script=identity(Path(__file__)),
                scope='No actuator publisher/subscriber/plant; ROS source lifecycle only')
    (args.output/'report.json').write_text(json.dumps(report,indent=2));print(json.dumps(report,indent=2))


if __name__=='__main__':main()
