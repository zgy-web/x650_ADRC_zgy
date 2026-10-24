#!/usr/bin/env python3
"""Latest-value real ROS/live_dispatcher, synthetic feedback, loopback UDP.

No simulator, aircraft, arm, worker or ingress is started. Every case owns its
child and sockets, uses a private DDS domain, and keeps its failed evidence.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time


def run_case(root, case, domain):
    os.environ['ROS_DOMAIN_ID'] = str(domain)
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from ament_index_python.packages import get_package_prefix
    from flyarm_control_interfaces import msg
    from std_msgs.msg import Float64
    from pymavlink.dialects.v20 import common

    out = root / case
    out.mkdir()
    report = {'passed': False, 'case': case, 'name':case,'consumption_policy':'latest_value_v1',
              'consumption_progress_budget_ms':40,'physical_acceptance':False,
              'scope': 'synthetic real ROS and loopback only; no physical validation'}
    rclpy.init()
    node = rclpy.create_node('streaming_transport_probe')
    qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
    pubs = {key: node.create_publisher(typ, '/flyarm/isolated/v1/' + key,
                                     1 if key in ('live_start', 'stop') else qos)
            for key, typ in [('live_start', msg.LiveStart), ('stop', msg.RuntimeStop),
                             ('source_status', msg.IngressStatus), ('actuator_state', msg.ActuatorState),
                             ('accepted', msg.AcceptedCommand), ('event', msg.GateEvent)]}
    sends, receipts, ticks, joint_values, decoded = [], [], [], [[], []], []
    subs = [node.create_subscription(Float64, '/arm/arm_joint_' + str(j+1) + '/cmd_vel',
                                     lambda m, j=j: joint_values[j].append(float(m.data)), 10) for j in range(2)]
    for typ, topic, store in [(msg.ActuatorSend, 'actuator_send', sends),
                              (msg.ExecutionReceipt, 'consumed', receipts), (msg.DispatchTick, 'dispatch', ticks)]:
        subs.append(node.create_subscription(typ, '/flyarm/isolated/v1/' + topic, lambda m, store=store: store.append(m), qos))
    subs.append(node.create_subscription(msg.ExecutionReceipt, '/flyarm/isolated/v1/consumed', lambda m: None, qos))
    peer = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    peer.bind(('127.0.0.1', 0)); peer.setblocking(False)
    inherited = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    inherited.bind(('127.0.0.1', 14550)); inherited.setblocking(False)
    binary = Path(get_package_prefix('flyarm_control_gateway')) / 'lib/flyarm_control_gateway/live_dispatcher'
    report.update(binary=str(binary), binary_sha256=hashlib.sha256(binary.read_bytes()).hexdigest())
    params = {'expected_session_id': 101, 'expected_epoch': 1, 'configuration_sha256': "'" + '07'*32 + "'",
              'consumption_policy':'latest_value_v1','consumption_progress_budget_ms':40,
              'simulation_live': True, 'sitl_udp_port': peer.getsockname()[1], 'sitl_socket_fd': inherited.fileno(),
              'sitl_system_id': 1, 'sitl_component_id': 1, 'ready_file': str(out / 'ready.txt')}
    command = [str(binary), '--ros-args']
    for key, value in params.items():
        command += ['-p', key + ':=' + (str(value).lower() if isinstance(value, bool) else str(value))]
    log = (out / 'dispatcher.log').open('x')
    child = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                             pass_fds=(inherited.fileno(),), start_new_session=True)
    parser = common.MAVLink(None)
    source = 0
    base = time.monotonic_ns()
    first = None
    command2 = False
    stop_sent = False
    start_sent = False
    source_frozen = False
    state_frozen = False
    observed_ack_ns = None
    old_observation = None

    def stop():
        nonlocal stop_sent
        m = msg.RuntimeStop(); m.session_id = 101; m.epoch = 1
        pubs['stop'].publish(m); stop_sent = True

    def candidate(seq):
        c = msg.ControlProposal(); c.schema_version = 1; c.session_id = 101; c.epoch = 1
        c.sequence = c.snapshot_sequence = seq; c.configuration_sha256 = [7]*32; c.replay_only = False
        c.accepted_optimum = c.nonlinear_check_passed = True
        c.position_reference_enu = [.4, -.2, 2.3]; c.velocity_feedforward_enu = [.1, -.1, .05]
        c.joint_velocity = [.01*seq, -.01*seq]
        a = msg.AcceptedCommand(); a.issued_monotonic_ns = time.monotonic_ns(); a.proposal = c
        pubs['accepted'].publish(a)

    def pump():
        nonlocal source, first, command2, source_frozen, state_frozen, observed_ack_ns, stop_sent, old_observation
        for _ in range(16): rclpy.spin_once(node, timeout_sec=0.)
        while True:
            try:
                data, _ = peer.recvfrom(4096)
                decoded.extend(m.to_dict() for m in (parser.parse_buffer(data) or []))
            except BlockingIOError: break
        now = time.monotonic_ns()
        normal = [s for s in sends if not s.stop_zero]
        if normal and first is None: first = normal[0]
        age = now - first.sent_monotonic_ns if first else -1
        if first and not command2 and age >= 8_000_000:
            candidate(2); command2 = True
        if first and case == 'source_stale': source_frozen = True
        if first and case == 'missing_state': state_frozen = True
        if first and case == 'stop_skips_unobserved' and age >= 21_000_000 and not stop_sent: stop()
        if first and case == 'sigint_active' and age >= 21_000_000 and not stop_sent:
            os.killpg(child.pid,signal.SIGINT);stop_sent=True
        successes=('late_observer_23ms','late_observer_30ms','counter_jump','mixed_joint_batches','duplicate_and_old_observations')
        if case in successes and any(r.command_sequence==2 and r.joint_consumed for r in receipts) and not stop_sent:
            if case!='duplicate_and_old_observations' or age>=31_000_000:stop()
        if not source_frozen:
            source += 1
            s = msg.IngressStatus(); s.session_id = 101; s.epoch = 1; s.source_sequence = source
            s.received_monotonic_ns = s.source_issued_monotonic_ns = now
            s.control_time_s = 10. + (now-base)*1e-9; pubs['source_status'].publish(s)
        if not state_frozen:
            s = msg.ActuatorState(); s.session_id = 101; s.epoch = 1; s.observed_monotonic_ns = now
            s.sim_s = 10. + (now-base)*1e-9
            s.consumed_counter = [100, 200]; s.consumed_velocity = [0., 0.]; s.consumed_sim_s = [10.,10.]
            threshold=30_000_000 if case=='late_observer_30ms' else 23_000_000
            # The source-stale case keeps exact actuator progress healthy so
            # that a separate 40 ms progress expiry cannot mask its source fault.
            no_progress=('missing_progress','missing_state','sigint_active','stop_skips_unobserved')
            if first and case not in no_progress and age>=threshold:
                if observed_ack_ns is None: observed_ack_ns = now
                acknowledged={int(r.dispatch_sequence) for r in receipts}
                consumed = next((x for x in normal if int(x.dispatch_sequence) not in acknowledged), normal[-1])
                chosen=[consumed,consumed]
                if case in ('counter_jump','wrong_counter'):chosen=[normal[-1],normal[-1]]
                if case=='mixed_joint_batches':chosen=[normal[-1],first if age<27_000_000 else normal[-1]]
                if case=='one_joint_no_progress':chosen=[normal[-1],None]
                for j,record in enumerate(chosen):
                    if record is not None:
                        s.consumed_counter[j]=int(record.expected_counter[j]);s.consumed_velocity[j]=float(record.joint_velocity[j]);s.consumed_sim_s[j]=float(record.sim_s)
                if case=='wrong_counter':s.consumed_counter=[int(x)+1 for x in normal[-1].expected_counter]
                if case=='wrong_velocity':s.consumed_velocity[0]+=0.1
                if case=='counter_regression' and age>=28_000_000:
                    s.consumed_counter=[100,200];s.consumed_velocity=[0.,0.];s.consumed_sim_s=[10.,10.]
                if old_observation is None:old_observation=s
            if sends and sends[-1].stop_zero and case in successes+('stop_skips_unobserved',) and age>=28_000_000:
                s.consumed_counter=[int(x) for x in sends[-1].expected_counter];s.consumed_velocity=[0.,0.]
                s.consumed_sim_s=[float(sends[-1].sim_s)]*2
            if case=='duplicate_and_old_observations' and age>=28_000_000 and old_observation is not None:
                pubs['actuator_state'].publish(old_observation)
            pubs['actuator_state'].publish(s)
            if case=='duplicate_and_old_observations' and age>=28_000_000:pubs['actuator_state'].publish(s)
        time.sleep(.0005)

    try:
        until = time.monotonic()+12
        while not (out/'ready.txt').exists():
            pump()
            if child.poll() is not None or time.monotonic()>until: raise RuntimeError('DDS readiness timeout')
        st = msg.LiveStart(); st.session_id=101; st.epoch=1; st.configuration_sha256=[7]*32
        st.ref_alt=10.7; st.yaw_ned=.3; pubs['live_start'].publish(st); start_sent=True
        until=time.monotonic()+.025
        while time.monotonic()<until: pump()
        candidate(1)
        until=time.monotonic()+.18
        while time.monotonic()<until: pump()
        normal=[s for s in sends if not s.stop_zero]; zeros=[s for s in sends if s.stop_zero]
        faults=[t for t in ticks if t.status==3]
        assert first is not None and len(zeros)==(0 if case=='sigint_active' else 1), (len(normal),len(zeros))
        if zeros:assert sends[-1].stop_zero and sends[-1].dispatch_sequence==zeros[0].dispatch_sequence
        assert all(len(v)==len(sends) for v in joint_values), [len(v) for v in joint_values]
        assert all(b.expected_counter[j]==a.expected_counter[j]+1 for a,b in zip(sends,sends[1:]) for j in range(2))
        successes=('late_observer_23ms','late_observer_30ms','counter_jump','mixed_joint_batches','duplicate_and_old_observations','stop_skips_unobserved')
        if case in successes:
            assert 2<=len(normal)<=3 and not faults, (len(normal),[(t.reason,t.sequence) for t in faults])
            if observed_ack_ns is not None:assert normal[1].sent_monotonic_ns<observed_ack_ns
            valid=[t for t in ticks if t.output_valid]
            assert len(valid)==len(normal) and all(b.scheduled_monotonic_ns-a.scheduled_monotonic_ns==20_000_000 for a,b in zip(valid,valid[1:]))
            assert all(0<b.sent_monotonic_ns-a.sent_monotonic_ns<=40_000_000 for a,b in zip(normal,normal[1:]))
            assert all(t.command_age_ns<=50_000_000 for t in valid)
            if case in ('counter_jump','mixed_joint_batches','stop_skips_unobserved'):
                assert not any(r.dispatch_sequence==first.dispatch_sequence for r in receipts)
            if case!='stop_skips_unobserved':assert any(r.dispatch_sequence==normal[1].dispatch_sequence and r.joint_consumed for r in receipts)
            if case.startswith('late_observer_'):assert any(r.dispatch_sequence==normal[0].dispatch_sequence and r.joint_consumed for r in receipts)
            assert any(r.dispatch_sequence==zeros[0].dispatch_sequence and r.joint_consumed for r in receipts)
        else:
            assert 2<=len(normal)<=3,len(normal)
            allowed={'wrong_counter':{9},'wrong_velocity':{9},'counter_regression':{9},'missing_state':{8},'source_stale':{2},'missing_progress':{12},'one_joint_no_progress':{12}}
            if case=='sigint_active':assert child.poll()==0 and not faults
            else:assert faults and int(faults[0].reason) in allowed[case],[(t.reason,t.sequence) for t in faults]
        pvas=[m for m in decoded if m['mavpackettype']=='SET_POSITION_TARGET_LOCAL_NED']
        assert len(pvas)==len(normal), (len(pvas),len(normal))
        report.update(passed=True, normal_send_count=len(normal), stop_count=len(zeros),
                      normal_send_ns=[int(s.sent_monotonic_ns) for s in normal],
                      first_fault_reason=int(faults[0].reason) if faults else None,
                      emitted_ack_observation_ns=observed_ack_ns, pva_count=len(pvas))
    except BaseException as e:
        report['error']=repr(e)
        raise
    finally:
        if child.poll() is None: os.killpg(child.pid, signal.SIGINT)
        child.wait(timeout=10); log.close()
        report['returncode']=child.returncode
        from rosidl_runtime_py.convert import message_to_ordereddict
        for name, values in [('sends',sends),('receipts',receipts),('ticks',ticks)]:
            (out/(name+'.json')).write_text(json.dumps([message_to_ordereddict(v) for v in values],default=list))
        trace=out/'actuator_ledger.json'
        if trace.exists():
            evidence=json.loads(trace.read_text());report['ledger_evidence']=evidence
            if not evidence['complete'] or evidence['overflow']: report['passed']=False
            if not evidence['have_epoch_baseline'] or not (out/'actuator_epoch_state.cdr').is_file():report['passed']=False
            if case=='duplicate_and_old_observations' and evidence['ignored_old_states']<1:report['passed']=False
            if case=='counter_jump' and evidence['rows'][0]['joint_status']!=[2,2]:report['passed']=False
            if case=='mixed_joint_batches' and (evidence['rows'][0]['joint_status']!=[2,1] or evidence['rows'][0]['paired']):report['passed']=False
        else: report['passed']=False;report['trace_error']='missing wait evidence'
        (out/'report.json').write_text(json.dumps(report,indent=2))
        peer.close(); inherited.close(); node.destroy_node(); rclpy.shutdown()
    return report


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--domain-id',type=int,default=209);args=p.parse_args()
    args.output.mkdir(parents=True,exist_ok=False)
    source_root=Path(__file__).resolve().parents[1]
    source_paths=[Path(__file__).resolve()]+[p for p in (source_root/'ros2_ws/src/flyarm_control_gateway').rglob('*') if p.is_file()]
    def identity(p):return {'path':str(p),'bytes':p.stat().st_size,'sha256':hashlib.sha256(p.read_bytes()).hexdigest()}
    sources_before=[identity(p) for p in sorted(source_paths)]
    reports=[]
    try:
        for case in ('late_observer_23ms','late_observer_30ms','counter_jump','mixed_joint_batches','duplicate_and_old_observations',
                     'wrong_counter','wrong_velocity','counter_regression','missing_progress','one_joint_no_progress',
                     'missing_state','source_stale','stop_skips_unobserved','sigint_active'):
            reports.append(run_case(args.output,case,args.domain_id))
            if not reports[-1]['passed']: raise RuntimeError('case failed: '+case)
    finally:
        binary=reports[0].get('binary') if reports else None
        sources_after=[identity(p) for p in sorted(source_paths)]
        artifacts=[identity(p) for p in sorted(args.output.glob('*/*')) if p.is_file()]
        binaries_equal=bool(binary) and all(r['binary']==binary and r['binary_sha256']==reports[0]['binary_sha256'] for r in reports) and hashlib.sha256(Path(binary).read_bytes()).hexdigest()==reports[0]['binary_sha256']
        (args.output/'report.json').write_text(json.dumps({'kind':'cpp_latest_value_contract_v1','passed':len(reports)==14 and all(r['passed'] for r in reports) and sources_after==sources_before and binaries_equal,
            'physical_acceptance':False,'consumption_policy':'latest_value_v1','consumption_progress_budget_ms':40,
            'dispatcher_binary':binary,'dispatcher_sha256':reports[0].get('binary_sha256') if reports else None,
            'sources':sources_before,'sources_unchanged':sources_after==sources_before,'binaries_equal':binaries_equal,'artifacts':artifacts,
            'scope':'software real ROS synthetic consumption only; no physical plant','cases':[dict(r,name=r['case']) for r in reports]},indent=2))


if __name__=='__main__':main()
