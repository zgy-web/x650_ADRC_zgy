#!/usr/bin/env python3
"""Actual C++ executable / ROS / UDP encoding check; synthetic actuator only.

No Gazebo or PX4 is started. A private DDS domain and a loopback UDP decoder
exercise the output adapter before it is admitted into a flight lifecycle.
"""
import argparse,json,os,signal,socket,subprocess,time
from pathlib import Path


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--consumption-policy',choices=['strict_per_tick','bounded_wait_v1','latest_value_v1'],default='strict_per_tick')
    p.add_argument('--streaming-contract-proof',type=Path);a=p.parse_args()
    contract=None
    if a.consumption_policy=='latest_value_v1':
        if a.streaming_contract_proof is None:raise ValueError('Explicit completed streaming contract proof required')
        import hashlib
        data=a.streaming_contract_proof.read_bytes();proof=json.loads(data)
        if not proof.get('passed') or proof.get('kind')!='cpp_latest_value_contract_v1' or proof.get('consumption_policy')!='latest_value_v1' or proof.get('consumption_progress_budget_ms')!=40:
            raise ValueError('Streaming contract proof not passed/bound')
        contract={'path':str(a.streaming_contract_proof.resolve()),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()}
    a.output.mkdir(parents=True,exist_ok=False)
    os.environ['ROS_DOMAIN_ID']='209'
    import rclpy
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from ament_index_python.packages import get_package_prefix
    from flyarm_control_interfaces import msg
    from std_msgs.msg import Float64
    from pymavlink.dialects.v20 import common
    rclpy.init();n=rclpy.create_node('live_transport_probe')
    q=QoSProfile(depth=1000,reliability=ReliabilityPolicy.BEST_EFFORT)
    pubs={key:n.create_publisher(typ,'/flyarm/isolated/v1/'+key,1 if key in ('live_start','stop') else q)
          for key,typ in [('live_start',msg.LiveStart),('stop',msg.RuntimeStop),('source_status',msg.IngressStatus),
                         ('actuator_state',msg.ActuatorState),('accepted',msg.AcceptedCommand),('event',msg.GateEvent)]}
    counters=[100,200];velocity=[0.,0.];sim=[10.];receipt_sim=[10.,10.];receipts=[];ticks=[];sends=[]
    def joint(m,j):counters[j]+=1;velocity[j]=float(m.data);receipt_sim[j]=sim[0]
    subs=[n.create_subscription(Float64,'/arm/arm_joint_'+str(j+1)+'/cmd_vel',lambda m,j=j:joint(m,j),10) for j in range(2)]
    subs += [n.create_subscription(typ,'/flyarm/isolated/v1/'+key,lambda m,store=store:store.append(m),q)
             for typ,key,store in [(msg.ExecutionReceipt,'consumed',receipts),(msg.DispatchTick,'dispatch',ticks),(msg.ActuatorSend,'actuator_send',sends)]]
    # Model the separate ingress consumer as well as the observer. Discovery
    # must satisfy the same endpoint contract as the actual runtime.
    subs.append(n.create_subscription(msg.ExecutionReceipt,'/flyarm/isolated/v1/consumed',lambda m:None,q))
    peer=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);peer.bind(('127.0.0.1',0));peer.setblocking(False)
    inherited=socket.socket(socket.AF_INET,socket.SOCK_DGRAM);inherited.bind(('127.0.0.1',14550));inherited.setblocking(False)
    log=(a.output/'dispatcher.log').open('x')
    params={'expected_session_id':101,'expected_epoch':1,'configuration_sha256':"'"+'07'*32+"'",'simulation_live':True,
            'consumption_policy':a.consumption_policy,'consumption_progress_budget_ms':40,
            'sitl_udp_port':peer.getsockname()[1],'sitl_socket_fd':inherited.fileno(),'sitl_system_id':1,'sitl_component_id':1}
    binary=Path(get_package_prefix('flyarm_control_gateway'))/'lib/flyarm_control_gateway/live_dispatcher'
    if contract and hashlib.sha256(binary.read_bytes()).hexdigest()!=proof['dispatcher_sha256']:
        raise ValueError('Streaming proof binary mismatch')
    command=[str(binary),'--ros-args']
    for k,v in params.items():command+=['-p',k+':='+str(v).lower() if isinstance(v,bool) else k+':='+str(v)]
    child=subprocess.Popen(command,stdout=log,stderr=subprocess.STDOUT,pass_fds=(inherited.fileno(),),start_new_session=True)
    decoded=[];parser=common.MAVLink(None);source=0
    def pump(duration):
        nonlocal source
        end=time.monotonic()+duration
        while time.monotonic()<end:
            for _ in range(12):rclpy.spin_once(n,timeout_sec=0.)
            try:
                b,_=peer.recvfrom(4096)
                for m in parser.parse_buffer(b) or []:decoded.append(m.to_dict())
            except BlockingIOError:pass
            source+=1;now=time.monotonic_ns();sim[0]+=.001
            s=msg.IngressStatus();s.session_id=101;s.epoch=1;s.source_sequence=source
            s.received_monotonic_ns=now;s.source_issued_monotonic_ns=now;s.control_time_s=sim[0]
            pubs['source_status'].publish(s)
            ac=msg.ActuatorState();ac.session_id=101;ac.epoch=1;ac.observed_monotonic_ns=now;ac.sim_s=sim[0]
            ac.consumed_counter=counters;ac.consumed_velocity=velocity;ac.consumed_sim_s=receipt_sim
            pubs['actuator_state'].publish(ac)
            time.sleep(.001)
    import hashlib
    report={'passed':False,'scope':'software transport only; no physical plant',
            'consumption_policy':a.consumption_policy,'consumption_progress_budget_ms':40,
            'streaming_contract_proof':contract,
            'probe_source':{'path':str(Path(__file__).resolve()),'bytes':Path(__file__).stat().st_size,
                            'sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()},
            'dispatcher_binary':str(binary),'dispatcher_sha256':hashlib.sha256(binary.read_bytes()).hexdigest()}
    try:
        deadline=time.monotonic()+10
        while pubs['live_start'].get_subscription_count()<1 or len(ticks)<2:
            pump(.01)
            if child.poll() is not None or time.monotonic()>deadline:raise RuntimeError('dispatcher not ready')
        st=msg.LiveStart();st.session_id=101;st.epoch=1;st.configuration_sha256=[7]*32;st.ref_alt=10.7;st.yaw_ned=.3
        pubs['live_start'].publish(st);pump(.02)
        for seq in range(1,5):
            now=time.monotonic_ns();c=msg.ControlProposal();c.schema_version=1;c.session_id=101;c.epoch=1
            c.sequence=c.snapshot_sequence=seq;c.replay_only=False;c.configuration_sha256=[7]*32;c.status=0
            c.accepted_optimum=c.nonlinear_check_passed=True;c.position_reference_enu=[.4,-.2,2.3]
            c.velocity_feedforward_enu=[.1,-.1,.05];c.joint_velocity=[.01*seq,-.01*seq]
            cmd=msg.AcceptedCommand();cmd.issued_monotonic_ns=now;cmd.proposal=c;pubs['accepted'].publish(cmd)
            pump(.025)
        stop=msg.RuntimeStop();stop.session_id=101;stop.epoch=1;pubs['stop'].publish(stop);pump(.08)
        pvas=[x for x in decoded if x['mavpackettype']=='SET_POSITION_TARGET_LOCAL_NED']
        assert len(pvas)>=4
        expected={'coordinate_frame':1,'type_mask':2048,'target_system':1,'target_component':1,
                  'x':-.2,'y':.4,'z':8.4,'vx':-.1,'vy':.1,'vz':-.05,'afx':0.,'afy':0.,'afz':0.,'yaw':.3,'yaw_rate':0.}
        for m in pvas:
            for key,value in expected.items():assert abs(m[key]-value)<1e-6,(key,m[key],value)
        actual={int(x.command_sequence) for x in receipts if x.joint_consumed and x.pva_send_returned}
        assert actual=={1,2,3,4},actual
        zeros=[x for x in sends if x.stop_zero];assert len(zeros)==1
        assert any(x.dispatch_sequence==zeros[0].dispatch_sequence and x.joint_consumed for x in receipts)
        assert velocity==[0.,0.]
        assert not any(t.status==3 for t in ticks)
        active=[t for t in ticks if t.output_valid]
        assert len(active)>=4
        # New arrivals must never reset an already active output phase.
        assert all(b.scheduled_monotonic_ns-a.scheduled_monotonic_ns==20_000_000
                   for a,b in zip(active,active[1:]))
        assert all(t.command_age_ns<=50_000_000 for t in active)
        report.update(passed=True,decoded_packets=pvas,consumed_candidate_sequences=sorted(actual),
            zero_consumed=True,dispatch_count=len(ticks),send_count=len(sends),receipt_count=len(receipts),
            active_schedule_period_ns=20_000_000,
            active_intervals_ns=[b.actual_monotonic_ns-a.actual_monotonic_ns for a,b in zip(active,active[1:])],
            active_command_ages_ns=[t.command_age_ns for t in active])
    except BaseException as e:report['error']=repr(e);raise
    finally:
        if child.poll() is None:os.killpg(child.pid,signal.SIGINT)
        child.wait(timeout=10);log.close();peer.close();inherited.close();n.destroy_node();rclpy.shutdown()
        (a.output/'report.json').write_text(json.dumps(report,indent=2))

if __name__=='__main__':main()
