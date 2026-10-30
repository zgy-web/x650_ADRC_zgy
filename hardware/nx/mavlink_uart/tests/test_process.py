"""PTY-only process checks; the production executable has no test device flag."""
import importlib.util, json, math, os, pty, select, subprocess, sys, time, threading

spec=importlib.util.spec_from_file_location('bound_mavlink',sys.argv[2])
m=importlib.util.module_from_spec(spec);spec.loader.exec_module(m)


def run_case(name):
    master,slave=pty.openpty();path=os.ttyname(slave)
    env=os.environ.copy();env['FLYARM_UART_TEST_PTY']=path
    mode='observe' if name=='observe' else ('stream_stop' if name in ('stream_stop','armed_stream_stop') else 'manual_exit')
    cmd=[sys.argv[1],'--baud','921600','--seconds','5','--case',mode,'--power-disconnected']
    if name=='configure':cmd.append('--configure-bench-streams')
    child=subprocess.Popen(cmd,stdout=subprocess.PIPE,stderr=subprocess.PIPE,env=env)
    output_chunks=[];error_chunks=[];ready_at=[None]
    def stdout():
        for line in child.stdout:
            output_chunks.append(line)
            try:event=json.loads(line)
            except ValueError:continue
            if event.get('event')=='ready':ready_at[0]=time.monotonic()
    readers=[threading.Thread(target=stdout,daemon=True),
             threading.Thread(target=lambda:error_chunks.append(child.stderr.read()),daemon=True)]
    for reader in readers:reader.start()
    tx=m.MAVLink(None,srcSystem=1,srcComponent=1);rx=m.MAVLink(None);rx.robust_parsing=True
    start=time.monotonic();next_att=next_rc=next_hb=next_mode=start
    sent=[];corrupted=False
    def emit(message):
        nonlocal tx
        data=message.pack(tx);tx.seq=(tx.seq+1)%256;os.write(master,data)
    try:
        while child.poll() is None and time.monotonic()-start<8:
            now=time.monotonic();elapsed=now-start;stamp=1000+int(elapsed*1000)
            # Operator arming happens only after observed READY, not an assumed
            # process-start delay. No state is pre-armed at the initial gate.
            action_time=(now-ready_at[0] if ready_at[0] is not None else -1) if name.startswith('armed_') else elapsed
            in_mode=action_time>=.65 and not (name in ('manual_exit','configure','armed_manual_exit') and action_time>=1.5)
            armed=name.startswith('armed_') and action_time>=.3
            if now>=next_att:
                sample=stamp
                if name=='repeated_stamp' and elapsed>=1.:sample=1900
                if name=='regression' and elapsed>=1.:sample=10
                # Duplicate test switches to a fixed timestamp at an exact saved stamp below.
                if name=='repeated_stamp':
                    if elapsed<.9:run_case.frozen_stamp=sample
                    else:sample=run_case.frozen_stamp
                a=m.MAVLink_attitude_quaternion_message(sample,1,0,0,0,0,0,0)
                if name=='bad_crc' and elapsed>=1. and not corrupted:
                    b=bytearray(a.pack(tx));b[-1]^=0x80;os.write(master,b);corrupted=True
                else:emit(a)
                next_att=now+.019
            if now>=next_rc:
                sa=2000 if in_mode else 1000
                throttle=1500 if name=='throttle' and elapsed>=1. else 1000
                if name!='missing_rc' or elapsed<1.:
                    channels=[1500,1500,throttle,1500,2000 if armed else 1000,sa,1500]+[1500]*11
                    emit(m.MAVLink_rc_channels_message(stamp,16,*channels,100))
                next_rc=now+.045
            if now>=next_hb:
                emit(m.MAVLink_heartbeat_message(m.MAV_TYPE_QUADROTOR,m.MAV_AUTOPILOT_PX4,128 if armed else 0,(6 if in_mode else 2)<<16,m.MAV_STATE_STANDBY,3));next_hb=now+.5
            if now>=next_mode:
                emit(m.MAVLink_current_mode_message(0,(6 if in_mode else 2)<<16,(6 if in_mode else 2)<<16));next_mode=now+.09
            if select.select([master],[],[],.002)[0]:
                raw=os.read(master,8192)
                for msg in rx.parse_buffer(raw) or []:
                    if msg.get_type()=='BAD_DATA':continue
                    sent.append(msg)
                    if msg.get_type()=='COMMAND_LONG':
                        assert msg.command==511 and msg.target_system==1 and msg.target_component==1
                        emit(m.MAVLink_command_ack_message(511,0))
        child.wait(timeout=3)
        for reader in readers:reader.join(timeout=3)
        assert all(not reader.is_alive() for reader in readers),'output drain incomplete'
        output=b''.join(output_chunks);err=b''.join(error_chunks)
        assert child.returncode==0,(name,err.decode())
        rows=[json.loads(line) for line in output.splitlines()]
        result=rows[-1];assert result['event']=='result' and result['settings_restored'] and result['fd_closed']
        assert result['error']=='',result['error']
        assert result['position_setpoints']==result['arm_commands']==result['mode_commands']==0
        sps=[x for x in sent if x.get_type()=='SET_ATTITUDE_TARGET']
        assert all(x.thrust==0 and x.type_mask==7 and x.q==[1.,0.,0.,0.] for x in sps)
        assert all(x.get_type() in ('HEARTBEAT','TIMESYNC','COMMAND_LONG','SET_ATTITUDE_TARGET') for x in sent)
        expected={'manual_exit':'manual_or_px4_exit','configure':'manual_or_px4_exit',
          'stream_stop':'planned_stream_stop','missing_rc':'rc_unavailable',
          'repeated_stamp':'attitude_unavailable','regression':'invalid_or_regressed_state',
          'throttle':'rc_mapping_or_throttle','bad_crc':'post_sync_crc_error',
          'armed_manual_exit':'manual_or_px4_exit','armed_stream_stop':'planned_stream_stop'}
        if name=='observe':assert not sent and result['setpoints']==0
        else:
            assert len(sps)>5 and result['stop_reason']==expected[name],(name,result['stop_reason'],len(sps))
            samples=[r for r in result['records'] if r['tx'] and bytes.fromhex(r['hex'])[7:10]==b'R\x00\x00']
            assert result['records'][-1]['ns']-samples[-1]['ns']>500_000_000,'must remain stopped while RX continues'
        if name=='configure':assert sum(x.get_type()=='COMMAND_LONG' for x in sent)==8
        return dict(case=name,passed=True,setpoints=result['setpoints'],reason=result['stop_reason'])
    finally:
        if child.poll() is None:child.terminate();child.wait(timeout=3)
        os.close(master);os.close(slave)


results=[]
for name in ('observe','manual_exit','stream_stop','configure','missing_rc','repeated_stamp','regression','throttle','bad_crc','armed_manual_exit','armed_stream_stop'):
    results.append(run_case(name));print(json.dumps(results[-1]),flush=True)
print(json.dumps(dict(passed=True,cases=results,hardware_accessed=False)))
