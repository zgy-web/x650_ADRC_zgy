"""Decode the tested UART executable's capture on PC with its pinned dialect.

Reports measured arrival rates and gaps, not physical-control acceptance.
No SSH, serial, network, parameter, or actuator access.
"""
import argparse
from collections import defaultdict
import hashlib
import importlib.util
import json
from pathlib import Path


def summarize(run,generated):
    manifest=json.loads((generated/'manifest.json').read_text())
    lock=json.loads((Path(__file__).resolve().parents[1]/'configs/hardware/nx_mavlink_dialect_lock.json').read_text())
    dialect=generated/'dialect.py'
    if manifest['lock']!=lock or hashlib.sha256(dialect.read_bytes()).hexdigest()!=manifest['files']['dialect.py']:
        raise ValueError('Capture dialect identity differs')
    spec=importlib.util.spec_from_file_location('flyarm_capture_dialect',dialect)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    events=[json.loads(x) for x in (run/'transport.jsonl').read_bytes().splitlines()]
    def single(name):
        selected=[x for x in events if x.get('event')==name]
        if len(selected)!=1:raise ValueError('Missing/ambiguous event '+name)
        return selected[0]
    result=single('result');exit_result=single('exit');cleanup=single('cleanup')
    if (not result['complete'] or result['error'] or not result['settings_restored'] or not result['fd_closed']
        or exit_result['returncode'] or exit_result['kernel_check_returncode'] or exit_result['kernel_faults']
        or exit_result['kernel_check_stderr'].strip() or not cleanup['memfd_closed'] or not cleanup['process_exited']):
        raise ValueError('Capture did not finish cleanly')
    parser=module.MAVLink(None);groups=defaultdict(list);tx_bytes=0;rx_frame_bytes=0
    heartbeat_armed=[];mode_main=[];attitude_max_norm_error=0.;commands=[];last_ns=0
    for record in result['records']:
        if record['ns']<last_ns:raise ValueError('Arrival timestamps regressed')
        last_ns=record['ns'];raw=bytes.fromhex(record['hex']);messages=parser.parse_buffer(raw)
        if not messages or len(messages)!=1 or messages[0].get_type()=='BAD_DATA':raise ValueError('Invalid serialized frame')
        m=messages[0];direction='tx' if record['tx'] else 'rx'
        stamp=getattr(m,'time_boot_ms',None)
        groups[(direction,m.get_srcSystem(),m.get_srcComponent(),m.get_msgId(),m.get_type())].append((record['ns'],stamp))
        if record['tx']:
            tx_bytes+=len(raw)
            if m.get_type()=='COMMAND_LONG':commands.append(dict(command=m.command,param1=m.param1,param2=m.param2))
        else:
            rx_frame_bytes+=len(raw)
            if m.get_srcSystem()==1 and m.get_srcComponent()==1:
                if m.get_type()=='HEARTBEAT':heartbeat_armed.append(bool(m.base_mode&128))
                if m.get_type()=='CURRENT_MODE':mode_main.append((m.custom_mode>>16)&255)
                if m.get_type()=='ATTITUDE_QUATERNION':
                    attitude_max_norm_error=max(attitude_max_norm_error,abs(sum(x*x for x in (m.q1,m.q2,m.q3,m.q4))-1))
    rows=[]
    for (direction,sysid,compid,mid,name),samples in sorted(groups.items()):
        times=[s[0] for s in samples];stamps=[s[1] for s in samples if s[1] is not None]
        span=(times[-1]-times[0])/1e9
        rows.append(dict(direction=direction,system=sysid,component=compid,id=mid,name=name,count=len(times),
            measured_hz=(len(times)-1)/span if span>0 else None,
            max_arrival_gap_ms=max(((b-a)/1e6 for a,b in zip(times,times[1:])),default=None),
            duplicate_source_stamps=sum(a==b for a,b in zip(stamps,stamps[1:])),
            regressed_source_stamps=sum(b<a for a,b in zip(stamps,stamps[1:]))))
    if tx_bytes!=result['tx_bytes']:raise ValueError('Transmitted byte count differs')
    return dict(kind='flyarm.mavlink_uart_capture_summary.v1',capture_complete=True,
        physical_acceptance=False,elapsed_s=result['elapsed_s'],messages=rows,
        rx_bytes=result['rx_bytes'],decoded_rx_frame_bytes=rx_frame_bytes,tx_bytes=tx_bytes,
        crc_errors_before_sync=result['crc_errors_before_sync'],crc_errors_after_sync=result['crc_errors_after_sync'],
        any_heartbeat_armed=any(heartbeat_armed) if heartbeat_armed else None,observed_mode_main=sorted(set(mode_main)),
        attitude_max_norm_squared_error=attitude_max_norm_error,commands=commands,
        setpoints=result['setpoints'],mpc_called=result['mpc_called'],seen_offboard=result['seen_offboard'],
        stop_reason=result['stop_reason'],max_send_gap_ms=result['max_send_gap_ms'],
        tested_binary_sha256=exit_result['sha256'],kernel_faults=exit_result['kernel_faults'],
        nx_persistent_files_created=exit_result['persistent_files_created'],cleanup=cleanup,
        evidence_sha256=hashlib.sha256((run/'transport.jsonl').read_bytes()).hexdigest())


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('run','generated','output'):p.add_argument('--'+name,type=Path,required=True)
    a=p.parse_args();r=summarize(a.run,a.generated)
    with a.output.open('x',encoding='utf-8') as f:json.dump(r,f,indent=2,allow_nan=False)
    print(json.dumps(r,indent=2))
