"""Observe NX UART in RAM; all captures and decoding stay on the PC.

The optional transmission is one fixed MAVLink parameter READ, never a write,
setpoint, heartbeat, arm request, or mode request. UART settings are restored.
"""
import argparse
import base64
import collections
import datetime
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import sys

from pymavlink.dialects.v20 import common


READ_PACKET_HEX = 'fd10000000f5bf140000ffff01014d41565f305f434f4e4649477c78'

REMOTE = r'''
import base64,fcntl,json,os,pathlib,select,subprocess,sys,termios,time,tty
p=json.load(sys.stdin)
if p['device']!='/dev/ttyTHS1' or p['baud']!=57600 or not 8<=p['duration']<=60:
 raise ValueError('Unsupported bounded UART probe')
packet=base64.b64decode(p.get('read_packet',''))
if packet and packet.hex()!='fd10000000f5bf140000ffff01014d41565f305f434f4e4649477c78':
 raise ValueError('Unexpected parameter-read packet')
tree=pathlib.Path('/sys/firmware/devicetree/base/bus@0')
mapping=(tree/'dma-controller@2600000/iommus').read_bytes()
if (tree/'serial@3100000/iommus').read_bytes()!=mapping:
 raise ValueError('UART DMA mapping is not the verified GPCDMA mapping')
users=subprocess.run(['/usr/bin/fuser',p['device']],capture_output=True,timeout=5)
if users.returncode!=1 or users.stdout.strip():raise RuntimeError('UART is in use or ownership check failed')
boot_id=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip()
start_wall=int(time.time());start=time.monotonic()
fd=os.open(p['device'],os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK)
original=None;exclusive=False;chunks=[];sent=[];error=None;restored=False
try:
 fcntl.flock(fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
 fcntl.ioctl(fd,termios.TIOCEXCL);exclusive=True
 original=termios.tcgetattr(fd)
 tty.setraw(fd,termios.TCSANOW)
 a=termios.tcgetattr(fd)
 a[2]=(a[2]&~(termios.CSIZE|termios.PARENB|termios.CSTOPB|termios.CRTSCTS))|termios.CS8|termios.CLOCAL|termios.CREAD
 a[4]=a[5]=termios.B57600
 termios.tcsetattr(fd,termios.TCSANOW,a)
 start=time.monotonic();sent_once=False;total=0
 while time.monotonic()-start<p['duration']:
  if packet and not sent_once and time.monotonic()-start>=5:
   n=os.write(fd,packet)
   sent.append({'elapsed_s':time.monotonic()-start,'bytes':n})
   if n!=len(packet):raise RuntimeError('Partial read-request transmission')
   sent_once=True
  ready,_,_=select.select([fd],[],[],.1)
  if not ready:continue
  try:data=os.read(fd,4096)
  except BlockingIOError:continue
  if not data:continue
  total+=len(data)
  if total>1000000:raise RuntimeError('Capture byte limit')
  chunks.append({'elapsed_s':time.monotonic()-start,'data':base64.b64encode(data).decode()})
 elapsed=time.monotonic()-start
except Exception as e:
 error=type(e).__name__+': '+str(e)
finally:
 if original is not None:
  try:
   termios.tcsetattr(fd,termios.TCSANOW,original)
   restored=termios.tcgetattr(fd)==original
  except Exception as e:error='Restore error: '+str(e)
 try:
  if exclusive:fcntl.ioctl(fd,termios.TIOCNXCL)
 finally:os.close(fd)
journal=subprocess.run(['/usr/bin/journalctl','-k','-b','--since','@'+str(start_wall),'-o','json','--no-pager'],capture_output=True,text=True,timeout=10)
faults=[]
for line in journal.stdout.splitlines():
 try:entry=json.loads(line)
 except ValueError:continue
 message=str(entry.get('MESSAGE',''))
 if int(entry.get('__MONOTONIC_TIMESTAMP','0'))>=int(start*1e6) and any(x.lower() in message.lower() for x in ('Unhandled context fault','VPR violation','Route Sanity error')):
  faults.append({'monotonic_us':entry.get('__MONOTONIC_TIMESTAMP'),'message':message})
raw=b''.join(base64.b64decode(x['data']) for x in chunks)
print(json.dumps({'boot_id':boot_id,'uart_iommus_hex':mapping.hex(),'start_unix':start_wall,'start_monotonic_s':start,'chunks':chunks,'sent':sent,'error':error,'settings_restored':restored,'fd_closed':True,'elapsed_s':locals().get('elapsed'),'bytes':len(raw),'zero_bytes':raw.count(0),'kernel_check_returncode':journal.returncode,'kernel_check_stderr':journal.stderr,'kernel_faults':faults}))
'''


def fixed_read_packet():
    mav = common.MAVLink(None, srcSystem=245, srcComponent=191)
    msg = common.MAVLink_param_request_read_message(1, 1, b'MAV_0_CONFIG', -1)
    packet = msg.pack(mav)
    parsed = common.MAVLink(None).parse_char(packet)
    assert parsed.get_type() == 'PARAM_REQUEST_READ'
    assert parsed.param_id == 'MAV_0_CONFIG' and parsed.param_index == -1
    assert packet.hex() == READ_PACKET_HEX
    return packet


def json_safe(value):
    if isinstance(value, dict):
        return {key: json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, bytes):
        return {'bytes_hex': value.hex()}
    return value


def decode(capture):
    mav = common.MAVLink(None)
    mav.robust_parsing = True
    counts = collections.Counter()
    last = {}
    times = collections.defaultdict(list)
    bad = []
    sync = False
    parameter_responses = []
    for chunk in capture.get('chunks', []):
        for msg in mav.parse_buffer(base64.b64decode(chunk['data'])) or []:
            name = msg.get_type()
            counts[name] += 1
            if name == 'BAD_DATA':
                bad.append({'after_sync':sync,'reason':str(msg.reason),'elapsed_s':chunk['elapsed_s']})
                continue
            if not name.startswith('UNKNOWN_'):
                sync = True
                value = msg.to_dict()
                if 'HEARTBEAT' == name:
                    value.update(armed=bool(msg.base_mode & 128),system_id=msg.get_srcSystem(),component_id=msg.get_srcComponent())
                last[name] = value
                if name == 'PARAM_VALUE' and msg.param_id == 'MAV_0_CONFIG':
                    parameter_responses.append({
                        'elapsed_s':chunk['elapsed_s'], 'system_id':msg.get_srcSystem(),
                        'component_id':msg.get_srcComponent(), 'param_type':msg.param_type,
                        'value_float':msg.param_value,
                        'value_int32_bytewise':struct.unpack('<i', struct.pack('<f', msg.param_value))[0]
                        if msg.param_type == common.MAV_PARAM_TYPE_INT32 else None})
            times[name].append(chunk['elapsed_s'])
    stats = {}
    for name, ts in times.items():
        stats[name] = {'count':len(ts),'rate_hz':(len(ts)-1)/(ts[-1]-ts[0]) if len(ts)>1 and ts[-1]>ts[0] else None,
                       'max_receive_gap_ms':max((b-a)*1000 for a,b in zip(ts,ts[1:])) if len(ts)>1 else None}
    return json_safe({'message_counts':dict(counts),'statistics':stats,'latest':last,
                      'bad_data':bad,'parameter_responses':parameter_responses})


def acceptance(report):
    capture=report.get('capture',{})
    decoded=report.get('decoded',{})
    heartbeat=decoded.get('latest',{}).get('HEARTBEAT',{})
    receive=(report['ssh_returncode']==0 and 'decode_error' not in report and not capture.get('error')
             and capture.get('settings_restored') is True and capture.get('fd_closed') is True
             and heartbeat.get('autopilot')==12 and heartbeat.get('system_id')==1
             and heartbeat.get('component_id')==1 and heartbeat.get('armed') is False
             and decoded.get('message_counts',{}).get('HEARTBEAT',0)>=3
             and not any(x['after_sync'] for x in decoded.get('bad_data',[])))
    kernel=(capture.get('kernel_check_returncode')==0 and not capture.get('kernel_check_stderr','').strip()
            and capture.get('kernel_faults')==[])
    parameter=None
    if report['parameter_read_requested']:
        sends=capture.get('sent',[])
        parameter=(len(sends)==1 and sends[0]['bytes']==len(bytes.fromhex(READ_PACKET_HEX))
                   and any(r['elapsed_s']>=sends[0]['elapsed_s']
                           and (r['system_id'],r['component_id'])==(1,1)
                           and r['param_type']==common.MAV_PARAM_TYPE_INT32
                           and r['value_int32_bytewise']==101
                           for r in decoded.get('parameter_responses',[])))
    return {'basic_receive_pass':receive,'kernel_fault_check_pass':kernel,
            'parameter_readback_pass':parameter,
            'scope_pass':receive and kernel and (parameter is not False),
            'control_path_verified':False, 'offboard_verified':False}


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',type=Path,required=True)
    ap.add_argument('--duration',type=float,default=20)
    ap.add_argument('--parameter-read',action='store_true')
    args=ap.parse_args()
    if args.output.exists():raise ValueError('Choose a fresh output file')
    packet=base64.b64encode(fixed_read_packet()).decode() if args.parameter_read else ''
    payload={'device':'/dev/ttyTHS1','baud':57600,'duration':args.duration,'read_packet':packet}
    code='import base64;exec(compile(base64.b64decode("'+base64.b64encode(REMOTE.encode()).decode()+'"),"<ram:uart-readonly>","exec"))'
    cmd=[r'C:\Windows\System32\OpenSSH\ssh.exe','-T','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes',
         '-o','ConnectTimeout=10','-o','IdentitiesOnly=yes','-i',r'C:\Users\Liym0817\.ssh\id_ed25519',
         '-o',r'UserKnownHostsFile=E:\research_my_studio\flyarm\safe-aerial-manipulator-testbed\artifacts\orin_nx_mpc_20261024\known_hosts',
         'jetson@192.168.30.145',"env PYTHONDONTWRITEBYTECODE=1 python3 -c '"+code+"'"]
    result=subprocess.run(cmd,input=json.dumps(payload).encode(),stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=args.duration+40)
    report={'utc':datetime.datetime.now(datetime.timezone.utc).isoformat(),'port':payload['device'],'baud':payload['baud'],
            'parameter_read_requested':args.parameter_read,'parameter_writes':0,'setpoints':0,'arm_commands':0,'mode_commands':0,
            'nx_output_files':[],'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            'ssh_returncode':result.returncode,'ssh_stderr':result.stderr.decode(errors='replace')}
    try:
        capture=json.loads(result.stdout)
        report['capture']=capture
        report['decoded']=decode(capture)
    except Exception as e:
        report['decode_error']=str(e)
        report['stdout']=result.stdout.decode(errors='replace')
    decoded=report.get('decoded',{})
    heartbeat=decoded.get('latest',{}).get('HEARTBEAT',{})
    report['acceptance']=acceptance(report)
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(json_safe(report),indent=2,ensure_ascii=False,allow_nan=False),encoding='utf-8')
    display={'report':str(args.output.resolve()),'acceptance':report['acceptance'],
             'message_counts':decoded.get('message_counts',{}),'heartbeat':heartbeat,
             'parameter_responses':decoded.get('parameter_responses',[]),
             'statistics':decoded.get('statistics',{}),
             'bad_data_preview':decoded.get('bad_data',[])[:3],
             'ssh_returncode':result.returncode,'ssh_stderr':report['ssh_stderr']}
    display['capture_summary']={k:v for k,v in report.get('capture',{}).items() if k!='chunks'}
    print(json.dumps(display,indent=2,ensure_ascii=False,allow_nan=False))
    return 0 if report['acceptance']['scope_pass'] else 2


if __name__=='__main__':sys.exit(main())
