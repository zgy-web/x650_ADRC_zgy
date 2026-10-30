"""Run a previously built bench executable from NX RAM; save results on PC.

Does not apply PX4 parameters, install software, switch USB, or arm the vehicle.
The caller/operator must separately approve the case and confirm power isolation.
"""
import argparse,base64,hashlib,json,shlex,subprocess
from pathlib import Path

REMOTE=r'''
import base64,hashlib,json,os,pathlib,resource,subprocess,sys,time
p=json.load(sys.stdin);resource.setrlimit(resource.RLIMIT_CORE,(0,0))
data=base64.b64decode(p['binary'],validate=True)
if hashlib.sha256(data).hexdigest()!=p['sha256']:raise ValueError('Binary identity differs')
if p['case'] not in ('observe','readback','manual_exit','stream_stop'):raise ValueError('Unknown case')
if p['baud'] not in (57600,921600) or not 5<=p['seconds']<=75:raise ValueError('Invalid limits')
users=subprocess.run(['/usr/bin/fuser','/dev/ttyTHS1'],capture_output=True,timeout=5)
if users.returncode!=1 or users.stdout.strip():raise ValueError('UART owned or ownership check unavailable')
tree=pathlib.Path('/sys/firmware/devicetree/base/bus@0')
if (tree/'serial@3100000/iommus').read_bytes()!=(tree/'dma-controller@2600000/iommus').read_bytes():
 raise ValueError('Verified UART mapping not active')
fd=os.memfd_create('flyarm-uart-bench',0);child=None
try:
 os.write(fd,data);os.fchmod(fd,0o700)
 args=['/proc/self/fd/'+str(fd),'--baud',str(p['baud']),'--seconds',str(p['seconds']),'--case',p['case']]
 if p['power_disconnected']:args.append('--power-disconnected')
 if p['configure']:args.append('--configure-bench-streams')
 start=int(time.time());mono=time.monotonic()
 child=subprocess.Popen(args,pass_fds=(fd,),stdout=subprocess.PIPE,stderr=subprocess.PIPE)
 for line in child.stdout:
  sys.stdout.buffer.write(line);sys.stdout.buffer.flush()
 err=child.stderr.read();code=child.wait(timeout=10)
 journal=subprocess.run(['/usr/bin/journalctl','-k','-b','--since','@'+str(start),'-o','json','--no-pager'],capture_output=True,text=True,timeout=10)
 faults=[]
 for line in journal.stdout.splitlines():
  try:r=json.loads(line)
  except ValueError:continue
  message=str(r.get('MESSAGE',''))
  if int(r.get('__MONOTONIC_TIMESTAMP','0'))>=int(mono*1e6) and any(s in message.lower() for s in ('unhandled context fault','vpr violation','route sanity error')):faults.append(message)
 print(json.dumps(dict(event='exit',returncode=code,stderr=err.decode(errors='replace'),kernel_check_returncode=journal.returncode,
  boot_id=pathlib.Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
  kernel_check_stderr=journal.stderr,kernel_faults=faults,sha256=p['sha256'],persistent_files_created=[])),flush=True)
finally:
 if child is not None and child.poll() is None:
  child.terminate()
  try:child.wait(timeout=3)
  except subprocess.TimeoutExpired:child.kill();child.wait(timeout=3)
 os.close(fd)
 print(json.dumps(dict(event='cleanup',memfd_closed=True,process_exited=child is None or child.poll() is not None)),flush=True)
'''


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('build','output','identity','known-hosts'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--ssh',default='ssh');p.add_argument('--host',default='192.168.30.145')
    p.add_argument('--case',choices=('observe','readback','manual_exit','stream_stop'),default='observe')
    p.add_argument('--baud',type=int,choices=(57600,921600),default=57600)
    p.add_argument('--seconds',type=int,default=30)
    p.add_argument('--power-disconnected',action='store_true');p.add_argument('--configure-bench-streams',action='store_true')
    a=p.parse_args()
    if not 5<=a.seconds<=75:raise ValueError('5..75 seconds required')
    if (a.case!='observe' or a.configure_bench_streams) and not a.power_disconnected:
        raise ValueError('Explicit operator confirmation required before bounded TX')
    events=[json.loads(x) for x in (a.build/'transport.jsonl').read_bytes().splitlines()]
    if not any(r.get('event')=='complete' and r.get('passed') is True for r in events):raise ValueError('Build not passed')
    if not any(r.get('event')=='cleanup' and r.get('removed') is True for r in events):raise ValueError('Build cleanup not confirmed')
    binary=(a.build/'flyarm_uart_bench').read_bytes();sha=hashlib.sha256(binary).hexdigest()
    identity=[r for r in events if r.get('name')=='flyarm_uart_bench']
    if len(identity)!=1 or identity[0]['sha256']!=sha:raise ValueError('Binary differs from tested export')
    request=dict(binary=base64.b64encode(binary).decode(),sha256=sha,case=a.case,baud=a.baud,seconds=a.seconds,
                 power_disconnected=a.power_disconnected,configure=a.configure_bench_streams)
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'request.json').write_text(json.dumps({k:v for k,v in request.items() if k!='binary'},indent=2))
    cmd=[a.ssh,'-T','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','IdentitiesOnly=yes','-o','ConnectTimeout=8',
         '-i',str(a.identity),'-o','UserKnownHostsFile='+str(a.known_hosts.resolve()),'jetson@'+a.host,
         shlex.join(['env','PYTHONDONTWRITEBYTECODE=1','python3','-u','-c',REMOTE])]
    child=subprocess.Popen(cmd,stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=subprocess.STDOUT)
    events=[];send_error=None
    try:
        child.stdin.write(json.dumps(request).encode());child.stdin.close()
    except OSError as exc:
        send_error=dict(event='request_delivery_failed',error=str(exc))
        try:child.stdin.close()
        except OSError:pass
    with (a.output/'transport.jsonl').open('wb') as log:
        if send_error:
            log.write((json.dumps(send_error)+'\n').encode());log.flush()
            events.append(send_error);print(json.dumps(send_error),flush=True)
        for line in child.stdout:
            log.write(line);log.flush()
            try:r=json.loads(line)
            except ValueError:print(line.decode(errors='replace'),end='');continue
            events.append(r);print(json.dumps({k:v for k,v in r.items() if k!='records'}),flush=True)
    status=child.wait()
    results=[r for r in events if r.get('event')=='result'];exits=[r for r in events if r.get('event')=='exit']
    cleanup=any(r.get('event')=='cleanup' and r.get('memfd_closed') and r.get('process_exited') for r in events)
    complete=send_error is None and status==0 and len(results)==len(exits)==1 and cleanup
    if complete:
        r=results[0];e=exits[0]
        complete=(r['error']=='' and r['settings_restored'] and r['fd_closed'] and e['returncode']==0
                  and e['kernel_check_returncode']==0 and not e['kernel_check_stderr'].strip() and not e['kernel_faults'])
    (a.output/'completion.json').write_text(json.dumps(dict(capture_complete=complete,case=a.case,
        physical_acceptance=False,explanation='Independent PX4 Debug readback and operator observation still required'),indent=2))
    if not complete:raise RuntimeError('Capture/cleanup incomplete; inspect PC evidence')


if __name__=='__main__':main()
