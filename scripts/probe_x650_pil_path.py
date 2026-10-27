#!/usr/bin/env python3
"""Measure server -> SSH loopback tunnel -> actual NX persistent worker.

No ROS/actuator publisher. Sources are recorded snapshots. All output is stored
on this workstation. The server's monotonic roundtrip is measured on the server.
"""
import argparse
import json
from pathlib import Path
import shlex
import socket
import subprocess
import threading
from run_x650_pil_stream import ssh_command

SERVER_PROBE = r'''
import sys,json,socket,struct,time,hashlib
doc=json.load(sys.stdin);port=doc['port'];stream=doc['stream']
s=socket.create_connection(('127.0.0.1',port),timeout=5);s.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1);s.settimeout(5)
def read(n):
 b=bytearray()
 while len(b)<n:
  p=s.recv(n-len(b))
  if not p:raise EOFError('NX path disconnected')
  b.extend(p)
 return bytes(b)
def request(value):
 raw=json.dumps(value,separators=(',',':'),allow_nan=False).encode('ascii')
 before=time.monotonic_ns();s.sendall(struct.pack('!I',len(raw))+raw)
 size=struct.unpack('!I',read(4))[0]
 if not 0<size<=131072:raise ValueError('Response size')
 response=read(size);after=time.monotonic_ns();r=json.loads(response)
 if r['request_sha256']!=hashlib.sha256(raw).hexdigest():raise ValueError('Request binding')
 return {'server_send_ns':before,'server_receive_ns':after,'roundtrip_ms':(after-before)/1e6,'response':r}
print(json.dumps({'stage':'start',**request(stream['start'])}),flush=True)
for item in stream['steps']:
 r=request(item['request']);print(json.dumps({'stage':'step','sequence':item['request']['snapshot']['sequence'],**r}),flush=True)
 if r['response']['status']!='candidate':raise RuntimeError(r['response'])
h=stream['start']['handoff']
print(json.dumps({'stage':'stop',**request(dict(op='stop',session_id=h['session_id'],epoch=h['epoch']))}),flush=True)
s.close()
'''


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stream',type=Path,required=True);p.add_argument('--build-report',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--host',required=True)
    p.add_argument('--key',type=Path,required=True);p.add_argument('--known-hosts',type=Path,required=True)
    p.add_argument('--server-host',required=True);p.add_argument('--server-port',type=int,required=True)
    p.add_argument('--server-known-hosts',type=Path,required=True)
    p.add_argument('--forward-port',type=int,default=43341);p.add_argument('--ssh',default='ssh')
    p.add_argument('--steps',type=int,default=100)
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    stream=json.loads(args.stream.read_bytes());stream['steps']=stream['steps'][:args.steps]
    build=json.loads(args.build_report.read_bytes())
    listener=socket.socket();listener.bind(('127.0.0.1',0));listener.listen(1);listener.settimeout(20)
    local_port=listener.getsockname()[1];relay_report={};worker_holder=[]
    def relay():
        try:
            connection,_=listener.accept();connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            with connection,(args.output/'nx_stderr.txt').open('xb') as error:
                worker=subprocess.Popen(ssh_command(args,build),stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=error,bufsize=0)
                worker_holder.append(worker)
                def upstream():
                    try:
                        while True:
                            chunk=connection.recv(65536)
                            if not chunk:break
                            offset=0
                            while offset<len(chunk):
                                n=worker.stdin.write(chunk[offset:])
                                if not n:raise EOFError('Worker input closed')
                                offset+=n
                    except (OSError,ValueError) as e:
                        relay_report['upstream_error']=str(e)
                    finally:
                        worker.stdin.close()
                t=threading.Thread(target=upstream,daemon=True);t.start()
                while True:
                    chunk=worker.stdout.read(65536)
                    if not chunk:break
                    connection.sendall(chunk)
                connection.shutdown(socket.SHUT_RDWR);t.join(timeout=3)
                relay_report['nx_exit_code']=worker.wait(timeout=8)
                worker.stdout.close()
        except Exception as e:
            relay_report['error']=repr(e)
        finally:
            listener.close()
    thread=threading.Thread(target=relay,daemon=True);thread.start()
    command=[args.ssh,'-T','-C','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','IdentitiesOnly=yes',
        '-o','ExitOnForwardFailure=yes','-o','ConnectTimeout=10','-o','ServerAliveInterval=15',
        '-i',str(args.key),'-o','UserKnownHostsFile='+str(args.server_known_hosts),'-p',str(args.server_port),
        '-R',f'127.0.0.1:{args.forward_port}:127.0.0.1:{local_port}',args.server_host,
        shlex.join(['python3','-c',SERVER_PROBE])]
    try:
        with (args.output/'server_roundtrip.jsonl').open('xb') as output,(args.output/'server_stderr.txt').open('xb') as error:
            result=subprocess.run(command,input=json.dumps(dict(port=args.forward_port,stream=stream)).encode(),
                stdout=output,stderr=error,timeout=240)
        thread.join(timeout=12)
        rows=[json.loads(x) for x in (args.output/'server_roundtrip.jsonl').read_bytes().splitlines()]
        steps=[x for x in rows if x['stage']=='step']
        from run_x650_pil_stream import stats
        report=dict(kind='x650_pil_actual_path_probe_v1',physical_closed_loop=False,actuator_publishers=0,
            complete=result.returncode==0 and len(steps)==len(stream['steps']) and relay_report.get('nx_exit_code')==0,
            server_exit_code=result.returncode,relay=relay_report,steps=len(steps),
            server_roundtrip_ms=stats([x['roundtrip_ms'] for x in steps]),
            nx_core_ms=stats([x['response'].get('core_wall_ns',0)/1e6 for x in steps]),
            late_sequences=[x['sequence'] for x in steps if x['roundtrip_ms']>50],
            path='server -> authenticated loopback SSH forwarding -> workstation byte relay -> authenticated NX SSH worker',
            deadline_ms=50)
        report['live_transport_admission']=report['complete'] and not report['late_sequences']
        (args.output/'report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))
    finally:
        listener.close()
        for worker in worker_holder:
            if worker.poll() is None:worker.terminate()


if __name__=='__main__':
    main()
