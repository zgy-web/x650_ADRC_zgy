#!/usr/bin/env python3
"""Workstation-owned SSH tunnel for a server simulation and code-only NX.

The supplied server command launches a predeclared simulation. It is not sent to
NX. NX receives bounded protocol frames in RAM; stderr remains on workstation.
"""
import argparse
import json
from pathlib import Path
import socket
import subprocess
import threading
from run_x650_pil_stream import ssh_command


def transport_passed(report):
    return (report.get('server_exit') == 0 and report.get('nx_exit') == 0
            and not report.get('relay_thread_alive')
            and not any(k.endswith('_error') for k in report))


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for k in ('host','server-host','server-command'):p.add_argument('--'+k,required=True)
    for k in ('key','known-hosts','server-known-hosts','build-report','output'):p.add_argument('--'+k,type=Path,required=True)
    p.add_argument('--server-port',type=int,required=True);p.add_argument('--forward-port',type=int,default=43341)
    p.add_argument('--ssh',default='ssh');p.add_argument('--timeout',type=int,default=1800)
    a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    build=json.loads(a.build_report.read_bytes());(a.output/'nx_build_binding.json').write_text(json.dumps(build,indent=2))
    listener=socket.socket();listener.bind(('127.0.0.1',0));listener.listen(1);listener.settimeout(a.timeout)
    local_port=listener.getsockname()[1];workers=[];report={'mode':'synchronous_simulation','wall_clock_qualified':False}
    def relay():
        try:
            connection,_=listener.accept();connection.setsockopt(socket.IPPROTO_TCP,socket.TCP_NODELAY,1)
            with connection,(a.output/'nx_stderr.txt').open('xb') as log:
                worker=subprocess.Popen(ssh_command(a,build),stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=log,bufsize=0);workers.append(worker)
                def upstream():
                    try:
                        while True:
                            chunk=connection.recv(65536)
                            if not chunk:break
                            offset=0
                            while offset<len(chunk):
                                n=worker.stdin.write(chunk[offset:])
                                if not n:raise EOFError('NX pipe closed')
                                offset+=n
                    except (OSError,ValueError) as e:report['upstream_error']=repr(e)
                    finally:worker.stdin.close()
                t=threading.Thread(target=upstream,daemon=True);t.start()
                while True:
                    data=worker.stdout.read(65536)
                    if not data:break
                    connection.sendall(data)
                try:connection.shutdown(socket.SHUT_RDWR)
                except OSError:pass
                t.join(timeout=3);report['nx_exit']=worker.wait(timeout=8)
        except Exception as e:report['relay_error']=repr(e)
    thread=threading.Thread(target=relay,daemon=True);thread.start()
    command=[a.ssh,'-T','-C','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes','-o','IdentitiesOnly=yes',
        '-o','ExitOnForwardFailure=yes','-o','ConnectTimeout=10','-o','ServerAliveInterval=15',
        '-i',str(a.key),'-o','UserKnownHostsFile='+str(a.server_known_hosts),'-p',str(a.server_port),
        '-R',f'127.0.0.1:{a.forward_port}:127.0.0.1:{local_port}',a.server_host,a.server_command]
    try:
        with (a.output/'server_output.log').open('xb') as out,(a.output/'server_error.log').open('xb') as err:
            result=subprocess.run(command,stdin=subprocess.DEVNULL,stdout=out,stderr=err,timeout=a.timeout)
        report['server_exit']=result.returncode;thread.join(timeout=10)
        report['relay_thread_alive']=thread.is_alive()
    finally:
        listener.close()
        for worker in workers:
            if worker.poll() is None:
                worker.terminate();worker.wait(timeout=5)
        report['transport_complete']=transport_passed(report)
        (a.output/'transport_report.json').write_text(json.dumps(report,indent=2))
        print(json.dumps(report,indent=2))
    if not report['transport_complete']:raise SystemExit(1)


if __name__=='__main__':main()
