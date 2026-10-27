#!/usr/bin/env python3
"""Exercise persistent NX code from recorded physics; outputs stay on workstation."""
import argparse
import hashlib
import json
from pathlib import Path
import shlex
import time
import traceback
from x650_pil_transport import Transport, validate_candidate, encode


def stats(values):
    x=sorted(values)
    def percentile(p):
        index=(len(x)-1)*p;i=int(index)
        return x[i]+(x[min(i+1,len(x)-1)]-x[i])*(index-i)
    return dict(count=len(x),min=x[0],p50=percentile(.5),p95=percentile(.95),p99=percentile(.99),max=x[-1]) if x else dict(count=0)


def ssh_command(args, report):
    libs=report['libraries'];exe=report['executable']
    native=[exe['path'],libs['predictor']['path'],libs['predictor']['sha256'],libs['core']['sha256'],
        libs['optimization']['sha256'],exe['sha256'],report['source_sha256']]
    return [args.ssh,'-T','-C','-o','BatchMode=yes','-o','StrictHostKeyChecking=yes',
        '-o','IdentitiesOnly=yes','-o','ConnectTimeout=10','-o','ServerAliveInterval=15',
        '-i',str(args.key),'-o','UserKnownHostsFile='+str(args.known_hosts),
        *(['-p',str(args.port)] if getattr(args,'port',None) else []),args.host,shlex.join(native)]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stream',type=Path,required=True);p.add_argument('--build-report',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);p.add_argument('--host',required=True)
    p.add_argument('--port',type=int)
    p.add_argument('--key',type=Path,required=True);p.add_argument('--known-hosts',type=Path,required=True)
    p.add_argument('--ssh',default='ssh');p.add_argument('--paced',action='store_true',help='Replay recorded simulated dt as workstation wall pacing; still no new physics')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    stream=json.loads(args.stream.read_bytes());build=json.loads(args.build_report.read_bytes())
    manifest=dict(kind='x650_pil_stream_run_v1', scope='continuous NX code on recorded states; no new plant feedback',
        continuous_memory=True,state_restored_each_step=False,physical_closed_loop=False,
        hardware_flight_qualified=False,complete=False,numerical_pass=False,exceptions=[],numerical_failures=[],
        stream_sha256=hashlib.sha256(args.stream.read_bytes()).hexdigest(),build=build,paced=args.paced,
        expected_steps=len(stream['steps']))
    (args.output/'protocol.json').write_text(json.dumps(manifest,indent=2))
    client=Transport(ssh_command(args,build),args.output/'ssh_stderr.txt')
    counters={k:[] for k in ('core_ms','core_thread_ms','nx_service_ms','host_roundtrip_ms','outside_service_ms')}
    actual=0;late=[];maximum={}
    try:
        start=client.request(stream['start'])[0]
        if start['status']!='ready':raise ValueError(start)
        manifest['start_response']=start
        next_wall=time.perf_counter()
        with (args.output/'responses.jsonl').open('x') as log:
            for item in stream['steps']:
                req=item['request'];s=req['snapshot']
                if args.paced:
                    next_wall+=s['dt_s'];time.sleep(max(0.,next_wall-time.perf_counter()))
                result,sent,received=client.request(req)
                row=dict(sequence=s['sequence'],request_sha256=hashlib.sha256(encode(req)).hexdigest(),
                    host_send_ns=sent,host_receive_ns=received,response=result)
                log.write(json.dumps(row,separators=(',',':'),allow_nan=False)+'\n')
                timings=validate_candidate(req,result,sent_ns=sent,received_ns=received,
                    source_sha256=build['source_sha256'],check_deadline=False)
                actual+=1
                for k,n in (('core_ms','nx_core_ns'),('nx_service_ms','nx_service_ns'),
                    ('host_roundtrip_ms','host_roundtrip_ns'),('outside_service_ms','outside_nx_service_ns')):
                    counters[k].append(timings[n]/1e6)
                counters['core_thread_ms'].append(result['core_thread_ns']/1e6)
                if received-sent>50_000_000:late.append(s['sequence'])
                failures=[]
                for key,expected in item['oracle'].items():
                    err=max(abs(a-b) for a,b in zip(result['outputs'][key],expected))
                    maximum[key]=max(maximum.get(key,0),err)
                    tol=1e-9 if key=='position_reference' else (1e-7/s['dt_s'] if key=='joint_acceleration_reference' else 1e-7)
                    if err>tol:failures.append(dict(field=key,error=err,tolerance=tol))
                if failures:manifest['numerical_failures'].append(dict(sequence=s['sequence'],fields=failures))
        h=stream['start']['handoff']
        manifest['stop_response']=client.request(dict(op='stop',session_id=h['session_id'],epoch=h['epoch']))[0]
        manifest['complete']=actual==len(stream['steps']) and manifest['stop_response']['steps']==actual
    except Exception:
        manifest['exceptions'].append(traceback.format_exc())
    finally:
        manifest['exit_code']=client.close();manifest['actual_steps']=actual
        manifest['timing_ms']={k:stats(v) for k,v in counters.items()}
        manifest['over_50ms_roundtrip_sequences']=late;manifest['maximum_absolute_error']=maximum
        manifest['numerical_pass']=manifest['complete'] and not manifest['numerical_failures']
        manifest['stream_integrity_pass']=manifest['complete'] and not manifest['exceptions'] and manifest['exit_code']==0
        manifest['roundtrip_budget_pass']=manifest['stream_integrity_pass'] and not late
        manifest['recording_sha256']=hashlib.sha256((args.output/'responses.jsonl').read_bytes()).hexdigest() if (args.output/'responses.jsonl').exists() else None
        (args.output/'report.json').write_text(json.dumps(manifest,indent=2,allow_nan=False))
    print(json.dumps({k:manifest[k] for k in ('complete','actual_steps','numerical_pass','stream_integrity_pass','roundtrip_budget_pass','timing_ms','maximum_absolute_error','exceptions')},indent=2))
    return 0 if manifest['stream_integrity_pass'] and manifest['numerical_pass'] else 1


if __name__=='__main__':
    raise SystemExit(main())
