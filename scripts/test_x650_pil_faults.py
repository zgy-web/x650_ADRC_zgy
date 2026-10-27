#!/usr/bin/env python3
"""Actual NX process faults plus server-side admission faults; no actuators."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import struct
import time
from run_x650_pil_stream import ssh_command
from x650_pil_transport import Transport, validate_candidate, encode


def main():
    p=argparse.ArgumentParser(description=__doc__)
    for name in ('stream','build-report','output','key','known-hosts'):
        p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--host',required=True);p.add_argument('--ssh',default='ssh')
    args=p.parse_args();args.output.mkdir(parents=True,exist_ok=False)
    stream=json.loads(args.stream.read_bytes());build=json.loads(args.build_report.read_bytes())
    start=stream['start'];first=stream['steps'][0]['request'];second=stream['steps'][1]['request']
    cases=[];saved_response=None
    def client(label):return Transport(ssh_command(args,build),args.output/(label+'.stderr.txt'))
    def rejected(label,mutation,prestep=False):
        c=client(label);row=dict(case=label,pass_=False)
        try:
            row['start']=c.request(start)[0]
            if prestep:row['first']=c.request(first)[0]
            bad=deepcopy(second if prestep else first);mutation(bad)
            row['bad']=c.request(bad)[0]
            row['after_fault']=c.request(second if prestep else first)[0]
            expected_steps=1 if prestep else 0
            row['pass_']=(row['bad']['status']=='rejected' and row['after_fault']['status']=='rejected'
                and row['bad']['memory_step_count']==row['after_fault']['memory_step_count']==expected_steps)
        except Exception as e:row['error']=repr(e)
        finally:row['exit_code']=c.close();cases.append(row)
    rejected('duplicate',lambda v:v.update(deepcopy(first)),True)
    rejected('out_of_order',lambda v:v['snapshot'].update(sequence=2))
    rejected('old_epoch',lambda v:v['snapshot'].update(epoch=v['snapshot']['epoch']+1))
    rejected('wrong_session',lambda v:v['snapshot'].update(session_id=v['snapshot']['session_id']+1))
    rejected('changed_hash',lambda v:v['snapshot']['configuration_sha256'].__setitem__(0,v['snapshot']['configuration_sha256'][0]^1))
    rejected('stale_joint',lambda v:v['snapshot'].update(joint_time_s=v['snapshot']['control_time_s']-.03))
    rejected('wrong_dt',lambda v:v['snapshot'].update(dt_s=.02))
    rejected('invalid_rotation',lambda v:v['snapshot']['rotation_row_major'].__setitem__(0,2.))
    rejected('joint_limit',lambda v:v['snapshot']['q'].__setitem__(0,2.))
    rejected('wrong_frame_schema',lambda v:v['snapshot'].update(schema_version=65537))
    rejected('oversize_history_count',lambda v:v['snapshot'].update(packet_count=4294967296))
    rejected('changed_reference',lambda v:v['snapshot']['target_position_enu'].__setitem__(0,100.))
    rejected('nonfinite_state',lambda v:v['snapshot']['position_enu'].__setitem__(0,'NaN'))
    # A fresh process reconstructs zero-step Memory from a fresh explicit handoff.
    for label in ('clean_restart','repeat_start','eof','truncated_frame','oversize_frame','idle_timeout'):
        c=client(label);row=dict(case=label,pass_=False)
        try:
            row['start']=c.request(start)[0]
            if label=='clean_restart':
                row['response'],sent,received=c.request(first);saved_response=row['response']
                h=start['handoff'];row['stop']=c.request(dict(op='stop',session_id=h['session_id'],epoch=h['epoch']))[0]
                row['pass_']=row['response']['status']=='candidate' and row['response']['memory_step_count']==1 and row['stop']['status']=='stopped'
            elif label=='repeat_start':
                row['bad']=c.request(start)[0];row['pass_']=row['bad']['status']=='rejected' and row['bad']['memory_step_count']==0
            elif label=='truncated_frame':
                c.proc.stdin.write(struct.pack('!I',100)+b'{}');c.proc.stdin.close()
            elif label=='oversize_frame':
                c.proc.stdin.write(struct.pack('!I',131073));c.proc.stdin.close()
            elif label=='eof':c.proc.stdin.close()
            elif label=='idle_timeout':
                c.proc.wait(timeout=7)
        except Exception as e:row['error']=repr(e)
        finally:
            row['exit_code']=c.close()
            if label in ('eof','truncated_frame','oversize_frame','idle_timeout'):
                row['pass_']=row['exit_code']==2 and 'error' not in row
            cases.append(row)
    # These are explicit tests of the workstation reference gate, not a claim
    # that a not-yet-connected physical actuator stopped.
    for label,modify,age in (
        ('server_deadline',lambda v:None,50_000_001),
        ('server_wrong_digest',lambda v:v.update(request_sha256='0'*64),10_000_000),
        ('server_refreshed_issue',lambda v:v.update(source_issue_ns=v['source_issue_ns']+1),10_000_000),
        ('server_bad_pva',lambda v:v['proposal_nx_clock']['velocity_feedforward_enu'].__setitem__(0,1.),10_000_000),
        ('server_nan_command',lambda v:v['proposal_nx_clock']['joint_velocity'].__setitem__(0,float('nan')),10_000_000),
        ('server_wrong_memory',lambda v:v.update(memory_step_count=2),10_000_000)):
        response=deepcopy(saved_response);modify(response);row=dict(case=label,pass_=False)
        try:validate_candidate(first,response,sent_ns=1000000000,received_ns=1000000000+age,source_sha256=build['source_sha256'])
        except ValueError as e:row.update(pass_=True,detected=str(e))
        cases.append(row)
    report=dict(kind='x650_pil_fault_tests_v1',cases=cases,passed=all(v['pass_'] for v in cases),
        total=len(cases),passed_count=sum(v['pass_'] for v in cases),
        physical_actuator_stop_tested=False,actuator_publishers=0,build=build)
    (args.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False))
    print(json.dumps(dict(passed=report['passed'],total=report['total'],passed_count=report['passed_count'],
        failed=[v for v in cases if not v['pass_']]),indent=2))
    return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
