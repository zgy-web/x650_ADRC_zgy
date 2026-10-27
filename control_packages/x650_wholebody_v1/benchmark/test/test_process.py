"""Standalone Linux process tests; run on a development host, not on the NX.

The benchmark has no output filename, and inputs/results remain on pipes. These
tests exercise real Core calls, failure retention, strict schema and no file
creation in the application working directory. They make no speed claim.
"""
import argparse
import copy
import hashlib
import json
import math
import pathlib
import subprocess
import tempfile


def fixture():
    cfg = dict(nodes=3, max_iterations=100, node_dt=.04, max_dt=.05,
               feasibility_tolerance=2e-7, kkt_tolerance=.002, arm_share=.2,
               joint_position_gain=3., weights=[400.,4.,.25,.02,.01,.01],
               velocity_limit=[.55,.35,.45,.45,.35], acceleration_limit=[.45,.30,.45,.60,.60],
               q_lower=[-.75,-.35],q_upper=[.45,.85],physical_lower=[-2.,-2.],physical_upper=[2.,2.],
               geometry=[.2,.2,-math.pi/4,math.pi/4,0.,0.,0.,0.,0.,0.],
               response=[4.,.8,4.,.8,.25],base_position_gain=[1.,1.,1.],
               motion=[.01,.1,.02,0.,.015,.03,.005,.01],kp=[6.,6.],kd=[.25,.25],
               bus_period=.005,integration_step=.001,trust_radius=.15)
    memory = dict(initialized=True,history_primed=True,has_last_t=True,has_stamp=False,has_origin=True,
                  anchor=dict(position=[0.,0.,0.],rotation=[1.,0.,0.,0.,1.,0.,0.,0.,1.],q=[0.,0.]),
                  previous_command=[0.]*5,last_t=0.,previous_stamp=0.,origin=0.,plan_t=0.,step_count=0,
                  plan=[],plan_h=[],joint_history=[dict(time=0.,velocity=[0.,0.])])
    target=dict(position=[.4*math.cos(math.pi/4),0.,0.],velocity=[0.]*3,acceleration=[0.]*3)
    inp=dict(t=.02,dt=.02,state=dict(position=[0.]*3,velocity=[0.]*3,omega_body=[0.]*3,
             acceleration=[0.]*3,rotation=[1.,0.,0.,0.,1.,0.,0.,0.,1.],q=[0.,0.],dq=[0.,0.],
             stamp=.02,control_time=.02,joint_stamp=.02),target=target,preview=[],
             context=dict(reference_time=0.,bus_origin=0.,reference=[0.,0.],commands=[dict(time=0.,velocity=[0.,0.])],
               packets=[dict(send_time=i*.005,q_ref=[0.,0.],dq_ref=[0.,0.],tau_ff=[0.,0.],kp=[6.,6.],kd=[.25,.25]) for i in range(-20,1)]))
    case=dict(id="analytic_stationary",memory=memory,input=inp,
              oracle=dict(command=[0.]*5,position_reference=[0.]*3,velocity_feedforward=[0.]*3))
    return dict(schema_version=1,kind="x650_mpc_memory_benchmark_input_v1",config=cfg,cases=[case],
                run=dict(warmup_rounds=1,measured_rounds=3,cpu=-1,oracle_absolute_tolerance=1e-7),
                provenance=dict(kind="analytic_stationary_test_only",performance_claimed=False))


def main():
    p=argparse.ArgumentParser();p.add_argument('--binary',type=pathlib.Path,required=True);p.add_argument('--predictor',type=pathlib.Path,required=True);a=p.parse_args()
    ph=hashlib.sha256(a.predictor.read_bytes()).hexdigest()
    count=0
    with tempfile.TemporaryDirectory(prefix='x650_memory_contract_') as tmp:
        wd=pathlib.Path(tmp)
        def run(value, *, raw=None, sha=None, extra=()):
            nonlocal count
            data=raw if raw is not None else json.dumps(value,allow_nan=False,separators=(',',':')).encode()
            command=[str(a.binary),'--predictor',str(a.predictor),'--predictor-sha256',ph,
                     '--input-sha256',sha or hashlib.sha256(data).hexdigest(),*extra]
            before=set(wd.rglob('*'))
            r=subprocess.run(command,input=data,stdout=subprocess.PIPE,stderr=subprocess.PIPE,cwd=wd,timeout=30)
            assert set(wd.rglob('*'))==before,'Benchmark created a working-directory file'
            assert r.stderr==b'',r.stderr.decode(errors='replace')
            result=json.loads(r.stdout)
            count+=1
            return r.returncode,result
        value=fixture();code,good=run(value)
        assert code==0 and good['complete'],good
        assert good['case_count']==1 and len(good['samples'])==3 and len(good['warmup_samples'])==1
        assert good['summary']['measured']['sample_count']==3
        assert good['scope']['application_file_writes']==0
        assert good['scope']['measurement_loop_output_bytes']==0
        assert good['scope']['cuda_implemented'] is False
        assert all(s['returned'] and s['oracle_match'] and s['memory_commit_valid'] for s in good['samples']),good['samples']
        assert all(s['step_count_after']==1 for s in good['samples']), 'Snapshots were not independently restored'
        assert all(s['core_step_wall_ns']>0 and s['core_step_thread_cpu_ns']>0 for s in good['samples'])
        failed=copy.deepcopy(value);failed['cases'][0]['id']='deliberately_invalid_dt';failed['cases'][0]['input']['dt']=0.
        code,result=run(failed)
        assert code==0 and result['complete'] and result['all_calls_valid'] is False
        assert len(result['samples'])==3 and result['summary']['measured']['core_error_count']==3
        assert all(s['core_error'] and not s['returned'] and s['memory_unchanged'] and s['step_count_after']==0 for s in result['samples'])
        assert result['summary']['measured']['wall']['sample_count']==3,'Failure timing was discarded'
        bad=copy.deepcopy(value);bad['run']['measured_rounds']=100001
        assert run(bad)[0]==2
        bad=copy.deepcopy(value);bad['run']['oracle_absolute_tolerance']=2e-7
        assert run(bad)[0]==2
        bad=copy.deepcopy(value);bad['config']['max_dt']=.1
        assert run(bad)[0]==2
        bad=copy.deepcopy(value);bad['cases'].append(copy.deepcopy(bad['cases'][0]))
        assert run(bad)[0]==2
        bad=copy.deepcopy(value);del bad['cases'][0]['input']['preview']
        assert run(bad)[0]==2
        assert run(value,sha='0'*64)[0]==2
        assert run(value,raw=b'{"x":1,"x":2}')[0]==2
        assert run(value,extra=('--output','unwanted.json'))[0]==2
        assert good['core_abi']==4 and good['optimization_abi']==2
        bad=copy.deepcopy(value);bad['provenance']['expected_core_abi']=3
        assert run(bad)[0]==2, 'Wrong Core ABI was accepted'
        bad=copy.deepcopy(value);bad['provenance']['expected_optimization_abi']=1
        assert run(bad)[0]==2, 'Wrong optimizer ABI was accepted'
        extended=copy.deepcopy(value);extended['config'].update(nodes=15,max_dt=.1,command_hold_s=.16)
        extended['provenance'].update(expected_core_abi=4,expected_optimization_abi=2)
        code,result=run(extended)
        assert code==0 and result['all_calls_valid'],result
        assert result['scope']['control_update_target_ms']==40
        assert result['scope']['continuous_dispatch_period_ms']==20
        assert result['scope']['configured_command_hold_s']==.16
        assert all(s['diagnostics']['hold_required'] and s['diagnostics']['hold_checked']
            and s['diagnostics']['hold_passed'] and s['diagnostics']['hold_substeps']>0
            for s in result['samples'])
        bad=copy.deepcopy(value);bad['cases'][0]['oracle']['position_reference']=[2e-9,0.,0.]
        code,result=run(bad)
        assert code==0 and result['complete']
        assert all(s['returned'] and s['oracle_checked'] and not s['oracle_match'] for s in result['samples']),'Original 1e-9 position tolerance was relaxed'
    print(json.dumps(dict(passed=True,process_cases=count,measured_samples_per_completed_run=3,
                          scope='Input/output contract and real small-Core calls; no hardware-performance claim')))


if __name__=='__main__':main()
