"""Independent Python FigureEight versus the compiled production C++ preview.

Only executable stdin/stdout is used. No ROS, controller solve or actuator I/O.
"""
import argparse
import ast
import hashlib
import json
from functools import lru_cache
from pathlib import Path
import subprocess
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
TRAJECTORY = dict(kind='fast_figure8', amplitude_x=.25, amplitude_z=.10,
                  period_s=8., cycles=2, start_s=2., ramp_s=2., duration_s=24.)
ORACLE = ROOT/'simulations/x650_wholebody_mpc_v1/run_case.py'


def identity(path):
    p=Path(path).resolve();data=p.read_bytes()
    return dict(path=str(p),bytes=len(data),sha256=hashlib.sha256(data).hexdigest())


@lru_cache(maxsize=1)
def _figure_eight_class():
    tree=ast.parse(ORACLE.read_text(encoding='utf-8'))
    classes=[n for n in tree.body if isinstance(n,ast.ClassDef) and n.name=='FigureEight']
    if len(classes)!=1:raise ValueError('Unique independent original FigureEight required')
    module=ast.fix_missing_locations(ast.Module(body=classes,type_ignores=[]));namespace={'np':np}
    exec(compile(module,str(ORACLE),'exec'),namespace)
    return namespace['FigureEight']


def figure_eight(center):
    return _figure_eight_class()(np.asarray(center),**{k:v for k,v in TRAJECTORY.items() if k not in ('kind','duration_s')})


def corpus():
    times=set(np.linspace(0.,24.,1201).tolist())
    for t in (0.,2.,4.,18.,20.,24.):times.update((t-1e-8,t,t+1e-8))
    steps=(.012,.016,.020,.024)
    rows=[]
    for center in ((0.,0.,2.),(1.2,-.3,2.1)):
        for i,t in enumerate(sorted(times)):
            rows.append(dict(kind=2,t=t,dt=steps[i%4],nodes=15,node_dt=.04,center=list(center)))
        for kind in (0,2):
            for t in (1.99,2.,3.99,4.,17.99,18.,19.99,20.,23.99):
                for dt in steps:rows.append(dict(kind=kind,t=t,dt=dt,nodes=15,node_dt=.04,center=list(center)))
    return rows


def input_text(cases):
    return ''.join(' '.join(str(v) for v in (i,c['kind'],c['t'],c['dt'],c['nodes'],c['node_dt'],*c['center']))+'\n' for i,c in enumerate(cases))


def expected(cases):
    rows=[]
    for i,c in enumerate(cases):
        law=figure_eight(c['center']);offset=0.
        for node in range(-1,c['nodes']):
            if node>=0:offset+=c['dt'] if node==0 else c['node_dt']
            at=c['t']+offset
            pva=np.concatenate(law.sample(at)) if c['kind'] else np.r_[c['center'],np.zeros(6)]
            rows.append((i,node,at,*pva))
    return np.asarray(rows)


def compare(actual,cases):
    wanted=expected(cases)
    if actual.shape!=wanted.shape or not np.isfinite(actual).all():raise ValueError('Missing/malformed C++ reference rows')
    if not np.array_equal(actual[:,:3],wanted[:,:3]):raise ValueError('Case/node/cumulative actual-dt time differs')
    current=actual[:,1]==-1;errors={}
    for prefix,mask in (('',current),('preview_',~current)):
        for index,name in enumerate(('position','velocity','acceleration')):
            value=float(np.max(abs(actual[mask,3+3*index:6+3*index]-wanted[mask,3+3*index:6+3*index])))
            if value>2e-12:raise ValueError('Independent reference differs: '+prefix+name)
            errors[prefix+name]=value
    return errors


def c2_checks():
    law=figure_eight([1.2,-.3,2.1]);jumps=[]
    for t in (2.,4.,18.,20.):
        jump=float(np.max(abs(np.concatenate(law.sample(t-1e-8))-np.concatenate(law.sample(t+1e-8)))))
        if jump>1e-7:raise ValueError('Original C2 boundary differs')
        jumps.append(dict(t=t,epsilon_s=1e-8,maximum_pva_difference=jump))
    return jumps


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--probe',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.mkdir(parents=True,exist_ok=False)
    sources=[Path(__file__),ORACLE,ROOT/'ros2_ws/src/flyarm_mpc_cpp/include/flyarm_mpc_cpp/trajectory_plan.hpp',
             ROOT/'ros2_ws/src/flyarm_state_cpp/include/flyarm_state_cpp/source_core.hpp',
             ROOT/'ros2_ws/src/flyarm_mpc_cpp/test/trajectory_probe.cpp']
    report=dict(kind='cpp_fast_reference_preview_v1',trajectory=TRAJECTORY,passed=False,complete=False,
        source_artifacts=[identity(x) for x in sources],binary=identity(a.probe),physical_acceptance=False)
    try:
        cases=corpus();(a.output/'cases.json').write_text(json.dumps(cases),encoding='utf-8')
        text=input_text(cases);(a.output/'corpus.txt').write_text(text,encoding='ascii')
        result=subprocess.run([str(a.probe)],input=text,text=True,capture_output=True,timeout=90)
        (a.output/'probe_stdout.txt').write_text(result.stdout,encoding='ascii')
        if result.returncode:raise RuntimeError(result.stderr)
        actual=np.loadtxt(a.output/'probe_stdout.txt')
        report.update(maximum_absolute_error=compare(actual,cases),c2_boundaries=c2_checks(),
            case_count=len(cases),target_sample_count=len(actual),actual_dt_s=sorted({c['dt'] for c in cases}),
            time_tolerance=0.,pva_tolerance=2e-12,cases=identity(a.output/'cases.json'),
            corpus=identity(a.output/'corpus.txt'),actual_output=identity(a.output/'probe_stdout.txt'))
        if report['source_artifacts']!=[identity(x) for x in sources] or report['binary']!=identity(a.probe):
            raise ValueError('Reference sources or compiled probe changed')
        report.update(passed=True,complete=True)
    except Exception as error:report['error']=repr(error)
    (a.output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps(report,indent=2));return 0 if report['passed'] else 1


if __name__=='__main__':raise SystemExit(main())
