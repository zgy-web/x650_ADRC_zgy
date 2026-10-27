#!/usr/bin/env python3
"""Build explicit equal-condition upper-controller inputs and run software tests."""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'scripts')]
from simulations.x650_upper_nmpc_v1 import task, qualification as q
from simulations.x650_upper_nmpc_v1.launch import runtime_hashes, source_hashes


def write(path, value):
    with Path(path).open('x', encoding='utf-8') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def binding(path):
    return {k:v for k,v in q.identity(path).items() if k != 'bytes'}


def command(cmd, log):
    with Path(log).open('x') as stream:
        result = subprocess.run(list(map(str,cmd)), cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT)
    q.require(result.returncode == 0, 'Software check failed: '+str(log))


def suite(build, out, name):
    tests = json.loads(subprocess.check_output(['ctest','--test-dir',str(build),'--show-only=json-v1'],text=True))['tests']
    q.require(tests, 'Empty test suite '+str(build))
    rows = []
    for i, test in enumerate(tests):
        log = out/(name+'_'+str(i)+'.log')
        command(test['command'], log)
        rows.append(dict(name=test['name'],returncode=0,binary=q.identity(test['command'][0]),log=q.identity(log)))
    path=out/(name+'_tests.json')
    write(path, dict(passed=True,tests=rows,physical_acceptance=False))
    return q.identity(path)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage',type=Path,required=True)
    p.add_argument('--case',choices=['hold','normal','fast'],required=True)
    p.add_argument('--backend',choices=['nmpc','mpc_callable'],required=True)
    p.add_argument('--prior',type=Path)
    p.add_argument('--input-name',default='inputs_v1')
    p.add_argument('--prepare-only',action='store_true')
    p.add_argument('--worker-probe',type=Path)
    a=p.parse_args(); stage=a.stage.resolve(strict=True)
    out=stage/(a.backend+'_'+a.case+'_'+a.input_name)
    out.mkdir()
    prior=ROOT/'artifacts/cpp_multirate_20261026/tip_hold_inputs_v5'
    cfg=task.build_task_config(a.case,a.backend,binding(prior/'task.json'),
        prior_stage_certificate=binding(a.prior) if a.prior else None)
    write(out/'task.json',cfg)
    protocol=task.protocol(cfg)
    old_protocol=q.read(prior/'protocol.json')
    for key in q.RESOURCE_KEYS:
        if key in old_protocol: protocol[key]=old_protocol[key]
    write(out/'protocol.json',protocol)
    startup=q.read(prior/'startup_manifest.json')
    startup.update(task_config=binding(out/'task.json'),trajectory=task.trajectory(cfg))
    # Bind sources actually used now, instead of borrowing historical hashes.
    paths=source_hashes(('simulations/x650_upper_nmpc_v1',
        'control_packages/x650_nmpc_controller_v1','control_packages/x650_mpc_controller_v1',
        'simulations/'+cfg['scenario'],'scenarios/'+cfg['scenario']+'.yaml'))
    startup['source_artifacts']=[dict(path=str(ROOT/name),sha256=digest) for name,digest in paths.items()]
    write(out/'startup_manifest.json',startup)
    binaries=runtime_hashes(startup)
    worker=next(Path(v) for v in binaries if Path(v).name=='isolated_worker')
    options=dict(schema_version=1,kind='x650_online_nmpc_options_v1',simulation_only=True,
        backend=a.backend,task_config_sha256=q.sha(out/'task.json'),worker_executable=binding(worker),
        options=dict(max_proposals=4,absolute_decrease=1e-10,relative_decrease=1e-7,proposal_budget_ms=40,total_budget_ms=50),
        source_artifacts=startup['source_artifacts'])
    write(out/'options.json',options)
    if a.prepare_only:
        print(json.dumps(dict(inputs=str(out),prepared=True,software_qualified=False)))
        return
    suites=[suite(stage/'native_build',out,'native'),suite(stage/'ros/build/flyarm_nmpc_cpp',out,'worker')]
    suites.append(suite(ROOT/'artifacts/cpp_multirate_20261026/ros/build/flyarm_control_gateway',out,'gateway'))
    if a.worker_probe:
        probe=a.worker_probe.resolve(strict=True)
    else:
        probe=out/'worker_probe/report.json'
        command([sys.executable,ROOT/'scripts/probe_x650_upper_worker.py',
            '--startup',out/'startup_manifest.json','--options',out/'options.json',
            '--worker',worker,'--output',probe.parent,
            '--recording',prior/'recording/recording.json'],out/'worker_probe.log')
    software=dict(kind='x650_upper_software_acceptance_v1',passed=True,complete=True,
        physical_acceptance=False,runtime_binaries=binaries,suites=suites,worker_probe=q.identity(probe))
    write(out/'software_acceptance.json',software)
    manifest=dict(kind='x650_upper_qualification_v1',simulation_only=True,
        task_config=q.identity(out/'task.json'),startup_manifest=q.identity(out/'startup_manifest.json'),
        protocol=q.identity(out/'protocol.json'),options=q.identity(out/'options.json'),runtime_binaries=binaries,
        software=q.identity(out/'software_acceptance.json'))
    write(out/'qualification_manifest.json',manifest)
    result=q.verify(out/'qualification_manifest.json',task_path=out/'task.json',
        startup_path=out/'startup_manifest.json',startup_sha256=q.sha(out/'startup_manifest.json'),
        protocol_path=out/'protocol.json',options_path=out/'options.json',binaries=binaries)
    print(json.dumps(dict(inputs=str(out),ready=result['ready'],physical_acceptance=False)))


if __name__=='__main__':main()
