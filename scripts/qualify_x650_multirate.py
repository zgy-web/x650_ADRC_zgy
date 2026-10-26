#!/usr/bin/env python3
"""Create new timing inputs and software evidence before one physical stage.

Runs native/contract tests and a real private-domain ROS worker, never a flight.
The resulting qualification manifest is consumed by the separate flight launcher.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from probe_x650_recovery_worker import identity,write_new,require
from simulations.x650_cpp_multirate_v1 import task,profile,qualification
from simulations.x650_cpp_live_v1.launch import runtime_hashes


def execute(command, log):
    with Path(log).open('x') as stream:
        result=subprocess.run(list(map(str,command)),cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
    require(result.returncode==0,'Software check failed; inspect '+str(log))
    return identity(log)


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage',type=Path,required=True)
    p.add_argument('--case',choices=['tip_hold','wrench_fast'],required=True)
    p.add_argument('--hold-certificate',type=Path)
    p.add_argument('--input-name',default='inputs_v1')
    a=p.parse_args();stage=a.stage.resolve(strict=True)
    out=stage/(a.case+'_'+a.input_name);out.mkdir()
    prior=ROOT/'artifacts/cpp_interval_flight_20261026/inputs_v1'
    old=json.loads((prior/'task.json').read_text())
    # Task bindings use the strict path/SHA schema; evidence identities also
    # contain bytes and are retained separately in the admission manifest.
    binding=old['frozen_cpp_hold'] if a.case=='tip_hold' else {k:v for k,v in identity(prior/'task.json').items() if k!='bytes'}
    hold_binding={k:v for k,v in identity(a.hold_certificate).items() if k!='bytes'} if a.hold_certificate else None
    cfg=task.build_task_config(a.case,binding,hold_certificate=hold_binding)
    write_new(out/'task.json',cfg)
    protocol=task.protocol(cfg);old_protocol=json.loads((prior/'protocol.json').read_text())
    for key in qualification.RESOURCE_KEYS:
        if key in old_protocol:protocol[key]=old_protocol[key]
    write_new(out/'protocol.json',protocol)
    startup=json.loads((prior/'startup_manifest.json').read_text())
    startup.update(timing_profile=profile.NAME,task_config={k:v for k,v in identity(out/'task.json').items() if k!='bytes'},trajectory=task.trajectory(cfg))
    startup['config'].update(max_dt=.1,command_hold_s=.16)
    for role in ('core','optimization'):
        path=stage/'build'/('libx650_mpc_core.so' if role=='core' else 'libx650_optimization.so')
        startup[role+'_library']={k:v for k,v in identity(path).items() if k!='bytes'}
    sources=[dict(identity(ROOT/name),relative_path=name) for name in sorted(qualification.EXECUTION_SOURCES)]
    startup['source_artifacts']=[{k:v for k,v in row.items() if k in ('path','sha256')} for row in sources]
    write_new(out/'startup_manifest.json',startup)
    binaries=runtime_hashes(startup)
    worker=next(Path(p) for p in binaries if Path(p).name=='isolated_worker')
    libraries={role:identity(startup[field]['path']) for role,field in qualification.LIBRARY_FIELDS.items()}
    # Each test output and binary is retained. Native CTest test names distinguish
    # baseline numerical preservation from the extended holding-time guard.
    suites=[('native',stage/'build',{'legacy_default_equivalence':'legacy_default_equivalence','extended_dt_hold_guard':'extended_dt_hold_guard'}),
            ('gateway',stage/'ros/build/flyarm_control_gateway',{'multirate_cadence':'named_multirate_timing_profile','lease_repetition':'named_multirate_timing_profile','timeout_and_stale_source':'named_multirate_timing_profile'})]
    reports={}
    for kind,build,required in suites:
        tests=json.loads(subprocess.check_output(['ctest','--test-dir',str(build),'--show-only=json-v1'],text=True))['tests']
        catalog={t['name']:t for t in tests};done={};rows=[]
        for label,name in required.items():
            require(name in catalog,'Missing required compiled test: '+name)
            t=catalog[name]
            if name not in done:done[name]=execute(t['command'],out/(name+'.log'))
            rows.append(dict(name=label,passed=True,returncode=0,binary=identity(t['command'][0]),result=done[name],ctest_name=name))
        report=dict(kind='x650_cpp_multirate_'+kind+'_acceptance_v1',complete=True,passed=True,physical_acceptance=False,
            timing_profile=profile.NAME,effective_profile=profile.effective(),tests=rows)
        report['libraries' if kind=='native' else 'runtime_binaries']=libraries if kind=='native' else binaries
        write_new(out/(kind+'_tests.json'),report);reports[kind]=identity(out/(kind+'_tests.json'))
    execute([sys.executable,ROOT/'scripts/prepare_x650_multirate_recording.py','--recording',
        ROOT/'artifacts/cpp_safety_margin_20261026/recording429_v1/recording.json','--startup',out/'startup_manifest.json',
        '--output',out/'recording'],out/'derive.log')
    recording=identity(out/'recording/recording.json')
    execute(['taskset','-c','4',stage/'x650_cpp_multirate_sequence',libraries['predictor']['path'],libraries['predictor']['sha256'],
        recording['path'],recording['sha256'],out/'oracle'],out/'oracle.log')
    oracle=identity(out/'oracle/report.json');s=identity(out/'startup_manifest.json');w=identity(worker)
    execute([sys.executable,ROOT/'scripts/probe_x650_multirate_worker.py','--recording',recording['path'],'--recording-sha256',recording['sha256'],
        '--startup',s['path'],'--startup-sha256',s['sha256'],'--worker',w['path'],'--worker-sha256',w['sha256'],
        '--oracle',oracle['path'],'--oracle-sha256',oracle['sha256'],'--output',out/'worker_probe','--domain-id','211',
        '--executor-cpu','4','--auxiliary-cpus','6,8,10','--probe-cpu','18'],out/'probe.log')
    software=dict(kind='x650_cpp_multirate_software_acceptance_v1',core_abi=4,complete=True,passed=True,physical_acceptance=False,
        timing_profile=profile.NAME,effective_profile=profile.effective(),code=dict(libraries=libraries,worker=w,runtime_binaries=binaries,sources=sources),
        native_tests=reports['native'],gateway_tests=reports['gateway'],ros_worker_probe=identity(out/'worker_probe/report.json'))
    write_new(out/'software_acceptance.json',software)
    historical=json.loads((prior/'qualification_manifest.json').read_text())['original_robust_qualification']
    manifest=dict(kind='x650_cpp_multirate_qualification_v1',simulation_only=True,
        task_config=identity(out/'task.json'),startup_manifest=s,protocol=identity(out/'protocol.json'),
        multirate_software_acceptance=identity(out/'software_acceptance.json'),historical_qualification=historical,
        prior_multirate_hold_acceptance=hold_binding)
    write_new(out/'qualification_manifest.json',manifest)
    print(json.dumps(dict(inputs=str(out),startup_sha256=s['sha256'],software_passed=True,physical_acceptance=False),indent=2))


if __name__=='__main__':main()
