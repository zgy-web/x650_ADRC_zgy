#!/usr/bin/env python3
"""Admit a fresh mass-mismatch case using the unchanged accepted 25/50 Hz code.

No compilation or flight occurs. Previously verified native/gateway tests are
reused only for the identical library/runtime hashes; each new task gets its own
derived source records, sequential C++ oracle and actual private-domain worker
probe. Historical nominal/hold flights never replace this case's physical audit.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/'scripts')]
from probe_x650_recovery_worker import identity,write_new,require
from simulations.x650_cpp_multirate_robustness_v1 import task,qualification
from simulations.x650_cpp_multirate_v1 import profile,qualification as accepted
from simulations.x650_cpp_live_v1.launch import runtime_hashes


def read(path):return json.loads(Path(path).read_text())


def binding(path):
    """Strict task bindings intentionally exclude the evidence-only byte count."""
    return {k:v for k,v in identity(path).items() if k in ('path','sha256')}


def execute(command,log):
    with Path(log).open('x') as stream:
        result=subprocess.run(list(map(str,command)),cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT)
    require(result.returncode==0,'Software check failed; inspect '+str(log))
    return identity(log)


def inherit_startup(original,task_binding,trajectory,sources):
    """Bind a new task without silently changing any native configuration."""
    require(original.get('kind')=='cpp_full_pipeline_v1'
        and original.get('simulation_only') is True and original.get('timing_profile')==profile.NAME,
        'Accepted 25/50 Hz complete-C++ startup required')
    value=deepcopy(original)
    value.update(task_config=deepcopy(task_binding),trajectory=deepcopy(trajectory),
        source_artifacts=[{k:v for k,v in item.items() if k in ('path','sha256')} for item in sources])
    allowed={'task_config','trajectory','source_artifacts'}
    require({k:v for k,v in value.items() if k not in allowed}
        =={k:v for k,v in original.items() if k not in allowed},
        'New scenario unexpectedly changed numerical configuration or library binding')
    return value


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--case',choices=['mass085_fast','mass115_fast'],required=True)
    p.add_argument('--original-inputs',type=Path,required=True)
    p.add_argument('--accepted-inputs',type=Path,required=True)
    p.add_argument('--hold-certificate',type=Path,required=True)
    p.add_argument('--sequence-executable',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--recording',type=Path,
        default=ROOT/'artifacts/cpp_safety_margin_20261026/recording429_v1/recording.json')
    p.add_argument('--domain-id',type=int,default=211)
    a=p.parse_args()
    prior=a.original_inputs.resolve(strict=True);tested=a.accepted_inputs.resolve(strict=True)
    sequence=a.sequence_executable.resolve(strict=True);out=a.output.resolve()
    require(not out.exists() and out.is_absolute(), 'A new absolute qualification output is required')
    require(0<=a.domain_id<=232,'ROS domain outside explicit private test range')
    out.mkdir(parents=True)
    original_task=read(prior/'task.json');historical=read(prior/'qualification_manifest.json')
    require(original_task['robustness']['case_id']==a.case
        and historical['kind']=='x650_cpp_robustness_qualification_v1'
        and historical['task_config']['sha256']==identity(prior/'task.json')['sha256'],
        'Original physical qualification/task belongs to another case')
    hold_binding=binding(a.hold_certificate)
    cfg=task.build_task_config(a.case,binding(prior/'task.json'),hold_certificate=hold_binding)
    write_new(out/'task.json',cfg)
    protocol=task.protocol(cfg);tested_protocol=read(tested/'protocol.json')
    for key in accepted.RESOURCE_KEYS:
        if key in tested_protocol:protocol[key]=tested_protocol[key]
    qualification.validate_protocol(protocol,cfg)
    write_new(out/'protocol.json',protocol)
    sources=[dict(identity(ROOT/name),relative_path=name) for name in sorted(qualification.EXECUTION_SOURCES)]
    startup=inherit_startup(read(tested/'startup_manifest.json'),binding(out/'task.json'),task.trajectory(cfg),sources)
    write_new(out/'startup_manifest.json',startup)
    binaries=runtime_hashes(startup)
    old_software=read(tested/'software_acceptance.json')
    require(old_software['code']['runtime_binaries']==binaries,
        'Actual runtime binary differs from accepted timing implementation; no rebuild permitted')
    # Fully verify the original native/gateway/worker evidence before reusing
    # only its native/gateway checks for byte-identical installed executables.
    old_startup=read(tested/'startup_manifest.json')
    original_software_proof=accepted.verify_software(old_software,old_startup,accepted.Evidence(),binaries)
    hold_proof=qualification.verify_hold_acceptance(hold_binding,cfg,binaries=binaries,startup=startup)
    write_new(out/'unchanged_backend_binding.json',dict(kind='x650_cpp_multirate_unchanged_backend_v1',
        verified=True,recompiled=False,algorithm_changed=False,physical_acceptance=False,
        original_software=identity(tested/'software_acceptance.json'),original_software_proof=original_software_proof,
        prior_hold_preparation=hold_proof,runtime_binaries=binaries,sequence_executable=identity(sequence),
        numerical_configuration_equal=(startup['config']==old_startup['config'])))
    worker=next(Path(path) for path in binaries if Path(path).name=='isolated_worker')
    libraries={role:identity(startup[field]['path']) for role,field in accepted.LIBRARY_FIELDS.items()}
    require(libraries==old_software['code']['libraries'], 'Actual native libraries differ from accepted tests')
    execute([sys.executable,ROOT/'scripts/prepare_x650_multirate_recording.py','--recording',a.recording.resolve(strict=True),
        '--startup',out/'startup_manifest.json','--output',out/'recording'],out/'derive.log')
    recording=identity(out/'recording/recording.json')
    execute(['taskset','-c','4',sequence,libraries['predictor']['path'],libraries['predictor']['sha256'],
        recording['path'],recording['sha256'],out/'oracle'],out/'oracle.log')
    oracle=identity(out/'oracle/report.json');s=identity(out/'startup_manifest.json');w=identity(worker)
    execute([sys.executable,ROOT/'scripts/probe_x650_multirate_worker.py','--recording',recording['path'],
        '--recording-sha256',recording['sha256'],'--startup',s['path'],'--startup-sha256',s['sha256'],
        '--worker',w['path'],'--worker-sha256',w['sha256'],'--oracle',oracle['path'],'--oracle-sha256',oracle['sha256'],
        '--output',out/'worker_probe','--domain-id',str(a.domain_id),
        '--executor-cpu','4','--auxiliary-cpus','6,8,10','--probe-cpu','18'],out/'probe.log')
    software=dict(kind='x650_cpp_multirate_software_acceptance_v1',core_abi=4,complete=True,passed=True,physical_acceptance=False,
        timing_profile=profile.NAME,effective_profile=profile.effective(),
        code=dict(libraries=libraries,worker=w,runtime_binaries=binaries,sources=sources),
        native_tests=old_software['native_tests'],gateway_tests=old_software['gateway_tests'],
        ros_worker_probe=identity(out/'worker_probe/report.json'))
    write_new(out/'software_acceptance.json',software)
    manifest=dict(kind='x650_cpp_multirate_robustness_qualification_v1',simulation_only=True,
        task_config=identity(out/'task.json'),startup_manifest=s,protocol=identity(out/'protocol.json'),
        multirate_software_acceptance=identity(out/'software_acceptance.json'),
        historical_qualification=identity(prior/'qualification_manifest.json'),prior_multirate_hold_acceptance=hold_binding)
    write_new(out/'qualification_manifest.json',manifest)
    proof=qualification.verify(out/'qualification_manifest.json',task_path=out/'task.json',startup_path=out/'startup_manifest.json',
        startup_sha256=s['sha256'],protocol_path=out/'protocol.json',binaries=binaries)
    write_new(out/'admission_report.json',proof)
    print(json.dumps(dict(inputs=str(out),case_id=a.case,startup_sha256=s['sha256'],
        software_passed=proof['ready'],physical_acceptance=False,recompiled=False,algorithm_changed=False),indent=2))


if __name__=='__main__':main()
