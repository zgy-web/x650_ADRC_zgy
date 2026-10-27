#!/usr/bin/env python3
"""Preserve closed synchronous PIL runs, including failed preparations/audits."""
import argparse
import json
from pathlib import Path

from archive_x650_robustness_raw import build_archives,files_below,identity,read

ROOT=Path(__file__).resolve().parents[1]


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();stage=a.stage.resolve(strict=True);out=a.output.resolve()
    if out.exists() or not stage.is_relative_to(ROOT) or not out.is_relative_to(stage):
        raise ValueError('Use a new delivery directory under the project stage')
    if (ROOT/'.amsim/runtime.json').exists():raise RuntimeError('Close the simulation before archiving')
    paths=files_below(stage);launches=[]
    for file in sorted(stage.glob('*/launch.json')):
        launch=read(file)
        if not launch.get('runtime_closed'):raise ValueError('An owned runtime is still open')
        launches.append(identity(file))
        for name in launch.get('run_directories',[]):
            run=Path(name).resolve(strict=True)
            if run.parent!=ROOT/'runs' or not run.name.startswith('x650_nx_sync_pil_v1_'):
                raise ValueError('Unexpected physical run path')
            paths.extend(files_below(run))
    for pattern in ('scripts/*x650*pil*.py','scripts/probe_x650_sync_clock.py',
                    'scripts/restore_x650_parameter_journal.py','simulations/x650_nx_sync_pil_v1/*.py',
                    'scenarios/x650_nx_sync_pil_v1.yaml','control_packages/x650_wholebody_v1/pil/*'):
        paths.extend(p for p in ROOT.glob(pattern) if p.is_file())
    out.mkdir()
    result=build_archives(paths,root=ROOT,output_directory=out,prefix='nx_sync_pil_evidence',
        context={'scope':'Synchronous physical simulation with actual NX; failures retained; no wall-clock or hardware-flight qualification',
                 'launches':launches},max_bytes=48*1024*1024)
    print(json.dumps({k:result[k] for k in ('complete','verified_member_count','archives')},indent=2))


if __name__=='__main__':main()
