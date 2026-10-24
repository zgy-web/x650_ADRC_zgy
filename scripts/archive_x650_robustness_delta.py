#!/usr/bin/env python3
"""Create a separate post-run evidence archive after an explicit ready marker.

Initial raw_v1 archives stay untouched. The marker lists closed evidence paths
relative to the batch directory; inherited meshes and prior ZIPs are excluded.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path, PurePosixPath
import sys
import time

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(Path(__file__).resolve().parent))
from archive_x650_robustness_raw import (CASES, build_archives, current_sources,
                                        files_below, identity, read)

CACHE_NAMES={'mesh_cache','geometry_cache','mesh-cache','geometry-cache',
             '.mesh_cache','.geometry_cache','__pycache__'}


def marker_paths(batch,marker):
    document=read(marker)
    if document.get('complete') is not True:
        raise ValueError('Explicit final evidence readiness marker is required')
    names=document.get('paths')
    if not isinstance(names,list) or not names or len(set(names))!=len(names):
        raise ValueError('Ready marker requires an explicit unique evidence path list')
    result=[]
    for name in names:
        if not isinstance(name,str) or not name or '\\' in name or ':' in name:
            raise ValueError('Evidence paths must be canonical relative POSIX paths')
        relative=PurePosixPath(name)
        if relative.is_absolute() or relative.as_posix()!=name or any(x in ('.','..') for x in relative.parts):
            raise ValueError('Evidence selection may not escape its batch directory')
        path=batch.joinpath(*relative.parts)
        if path.is_symlink() or not path.resolve(strict=True).is_relative_to(batch):
            raise ValueError('Evidence selection may not use symlinks or escape its batch directory')
        result.append(path)
    return document,result


def selected_files(batch,selected):
    files=[];excluded=[]
    for item in selected:
        for path in (files_below(item) if item.is_dir() else [item]):
            relative=path.relative_to(batch)
            if (any(x in CACHE_NAMES for x in relative.parts)
                    or path.name.startswith(('raw_v1','support_raw_v1','support_delta_v2'))
                    or (path.suffix=='.zip' and ('raw' in path.stem))):
                excluded.append({'path':str(path),'bytes':path.stat().st_size,
                                 'reason':'prior raw archive or generated geometry cache'})
            else:files.append(path)
    if not files:raise ValueError('No selected post-run evidence files remain')
    return sorted(set(files)),excluded


def base_receipts(batch):
    results=[]
    for directory,prefix in [(batch/case,'raw_v1') for case in CASES]+[(batch,'support_raw_v1')]:
        path=directory/(prefix+'_receipt.json');doc=read(path)
        if doc.get('complete') is not True or doc.get('all_zip_members_read_back') is not True:
            raise ValueError('Original raw archive verification is incomplete: '+str(path))
        for archive in doc['archives']:
            if identity(archive['path'])!=archive:
                raise ValueError('Original raw archive changed: '+archive['path'])
        if identity(doc['manifest']['path'])!=doc['manifest']:
            raise ValueError('Original raw manifest changed')
        results.append({'receipt':identity(path),'manifest':doc['manifest'],'archives':doc['archives']})
    return results


def make_delta(root,batch,marker):
    root,batch,marker=root.resolve(strict=True),batch.resolve(strict=True),marker.resolve(strict=True)
    if (root/'.amsim/runtime.json').exists():
        raise RuntimeError('No post-run finalization archive is created while a flight runtime is active')
    document,selected=marker_paths(batch,marker)
    for case in ('mass085_fast','mass115_fast'):
        report=batch/case/'audit_v2/report.json'
        if read(report).get('passed') is not True:
            raise ValueError('Expected completed second independent mass audit: '+str(report))
    originals=base_receipts(batch)
    paths,excluded=selected_files(batch,selected)
    paths.extend(current_sources(root));paths.append(marker)
    paths.append(root/'scripts/archive_x650_robustness_delta.py')
    context={'scope':'Separate post-run delta: repaired offline source-proof interface and re-audits of unchanged physical records; failed external-wrench analysis and final figures/results. No new flight is implied.',
             'ready_marker':identity(marker),'explicit_paths':document['paths'],
             'prior_raw_archives_verified_unchanged':originals,
             'excluded_generated_caches_and_prior_archives':excluded,
             'delay_tests_enabled':False,'original_raw_v1_archives_not_rewritten':True}
    receipt=build_archives(paths,root=root,output_directory=batch,prefix='support_delta_v2',context=context)
    if originals!=base_receipts(batch):
        raise RuntimeError('Original archive identity changed during post-run finalization')
    return receipt


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=ROOT)
    parser.add_argument('--batch',type=Path,required=True)
    parser.add_argument('--marker',type=Path)
    parser.add_argument('--wait',action='store_true')
    args=parser.parse_args(argv)
    marker=args.marker or args.batch/'final_outputs_ready.json'
    while not marker.is_file():
        if not args.wait:return 2
        time.sleep(10)
    receipt=make_delta(args.root,args.batch,marker)
    print(json.dumps(receipt,indent=2,allow_nan=False),flush=True)
    return 0


if __name__=='__main__':raise SystemExit(main())
