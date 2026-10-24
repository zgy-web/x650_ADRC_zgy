#!/usr/bin/env python3
"""Archive closed robustness evidence; verify every original and ZIP member.

Run with ``taskset -c 0 nice -n 10`` on the simulation server. Cases are copied
only after their physical lifecycle and audit have closed. Large archives are
split into independently readable ZIPs; original evidence is never deleted.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
import zipfile
import zlib

ROOT = Path(__file__).resolve().parents[1]
CASES = ('wrench_fast','mass085_fast','mass115_fast')
MAX_BYTES = 90*1024*1024
BUFFER_BYTES = 1024*1024
TEXT_SUFFIXES = {'.sdf','.config','.urdf','.json','.yaml','.yml','.xml','.txt'}


def sha(path):
    h = hashlib.sha256()
    with Path(path).open('rb') as stream:
        for data in iter(lambda: stream.read(BUFFER_BYTES), b''):
            h.update(data)
    return h.hexdigest()


def identity(path):
    p = Path(path).resolve(strict=True)
    return {'path':str(p),'bytes':p.stat().st_size,'sha256':sha(p)}


def read(path):
    return json.loads(Path(path).read_text(encoding='utf-8'))


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix(path.suffix+'.tmp')
    with temporary.open('x', encoding='utf-8') as stream:
        json.dump(value,stream,indent=2,ensure_ascii=False,allow_nan=False)
        stream.write('\n')
    temporary.rename(path)


def files_below(directory):
    directory = Path(directory).resolve(strict=True)
    paths = []
    for path in sorted(directory.rglob('*')):
        if path.is_symlink() or not path.resolve().is_relative_to(directory):
            raise ValueError('Evidence contains an unsafe symbolic link: '+str(path))
        if path.is_file() and '__pycache__' not in path.parts:
            paths.append(path)
    return paths


def snapshot_file(path, root):
    path,root = Path(path).resolve(strict=True),Path(root).resolve(strict=True)
    if not path.is_file() or path.is_symlink() or not path.is_relative_to(root):
        raise ValueError('Only regular project evidence files may be archived: '+str(path))
    before = path.stat()
    digest = hashlib.sha256(); compressor = zlib.compressobj(level=6,wbits=-15)
    compressed, size = 0,0
    with path.open('rb') as stream:
        for data in iter(lambda: stream.read(BUFFER_BYTES), b''):
            digest.update(data);size+=len(data);compressed+=len(compressor.compress(data))
    compressed += len(compressor.flush())
    after = path.stat()
    if (before.st_size,before.st_mtime_ns) != (after.st_size,after.st_mtime_ns) or size!=after.st_size:
        raise RuntimeError('Source changed while snapshotting: '+str(path))
    return {'source':str(path),'archive_entry':path.relative_to(root).as_posix(),
            'bytes':size,'sha256':digest.hexdigest(),'compressed_size_estimate':compressed}


def partition_files(files, max_bytes):
    # Reserve enough for the full manifest embedded in each ZIP. Each file's
    # compressed size was measured using the same raw-deflate level as zipfile.
    reserve = min(2*1024*1024,max_bytes//4)
    groups, group, used = [],[],reserve
    for item in files:
        estimate=item['compressed_size_estimate']+2*len(item['archive_entry'].encode())+512
        if estimate+reserve > max_bytes:
            raise RuntimeError('An individual compressed file exceeds the archive cap; preserve it and choose explicit byte fragments: '+item['source'])
        if group and used+estimate > max_bytes:
            groups.append(group);group=[];used=reserve
        group.append(item);used+=estimate
    if group:groups.append(group)
    return groups


def verify_archive(archive, items, manifest_bytes):
    expected={item['archive_entry']:item for item in items}
    with zipfile.ZipFile(archive) as z:
        if len(z.namelist()) != len(set(z.namelist())):
            raise RuntimeError('Duplicate archive members')
        if set(z.namelist()) != set(expected)|{'_archive_manifest.json'}:
            raise RuntimeError('Archive file set mismatch')
        if z.read('_archive_manifest.json') != manifest_bytes:
            raise RuntimeError('Embedded manifest bytes differ')
        for name,item in expected.items():
            digest=hashlib.sha256();size=0
            with z.open(name) as stream:
                for data in iter(lambda:stream.read(BUFFER_BYTES),b''):
                    digest.update(data);size+=len(data)
            if size!=item['bytes'] or digest.hexdigest()!=item['sha256']:
                raise RuntimeError('Archive member changed: '+name)
    return len(expected)


def build_archives(paths, *, root, output_directory, prefix, context, max_bytes=MAX_BYTES):
    root,output_directory=Path(root).resolve(strict=True),Path(output_directory).resolve(strict=True)
    manifest_path=output_directory/(prefix+'_manifest.json')
    receipt_path=output_directory/(prefix+'_receipt.json')
    if manifest_path.exists() or receipt_path.exists() or list(output_directory.glob(prefix+'*.zip')):
        raise FileExistsError('Refusing to replace an existing evidence archive: '+str(output_directory/prefix))
    entries=[snapshot_file(p,root) for p in sorted(set(map(Path,paths)))]
    if not entries:raise ValueError('Empty raw evidence archive')
    groups=partition_files(entries,max_bytes)
    for index,group in enumerate(groups):
        name=prefix+('.zip' if len(groups)==1 else '_part'+str(index+1).zfill(2)+'.zip')
        for item in group:item['archive']=name
    manifest={'kind':'x650_robustness_raw_archive_manifest_v1','context':context,
              'original_files':entries,'original_file_count':len(entries),
              'original_bytes':sum(x['bytes'] for x in entries),
              'archive_count':len(groups),'archive_size_cap_bytes':max_bytes,
              'source_root':str(root),'archiver':identity(Path(__file__)),
              'policy':'Original evidence is immutable; every member SHA/size and embedded manifest are read back; omitted inherited meshes are separately hash-listed.'}
    manifest_bytes=(json.dumps(manifest,indent=2,ensure_ascii=False,allow_nan=False)+'\n').encode()
    pending=[]; archives=[];verified=0
    for group in groups:
        final=output_directory/group[0]['archive'];temporary=final.with_suffix('.zip.tmp')
        with zipfile.ZipFile(temporary,'x',compression=zipfile.ZIP_DEFLATED,compresslevel=6,allowZip64=True) as z:
            z.writestr('_archive_manifest.json',manifest_bytes)
            for item in group:
                z.write(item['source'],item['archive_entry'])
        if temporary.stat().st_size>max_bytes:
            raise RuntimeError('Archive exceeds explicit cap; original evidence preserved: '+str(temporary))
        verified+=verify_archive(temporary,group,manifest_bytes)
        pending.append((temporary,final))
    # Check the actual originals again after all ZIPs were fully read back.
    for item in entries:
        actual=identity(item['source'])
        if actual['bytes']!=item['bytes'] or actual['sha256']!=item['sha256']:
            raise RuntimeError('Original evidence changed during archive verification: '+item['source'])
    with manifest_path.open('xb') as stream:stream.write(manifest_bytes)
    for temporary,final in pending:
        temporary.rename(final);archives.append(identity(final))
    receipt={'kind':'x650_robustness_raw_archive_receipt_v1','complete':True,
             'all_original_sizes_and_sha256_verified':True,'all_zip_members_read_back':True,
             'embedded_manifests_byte_identical':True,'verified_member_count':verified,
             'manifest':identity(manifest_path),'archives':archives,'context':context,
             'completed_unix':time.time()}
    atomic_json(receipt_path,receipt)
    return receipt


def current_sources(root):
    paths=[]
    for folder in ('simulations/x650_cpp_robustness_v1',):
        paths.extend(files_below(root/folder))
    for pattern in ('scripts/*x650*robust*.py','tests/unit/*x650*robust*.py',
                    'configs/experiments/*x650*robust*.json','scenarios/*x650*robust*.yaml'):
        paths.extend(root.glob(pattern))
    return sorted(set(paths))


def case_ready(batch, case):
    directory=batch/case
    if not (directory/'physical_run_v1.exit').is_file() or not (batch/'batch_result_v1.json').is_file():return None
    entries=[x for x in read(batch/'batch_result_v1.json') if x.get('case_id')==case]
    if len(entries)!=1:return None
    result=entries[0]
    if int((directory/'physical_run_v1.exit').read_text().strip()) != result['run_exit']:
        raise RuntimeError('Physical exit and batch result differ')
    if result['run_exit']==0:
        if not (directory/'audit_v1.exit').is_file() or not (directory/'audit_v1/report.json').is_file():return None
        if int((directory/'audit_v1.exit').read_text().strip()) != result.get('audit_exit'):
            raise RuntimeError('Audit exit and batch result differ')
    if 'launch' in result:
        launch=read(Path(result['launch'])/'launch.json')
        if launch.get('runtime_closed') is not True:return None
        # The wrapper can fail post-flight identity checks after an underlying
        # scenario returned zero. Preserve both observed exits for failures.
        if result['run_exit']==0 and launch.get('exit_code')!=0:
            raise RuntimeError('Successful wrapper disagrees with launch exit')
    return result


def model_evidence(root, cfg):
    directory=root/'generated/imported_models'/cfg['model']
    paths=[];omitted=[]
    for path in files_below(directory):
        if path.suffix.lower() in TEXT_SUFFIXES or path.name=='model.config':paths.append(path)
        else:omitted.append(identity(path))
    return paths,omitted


def archive_case(root,batch,case):
    result=case_ready(batch,case)
    if result is None:return None
    directory=batch/case;cfg=read(directory/'task.json')
    paths=[p for p in files_below(directory) if not p.name.startswith('raw_v1')]
    for key in ('launch',):
        if key in result:paths.extend(files_below(result[key]))
    for run in result.get('runs',[]):paths.extend(files_below(run))
    models,omitted=model_evidence(root,cfg);paths.extend(models);paths.extend(current_sources(root))
    paths.extend(root/name for name in ('scripts/archive_x650_robustness_raw.py',))
    audit=identity(directory/'audit_v1/report.json') if (directory/'audit_v1/report.json').exists() else None
    context={'case_id':case,'batch_result_entry':result,'case_task':identity(directory/'task.json'),
             'audit_report':audit,'physical_model':cfg['model'],
             'omitted_inherited_binary_resources':omitted,
             'omission_meaning':'All mesh/texture bytes remain available through the nominal model import manifest and prior accepted raw archive; actual model XML and full resource hashes are included.',
             'scope':'Exact closed simulation evidence, including failures if present; archive completion is not physical acceptance.'}
    return build_archives(paths,root=root,output_directory=directory,prefix='raw_v1',context=context)


def archive_support(root,batch):
    if not (batch/'batch_complete_v1.json').is_file() or not (batch/'run_batch_v1.exit').is_file():return None
    complete=read(batch/'batch_complete_v1.json')
    if complete.get('complete') is not True:return None
    if any(case_ready(batch,case) is None for case in CASES):return None
    paths=[p for p in batch.iterdir() if p.is_file() and not p.name.startswith(('support_raw_v1','archive_monitor'))]
    paths.extend(current_sources(root))
    return build_archives(paths,root=root,output_directory=batch,prefix='support_raw_v1',
        context={'scope':'Closed batch commands, model preparation successes and preserved failures, independent static probes, new source and regression tests; no other case raw directory is included.',
                 'batch_complete':identity(batch/'batch_complete_v1.json'),
                 'delay_tests_enabled':False})


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root',type=Path,default=ROOT)
    parser.add_argument('--batch',type=Path,required=True)
    parser.add_argument('--wait',action='store_true')
    args=parser.parse_args(argv)
    root,batch=args.root.resolve(strict=True),args.batch.resolve(strict=True)
    while True:
        for case in CASES:
            receipt=batch/case/'raw_v1_receipt.json'
            if receipt.exists():continue
            result=archive_case(root,batch,case)
            if result is not None:print(json.dumps({'case':case,'receipt':identity(receipt),'archives':result['archives']}),flush=True)
        support=batch/'support_raw_v1_receipt.json'
        if not support.exists():
            result=archive_support(root,batch)
            if result is not None:print(json.dumps({'support':identity(support),'archives':result['archives']}),flush=True)
        if support.exists() and all((batch/case/'raw_v1_receipt.json').exists() for case in CASES):return 0
        if not args.wait:return 2
        time.sleep(10)


if __name__=='__main__':
    raise SystemExit(main())
