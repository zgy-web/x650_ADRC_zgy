#!/usr/bin/env python3
"""Independently stream-check delivered raw ZIPs without extracting originals."""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time
import zipfile

BUFFER=1024*1024


def digest_file(path):
    h=hashlib.sha256();size=0
    with Path(path).open('rb') as stream:
        for data in iter(lambda:stream.read(BUFFER),b''):
            h.update(data);size+=len(data)
    return {'path':str(Path(path).resolve()),'bytes':size,'sha256':h.hexdigest()}


def require(value,message):
    if not value:raise ValueError(message)


def verify_receipt(receipt_path):
    receipt_path=Path(receipt_path).resolve(strict=True)
    receipt=json.loads(receipt_path.read_bytes())
    require(receipt.get('complete') is True and receipt.get('all_zip_members_read_back') is True,
            'Incomplete original archive receipt')
    manifest_path=receipt_path.parent/Path(receipt['manifest']['path']).name
    manifest_bytes=manifest_path.read_bytes()
    manifest_digest=digest_file(manifest_path)
    require(manifest_digest['bytes']==receipt['manifest']['bytes']
            and manifest_digest['sha256']==receipt['manifest']['sha256'],'External manifest hash/size differs from receipt')
    manifest=json.loads(manifest_bytes)
    entries=manifest['original_files']
    require(len(entries)==manifest['original_file_count']==receipt['verified_member_count'],
            'Original file count is inconsistent')
    require(len(entries)==len({x['archive_entry'] for x in entries}),'Duplicate original manifest entries')
    original_total=sum(x['bytes'] for x in entries)
    require(original_total==manifest['original_bytes'],'Original byte total differs')
    groups={}
    for entry in entries:groups.setdefault(entry['archive'],{})[entry['archive_entry']]=entry
    require(set(groups)=={Path(x['path']).name for x in receipt['archives']},'Archive set differs from manifest')
    checked=[];member_count=0;member_bytes=0
    for archive in receipt['archives']:
        path=receipt_path.parent/Path(archive['path']).name
        actual=digest_file(path)
        require(actual['bytes']==archive['bytes'] and actual['sha256']==archive['sha256'],
                'Transferred ZIP hash/size mismatch: '+str(path))
        expected=groups[path.name];checks=[]
        with zipfile.ZipFile(path) as z:
            names=z.namelist()
            require(len(names)==len(set(names)),'ZIP has duplicate members')
            require(set(names)==set(expected)|{'_archive_manifest.json'},'ZIP member set differs')
            require(z.read('_archive_manifest.json')==manifest_bytes,'Embedded manifest bytes differ from external manifest')
            for name,item in expected.items():
                h=hashlib.sha256();size=0
                with z.open(name) as stream:
                    for data in iter(lambda:stream.read(BUFFER),b''):
                        h.update(data);size+=len(data)
                require(size==item['bytes'] and h.hexdigest()==item['sha256'],
                        'Delivered member bytes differ: '+name)
                checks.append({'entry':name,'bytes':size,'sha256':h.hexdigest(),'passed':True})
                member_count+=1;member_bytes+=size
        checked.append({'archive':actual,'embedded_manifest_matches':True,
                        'member_count':len(checks),'members':checks})
    require(member_count==len(entries) and member_bytes==original_total,'Member verification coverage is incomplete')
    return {'passed':True,'receipt':digest_file(receipt_path),'external_manifest':manifest_digest,
            'original_file_count':member_count,'original_bytes':member_bytes,'archives':checked}


def main(argv=None):
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--receipt',type=Path,action='append',required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args(argv)
    if args.output.exists():raise FileExistsError('Do not overwrite independent local verification evidence')
    results=[verify_receipt(p) for p in args.receipt]
    report={'kind':'x650_robustness_local_raw_verification_v1','passed':True,
            'independent_streaming_zip_member_verification':True,'originals_extracted':False,
            'all_embedded_manifests_match_external_bytes':True,
            'total_archive_count':sum(len(x['archives']) for x in results),
            'total_original_file_count':sum(x['original_file_count'] for x in results),
            'total_original_bytes':sum(x['original_bytes'] for x in results),
            'verifier':digest_file(Path(__file__)),'archives':results,'completed_unix':time.time()}
    args.output.write_text(json.dumps(report,indent=2,ensure_ascii=False,allow_nan=False)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in report.items() if k!='archives'},indent=2))


if __name__=='__main__':main()
