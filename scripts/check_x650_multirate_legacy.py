#!/usr/bin/env python3
"""Compare ABI4 legacy defaults against preserved ABI3 full numerical records."""
import argparse
import json
from pathlib import Path
import subprocess
from probe_x650_recovery_worker import identity,receipt,write_new,require

FIELDS=('A','b','H','g','C','lower','upper','solution','plan','predicted_states','predicted_tip',
        'predicted_rotations','candidate_states','candidate_envelope','horizon','query_times',
        'interval_canonical_states','interval_canonical_envelope','interval_constraints')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--stage',type=Path,required=True)
    p.add_argument('--regression',type=Path,required=True)
    a=p.parse_args();out=a.stage/'legacy_cross_version';out.mkdir()
    receipt(identity(a.regression));reg=json.loads(a.regression.read_text())
    original=json.loads(Path(receipt(reg['original_report'])['path']).read_text())
    predictor=Path('/root/autodl-tmp/libts20_prediction_v3.so');pred=identity(predictor)
    rows=[]
    for seq in (1,300,429):
        before_id=receipt(reg['rows'][seq-1]['raw']);before=json.loads(Path(before_id['path']).read_text())
        fixture=receipt(original['rows'][seq-1]['fixture']);path=out/f'{seq:06d}.json'
        result=subprocess.run(['taskset','-c','4',str(a.stage/'legacy_replay'),str(predictor),pred['sha256'],fixture['path'],fixture['sha256'],str(path)])
        require(result.returncode==0,'New legacy-default replay rejected')
        after=json.loads(path.read_text())
        require(before['core_abi']==3 and after['core_abi']==4,'Cross-version evidence missing')
        require(all(before[key]==after[key] for key in ('memory_before','memory_after','output')), 'Legacy Memory or complete result changed')
        require(all(before['diagnostics'][key]==after['diagnostics'][key] for key in FIELDS), 'Legacy matrix/prediction/interval values changed')
        rows.append(dict(sequence=seq,passed=True,exact_numeric_equality=True,original=before_id,fixture=fixture,actual=identity(path)))
    write_new(out/'report.json',dict(kind='x650_cpp_multirate_legacy_cross_version_v1',complete=True,passed=True,
        physical_acceptance=False,rows=rows,fields=list(FIELDS),scope='Three fixed original full-Memory calls including the interval-recovery rejection boundary; no new plant rollout'))
    print(json.dumps(dict(passed=True,sequences=[r['sequence'] for r in rows])))


if __name__=='__main__':main()
