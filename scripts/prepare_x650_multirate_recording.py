#!/usr/bin/env python3
"""Derive an explicitly relabelled low-rate software probe from original CDR.

Physical state and actual packet history are unchanged. Selection, sequence,
task hash and measured inter-snapshot dt change. This is not plant validation.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
from probe_x650_recovery_worker import identity, receipt, write_new, require


def select_rows(rows, period=.04, count=24):
    selected=[];last=0.
    for row in rows:
        t=float(row['snapshot']['task_time_s'])
        if t-last < period-1e-9:
            continue
        require(t-last <= .1, 'Derived real inter-snapshot interval exceeds 100 ms')
        selected.append((row,t-last));last=t
        if len(selected)==count:
            return selected
    raise ValueError('Not enough source snapshots for the declared sequence')


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--recording',type=Path,required=True)
    p.add_argument('--startup',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    p.add_argument('--count',type=int,default=24)
    a=p.parse_args();require(a.output.is_absolute() and not a.output.exists(),'New absolute output required')
    original=json.loads(a.recording.read_text());startup=json.loads(a.startup.read_text())
    require(original['originals_verified_unchanged'] is True,'Verified original export required')
    require(startup['timing_profile']=='mpc25_dispatch50_v1','Explicit timing profile required')
    from rclpy.serialization import deserialize_message,serialize_message
    from flyarm_control_interfaces.msg import ControlSnapshot,ControlProposal,LiveStart
    a.output.mkdir(parents=True)
    rows=[]
    for binding in original['rows']:
        receipt(binding);row=json.loads(Path(binding['path']).read_text())
        row['_binding']=binding;rows.append(row)
    chosen=select_rows(rows,count=a.count)
    # Both current hold and fast-eight tasks share a fixed initial target. Use
    # only this preparation prefix, including the full 0.6 s preview window.
    require(chosen[-1][0]['snapshot']['task_time_s']+.6 < 2.,'Probe must stay before trajectory departure')
    digest=list(bytes.fromhex(startup['task_config']['sha256']))
    start=deserialize_message(Path(receipt(original['live_start_cdr_identity'])['path']).read_bytes(),LiveStart)
    start.configuration_sha256=digest
    start_path=a.output/'live_start.cdr';start_path.write_bytes(serialize_message(start))
    doc=deepcopy(original)
    doc.update(kind='x650_cpp_multirate_derived_recording_v1',original_recording=identity(a.recording),
        physical_validation=False,counterfactual_closed_loop_claimed=False,
        original_states_and_actual_history_preserved=True,derived_fields=['selection >=40ms','sequence','configuration_sha256','dt_s from actual selected task-time difference'],
        config=startup['config'],trajectory=startup['trajectory'],timing_profile=startup['timing_profile'],
        rows=[],live_start_cdr_identity=identity(start_path))
    doc['live_start']['configuration_sha256']=digest
    for seq,(source,dt) in enumerate(chosen,1):
        row=deepcopy(source);binding=row.pop('_binding')
        row['source_row']=binding;row['original_sequence']=source['sequence'];row['sequence']=seq
        for field,cls in [('snapshot',ControlSnapshot),('proposal',ControlProposal)]:
            source_id=receipt(source[field+'_identity'])
            msg=deserialize_message(Path(source_id['path']).read_bytes(),cls)
            row['original_'+field+'_identity']=source_id
            msg.sequence=seq;msg.configuration_sha256=digest
            row[field]['sequence']=seq;row[field]['configuration_sha256']=digest
            if field=='snapshot':
                msg.dt_s=dt;row[field]['dt_s']=dt
            else:
                msg.snapshot_sequence=seq;row[field]['snapshot_sequence']=seq
            path=a.output/f'{seq:06d}.{field}.cdr';path.write_bytes(serialize_message(msg))
            row[field+'_identity']=identity(path)
        path=a.output/f'{seq:06d}.json';write_new(path,row)
        doc['rows'].append(dict(identity(path),sequence=seq))
    write_new(a.output/'recording.json',doc)
    print(json.dumps(dict(recording=identity(a.output/'recording.json'),count=len(chosen),last_task_s=chosen[-1][0]['snapshot']['task_time_s'])))


if __name__=='__main__':main()
