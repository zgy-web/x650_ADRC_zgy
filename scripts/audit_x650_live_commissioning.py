#!/usr/bin/env python3
"""Audit actual I/O and recovery, including failed short commissioning runs.

This report never grants a hold/figure-eight certificate. It retains failures
and can prove specific transport facts without relabelling a partial run.
"""
import argparse
import hashlib
import json
from pathlib import Path
import traceback
import zipfile
import numpy as np
from audit_x650_isolated_live import (read, sha, require, rows, load_calls,
    check_consumption, command_view, check_feedback, replay, upper, ROOT)


def audit(folder, launch, output, *, numeric_replay=True):
    live=folder/'isolated_live'
    meta=read(folder/'controller_metadata.json');cfg=read(folder/'controller_task.json')
    run=read(launch/'launch.json');life=read(live/'lifecycle.json');start=read(live/'live_start.json')
    sends=read(live/'sends.json');receipts=read(live/'receipts.json');ticks=read(live/'dispatch.json')
    report={'kind':'actual_simulation_io_commissioning_v1','source_run':str(folder),
        'source_launch':str(launch),'stage_certificate':False,'hardware_qualified':False,
        'declared_hold_duration_s':cfg['duration_s'],'flight_complete':meta.get('complete'),
        'flight_error':meta.get('error'),'checks':{},'failed_checks':{}}

    def check(name, function):
        try:report['checks'][name]=function()
        except Exception as error:report['failed_checks'][name]={'error':repr(error),'traceback':traceback.format_exc()}

    def identity():
        before=read(launch/'source_manifest.json')
        require(before==read(launch/'source_manifest_after.json'),'Sources changed during run')
        require(run['runtime_binaries_unchanged'] and run['sources_unchanged'],'Runtime identity changed')
        require(cfg==run['task']==read(launch/'task.json'),'Task differs')
        protocol=run['protocol']
        require(sha(launch/'task.json')==protocol['frozen_hold_task_sha256'],'Frozen task differs')
        with zipfile.ZipFile(launch/'sources.zip') as z:
            require(set(z.namelist())==set(before),'Archive membership differs')
            for n,h in before.items():require(hashlib.sha256(z.read(n)).hexdigest()==h,'Source archive hash differs')
        return {'verified':True,'source_files':len(before),'task_sha256':sha(launch/'task.json'),
            'native_library_sha256':cfg['numerical_backend']['library_sha256']}

    def recovery():
        require(meta.get('landed_and_disarmed') is True,'Landing/disarm not confirmed')
        require(meta.get('diagnostic_parameters_restored') is True and meta.get('diagnostic_restore_pending') is False,
                'Parameter restoration not complete')
        require(run['runtime_closed'],'Runtime did not close')
        require(life['release']['writer_exited'] and life['release']['zero_consumed'],'New owner not released')
        require(all(p['returncode']==0 for p in life['processes'].values()),'Owned process did not exit cleanly')
        return {'landed_and_disarmed':True,'parameters_restored':True,'runtime_closed':True,
                'zero_consumed':True,'all_owned_processes_exited':True}

    check('frozen_identity',identity)
    check('recovery',recovery)
    check('joint_consumption_and_single_owner',lambda:check_consumption(folder,sends,receipts,start,life['release']))
    calls=[]
    try:calls=load_calls(live)
    except Exception as e:report['failed_checks']['candidate_records']={'error':repr(e)}
    if calls:
        reference=read(folder/'hover_reference.json')
        check('actual_feedback',lambda:check_feedback(folder,calls,reference))

        def wire_and_px4():
            from pyulog import ULog
            _,derived=command_view(folder,sends,calls,start)
            require(len(derived)>0,'No PVA command was sent')
            received_t,received=upper.trajectory_matrix(ULog(str(folder/'px4.ulg')))
            matched=[]
            for s in sends:
                if s['stop_zero']:continue
                value=np.array(s['position_ned']+s['velocity_ned']+s['acceleration_ned']+[s['yaw_ned']])
                indices=np.flatnonzero((received_t>=s['sim_s']-upper.CLOCK_BOUNDARY_S-1e-9)
                                      &(received_t<=s['sim_s']+upper.PVA_DELIVERY_MAX_S+1e-9))
                matches=indices[np.max(abs(received[indices]-value),axis=1)<=upper.PVA_FLOAT_TOLERANCE]
                require(len(matches)>0,'Sent candidate absent from actual PX4 trajectory_setpoint')
                # Report every bounded match; do not infer a unique ACK from
                # equal held values, or from the local send return.
                matched.append({'dispatch_sequence':s['dispatch_sequence'],'candidate_sequence':s['command_sequence'],
                    'actual_send_sim_s':s['sim_s'],'matching_px4_ulog_times_s':received_t[matches].tolist()})
            return {'verified':True,'matched_sends':len(matched),'matches':matched,
                'local_frame':1,'mask':2048,'zero_acceleration':True,'source_proposal_encoding_verified':True,
                'per_message_ack_claimed':False,'full_stage_coverage_claimed':False,
                'same_raw_clock':True,'float_tolerance':upper.PVA_FLOAT_TOLERANCE,
                'clock_boundary_allowance_s':upper.CLOCK_BOUNDARY_S,
                'delivery_allowance_s':upper.PVA_DELIVERY_MAX_S}
        check('px4_actual_received_encoding',wire_and_px4)
        if numeric_replay:check('frozen_numeric_replay',lambda:replay(live,launch/'task.json',calls,start))

    faults=[t for t in ticks if t['status']==3]
    valid=[t for t in ticks if t['output_valid']]
    report['observed_execution']={'output_count':len(valid),'source_samples':life['source_samples'],
        'faults':[{'dispatch_sequence':t['sequence'],'reason':t['reason'],'command_age_ms':t['command_age_ns']/1e6} for t in faults],
        'accepted_candidate_count':sum(e['accepted'] for e in read(live/'events.json')),
        'first_last_valid_output_wall_span_s':(valid[-1]['actual_monotonic_ns']-valid[0]['actual_monotonic_ns'])/1e9 if valid else None}
    report['io_checks_passed']=not report['failed_checks']
    report['hold_accepted']=False
    report['evidence_sha256']={str(p):sha(p) for p in [folder/'controller_metadata.json',folder/'px4.ulg',
        folder/'ts20_state.csv',live/'sends.json',live/'receipts.json',live/'dispatch.json',
        live/'recording/diagnostics.jsonl',launch/'launch.json',launch/'sources.zip']}
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as stream:json.dump(report,stream,indent=2,allow_nan=False)
    return report


def main():
    p=argparse.ArgumentParser()
    for name in ('run-dir','launch-dir','output'):p.add_argument('--'+name,type=Path,required=True)
    p.add_argument('--skip-numeric-replay',action='store_true')
    a=p.parse_args();d=audit(a.run_dir.resolve(),a.launch_dir.resolve(),a.output.resolve(),numeric_replay=not a.skip_numeric_replay)
    print(json.dumps({k:d[k] for k in ('io_checks_passed','hold_accepted','observed_execution','failed_checks')},indent=2))
    return 0 if d['io_checks_passed'] else 1


if __name__=='__main__':raise SystemExit(main())
