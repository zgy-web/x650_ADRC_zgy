#!/usr/bin/env python3
"""Independent complete-window acceptance for the C++ simulation hold.

All observations remain offline. This never starts a controller or changes an
original file. A prefix, software proof, or successful recovery is insufficient.
The certificate concerns this simulation condition, never hardware readiness.
"""
from pathlib import Path
import argparse
import csv
import hashlib
import json
import sys
import traceback
import zipfile

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT/'scripts'), str(ROOT/'control_packages/x650_wholebody_v1')]
from simulations.x650_cpp_live_v1 import qualification as qualification
from report_x650_cpp_live import identity, require, read


def same(a, b, tolerance=1e-10):
    a, b = np.asarray(a), np.asarray(b)
    return a.shape == b.shape and bool(np.isfinite(a).all() and np.isfinite(b).all()
        and np.all(np.abs(a-b) <= tolerance))


def rows(path):
    with Path(path).open(newline='') as stream:
        return list(csv.DictReader(stream))


def summary(values):
    a = np.asarray(values, dtype=float)
    require(a.size > 0 and np.isfinite(a).all(), 'Empty/nonfinite timing sample')
    return dict(count=int(a.size), minimum=float(a.min()), median=float(np.median(a)),
                p95=float(np.percentile(a, 95)), p99=float(np.percentile(a, 99)), maximum=float(a.max()))


def check_archives(launch, run, meta):
    record = read(launch/'launch.json')
    require(all(record.get(k) is True for k in ('complete', 'runtime_closed', 'sources_unchanged',
        'runtime_binaries_unchanged', 'launch_inputs_unchanged')), 'Incomplete/changing launch')
    require(record.get('exit_code') == 0 and record.get('execution_scenario') == 'x650_cpp_live_v1'
        and record.get('run_directories') == [str(run.resolve())], 'Launch/run lifecycle differs')
    sources = read(launch/'source_manifest.json')
    require(sources == read(launch/'source_manifest_after.json'), 'Execution sources changed')
    with zipfile.ZipFile(launch/'sources.zip') as archive:
        require(len(archive.namelist()) == len(sources) and set(archive.namelist()) == set(sources),
                'Source archive membership differs')
        for name, digest in sources.items():
            require(hashlib.sha256(archive.read(name)).hexdigest() == digest, 'Archived source changed: '+name)
    before = read(launch/'runtime_binary_manifest.json')
    after = read(launch/'runtime_binary_manifest_after.json')
    require(len({v['path'] for v in before}) == len(before)
        and {v['path']:v['sha256'] for v in before} == after == record['runtime_binaries_sha256'],
        'Runtime identities differ')
    with zipfile.ZipFile(launch/'runtime_binaries.zip') as archive:
        require(len(archive.namelist()) == len(before)
            and set(archive.namelist()) == {v['archive_entry'] for v in before}, 'Runtime archive differs')
        for item in before:
            data = archive.read(item['archive_entry'])
            require(len(data) == item['bytes'] and hashlib.sha256(data).hexdigest() == item['sha256'],
                    'Archived binary changed')
    cfg, protocol, startup = (read(launch/name) for name in ('task.json','protocol.json','startup_manifest.json'))
    require(cfg == record['task'] == read(run/'controller_task.json') == meta['task'], 'Actual task differs')
    require(protocol == record['protocol'] and cfg['duration_s'] == protocol['tip_hold_duration_s'] == 30.,
            'Complete frozen 30 s hold required')
    for artifact in record['launch_inputs']:
        require(identity(artifact['path']) == artifact, 'Launch input changed')
    original = read(launch/'original_startup_manifest.json')
    require({k:v for k,v in startup.items() if k != 'task_config'} ==
            {k:v for k,v in original.items() if k != 'task_config'}
            and startup['task_config']['sha256'] == original['task_config']['sha256'] == qualification.sha(launch/'task.json'),
            'Startup derivation changed something besides identical task path')
    # Re-evaluate every original proof and its transitive artifacts. This is
    # deliberately more than trusting the archived ground-admission pass flag.
    manifest = read(launch/'qualification_manifest.json')
    qproof = qualification.verify(launch/'qualification_manifest.json', task_path=Path(manifest['task_config']['path']),
        startup_path=Path(manifest['startup_manifest']['path']), startup_sha256=manifest['startup_manifest']['sha256'],
        protocol_path=Path(manifest['protocol']['path']), binaries=after)
    require(manifest['task_config']['sha256'] == qualification.sha(launch/'task.json')
        and manifest['protocol']['sha256'] == qualification.sha(launch/'protocol.json')
        and manifest['startup_manifest']['sha256'] == qualification.sha(launch/'original_startup_manifest.json'),
        'Qualified configuration is not the archived executed configuration')
    return cfg, protocol, startup, record, qproof


def numeric_array(value, size, label):
    result = np.asarray(value, dtype=float)
    require(result.shape == (size,) and np.isfinite(result).all(), 'Malformed native array: '+label)
    return result


def check_numeric_result(value, s, p, config, previous_command):
    """Check recorded owning native output against wire and physical limits."""
    require(value.get('kind') == 'cpp_full_pipeline_call_v1' and value.get('sequence') == int(s.sequence)
        and value.get('error') == '' and value.get('diagnostic_mode') == 'compact', 'Failed/misidentified actual Core call')
    r = value['result']; d = r['diagnostics']; n = config['nodes']
    require(all(d.get(k) is True for k in ('accepted','nonlinear_passed','actual_bounds'))
        and d.get('previous_plan_fallback') is False, 'Native candidate rejected/fallback')
    for key, bound in [('primal_violation',config['feasibility_tolerance']),
                       ('kkt_stationarity',config['kkt_tolerance']), ('compute_ms',50.),
                       ('q_linearization_error',.006), ('dq_linearization_error',.10)]:
        require(type(d[key]) in (int,float) and np.isfinite(d[key]) and 0 <= d[key] <= bound,
                'Native quality/deadline rejected: '+key)
    require(d['subproblems'] == len(d['attempts']) and 1 <= d['subproblems'] <= 3
        and d['attempts'][-1]['passed'] is True and d['attempts'][-1]['actual_bounds'] is True
        and d['candidate_substeps'] > 0, 'Missing actual nonlinear candidate attempts/extrema')
    limits = numeric_array(config['velocity_limit'],5,'limits')
    command = numeric_array(r['command'],5,'command')
    plan = numeric_array(d['plan'],n*5,'plan').reshape(n,5)
    h = numeric_array(d['horizon'],n,'horizon')
    require(same(h, [s.dt_s]+[config['node_dt']]*(n-1)) and np.all(h > 0)
        and same(command,plan[0]) and np.all(abs(plan) <= limits+2e-7), 'Plan/horizon/command differs')
    delta = np.diff(np.vstack((previous_command,plan)),axis=0)
    require(np.all(abs(delta) <= h[:,None]*np.asarray(config['acceleration_limit'])+2e-7), 'Actual plan delta limit exceeded')
    for field, output in [('position_reference_enu','position_reference'),('velocity_feedforward_enu','velocity_feedforward'),
                           ('acceleration_feedforward_enu','acceleration_feedforward'),('joint_velocity','joint_velocity')]:
        require(same(getattr(p,field),r[output]), 'Recorded result and actual proposal differ: '+field)
    require(same(p.base_velocity_target,command[:3]) and same(p.joint_velocity,command[3:])
        and same(p.acceleration_feedforward_enu,[0,0,0],0)
        and same(p.velocity_feedforward_enu, command[:3]-np.asarray(config['base_position_gain'])*
                 (np.asarray(p.position_reference_enu)-np.asarray(s.position_enu))), 'PX4 command decomposition differs')
    require(same([p.primal_violation,p.kkt_residual],[d['primal_violation'],d['kkt_stationarity']],0)
        and int(p.subproblems) == d['subproblems'], 'Wire quality differs from actual Core result')
    envelope = numeric_array(d['candidate_envelope'],8,'envelope')
    lo, hi = np.asarray(config['q_lower']), np.asarray(config['q_upper'])
    require(np.all(envelope[:2] <= envelope[2:4]) and np.all(envelope[4:6] <= envelope[6:8])
        and np.all(envelope[:2]-.0003 >= lo-2e-7) and np.all(envelope[2:4]+.0003 <= hi+2e-7)
        and np.all(envelope[4:6]-.03 >= -limits[3:]-2e-7)
        and np.all(envelope[6:8]+.03 <= limits[3:]+2e-7), 'Actual candidate envelope exceeds tightened joint bounds')
    state = numeric_array(d['predicted_states'],n*12,'predicted_states').reshape(n,12)
    numeric_array(d['predicted_tip'],n*3,'predicted_tip')
    numeric_array(d['predicted_rotations'],n*9,'predicted_rotations')
    require(np.all(state[:,6:8] >= lo-2e-7) and np.all(state[:,6:8] <= hi+2e-7)
        and np.all(abs(state[:,8:10]) <= limits[3:]+2e-7), 'Candidate nodes exceed physical limits')
    return command


def check_worker(live, start, config, events, resources, protocol, *, decode=None):
    if decode is None:
        from rclpy.serialization import deserialize_message
        from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal
        decode = lambda raw, proposal: deserialize_message(raw, ControlProposal if proposal else ControlSnapshot)
    final, sender, timing = (read(live/name) for name in ('worker_final.json','diagnostic_sender.json','worker_timing.json'))
    files = sorted((live/'native_records').glob('*.json'))
    require(files and final['implementation'] == 'cpp_full_pipeline_v1' and final['live_started'] is True
        and final['faulted'] is False and final['last_error'] == ''
        and final['requests'] == final['accepted'] == len(files), 'Native worker did not complete cleanly')
    require(sender['complete'] is True and sender['failed'] is False and sender['dropped'] == 0
        and sender['offered'] == sender['written'] == len(files), 'Lost actual Core evidence')
    columns = timing['columns']
    require(timing['kind'] == 'cpp_worker_phase_timing_v1' and timing['complete'] is True and timing['dropped'] == 0
        and len(columns) == len(set(columns)) and len(timing['rows']) == len(files)
        and all(len(r)==len(columns) for r in timing['rows']), 'Incomplete bounded native phase evidence')
    phase = [dict(zip(columns,r)) for r in timing['rows']]
    require(len(events) <= len(files) <= len(events)+1 and all(e['accepted'] is True for e in events)
        and [e['sequence'] for e in events] == list(range(1,len(events)+1))
        and [e['proposal_sequence'] for e in events] == [e['snapshot_sequence'] for e in events]
        == list(range(1,len(events)+1)), 'Native candidate/gate prefix differs')
    affinity = read(live/'worker_affinity_final.json')
    ready = read(live/'worker_affinity_ready.json')
    executor = protocol['worker_thread_roles']['executor']
    auxiliary = protocol['worker_thread_roles']['auxiliary']
    require(len(executor)==1 and sorted(executor+auxiliary)==resources['roles']['worker']
        and affinity['executor_cpu']==executor[0] and affinity['auxiliary_cpus']==auxiliary
        and affinity['stages'][:1]==ready['stages'] and [v['stage'] for v in affinity['stages']]==['ready','live_start','final'],
        'Worker thread placement declaration/stages differ')
    identity_set = None
    for stage in affinity['stages']:
        current={(r['tid'],r['starttime_ticks']) for r in stage['threads']}
        require(len(current)==len(stage['threads']) and (identity_set is None or current==identity_set), 'Worker TIDs changed')
        identity_set=current
        require(all(r['cpus']==(executor if r['tid']==affinity['main_tid'] else auxiliary) for r in stage['threads']),
                'Worker stage CPU masks differ')
    calls=[]; stats={name:[] for name in ('core_ms','callback_ms','thread_cpu_ms','publish_ms','record_after_publish_ms','gate_roundtrip_ms')}
    previous = np.asarray(start['history_command'][(start['history_count']-1)*5:start['history_count']*5])
    for seq,(path,t) in enumerate(zip(files,phase),1):
        value=read(path)
        for name in ('snapshot_cdr','proposal_cdr'):
            require(identity(value[name]['path'])==value[name], 'Native raw CDR bytes changed')
        s=decode(Path(value['snapshot_cdr']['path']).read_bytes(),False)
        p=decode(Path(value['proposal_cdr']['path']).read_bytes(),True)
        require(s.sequence==p.sequence==p.snapshot_sequence==seq==t['sequence']
            and s.session_id==p.session_id==start['session_id'] and s.epoch==p.epoch==start['epoch']
            and bytes(s.configuration_sha256)==bytes(p.configuration_sha256)==bytes(start['configuration_sha256'])
            and bytes(s.configuration_sha256).hex()==final['configuration_sha256']
            and p.status==0 and p.accepted_optimum and p.nonlinear_check_passed and not p.replay_only,
            'Native actual candidate identity/status differs')
        require(t['snapshot_issued_ns']==s.issued_monotonic_ns and t['callback_entry_ns']==p.compute_start_monotonic_ns,
                'Native phase/CDR timestamp differs')
        stamps=[s.issued_monotonic_ns,t['callback_entry_ns'],t['core_begin_ns'],t['core_end_ns'],p.compute_end_monotonic_ns,
                t['publish_begin_ns'],t['publish_end_ns'],t['record_offer_end_ns'],t['callback_exit_ns']]
        require(all(type(v) is int and v>0 for v in stamps) and all(b>=a for a,b in zip(stamps,stamps[1:]))
            and t['record_offered']==1 and t['thread_cpu_end_ns']>=t['thread_cpu_begin_ns']
            and t['tid_begin']==t['tid_end']==affinity['main_tid'] and t['cpu_begin']==t['cpu_end']==executor[0],
            'Native phase/CPU identity invalid')
        require(0<s.dt_s<=.05 and 0<=s.control_time_s-s.joint_time_s<=.025+1e-10
            and p.compute_end_monotonic_ns-p.compute_start_monotonic_ns<=50_000_000,
            'Original native interval/joint freshness/computation deadline exceeded')
        if seq<=len(events):
            e=events[seq-1]
            require(e['session_id']==s.session_id and e['epoch']==s.epoch
                and e['received_monotonic_ns']>=t['publish_begin_ns']
                and e['roundtrip_ns']==e['received_monotonic_ns']-s.issued_monotonic_ns
                and e['roundtrip_ns']<=50_000_000, 'Gate altered source age or accepted a stale candidate')
            stats['gate_roundtrip_ms'].append(e['roundtrip_ns']/1e6)
        previous=check_numeric_result(value,s,p,config,previous)
        for name,a,b in [('core_ms','core_begin_ns','core_end_ns'),('callback_ms','callback_entry_ns','callback_exit_ns'),
                         ('thread_cpu_ms','thread_cpu_begin_ns','thread_cpu_end_ns'),('publish_ms','publish_begin_ns','publish_end_ns'),
                         ('record_after_publish_ms','publish_end_ns','callback_exit_ns')]: stats[name].append((t[b]-t[a])/1e6)
        calls.append((s,p,value))
    # Admission source stamps and actual-send releases are independent of worker
    # phase observations; never infer them from a later accepted proposal.
    ingress=read(live/'ingress_timing.json'); names=ingress['columns']
    require(ingress['complete'] is True and ingress['overflow']==0 and len(names)==len(set(names))
        and all(len(r)==len(names) for r in ingress['rows']), 'Incomplete ingress timing')
    admissions=[dict(zip(names,r)) for r in ingress['rows'] if r[names.index('kind')]==4 and r[names.index('forward')]]
    by_seq={r['sequence']:r for r in admissions}
    require(len(by_seq)==len(admissions), 'Duplicate admission')
    for s,_,_ in calls:
        require(s.sequence in by_seq, 'Candidate has no actual ingress admission')
        a=by_seq[s.sequence]
        require(a['source_issued_ns']==s.issued_monotonic_ns and a['latest_control_s']==s.control_time_s
            and abs(a['latest_control_s']-a['last_control_s']-s.dt_s)<=1e-10
            and (s.sequence==1 or a['released_sequence']==a['last_admitted_sequence']==s.sequence-1),
            'Admission source/dt/actual-send barrier differs')
    return calls, dict(verified=True, calls=len(calls), no_lost_diagnostics=True, executor_cpu=executor[0],
        numerical_scope='Actual owning outputs checked against original wire; frozen independent software proofs separately reverified',
        durations={k:summary(v) for k,v in stats.items()})


def check_ticks(ticks, sends, calls, start, *, convert=None, source_issues=None):
    if convert is None:
        from rosidl_runtime_py.convert import message_to_ordereddict
        convert = message_to_ordereddict
    require([r['sequence'] for r in ticks]==list(range(1,len(ticks)+1))
        and all(r['session_id']==start['session_id'] and r['epoch']==start['epoch'] for r in ticks)
        and not any(r['status']==3 for r in ticks), 'Dispatch identity/gap/fault')
    valid=[r for r in ticks if r['output_valid']]; normal=[r for r in sends if not r['stop_zero']]
    require(len(valid)>1 and [v['sequence'] for v in valid]==[v['dispatch_sequence'] for v in normal],
            'Actual send/valid tick identity differs')
    by_seq={int(s.sequence):(s,p) for s,p,_ in calls}
    for t,s in zip(valid,normal):
        require(t['command_sequence'] in by_seq,'Output lacks original candidate')
        source,p=by_seq[t['command_sequence']]
        require(t['command_issued_monotonic_ns']==source.issued_monotonic_ns
            and t['command_age_ns']==t['actual_monotonic_ns']-source.issued_monotonic_ns
            and 0<=t['command_age_ns']<=50_000_000
            and 0<=t['actual_monotonic_ns']-t['source_received_monotonic_ns']<=25_000_000
            and 0<=s['sent_monotonic_ns']-source.issued_monotonic_ns<=50_000_000
            and t['command_sequence']==s['command_sequence'] and t['repeated']==s['repeated'],
            'Actual output age/source/candidate differs')
        require(0<=t['actual_monotonic_ns']-t['scheduled_monotonic_ns']<=20_000_000
            and s['sent_monotonic_ns']>=t['actual_monotonic_ns'], 'Actual dispatch missed its original absolute slot')
        if source_issues is not None:
            require(t['source_sequence'] in source_issues, 'Tick source absent from raw C++ evidence')
            issued=source_issues[t['source_sequence']]
            require(0<issued<=t['source_received_monotonic_ns']
                and 0<=t['actual_monotonic_ns']-issued<=25_000_000, 'Original source issue age exceeded 25 ms')
        # Re-encoding the same proposal cannot create a new lease.
        require(t['command']==convert(p), 'Dispatch candidate differs from actual worker CDR')
    gaps=np.diff([s['sent_monotonic_ns'] for s in normal])/1e6
    require(np.all(gaps>0) and np.max(gaps)<=40., 'Actual send gap exceeded original 40 ms bound')
    due=np.diff([t['scheduled_monotonic_ns'] for t in valid])
    require(np.all(due==20_000_000), 'Absolute output slots skipped or phase rewritten')
    return dict(verified=True, actual_send_gap_ms=summary(gaps), command_age_ms=summary([v['command_age_ns']/1e6 for v in valid]),
                original_absolute_period_ns=20_000_000, normal_sends=len(normal))


def check_actual_send_barrier(ingress, sends, calls):
    columns=ingress['columns']
    require(len(columns)==len(set(columns)) and all(len(r)==len(columns) for r in ingress['rows']), 'Invalid ingress rows')
    traces=[dict(zip(columns,r)) for r in ingress['rows']]
    admissions=[r for r in traces if r['kind']==4 and r['forward']]
    releases=[r for r in traces if r['kind']==5 and r['forward']]
    by_admit={r['sequence']:r for r in admissions};by_release={r['sequence']:r for r in releases}
    require(len(by_admit)==len(admissions) and len(by_release)==len(releases), 'Repeated admission/send release')
    first={}
    for sent in sends:
        if not sent['stop_zero']:first.setdefault(sent['command_sequence'],sent)
    for s,p,_ in calls[1:]:
        require(s.sequence-1 in first and s.sequence-1 in by_release and s.sequence in by_admit,
                'Next computation lacks actual previous send and ingress release')
        sent=first[s.sequence-1];release=by_release[s.sequence-1];admit=by_admit[s.sequence]
        require(sent['sent_monotonic_ns']<=release['entry_ns']<=admit['entry_ns']<=p.compute_start_monotonic_ns
            and release['entry_ns']-sent['sent_monotonic_ns']<=25_000_000,
            'Next computation preceded actual send/release or used a stale send observation')
    return dict(verified=True,actual_send_barrier_pairs=max(0,len(calls)-1),
                scope='Actual ActuatorSend CDR -> ingress kind5 -> next admission -> original worker callback')


def check_recovery(status, land, flight, window_end):
    """Independent ULog state changes and final received flight observations."""
    st=np.asarray(status['timestamp'],dtype=float)/1e6
    lt=np.asarray(land['timestamp'],dtype=float)/1e6
    armed=np.asarray(status['arming_state']);landed=np.asarray(land['landed'])
    failsafe=np.asarray(status['failsafe'])
    require(len(st)>0 and len(lt)>0 and np.isfinite(st).all() and np.isfinite(lt).all()
        and np.all(np.diff(st)>0) and np.all(np.diff(lt)>0), 'Invalid original recovery ULog chronology')
    require(st[-1]>window_end and lt[-1]>window_end and armed[-1]==1 and landed[-1]==1,
            'PX4 ULog does not independently end landed and disarmed after the complete hold')
    recovering=st>window_end
    require(np.all(failsafe[recovering]==0), 'PX4 failsafe during recovery')
    require(flight and float(flight[-1]['sim_time_s'])>=max(st[-1],lt[-1])
        and flight[-1]['armed']=='0' and flight[-1]['landed']=='1' and flight[-1]['failsafe']=='0',
        'Final actual flight observations do not confirm ULog landed/disarmed state')
    return dict(verified=True,ulog_status_final_s=float(st[-1]),ulog_land_final_s=float(lt[-1]),
        final_observation_sim_s=float(flight[-1]['sim_time_s']),ulog_arming_state=int(armed[-1]),
        ulog_landed=bool(landed[-1]),recovery_failsafe_samples=int(np.count_nonzero(failsafe[recovering])))


def ulog_value_view(control, derived, sends):
    """Fold only an exact same-clock repeated value for the ULog matcher.

    ActuatorSend.sim_s is the held actuator-state clock, not a distinct publish
    timestamp. Two real periodic sends can therefore share it. Their original
    monotonic send records remain independently checked and are never removed.
    This view makes no claim that PX4 acknowledges every repeated publication.
    """
    normal = [s for s in sends if not s['stop_zero']]
    require(len(normal) == len(derived) and len({id(r) for r in derived}) == len(derived),
            'ULog value view lost original send identities')
    require(all(sum(r is d for r in control) == 1 for d in derived),
            'ULog control view does not retain every original derived row')
    wire_fields = ('coordinate_frame', 'type_mask', 'yaw_ned', 'position_ned',
                   'velocity_ned', 'acceleration_ned', 'joint_velocity')
    omitted = set()
    equivalent = []
    for i, (s, row) in enumerate(zip(normal, derived)):
        require(float(row['sim_s']) == float(s['sim_s']), 'ULog value/send clock differs')
        if not i or float(row['sim_s']) != float(derived[i-1]['sim_s']):
            continue
        prior = normal[i-1]
        require(s['repeated'] is True and s['command_sequence'] == prior['command_sequence']
                and row == derived[i-1] and all(s[k] == prior[k] for k in wire_fields),
                'Same-clock send is not an exact repeated command/payload/feedback')
        require(s['sent_monotonic_ns'] > prior['sent_monotonic_ns']
                and s['dispatch_sequence'] > prior['dispatch_sequence'],
                'Repeated value does not have a distinct ordered actual publication')
        omitted.add(id(row))
        equivalent.append(dict(dispatch_sequences=[prior['dispatch_sequence'], s['dispatch_sequence']],
            sent_monotonic_ns=[prior['sent_monotonic_ns'], s['sent_monotonic_ns']],
            command_sequence=s['command_sequence'], held_state_sim_s=s['sim_s']))
    result = [row for row in control if id(row) not in omitted]
    return result, dict(actual_normal_send_count=len(normal),
        distinct_pva_value_row_count=len(derived)-len(omitted),
        equivalent_repeated_publication_count=len(omitted), equivalent_repeated_publications=equivalent,
        original_send_records_preserved=True, original_derived_rows_preserved=True,
        timestamps_changed=False,
        scope='Only the ULog value-matching view folds exact same-clock, same-candidate repeated payloads and feedback; this is not a per-publication PX4 acknowledgement')


def audit(launch, run, out):
    report=dict(kind='x650_cpp_live_complete_hold_certificate_v1',passed=False,complete=False,
        certificate=False,physical_acceptance=False,hardware_qualified=False,launch_directory=str(launch),run_directory=str(run))
    artifacts=[]
    try:
        # Even a failed audit retains the hashes of its exact original evidence.
        paths={p for base in (launch,run) for p in base.rglob('*') if p.is_file()}
        paths.update(ROOT/name for name in ('scripts/audit_x650_cpp_hold.py','scripts/report_x650_cpp_live.py',
            'scripts/audit_x650_cpp_source.py','scripts/audit_x650_streaming_consumption.py',
            'scripts/audit_x650_isolated_live.py','scripts/audit_x650_precision_stage.py','scripts/audit_x650_upper_stage.py',
            'scripts/analyze_x650_hover_precision.py','simulations/x650_cpp_live_v1/qualification.py',
            'simulations/x650_ts20_hold_v1/audit.py','simulations/x650_ts20_hold_v1/task.py'))
        artifacts=[identity(p) for p in sorted(paths)]
        meta=read(run/'controller_metadata.json')
        require(all(meta.get(k) is True for k in ('complete','landed_and_disarmed','diagnostic_parameters_restored'))
            and meta.get('diagnostic_restore_pending') is False, 'Incomplete flight lifecycle: '+str(meta.get('error')))
        stage=read(run/'stage_result.json')
        require(all(stage.get(k) is True for k in ('complete','passed','success','landed_and_disarmed'))
            and read(run/'result.json').get('success') is True, 'Original provisional stage/run failed')
        cfg,protocol,startup,launch_record,proof=check_archives(launch,run,meta)
        report['software_qualification']=proof
        live=run/'isolated_live'; lifecycle=read(live/'lifecycle.json'); start=read(live/'live_start.json')
        require(lifecycle['started'] is True and lifecycle['error'] is None
            and all(p['returncode']==0 for p in lifecycle['processes'].values()), 'Native child lifecycle failed')
        require(start['history_count']>0 and abs(meta['task_end_sim_s']-start['origin_s']-30.)<=.05,
                'Incomplete full 30 s task')
        window=[start['origin_s'],start['origin_s']+30.]
        import audit_x650_isolated_live as old
        report['ground_initialization']=old.check_preflight_transport(live,meta,start)
        report['fault_supervision']=old.check_fault_supervision(live,start)
        resources=read(live/'execution_resources.json')
        require(resources==launch_record['execution_resources'] and resources['roles']==protocol['cpu_roles'], 'Actual resources changed')
        observed=read(live/'execution_affinity_at_handoff.json')
        require(set(observed)=={'source',*lifecycle['processes']}, 'Incomplete process placement observation')
        for name,threads in observed.items():
            if name=='worker':continue
            require(threads and all(mask==resources['roles'].get(name,resources['background']) for mask in threads.values()),
                    'Handoff CPU partition changed: '+name)
        actual_startup=read(live/'worker_startup.json')
        require(actual_startup['startup_manifest']==startup and actual_startup['python_in_worker'] is False
            and actual_startup['startup_controller_steps']==0 and actual_startup['diagnostic_mode']=='compact'
            and actual_startup['session_id']==start['session_id'] and actual_startup['epoch']==start['epoch']
            and actual_startup['configuration_sha256']==qualification.sha(launch/'task.json'), 'Actual worker startup differs')
        for field,loaded in [('core_library','loaded_core_path'),('optimization_library','loaded_optimization_path'),('predictor_library','loaded_predictor_path')]:
            require(str(Path(startup[field]['path']).resolve())==actual_startup[loaded], 'Actually loaded native library differs')
        import audit_x650_streaming_consumption as streaming
        messages=streaming.original_messages(live)
        sends,receipts,ticks,events=(messages[k] for k in ('sends','receipts','dispatch','events'))
        calls,report['worker']=check_worker(live,start,startup['config'],events,resources,protocol)
        report['actual_send_barrier']=check_actual_send_barrier(read(live/'ingress_timing.json'),sends,calls)
        source_rows=rows(live/'cpp_source/source_selections.csv')
        source_issues={int(v['sequence']):int(v['issued_ns']) for v in source_rows}
        require(len(source_issues)==len(source_rows), 'Repeated source issue identity')
        report['timing']=check_ticks(ticks,sends,calls,start,source_issues=source_issues)
        publications=[json.loads(line) for line in (run/'control_execution_trace.jsonl').open() if line.strip()]
        report['consumption']=streaming.audit_consumption(sends,receipts,rows(run/'ts20_state.csv'),start,lifecycle['release'],publications)
        require(report['consumption']['passed'] is True, 'Asynchronous physical consumption accounting failed')
        from flyarm_control_interfaces.msg import ActuatorState
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.convert import message_to_ordereddict
        baseline=message_to_ordereddict(deserialize_message((live/'actuator_epoch_state.cdr').read_bytes(),ActuatorState))
        report['consumption']['online_ledger']=streaming.audit_async_ledger(read(live/'actuator_ledger.json'),baseline,
            sends,receipts,rows(run/'ts20_state.csv'),start,lifecycle['release'])
        require(report['consumption']['online_ledger']['passed'] is True, 'Online asynchronous progress evidence failed')
        from audit_x650_cpp_source import audit_source
        reference=read(run/'hover_reference.json')
        report['feedback']=audit_source(live/'cpp_source',run/'ts20_state.csv',run/'ts20_packets.csv',reference,
            read(run/'estimated_feedback.json')['extra_reset_counters_at_bind'],worker_directory=live/'native_records',
            expected_binding=dict(configuration_sha256=actual_startup['configuration_sha256'],
                position_gains=startup['config']['base_position_gain'],bus_period_s=startup['config']['bus_period'],delay_s=startup['config']['motion'][4]))
        require(all(report['feedback'][key] is True for key in ('passed','record_integrity_passed','clean_source_stop','worker_link_checked'))
            and report['feedback']['original_worker_snapshots_verified']==len(calls), 'Raw C++ feedback/source validation failed')
        docs={k:read(run/n) for k,n in {'session':'diagnostic_parameter_session.json','journal':'diagnostic_restore_journal_completed.json',
            'before':'diagnostic_parameters_before.json','applied':'diagnostic_parameters_applied.json','restored':'diagnostic_parameters_restored.json',
            'px4_before':'px4_parameters_before.json','px4_after':'px4_parameters_after.json'}.items()}
        _,pid,_=old.frozen_research_pid(cfg)
        for name,document in [('metadata',meta),('stage',stage)]:
            old.fixed.verify_condition(document,old.condition_for_task(cfg),name)
        report['parameters']=old.fixed.verify_parameter_session(docs,meta,old.condition_for_task(cfg),pid)
        old.precision.verify_reference(reference,old.precision.DEFAULT_REFERENCE_SOURCE)
        manifest=read(run/'native_trace_manifest.json')
        require(manifest['complete'] is True and manifest['sha256']==qualification.sha(run/'native_trace.csv'), 'Incomplete native physical trace')
        old.precision.capture_window_covers(manifest,window)
        native=old.precision.native_states(run/'native_trace.csv',window)
        from pyulog import ULog
        ulog=ULog(str(run/'px4.ulg'))
        report['recovery']=check_recovery(old.precision.dataset(ulog,'vehicle_status'),
            old.precision.dataset(ulog,'vehicle_land_detected'),rows(run/'flight_samples.csv'),window[1])
        report['configuration']=old.fixed.effective_configuration(ulog,native,reference,window,old.fixed.records(docs['applied']))
        control,derived=old.command_view(run,sends,calls,start)
        (out/'derived_command_audit_view.json').write_text(json.dumps(dict(scope='Actual send CDR matched to original worker/source; originals untouched',rows=derived),allow_nan=False))
        value_control,value_proof=ulog_value_view(control,derived,sends)
        (out/'distinct_pva_value_audit_view.json').write_text(json.dumps(dict(**value_proof,rows=value_control),allow_nan=False))
        report['px4_value_view']=value_proof
        report['px4_commands']=old.upper.upper_commands(ulog,native,reference,value_control,window,pid,response=False)
        report['px4_commands']['all_distinct_pva_value_rows_observed']=report['px4_commands'].pop('all_measurement_sends_observed')
        report['px4_commands']['distinct_pva_measurement_value_rows']=report['px4_commands'].pop('measurement_sends')
        report['px4_commands']['observation_scope']=value_proof['scope']
        report['flight']=old.upper.flight_states(run/'flight_samples.csv',window)
        report['ground']=old.fixed.native_ground_check(run,read(run/'native_ground.json'))
        report['actuator']=old.prior_audit.actuator_check(run,cfg,window)
        report['actuator'].pop('control_snapshot_to_plugin_consumption_lag',None)
        require(report['actuator']['new_joint_prediction_verified'], 'Independent whole-window joint prediction failed')
        endpoint=old.shared.native_tip_metrics(run,old.shared.ConstantTarget(np.asarray(start['target_position_enu'])).sample,window[0],(0.,30.))
        limits=cfg['ts20_protocol'];tilt=float(native['tilt'].max())
        require(endpoint['position_rmse_m']<=limits['tip_rms_max_m'] and endpoint['position_max_m']<=limits['tip_peak_max_m']
            and tilt<=limits['tilt_max_deg'], 'Original full-window physical hold envelope exceeded')
        report.update(endpoint=endpoint,tilt_peak_deg=tilt,window_sim_s=window)
        require(all(identity(v['path'])==v for v in artifacts), 'Original evidence changed while auditing')
        report.update(passed=True,complete=True,certificate=True,physical_acceptance=True,
            full_window_ulog_verified=True,parameter_restore_verified=True,native_ground_verified=True,
            scope='One complete original 30 s simulation hold; not hardware qualification or a worst-case timing guarantee')
    except Exception as error:
        report.update(error=repr(error),traceback=traceback.format_exc())
    report['artifacts']=artifacts
    (out/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf-8')
    return report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    for name in ('launch-dir','run-dir','output'): parser.add_argument('--'+name,type=Path,required=True)
    args=parser.parse_args(); launch=args.launch_dir.resolve();run=args.run_dir.resolve();out=args.output.resolve()
    require(not out.exists() and not out.is_relative_to(run) and not out.is_relative_to(launch), 'New audit output must be outside originals')
    out.mkdir(parents=True)
    result=audit(launch,run,out)
    print(json.dumps({k:result.get(k) for k in ('passed','error','endpoint','timing')},indent=2))
    return 0 if result['passed'] else 1


if __name__=='__main__': raise SystemExit(main())
