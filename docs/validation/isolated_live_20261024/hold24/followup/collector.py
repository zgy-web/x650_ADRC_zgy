"""Prepare/collect a compact, immutable snapshot after an explicitly confirmed result.

No subprocess, test, simulator, renderer, network, runtime/source modification,
retry or wait. Default prints a plan; --execute requires a terminal status and
a new ZIP path. Full raw evidence remains required at its original server paths.
"""
import argparse
import datetime
import hashlib
import json
from pathlib import Path
import zipfile

BASE_FILES = (
    'quiet_task_cpp_v10.json', 'quiet_task_cpp_v11.json',
    'execution_protocol_v17.json', 'execution_protocol_v17_ready2.json',
    'prepare_hold_v24.py', 'prepare_hold_v24_status.json',
    'hold_v24_preflight.sh', 'hold_v24_preflight.log',
    'prepare_hold_v24_gate_refresh.py', 'prepare_hold_v24_gate_refresh_status.json',
    'prepare_hold_v24_gate_refresh.log', 'hold_v24_ready2.sh',
    'hold_v24_preflight_v2.sh', 'hold_v24_preflight_v2.log', 'hold_v24_preflight_v2.exit',
    'renew_ts20_prerequisites_v17.py', 'prerequisites_v17.driver.log',
    'hold_v24.log', 'hold_v24.exit', 'hold_v24_runner.sh',
    'hold_v24_resource_observer.py', 'hold_v24_resources.jsonl',
    'hold_v24_audit_v1.log', 'hold_v24_audit_v1.exit',
    'hold_v24_plot.log', 'hold_v24_plot.exit',
    'progress_v8.py', 'progress_v8.json', 'progress_v8.log',
    'normal_sequence_v8.log', 'normal_sequence_v8.exit',
    'preparation_software_v5/native_acceptance.json',
    'preparation_software_v5/software_acceptance.json',
)
FOLDERS = ('prerequisites_v17', 'hold_v24_audit_v1', 'hold_v24_plot', 'normal_sequence_v8')
ALLOWED_SUFFIXES = {'.json', '.log', '.exit', '.txt', '.py', '.sh', '.png', '.pdf'}
LAUNCH_FILES = ('launch.json', 'task.json', 'source_manifest.json', 'source_manifest_after.json', 'flight.log')
RUN_FILES = (
    'controller_metadata.json', 'controller_task.json', 'controller_result.json',
    'native_trace_manifest.json', 'native_ground.json', 'hover_reference.json',
    'diagnostic_parameter_session.json', 'diagnostic_restore_journal_completed.json',
    'diagnostic_parameters_before.json', 'diagnostic_parameters_applied.json',
    'diagnostic_parameters_restored.json', 'px4_parameters_before.json', 'px4_parameters_after.json',
    'isolated_live/lifecycle.json', 'isolated_live/live_start.json',
    'isolated_live/fault_supervision.json', 'isolated_live/worker_final.json',
    'isolated_live/worker_preflight_gc.json', 'isolated_live/preflight_transport.json',
    'isolated_live/execution_resources.json', 'isolated_live/execution_affinity_at_handoff.json',
    'isolated_live/events.json', 'isolated_live/rejected_snapshot.json',
    'result.json', 'run_manifest.json', 'stage_result.json', 'tip_hold.json',
    'flight_samples_manifest.json', 'control_execution_trace_manifest.json',
    'ts20_recording_manifest.json', 'source_import_manifest.json',
    'loop_timing_evidence.json', 'isolated_live/worker_timing.json',
    'isolated_live/ingress_timing.json', 'isolated_live/source_timing.json',
    'isolated_live/source_pump_timing.json', 'isolated_live/source_sender.json',
    'isolated_live/source_recording/manifest.json', 'isolated_live/source_recording_reconciled.json',
    'isolated_live/recording/manifest.json', 'isolated_live/diagnostic_sender.json',
    'isolated_live/sends.json', 'isolated_live/receipts.json',
    'isolated_live/dispatch.json', 'isolated_live/feedback_sources.json',
    'isolated_live/dispatcher.log', 'isolated_live/consumption.log',
    'isolated_live/ingress.log', 'isolated_live/worker_startup.json',
)


def digest(data): return hashlib.sha256(data).hexdigest()


def read(path): return json.loads(path.read_text()) if path.is_file() else None


def terminal(base, scope):
    prerequisites = read(base/'prerequisites_v17/status.json') or {}
    prepare = read(base/'prepare_hold_v24_gate_refresh_status.json') or {}
    progress = read(base/'progress_v8.json') or {}
    normal = read(base/'normal_sequence_v8/status.json') or {}
    if scope == 'prerequisites':
        done = prerequisites.get('complete') is True or bool(prerequisites.get('error'))
    elif scope == 'hold':
        done = (bool(prepare.get('error')) or bool(progress.get('error'))
                or progress.get('hold_passed') is True)
    else:
        done = (bool(prepare.get('error')) or bool(progress.get('error'))
                or progress.get('complete') is True or bool(normal.get('error')))
    return bool(done), dict(prerequisites=prerequisites, prepare=prepare, progress=progress, normal=normal)


def collect_plan(repo, base, states):
    selected = {base/name for name in BASE_FILES if (base/name).is_file()}
    missing = [name for name in BASE_FILES if not (base/name).is_file()]
    for name in FOLDERS:
        folder = base/name
        if folder.is_dir():
            selected.update(p for p in folder.rglob('*') if p.is_file()
                            and p.suffix in ALLOWED_SUFFIXES and 'row_evidence' not in p.parts)
    links = []
    for stage in states['prerequisites'].get('stages', []):
        if stage.get('launch_directory'): links.append(('launch', stage['launch_directory']))
        links.extend(('run', value) for value in stage.get('run_directories', []))
    for key, kind in (('hold_launch','launch'), ('hold_run','run')):
        if states['progress'].get(key): links.append((kind, states['progress'][key]))
    for key, kind in (('launch_directory','launch'), ('run_directory','run')):
        if states['normal'].get(key): links.append((kind, states['normal'][key]))
    # A launch may fail before the conditional driver writes its path. Keep
    # only a uniquely reported path from the original log; never pick by mtime.
    log = base/'hold_v24.log'
    if log.is_file() and not any(kind=='launch' and value==states['progress'].get('hold_launch') for kind,value in links):
        reported = [line.split(' ',1)[1] for line in log.read_text(errors='replace').splitlines()
                    if line.startswith('ISOLATED_LAUNCH ')]
        if len(reported) == 1: links.append(('launch', reported[0]))
    for kind,value in list(links):
        path = Path(value).resolve()
        if not any(path.is_relative_to(repo/folder) for folder in ('runs','artifacts')):
            raise ValueError('Linked evidence escapes repository evidence roots: '+str(path))
        if kind == 'launch':
            document = read(path/'launch.json') or {}
            links.extend(('run', value) for value in document.get('run_directories', []))
    for kind,value in links:
        directory = Path(value).resolve()
        if not any(directory.is_relative_to(repo/folder) for folder in ('runs','artifacts')):
            raise ValueError('Linked run escapes evidence roots')
        for name in LAUNCH_FILES if kind=='launch' else RUN_FILES:
            path = directory/name
            if path.is_file(): selected.add(path)
    return sorted(selected), missing, links


def unchanged_parameters(base):
    old, new = read(base/'quiet_task_cpp_v10.json'), read(base/'quiet_task_cpp_v11.json')
    previous, current = read(base/'execution_protocol_v17.json'), read(base/'execution_protocol_v17_ready2.json')
    result = dict(task_v11_present=new is not None, protocol_ready2_present=current is not None)
    if old is not None and new is not None:
        result['only_stage_certificate_paths_changed'] = (
            {k:v for k,v in old.items() if k!='stage_evidence'} ==
            {k:v for k,v in new.items() if k!='stage_evidence'})
        if not result['only_stage_certificate_paths_changed']: raise ValueError('Task numerical/parameter content changed')
    if previous is not None and current is not None:
        result['only_protocol_task_sha_changed'] = (
            {k:v for k,v in previous.items() if k!='frozen_hold_task_sha256'} ==
            {k:v for k,v in current.items() if k!='frozen_hold_task_sha256'})
        if not result['only_protocol_task_sha_changed']: raise ValueError('Protocol content changed beyond task SHA')
        if new is not None and current['frozen_hold_task_sha256'] != digest((base/'quiet_task_cpp_v11.json').read_bytes()):
            raise ValueError('Ready protocol does not bind the archived v11 task')
    return result


def hold_summary(repo, base, links):
    """Derived scalar observations, never substituted for original evidence."""
    runs = sorted({Path(value).resolve() for kind, value in links
                   if kind == 'run' and 'x650_isolated_live_v1_' in Path(value).name})
    if not runs: return {'available': False}
    if len(runs) != 1: raise ValueError('Exactly one identified live hold run is required')
    run = runs[0]; live = run/'isolated_live'
    def original(path):
        raw = path.read_bytes()
        return {'path': str(path), 'bytes': len(raw), 'sha256': digest(raw)}
    def rows(name):
        value = read(live/name)
        return [dict(zip(value['columns'], row)) for row in value['rows']]
    source, pumps = rows('source_timing.json'), rows('source_pump_timing.json')
    supervisor = read(live/'fault_supervision.json')
    fault = supervisor['first_fault']; stamp = fault['producer_monotonic_ns']
    worker = read(live/'worker_final.json'); meta = read(run/'controller_metadata.json')
    natural_gc = read(run/'loop_timing_evidence.json')['natural_gc']
    resources = [json.loads(line) for line in (base/'hold_v24_resources.jsonl').read_text().splitlines() if line]
    nearest = sorted(resources, key=lambda row: abs(row['monotonic_ns']-stamp))[:5]
    nearest.sort(key=lambda row: row['monotonic_ns'])
    return {'derived_analysis': True, 'run_directory': str(run), 'hold_accepted': False,
            'source_count': len(source), 'successful_pumps': len(pumps),
            'worker_requests': worker['engine']['requests'], 'worker_accepted': worker['engine']['accepted'],
            'task_start_sim_s': meta['task_start_sim_s'], 'last_source': source[-1],
            'last_successful_pump': pumps[-1], 'first_fault': fault,
            'last_source_age_at_fault_ms': (stamp-source[-1]['issued_ns'])/1e6,
            'last_publish_duration_ms': (source[-1]['publish_return_ns']-source[-1]['issued_ns'])/1e6,
            'last_completed_pump_to_observation_ms': (fault['observed_monotonic_ns']-pumps[-1]['end_ns'])/1e6,
            'max_completed_pump_ms': max((row['end_ns']-row['begin_ns'])/1e6 for row in pumps),
            'error': meta.get('error'), 'landed_and_disarmed': meta.get('landed_and_disarmed'),
            'natural_gc_all_run_generation_peaks': natural_gc['per_generation'],
            'near_fault_resource_rows': nearest,
            'source_activity_instrumentation_present': (live/'source_activity.json').exists(),
            'limits_of_inference': [
                'The failing pump has no final stage record; its first supervision poll raised.',
                'GC recent ring and all-run peaks continued during recovery; absence of overlap does not exclude a smaller earlier GC event.',
                'Independent fault executor is polled by the same source thread; producer-to-observer time is not a measured DDS transport latency.',
                'No source_activity artifact is created retrospectively.',
            ], 'original_inputs': [original(path) for path in [live/'source_timing.json',live/'source_pump_timing.json',
                live/'worker_final.json',live/'fault_supervision.json',run/'controller_metadata.json',
                run/'loop_timing_evidence.json',base/'hold_v24_resources.jsonl']]}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path,required=True)
    parser.add_argument('--base',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True,help='New dedicated ZIP; never an existing artifact')
    parser.add_argument('--scope',choices=('prerequisites','hold','normal'),default='normal')
    parser.add_argument('--max-file-bytes',type=int,default=2*1024*1024)
    parser.add_argument('--execute',action='store_true')
    args=parser.parse_args();repo=args.repo.resolve();base=args.base.resolve();output=args.output.resolve()
    if not base.is_relative_to(repo/'artifacts') or not output.is_relative_to(base):
        raise ValueError('Base/output must remain in the explicitly selected artifact tree')
    done,states=terminal(base,args.scope)
    selected,missing,links=collect_plan(repo,base,states)
    if not args.execute:
        print(json.dumps(dict(dry_run=True,terminal=done,scope=args.scope,selected=[str(p) for p in selected],
                              missing=missing,output=str(output)),indent=2));return
    if not done: raise RuntimeError('Requested result scope is still active; collection refused')
    if output.exists(): raise FileExistsError('Fresh archive required; old evidence is never replaced')
    constraints=unchanged_parameters(base)
    data={};identities=[];omitted=[]
    for path in selected:
        resolved=path.resolve()
        if not resolved.is_relative_to(repo) or resolved.suffix.lower() in ('.mp4','.avi','.ulg','.csv','.cdr'):
            raise ValueError('Unexpected evidence path: '+str(path))
        raw=resolved.read_bytes();entry=dict(server_path=str(resolved),bytes=len(raw),sha256=digest(raw))
        relative=resolved.relative_to(repo).as_posix();entry['archive_path']=relative
        if len(raw)>args.max_file_bytes:
            omitted.append(entry|{'reason':'above_compact_file_limit'});continue
        data[relative]=raw;identities.append(entry)
    # Detect a concurrently changing report/log. Do not label a moving snapshot
    # complete; let the orchestrator decide later, with no automatic retry.
    for entry in identities+omitted:
        path=Path(entry['server_path'])
        if digest(path.read_bytes())!=entry['sha256']: raise RuntimeError('Evidence changed during collection: '+str(path))
    hold_report=read(base/'hold_v24_audit_v1/report.json') or {}
    normal_report=read(base/'normal_sequence_v8/normal_audit/report.json') or {}
    manifest=dict(kind='execution_v17_followup_compact_snapshot',captured_utc=datetime.datetime.now(datetime.timezone.utc).isoformat(),
                  scope=args.scope,terminal_confirmed=True,producer_state=states,parameter_comparison=constraints,
                  hold_passed=hold_report.get('passed') is True and hold_report.get('complete') is True,
                  normal_passed=normal_report.get('passed') is True and normal_report.get('complete') is True,
                  video_rendered=False,fast_experiment_started=False,files=identities,omitted_large_files=omitted,
                  missing_optional_files=missing,portable_complete_proof=False,
                  external_raw_evidence_required=True,
                  external_formal_row_directories=[str(base/'preparation_software_v5/matrix/row_evidence'),str(base/'preparation_replay_v5/row_evidence')])
    summary = hold_summary(repo,base,links) if args.scope=='hold' else None
    if summary is not None:
        summary_raw=(json.dumps(summary,indent=2)+'\n').encode()
        data['hold24_source_failure_summary.json']=summary_raw
        manifest['derived_summary']={'archive_path':'hold24_source_failure_summary.json',
                                     'bytes':len(summary_raw),'sha256':digest(summary_raw)}
    collector_raw=Path(__file__).read_bytes()
    data['collector.py']=collector_raw
    manifest['collector']={'archive_path':'collector.py','bytes':len(collector_raw),'sha256':digest(collector_raw)}
    data['collection_manifest.json']=(json.dumps(manifest,indent=2)+'\n').encode()
    data['README.md']=(
        '# Execution revision 17 follow-up evidence snapshot\n\n'
        'This archive preserves the original failed ground preflight, new prerequisite\n'
        'certificates, v10/v11 tasks and old/ready2 protocol when available. The manifest\n'
        'records actual terminal statuses and parameter comparisons; a failed or missing\n'
        'stage is not relabelled as successful. No test, simulation or video was run by\n'
        'this collector. The producer remains responsible for lifecycle/recovery.\n\n'
        'Full CDR/CSV/ULog records and all 418 formal software row-evidence files remain\n'
        'at original server paths bound by the original reports. Large files omitted by\n'
        'the size cap are listed with identities. This compact ZIP is insufficient for\n'
        'independent complete revalidation without those external dependencies.\n'
        '\nThe refreshed prerequisites passed (20 s folded hover; 30 s joint probe).\n'
        'Hold24 did start, but stopped after about 4.92 simulation seconds of its\n'
        'requested 30 s window: output reason 2 (source age). Its worker accepted\n'
        '247/247 requests; this does not constitute full-hold acceptance.\n'
        'The ordinary and fast figure-eight stages did not start. Actual terminal\n'
        'status is preserved even if an earlier preparation status says running.\n'
        'No source-activity instrumentation existed in this run; the derived\n'
        'summary explains the missing exceptional-pump/GC evidence boundaries.\n'
    ).encode()
    with zipfile.ZipFile(output,'x',zipfile.ZIP_DEFLATED) as archive:
        for name,raw in sorted(data.items()):archive.writestr(name,raw)
    print(json.dumps(dict(archive=str(output),bytes=output.stat().st_size,sha256=digest(output.read_bytes()),
                          files=len(identities),omitted=len(omitted),hold_passed=manifest['hold_passed'],
                          normal_passed=manifest['normal_passed'],portable_complete_proof=False),indent=2))


if __name__=='__main__':main()
