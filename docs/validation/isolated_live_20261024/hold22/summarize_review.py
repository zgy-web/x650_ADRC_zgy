"""Summarize previously retrieved hold22 evidence; no network or simulation."""
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent
read = lambda name: json.loads((ROOT/name).read_text())


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def stats(values):
    a = np.asarray(values, dtype=float)
    assert len(a) and np.isfinite(a).all()
    return dict(zip(['minimum','p50','p95','p99','maximum'], map(float,np.percentile(a,[0,50,95,99,100]))))


def table(name):
    t = read('isolated_live/'+name+'.json')
    return [dict(zip(t['columns'],row)) for row in t['rows']]


def cpu_stat(value):
    return {key:int(value) for key,value in (line.split() for line in value.splitlines())}


manifest = read('retrieval_manifest.json')
for name, entry in manifest['files'].items():
    p = ROOT/name
    assert p.stat().st_size == entry['bytes'] and sha(p) == entry['sha256']
meta, launch = read('controller_metadata.json'), read('launch/launch.json')
start = read('isolated_live/live_start.json')
supervision = read('isolated_live/fault_supervision.json')
fault = supervision['first_fault']
fault_ns, start_ns = fault['producer_monotonic_ns'], start['issued_monotonic_ns']
assert fault['observed_monotonic_ns']-fault_ns == fault['producer_to_observation_ns']
events = read('isolated_live/events.json')
assert events[-1]['received_monotonic_ns'] == fault_ns and not events[-1]['accepted'] and events[-1]['reason'] == 15
worker, source, ingress, pumps = [table(n) for n in ['worker_timing','source_timing','ingress_timing','source_pump_timing']]
wf = read('isolated_live/worker_final.json')
physical = read('physical_consumption_review.json')
remote = read('remote_cdr_ulog_review.json')
assert remote['cdr']['worker_sha256'] == read('isolated_live/recording/manifest.json')['records_sha256']
assert remote['cdr']['source_sha256'] == read('isolated_live/source_recording/manifest.json')['records_sha256']
assert physical['source_files']['isolated_live/recording/diagnostics.jsonl']['sha256'] == remote['cdr']['worker_sha256']
assert len(worker) == len(remote['cdr']['candidates']) == wf['engine']['requests'] == wf['engine']['accepted'] == 18
for name, entry in physical['source_files'].items():
    if name in manifest['files']:
        assert entry['sha256'] == manifest['files'][name]['sha256']
assert remote['cdr']['all_worker_candidates_numerically_accepted'] and remote['cdr']['all_materialized_ros_fields_match_original_cdr']
assert physical['recovery_checks_passed']
ulog = remote['ulog']
assert all(row['failsafe'] == 0 for row in ulog['vehicle_status_changes'])
assert ulog['recovery']['landed_and_not_armed_at_log_end']
before, restored = read('diagnostic_parameters_before.json'), read('diagnostic_parameters_restored.json')
assert before.keys() == restored.keys()
assert all((before[k]['param_type'],before[k]['raw_value_hex']) == (restored[k]['param_type'],restored[k]['raw_value_hex']) for k in before)
restore = read('diagnostic_restore_journal_completed.json')
assert restore['restored_verified'] and restore['storage_save']['returncode'] == 0
with (ROOT/'flight_samples.csv').open() as f:
    flight = list(csv.DictReader(f))
assert flight[-1]['armed'] == '0' and flight[-1]['landed'] == '1'
assert not any(row['failsafe'] == '1' for row in flight)
assert launch['runtime_closed'] and launch['sources_unchanged'] and launch['runtime_binaries_unchanged']
lifecycle = read('isolated_live/lifecycle.json')
assert all(value['returncode'] == 0 for value in lifecycle['processes'].values())
resource = [json.loads(s) for s in (ROOT/'hold_v22_resources.jsonl').read_text().splitlines()]
left = max(i for i,r in enumerate(resource) if r['monotonic_ns'] <= start_ns)
right = min(i for i,r in enumerate(resource) if r['monotonic_ns'] >= fault_ns)
bracket = resource[left:right+1]
cs0,cs1 = cpu_stat(bracket[0]['cpu.stat']),cpu_stat(bracket[-1]['cpu.stat'])
throttle = {key:cs1[key]-cs0[key] for key in ['nr_periods','nr_throttled','throttled_usec']}
assert throttle['nr_throttled'] == 0 and throttle['throttled_usec'] == 0
last_source = source[-1]
last_send = read('isolated_live/sends.json')[-2]
source_arrivals = [row for row in ingress if row['kind']==1]
source_by_id = {row['sequence']:row for row in source}
callbacks = [(row['entry_ns']-source_by_id[row['sequence']]['issued_ns'])/1e6 for row in source_arrivals]
active_gc = [row for row in wf['gc_events'] if row['start_ns'] >= start_ns]
report = {
 'kind':'hold22_independent_failure_performance_recovery_review_v1',
 'derived_analysis':True,'full_hold_passed':False,'eligible_for_figure8':False,
 'run':'x650_isolated_live_v1_20261024_120535_11859','launch':'x650_isolated_launch_20261024_120448_11858',
 'task_sha256':launch['task_sha256'],'numerical_library_sha256':wf['engine']['numerical_backend']['library_sha256'],
 'numerical_proof_sha256':wf['engine']['numerical_backend']['software_proof_sha256'],
 'reported_error':meta['error'],'task_start_sim_s':meta['task_start_sim_s'],
 'last_worker_task_time_s':remote['cdr']['candidates'][-1]['proposal']['task_time_s'],
 'requested_hold_s':meta['requested_duration_s'],'first_rejected_gate':events[-1],
 'supervision':{'exact_observation_record':supervision,'producer_to_callback_ms':fault['producer_to_observation_ns']/1e6,
                'semantics':'Independent supervisor callback timestamp minus gate producer timestamp; unlike hold21 this is not recorder sink time.'},
 'worker':{'count':len(worker),'all_numerically_accepted':True,'candidate19_never_entered_worker':True,
   'engine_wall_ms':stats([(r['engine_return_ns']-r['callback_entry_ns'])/1e6 for r in worker]),
   'whole_callback_wall_ms':stats([(r['callback_exit_ns']-r['callback_entry_ns'])/1e6 for r in worker]),
   'whole_callback_thread_cpu_ms':stats([(r['callback_thread_cpu_end_ns']-r['callback_thread_cpu_begin_ns'])/1e6 for r in worker]),
   'snapshot_to_callback_ms':stats([(r['callback_entry_ns']-r['snapshot_issued_ns'])/1e6 for r in worker]),
   'proposal_publish_wall_ms':stats([(r['publish_end_ns']-r['publish_begin_ns'])/1e6 for r in worker]),
   'qp_ms':stats([r['solve_ms'] for r in physical['worker_diagnostics']]),
   'gc_generation_counts_after_handoff':dict(Counter(r['generation'] for r in active_gc)),
   'gc_max_us_after_handoff':max(r['end_ns']-r['start_ns'] for r in active_gc)/1e3,
   'qualification_limit':'18 actual calls only; this short rejected run is not a sustained deadline or full-hold acceptance.'},
 'source_and_ingress':{'source_count':len(source),'pump_count':len(pumps),
   'source_issue_to_publish_return_ms':stats([(r['publish_return_ns']-r['issued_ns'])/1e6 for r in source]),
   'source_issue_to_ingress_entry_ms':stats(callbacks),
   'source_sim_progress_step_s':stats(np.diff([r['control_s'] for r in source])),
   'source_issue_intervals_ms':stats(np.diff([r['issued_ns'] for r in source])/1e6),
   'pump_wall_ms':stats([(r['end_ns']-r['begin_ns'])/1e6 for r in pumps]),
   'extra_callback_counts':dict(Counter(r['extra_callbacks'] for r in pumps)),
   'last_source':last_source,'last_normal_send':last_send,
   'last_source_wall_age_at_gate_rejection_ms':(fault_ns-last_source['issued_ns'])/1e6,
   'last_source_clock_behind_latest_actual_send_sim_ms':(last_send['sim_s']-last_source['control_s'])*1e3,
   'gate19_related_ingress_tail':ingress[-5:],
   'rejected_candidate19_snapshot_cdr_available':False,
   'interpretation':'The reported 16ms difference is a clock observation, not an identified rejection predicate. Fresh wall receipt does not by itself establish all context-history invariants. Exact context rejection predicate is investigated separately; reconstruction must not be presented as original rejected CDR, and this report does not attribute the failure to solver timeout.'},
 'resource_monitor':{'source_sha256':manifest['files']['hold_v22_resources.jsonl']['sha256'],
   'retrieved_rows':len(resource),'retrieved_snapshot_first_ns':resource[0]['monotonic_ns'],'retrieved_snapshot_last_ns':resource[-1]['monotonic_ns'],
   'monitor_start_relative_to_live_start_s':(resource[0]['monotonic_ns']-start_ns)/1e9,
   'covers_entire_live_start_to_gate_rejection':True,'active_window_ms':(fault_ns-start_ns)/1e6,
   'samples_strictly_inside_window':sum(start_ns<=r['monotonic_ns']<=fault_ns for r in resource),
   'bracketing_samples':bracket,'bracketing_interval_ms':(bracket[-1]['monotonic_ns']-bracket[0]['monotonic_ns'])/1e6,
   'cgroup_cpu_stat_deltas':throttle,'bracket_average_cgroup_used_cores':(cs1['usage_usec']-cs0['usage_usec'])*1e3/(bracket[-1]['monotonic_ns']-bracket[0]['monotonic_ns']),
   'sample_spacing_ms':stats(np.diff([r['monotonic_ns'] for r in bracket])/1e6),
   'worker_cpu_frequency_snapshot_khz':{key:stats([int(r['frequency_khz'][key]) for r in bracket]) for key in bracket[0]['frequency_khz']},
   'limits':'Retrieved file snapshot, not a claim that monitoring has ended. Bracketing cgroup counters show no new quota throttling in this interval. About 50ms frequency snapshots do not identify the precise CPU/frequency of each worker callback or bound sub-sample scheduling delays.'},
 'consumption':{key:physical[key] for key in ['all_sends','normal_sends','normal_repeats','all_receipts','individual_joint_commands','per_joint_counter_range','exact_send_receipt_identity_and_raw_joint_counter_value_time_matches','normal_output_after_gate_rejection','stop_zero_send_after_gate_rejection_ms','stop_zero_receipt_after_gate_rejection_ms','stop_zero_send','stop_zero_receipt','no_legacy_publication_overlap','legacy_publication_records_checked']},
 'dispatcher_fault':{k:v for k,v in physical['first_dispatch_fault'].items() if k!='command'},
 'dispatcher_fault_semantics':'reason4 GATE_REJECTED, command age46.057528ms; this is not the prior STALE_COMMAND lease fault.',
 'recovery':{'release':lifecycle['release'],'landed_and_disarmed':True,'final_flight_sample':flight[-1],
   'ulog_source_sha256':ulog['source_sha256'],'ulog_any_recorded_failsafe':False,
   'ulog_status_changes':ulog['vehicle_status_changes'],'ulog_offboard_lost_changes':ulog['offboard_signal_lost_changes'],
   'ulog_recovery':ulog['recovery'],
   'ulog_interpretation':'ULog records AUTO_LAND nav18 at42.492s before offboard signal lost at43.032s; every recorded vehicle_status has failsafe0. These are original ULog microseconds/1e6, without host-time fitting.',
   'restored_typed_parameter_count':len(before),'synchronous_save_returncode':restore['storage_save']['returncode'],
   'runtime_closed':True,'owned_process_returncodes':{k:v['returncode'] for k,v in lifecycle['processes'].items()}},
 'evidence_scope':{'software_numerical_acceptance': '18 in-run proposal checks and their exact raw CDR, not a replacement for full frozen-backend qualification.',
   'simulated_physical_acceptance':'Original TS20 plugin consumption and PX4 ULog recovery verified. Full endpoint-hold performance failed; no hardware or all-fault-scenarios qualification.',
   'large_originals':'ULog, TS20 state, raw CDR and publication trace remained on server; exact SHA/bytes recorded in the remote review files. No large archive produced.'}}
(ROOT/'review_summary.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n',encoding='utf8')
print(json.dumps({'full_hold_passed':False,'worker_count':len(worker),'supervisor_latency_ms':report['supervision']['producer_to_callback_ms'],
                  'recovery_verified':True,'ulog_failsafe':False,'monitor_window_samples':len(bracket),'new_throttled_periods':throttle['nr_throttled']}))
