"""Join local worker observations to original CDR; never repair timestamps."""
import json
from pathlib import Path


def verify_worker_phases(live, calls, events):
    document = json.loads((Path(live)/'worker_timing.json').read_text())
    def require(value, message):
        if not value:
            raise ValueError('Worker phase evidence: '+message)
    require(document.get('kind') == 'isolated_worker_phase_timing_v1'
            and document.get('complete') is True and document.get('dropped') == 0,
            'missing or incomplete bounded trace')
    columns = document['columns']
    require(len(columns) == len(set(columns)), 'duplicate columns')
    raw = document['rows']
    require(len(raw) == len(calls), 'trace/CDR count differs')
    require(all(len(row) == len(columns) for row in raw), 'row width differs')
    rows = [dict(zip(columns, row)) for row in raw]
    # An orderly shutdown may leave one completed candidate awaiting its gate
    # callback. Every earlier candidate must have exactly one accepted event,
    # in order; a dict alone would hide duplicates or unrelated event IDs.
    require(len(events) <= len(calls) <= len(events)+1, 'gate/CDR count differs')
    require(all(e['accepted'] is True for e in events), 'unaccepted gate event')
    expected = [int(s.sequence) for s,_,_ in calls[:len(events)]]
    require([int(e['proposal_sequence']) for e in events] == expected
            and [int(e['snapshot_sequence']) for e in events] == expected,
            'gate events do not match the recorded candidate prefix')
    require(len(set(expected)) == len(expected), 'duplicate recorded candidate sequence')
    event_by_sequence = {int(e['proposal_sequence']):e for e in events}
    durations = {key:[] for key in ('engine_ms', 'publish_ms', 'publish_thread_cpu_ms',
        'post_publish_recording_ms', 'compute_end_to_gate_ms', 'publish_begin_to_gate_ms')}
    for row, (snapshot, proposal, _) in zip(rows, calls):
        sequence = int(snapshot.sequence)
        require(row['sequence'] == sequence == int(proposal.sequence)
                == int(proposal.snapshot_sequence), 'sequence differs')
        require(snapshot.session_id == proposal.session_id and snapshot.epoch == proposal.epoch,
                'proposal/session identity differs')
        require(row['snapshot_issued_ns'] == int(snapshot.issued_monotonic_ns)
                and row['proposal_compute_end_ns'] == int(proposal.compute_end_monotonic_ns),
                'original CDR timestamps differ')
        stamps = [row['callback_entry_ns'], int(proposal.compute_start_monotonic_ns),
            int(proposal.compute_end_monotonic_ns), row['engine_return_ns'],
            row['publish_begin_ns'], row['publish_end_ns'], row['snapshot_cdr_begin_ns'],
            row['snapshot_cdr_end_ns'], row['proposal_cdr_begin_ns'], row['proposal_cdr_end_ns'],
            row['offer_begin_ns'], row['pickle_begin_ns'], row['pickle_end_ns'],
            row['socket_send_begin_ns'], row['socket_send_end_ns'], row['sender_end_ns'],
            row['offer_end_ns'], row['callback_exit_ns']]
        require(all(type(t) is int and t > 0 for t in stamps)
                and all(b >= a for a,b in zip(stamps,stamps[1:])), 'phase chronology differs')
        require(row['diagnostic_offered'] == 1, 'diagnostic not offered')
        require(row['publish_thread_cpu_end_ns'] >= row['publish_thread_cpu_begin_ns'],
                'thread CPU clock regressed')
        durations['engine_ms'].append((row['engine_return_ns']-row['callback_entry_ns'])/1e6)
        durations['publish_ms'].append((row['publish_end_ns']-row['publish_begin_ns'])/1e6)
        durations['publish_thread_cpu_ms'].append(
            (row['publish_thread_cpu_end_ns']-row['publish_thread_cpu_begin_ns'])/1e6)
        durations['post_publish_recording_ms'].append((row['callback_exit_ns']-row['publish_end_ns'])/1e6)
        if sequence in event_by_sequence:
            event = event_by_sequence[sequence]
            require(event['session_id'] == snapshot.session_id and event['epoch'] == snapshot.epoch,
                    'gate/session identity differs')
            received = int(event['received_monotonic_ns'])
            require(received >= row['publish_begin_ns'], 'gate predates publishing')
            require(event['roundtrip_ns'] == received-int(snapshot.issued_monotonic_ns),
                    'gate roundtrip differs from original CDR issue time')
            durations['compute_end_to_gate_ms'].append((received-row['proposal_compute_end_ns'])/1e6)
            durations['publish_begin_to_gate_ms'].append((received-row['publish_begin_ns'])/1e6)
    def summarize(values):
        ordered=sorted(values)
        return {'count':len(ordered), 'min':min(ordered), 'max':max(ordered),
                'median':ordered[len(ordered)//2], 'p95':ordered[int((len(ordered)-1)*.95)]} if ordered else None
    ingress_path=Path(live)/'ingress_timing.json'
    ingress=json.loads(ingress_path.read_text())
    require(ingress.get('kind')=='isolated_ingress_timing_v1'
            and ingress.get('complete') is True and ingress.get('overflow')==0,
            'missing or incomplete ingress trace')
    ingress_columns=ingress['columns']
    require(len(ingress_columns)==len(set(ingress_columns)), 'duplicate ingress columns')
    require(all(len(r)==len(ingress_columns) for r in ingress['rows']), 'ingress row width differs')
    ingress_rows=[dict(zip(ingress_columns,r)) for r in ingress['rows']]
    admitted_rows=[r for r in ingress_rows if r['kind']==4 and r['forward']]
    admitted={int(r['sequence']):r for r in admitted_rows}
    require(len(admitted)==len(admitted_rows), 'duplicate forwarded ingress admission')
    for snapshot,_,_ in calls:
        sequence=int(snapshot.sequence)
        require(sequence in admitted,'missing recorded admission')
        admission=admitted[sequence]
        require(admission['source_issued_ns']==int(snapshot.issued_monotonic_ns)
                and admission['latest_control_s']==float(snapshot.control_time_s)
                and abs(admission['latest_control_s']-admission['last_control_s']-float(snapshot.dt_s))<=1e-10,
                'admission rewrote source time or actual dt')
        if sequence>1:
            require(admission['released_sequence']==sequence-1
                    and admission['last_admitted_sequence']==sequence-1,
                    'next solve lacks actual-send release')
    return {'verified':True, 'pairs':len(rows), 'original_cdr_timestamps_unchanged':True,
        'ingress_trace_complete':True, 'actual_dt_and_send_release_verified':True,
        'durations':{key:summarize(value) for key,value in durations.items()},
        'scope':'observed wall/CPU phase durations; not a hard real-time upper bound; gate can receive before publish returns'}
