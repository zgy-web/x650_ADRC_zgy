"""Read-only hold24 physical-consumption and publication review; no ROS node."""
import csv
from collections import Counter
import hashlib
import json
from pathlib import Path

ROOT = Path('/root/autodl-tmp/safe-aerial-manipulator-testbed/runs/x650_isolated_live_v1_20261024_124523_18157')
sources = {}


def raw(name):
    path = ROOT / name
    data = path.read_bytes()
    sources[name] = {'source_path': str(path), 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()}
    return data


def read(name):
    return json.loads(raw(name))


live = lambda name: read('isolated_live/' + name + '.json')
sends, receipts, events, start = [live(n) for n in ['sends', 'receipts', 'events', 'live_start']]
lifecycle = live('lifecycle')
ticks = live('dispatch')
fault = next(row for row in ticks if row['status'] == 3)
fault_ns = fault['actual_monotonic_ns']
by_send = {s['dispatch_sequence']: s for s in sends}
by_receipt = {r['dispatch_sequence']: r for r in receipts}
assert len(by_send) == len(sends) == len(by_receipt) == len(receipts)
assert by_send.keys() == by_receipt.keys()
state_rows = list(csv.DictReader(raw('ts20_state.csv').decode().splitlines()))
states = [{}, {}]
for row in state_rows:
    for j in range(2):
        states[j].setdefault(int(row[f'upper_sequence{j+1}']), row)
lag = []
for i, send in enumerate(sends):
    receipt = by_receipt[send['dispatch_sequence']]
    assert all(send[k] == receipt[k] == start[k] for k in ['session_id', 'epoch'])
    assert send['command_sequence'] == receipt['command_sequence']
    assert receipt['joint_consumed'] and not receipt['failed']
    assert receipt['pva_send_returned'] is (not send['stop_zero'])
    assert receipt['consumed_counter'] == send['expected_counter']
    if i:
        assert send['expected_counter'] == [c+1 for c in sends[i-1]['expected_counter']]
        if not send['stop_zero']:
            assert by_receipt[sends[i-1]['dispatch_sequence']]['observed_monotonic_ns'] <= send['sent_monotonic_ns']
    for j in range(2):
        row = states[j][send['expected_counter'][j]]
        assert abs(float(row[f'u{j+1}']) - send['joint_velocity'][j]) <= 1e-12
        stamp = float(row[f'receipt{j+1}_s'])
        assert abs(stamp - receipt['consumed_sim_s'][j]) <= 1e-9
        assert -.008 <= stamp - send['sim_s'] <= .05
        lag.append(stamp - send['sim_s'])
zeros = [s for s in sends if s['stop_zero']]
assert len(zeros) == 1 and zeros[0] is sends[-1] and zeros[0]['joint_velocity'] == [0., 0.]
assert all(s['sent_monotonic_ns'] < fault_ns for s in sends if not s['stop_zero'])
release = lifecycle['release']
assert release['zero_consumed'] and release['writer_exited']
trace = [json.loads(line) for line in raw('control_execution_trace.jsonl').splitlines()]
publications = [row for row in trace if 'publish_before' in row]
assert not any(start['issued_monotonic_ns'] <= row['publish_before'].get('wall_monotonic_s', -1)*1e9 < release['wall_monotonic_ns'] for row in publications)
ticks = live('dispatch')
report = {
    'kind': 'hold24_physical_consumption_review_v1', 'derived_analysis': True,
    'full_hold_pass': False, 'recovery_checks_passed': True,
    'first_output_fault': fault,
    'dispatch_status_counts': dict(Counter(r['status'] for r in ticks)),
    'first_dispatch_fault': next((r for r in ticks if r['status'] == 3), None),
    'final_dispatch': ticks[-1],
    'all_sends': len(sends), 'normal_sends': len(sends)-1,
    'normal_repeats': sum(r['repeated'] for r in sends if not r['stop_zero']),
    'all_receipts': len(receipts), 'individual_joint_commands': 2*len(sends),
    'per_joint_counter_range': [sends[0]['expected_counter'][0], sends[-1]['expected_counter'][0]],
    'exact_send_receipt_identity_and_raw_joint_counter_value_time_matches': True,
    'normal_send_waited_for_previous_actual_receipt': True,
    'normal_output_after_output_fault': False,
    'delivery_lag_sim_s': lag, 'stop_zero_send': zeros[0],
    'stop_zero_receipt': by_receipt[zeros[0]['dispatch_sequence']],
    'stop_zero_send_after_output_fault_ms': (zeros[0]['sent_monotonic_ns']-fault_ns)/1e6,
    'stop_zero_receipt_after_output_fault_ms': (by_receipt[zeros[0]['dispatch_sequence']]['observed_monotonic_ns']-fault_ns)/1e6,
    'release': release, 'no_legacy_publication_overlap': True,
    'legacy_publication_records_checked': len(publications),
    'source_files': sources,
    'scope': 'Original plugin consumption and command ownership for this failed attempt; not a full-hold or hardware qualification.'}

import base64
from rclpy.serialization import deserialize_message
from flyarm_control_interfaces import msg
def plain(value):
    if hasattr(value, "get_fields_and_field_types"):
        return {key: plain(getattr(value, key)) for key in value.get_fields_and_field_types()}
    if isinstance(value, dict):
        return {key: plain(item) for key, item in value.items()}
    if hasattr(value, "tolist"):
        return value.tolist()
    if hasattr(value, "item"):
        return value.item()
    if not isinstance(value, (str, int, float, bool, type(None))):
        return [plain(item) for item in value]
    return value


kinds={'dispatch':msg.DispatchTick,'sends':msg.ActuatorSend,'receipts':msg.ExecutionReceipt,'events':msg.GateEvent}
expected={'dispatch':ticks,'sends':sends,'receipts':receipts,'events':events};counts=Counter()
for line in raw('isolated_live/source_recording/diagnostics.jsonl').splitlines():
 r=json.loads(line)
 if r['kind']!='ros':continue
 name=r['topic'];message=deserialize_message(base64.b64decode(r['cdr']['base64']),kinds[name]);assert plain(message)==expected[name][counts[name]];counts[name]+=1
assert all(counts[k]==len(expected[k]) for k in kinds)
report.update(all_materialized_ros_fields_match_original_cdr=True,cdr_counts=dict(counts),dispatch_tail=[r for r in ticks if fault['sequence']-5<=r['sequence']<=fault['sequence']+1])


import contextlib,io,sys
sys.path.insert(0,"/root/autodl-tmp/amsim-venv/lib/python3.10/site-packages")
scope={"__name__":"review_module"}
exec('"""Read-only extraction of recovery evidence from the original hold24 ULog.\n\nRun with an existing pyulog environment: python review_ulog.py /path/to/px4.ulg.\nPrints derived JSON; never modifies the ULog or writes server files.\n"""\nimport hashlib\nimport json\nimport math\nfrom pathlib import Path\nimport sys\n\nfrom pyulog import ULog\n\n\ndef scalar(value):\n    value = value.item() if hasattr(value, "item") else value\n    if isinstance(value, float) and not math.isfinite(value):\n        return "NaN" if math.isnan(value) else ("+Inf" if value > 0 else "-Inf")\n    return value\n\n\ndef rows(dataset, fields):\n    return [{key: scalar(dataset.data[key][i]) for key in ["timestamp", *fields]}\n            for i in range(len(dataset.data["timestamp"]))]\n\n\ndef changes(records, fields):\n    return [row for i, row in enumerate(records)\n            if i == 0 or i == len(records) - 1\n            or any(row[field] != records[i - 1][field] for field in fields)]\n\n\ndef main(path):\n    raw = Path(path).read_bytes()\n    ulog = ULog(str(path))\n    get = lambda name: next(d for d in ulog.data_list if d.name == name and d.multi_id == 0)\n    status_fields = ["arming_state", "nav_state", "failsafe"]\n    status = rows(get("vehicle_status"), status_fields)\n    land = rows(get("vehicle_land_detected"), ["landed", "ground_contact", "maybe_landed"])\n    commands = rows(get("vehicle_command"), ["command", "param1", "param2", "param3", "from_external"])\n    acknowledgments = rows(get("vehicle_command_ack"), ["command", "result", "from_external"])\n    offboard = rows(get("offboard_control_mode"), ["position", "velocity", "acceleration", "attitude", "body_rate"])\n    lost = rows(get("failsafe_flags"), ["offboard_control_signal_lost"])\n    armed_seen = any(row["arming_state"] == 2 for row in status)\n    airborne_seen = any(not row["landed"] for row in land)\n    land_commands = [row for row in commands if row["command"] == 21]\n    last_land_command = land_commands[-1]["timestamp"] if land_commands else None\n    landing_after_command = [row for row in land if last_land_command is not None\n                             and row["timestamp"] >= last_land_command and row["landed"]]\n    disarming_after_command = [row for row in status if last_land_command is not None\n                               and row["timestamp"] >= last_land_command and row["arming_state"] != 2]\n    report = {\n        "kind": "hold24_ulog_recovery_extraction_v1", "derived_analysis": True,\n        "source_path": str(path), "source_bytes": len(raw),\n        "source_sha256": hashlib.sha256(raw).hexdigest(),\n        "timestamp_semantics": "Original PX4 ULog microseconds; no fitted conversion to host monotonic or simulation time.",\n        "nonfinite_semantics": "Nonfinite optional command parameters are preserved as NaN/+Inf/-Inf strings.",\n        "start_timestamp_us": int(ulog.start_timestamp), "last_timestamp_us": int(ulog.last_timestamp),\n        "vehicle_status_rows": len(status), "vehicle_status_changes": changes(status, status_fields),\n        "land_detector_rows": len(land), "land_detector_changes": changes(land, ["landed"]),\n        "vehicle_commands": commands, "vehicle_command_acknowledgments": acknowledgments,\n        "offboard_control_mode": {"count": len(offboard), "first": offboard[0], "last": offboard[-1],\n                                  "changes": changes(offboard, ["position", "velocity", "acceleration", "attitude", "body_rate"])},\n        "offboard_signal_lost_changes": changes(lost, ["offboard_control_signal_lost"]),\n        "recovery": {"armed_seen": armed_seen, "airborne_seen": airborne_seen,\n                     "land_command_present": bool(land_commands),\n                     "first_landed_report_after_land_command": landing_after_command[0] if landing_after_command else None,\n                     "first_not_armed_report_after_land_command": disarming_after_command[0] if disarming_after_command else None,\n                     "final_status": status[-1], "final_land_detector": land[-1],\n                     "landed_and_not_armed_at_log_end": bool(land[-1]["landed"] and status[-1]["arming_state"] != 2)},\n        "scope": "Recovery and offboard-state evidence only; not full-window controller performance or command-by-command PVA acceptance."\n    }\n    assert armed_seen and airborne_seen and landing_after_command and disarming_after_command\n    assert report["recovery"]["landed_and_not_armed_at_log_end"]\n    print(json.dumps(report, indent=2, allow_nan=False))\n\n\nif __name__ == "__main__":\n    main(sys.argv[1])\n',scope)
stream=io.StringIO()
with contextlib.redirect_stdout(stream):scope["main"](str(ROOT/"px4.ulg"))
report["ulog"]=json.loads(stream.getvalue())

metadata=read('controller_metadata.json')
report['metadata']={k:metadata.get(k) for k in ('complete','phase','error','task_start_sim_s','task_end_sim_s',
    'landed_and_disarmed','diagnostic_parameters_restored','diagnostic_restore_pending',
    'failed_recovery_handoff','failed_recovery_handoff_error','unverified_owner_land_zero_publish_suppressed')}
report['gate_events']={'count':len(events),'accepted':sum(e['accepted'] for e in events),
    'rejected':[e for e in events if not e['accepted']]}
report['supervision']=live('fault_supervision')
report['lifecycle']=lifecycle
report['worker_final']=live('worker_final')
w=live('worker_timing');wc=w['columns'];wr=[dict(zip(wc,row)) for row in w['rows']]
assert w['complete'] and w['dropped']==0
invalid=[];durations=[];boundaries=[]
for i,row in enumerate(wr):
 fields=('publish_end_ns','diagnostic_complete_begin_ns','diagnostic_complete_end_ns','snapshot_cdr_begin_ns')
 values=[row.get(k,0) for k in fields]
 if any(v<=0 for v in values) or values!=sorted(values):invalid.append({'sequence':row['sequence'],'stamps':dict(zip(fields,values))})
 else:durations.append((values[2]-values[1])/1e6)
 if i+1<len(wr):
  nxt=wr[i+1]
  boundaries.append({'sequence':row['sequence'],'next_sequence':nxt['sequence'],
     'post_publish_ms':(row['callback_exit_ns']-row['publish_end_ns'])/1e6,
     'next_callback_after_previous_exit_ms':(nxt['callback_entry_ns']-row['callback_exit_ns'])/1e6})
report['deferred_diagnostics']={'rows':len(wr),'zero_stamps_are_incomplete':True,'invalid_rows':invalid,
    'completion_error':report['worker_final'].get('diagnostic_completion_error'),
    'complete_phase_order_verified':not invalid,
    'max_finish_ms':max(durations) if durations else None,
    'boundary_tail':boundaries[-6:]}
before=read('diagnostic_parameters_before.json');restored=read('diagnostic_parameters_restored.json')
report['restore_raw_documents_equal']=before==restored
launch=ROOT.parent/'x650_isolated_launch_20261024_124434_18155'
data=(launch/'launch.json').read_bytes();report['launch']=json.loads(data)
sources['launch/launch.json']={'source_path':str(launch/'launch.json'),'bytes':len(data),'sha256':hashlib.sha256(data).hexdigest()}
# Keep full phase data in separate immutable local originals; this report is compact.
report['source_files']=sources
print(json.dumps(report,indent=2,allow_nan=False))
