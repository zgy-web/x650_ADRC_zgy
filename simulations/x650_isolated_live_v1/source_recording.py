"""Loss-accounted source evidence; keep no growing Python message graph in control.

Only immutable binary copies cross the private nonblocking process boundary.
JSON reconstruction and independent count/hash reconciliation happen after exit.
"""
import base64
import hashlib
import json
from pathlib import Path


def materialize(folder, message_types, deserialize, to_dict, write):
    folder = Path(folder)
    sender = json.loads((folder/'source_sender.json').read_text())
    sink = json.loads((folder/'source_recording/manifest.json').read_text())
    raw = folder/'source_recording/diagnostics.jsonl'
    digest = hashlib.sha256(raw.read_bytes()).hexdigest()
    valid = (sender['complete'] and sender['dropped'] == 0
             and sender['attempts'] == sender['sent'] == sink['rows'] == sink['last_sequence']
             and not sink['gaps'] and sink['error'] is None and sink['drained_and_fsynced']
             and digest == sink['records_sha256'])
    if not valid:
        raise RuntimeError('Source evidence count/hash/drain reconciliation failed')
    output = {name: [] for name in message_types}
    feedback = []
    with raw.open() as stream:
        for expected, line in enumerate(stream, 1):
            row = json.loads(line)
            if row['sequence'] != expected:
                raise RuntimeError('Source evidence reordered or missing')
            if row['kind'] == 'feedback':
                value = row['value']
                if value['sequence'] != len(feedback)+1:
                    raise RuntimeError('Feedback source sequence gap')
                feedback.append(value)
            elif row['kind'] == 'ros' and row['topic'] in message_types:
                name = row['topic']
                message = deserialize(base64.b64decode(row['cdr']['base64']), message_types[name])
                output[name].append(to_dict(message))
            else:
                raise RuntimeError('Unexpected source evidence record')
    timing = json.loads((folder/'source_timing.json').read_text())
    if len(feedback) != len(timing['rows']):
        raise RuntimeError('Source evidence does not cover every published snapshot')
    for name, values in output.items():
        write(folder/(name+'.json'), values)
    write(folder/'feedback_sources.json', feedback)
    result = {'complete': True, 'attempts': sender['attempts'], 'dropped': sender['dropped'],
              'feedback_rows': len(feedback), 'message_rows': {k: len(v) for k,v in output.items()},
              'records_sha256': digest, 'natural_gc_policy_unchanged': True,
              'materialization': 'postflight from original private binary copies'}
    write(folder/'source_recording_reconciled.json', result)
    return result
