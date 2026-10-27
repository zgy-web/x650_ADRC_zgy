#!/usr/bin/env python3
"""Ground-only recovery of one retained, typed PX4 parameter transaction.

Requires an already owned simulation runtime with exactly the journal's model.
Does not arm, fly, delete an unverified journal or modify its original evidence.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[1]
for directory in (ROOT/'src',ROOT/'simulations/x650_wholebody_mpc_v1'):
    sys.path.insert(0,str(directory))


def main():
    from amsim import ScenarioContext
    from diagnostic_parameters import DiagnosticParameterSession,PRECISION_PROFILE,PID_CANDIDATES,PENDING_JOURNAL
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    a=p.parse_args();journal=json.loads(PENDING_JOURNAL.read_bytes())
    if (journal['kind']!='x650_diagnostic_restore_pending' or journal['purpose']!='research'
            or journal['profile_name']!='ev_precision_xyz_velocity_v1'
            or journal['pid_tuning_candidate_name']!='velocity_high'):
        raise ValueError('Unsupported journal kind; explicit review required')
    a.output.mkdir(parents=True,exist_ok=False)
    (a.output/'pending_original.json').write_bytes(PENDING_JOURNAL.read_bytes())
    with ScenarioContext() as ctx:
        if ctx.model!=journal['model']:raise ValueError('Runtime model differs from pending transaction')
        ctx.vehicle.wait_ready()
        originals=journal['original']
        session=DiagnosticParameterSession(ctx,PRECISION_PROFILE,a.output,
            protected_pid_records={k:v for k,v in originals.items() if k.startswith(('MC_','MPC_'))},
            pid_candidate=PID_CANDIDATES['velocity_high'],purpose='research',
            operating_condition=journal['operating_condition'],journal_path=a.output/'unused_construction_guard.json')
        if set(originals)!=set(session.names):raise ValueError('Saved original parameter set differs')
        if journal['allowlist']!=session.mutable_parameters:raise ValueError('Saved mutable allowlist differs')
        session.session_id=journal['session_id'];session.journal_path=PENDING_JOURNAL
        session.original=deepcopy(originals);session.touched=bool(journal['touched'])
        session.document.update(recovery_of_session_id=journal['session_id'],before=deepcopy(originals),
            restore_journal_path=str(PENDING_JOURNAL),ground_only_recovery=True)
        session.restore()
        print(json.dumps({'restored_verified':session.document['restored_verified'],
            'pending_removed':not PENDING_JOURNAL.exists(),'output':str(a.output)},indent=2))


if __name__=='__main__':main()
