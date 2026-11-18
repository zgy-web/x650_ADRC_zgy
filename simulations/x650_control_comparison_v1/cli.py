"""Launch isolated comparisons and archive every attempted trial."""
from __future__ import annotations

import argparse
import csv
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import shutil
import subprocess
import sys
import time
import uuid

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
from protocol import CASES, ALGORITHMS
from processes import TRIAL_ENV, process_snapshot, wait_for_exit_snapshot


def write_json(path, value):
    temp = path.with_suffix(path.suffix + '.tmp')
    temp.write_text(json.dumps(value, indent=2, allow_nan=False) + '\n')
    temp.replace(path)


def digest(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_frozen(folder, manifest):
    if {p.name:digest(p) for p in HERE.glob('*.py')} != manifest['source_sha256']:
        raise RuntimeError('Source changed since batch freeze; start a new batch instead of mixing versions')
    if digest(folder/'controller_config.json') != manifest['config_sha256'] or digest(folder/'response_model.json') != manifest['model_sha256']:
        raise RuntimeError('Frozen batch configuration changed')


def read_receipt(record):
    path = record.get('receipt')
    if path and Path(path).is_file():
        record.update(json.loads(Path(path).read_text()))


def run_one(args):
    from amsim.run_manager import run
    from amsim.paths import results_dir
    os.environ['X650_COMPARE_ALGORITHM'] = args.algorithm
    os.environ['X650_COMPARE_CASE'] = args.case
    os.environ['X650_COMPARE_CONFIG'] = str(args.config.resolve())
    os.environ['X650_COMPARE_MODEL'] = str(args.model.resolve())
    partition = os.environ.setdefault('GZ_PARTITION', 'amsim_x650_compare')
    trial_id = uuid.uuid4().hex
    os.environ[TRIAL_ENV] = trial_id
    destination = results_dir()
    destination.mkdir(parents=True, exist_ok=True)
    audit_path = (args.receipt.with_suffix('.audit.json') if args.receipt else
                  destination/f'process_audit_{trial_id}.json').resolve()
    receipt = dict(algorithm=args.algorithm, case=args.case, result_dir=None, returncode=1,
                   process_audit_file=str(audit_path), process_audit_trial_id=trial_id,
                   process_audit_accepted=False)
    audit = dict(schema_version=1, trial_id=trial_id, algorithm=args.algorithm, case=args.case,
                 partition=partition, launch_attempted=False, accepted=False, before=None, after=None)
    original_runs = set(destination.glob('x650_control_comparison_*'))
    # All runner instances in this checkout share a PX4 runtime state file and UDP ports.
    with (ROOT/'.amsim'/'x650_comparison.lock').open('a') as lock:
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
            audit['before'] = process_snapshot(partition)
            write_json(audit_path, audit)
            if not audit['before']['clean']:
                raise RuntimeError(f'Simulation processes remain or their state is unknown; see {audit_path}')
            audit['launch_attempted'] = True
            write_json(audit_path, audit)
            folder = run('x650_control_comparison', headless=not args.gui)
            receipt.update(result_dir=str(folder.resolve()), returncode=0)
        except Exception as exc:
            receipt['error'] = str(exc)
            print(f'Comparison failed: {exc}', file=sys.stderr)
        finally:
            if audit['launch_attempted'] and receipt['result_dir'] is None:
                created = set(destination.glob('x650_control_comparison_*')) - original_runs
                if len(created) == 1:
                    receipt['result_dir'] = str(next(iter(created)).resolve())
            audit['after'] = wait_for_exit_snapshot(partition)
            audit['accepted'] = bool(audit['launch_attempted'] and audit['before']['clean']
                                     and audit['after']['clean'])
            write_json(audit_path, audit)
            if receipt['result_dir']:
                shutil.copyfile(audit_path, Path(receipt['result_dir'])/'process_audit.json')
            receipt.update(process_audit_sha256=digest(audit_path), process_audit_accepted=audit['accepted'])
            if not audit['accepted']:
                receipt['returncode'] = 2
                receipt.setdefault('error', f'Process audit failed; see {audit_path}')
            if args.receipt:
                write_json(args.receipt, receipt)
            print(json.dumps(receipt), flush=True)
    return receipt['returncode']


def fit(args):
    path = args.run_dir / 'control_trace.csv'
    result = json.loads((args.run_dir / 'result.json').read_text())
    if result.get('case') != 'identify' or not result.get('execution_success'):
        raise ValueError('Fit requires a completed identification trial')
    with path.open() as stream:
        rows = [r for r in csv.DictReader(stream) if float(r['t']) >= 0]
    from identification import fit_response
    model = fit_response(rows)
    model.update(model_sha256=result['model_sha256'],
                 source_run=str(args.run_dir.resolve()), source_trace_sha256=digest(path))
    if args.output.exists() and not args.replace:
        raise FileExistsError('Output exists; use --replace to replace the selected response model explicitly')
    write_json(args.output, model)
    print(json.dumps(model, indent=2))
    return 0 if model['identified'] else 1


def batch(args):
    from analysis import build_report as report
    if args.resume:
        folder = args.resume.resolve()
        manifest = json.loads((folder/'manifest.json').read_text())
    else:
        response = json.loads(args.model.read_text())
        config = json.loads(args.config.read_text())
        if (not response.get('identified') or not response.get('validation_accepted') or
                response.get('model_sha256') != config['model_sha256']) and not args.development:
            raise ValueError('Formal batch requires an identified response model; use fit first')
        folder = (args.output or ROOT/'runs'/time.strftime('x650_comparison_%Y%m%d_%H%M%S')).resolve()
        folder.mkdir(parents=True, exist_ok=False)
        (folder/'trials').mkdir()
        for source, name in [(args.config,'controller_config.json'),(args.model,'response_model.json')]:
            (folder/name).write_bytes(source.read_bytes())
        source_hashes = {p.name:digest(p) for p in HERE.glob('*.py')}
        manifest = dict(schema_version=2, process_audit_required=True,
            partition=os.environ.get('GZ_PARTITION','amsim_x650_compare'), purpose='development' if args.development else 'formal',
            created=time.strftime('%Y-%m-%d %H:%M:%S %z'), runs=[], source_sha256=source_hashes,
            config_sha256=digest(folder/'controller_config.json'), model_sha256=digest(folder/'response_model.json'),
            schedule=[], repeated_runs='Fresh processes; sensor noise configuration unchanged; no claim of identical noise realizations')
        for repetition in range(args.repeats):
            algorithms = list(args.algorithms)
            shift = repetition % len(algorithms)
            algorithms = algorithms[shift:]+algorithms[:shift]
            for case in args.cases:
                for algorithm in algorithms:
                    manifest['schedule'].append(dict(algorithm=algorithm,case=case,repetition=repetition+1))
        write_json(folder/'manifest.json', manifest)
    check_frozen(folder,manifest)
    if manifest.get('process_audit_required') and manifest['partition'] != os.environ.get('GZ_PARTITION','amsim_x650_compare'):
        raise ValueError('Batch partition changed since the manifest was created')
    env = dict(os.environ, AMSIM_RESULTS_DIR=str(folder/'trials'))
    print('Batch results: '+str(folder), flush=True)
    for index, item in enumerate(manifest['schedule']):
        if index < len(manifest['runs']) and manifest['runs'][index].get('finished'):
            if manifest.get('process_audit_required') and not manifest['runs'][index].get('process_audit_accepted'):
                print('Previous trial failed its process audit; start a new batch after resolving it.', file=sys.stderr)
                return 2
            continue
        check_frozen(folder,manifest)
        receipt_path = folder/f'trial_{index+1:02d}_receipt_{time.time_ns()}.json'
        log_path = folder/f'trial_{index+1:02d}.log'
        record = dict(item,index=index+1,finished=False,result_dir=None,returncode=None,receipt=str(receipt_path),
                      process_audit_file=str(receipt_path.with_suffix('.audit.json')))
        if index < len(manifest['runs']):
            # An interrupted process is an attempted failure, not silently erased.
            read_receipt(manifest['runs'][index])
            manifest.setdefault('interrupted_attempts',[]).append(manifest['runs'][index])
            manifest['runs'][index] = record
        else:
            manifest['runs'].append(record)
        write_json(folder/'manifest.json',manifest)
        print(f"[{index+1}/{len(manifest['schedule'])}] {item}",flush=True)
        command = [sys.executable, str(__file__), 'run', '--algorithm',item['algorithm'],'--case',item['case'],
            '--config',str(folder/'controller_config.json'),'--model',str(folder/'response_model.json'),
            '--receipt',str(receipt_path)]
        with log_path.open('a') as log:
            child = subprocess.Popen(command,env=env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
            try:
                rc = child.wait(timeout=420)
            except (KeyboardInterrupt,subprocess.TimeoutExpired) as exc:
                os.killpg(child.pid,signal.SIGINT)
                try:
                    child.wait(timeout=60)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    child.wait(timeout=15)
                record['error'] = type(exc).__name__
                read_receipt(record)
                write_json(folder/'manifest.json',manifest)
                raise
        read_receipt(record)
        record.update(returncode=rc,finished=True,log=str(log_path))
        write_json(folder/'manifest.json',manifest)
        partial = report(folder)
        if manifest.get('process_audit_required') and not (
            record.get('process_audit_accepted') and
            partial['runs'][-1].get('process_audit', {}).get('verified')
        ):
            print(f'Batch stopped on missing or failed process audit: {receipt_path}', file=sys.stderr)
            return 2
    summary = report(folder)
    if manifest['purpose'] == 'formal' and not summary.get('evidence_accepted',False):
        print('Formal evidence check failed; inspect summary.json',file=sys.stderr)
        return 2
    return 0 if all(r['returncode']==0 for r in manifest['runs']) else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    subs = parser.add_subparsers(dest='command',required=True)
    for name in ('run','batch'):
        p = subs.add_parser(name)
        p.add_argument('--config',type=Path,default=HERE/'config.json')
        p.add_argument('--model',type=Path,default=HERE/'response_model.json')
        if name == 'run':
            p.add_argument('--algorithm',choices=ALGORITHMS,default='pid')
            p.add_argument('--case',choices=CASES,default='hover')
            p.add_argument('--gui',action='store_true')
            p.add_argument('--receipt',type=Path)
        else:
            p.add_argument('--algorithms',nargs='+',choices=ALGORITHMS,default=list(ALGORITHMS))
            p.add_argument('--cases',nargs='+',choices=CASES,default=['hover','step','arm','force'])
            p.add_argument('--repeats',type=int,choices=range(1,101),default=3)
            p.add_argument('--output',type=Path)
            p.add_argument('--resume',type=Path)
            p.add_argument('--development',action='store_true')
    p=subs.add_parser('fit')
    p.add_argument('run_dir',type=Path)
    p.add_argument('--output',type=Path,default=HERE/'response_model.json')
    p.add_argument('--replace',action='store_true')
    p=subs.add_parser('report')
    p.add_argument('batch_dir',type=Path)
    args=parser.parse_args()
    if args.command=='run':
        return run_one(args)
    if args.command=='fit':
        return fit(args)
    if args.command=='batch':
        return batch(args)
    from analysis import build_report as report
    report(args.batch_dir)
    return 0


if __name__=='__main__':
    raise SystemExit(main())
