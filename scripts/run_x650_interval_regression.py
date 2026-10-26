#!/usr/bin/env python3
"""Fixed-original-state interval-constraint regression; no ROS or actuator I/O.

Every input restores its original C++ Memory. This is NOT a closed-loop replay.
Native candidate propagation is repeated with the original canonical grid and
the hash-bound, unchanged TS20 library. All exits and failures are retained.
"""
from __future__ import annotations

import argparse
import ctypes
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np

import audit_x650_recovery_replay as common


def write_new(path, value):
    with Path(path).open('x', encoding='utf-8', newline='\n') as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write('\n')


def canonical_native_check(fixture, raw, predictor):
    """Re-run the unchanged native model, independently rebuilding its grid."""
    c, inp, d = fixture['config'], fixture['input'], raw['diagnostics']
    t = float(inp['t'])
    elapsed, nodes = 0., []
    for i in range(c['nodes']):
        elapsed += inp['dt'] if i == 0 else c['node_dt']
        nodes.append(t+elapsed)
    packets = d['candidate_packets']
    common.check_packets(fixture, packets, d['plan'])
    knots = [p['send_time']+c['motion'][4] for p in packets]
    queries = sorted(set([t]+nodes+[k for k in knots if t <= k <= nodes[-1]]))
    state = inp['state']
    start = t-(state['control_time']-state['joint_stamp'])
    boundaries = sorted(set([start]+queries+[k for k in knots if start < k <= queries[-1]]))
    common.require(len(queries) == len(d['query_times']) and
        np.max(np.abs(np.array(queries)-d['query_times'])) <= 1e-12,
        'Candidate added or changed canonical queries')
    common.require(len(boundaries) == len(d['candidate_boundary_times']) and
        np.max(np.abs(np.array(boundaries)-d['candidate_boundary_times'])) <= 1e-12,
        'Candidate changed canonical propagation boundaries')
    # Exact expected grid, not a mesh enlarged by the new constraint times.
    arrays = [np.ascontiguousarray(x, dtype=np.float64) for x in (
        c['motion'], c['kp']+c['kd'], state['q']+state['dq'], knots,
        [[c['kp'][j]*p['q_ref'][j]+c['kd'][j]*p['dq_ref'][j]+p['tau_ff'][j]
          for j in range(2)] for p in packets], boundaries)]
    model, gains, initial, knots_a, forcing, boundary_a = arrays
    count = len(boundaries)
    states = np.empty((count,4),dtype=np.float64)
    modes = np.empty((count,2),dtype=np.int32)
    zeros = np.empty(count,dtype=np.int32)
    envelope = np.empty(8,dtype=np.float64)
    extrema = np.empty((4096,5),dtype=np.float64)
    nevents, nsteps, error = ctypes.c_int(), ctypes.c_int(), ctypes.create_string_buffer(512)
    dp, ip = ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int)
    lib = ctypes.CDLL(str(predictor))
    fn = lib.ts20_prediction_batch
    fn.restype = ctypes.c_int
    fn.argtypes = [dp,dp,ctypes.c_double,ctypes.c_double,dp,dp,dp,ctypes.c_int,
        dp,ctypes.c_int,ctypes.c_int,dp,ip,ip,dp,dp,ctypes.c_int,ip,ip,ctypes.c_char_p,ctypes.c_int]
    p = lambda a: a.ctypes.data_as(dp)
    status = fn(p(model),p(gains),c['integration_step'],start,p(initial),p(knots_a),
        p(forcing),len(packets),p(boundary_a),count,1,p(states),modes.ctypes.data_as(ip),
        zeros.ctypes.data_as(ip),p(envelope),p(extrema),4096,ctypes.byref(nevents),
        ctypes.byref(nsteps),error,512)
    common.require(status == 0,error.value.decode(errors='replace'))
    indices = [boundaries.index(q) for q in queries]
    comparisons = ((states[indices].ravel(),d['candidate_states']),
        (envelope,d['candidate_envelope']),
        (extrema[:nevents.value].ravel(),d['candidate_extrema']))
    maximum = 0.
    for left, right in comparisons:
        right = np.asarray(right,dtype=float)
        common.require(left.shape == right.shape and np.isfinite(left).all()
            and np.isfinite(right).all(),'Canonical numeric evidence shape/finite values differ')
        if left.size:
            maximum = max(maximum,float(np.max(np.abs(left-right))))
    common.require(maximum <= 1e-13,'Canonical TS20 repropagation disagrees with candidate')
    common.require(nsteps.value == d['candidate_substeps']
        and modes.ravel().tolist() == d['candidate_modes']
        and zeros.tolist() == d['candidate_zero_events'],'Canonical substeps/modes/zero events differ')
    return dict(kind='x650_interval_canonical_repropagation_v1',passed=True,
        canonical_query_times=queries,canonical_boundary_times=boundaries,
        states=states.tolist(),envelope=envelope.tolist(),extrema=extrema[:nevents.value].tolist(),
        substeps=nsteps.value,maximum_absolute_error=maximum,
        scope='Same unchanged numerical model on independently rebuilt original grid; not hardware or continuous-time mathematical certification')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('original-report','executable','predictor','output'):
        parser.add_argument('--'+name,type=Path,required=True)
    parser.add_argument('--sequences',default='1,378,379,380,383,384,428,429')
    parser.add_argument('--core-sha256',required=True)
    parser.add_argument('--optimization-sha256',required=True)
    parser.add_argument('--baseline-raw-dir',type=Path)
    args = parser.parse_args()
    import audit_x650_interval_constraints as checker
    common.require(not args.output.exists(),'Evidence directory must be new')
    args.output.mkdir(parents=True)
    report = common.strict_json(args.original_report)
    common.require(report.get('all_original_outcomes_reproduced') is True
        and report.get('memory_reconstructed_in_cpp') is True
        and report.get('history_from_proposals') is False,'Original Memory reconstruction not verified')
    expected = dict(core=args.core_sha256,optimization=args.optimization_sha256,
        predictor=report['loaded_code']['predictor']['sha256'])
    common.require(common.identity(args.predictor)['sha256'] == expected['predictor'],
        'Original TS20 predictor changed')
    sequences = ([row['sequence'] for row in report['rows']] if args.sequences == 'all'
        else [int(x) for x in args.sequences.split(',')])
    common.require(len(sequences) == len(set(sequences)),'Duplicate regression sequence')
    sources = {row['sequence']:row for row in report['rows']}
    result = dict(kind='x650_interval_fixed_original_regression_v1',complete=False,passed=False,
        original_report=common.identity(args.original_report),executable=common.identity(args.executable),
        expected_libraries=expected,sequences=sequences,rows=[],started_unix=time.time(),
        closed_loop=False,actuator_io=False,delay_injection=False,
        scope='Each call restores its own original Memory and actual history; no counterfactual closed-loop claim')
    environment = os.environ.copy()
    environment.pop('LD_LIBRARY_PATH',None)
    environment.update(OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1',MKL_NUM_THREADS='1')
    for sequence in sequences:
        folder = args.output/f'{sequence:06d}'
        folder.mkdir()
        row = dict(sequence=sequence,passed=False,error=None)
        try:
            source = sources[sequence]['fixture']
            common.require(common.identity(source['path']) == source,'Original input receipt changed')
            fixture = common.strict_json(source['path'])
            common.require(fixture['memory']['step_count'] == sequence-1,'Original Memory prefix differs')
            raw_path = folder/'raw.json'
            command = [str(args.executable),str(args.predictor),expected['predictor'],
                source['path'],source['sha256'],str(raw_path)]
            proc = subprocess.run(command,env=environment,capture_output=True,timeout=15.)
            (folder/'stdout.txt').write_bytes(proc.stdout)
            (folder/'stderr.txt').write_bytes(proc.stderr)
            write_new(folder/'process.json',dict(command=command,returncode=proc.returncode))
            raw = common.strict_json(raw_path)
            row.update(returncode=proc.returncode,returned=raw.get('returned'),
                core_error=raw.get('error'),core_ms=raw['diagnostics']['compute_ms'],
                subproblems=raw['diagnostics']['subproblems'],
                interval_rows=raw['diagnostics'].get('interval_rows'),raw=common.identity(raw_path))
            audit = checker.audit(fixture,raw,source['sha256'],expected)
            write_new(folder/'audit.json',audit)
            common.require(proc.returncode == 0 and raw.get('returned') is True,
                'Controller rejected original fixed input: '+str(raw.get('error')))
            proof = canonical_native_check(fixture,raw,args.predictor)
            proof.update(input_sha256=source['sha256'],raw_sha256=row['raw']['sha256'],
                predictor_sha256=expected['predictor'])
            write_new(folder/'canonical_proof.json',proof)
            common.require(audit.get('passed') is True and proof['passed'],'Interval audit failed')
            if args.baseline_raw_dir is not None:
                baseline_path = args.baseline_raw_dir/f'{sequence:06d}.raw.json'
                if baseline_path.is_file():
                    baseline = common.strict_json(baseline_path)
                    common.require(baseline['input_sha256'] == source['sha256']
                        and baseline['memory_before'] == fixture['memory'],
                        'Normal-path baseline is a different original input')
                    if not raw['diagnostics']['interval_repairs']:
                        common.require(baseline['returned'] is True
                            and baseline['output'] == raw['output']
                            and baseline['memory_after'] == raw['memory_after'],
                            'Untriggered normal path changed output or committed Memory')
                        fields = ('A','b','H','g','C','lower','upper','solution','plan',
                            'predicted_states','candidate_envelope')
                        common.require(all(baseline['diagnostics'][k] == raw['diagnostics'][k]
                            for k in fields),'Untriggered normal path changed numerical problem/result')
                        row['normal_path_exact_equivalence'] = dict(passed=True,
                            baseline=common.identity(baseline_path),checked_fields=list(fields),
                            complete_output_and_memory=True)
            row.update(passed=True,canonical_maximum_error=proof['maximum_absolute_error'],
                candidate_envelope=proof['envelope'])
        except Exception as error:
            row['error'] = str(error)
        result['rows'].append(row)
        write_new(folder/'result.json',row)
        print(json.dumps(row,allow_nan=False),flush=True)
    result.update(complete=True,passed=all(row['passed'] for row in result['rows']),
        passed_cases=sum(row['passed'] for row in result['rows']),finished_unix=time.time())
    write_new(args.output/'report.json',result)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
