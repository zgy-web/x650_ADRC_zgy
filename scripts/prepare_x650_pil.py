#!/usr/bin/env python3
"""Build a local, immutable continuous-stream corpus and code-only PIL package."""
import argparse
from pathlib import Path
from prepare_x650_orin_benchmark import ROOT, atomic_write, json_bytes, identity, package_sources
from prepare_x650_abi4_benchmark import Evidence, GROUPS, recover
from prepare_x650_recorded_core_benchmark import decode_cdr, message_layout

SOURCES = tuple('control_packages/x650_wholebody_v1/'+p for p in (
    'pil/CMakeLists.txt', 'pil/worker.cpp', 'pil/test_reference.cpp', 'benchmark/include/benchmark_io.hpp',
    'native/mpc_core.hpp', 'native/joint_safety.hpp')) + tuple('ros2_ws/src/'+p for p in (
    'flyarm_mpc_cpp/include/flyarm_mpc_cpp/worker_contract.hpp',
    'flyarm_mpc_cpp/include/flyarm_mpc_cpp/config_io.hpp',
    'flyarm_mpc_cpp/include/flyarm_mpc_cpp/diagnostics_json.hpp',
    'flyarm_mpc_cpp/include/flyarm_mpc_cpp/trajectory_plan.hpp',
    'flyarm_state_cpp/include/flyarm_state_cpp/source_core.hpp',
    'flyarm_control_gateway/include/flyarm_control_gateway/shadow_gate.hpp',
    'flyarm_control_gateway/include/flyarm_control_gateway/timing_profile.hpp'))


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    protocol = dict(kind='x650_pil_protocol_v1', simulation_only=True,
        continuous_memory=True, state_restored_each_step=False, core_abi=4,
        mpc_period_ms=40, dispatch_period_ms=20, core_limit_ms=50,
        server_issue_to_result_limit_ms=50, command_hold_ms=160,
        source_and_nx_clock_subtraction=False, persistent_nx_data=False,
        numerical_tolerance=dict(position_reference=1e-9, other=1e-7,
            joint_acceleration_reference='1e-7 / actual_dt'),
        test_order=['recorded continuous streams', 'faults without actuators',
                    'actual simulation shadow', 'new complete hold', 'normal eight'],
        scope='Recorded streams validate continuous code, not new physical feedback or NX closed-loop flight')
    atomic_write(args.output/'protocol.json', json_bytes(protocol))
    receipts = {'multirate': ROOT/'docs/validation/cpp_multirate_20261026/multirate_evidence_receipt.json',
        'robustness': ROOT/'docs/validation/cpp_multirate_robustness_20261027/delivery_v1/multirate_robustness_evidence_receipt.json'}
    layouts = {k: message_layout((ROOT/f'ros2_ws/src/flyarm_control_interfaces/msg/{k}.msg').read_bytes())
               for k in ('LiveStart', 'ControlSnapshot')}
    manifest = dict(kind='x650_pil_preparation_v1', streams=[], protocol=identity(args.output/'protocol.json'))
    for family, receipt in receipts.items():
        evidence = Evidence(receipt)
        try:
            for name, (source, run, launch, count, _) in GROUPS.items():
                if source != family:
                    continue
                # The native mathematics and all wire fields remain byte bound.
                # The new cross-architecture reference comparator is the only
                # allowed adapter change; retain both versions' hashes explicitly.
                config, cases, provenance = recover(evidence, name, allow_reference_contract_change=True)
                prefix = 'runs/'+run+'/isolated_live/'
                handoff = decode_cdr(evidence.read(prefix+'worker_live_start.cdr'), layouts['LiveStart'])
                import json
                startup = json.loads(evidence.read('runs/'+launch+'/original_startup_manifest.json'))
                start = dict(op='start', scope='recorded_stream', config=config,
                    trajectory=startup['trajectory'], handoff=handoff)
                steps = []
                for i, case in enumerate(cases, 1):
                    snapshot = decode_cdr(evidence.read(prefix+f'native_records/{i:06d}.snapshot.cdr'), layouts['ControlSnapshot'])
                    steps.append(dict(request=dict(op='step', snapshot=snapshot), oracle=case['oracle']))
                doc = dict(kind='x650_pil_recorded_stream_v1', start=start, steps=steps,
                    provenance=provenance, continuous_memory=True, historical_physics=True,
                    new_physical_closed_loop=False)
                path = args.output/(name+'.json');atomic_write(path, json_bytes(doc))
                manifest['streams'].append(dict(name=name, count=count, **identity(path)))
        finally:
            evidence.close()
    manifest['source_package'] = package_sources(ROOT, args.output/'code_only_sources.tar.gz', source_paths=SOURCES)
    manifest['complete'] = True
    atomic_write(args.output/'manifest.json', json_bytes(manifest))
    print(json_bytes({k: manifest[k] for k in ('complete', 'streams')}).decode())


if __name__ == '__main__':
    main()
