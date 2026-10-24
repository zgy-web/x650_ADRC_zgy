#!/usr/bin/env python3
"""Wait for a fresh read-only ground snapshot; never publish or change runtime."""
import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
import time


def ready_snapshot(summary, now, minimum_wall_s):
    """Reuse every existing predicate and additionally require fresh land."""
    if summary.failure or now-summary.started_wall < minimum_wall_s:
        return None
    age = summary.topics.get('land', {}).get('last_wall_age_s')
    if age is None or not 0. <= age <= 1.:
        return None
    report = summary.report(now)
    return report if report['passed'] and report['ground_disarmed_verified'] else None


def probe(root, minimum_wall_s, timeout_wall_s):
    if not (math.isfinite(minimum_wall_s) and math.isfinite(timeout_wall_s)
            and 1. <= minimum_wall_s < timeout_wall_s <= 60.):
        raise ValueError('Require 1 <= minimum-wall-s < timeout-wall-s <= 60')
    reference = root/'scripts/probe_px4_estimator_telemetry.py'
    spec = importlib.util.spec_from_file_location('background_telemetry_reference', reference)
    module = importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
    sys.path.insert(0, str(root/'src'))
    import rclpy
    from amsim.api.node import ApiNode
    from amsim.runtime.model_metadata import resolve_active_model
    from amsim.runtime.process import status
    runtime = status()
    if runtime.get('running') is not True:
        raise RuntimeError('Requires already running ground runtime; no startup performed')
    model = resolve_active_model(runtime_state=runtime)
    owns_ros = not rclpy.ok()
    if owns_ros:
        rclpy.init()
    node = None
    try:
        node = ApiNode(model['joint_names'])
        node.enable_estimator_feedback()
        started = time.monotonic()
        summary = module.TelemetrySummary(model['joint_names'], started)
        deadline = started+timeout_wall_s
        result = None
        reason = 'timeout'
        while time.monotonic() < deadline:
            node.spin_once(timeout_sec=min(.01, max(0., deadline-time.monotonic())))
            now = time.monotonic()
            if not summary.observe(node, now):
                reason = 'latched_observation_failure'
                break
            result = ready_snapshot(summary, now, minimum_wall_s)
            if result is not None:
                reason = 'fresh_complete_ground_snapshot'
                break
        if result is None:
            result = summary.report(time.monotonic())
            result['passed'] = False
            result['failure'] = result.get('failure') or 'No complete ground snapshot with land age <= 1 s before timeout'
        result.update(kind='unarmed_background_readiness_v1', exit_reason=reason,
            minimum_wall_s=minimum_wall_s, timeout_wall_s=timeout_wall_s,
            land_max_final_wall_age_s=1., original_stream_max_final_wall_age_s=4.,
            model=model, runtime_pid=runtime.get('pid'), runtime_meta=runtime.get('runtime_meta'),
            source_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
            reused_probe={'path':str(reference), 'sha256':hashlib.sha256(reference.read_bytes()).hexdigest()},
            scope='connectivity/background-readiness diagnostic only; no flight acceptance; original predicates unchanged')
        clock = result.get('topics', {}).get('clock', {})
        if clock.get('first_received_sim_s') is not None and clock.get('received_sim_s') is not None:
            wall = clock['received_wall_s']-clock['first_seen_wall_s']
            result['observed_sim_over_wall'] = ((clock['received_sim_s']-clock['first_received_sim_s'])/wall if wall>0 else None)
        return result
    finally:
        if node is not None:
            # ApiNode.close() publishes zero_all. Preserve the original probe's
            # command-free cleanup instead.
            node._executor.shutdown()
            node.destroy_node()
        if owns_ros and rclpy.ok():
            rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo-root', type=Path, required=True)
    parser.add_argument('--minimum-wall-s', type=float, required=True)
    parser.add_argument('--timeout-wall-s', type=float, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.output.exists():
        parser.error('Output already exists; preserve previous observations')
    result = probe(args.repo_root.resolve(), args.minimum_wall_s, args.timeout_wall_s)
    text = json.dumps(result, indent=2, allow_nan=False)+'\n'
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open('x', encoding='utf-8') as stream:
        stream.write(text)
    print(text, end='')
    return 0 if result['passed'] else 2


if __name__ == '__main__':
    raise SystemExit(main())
