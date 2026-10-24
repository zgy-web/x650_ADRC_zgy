#!/usr/bin/env python3
"""Private-DDS software probe of callback iteration and real FaultSupervisor.

No vehicle, MAVLink socket, Gazebo, controller or actuator is constructed.
VehicleAPI._pump is called unbound with a no-message fake MAVLink link and a
real ROS executor. A declared sleeping callback cost keeps this a low-CPU
adversarial scheduling fixture, not a physical timing qualification.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
import traceback
import uuid

ROOT = Path(__file__).resolve().parents[1]


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--domain-id', type=int, required=True)
    parser.add_argument('--steps', type=int, default=160)
    parser.add_argument('--repo-root', type=Path, default=ROOT)
    parser.add_argument('--vehicle-source', type=Path, default=ROOT / 'src/amsim/api/vehicle.py')
    parser.add_argument('--supervision-source', type=Path, default=ROOT / 'simulations/x650_isolated_live_v1/supervision.py')
    args = parser.parse_args()
    require(sys.platform.startswith('linux'), 'A Linux sourced ROS workspace is required')
    require(1 <= args.domain_id <= 232, 'An explicit private nondefault DDS domain is required')
    require(120 <= args.steps <= 400, 'Use 120..400 bounded fixture steps')
    args.output = args.output.resolve()
    args.output.mkdir(parents=True, exist_ok=False)
    repo = args.repo_root.resolve(strict=True)
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY='1')
    sys.path[:0] = [str(repo), str(repo / 'src')]
    report = {'kind': 'ros_callback_fairness_synthetic_probe_v1', 'passed': False,
        'synthetic_software_fixture_only': True, 'physical_validation': False,
        'flight_accepted': False, 'eligible_as_stage_gate': False,
        'domain_id': args.domain_id, 'steps': args.steps,
        'callback_service_sleep_s': .00035, 'drain_budget_ns': 250_000,
        'publish_delivery_sleep_s': .0005, 'cases': [], 'source_hashes': {}}
    initialized = False
    publisher_node = None
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import UInt64MultiArray
        from flyarm_control_interfaces import msg
        def load_source(name, path):
            specification = importlib.util.spec_from_file_location(name, path.resolve(strict=True))
            module = importlib.util.module_from_spec(specification)
            specification.loader.exec_module(module)
            return module
        # Explicit staged copies permit reviewing proposed source without
        # changing any production module on the server. Relative imports in
        # VehicleAPI still resolve against the installed project package.
        VehicleAPI = load_source('amsim.api._fairness_vehicle', args.vehicle_source).VehicleAPI
        FaultSupervisor = load_source('_fairness_supervision', args.supervision_source).FaultSupervisor
        import rclpy.executors as executors
        report['source_hashes'] = {str(p): sha(p) for p in [
            Path(__file__), Path(executors.__file__), args.vehicle_source.resolve(),
            repo / 'src/amsim/api/node.py', args.supervision_source.resolve()]}
        rclpy.init()
        initialized = True
        publisher_node = Node('fairness_fixture_publisher_' + uuid.uuid4().hex[:8])
        time.sleep(.25)
        require(len(publisher_node.get_node_names()) == 1, 'Private DDS domain is not empty')
        qos_latest = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        qos_evidence = QoSProfile(depth=4000, reliability=ReliabilityPolicy.BEST_EFFORT)

        class TracedExecutor(SingleThreadedExecutor):
            def __init__(self):
                super().__init__()
                self.generator_creations = 0

            def _wait_for_ready_callbacks(self, *values, **keywords):
                self.generator_creations += 1
                yield from super()._wait_for_ready_callbacks(*values, **keywords)

        class EmptyLink:
            @staticmethod
            def recv_match(**kwargs):
                return None

        class ExecutorNodeView:
            def __init__(self, executor):
                self.executor = executor

            def spin_once(self, timeout_sec=.02):
                self.executor.spin_once(timeout_sec=timeout_sec)

        class VehicleFixture:
            def __init__(self, executor):
                self.node = ExecutorNodeView(executor)
                self.link = EmptyLink()
                self.target = None
                self.heartbeat_time = 0.

        for case_name in ('mixed_timeout', 'uniform_zero', 'independent_supervisor'):
            case = {'name': case_name, 'callback_columns': ['phase', 'topic', 'sequence',
                'producer_ns', 'callback_entry_ns', 'callback_exit_ns', 'fault'],
                'callbacks': [], 'pump_columns': ['step', 'entry_ns', 'first_return_ns',
                'drain_return_ns', 'drain_calls', 'supervisor_error'], 'pumps': [],
                'published_columns': ['step', 'producer_ns', 'status'], 'published': []}
            report['cases'].append(case)
            executor = TracedExecutor()
            node = Node('fairness_' + case_name + '_' + uuid.uuid4().hex[:8])
            executor.add_node(node)
            vehicle = VehicleFixture(executor)
            supervisor = None
            publishers = []
            subscriptions = []
            session = int.from_bytes(os.urandom(4), 'little') & 0x3fffffff or 1
            phase = 'active'
            first_fault = None
            prefix = '/synthetic_fairness/case_' + uuid.uuid4().hex

            def record(topic, sequence, producer, fault):
                nonlocal first_fault
                begin = time.monotonic_ns()
                if topic < 6:
                    time.sleep(report['callback_service_sleep_s'])
                case['callbacks'].append([phase, topic, int(sequence), int(producer),
                    begin, time.monotonic_ns(), bool(fault)])
                if fault and first_fault is None and phase == 'active':
                    first_fault = begin

            try:
                for index in range(6):
                    topic = prefix + '/sensor_' + str(index)
                    publishers.append(publisher_node.create_publisher(UInt64MultiArray, topic, qos_latest))
                    subscriptions.append(node.create_subscription(UInt64MultiArray, topic,
                        lambda value, index=index: record(index, value.data[0], value.data[1], False), qos_latest))
                dispatch_pub = publisher_node.create_publisher(msg.DispatchTick, '/flyarm/isolated/v1/dispatch', qos_latest)
                event_pub = publisher_node.create_publisher(msg.GateEvent, '/flyarm/isolated/v1/event', qos_latest)
                publishers += [dispatch_pub, event_pub]
                subscriptions += [node.create_subscription(msg.DispatchTick, '/flyarm/isolated/v1/dispatch',
                    lambda value: record(6, value.sequence, value.actual_monotonic_ns, value.status == 3)
                        if value.session_id == session else None, qos_evidence),
                    node.create_subscription(msg.GateEvent, '/flyarm/isolated/v1/event',
                    lambda value: record(7, value.sequence, value.received_monotonic_ns, not value.accepted)
                        if value.session_id == session else None, qos_evidence)]
                if case_name == 'independent_supervisor':
                    supervisor = FaultSupervisor(publisher_node.context, msg, session, 1)
                deadline = time.monotonic() + 8.
                expected_fault_subscribers = 2 if supervisor is not None else 1
                while (any(p.get_subscription_count() != 1 for p in publishers[:6])
                       or any(p.get_subscription_count() != expected_fault_subscribers for p in publishers[6:])):
                    require(time.monotonic() < deadline, 'DDS discovery timeout')
                    executor.spin_once(timeout_sec=0.)
                    if supervisor is not None:
                        supervisor.poll()
                    time.sleep(.005)
                generator_initial = executor.generator_creations
                first_fault_issued = None
                for step in range(args.steps):
                    now = time.monotonic_ns()
                    value = UInt64MultiArray()
                    value.data = [step, now]
                    for pub in publishers[:6]:
                        pub.publish(value)
                    # A later nonfault status tests that the real supervisor's
                    # first fault survives later STOPPED/WAITING observations.
                    status = 0 if step < 30 else 3 if step < 100 else 4
                    if step == 30:
                        first_fault_issued = now
                    dispatch = msg.DispatchTick()
                    dispatch.session_id = session; dispatch.epoch = 1; dispatch.sequence = step + 1
                    dispatch.actual_monotonic_ns = now; dispatch.status = status
                    dispatch.reason = 1 if status == 3 else 0
                    dispatch_pub.publish(dispatch)
                    event = msg.GateEvent()
                    event.session_id = session; event.epoch = 1; event.sequence = step + 1
                    event.accepted = True; event.received_monotonic_ns = now
                    event_pub.publish(event)
                    case['published'].append([step, now, status])
                    time.sleep(report['publish_delivery_sleep_s'])
                    begin = time.monotonic_ns()
                    if case_name == 'uniform_zero':
                        VehicleAPI._pump(vehicle, ros_timeout_s=0.)
                    else:
                        VehicleAPI._pump(vehicle)  # actual default .02
                    first_return = time.monotonic_ns()
                    until = first_return + report['drain_budget_ns']
                    calls = 0
                    while calls < 32 and time.monotonic_ns() < until:
                        vehicle.node.spin_once(timeout_sec=0.)
                        calls += 1
                    drained = time.monotonic_ns()
                    error = supervisor.poll() if supervisor is not None else None
                    case['pumps'].append([step, begin, first_return, drained, calls, error])
                case['generator_creations_active'] = executor.generator_creations - generator_initial
                case['main_first_fault_observed_ns'] = first_fault
                case['first_fault_issued_ns'] = first_fault_issued
                active = [row for row in case['callbacks'] if row[0] == 'active']
                case['active_callback_counts'] = [sum(row[1] == index for row in active) for index in range(8)]
                if supervisor is not None:
                    case['supervisor'] = supervisor.latch.report()
                phase = 'after_publishing_stopped'
                drain_deadline = time.monotonic() + .08
                while time.monotonic() < drain_deadline:
                    executor.spin_once(timeout_sec=0.)
                case['queued_fault_seen_after_drain'] = any(row[0] != 'active' and row[6] for row in case['callbacks'])
            finally:
                if supervisor is not None:
                    supervisor.close()
                executor.remove_node(node)
                executor.shutdown(timeout_sec=0.)
                node.destroy_node()
                for pub in publishers:
                    publisher_node.destroy_publisher(pub)
            (args.output / (case_name + '.json')).write_text(json.dumps(case, indent=2) + '\n')

        mixed, uniform, independent = report['cases']
        uniform_latency = (None if uniform['main_first_fault_observed_ns'] is None else
            (uniform['main_first_fault_observed_ns'] - uniform['first_fault_issued_ns']) / 1e6)
        first = independent['supervisor']['first_fault']
        independent_latency = (None if first is None else
            (first['observed_monotonic_ns'] - independent['first_fault_issued_ns']) / 1e6)
        report['summary'] = {'callback_counts': {c['name']: c['active_callback_counts'] for c in report['cases']},
            'generator_creations': {c['name']: c['generator_creations_active'] for c in report['cases']},
            'uniform_first_fault_latency_ms': uniform_latency,
            'uniform_zero_with_full_evidence_queue_is_not_a_safety_latency_guarantee': True,
            'uniform_full_queue_under_25ms_observed': uniform_latency is not None and 0 <= uniform_latency < 25.,
            'independent_first_fault_latency_ms': independent_latency,
            'mixed_fault_observed_active': mixed['main_first_fault_observed_ns'] is not None,
            'mixed_queued_fault_confirmed_after_drain': mixed['queued_fault_seen_after_drain']}
        require(all(uniform['active_callback_counts']), 'Uniform timeout did not service every ready topic')
        # Uniform wait arguments restore iterator progress, but a deep history
        # queue can still delay a newly issued fault. Preserve that measured
        # latency; do not promote fairness into a safety-delivery guarantee.
        require(uniform_latency is not None and uniform_latency >= 0., 'Uniform callback progress did not reach the queued fault')
        require(independent_latency is not None and 0 <= independent_latency < 25., 'Independent fault observation exceeded fixture bound')
        require(first['reason'] == 1 and first['topic'] == 'dispatch', 'Real supervisor lost initial fault identity')
        require(mixed['generator_creations_active'] > uniform['generator_creations_active'] * 1.5,
            'Mixed timeout generator reset effect was not reproduced')
        require(mixed['active_callback_counts'][6] < uniform['active_callback_counts'][6],
            'Late-subscription starvation differential was not reproduced')
        require(all(sha(path) == digest for path, digest in report['source_hashes'].items()), 'Reviewed source changed during probe')
        report['passed'] = True
    except BaseException as error:
        report.update(error=repr(error), traceback=traceback.format_exc())
    finally:
        if publisher_node is not None:
            publisher_node.destroy_node()
        if initialized and rclpy.ok():
            rclpy.shutdown()
    (args.output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False) + '\n')
    print(json.dumps({key: report.get(key) for key in ('passed', 'summary', 'error')}, indent=2))
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
