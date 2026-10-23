#!/usr/bin/env python3
"""Exercise the opt-in ROS 2 process boundaries with a synthetic hold fixture.

This never starts PX4/Gazebo or publishes actuator topics.  It is neither a
flight certificate nor a native-MPC/real-time qualification.  Run only after
sourcing the existing ROS 2, px4_msgs and new workspace installation.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import traceback


COMMAND_FIELDS = ("position_reference_enu", "velocity_feedforward_enu",
                  "acceleration_feedforward_enu", "base_velocity_target", "joint_velocity")
ACTIVE_OUTPUT = None


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def write_json(path, value):
    with Path(path).open("x", encoding="utf-8") as stream:
        json.dump(value, stream, indent=2, allow_nan=False)
        stream.write("\n")


def make_snapshot(cls, *, sequence, session, config_sha, target, gains):
    msg = cls()
    msg.schema_version, msg.session_id, msg.epoch, msg.sequence = 1, session, 1, sequence
    msg.replay_only = True
    msg.configuration_sha256 = list(bytes.fromhex(config_sha))
    t = sequence * .02
    msg.control_time_s, msg.task_time_s, msg.dt_s = 10. + t, t, .02
    msg.state_time_s = msg.joint_time_s = msg.control_time_s
    msg.position_enu = [0., 0., 2.]
    msg.rotation_row_major = [1., 0., 0., 0., 1., 0., 0., 0., 1.]
    for name, size in (("velocity_enu", 3), ("omega_body_flu", 3), ("acceleration_enu", 3),
                       ("q", 2), ("dq", 2), ("target_velocity_enu", 3),
                       ("target_acceleration_enu", 3), ("reference_r", 2)):
        setattr(msg, name, [0.] * size)
    msg.target_position_enu, msg.position_gains = list(target), list(gains)
    msg.reference_time_s, msg.bus_origin_s = t, 0.
    msg.command_count = 1
    values = [0.] * 64
    values[0] = t - .02
    msg.command_time_s = values
    msg.command_velocity = [0.] * 128
    # Include the active delayed MIT packet and the pending latest send.  The
    # fixture uses exact zero references, not postflight-derived command data.
    ticks = list(range(math.floor((t - .02) / .005 + 1e-9), math.floor(t / .005 + 1e-9) + 1))
    msg.packet_count = len(ticks)
    times, kp, kd = [0.] * 64, [0.] * 128, [0.] * 128
    for index, tick in enumerate(ticks):
        times[index] = tick * .005
        kp[2 * index:2 * index + 2] = [6., 6.]
        kd[2 * index:2 * index + 2] = [.25, .25]
    msg.packet_send_time_s = times
    msg.packet_kp, msg.packet_kd = kp, kd
    for name in ("packet_q_ref", "packet_dq_ref", "packet_tau_ff"):
        setattr(msg, name, [0.] * 128)
    msg.issued_monotonic_ns = time.monotonic_ns()
    return msg


def values(msg):
    return {name: [float(x) for x in getattr(msg, name)] for name in COMMAND_FIELDS}


def stop_owned_process(proc):
    """Bounded cleanup of one process group, including already-exited races."""
    errors = []
    for sig, timeout in ((signal.SIGINT, 8.), (signal.SIGTERM, 5.), (signal.SIGKILL, 3.)):
        if proc.poll() is not None:
            return errors
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            pass
        except Exception as error:
            errors.append(repr(error))
        try:
            proc.wait(timeout=timeout)
            return errors
        except subprocess.TimeoutExpired:
            continue
        except Exception as error:
            errors.append(repr(error))
    if proc.poll() is None:
        errors.append("Owned process group did not exit after SIGKILL")
    return errors


def owned_executables(processes):
    groups = {proc.pid: name for name, proc in processes.items()}
    result = []
    for path in Path("/proc").iterdir():
        if not path.name.isdigit():
            continue
        try:
            # Fields after the parenthesized comm start at stat field 3.
            fields = (path / "stat").read_text().rsplit(") ", 1)[1].split()
            group = int(fields[2])
            if group not in groups:
                continue
            executable = (path / "exe").resolve(strict=True)
            result.append({"pid": int(path.name), "group": group, "owner": groups[group],
                           "executable": str(executable), "sha256": sha(executable)})
        except (OSError, IndexError, ValueError):
            continue
    return result


def main(argv=None):
    global ACTIVE_OUTPUT
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--task-config", type=Path, required=True)
    parser.add_argument("--task-config-sha256", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--requests", type=int, default=5)
    parser.add_argument("--domain-id", type=int, default=197)
    parser.add_argument("--case", choices=("healthy", "worker_timeout"), default="healthy")
    args = parser.parse_args(argv)
    if not 1 <= args.requests <= 50 or not 1 <= args.domain_id <= 232:
        raise ValueError("Bounded request count and explicit private ROS domain required")
    if sha(args.task_config) != args.task_config_sha256:
        raise ValueError("Caller-pinned task configuration changed")
    args.output.mkdir(parents=True, exist_ok=False)
    out, root = args.output.resolve(), args.repo_root.resolve()
    ACTIVE_OUTPUT = out
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    for name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
        os.environ[name] = "1"
    # Import ROS only after choosing the isolated domain.  No global environment
    # or existing workspace installation is edited by this test.
    import numpy as np
    import rclpy
    from rclpy.serialization import deserialize_message
    from ament_index_python.packages import get_package_prefix
    from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal, GateEvent, DrainRequest, RecordingReceipt
    from flyarm_mpc_reference.factory import create_engine
    sys.path.insert(0, str(root / "control_packages/x650_wholebody_v1"))
    from x650_wholebody.model import ArmModel, tip_world
    cfg = json.loads(args.task_config.read_text(encoding="utf-8"))
    model = ArmModel.from_report(cfg["preparation_report"])
    target = tip_world(np.array([0., 0., 2.]), np.eye(3), np.zeros(2), model).tolist()
    session = int.from_bytes(os.urandom(4), "little") & 0x7fffffff or 1
    init = {"schema_version": 1, "synthetic_software_fixture_only": True,
        "task_origin_sim_s": 10., "anchor_position_enu": [0., 0., 2.],
        "anchor_rotation_row_major": [1., 0., 0., 0., 1., 0., 0., 0., 1.],
        "anchor_q": [0., 0.], "target_position_enu": target,
        "published_prestream_history": [
            {"control_time_s": 9.98, "command": [0.] * 5},
            {"control_time_s": 10., "command": [0.] * 5}]}
    init_path = out / "initialization.json"
    write_json(init_path, init)
    engine = create_engine(root, args.task_config, args.task_config_sha256, init_path, sha(init_path), session, 1)
    # Actual PID readback is taken from the bound task, not hardcoded for the
    # synthetic fixture.  It is also checked by the worker/gateway.
    sys.path.insert(0, str(root))
    from simulations.x650_ts20_hold_v1.task import frozen_research_pid
    _, pid, _ = frozen_research_pid(cfg)
    gains = [pid["MPC_XY_P"], pid["MPC_XY_P"], pid["MPC_Z_P"]]
    count = args.requests if args.case == "healthy" else 1
    baseline = []
    for sequence in range(1, count + 1):
        request = make_snapshot(ControlSnapshot, sequence=sequence, session=session,
            config_sha=args.task_config_sha256, target=target, gains=gains)
        proposal = engine.process(request, ControlProposal)
        if proposal.status != 0:
            raise RuntimeError("Direct reference rejected the fixture before transport: " + str(proposal.status))
        baseline.append(values(proposal))
    write_json(out / "direct_reference.json", {"synthetic_software_fixture_only": True, "commands": baseline})
    prefix = "/flyarm/offline/v1"
    processes, streams = {}, []
    node = None
    report = {"kind": "x650_ros2_structure_smoke_v1", "complete": False, "passed": False,
        "case": args.case, "synthetic_software_fixture_only": True, "native_mpc": False,
        "flight_started": False, "actuator_publishers_present": None, "hard_realtime_qualified": False,
        "task_configuration_sha256": args.task_config_sha256,
        "session_id": session, "epoch": 1, "domain_id": args.domain_id,
        "controller_process_separate_from_recorder": True, "sources": {}}
    for path in sorted((root / "ros2_ws/src").rglob("*")):
        if path.is_file() and "__pycache__" not in path.parts and not path.suffix == ".pyc":
            report["sources"][path.relative_to(root).as_posix()] = sha(path)
    report["sources"][Path(__file__).resolve().relative_to(root).as_posix()] = sha(__file__)
    installed_gateway = Path(get_package_prefix("flyarm_control_gateway")) / "lib/flyarm_control_gateway/shadow_gateway"
    report["installed_gateway"] = {"path": str(installed_gateway.resolve(strict=True)), "sha256": sha(installed_gateway)}

    def launch(name, package, executable, params):
        stream = (out / (name + ".log")).open("x", encoding="utf-8")
        streams.append(stream)
        command = ["ros2", "run", package, executable, "--ros-args"]
        for key, value in params.items():
            command += ["-p", key + ":=" + str(value)]
        proc = subprocess.Popen(command, stdout=stream, stderr=subprocess.STDOUT,
                                cwd=root, env=os.environ.copy(), start_new_session=True)
        processes[name] = proc
        return proc

    try:
        rclpy.init()
        node = rclpy.create_node("flyarm_structure_smoke")
        proposals, events, receipts, shadow, sent_requests = {}, {}, [], {}, {}
        subscriptions = [
            node.create_subscription(ControlProposal, prefix + "/output/proposal", lambda m: proposals.setdefault(int(m.sequence), m), 100),
            node.create_subscription(GateEvent, prefix + "/output/gate_event", lambda m: events.setdefault(int(m.sequence), m), 100),
            node.create_subscription(RecordingReceipt, prefix + "/recorder/output/receipt", receipts.append, 100),
            node.create_subscription(ControlProposal, prefix + "/output/shadow_command", lambda m: shadow.setdefault(int(m.sequence), m), 100)]
        publisher = node.create_publisher(ControlSnapshot, prefix + "/input/snapshot", 100)
        drain = node.create_publisher(DrainRequest, prefix + "/recorder/input/drain", 10)
        common = {"expected_session_id": session, "expected_epoch": 1, "prefix": prefix}
        launch("recorder", "flyarm_control_recorder", "recorder_node", {
            **common, "output_dir": out / "recording", "capacity": 8192})
        launch("gateway", "flyarm_control_gateway", "shadow_gateway", {
            **common, "configuration_sha256": args.task_config_sha256, "position_gains": str(gains)})
        launch("reference", "flyarm_mpc_reference", "reference_node", {
            **common, "repo_root": root, "task_config": args.task_config.resolve(),
            "task_config_sha256": args.task_config_sha256,
            "initialization_manifest": init_path, "initialization_sha256": sha(init_path),
            "startup_metadata_path": out / "reference_startup.json"})

        def wait_for(predicate, timeout, description, *, allow_dead_reference=False):
            end = time.monotonic() + timeout
            while not predicate():
                for name, proc in processes.items():
                    if proc.poll() is not None and not (allow_dead_reference and name == "reference"):
                        raise RuntimeError(name + " exited; inspect its log")
                if time.monotonic() >= end:
                    raise TimeoutError(description)
                rclpy.spin_once(node, timeout_sec=.005)

        wait_for(lambda: publisher.get_subscription_count() >= 2
            and node.count_subscribers(prefix + "/admitted/snapshot") >= 1
            and node.count_publishers(prefix + "/output/proposal") >= 1
            and node.count_publishers(prefix + "/output/gate_event") >= 1
            and node.count_publishers(prefix + "/recorder/output/receipt") >= 1,
            45., "Separate processes did not become ready")
        report["actual_process_executables"] = owned_executables(processes)
        report["observed_topics"] = {name: types for name, types in node.get_topic_names_and_types()}
        actuator_topics = [name for name in report["observed_topics"]
                           if (name.startswith("/fmu/in/") or name.startswith("/arm/"))
                           and node.count_publishers(name) > 0]
        report["actuator_publishers_present"] = bool(actuator_topics)
        if actuator_topics:
            raise RuntimeError("Actuator publisher appeared in the isolated offline domain")
        if not any(item["owner"] == "gateway" and item["executable"] == report["installed_gateway"]["path"]
                   and item["sha256"] == report["installed_gateway"]["sha256"] for item in report["actual_process_executables"]):
            raise RuntimeError("The actually running gateway ELF differs from the selected installation")
        startup_path = out / "reference_startup.json"
        if not startup_path.is_file():
            raise RuntimeError("Worker did not save its actual loaded-source identity")
        report["reference_startup"] = {"path": str(startup_path), "sha256": sha(startup_path)}
        startup = json.loads(startup_path.read_text())
        if not startup.get("complete") or not startup.get("created_before_ros_publishers_and_subscriptions"):
            raise RuntimeError("Worker startup provenance is incomplete")
        if not any(item["owner"] == "reference" and item["pid"] == startup["pid"]
                   and item["group"] == startup["pgid"] for item in report["actual_process_executables"]):
            raise RuntimeError("Startup source receipt does not belong to the actual worker process")
        for name, item in startup["loaded_module_sources"].items():
            if sha(item["path"]) != item["sha256"]:
                raise RuntimeError("Loaded worker source changed: " + name)
            if name.startswith("flyarm_mpc_reference."):
                expected_source = root / "ros2_ws/src/flyarm_mpc_reference" / Path(*name.split(".")).with_suffix(".py")
                if sha(expected_source) != item["sha256"]:
                    raise RuntimeError("Installed worker is stale: " + name)
        report["actual_worker_sources"] = startup["loaded_module_sources"]
        report["actually_loaded_native_library"] = startup["actually_loaded_native_library"]
        if args.case == "worker_timeout":
            os.killpg(processes["reference"].pid, signal.SIGINT)
            processes["reference"].wait(timeout=10.)
        for sequence in range(1, count + 1):
            request = make_snapshot(ControlSnapshot, sequence=sequence, session=session,
                config_sha=args.task_config_sha256, target=target, gains=gains)
            sent_requests[sequence] = request
            publisher.publish(request)
            wait_for(lambda: sequence in events, 5., "No gateway decision", allow_dead_reference=args.case == "worker_timeout")
            event = events[sequence]
            if args.case == "worker_timeout":
                if event.accepted or shadow:
                    raise RuntimeError("A dead worker resulted in an accepted shadow command")
            else:
                if not event.accepted:
                    raise RuntimeError("Gateway rejected: reason=" + str(event.reason))
                wait_for(lambda: sequence in proposals and sequence in shadow, 3., "Missing proposal/shadow output")
                actual = values(proposals[sequence])
                for key in COMMAND_FIELDS:
                    if not np.allclose(actual[key], baseline[sequence - 1][key], rtol=0., atol=1e-10):
                        raise RuntimeError("Direct/reference-node command mismatch: " + key)
                    if not np.array_equal(actual[key], getattr(shadow[sequence], key)):
                        raise RuntimeError("Gateway changed the accepted proposal: " + key)
        request = DrainRequest()
        request.schema_version, request.session_id, request.epoch, request.replay_only = 1, session, 1, True
        request.final_sequences = [count, count if args.case == "healthy" else 0, count]
        drain.publish(request)
        wait_for(lambda: any(m.stream == 255 for m in receipts), 15., "No final persistence receipt",
                 allow_dead_reference=args.case == "worker_timeout")
        final = next(m for m in receipts if m.stream == 255)
        if not final.persisted or not final.complete or final.failed:
            raise RuntimeError("Recorder did not certify complete persistence")
        manifests = list((out / "recording").glob("*manifest*.json"))
        if len(manifests) != 1 or bytes(final.sha256).hex() != sha(manifests[0]):
            raise RuntimeError("Final receipt is not bound to the actual closed manifest")
        sys.path.insert(0, str(root / "scripts"))
        from verify_x650_ros2_recording import verify
        expected = {(0, seq): msg for seq, msg in sent_requests.items()}
        expected.update({(1, seq): msg for seq, msg in proposals.items()})
        expected.update({(2, seq): msg for seq, msg in events.items()})
        classes = (ControlSnapshot, ControlProposal, GateEvent)
        recording_verification = verify(out / "recording",
            deserialize=lambda payload, stream: deserialize_message(payload, classes[stream]),
            expected_messages=expected)
        report["recording_verification"] = recording_verification
        actual_receipts = {(int(m.stream), int(m.sequence)): m for m in receipts}
        for expected_receipt in recording_verification["row_receipts"]:
            received = actual_receipts.get((expected_receipt["stream"], expected_receipt["sequence"]))
            if received is None or not received.persisted or received.failed or bytes(received.sha256).hex() != expected_receipt["sha256"]:
                raise RuntimeError("A durable row receipt was missing or did not bind its exact line")
        write_json(out / "received_receipts.json", [{"stream": int(m.stream), "sequence": int(m.sequence),
            "persisted": bool(m.persisted), "complete": bool(m.complete), "failed": bool(m.failed),
            "sha256": bytes(m.sha256).hex()} for m in receipts])
        report.update(complete=True, passed=True, expected_stream_counts=list(request.final_sequences),
            accepted_shadow_commands=len(shadow), proposals=len(proposals), gate_decisions=len(events),
            decision_roundtrip_ns=[int(m.roundtrip_ns) for m in events.values()],
            recorder_manifest={"path": str(manifests[0]), "sha256": sha(manifests[0])},
            command_equivalence_scope="same Python core direct vs ROS node; not native port equivalence",
            source_contract="synthetic fixed-state hold; not replay of historical read availability",
            actual_ros_messages="installed generated types; separate OS processes; local isolated DDS domain")
    except BaseException as error:
        report.update(error=repr(error), traceback=traceback.format_exc())
    finally:
        cleanup_errors = []
        for name, proc in reversed(list(processes.items())):
            try:
                cleanup_errors.extend(name + ": " + text for text in stop_owned_process(proc))
                report.setdefault("process_exit_codes", {})[name] = proc.returncode
            except BaseException as error:
                cleanup_errors.append(name + ": " + repr(error))
        if node is not None:
            try:
                node.destroy_node()
            except BaseException as error:
                cleanup_errors.append("node: " + repr(error))
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except BaseException as error:
            cleanup_errors.append("ROS context: " + repr(error))
        for stream in streams:
            try:
                stream.close()
            except BaseException as error:
                cleanup_errors.append("log: " + repr(error))
        try:
            if sha(installed_gateway) != report["installed_gateway"]["sha256"]:
                cleanup_errors.append("Installed gateway changed during the test")
            for name, item in report.get("actual_worker_sources", {}).items():
                if sha(item["path"]) != item["sha256"]:
                    cleanup_errors.append("Worker source changed: " + name)
        except Exception as error:
            cleanup_errors.append("Source recheck: " + repr(error))
        report["cleanup_errors"] = cleanup_errors
        if cleanup_errors:
            report["passed"], report["complete"] = False, False
        write_json(out / "structure_report.json", report)
    print(json.dumps({key: report.get(key) for key in ("kind", "case", "complete", "passed", "error", "accepted_shadow_commands", "decision_roundtrip_ns")}, indent=2))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        if ACTIVE_OUTPUT is not None and not (ACTIVE_OUTPUT / "structure_report.json").exists():
            write_json(ACTIVE_OUTPUT / "structure_report.json", {
                "kind": "x650_ros2_structure_smoke_v1", "complete": False, "passed": False,
                "flight_started": False, "native_mpc": False, "error": repr(error),
                "traceback": traceback.format_exc(), "stage": "pre-transport setup"})
        raise
