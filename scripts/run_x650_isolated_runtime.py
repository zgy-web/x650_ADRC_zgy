#!/usr/bin/env python3
"""Sustained ROS 2 isolation test with the actual frozen MPC numerical core.

The plant is NOT started. Five owned processes separate a software state source,
C++ reception, frozen Python/C++ algorithm, C++ periodic output, and recording.
No physical precision or flight certificate can be issued by this program.
"""
import argparse
import base64
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import traceback
from run_x650_ros2_structure_smoke import sha, write_json, stop_owned_process, owned_executables, values


def stats(values):
    if not values:
        return {"count": 0}
    import numpy as np
    return {"count": len(values), "min": float(min(values)), "median": float(np.median(values)),
            "p99": float(np.percentile(values, 99)), "max": float(max(values))}


def reconcile_recording(out):
    sender = json.loads((out / "diagnostic_sender.json").read_text())
    sink = json.loads((out / "recording/manifest.json").read_text())
    complete = (sender["dropped"] == 0 and sender["attempts"] == sender["sent"] == sink["rows"]
                and sink["last_sequence"] == sender["attempts"] and not sink["gaps"]
                and sink["drained_and_fsynced"] and sink["error"] is None
                and sha(out / "recording/diagnostics.jsonl") == sink["records_sha256"])
    return {"complete": complete, "attempts": sender["attempts"], "sent": sender["sent"],
            "dropped": sender["dropped"], "persisted": sink["rows"], "first_error": sender["first_error"],
            "offer_ms": stats([v/1e6 for v in sender["offer_ns"]]), "max_packet_bytes": sender["max_packet_bytes"],
            "records_sha256": sink["records_sha256"]}


def replay_recording(root, out, task, task_sha, init, session):
    from flyarm_mpc_reference.factory import create_engine
    from flyarm_control_interfaces.msg import ControlSnapshot, ControlProposal
    from rclpy.serialization import deserialize_message
    import numpy as np
    engine = create_engine(root, task, task_sha, init, sha(init), session, 1)
    pairs, max_error, rejected, compute = 0, 0., [], []
    sequences = []
    qp = []
    for line in (out / "recording/diagnostics.jsonl").open():
        row = json.loads(line)
        s = deserialize_message(base64.b64decode(row["snapshot_cdr"]["base64"]), ControlSnapshot)
        actual = deserialize_message(base64.b64decode(row["proposal_cdr"]["base64"]), ControlProposal)
        if actual.status != 0:
            rejected.append(int(actual.sequence))
            break
        # Preserve every numeric input/context/history and sequence. Only the
        # transport wall clock is rebased for the independent offline call.
        s.issued_monotonic_ns = time.monotonic_ns()
        direct = engine.process(s, ControlProposal)
        if direct.status != 0:
            raise RuntimeError("Direct replay rejected: " + str(engine.last_error))
        err = max(float(np.max(np.abs(np.asarray(values(actual)[k])-np.asarray(values(direct)[k])))) for k in values(actual))
        max_error = max(max_error, err)
        pairs += 1
        sequences.append(int(actual.sequence))
        compute.append((actual.compute_end_monotonic_ns-actual.compute_start_monotonic_ns)/1e6)
        diagnostic = row["diagnostic"] or {}
        if "solve_ms" in diagnostic:
            qp.append(float(diagnostic["solve_ms"]))
    return {"pairs": pairs, "sequences": sequences, "max_command_absolute_error": max_error, "rejected_recorded_sequences": rejected,
            "worker_compute_ms": stats(compute), "solver_ms": stats(qp),
            "same_numeric_snapshot_and_context": True, "transport_wall_clock_rebased": True}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--task-config", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--case", choices=["healthy", "worker_pause", "worker_short_pause", "source_pause", "recorder_pause"], required=True)
    parser.add_argument("--domain-id", type=int, default=201)
    parser.add_argument("--commissioning", action="store_true", help="3 s engineering check, never a full protocol pass")
    args = parser.parse_args(argv)
    if not 1 <= args.domain_id <= 232:
        raise ValueError("Private DDS domain required")
    root = Path(__file__).resolve().parents[1]
    protocol_path = root / "configs/experiments/x650_isolated_runtime_v1.json"
    protocol = json.loads(protocol_path.read_text())
    if sha(args.task_config) != protocol["task_sha256"]:
        raise ValueError("Frozen task changed")
    cfg = json.loads(args.task_config.read_text())
    if (cfg["numerical_backend"]["library_sha256"] != protocol["native_optimization_sha256"]
            or sha(cfg["numerical_backend"]["library"]) != protocol["native_optimization_sha256"]):
        raise ValueError("Frozen C++ library changed")
    args.output.mkdir(parents=True, exist_ok=False)
    out = args.output.resolve()
    os.environ.update(ROS_DOMAIN_ID=str(args.domain_id), ROS_LOCALHOST_ONLY="1",
                      OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1", MKL_NUM_THREADS="1")
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from ament_index_python.packages import get_package_prefix
    from flyarm_control_interfaces.msg import DispatchTick, IngressStatus, GateEvent, RuntimeStop
    import numpy as np
    sys.path.insert(0, str(root))
    sys.path.insert(0, str(root / "control_packages/x650_wholebody_v1"))
    from x650_wholebody.model import ArmModel, tip_world
    from x650_wholebody.native_qualification import verify_backend
    from simulations.x650_ts20_hold_v1.task import frozen_research_pid
    proof_before = verify_backend(root, cfg["numerical_backend"])
    model = ArmModel.from_report(cfg["preparation_report"])
    target = tip_world(np.array([0., 0., 2.]), np.eye(3), np.zeros(2), model).tolist()
    _, pid, _ = frozen_research_pid(cfg)
    gains = [pid["MPC_XY_P"], pid["MPC_XY_P"], pid["MPC_Z_P"]]
    session = int.from_bytes(os.urandom(4), "little") & 0x7fffffff or 1
    init = {"schema_version": 1, "synthetic_software_fixture_only": True,
        "task_origin_sim_s": 10., "anchor_position_enu": [0., 0., 2.],
        "anchor_rotation_row_major": [1., 0., 0., 0., 1., 0., 0., 0., 1.],
        "anchor_q": [0., 0.], "target_position_enu": target,
        "published_prestream_history": [{"control_time_s": 9.98, "command": [0.]*5},
                                       {"control_time_s": 10., "command": [0.]*5}]}
    init_path = out / "initialization.json"
    write_json(init_path, init)
    private = Path(tempfile.mkdtemp(prefix="farm-runtime-"))
    private.chmod(0o700)
    sock = private / "diag.sock"
    source_cfg = {"out": str(out), "start_file": str(out / "start"), "session": session,
                  "task_sha256": protocol["task_sha256"], "target": target, "gains": gains}
    write_json(out / "source_config.json", source_cfg)
    report = {"kind": "x650_isolated_runtime_v1", "case": args.case, "passed": False, "complete": False,
        "protocol_complete": False, "commissioning": args.commissioning, "flight_started": False,
        "physical_hold_qualified": False, "hard_realtime_qualified": False, "session_id": session,
        "frozen_task_sha256": sha(args.task_config), "protocol_sha256": sha(protocol_path),
        "native_core": proof_before, "publication_scope": "typed shadow output; not actuator application"}
    source_paths = list((root / "ros2_ws/src").rglob("*.py")) + list((root / "ros2_ws/src").rglob("*.cpp"))
    source_paths += list((root / "ros2_ws/src").rglob("*.hpp")) + list((root / "ros2_ws/src").rglob("*.msg"))
    source_paths += [Path(__file__), root / "scripts/x650_isolated_state_fixture.py", root / "scripts/x650_isolated_diagnostic_recorder.py", protocol_path]
    report["sources"] = {str(p.relative_to(root)): sha(p) for p in source_paths}
    procs, logs = {}, []
    node = None
    ticks, sources, events = [], [], []
    paused = None
    def launch(name, command):
        log = (out / (name + ".log")).open("x")
        logs.append(log)
        procs[name] = subprocess.Popen(command, cwd=root, stdin=subprocess.DEVNULL,
            stdout=log, stderr=subprocess.STDOUT, start_new_session=True, env=os.environ.copy())
    def ros_command(package, executable, params):
        binary = Path(get_package_prefix(package)) / "lib" / package / executable
        report.setdefault("executables", {})[executable] = {"path": str(binary), "sha256": sha(binary)}
        command = [str(binary), "--ros-args"]
        for key, value in params.items(): command += ["-p", str(key) + ":=" + str(value)]
        return command
    try:
        rclpy.init()
        node = rclpy.create_node("isolated_runtime_observer")
        qos = QoSProfile(depth=10000, reliability=ReliabilityPolicy.BEST_EFFORT)
        def tick(m):
            ticks.append({name: getattr(m, name) for name in ("sequence", "scheduled_monotonic_ns", "actual_monotonic_ns",
                "source_sequence", "source_received_monotonic_ns", "command_sequence", "command_issued_monotonic_ns",
                "command_age_ns", "status", "reason", "output_valid", "repeated")})
        def source(m):
            sources.append({name: getattr(m, name) for name in ("source_sequence", "received_monotonic_ns", "source_issued_monotonic_ns", "faulted")})
        def event(m):
            events.append({name: getattr(m, name) for name in ("sequence", "snapshot_sequence", "proposal_sequence", "accepted", "reason", "received_monotonic_ns", "roundtrip_ns")})
        subs = [node.create_subscription(DispatchTick, "/flyarm/isolated/v1/dispatch", tick, qos),
                node.create_subscription(IngressStatus, "/flyarm/isolated/v1/source_status", source, qos),
                node.create_subscription(GateEvent, "/flyarm/isolated/v1/event", event, qos)]
        stop = node.create_publisher(RuntimeStop, "/flyarm/isolated/v1/stop", 1)
        launch("recorder", [sys.executable, str(root / "scripts/x650_isolated_diagnostic_recorder.py"),
            "--socket", str(sock), "--output", str(out / "recording")])
        end = time.monotonic() + 10
        while not sock.exists():
            if time.monotonic() > end or procs["recorder"].poll() is not None: raise RuntimeError("Recorder startup failed")
            time.sleep(.01)
        common = {"expected_session_id": session, "expected_epoch": 1, "configuration_sha256": protocol["task_sha256"]}
        launch("ingress", ros_command("flyarm_control_gateway", "state_ingress", {**common, "position_gains": str(gains), "task_origin_s": 10.}))
        launch("dispatcher", ros_command("flyarm_control_gateway", "periodic_dispatcher", common))
        launch("worker", ros_command("flyarm_mpc_reference", "isolated_node", {
            "expected_session_id": session, "expected_epoch": 1, "repo_root": root,
            "task_config": args.task_config.resolve(), "task_config_sha256": protocol["task_sha256"],
            "initialization_manifest": init_path, "initialization_sha256": sha(init_path),
            "output_dir": out, "diagnostic_socket": sock}))
        launch("source", [sys.executable, str(root / "scripts/x650_isolated_state_fixture.py"), "--config", str(out / "source_config.json")])
        def spin_until(predicate, timeout):
            deadline = time.monotonic()+timeout
            while not predicate():
                for name, p in procs.items():
                    if p.poll() is not None: raise RuntimeError(name + " unexpectedly exited")
                if time.monotonic()>deadline: raise TimeoutError("Runtime readiness/stop")
                rclpy.spin_once(node, timeout_sec=.002)
        spin_until(lambda: (out/"worker_startup.json").exists() and (out/"source_ready").exists()
            and node.count_subscribers("/flyarm/isolated/v1/source")>=1
            and node.count_subscribers("/flyarm/isolated/v1/admitted/snapshot")>=1
            and node.count_subscribers("/flyarm/isolated/v1/accepted")>=1
            and node.count_subscribers("/flyarm/isolated/v1/source_status")>=2
            and stop.get_subscription_count()>=2 and len(ticks)>2, 45.)
        report["actual_processes"] = owned_executables(procs)
        topics = dict(node.get_topic_names_and_types())
        report["actuator_topics"] = [n for n in topics if n.startswith(("/fmu/in/", "/arm/"))]
        if report["actuator_topics"]: raise RuntimeError("Unexpected actuator topic")
        start = time.monotonic_ns()
        (out/"start").touch()
        duration = 3. if args.commissioning else protocol["healthy_duration_s" if args.case=="healthy" else "fault_duration_s"]
        injection = None
        while time.monotonic_ns()-start < duration*1e9:
            elapsed = (time.monotonic_ns()-start)/1e9
            if args.case!="healthy" and injection is None and elapsed>=protocol["injection_at_s"]:
                owner = "source" if args.case=="source_pause" else "recorder" if args.case=="recorder_pause" else "worker"
                pause_ms = protocol[args.case+"_ms"]
                os.killpg(procs[owner].pid, signal.SIGSTOP)
                injection = {"owner": owner, "stop_ns": time.monotonic_ns(), "requested_ms": pause_ms}
                paused = owner
            if paused and time.monotonic_ns()-injection["stop_ns"] >= injection["requested_ms"]*1e6:
                os.killpg(procs[paused].pid, signal.SIGCONT)
                injection["resume_ns"] = time.monotonic_ns()
                paused = None
            for name, p in procs.items():
                if p.poll() is not None: raise RuntimeError(name+" unexpectedly exited")
            rclpy.spin_once(node, timeout_sec=.001)
        report["window"] = {"start_ns": start, "end_ns": time.monotonic_ns(), "duration_s": duration}
        report["injection"] = injection
        m = RuntimeStop(); m.session_id=session; m.epoch=1
        report["stop_requested_ns"] = time.monotonic_ns()
        stop.publish(m)
        spin_until(lambda: ticks and ticks[-1]["status"]==4, 3.)
        # Let an already computed proposal be received/drained; stop is one-way
        # in both native processes, so it cannot renew the last command lease.
        until = time.monotonic()+.1
        while time.monotonic()<until: rclpy.spin_once(node, timeout_sec=.002)
        report["complete"] = True
    except BaseException as error:
        report.update(error=repr(error), traceback=traceback.format_exc())
    finally:
        if paused:
            os.killpg(procs[paused].pid, signal.SIGCONT)
        cleanup = []
        # Producer before consumer: recorder drains all queued messages before
        # its final fsync and count reconciliation.
        for name in ("source", "worker", "ingress", "dispatcher", "recorder"):
            if name in procs:
                cleanup += [name+": "+e for e in stop_owned_process(procs[name])]
        if node is not None: node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
        for log in logs: log.close()
        if sock.exists(): sock.unlink()
        private.rmdir()
        report["cleanup_errors"] = cleanup
        report["remaining_owned_processes"] = owned_executables(procs)
        write_json(out/"dispatch_ticks.json", ticks)
        write_json(out/"ingress_receipts.json", sources)
        write_json(out/"gate_events.json", events)
    try:
        report["native_core_unchanged"] = verify_backend(root, cfg["numerical_backend"]) == proof_before
        report["sources_unchanged"] = all(sha(root/p)==h for p,h in report["sources"].items())
        report["recording"] = reconcile_recording(out)
        # Replay requires an uninterrupted request sequence. Missing diagnostic
        # data is reported as evidence failure, never interpolated or fabricated.
        if report["recording"]["complete"]:
            report["equivalence"] = replay_recording(root, out, args.task_config, protocol["task_sha256"], init_path, session)
        startup = json.loads((out/"worker_startup.json").read_text())
        report["actually_loaded_native_optimization"] = startup["actually_loaded_native_optimization"]
        if startup["actually_loaded_native_optimization"]["sha256"] != protocol["native_optimization_sha256"]:
            raise RuntimeError("Worker loaded a different C++ core")
        window = report["window"]
        active = [t for t in ticks if window["start_ns"] <= t["actual_monotonic_ns"] < window["end_ns"]]
        valid = [t for t in active if t["output_valid"]]
        intervals = [(b["actual_monotonic_ns"]-a["actual_monotonic_ns"])/1e6 for a,b in zip(active,active[1:])]
        faults = [t for t in active if t["status"]==3]
        report["timing"] = {"dispatch_interval_ms": stats(intervals),
            "dispatch_lateness_ms": stats([(t["actual_monotonic_ns"]-t["scheduled_monotonic_ns"])/1e6 for t in active]),
            "valid_command_age_ms": stats([t["command_age_ns"]/1e6 for t in valid]),
            "valid_ticks": len(valid), "repeated_valid_ticks": sum(t["repeated"] for t in valid),
            "fault_ticks": len(faults), "first_fault": faults[0] if faults else None,
            "accepted_candidates": sum(e["accepted"] for e in events), "gate_rejections": [e for e in events if not e["accepted"]]}
        accepted_ids = {e["proposal_sequence"] for e in events if e["accepted"]}
        output_ids = {t["command_sequence"] for t in ticks if t["output_valid"]}
        report["output_receipts"] = {"accepted_count": len(accepted_ids), "unique_output_count": len(output_ids),
            "accepted_not_output": sorted(accepted_ids-output_ids), "unknown_output": sorted(output_ids-accepted_ids),
            "receipt_scope": "C++ shadow publication, not physical actuator consumption"}
        checks = {"complete_run": report["complete"], "clean_shutdown": not cleanup and not report["remaining_owned_processes"],
            "frozen_sources": report["sources_unchanged"] and report["native_core_unchanged"],
            "no_stale_output": bool(valid) and all(t["command_age_ns"]<=50_000_000 for t in valid),
            "periodic_dispatch": bool(intervals) and max(intervals)<=protocol["max_dispatch_interval_ms"],
            "no_output_after_fault": not faults or not any(t["output_valid"] and t["actual_monotonic_ns"]>=faults[0]["actual_monotonic_ns"] for t in active),
            "complete_tick_observation": all(b["sequence"]==a["sequence"]+1 for a,b in zip(active,active[1:]))}
        if args.case=="healthy":
            accepted_sequences = [e["proposal_sequence"] for e in events if e["accepted"]]
            recorded_sequences = report.get("equivalence",{}).get("sequences",[])
            unaccepted = sorted(set(recorded_sequences)-set(accepted_sequences))
            # Stop may win the race with one already in-flight worker result.
            # Keep and replay that row, but never call it an accepted/executed
            # command. An interior missing acceptance is still a test failure.
            report["recorded_but_not_accepted_at_stop"] = unaccepted
            not_output = sorted(accepted_ids-output_ids)
            checks.update(no_gate_rejection=not report["timing"]["gate_rejections"], no_output_fault=not faults,
                complete_recording=report["recording"]["complete"],
                direct_equivalence=report.get("equivalence",{}).get("pairs",0)>0 and
                  report.get("equivalence",{}).get("max_command_absolute_error",1)<=protocol["direct_command_abs_tolerance"],
                all_accepted_candidates_recorded=bool(accepted_sequences) and set(accepted_sequences).issubset(recorded_sequences),
                only_one_terminal_inflight=not unaccepted or unaccepted==[max(accepted_sequences,default=0)+1],
                no_silent_candidate_overwrite=not not_output or not_output==[max(accepted_ids)],
                only_accepted_output=not (output_ids-accepted_ids))
        else:
            inj = report["injection"]
            during = [t for t in active if inj["stop_ns"]<=t["actual_monotonic_ns"]<inj["resume_ns"]]
            report["injection"]["dispatch_ticks_during_pause"] = len(during)
            report["injection"]["state_receipts_during_pause"] = sum(inj["stop_ns"]<=s["received_monotonic_ns"]<inj["resume_ns"] for s in sources)
            checks["injection_observed"] = inj["resume_ns"]-inj["stop_ns"]>=inj["requested_ms"]*1e6
            if args.case in ("worker_pause", "source_pause"):
                checks["expiry_rejected"] = bool(faults) and faults[0]["actual_monotonic_ns"]>=inj["stop_ns"]
                checks["dispatcher_survived_pause"] = len(during)>=3
            if args.case=="worker_pause": checks["receiver_survived_pause"] = report["injection"]["state_receipts_during_pause"]>=10
            if args.case=="recorder_pause":
                checks.update(log_loss_visible=report["recording"]["dropped"]>0 and not report["recording"]["complete"],
                              control_independent=not faults and not report["timing"]["gate_rejections"])
        report["checks"] = checks
        report["passed"] = all(checks.values())
        report["protocol_complete"] = report["passed"] and not args.commissioning
    except BaseException as error:
        report.update(analysis_error=repr(error), analysis_traceback=traceback.format_exc(), passed=False)
    write_json(out/"runtime_report.json", report)
    print(json.dumps({k:report.get(k) for k in ("case","passed","protocol_complete","error","analysis_error","checks","timing","recording","equivalence")},indent=2))
    return 0 if report["passed"] else 1


if __name__=="__main__": raise SystemExit(main())
