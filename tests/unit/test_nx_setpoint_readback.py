"""Independent field oracle and rejection tests; no hardware access."""
import copy
import json
import math
from pathlib import Path
import sys

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import px4_debug_console as debug
import summarize_nx_setpoint_readback as audit


STATUS = "timestamp: 100\narming_state: 1\nnav_state: 4\nhil_state: 0\nfailsafe: False\n"


def frame_text(stamp, scale=1.):
    # An independent NED fixture, not generated with the audit's pattern helper.
    def vector(values):
        return ", ".join(f"{x*scale:.5f}" for x in values)
    return (f"\nTOPIC: trajectory_setpoint\n timestamp: {stamp} (0.001 seconds ago)\n"
            f" position: [{vector([-.25, .125, -.5])}]\n"
            f" velocity: [{vector([-.0625, .03125, -.015625])}]\n"
            f" acceleration: [{vector([.125, .0625, .03125])}]\n"
            " jerk: [nan, nan, nan]\n yaw: 1.57080\n yawspeed: nan\n")


def stream_text():
    return "".join(frame_text(1000+i*100+j, scale) for i, scale in enumerate((1., -1., .5)) for j in (1, 2))


def baseline_text(stamp):
    return (f"timestamp: {stamp}\nposition: [nan, nan, nan]\nvelocity: [nan, nan, 0.70000]\n"
            "acceleration: [0, 0, nan]\njerk: [nan, nan, nan]\nyaw: -2.81500\nyawspeed: nan\n")


def fixture(root):
    status = dict(complete=True, port_closed=True, before=debug.status_fields(STATUS),
                  after=dict(debug.status_fields(STATUS), timestamp=2000))
    identity = dict(binary=dict(sha256="a"*64, source_sha256="b"*64))
    events = [dict(event="session_started", physical_px4=True, binary_sha256="a"*64, source_sha256="b"*64)]
    counts = [0, 0, 0]
    for n in range(150):
        t = n / 15.
        i = 0 if t < 3 else (1 if t < 6 else 2)
        scale = (1., -1., .5)[i]
        counts[i] += 1
        payload = dict(event="setpoint_sent", seq=n+1, pattern=i, elapsed_s=t, timestamp=10_000+n,
                       position=[-.25*scale, .125*scale, -.5*scale],
                       velocity=[-.0625*scale, .03125*scale, -.015625*scale],
                       acceleration=[.125*scale, .0625*scale, .03125*scale],
                       yaw=1.5707963705062866, jerk=[None]*3, yawspeed=None, null_represents_nan=True)
        events.append(dict(event="probe", payload=payload))
    final = dict(event="probe_final", complete=True, sent=150, pattern_counts=counts,
                 armed_seen=False, offboard_seen=False, hil_seen=False, status_fault=False,
                 arm_requests_sent=0, offboard_requests_sent=0, actuator_commands_sent=0)
    events += [dict(event="probe", payload=final),
               dict(event="process_cleanup", agent_stopped=True, probe_stopped=True),
               dict(event="cleanup", ram_removed=True, persistent_files_created=[]),
               dict(event="complete", passed=True)]
    console = [dict(command=debug.LISTENER, text=stream_text()),
               dict(command="ver all", text="PX4 git-hash: "+debug.FIRMWARE),
               dict(command="listener trajectory_setpoint -n 1", text=baseline_text(50))]
    return status, identity, events, console


def evaluate(root, data):
    status, identity, events, console = data
    for name, value in (("session_status", status), ("build_identity", identity), ("console", console)):
        (root/(name+".json")).write_text(json.dumps(value), encoding="utf-8")
    (root/"remote.jsonl").write_text("\n".join(json.dumps(x) for x in events), encoding="utf-8")
    return audit.summarize(root)


def test_independent_field_match_is_not_a_control_execution_claim(tmp_path):
    report = evaluate(tmp_path, fixture(tmp_path))
    assert report["passed"] and report["matched_counts"] == [2, 2, 2]
    assert report["sent_count"] == 150
    assert not any(report[k] for k in ("control_execution_verified", "mpc_live_validation",
                                      "offboard_takeover_tested", "actuators_tested", "latency_measured"))


@pytest.mark.parametrize("fault", ["missing_end", "unsafe", "unexpected_publisher", "wrong_binary", "left_files", "live_agent",
                                  "short", "wrong_axis", "wrong_nan", "vector_dimension", "late", "bad_sequence",
                                  "missing_pattern", "old_console_timestamp", "reordered_pattern", "partial_console",
                                  "wrong_firmware", "unexplained_wire_frame"])
def test_corrupt_or_incomplete_run_cannot_pass(tmp_path, fault):
    status, identity, events, console = fixture(tmp_path)
    if fault == "missing_end": events.pop()
    elif fault == "unsafe": status["after"]["arming_state"] = 2
    elif fault == "unexpected_publisher": events[-4]["payload"]["offboard_requests_sent"] = 1
    elif fault == "wrong_binary": events[0]["binary_sha256"] = "c"*64
    elif fault == "left_files": events[-2]["persistent_files_created"] = ["unexpected.log"]
    elif fault == "live_agent": events[-3]["agent_stopped"] = False
    elif fault == "short": del events[1:80]
    elif fault == "wrong_axis": events[1]["payload"]["position"] = [.125, -.25, -.5]
    elif fault == "wrong_nan": events[1]["payload"]["jerk"][0] = 0.
    elif fault == "vector_dimension": events[1]["payload"]["position"] = []
    elif fault == "late": events[1]["payload"]["elapsed_s"] = 10.
    elif fault == "bad_sequence": events[1]["payload"]["seq"] = 2
    elif fault == "missing_pattern": console[0]["text"] = frame_text(1001)+frame_text(1002)
    elif fault == "old_console_timestamp": status["before"]["timestamp"] = 1100
    elif fault == "reordered_pattern": console[0]["text"] = stream_text()+frame_text(1500)
    elif fault == "partial_console": console[0]["text"] += "\ntimestamp: 1600\nposition: [0,0,0]\n"
    elif fault == "wrong_firmware": console[1]["text"] = "unrecognized"
    elif fault == "unexplained_wire_frame": console[0]["text"] += frame_text(1600).replace("0.70000", "0.71000").replace("-0.25000", "0.25100")
    with pytest.raises(ValueError): evaluate(tmp_path, (status, identity, events, console))


def test_interleaved_internal_values_are_not_treated_as_nx_frames():
    frames = audit.parse_frames(stream_text())
    frames.insert(3, audit.parse_frames(baseline_text(1050))[0])
    report = audit.check_frames(frames, 100, 2000, audit.parse_frames(baseline_text(50))[0])
    assert report["matched_counts"] == [2, 2, 2]
    assert report["unmatched_internal_frames"] == 1


def test_duplicates_do_not_inflate_independent_evidence():
    frames = audit.parse_frames(frame_text(1000)) * 6
    with pytest.raises(ValueError, match="Fewer than two"):
        audit.check_frames(frames, 100, 2000)
    conflict = copy.deepcopy(frames[0]); conflict["position"][0] = 9.
    with pytest.raises(ValueError, match="Conflicting"):
        audit.check_frames([frames[0], conflict], 100, 2000)


@pytest.mark.parametrize("replacement", ["Falsex", "false", "0", "nan", "True"])
def test_invalid_or_unsafe_status_is_not_silently_accepted(replacement):
    with pytest.raises(ValueError): debug.status_fields(STATUS.replace("False", replacement))


def test_ambiguous_status_is_rejected():
    with pytest.raises(ValueError): debug.status_fields(STATUS+STATUS)


@pytest.mark.parametrize("command", [debug.START_DDS, debug.STOP_USB_MAVLINK, debug.STOP_DDS, "commander arm", "param set SYS_USB_AUTO 1"])
def test_readonly_console_cannot_mutate(command):
    console = debug.DebugConsole("COM13")  # Constructing does not open the port.
    with pytest.raises(ValueError): console.query(command)
    assert not console.s.is_open
