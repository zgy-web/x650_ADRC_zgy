from pathlib import Path
import sys
import time
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import px4_accel_calibration_live as m


def test_no_success_without_start_and_all_six_faces():
    p = m.Progress()
    p.feed("[cal] calibration done: accel\n")
    assert p.phase == "prepared"
    p.feed("[cal] calibration started: 2 accel\n[cal] down side done, rotate to a different side\n")
    p.feed("[cal] calibration done: accel\n")
    assert p.phase == "unknown"


def test_duplicate_messages_do_not_count_faces_twice():
    p = m.Progress()
    p.feed("[cal] calibration started: 2 accel\n")
    for side in m.ORDER:
        p.feed(f"[cal] {side} orientation detected\n")
        assert p.measuring == side
        p.feed(f"[cal] {side} side done, rotate to a different side\n" * 2)
    assert len(p.done) == 6 and p.phase == "running"
    p.feed("[cal] calibration done: accel\n")
    assert p.phase == "done"


@pytest.mark.parametrize("message,expected", [
    ("[cal] calibration failed: reading sensor", "failed"),
    ("[cal] calibration cancelled", "cancelled"),
])
def test_failures_are_terminal(message, expected):
    p = m.Progress()
    p.feed("[cal] calibration started: 2 accel\n" + message + "\n")
    assert p.phase == expected
    p.feed("[cal] calibration done: accel\n")
    assert p.phase == expected


def test_half_page_or_unrelated_calibration_cannot_succeed():
    p = m.Progress()
    p.feed("[cal] calibration started: 2 gyro\n[cal] down side done, rotate to a different side\n")
    assert not p.started and not p.done


def test_operator_attestation_is_required(tmp_path):
    with pytest.raises(ValueError):
        m.Session(tmp_path / "no", "COM13", operator_power_off=False, operator_ready=True)
    assert not (tmp_path / "no").exists()


def test_only_one_start_with_recent_live_display(monkeypatch, tmp_path):
    started = []
    class FakeThread:
        def __init__(self, *a, **kw): pass
        def start(self): started.append(True)
    monkeypatch.setattr(m.threading, "Thread", FakeThread)
    s = m.Session(tmp_path / "case", "unused", operator_power_off=True, operator_ready=True)
    assert not s.start()
    s.last_ui_poll = time.monotonic()
    assert s.start()
    assert not s.start()
    assert len(started) == 1 and s.report["command_attempts"] == 0


def test_bad_firmware_never_sends_calibration(monkeypatch, tmp_path):
    sent = []
    class FakeConsole:
        def __init__(self, *a):
            self.records=[]
            self.s=type("S", (), {"is_open":False})()
        def open(self): self.s.is_open=True
        def close(self): self.s.is_open=False
        def query(self, c): sent.append(c);return "wrong firmware"
    monkeypatch.setattr(m, "ImuConsole", FakeConsole)
    s = m.Session(tmp_path / "case", "unused", operator_power_off=True, operator_ready=True)
    s.run()
    assert sent == ["ver all"]
    assert s.report["command_attempts"] == 0 and s.report["port_closed"]


def test_direction_labels_match_pinned_firmware():
    assert m.ORDER == ("down", "left", "up", "right", "front", "back")
    assert m.Progress().data()["order"] == list(m.ORDER)
    assert "机头朝上" in m.LABELS["back"]
    assert "机头朝下" in m.LABELS["front"]
    assert m.COMMAND == "commander calibrate accel"
