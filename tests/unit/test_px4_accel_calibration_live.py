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


@pytest.mark.parametrize("failed_save", [2, 3])
def test_windows_progress_file_lock_does_not_abort_serial_monitor(monkeypatch, tmp_path, failed_save):
    writes = []
    cal = ("[cal] calibration started: 2 accel\n" + "".join(
        f"[cal] {side} side done, rotate to a different side\n" for side in m.ORDER
    ) + "[cal] calibration done: accel\n").encode()
    class FakeConsole:
        def __init__(self, *a):
            self.records = []
            self.s = type("S", (), {"is_open": False, "write": lambda _, data: writes.append(data)})()
            self.chunks = iter((b"", cal, b"nsh> "))
        def open(self): self.s.is_open = True
        def close(self): self.s.is_open = False
        def query(self, command): return m.FIRMWARE
        def read(self, *args): return next(self.chunks)
        def record(self, command, data):
            self.records.append(dict(command=command, text=data.decode()))
            return data.decode()
    original = Path.replace
    saves = []
    def locked_once(path, target):
        saves.append(str(path))
        if len(saves) == failed_save:  # Initial record, durable intention, live progress.
            raise PermissionError("Simulated Windows sharing violation")
        return original(path, target)
    monkeypatch.setattr(Path, "replace", locked_once)
    monkeypatch.setattr(m, "ImuConsole", FakeConsole)
    monkeypatch.setattr(m, "guard", lambda *a, **kw: {})
    monkeypatch.setattr(m, "read_parameter", lambda c, p: 24 if p == "MOT_POLE_COUNT" else 0)
    monkeypatch.setattr(m.time, "sleep", lambda _: None)
    s = m.Session(tmp_path / "case", "unused", operator_power_off=True, operator_ready=True)
    s.last_ui_poll = time.monotonic()
    s.run()
    assert s.report["progress_persistence_failures"] == 1
    assert s.report["port_closed"]
    if failed_save == 2:
        assert not s.report["calibration_success"] and not s.report["command_write_attempted"]
        assert (m.COMMAND + "\r").encode() not in writes
        assert s.progress.phase == "failed"
    else:
        assert s.report["calibration_success"] and s.report["post_readback_complete"]
        assert writes.count((m.COMMAND + "\r").encode()) == 1
        assert m.json.loads((s.out / "status.json").read_text(encoding="utf-8"))["phase"] == "done"


def test_binary_serial_data_is_not_mistaken_for_normal_text():
    assert m.binary_corruption(b"\x00\x80\x01\x03" * 30)
    assert not m.binary_corruption(b"\x1b[KINFO  [commander] [cal] hold vehicle still\r\n")
    assert not m.binary_corruption(b"\x00")
