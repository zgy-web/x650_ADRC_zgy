import copy
import json
from pathlib import Path
import sys
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
import px4_restore_accel_snapshot as m


def fixture():
    old = {p: 0 for p in m.PARAMETERS}
    old.update({p: 1 for p in m.NAMES if p.endswith("SCALE")})
    current = dict(old)
    current.update({p: 1.05 if p.endswith("SCALE") else .1 for p in m.NAMES})
    return {"report": dict(firmware=m.FIRMWARE, calibration_success=True,
                           post_readback_complete=True, before_parameters=old, after_parameters=current)}


def test_only_eighteen_calibration_values_and_no_other_write():
    plan = m.restore_plan(fixture())
    assert len(plan) == 18
    c = m.RestoreConsole("COM13", plan)
    for cmd in ("commander arm", "reboot", "param set MOT_POLE_COUNT 14", "commander calibrate accel",
                "param set CAL_ACC0_XOFF 99", "param set CAL_ACC0_XOFF 0\rcommander arm"):
        with pytest.raises(ValueError): c.query(cmd)
    assert not c.s.is_open


@pytest.mark.parametrize("change", ["firmware", "unrelated", "incomplete", "scale", "not_done"])
def test_unreviewed_snapshots_are_rejected(change):
    s=fixture();r=s["report"]
    if change == "firmware": r["firmware"]="unknown"
    elif change == "unrelated": r["after_parameters"]["MOT_POLE_COUNT"]=14
    elif change == "incomplete": del r["before_parameters"]["CAL_ACC0_ID"]
    elif change == "scale": r["before_parameters"]["CAL_ACC0_XSCALE"]=0
    else: r["calibration_success"]=False
    with pytest.raises(ValueError):m.restore_plan(s)


@pytest.mark.parametrize("mismatch", [False, True])
def test_reviewed_recovery_and_no_write_on_current_mismatch(monkeypatch, tmp_path, mismatch):
    s=fixture();values=dict(s["report"]["after_parameters"]);writes=[]
    if mismatch:values["MOT_POLE_COUNT"]=14
    class FakeConsole:
        def __init__(self,*args):self.records=[];self.s=type("S",(),{"is_open":False})()
        def open(self):self.s.is_open=True
        def close(self):self.s.is_open=False
        def query(self,command):
            if command=="ver all":return m.FIRMWARE
            writes.append(command)
            if command.startswith("param set "):
                _,_,name,value=command.split();values[name]=float(value)
            return "nsh>"
    monkeypatch.setattr(m,"RestoreConsole",FakeConsole)
    monkeypatch.setattr(m,"read_parameter",lambda c,p:values[p])
    monkeypatch.setattr(m,"guard",lambda *a,**kw:{})
    src=tmp_path/"session.json";src.write_text(json.dumps(s))
    result=m.run(src,tmp_path/"output",operator_power_off=True)
    assert result["port_closed"]
    if mismatch:
        assert not result["complete"] and not writes
    else:
        assert result["complete"] and values==s["report"]["before_parameters"]
        assert len(writes)==19 and writes[-1]=="param save"


def test_no_attestation_no_files(tmp_path):
    with pytest.raises(ValueError):m.run(tmp_path/"missing",tmp_path/"out")
    assert not (tmp_path/"out").exists()
