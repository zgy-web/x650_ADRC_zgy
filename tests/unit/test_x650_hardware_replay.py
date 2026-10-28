import copy
from contextlib import contextmanager
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
def load(name):
    spec=importlib.util.spec_from_file_location(name,ROOT/"scripts"/(name+".py"))
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module);return module
prep=load("prepare_x650_hardware_replay")
summary=load("summarize_nx_full_interface_replay")


class PreparationTests(unittest.TestCase):
    def test_message_convention(self):
        state=dict(rotation=[1,0,0,0,1,0,0,0,1],position=[1,2,3],velocity=[4,5,6],
                   acceleration=[7,8,9],omega_body=[1,2,3],q=[.1,.2],dq=[.3,.4],stamp=1,control_time=1.01,joint_stamp=1.005)
        frame=prep.frame(dict(id="golden",input=dict(state=state)))
        self.assertEqual(frame["position_ned"],[2,1,-3]);self.assertEqual(frame["omega_frd"],[1,-2,-3])
        self.assertEqual(frame["sample_us"],1000000);self.assertEqual(frame["joint_ns"],1005000000)
        for a,b in zip(frame["expected_state"]["rotation"],state["rotation"]):self.assertAlmostEqual(a,b,places=14)

    def test_quaternion_branches(self):
        for r in ([[1,0,0],[0,1,0],[0,0,1]],[[1,0,0],[0,-1,0],[0,0,-1]],
                  [[-1,0,0],[0,1,0],[0,0,-1]],[[-1,0,0],[0,-1,0],[0,0,1]]):
            rebuilt=prep.rotation(prep.quaternion(r))
            for i in range(3):
                for j in range(3):self.assertAlmostEqual(r[i][j],rebuilt[i][j])

    def test_nonfinite(self):
        with self.assertRaises(ValueError):prep.f32(math.nan)


class SummaryTests(unittest.TestCase):
    def setUp(self):
        self.item=dict(name="test",count=1,sha256="input",source_sha256="source")
        self.libs={key:dict(sha256=key) for key in ("core","optimization","predictor")}
        row={k:True for k in summary.FLAGS};row.update(id="one",adapter_state_max_error=0,
            historical_baseline_max_error=0,quantized_input_command_difference=1e-8,quantized_input_position_reference_difference=0,
            quantized_input_command_absolute_differences=[1e-8,0,0,0,0],
            expected_setpoint=dict(timestamp=123,position=[1,2,3],velocity=[1,2,3],acceleration=[1,2,3],yaw=.1,
                                   jerk=[None]*3,yawspeed=None,null_represents_nan=True))
        self.result=dict(kind="x650_hardware_interface_replay_result_v1",complete=True,passed=True,input_sha256="input",
            source_sha256="source",physical_validation=False,raw_cdr_recording=False,publishers_created=0,row_count=1,rows=[row],
            loaded_core=dict(sha256="core"),loaded_optimizer=dict(sha256="optimization"),loaded_predictor=dict(sha256="predictor"))
    def verify(self):return summary.checked_result(self.result,self.item,self.libs,["one"])
    def test_healthy(self):self.assertTrue(self.verify()["passed"])
    def test_missing_rows(self):
        self.result["rows"]=[]
        with self.assertRaises(ValueError):self.verify()
    def test_changed_code(self):
        self.result["loaded_core"]["sha256"]="wrong"
        with self.assertRaises(ValueError):self.verify()
    def test_failed_row(self):
        self.result["rows"][0]["cdr_setpoint_fields_exact"]=False
        with self.assertRaises(ValueError):self.verify()
    def test_overclaim(self):
        self.result["physical_validation"]=True
        with self.assertRaises(ValueError):self.verify()
    def test_nan_mask(self):
        self.result["rows"][0]["expected_setpoint"]["yawspeed"]=0
        with self.assertRaises(ValueError):self.verify()
    def test_order(self):
        self.result["rows"][0]["id"]="different"
        with self.assertRaises(ValueError):self.verify()
    def test_nonfinite_evidence(self):
        self.result["rows"][0]["adapter_state_max_error"]=math.nan
        with self.assertRaises(ValueError):self.verify()

    @contextmanager
    def session(self):
        with tempfile.TemporaryDirectory() as tmp:
            run=Path(tmp)/"run";inputs=Path(tmp)/"inputs";run.mkdir();inputs.mkdir()
            raw=json.dumps(dict(recorded=dict(cases=[dict(id="one")]))).encode()
            item=dict(self.item,file="test.json",sha256=hashlib.sha256(raw).hexdigest())
            (inputs/"test.json").write_bytes(raw)
            (inputs/"manifest.json").write_text(json.dumps(dict(inputs=[item],row_count=1)))
            request=dict(inputs=[item],libraries=self.libs,archive_sha256="archive",files={"bound_native/mpc_core.hpp":"header"})
            (run/"request_manifest.json").write_text(json.dumps(request))
            result=copy.deepcopy(self.result);result["input_sha256"]=item["sha256"]
            artifacts={"configure.log":b"ok","build.log":b"ok","ctest.log":b"ok","test.log":json.dumps(result).encode()}
            events=[]
            for name,data in artifacts.items():
                (run/name).write_bytes(data)
                events.append(dict(event="artifact",name=name,sha256=hashlib.sha256(data).hexdigest(),exit_code=0))
            events.extend([dict(event="build_identity",source_archive_sha256="archive",bound_header_sha256="header",
                                math_libraries={k:k for k in self.libs},ram_only=True),
                           dict(event="cleanup",ram_removed=True,libraries_unchanged=True,persistent_files_created=[]),
                           dict(event="complete",passed=True,results=[dict(name="test",count=1,passed=True)])])
            (run/"transport.jsonl").write_text("\n".join(json.dumps(e) for e in events))
            yield run,inputs,events

    def test_session_coverage_and_identity(self):
        with self.session() as (run,inputs,events):
            self.assertEqual(summary.summarize(run,inputs)["row_count"],1)

    def test_session_rejects_missing_or_changed_evidence(self):
        for fault in ("duplicate_artifact","failed_phase","wrong_header","missing_batch","changed_ctest","empty_request"):
            with self.subTest(fault=fault), self.session() as (run,inputs,events):
                if fault=="duplicate_artifact":events.insert(0,events[0])
                elif fault=="failed_phase":events[0]["exit_code"]=1
                elif fault=="wrong_header":events[-3]["bound_header_sha256"]="wrong"
                elif fault=="missing_batch":events[-1]["results"]=[]
                elif fault=="changed_ctest":(run/"ctest.log").write_text("changed")
                elif fault=="empty_request":
                    p=run/"request_manifest.json";r=json.loads(p.read_text());r["inputs"]=[];p.write_text(json.dumps(r))
                (run/"transport.jsonl").write_text("\n".join(json.dumps(e) for e in events))
                with self.assertRaises(ValueError):summary.summarize(run,inputs)


if __name__=="__main__":unittest.main()
