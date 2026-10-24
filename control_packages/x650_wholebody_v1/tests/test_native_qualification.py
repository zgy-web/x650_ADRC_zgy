"""Ground admission must reopen new streamed evidence, while reading old proofs."""
import copy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from x650_wholebody.native_qualification import (
    STREAMED_VERIFIER, verify_backend, verify_numerical_evidence,
)


# A tiny contract fixture proves that admission executes the source-bound
# verifier and propagates its artifact checks. The full verifier's row/GC/
# candidate semantics are tested separately by its qualification helper tests.
VERIFIER = '''import hashlib,json
from pathlib import Path
def verify_streamed_evidence(output, report):
    for item in report["rows"]:
        receipt=item["row_evidence"]
        data=Path(receipt["path"]).read_bytes()
        if len(data)!=receipt["bytes"] or hashlib.sha256(data).hexdigest()!=receipt["sha256"]:
            raise ValueError("Fixture full row changed")
    return dict(passed=True,rows=report["total"],candidates=1,
        complete_gc_trace_verified=True,artifact_identity="path+bytes+sha256")
'''


def identity(path):
    data=path.read_bytes()
    return dict(path=str(path.resolve()),bytes=len(data),sha256=hashlib.sha256(data).hexdigest())


class NativeQualificationEvidence(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root=Path(self.directory.name)
        self.sources={}
        for name in ("native_mpc.py","native_optimization.py","native_qualification.py",
                     "ts20_mpc.py","mpc.py","native_joint_prediction.py"):
            self.source("control_packages/x650_wholebody_v1/x650_wholebody/"+name,"# fixture\n")
        self.source(STREAMED_VERIFIER,VERIFIER)
        self.library=self.root/"native.so"
        self.library.write_bytes(b"not executed: qualification binding fixture")
        self.native=dict(path=str(self.library),sha256=identity(self.library)["sha256"],abi=1)
        self.evidence=self.root/"evidence"
        self.evidence.mkdir()
        self.full_row=self.evidence/"full_row.json"
        self.full_row.write_text('{"fixture":"complete row"}\n')
        self.matrix=self.report(30)
        self.recorded=self.report(1)
        self.tests=dict(pass_=True,tests=11,failures=0,errors=0,skipped=0)
        self.tests["pass"]=self.tests.pop("pass_")

    def source(self,relative,text):
        path=self.root/relative
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(text)
        self.sources[relative]=identity(path)["sha256"]

    def report(self,count):
        return {"pass":True,"complete":True,"total":count,"passed":count,
                "sources_unchanged":True,"native_library":self.native,
                "deployment_sources":dict(self.sources)}

    def stream(self,report):
        report.update(evidence_layout="streamed_rows_v1",natural_gc={"fixture":True},
                      rows=[dict(index=0,row_evidence=identity(self.full_row))])

    def write_json(self,path,value):
        path.write_text(json.dumps(value))
        return identity(path)

    def bind(self,*,receipt=True,**proof_updates):
        evidence={key:self.write_json(self.evidence/(key+".json"),value)
                  for key,value in (("matrix",self.matrix),("recorded",self.recorded),("native_tests",self.tests))}
        proof=dict(kind="native_numerics_software_acceptance_v1",no_flight=True,
                   native_library=self.native,deployment_sources=dict(self.sources),**evidence)
        proof["pass"]=True
        if receipt and "evidence_layout" in self.recorded:
            proof["streamed_evidence"]={"recorded":verify_numerical_evidence(
                self.root,self.evidence/"recorded.json",self.recorded,self.sources)}
        proof.update(proof_updates)
        self.proof_path=self.root/"acceptance.json"
        evidence=self.write_json(self.proof_path,proof)
        return dict(kind="cpp_numerics_v1",library=str(self.library),library_sha256=self.native["sha256"],
                    software_proof=str(self.proof_path),software_proof_sha256=evidence["sha256"])

    def test_historical_inline_proof_does_not_need_streamed_receipt(self):
        self.sources.pop(STREAMED_VERIFIER)
        for report in (self.matrix,self.recorded):report["deployment_sources"]=dict(self.sources)
        (self.root/STREAMED_VERIFIER).unlink()
        result=verify_backend(self.root,self.bind())
        self.assertEqual(result["recorded_calls"],1)

    def test_streamed_report_is_reverified_at_ground_admission(self):
        self.stream(self.recorded)
        result=verify_backend(self.root,self.bind())
        self.assertEqual(result["recorded_calls"],1)

    def test_external_row_missing_or_changed_rejects_unchanged_compact_report(self):
        self.stream(self.recorded)
        binding=self.bind()
        original=self.full_row.read_bytes()
        with self.subTest("changed"):
            self.full_row.write_bytes(b"changed full row")
            with self.assertRaisesRegex(ValueError,"full row changed"):
                verify_backend(self.root,binding)
        self.full_row.write_bytes(original)
        with self.subTest("missing"):
            self.full_row.unlink()
            with self.assertRaises(FileNotFoundError):verify_backend(self.root,binding)

    def test_streamed_marker_cannot_skip_verifier_by_removing_row_receipts(self):
        self.stream(self.recorded)
        self.recorded["rows"]=[{"index":0}]
        with self.assertRaises(KeyError):
            verify_numerical_evidence(self.root,self.evidence/"recorded.json",self.recorded,self.sources)

    def test_new_layout_requires_completed_report_and_bound_verifier(self):
        self.stream(self.recorded)
        for label,mutate in (
            ("incomplete",lambda r:r.update(complete=False)),
            ("unknown layout",lambda r:r.update(evidence_layout="other")),
            ("unbound",lambda r:r["deployment_sources"].pop(STREAMED_VERIFIER)),
        ):
            with self.subTest(label):
                report=copy.deepcopy(self.recorded);mutate(report)
                with self.assertRaises(ValueError):
                    verify_numerical_evidence(self.root,self.evidence/"recorded.json",report,self.sources)

    def test_verifier_exact_source_hash_is_checked_before_execution(self):
        self.stream(self.recorded)
        binding=self.bind()
        (self.root/STREAMED_VERIFIER).write_text('raise AssertionError("must not execute")\n')
        with self.assertRaisesRegex(ValueError,"source differs"):
            verify_backend(self.root,binding)

    def test_draft_reports_or_proof_cannot_be_signed_or_deployed(self):
        for flag,value in (("scratch_overlay_only",True),("deployment_eligible",False)):
            with self.subTest(flag,scope="proof"):
                with self.assertRaisesRegex(ValueError,"Scratch"):
                    verify_backend(self.root,self.bind(**{flag:value}))
            for key in ("matrix","recorded","tests"):
                with self.subTest(flag,scope=key):
                    report=getattr(self,key);report[flag]=value
                    with self.assertRaisesRegex(ValueError,"Scratch"):
                        verify_backend(self.root,self.bind())
                    report.pop(flag)

    def test_new_proof_requires_matching_external_verification_receipt(self):
        self.stream(self.recorded)
        with self.assertRaisesRegex(ValueError,"qualification receipt"):
            verify_backend(self.root,self.bind(receipt=False))

    def test_new_proof_binds_packed_predictor_without_breaking_old_proofs(self):
        missing="control_packages/x650_wholebody_v1/x650_wholebody/native_joint_prediction.py"
        self.sources.pop(missing)
        for report in (self.matrix,self.recorded):report["deployment_sources"]=dict(self.sources)
        verify_backend(self.root,self.bind())
        self.stream(self.recorded)
        with self.assertRaisesRegex(ValueError,"packed joint predictor"):
            verify_backend(self.root,self.bind())

    def test_failed_or_partial_verifier_result_cannot_authorize(self):
        self.stream(self.recorded)
        for code in ('return dict(passed=True)', 'return dict(passed=False,rows=1,candidates=1,'
                     'complete_gc_trace_verified=True,artifact_identity="path+bytes+sha256")'):
            with self.subTest(code):
                self.source(STREAMED_VERIFIER,"def verify_streamed_evidence(output, report):\n    "+code+"\n")
                self.recorded["deployment_sources"]=dict(self.sources)
                with self.assertRaisesRegex(ValueError,"Incomplete streamed"):
                    verify_numerical_evidence(self.root,self.evidence/"recorded.json",self.recorded,self.sources)


if __name__=="__main__":unittest.main()
