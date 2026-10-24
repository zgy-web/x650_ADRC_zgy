"""Read-only admission of an explicitly qualified native numerical backend."""
from pathlib import Path
import hashlib
import json
from types import ModuleType

KIND="cpp_numerics_v1"
STREAMED_VERIFIER="scripts/qualification_recording.py"


def reject_draft_evidence(value):
    if value.get("scratch_overlay_only") is True or value.get("deployment_eligible") is False:
        raise ValueError("Scratch numerical evidence is not eligible for deployment")


def verify_numerical_evidence(root,report_path,report,sources):
    """Verify external evidence only at qualification/ground admission.

    Historical inline reports remain readable. New streamed reports cannot
    become admissible merely by preserving their compact summary JSON.
    This function is intentionally never called by an active controller step.
    """
    reject_draft_evidence(report)
    rows=report.get("rows",[])
    streamed=("evidence_layout" in report or "natural_gc" in report
              or any(isinstance(row,dict) and
                     ("row_evidence" in row or "candidate_execution_count" in row) for row in rows))
    if not streamed:
        return None
    if (report.get("complete") is not True or report.get("pass") is not True
            or report.get("evidence_layout","streamed_rows_v1")!="streamed_rows_v1"):
        raise ValueError("Completed streamed numerical evidence is required")
    if (STREAMED_VERIFIER not in sources
            or report.get("deployment_sources",{}).get(STREAMED_VERIFIER)!=sources[STREAMED_VERIFIER]):
        raise ValueError("Streamed evidence verifier is not source-bound")
    path=Path(root).resolve()/STREAMED_VERIFIER
    source=path.read_bytes()
    if hashlib.sha256(source).hexdigest()!=sources[STREAMED_VERIFIER]:
        raise ValueError("Streamed evidence verifier source changed")
    # Execute the exact hashed source, not an import cache or stale .pyc.
    module=ModuleType("_x650_qualified_evidence_verifier")
    module.__file__=str(path)
    exec(compile(source,str(path),"exec"),module.__dict__)
    result=module.verify_streamed_evidence(Path(report_path).resolve().parent,report)
    if (not isinstance(result,dict) or result.get("passed") is not True
            or result.get("rows")!=report.get("total") or result.get("candidates",0)<1
            or result.get("complete_gc_trace_verified") is not True
            or result.get("artifact_identity")!="path+bytes+sha256"):
        raise ValueError("Incomplete streamed numerical evidence verification")
    return result


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def validate_binding(binding):
    if (not isinstance(binding,dict) or set(binding)!={"kind","library","library_sha256",
            "software_proof","software_proof_sha256"} or binding["kind"]!=KIND):
        raise ValueError("Explicit native numerical backend binding required")
    for key in ("library","software_proof"):
        if not isinstance(binding[key],str) or not Path(binding[key]).is_absolute():
            raise ValueError("Absolute native backend input required: "+key)
        value=binding[key+"_sha256"]
        if not isinstance(value,str) or len(value)!=64 or set(value)-set("0123456789abcdef"):
            raise ValueError("Explicit native backend SHA-256 required: "+key)
    return binding


def verify_backend(root,binding):
    root=Path(root).resolve();validate_binding(binding)
    for key in ("library","software_proof"):
        if digest(binding[key])!=binding[key+"_sha256"]:
            raise ValueError("Native backend artifact changed: "+key)
    proof=json.loads(Path(binding["software_proof"]).read_text())
    reject_draft_evidence(proof)
    if (proof.get("kind")!="native_numerics_software_acceptance_v1"
            or proof.get("pass") is not True or proof.get("no_flight") is not True):
        raise ValueError("Completed native numerical acceptance is required")
    library=proof.get("native_library",{})
    if (Path(library.get("path","")).resolve()!=Path(binding["library"]).resolve()
            or library.get("sha256")!=binding["library_sha256"]):
        raise ValueError("Qualified and selected native libraries differ")
    sources=proof.get("deployment_sources",{})
    required={"control_packages/x650_wholebody_v1/x650_wholebody/"+name for name in
              ("native_mpc.py","native_optimization.py","native_qualification.py","ts20_mpc.py","mpc.py")}
    if not required.issubset(sources):
        raise ValueError("Native proof lacks actually loaded source scope")
    for relative,expected in sources.items():
        path=(root/relative).resolve()
        if path.relative_to(root).as_posix()!=relative or digest(path)!=expected:
            raise ValueError("Native qualified source differs: "+relative)
    evidence={};streamed_evidence={}
    for key in ("matrix","recorded","native_tests"):
        item=proof[key]
        if digest(item["path"])!=item["sha256"]:
            raise ValueError("Native acceptance evidence changed: "+key)
        report=json.loads(Path(item["path"]).read_text())
        if report.get("pass") is not True:
            raise ValueError("Failed native acceptance evidence: "+key)
        reject_draft_evidence(report)
        if key in ("matrix","recorded"):
            verified=verify_numerical_evidence(root,item["path"],report,sources)
            if verified is not None:streamed_evidence[key]=verified
        evidence[key]=report
    if streamed_evidence!=proof.get("streamed_evidence",{}):
        raise ValueError("Streamed numerical evidence differs from its qualification receipt")
    if streamed_evidence and "control_packages/x650_wholebody_v1/x650_wholebody/native_joint_prediction.py" not in sources:
        raise ValueError("Streamed native proof lacks the packed joint predictor source")
    matrix,recorded,tests=(evidence[key] for key in ("matrix","recorded","native_tests"))
    if (matrix.get("total")!=30 or matrix.get("passed")!=30
            or recorded.get("passed")!=recorded.get("total") or recorded.get("total",0)<1
            or tests.get("tests",0)<11 or tests.get("errors")!=0 or tests.get("failures")!=0
            or tests.get("skipped")!=0):
        raise ValueError("Native required coverage is incomplete")
    for report in (matrix,recorded):
        if (report.get("native_library")!=library or report.get("sources_unchanged") is not True
                or any(sources.get(p)!=h for p,h in report["deployment_sources"].items())):
            raise ValueError("Native report/source identities differ")
    return {"binding":dict(binding),"matrix_cases":30,"recorded_calls":recorded["total"],
            "native_tests":tests["tests"],"no_hardware_qualification":True}
