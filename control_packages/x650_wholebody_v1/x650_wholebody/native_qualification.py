"""Read-only admission of an explicitly qualified native numerical backend."""
from pathlib import Path
import hashlib
import json

KIND="cpp_numerics_v1"


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
    evidence={}
    for key in ("matrix","recorded","native_tests"):
        item=proof[key]
        if digest(item["path"])!=item["sha256"]:
            raise ValueError("Native acceptance evidence changed: "+key)
        report=json.loads(Path(item["path"]).read_text())
        if report.get("pass") is not True:
            raise ValueError("Failed native acceptance evidence: "+key)
        evidence[key]=report
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
