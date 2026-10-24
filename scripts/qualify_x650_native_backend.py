"""Fresh regression/matrix proof for explicitly selected native quiet-hold math.

Reuses only the previous proof's unchanged plant/oracle identities. It runs
all original control/adapter/auditor regression groups plus native tests. The
recorded-call report is verified separately and never relabelled as a flight.
"""
from __future__ import annotations
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import zipfile

ROOT=Path(__file__).resolve().parents[1]
sys.path[:0]=[str(ROOT),str(ROOT/"scripts"),str(ROOT/"control_packages/x650_wholebody_v1"),
             str(ROOT/"src"),str(ROOT/"simulations/x650_wholebody_mpc_v1")]
from validate_x650_ts20_causal_software import run_tests,test_files,identity,write_new
from validate_x650_native_numerics import source_receipt
from x650_wholebody.native_qualification import reject_draft_evidence,verify_numerical_evidence


def main(argv=None):
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--previous-proof",type=Path,required=True)
    ap.add_argument("--previous-sha256",required=True)
    ap.add_argument("--recorded-report",type=Path,required=True)
    ap.add_argument("--recorded-sha256",required=True)
    ap.add_argument("--native-library",type=Path,required=True)
    ap.add_argument("--native-sha256",required=True)
    ap.add_argument("--output",type=Path,required=True)
    args=ap.parse_args(argv);args.output.mkdir(parents=True,exist_ok=False);out=args.output.resolve()
    for p,h in ((args.previous_proof,args.previous_sha256),(args.recorded_report,args.recorded_sha256),
                (args.native_library,args.native_sha256)):
        if identity(p)["sha256"]!=h:raise ValueError("Pinned artifact changed: "+str(p))
    prior=json.loads(args.previous_proof.read_text());recorded=json.loads(args.recorded_report.read_text())
    reject_draft_evidence(prior);reject_draft_evidence(recorded)
    assert recorded["pass"] and recorded["total"]==recorded["passed"] and recorded["sources_unchanged"]
    native={"path":str(args.native_library.resolve()),"sha256":args.native_sha256,"abi":1}
    assert recorded["native_library"]==native
    for key in ("native_library","oracle","operating_condition"):
        entry=prior[key];assert identity(entry["path"])["sha256"]==entry["sha256"]
    for name,sha in recorded["deployment_sources"].items():
        assert identity(ROOT/name)["sha256"]==sha,name
    recorded_streamed=verify_numerical_evidence(ROOT,args.recorded_report,recorded,recorded["deployment_sources"])
    paths=set(prior["deployment_sources"])|set(source_receipt())|{Path(__file__).relative_to(ROOT).as_posix()}
    groups=test_files()
    paths.update(p.relative_to(ROOT).as_posix() for rows in groups.values() for p in rows)
    before={p:identity(ROOT/p)["sha256"] for p in sorted(paths)}
    write_new(out/"sources_before.json",before)
    with zipfile.ZipFile(out/"executed_sources.zip","x",zipfile.ZIP_DEFLATED) as z:
        for p in before:z.write(ROOT/p,p)
    os.environ.update(TS20_TEST_NATIVE_LIB=prior["native_library"]["path"],
        X650_NATIVE_PREDICTION_LIBRARY=prior["native_library"]["path"],
        X650_NATIVE_OPTIMIZATION_LIBRARY=str(args.native_library.resolve()))
    suite=run_tests(groups,out)
    assert suite["successful"] and suite["skipped"]==0
    test_report={"pass":True,"tests":suite["tests"],"failures":suite["failures"],
                 "errors":suite["errors"],"skipped":suite["skipped"],
                 "scope":"all original package/interface/auditor/shared-loop groups plus native negative/gate tests",
                 "report":identity(out/"package_report.json")}
    write_new(out/"native_tests.json",test_report)
    cmd=[sys.executable,str(ROOT/"scripts/validate_x650_native_numerics.py"),
         "--matrix-script",str(ROOT/"artifacts/ts20_prediction_chain_20261022/benchmark_gain_kd025_matrix_v1.py"),
         "--matrix-sha256",prior["software_matrix"]["original_script_sha256"],
         "--native-library",str(args.native_library.resolve()),"--native-sha256",args.native_sha256,
         "--output",str(out/"matrix")]
    with (out/"matrix.log").open("x") as log:
        subprocess.run(cmd,cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,check=True)
    matrix_path=out/"matrix/matrix_report.json";matrix=json.loads(matrix_path.read_text())
    assert matrix["pass"] and matrix["total"]==matrix["passed"]==30
    assert all(identity(ROOT/p)["sha256"]==h for p,h in before.items())
    for report in (matrix,recorded):
        assert all(before.get(p)==h for p,h in report["deployment_sources"].items())
    matrix_streamed=verify_numerical_evidence(ROOT,matrix_path,matrix,before)
    # Reopen the recorded dependencies after regression/matrix execution too.
    if recorded_streamed!=verify_numerical_evidence(ROOT,args.recorded_report,recorded,before):
        raise ValueError("Recorded streamed evidence changed during qualification")
    streamed_evidence={key:value for key,value in (("matrix",matrix_streamed),("recorded",recorded_streamed))
                       if value is not None}
    proof={"kind":"native_numerics_software_acceptance_v1","pass":True,"no_flight":True,
           "native_library":native,"deployment_sources":before,
           "matrix":identity(matrix_path),"recorded":identity(args.recorded_report),
           "streamed_evidence":streamed_evidence,
           "native_tests":identity(out/"native_tests.json"),
           "original_deadline_and_controller_limits_unchanged":True,"hardware_qualified":False,
           "orchestration":"Python/ROS; C++ matrix assembly, QP and candidate gates",
           "scope":"numerical and software admission; full endpoint hold still required"}
    write_new(out/"native_acceptance.json",proof)
    # The existing lifecycle launcher retains its original acceptance contract.
    # All current regression/fixture results below were obtained in this run.
    software={"schema":"ts20_native_numerics_full_regression_v1","no_flight":True,
        "deployment_sources":before,"native_library":prior["native_library"],"oracle":prior["oracle"],
        "operating_condition":prior["operating_condition"],"prior_proof_lineage_only":identity(args.previous_proof),
        "previous_test_results_inherited":False,"native_numerics_acceptance":identity(out/"native_acceptance.json"),
        "package_regression":{**test_report,"report_path":str(out/"package_report.json"),
                              "report_sha256":identity(out/"package_report.json")["sha256"]},
        "software_matrix":{"total":30,"passed":30,"failures":[],
            "max_wall_ms":max(v["native_uninstrumented_ms"] for v in matrix["rows"]),
            "summary_path":str(matrix_path),"summary_sha256":identity(matrix_path)["sha256"],
            "original_script_sha256":prior["software_matrix"]["original_script_sha256"]}}
    write_new(out/"software_acceptance.json",software)
    print(json.dumps({"pass":True,"tests":suite["tests"],"matrix":30,"recorded":recorded["total"],
        "native_acceptance":identity(out/"native_acceptance.json"),
        "software_acceptance":identity(out/"software_acceptance.json")},indent=2))
    return 0


if __name__=="__main__":raise SystemExit(main())
