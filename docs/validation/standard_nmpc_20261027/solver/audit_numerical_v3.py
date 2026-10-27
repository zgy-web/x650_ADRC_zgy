"""Read-only replay audits of already-completed, hash-bound numerical v3."""
import ctypes
import hashlib
import json
import sys
import time
from pathlib import Path

import numpy as np
from scipy.optimize import lsq_linear, nnls

stage = Path(sys.argv[1]).resolve()
out = stage / "numerical_v3"
sys.path.insert(0, str(stage / "source_numeric_v3/control_packages/x650_nmpc_standard_v1/python"))
from x650_nmpc_standard.model import build_model, build_case
from x650_nmpc_standard.solver import CondensedNlp, SolverOptions


def identity(path):
    return dict(path=str(path), bytes=path.stat().st_size,
                sha256=hashlib.sha256(path.read_bytes()).hexdigest())


libraries = []
objects = []
for mode in ("sqp", "rti"):
    code = out / ("generated_" + mode) / "c_generated_code"
    path = code / "libacados_ocp_solver_x650_standard.so"
    lib = ctypes.CDLL(str(path))
    objects.append(lib)
    settings = json.loads((code / "acados_ocp.json").read_text())
    source = code / "acados_solver_x650_standard.c"
    plan_lines = [line.strip() for line in source.read_text().splitlines()
                  if "nlp_solver_plan->nlp_solver =" in line]
    libraries.append(dict(artifact=identity(path), actual_cdll_name=lib._name,
        handle=lib._handle, solver_mode=settings["solver_options"]["nlp_solver_type"],
        generated_json=identity(code / "acados_ocp.json"), plan_source_lines=plan_lines,
        create_symbol_address=ctypes.cast(lib.x650_standard_acados_create_capsule,
                                         ctypes.c_void_p).value,
        dynamics_symbol_address=ctypes.cast(lib.x650_standard_dyn_disc_phi_fun,
                                           ctypes.c_void_p).value))
maps = [line for line in Path("/proc/self/maps").read_text().splitlines()
        if "libacados_ocp_solver_x650_standard.so" in line]
loader = dict(kind="x650_standard_solver_load_isolation_v1",
    passed=ctypes.DEFAULT_MODE == ctypes.RTLD_LOCAL and
        libraries[0]["handle"] != libraries[1]["handle"] and
        libraries[0]["create_symbol_address"] != libraries[1]["create_symbol_address"] and
        libraries[0]["dynamics_symbol_address"] != libraries[1]["dynamics_symbol_address"] and
        [x["solver_mode"] for x in libraries] == ["SQP", "SQP_RTI"],
    default_mode=ctypes.DEFAULT_MODE, rtld_local=ctypes.RTLD_LOCAL,
    libraries=libraries, process_memory_maps=maps,
    scope="Separate simultaneous absolute-path CDLL handles with default RTLD_LOCAL; "
          "generated JSON/source mode binding, distinct entry and dynamics symbols.")
(stage / "numerical_v3_loader_audit.json").write_text(json.dumps(loader, indent=2) + "\n")
print(json.dumps(dict(loader_passed=loader["passed"])), flush=True)

selection = json.loads((stage / "wrench_000_selected.json").read_text())
record = selection["cases"][2]
case = record["case"]
model = build_model(selection["config"])
nlp = CondensedNlp(model, build_case(selection["config"], case))
result_file = out / "case_010/ipopt.json"
result = json.loads(result_file.read_text())
z = np.asarray(result["plan_flat"])
_, g, gradient, jacobian = nlp.evaluate(z)
g, gradient, jacobian = np.asarray(g).ravel(), np.asarray(gradient).ravel(), np.asarray(jacobian)
columns, slacks = [], []
for derivative, slack in ((-jacobian, g - nlp.lower_g),
                           (jacobian, nlp.upper_g - g),
                           (-np.eye(len(z)), z - nlp.lower_z),
                           (np.eye(len(z)), nlp.upper_z - z)):
    chosen = slack <= SolverOptions().active_tolerance
    columns.extend(derivative[chosen]); slacks.extend(slack[chosen])
A, b = np.asarray(columns).T, -gradient
rows = []
for method in ("trf", "bvls"):
    start = time.monotonic()
    fit = lsq_linear(A, b, bounds=(np.zeros(A.shape[1]), np.full(A.shape[1], np.inf)),
                     method=method, tol=1e-12, max_iter=500, lsq_solver="exact")
    rows.append(dict(method=method, success=bool(fit.success), status=int(fit.status),
        message=fit.message, iterations=int(fit.nit), optimality=float(fit.optimality),
        elapsed_ms=(time.monotonic()-start)*1000,
        stationarity_inf=float(np.max(abs(A@fit.x-b))),
        complementarity_inf=float(np.max(abs(np.asarray(slacks)*fit.x))),
        multipliers=fit.x.tolist()))
start = time.monotonic()
x, norm = nnls(A, b, maxiter=500)
rows.append(dict(method="nnls", norm=float(norm), elapsed_ms=(time.monotonic()-start)*1000,
    stationarity_inf=float(np.max(abs(A@x-b))),
    complementarity_inf=float(np.max(abs(np.asarray(slacks)*x))), multipliers=x.tolist()))
diagnostic = dict(kind="x650_standard_multiplier_reconstruction_diagnostic_v1",
    source_result=identity(result_file), graph_sha256=nlp.graph_sha256,
    binding_sha256=nlp.binding_sha256, active_tolerance=SolverOptions().active_tolerance,
    dimensions=list(A.shape), column_norm_range=[float(np.min(np.linalg.norm(A,axis=0))),
                                               float(np.max(np.linalg.norm(A,axis=0)))],
    rows=rows, original_results_modified=False, solver_rerun=False)
(stage / "numerical_v3_multiplier_diagnostic.json").write_text(json.dumps(diagnostic, indent=2)+"\n")
print(json.dumps({"fits": [{k:v for k,v in row.items() if k!="multipliers"} for row in rows]}), flush=True)
