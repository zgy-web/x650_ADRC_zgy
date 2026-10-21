"""Repeatable local MPC timing exercise, with an explicitly hash-bound model.

The rollout only produces changing measured-state inputs for warm-start timing.
It is the same engineering response model as the controller, so this script
cannot establish independent tracking performance or physical flight stability.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--nodes", type=int, default=15)
    parser.add_argument("--blas-threads", type=int)
    args = parser.parse_args()
    if args.steps < 1 or args.nodes < 2 or (args.blas_threads is not None and args.blas_threads < 1):
        parser.error("Positive steps/BLAS threads and at least two nodes are required")
    if args.output.exists():
        parser.error("Use a new output path; timing evidence is never overwritten")
    # Set only this subprocess's backend variables, before importing NumPy or
    # SciPy. They are requested limits, not evidence every linked backend obeys.
    if args.blas_threads is not None:
        for name in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):
            os.environ[name] = str(args.blas_threads)
    import numpy as np
    import scipy
    package = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(package))
    from x650_wholebody.api import ControlState, EndpointTarget
    from x650_wholebody.model import ArmModel, tip_world
    from x650_wholebody.mpc import MpcController, response_matrices

    model = ArmModel.from_report(args.report)
    measured = ControlState([0.,0.,2.],np.eye(3),[0.]*3,[0.]*3,[0.]*2,[0.]*2,1.)
    center = tip_world(measured.position,measured.rotation,measured.q,model)
    class FigureEight:
        def sample(self,t):
            w = 2*np.pi/12
            return (center+np.array([.25*np.sin(w*t),0.,.10*np.sin(2*w*t)]),
                    np.array([.25*w*np.cos(w*t),0.,.20*w*np.cos(2*w*t)]),
                    np.array([-.25*w*w*np.sin(w*t),0.,-.40*w*w*np.sin(2*w*t)]))
    source = FigureEight()
    # Explicit test gains, not a claim of native PX4 readback or tuned X650 PID.
    controller = MpcController(model,{"MPC_XY_P":1.7,"MPC_Z_P":.9},trajectory=source,
                               config={"nodes":args.nodes})
    controller.reset(measured.position,measured.rotation,measured.q)
    rows = []
    for step in range(args.steps):
        t = step*.02
        command = controller.step(t,.02,measured,EndpointTarget(*source.sample(t)))
        diagnostics = command["diagnostics"]
        rows.append({key:diagnostics[key] for key in (
            "solve_ms","compute_ms","solver_iterations","primal_violation","kkt_stationarity_inf")})
        A,B = response_matrices(.02,controller.base_tau)
        state = A@np.r_[measured.position,measured.velocity,measured.q,measured.dq]+B@np.r_[command["base_velocity_cmd"],command["qd_cmd"]]
        measured = ControlState(state[:3],np.eye(3),state[3:6],[0.]*3,state[6:8],state[8:10],measured.stamp_s+.02)
    summary = {name:{"p50":float(np.percentile([r[name] for r in rows],50)),
                     "p95":float(np.percentile([r[name] for r in rows],95)),
                     "max":float(max(r[name] for r in rows))} for name in rows[0]}
    info = {"kind":"x650_mpc_local_numerical_timing", "steps":args.steps,"variables":args.nodes*5,
            "scope":"Local engineering-model rollout for timing only; not Gazebo/PX4 or independent physical validation",
            "python":platform.python_version(),"numpy":np.__version__,"scipy":scipy.__version__,
            "platform":platform.platform(),"requested_blas_threads":args.blas_threads,
            "backend_environment":{name:os.environ.get(name) for name in ("OPENBLAS_NUM_THREADS","OMP_NUM_THREADS","MKL_NUM_THREADS")},
            "controller_source_sha256":hashlib.sha256((package/"x650_wholebody/mpc.py").read_bytes()).hexdigest(),
            "composition_report_sha256":model.source_report_sha256,"metadata":controller.metadata(),
            "summary":summary,"steps_detail":rows}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    args.output.write_text(json.dumps(info,indent=2)+"\n",encoding="utf-8")
    print(json.dumps({"output":str(args.output),"requested_blas_threads":args.blas_threads,"summary":summary},indent=2))


if __name__ == "__main__":
    main()
