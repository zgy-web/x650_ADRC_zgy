"""Conditional late-warm failure reconstruction with explicit reader ambiguity.

No plant or control parameter changes, no GC policy changes, no flight. Both
pending-packet branches are retained; a historical wall-time cause cannot be
deduced merely because an offline reconstruction is quicker.
"""
import argparse
import csv
import gc
import hashlib
import json
from pathlib import Path
import shutil
import sys
from time import perf_counter_ns, process_time_ns, thread_time_ns

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT/"scripts"), str(ROOT/"control_packages/x650_wholebody_v1")]
from profile_x650_ts20_matrix_fixture import fresh_output, file_identity, plain, loaded_identity, verify_identity


class ReadOnce:
    def __init__(self, rows, path):
        self.rows, self.path = rows, path
    def read(self):
        out, self.rows = self.rows, []
        return out
    def close(self):
        pass


def clocks():
    return np.array([perf_counter_ns(), thread_time_ns(), process_time_ns()], dtype=np.int64)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--fixture", type=Path, required=True)
    ap.add_argument("--fixture-sha256", required=True)
    ap.add_argument("--run-dir", type=Path, required=True)
    ap.add_argument("--output", type=Path, required=True)
    args = ap.parse_args()
    out = fresh_output(args.output)
    if file_identity(args.fixture)["sha256"] != args.fixture_sha256:
        raise ValueError("Failure fixture identity mismatch")
    data = json.loads(args.fixture.read_text())
    cfg = json.loads((args.run_dir/"controller_task.json").read_text())
    meta = json.loads((args.run_dir/"controller_metadata.json").read_text())
    from x650_wholebody.api import ControlState, EndpointTarget
    from x650_wholebody.model import ArmModel
    from x650_wholebody.ts20 import FrozenTs20Model
    from x650_wholebody.ts20_prediction import Ts20PredictionModel
    from x650_wholebody.ts20_runtime import Ts20Runtime
    from x650_wholebody.ts20_mpc import Ts20MpcController
    from x650_wholebody import ts20_mpc, mpc, prediction_chain
    failure = data["failure"]
    state, target = ControlState(**failure["state"]), EndpointTarget(**failure["target"])
    inputs = cfg["ts20_inputs"]
    pms = json.loads(Path(inputs["plant_manifest"]).read_text())["parameters"]
    source = FrozenTs20Model(inputs["source_dir"], json.loads(Path(inputs["source_manifest"]).read_text()))
    model = ArmModel(**meta["physical_profile"])
    mpc_cfg = {**cfg["mpc_config"], "response_model": meta["controller_details"]["allocator"]["response_model"]}
    with (args.run_dir/"ts20_packets.csv").open() as stream:
        bus_origin = float(next(csv.DictReader(stream))["send_s"])
    selected = {key:float(value) for key,value in data["exact_selected_actuator_row"].items()}
    rows = [{k:float(v) for k,v in row.items()} for row in data["causal_actual_state_rows"]]
    packets = [{k:float(v) for k,v in row.items()} for row in data["causal_actual_packet_rows"]]
    successful_dts = []
    with (args.run_dir/"control_execution_trace.jsonl").open() as stream:
        for line in stream:
            row = json.loads(line)
            if row.get("event") == "controller_step_returned" and row.get("phase") == "measure" and row["t_s"] < failure["t_s"]:
                successful_dts.append(row["dt_s"])
    if len(successful_dts) != data["reconstructed_committed_mpc_state"]["step_count"]:
        raise ValueError("Successful-step dt sequence differs from committed step_count")
    expected = failure["exception"]["diagnostics"]["ts20_relinearization_attempts"][0]
    results = []
    identity = None
    for include_pending in (False, True):
        branch = "pending_packet_included" if include_pending else "pending_packet_omitted"
        predictor = Ts20PredictionModel(source.metadata()["motion_model"], kp=pms["kp"], kd=pms["kd"], bus_period_s=pms["bus_period_s"])
        predictor.enable_native(inputs["native_library"], expected_sha256=inputs["native_library_sha256"])
        rt = Ts20Runtime(args.run_dir/"ts20_state.csv", args.run_dir/"ts20_packets.csv", predictor)
        used_packets = [dict(row) for row in packets if include_pending or row["send_s"] <= selected["last_send_s"]+1e-10]
        rt.states = ReadOnce([dict(row) for row in rows], args.run_dir/"ts20_state.csv")
        rt.packets = ReadOnce(used_packets, args.run_dir/"ts20_packets.csv")
        observed, stamp = rt.measured_state(state)
        rt.bus_origin_s = bus_origin
        np.testing.assert_array_equal(observed.q, state.q)
        np.testing.assert_array_equal(observed.dq, state.dq)
        if stamp != state.joint_stamp_s:
            raise ValueError("Selected source row changed")
        context = rt.context(state, failure["t_s"])
        c = Ts20MpcController(model, meta["actual_pid_parameters"], predictor=predictor,
            context_provider=rt.context, trajectory=lambda t:target, config=mpc_cfg)
        c.reset(meta["reference_base_enu_m"], meta["reference_rotation"], [0.,0.])
        c.prime_command_history([(r["control_time_s"],r["command"]) for r in meta["activation_command_history"]], data["origin_s"])
        memory = data["reconstructed_committed_mpc_state"]
        for key, value in memory.items():
            if key in ("_plan", "_plan_h", "previous_command"):
                value = np.array(value, dtype=float)
            elif key == "_joint_history":
                value = [(float(t), np.array(u,dtype=float)) for t,u in value]
            setattr(c, key, value)
        verified_memory = {key: plain(getattr(c,key)) == plain(value) for key,value in memory.items()}
        if not all(verified_memory.values()):
            raise ValueError("Committed memory did not round-trip")
        # State-independent cache only: exact dt sequence/FIFO order. No
        # synthetic optimization or guessed transition-cache state is run.
        for dt in successful_dts:
            c._horizon(dt)
        if identity is None:
            identity = loaded_identity(out, predictor)
            identity["runtime"] = file_identity(sys.modules[Ts20Runtime.__module__].__file__)
            identity["runtime_used"] = True
            identity["runtime_scope"] = "Actual Ts20Runtime context logic over supplied causal audit rows; historical reader offset and pending-packet availability are not serialized"
            identity["late_warm_harness"] = file_identity(__file__)
            shutil.copyfile(__file__,out/"executed_sources/profile_x650_ts20_late_warm.py")
        timings, originals, accepted, gc_events = {}, [], {}, []
        def instrument(obj, name, label=None):
            original = getattr(obj,name)
            def wrapper(*a,**kw):
                start = clocks()
                try:
                    result = original(*a,**kw)
                    if label == "base_mpc_step": accepted["allocation"] = result
                    if name == "_horizon":
                        accepted["horizon_was_cached"] = id(result) in prior_horizon_ids
                    return result
                finally:
                    timings.setdefault(label or name,[]).append(((clocks()-start)/1e6).tolist())
            setattr(obj,name,wrapper); originals.append((obj,name,original))
        prior_horizon_ids = {id(value) for value in c._horizon_cache.values()}
        for obj,names in ((c,["_problem","_horizon","_joint_prediction","_quality","_validate_candidate","_solver_constraint_view"]),
            (predictor,["predict","_future_packets","_rollout_native","candidate_rollout"]),
            (prediction_chain,["endpoint_outputs"])):
            for name in names:instrument(obj,name)
        instrument(mpc.MpcController,"step","base_mpc_step")
        instrument(ts20_mpc,"deepcopy","ts20_deepcopy")
        instrument(mpc,"deepcopy","mpc_deepcopy")
        instrument(rt,"context","runtime_context");c.context_provider=rt.context
        def on_gc(phase, info): gc_events.append((phase,info["generation"],perf_counter_ns(),thread_time_ns(),info.get("collected")))
        gc_before={"count":gc.get_count(),"threshold":gc.get_threshold(),"stats":gc.get_stats()}
        gc.callbacks.append(on_gc)
        start=clocks()
        try:
            returned=c.step(failure["t_s"],failure["dt_s"],state,target)
            status={"returned":True,"diagnostics":returned["diagnostics"]}
        except Exception as exc:
            status={"returned":False,"exception":repr(exc),"diagnostics":getattr(exc,"diagnostics",{})}
        finally:
            total=((clocks()-start)/1e6).tolist()
            gc.callbacks.remove(on_gc)
            for obj,name,original in reversed(originals):setattr(obj,name,original)
        allocation=accepted.get("allocation")
        diag=allocation["diagnostics"] if allocation is not None else status["diagnostics"]
        keys=("q_linearization_error_rad","dq_linearization_error_rad_s")
        projections={key:{"recorded":expected[key],"replayed":diag.get("ts20_candidate_"+key),
            "difference":None if diag.get("ts20_candidate_"+key) is None else diag["ts20_candidate_"+key]-expected[key]} for key in keys}
        report={"branch":branch,"committed_memory_verified":verified_memory,"runtime_context":context,
            "clock_columns":["wall_ms","thread_cpu_ms","process_cpu_ms"],"total":total,
            "parts":{key:{"calls":len(value),"total":np.sum(value,axis=0),"values":value} for key,value in timings.items()},
            "status":status,"historical_projection_comparison":projections,
            "horizon_was_cached":accepted.get("horizon_was_cached"),"cache_entries_before":len(prior_horizon_ids),
            "predictor_transition_cache": "fresh; historical cache values and object layout were not serialized",
            "gc_before":gc_before,"gc_after":{"count":gc.get_count(),"stats":gc.get_stats()},"gc_events":gc_events,
            "allocation_before_outer_deadline":allocation,"claim_limits":data["claim_limits"]}
        results.append(report);rt.close()
        (out/(branch+".json")).write_text(json.dumps(plain(report),indent=2))
    verify_identity(identity)
    summary={"kind":"ts20_late_warm_reconstruction_profile_v1","fixture":file_identity(args.fixture),
        "identity":identity,"results":[{key:row[key] for key in ("branch","total","parts","historical_projection_comparison","horizon_was_cached","gc_events")} for row in results],
        "historical_wall_time_cause_established":False,"full_runtime_context_exact_claimed":False,
        "success_dt_cache_sequence_count":len(successful_dts),"claim_limits":data["claim_limits"]}
    (out/"profile_summary.json").write_text(json.dumps(plain(summary),indent=2))
    print(json.dumps(plain({key:value for key,value in summary.items() if key != "identity"}),indent=2))


if __name__=="__main__":main()
