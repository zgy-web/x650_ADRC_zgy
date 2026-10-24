"""Hash-bound C++ numerical kernels; no controller-state mutation or dispatch.

Python retains observation/packet preparation and lifecycle. The C ABI owns
matrix assembly, the convex QP and its full-row KKT check, and candidate gates.
All arrays passed to ctypes remain owned by this stack for the entire call.
"""
from __future__ import annotations
import ctypes as ct
import hashlib
from pathlib import Path
import numpy as np
from .model import rotation_matrix, vector
from .ts20_prediction import _borrow_native_pointer


def array(value, shape=None):
    out = np.ascontiguousarray(value, dtype=np.float64)
    if (shape is not None and out.shape != shape) or not np.isfinite(out).all():
        raise ValueError("Native array shape or finite-value contract")
    return out


class NativeOptimization:
    def __init__(self, path, expected_sha256):
        path = Path(path).resolve(strict=True)
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if not isinstance(expected_sha256, str) or digest != expected_sha256:
            raise ValueError("Native optimization library hash differs")
        self.receipt = {"path": str(path), "sha256": digest, "abi": 1}
        self.lib = ct.CDLL(str(path))
        p = ct.POINTER(ct.c_double)
        i = ct.c_int
        ip = ct.POINTER(ct.c_int)
        text = ct.POINTER(ct.c_char)
        signatures = {
            "x650_joint_sensitivity": [i,i,i,p,p,ip,p,text,i],
            "x650_assemble": [i,p,i,p,i,ip,text,i],
            "x650_solve": [i,i,p,p,p,p,p,i,p,p,ip,text,i],
            "x650_endpoint": [i,p,p,p,p,p,p,text,i],
            "x650_candidate": [i,i,p,p,p,p,p,p,p,p,p,text,i],
        }
        self.lib.x650_optimization_abi.restype = i
        if self.lib.x650_optimization_abi() != 1:
            raise ValueError("Native optimization ABI mismatch")
        for name, args in signatures.items():
            function = getattr(self.lib, name)
            function.argtypes, function.restype = args, i

    def call(self, name, *args):
        error = ct.create_string_buffer(512)
        # NumPy 1.21 ndpointer.from_param calls data_as(), which can create
        # ctypes ownership cycles. Borrow integer-address pointers instead;
        # args owns even temporary arrays until the synchronous call returns.
        pointers = tuple(_borrow_native_pointer(v,ct.POINTER(ct.c_int) if v.dtype==np.int32
                        else ct.POINTER(ct.c_double)) if isinstance(v,np.ndarray) else v for v in args)
        if getattr(self.lib, name)(*pointers, error, len(error)):
            raise ValueError(name+": "+error.value.decode(errors="replace"))

    @staticmethod
    def geometry(model):
        return array([*model.lengths, *model.angle_offsets, *model.mount_body,
                      *model.elbow_offset_body_m], (10,))

    @staticmethod
    def response(controller):
        p = controller.response_model["parameters"]
        return array([p["x"]["wn_rad_s"], p["x"]["zeta"], p["y"]["wn_rad_s"],
                      p["y"]["zeta"], p["z"]["tau_s"]], (5,))

    def joint_sensitivities(self, predictor, start, boundaries, queries, sends, P, V):
        """Same cached transitions/packet indices as the reference recurrence."""
        bounds, queries, sends = array(boundaries), array(queries), array(sends)
        if (bounds.ndim != 1 or queries.ndim != 1 or sends.ndim != 1
                or len(bounds)==0 or len(queries)==0 or len(sends)==0
                or np.any(np.diff(bounds)<=0) or np.any(np.diff(queries)<=0)
                or np.any(np.diff(sends)<=0) or bounds[0]<start):
            raise ValueError("Joint sensitivity ordered time contract")
        P,V = array(P),array(V)
        if P.ndim!=3 or P.shape!=V.shape or P.shape[:2]!=(len(sends),2):
            raise ValueError("Joint packet sensitivity shape")
        n=P.shape[2]
        durations=np.diff(np.concatenate(([start],bounds)))
        transitions=np.empty((len(bounds),2,2,3))
        unchanged=(np.broadcast_to(np.eye(2),(2,2,2)),np.zeros((2,2)))
        blocks=[predictor._transition(duration) if duration>0 else unchanged
                for duration in durations]
        transitions[:,:,:,:2]=np.asarray([block[0] for block in blocks])
        transitions[:,:,:,2]=np.asarray([block[1] for block in blocks])
        # A held packet changes only after reaching its application boundary.
        starts=np.concatenate(([start],bounds[:-1]))
        indices=np.maximum(0,np.searchsorted(sends,starts+1e-11,side="right")-1)
        forces = predictor.kp[None,:,None]*P+predictor.kd[None,:,None]*V
        query_index=np.searchsorted(bounds,queries).astype(np.int32)
        if np.any(query_index>=len(bounds)) or np.any(abs(bounds[query_index]-queries)>1e-9):
            raise ValueError("Joint query coverage")
        output=np.empty((len(queries),2,2,n))
        self.call("x650_joint_sensitivity",n,len(bounds),len(queries),
                  transitions,array(forces[indices]),query_index,output)
        if not np.isfinite(output).all():
            raise ValueError("Nonfinite joint sensitivity output")
        return output

    def assemble(self, controller, t, dt, state, target):
        c = controller
        N, n = c.nodes, 5*c.nodes
        if state.control_time_s is None or state.joint_stamp_s is None:
            raise ValueError("Prediction chain requires measured source timestamps")
        origin = state.control_time_s-t
        if c._chain_clock_origin is not None and abs(origin-c._chain_clock_origin) > 1e-8:
            raise ValueError("Prediction-chain task/control time origin changed")
        h = np.full(N,c.node_dt); h[0] = dt
        times = np.cumsum(h)
        warm = c._warm_start(t, h)
        prediction = c._joint_prediction(t, times, h, state, warm)
        targets = [c._target(t+x, target, x) for x in times]
        age = prediction["measurement_age_s"]
        radius = min(c.actuator_predictor.trust_radius_normalized, .02)
        recent = [u for when, u in c._actuator_context["command_history"]
                  if when >= t-.05-age-c.max_dt]
        if len(prediction["nodes"]) != N or len(prediction["reference_nodes"]) != N:
            raise ValueError("Native joint prediction must cover every horizon node")
        # The packed ABI shape is fixed. Direct slices avoid per-node np.r_
        # dtype inference and temporary concatenations, retaining exact order.
        joints = np.empty((N,4+4*n))
        for row,(_,q,dq,Q,D) in zip(joints,prediction["nodes"]):
            row[:2],row[2:4],row[4:4+2*n],row[4+2*n:] = q,dq,Q.ravel(),D.ravel()
        references = np.empty((N,2+2*n))
        for row,(_,r,R) in zip(references,prediction["reference_nodes"]):
            row[:2],row[2:] = r,R.ravel()
        x0 = np.concatenate((state.position,state.velocity,prediction["q_current"],
                   prediction["dq_current"],vector(state.acceleration_enu,3,"acceleration")[:2]))
        pieces = [[dt,c.node_dt,c.max_dt,age,radius],
                  [c.weights[k] for k in ("tip_position","tip_velocity","base_position",
                                         "joint_posture","input_nominal","input_delta")],
                  c.velocity_limit,c.acceleration_limit,c.previous_command,c.q_lower,c.q_upper,
                  x0,rotation_matrix(state.rotation),state.omega_body,state.acceleration_enu,
                  self.response(c),self.geometry(c.model),c.p0,c.R0,c.q0,[c.arm_share],
                  c.model.joint_lower,c.model.joint_upper,state.dq,prediction["q_current"],
                  prediction["dq_current"],[len(recent)],recent,warm,
                  [[*v.position,*v.velocity] for v in targets],joints,references]
        packed = array(np.concatenate([np.asarray(x,dtype=float).ravel() for x in pieces]))
        # Upper bound, not actual output length. Native writer checks capacity.
        output = np.empty(21*N*(n+1)+n*n+n+(15*N+5)*(n+2)+N*(53+12*n)+17)
        shape = np.zeros(3,dtype=np.int32)
        self.call("x650_assemble",N,packed,len(packed),output,len(output),shape)
        ar,cr,used = map(int,shape)
        at = 0
        def take(*dims):
            nonlocal at
            size = int(np.prod(dims)); result = output[at:at+size].reshape(dims).copy(); at += size
            return result
        out = {"A":take(ar,n),"b":take(ar),"H":take(n,n),"g":take(n),
               "C":take(cr,n),"lo":take(cr),"hi":take(cr),"offsets":take(N,12),
               "jacobians":take(N,12,n),"linearization":take(N,12),"rotations":take(N,3,3)}
        context = take(N,20)
        out.update(first_lower=take(5),first_upper=take(5))
        guard_lo,guard_hi,residual = take(2),take(2),take(3)
        if at != used or not np.isfinite(output[:used]).all():
            raise ValueError("Native assembly output contract")
        scale = np.tile(c.velocity_limit,N)
        out.update(scale=scale,warm=warm.ravel()/scale,h=h,initial_state=x0,
                   targets=targets,identity=np.eye(n),joint_prediction=prediction,
                   native_context=context,
                   chain_context={"yaw":context[:,0],"yaw_rate":context[:,1],
                       "residual_R":context[:,2:11].reshape(N,3,3),
                       "residual_Rdot":context[:,11:20].reshape(N,3,3),
                       "initial_residual_rotvec":residual,"gravity":9.8},
                   joint_guard={"q_guard_lower":guard_lo,"q_guard_upper":guard_hi,
                       "guard_interval_s":.05+age,"trust_radius_normalized":radius,
                       "pending_guard_is_robust_proof":False,
                       "actual_continuous_bounds_checked_before_publish":True})
        return out

    def solve(self, problem, max_iterations):
        n,m = len(problem["g"]),len(problem["lo"])
        if type(max_iterations) is not int or not 1 <= max_iterations <= 10000:
            raise ValueError("Native QP iteration contract")
        inputs = [array(problem[key], shape) for key,shape in
                  (("H",(n,n)),("g",(n,)),("C",(m,n)),("lo",(m,)),("hi",(m,)))]
        z,q,mode = np.empty(n),np.empty(4),np.zeros(1,dtype=np.int32)
        self.call("x650_solve",n,m,*inputs,max_iterations,z,q,mode)
        quality = dict(zip(("primal_violation","kkt_stationarity_inf",
                            "kkt_complementarity_inf","active_constraints"),q.tolist()))
        quality["active_constraints"] = int(quality["active_constraints"])
        return z,quality,bool(mode[0] == 0)

    def endpoint(self, c, problem, predicted, plan):
        N = c.nodes
        output = np.empty((N,15))
        self.call("x650_endpoint",N,array(predicted,(N,12)),array(plan,(N,5)),
                  array(problem["native_context"],(N,20)),self.geometry(c.model),self.response(c),output)
        if not np.isfinite(output).all():
            raise ValueError("Native endpoint output contract")
        return {"position":output[:,:3],"velocity":output[:,3:6],"rotations":output[:,6:].reshape(N,3,3)}

    def candidate(self, c, predicted, q, dq, actual):
        env = actual.get("envelope")
        if not env or not env.get("collected"):
            raise ValueError("TS20 candidate lacks full-substep safety envelope")
        envelope = array([*env["q_lower"],*env["q_upper"],*env["dq_lower"],*env["dq_upper"]],(8,))
        N = c.nodes; result = np.empty(4)
        self.call("x650_candidate",N,N,array(predicted,(N,12)),array(q,(N,2)),array(dq,(N,2)),
                  array(actual["reference_nodes"],(N,2)),envelope,array(c.q_lower,(2,)),
                  array(c.q_upper,(2,)),array(c.velocity_limit[3:],(2,)),result)
        return float(result[0]),float(result[1]),bool(result[2]),bool(result[3])
