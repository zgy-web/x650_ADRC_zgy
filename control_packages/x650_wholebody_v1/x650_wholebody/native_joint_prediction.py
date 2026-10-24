"""Native-only preparation acceleration for the frozen event model.

Only the native controller uses this adapter, including its batch-packed event
wrapper. The independent reference predictor and physical propagation library
retain their original bytes. Scalar validation, packet quantization, event
propagation and relinearization retain the reference definitions; sliding
sensitivities use the hash-bound C++ kernel. Full predictions are compared with
the independent reference, including candidate envelopes and wire packets.
"""
import ctypes
import math
import numpy as np
from .ts20 import quantize_half_away, POSITION_STEP_RAD, VELOCITY_STEP_RAD_S
from .ts20_prediction import Ts20PredictionModel, _scalar, _finite, _borrow_native_pointer


class NativeJointMatrices(Ts20PredictionModel):
    def __init__(self, reference, kernel):
        if not isinstance(reference, Ts20PredictionModel) or reference._native is None:
            raise ValueError("Native joint matrices require the bound event predictor")
        # Model parameters and the immutable native handle are shared; the
        # transition cache is private to this controller-owned adapter.
        self.__dict__ = reference.__dict__.copy()
        self._transition_cache = dict(reference._transition_cache)
        self._affine_kernel = kernel

    def _rollout_native(self, start, q, dq, packets, times, *, track_extrema=False):
        sends = np.array([p["send_time_s"] for p in packets])
        # Assemble the same held forcing in packet order, with each multiply
        # and addition in the original order, without a NumPy operation per
        # scalar packet field.
        references = np.asarray([p["q_ref"] for p in packets])
        velocities = np.asarray([p["dq_ref"] for p in packets])
        torques = np.asarray([p["tau_ff"] for p in packets])
        return self._rollout_native_arrays(start,q,dq,sends,references,velocities,torques,times,
                                           track_extrema=track_extrema)

    def _rollout_native_arrays(self,start,q,dq,sends,references,velocities,torques,times,
                               *,track_extrema=False,materialize_events=True):
        """Private validated-array propagation; public callers materialize all events.

        Skipping ordinary boundary records never skips native propagation or
        extrema collection. All arrays are owned until the synchronous call
        completes; no reusable scratch storage escapes this method.
        """
        knots = np.ascontiguousarray(sends+self.delay_s)
        boundaries = np.array(sorted(set([start,*times,*knots[(knots>start)&(knots<=times[-1])]])))
        forcing = np.ascontiguousarray(self.kp*references+self.kd*velocities+torques)
        parameters = np.array([self.J, self.b, self.Fc, self.offset, self.delay_s, self.Fs, self.delta, self.vs])
        gains, initial = np.r_[self.kp, self.kd], np.r_[q, dq]
        states = np.empty((len(boundaries), 4)); modes = np.empty((len(boundaries), 2), dtype=np.int32)
        zeros = np.empty(len(boundaries), dtype=np.int32); envelope = np.empty(8)
        extrema = np.empty((4096, 5)); event_count, substeps = ctypes.c_int(), ctypes.c_int()
        error = ctypes.create_string_buffer(512)
        dp, ip = ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int)
        double = lambda a: _borrow_native_pointer(a, dp)
        integer = lambda a: _borrow_native_pointer(a, ip)
        # ctypes.CDLL calls are synchronous. These references include every
        # ndarray passed below, including both integer output arrays.
        owners = (parameters, gains, initial, knots, forcing, boundaries,
                  states, modes, zeros, envelope, extrema)
        code = self._native.ts20_prediction_batch(double(parameters), double(gains), self.max_step_s, start,
            double(initial), double(knots), double(forcing), len(sends), double(boundaries), len(boundaries),
            int(track_extrema), double(states), integer(modes), integer(zeros),
            double(envelope), double(extrema), len(extrema), ctypes.byref(event_count), ctypes.byref(substeps), error, len(error))
        del owners  # Only after the synchronous native return.
        if code:
            raise ValueError("TS20 numerical accelerator rejected propagation: "+error.value.decode(errors="replace"))
        indices = np.searchsorted(boundaries, times)
        if np.any(np.abs(boundaries[indices]-times) > 1e-9):
            raise ValueError("TS20 native query coverage failure")
        # Copy the three event fields as batches. Public rows remain disjoint
        # from one another, from the separate query outputs, and from inputs;
        # mutating a returned event cannot change another public field.
        detailed = [{"time_s": float(row[0]), "joint": int(row[1]),
            "kind": "velocity_zero" if row[2] == 1 else "acceleration_zero", "q": float(row[3]), "dq": float(row[4])}
            for row in extrema[:event_count.value]]
        result = {"time_s": times, "q": states[indices, :2], "dq": states[indices, 2:],
            "modes": modes[indices],
            "envelope": {"q_lower": envelope[:2], "q_upper": envelope[2:4],
                "dq_lower": envelope[4:6], "dq_upper": envelope[6:8], "events": detailed,
                "substeps": substeps.value, "collected": track_extrema,
                "method": "all substep endpoints plus frozen-substep velocity/acceleration roots",
                "continuous_published_model_bound_claimed": False, "max_step_s": self.max_step_s}}
        if materialize_events:
            event_q,event_dq,event_modes = states[:,:2].copy(),states[:,2:].copy(),modes.copy()
            result.update(events=[{"time_s":float(when),"q":position,"dq":velocity,
                "modes":mode,"zero_events":int(zero)}
                for when,position,velocity,mode,zero in zip(boundaries,event_q,event_dq,event_modes,zeros)],
                metadata=self.metadata())
        else:
            result["_boundaries"] = boundaries
        return result


    def _future_packets(self, t, nodes, nominal, limits, r_now, packets, phase,
                        *, sensitivities=True):
        """Generate fixed-bus packets in one quantization call per field."""
        data,P,V = self._future_packet_data(t,nodes,nominal,limits,r_now,
            packets[-1]["send_time_s"] if packets else -np.inf,len(packets),phase,
            sensitivities=sensitivities)
        send_times,q_encoded,v_encoded = data
        torque = np.zeros((len(send_times), 2))
        gains_p = np.broadcast_to(self.kp, (len(send_times), 2)).copy()
        gains_d = np.broadcast_to(self.kd, (len(send_times), 2)).copy()
        for b, send in enumerate(send_times):
            packets.append({"send_time_s": float(send), "q_ref": q_encoded[b], "dq_ref": v_encoded[b],
                            "tau_ff": torque[b], "kp": gains_p[b], "kd": gains_d[b]})
        return packets, P, V

    def _future_packet_data(self,t,nodes,nominal,limits,r_now,last_send,sent_count,phase,
                            *,sensitivities=True):
        """Single bus-grid/integrator/quantizer definition for both representations."""
        n, h = len(nodes)*5, np.diff(np.concatenate(([0.], nodes)))
        relative_starts = np.concatenate(([0.], nodes[:-1]))
        first_bus = phase+math.ceil((t-phase)/self.bus_period_s-1e-9)*self.bus_period_s
        if first_bus <= last_send+1e-10:
            first_bus += self.bus_period_s
        send_times = np.arange(first_bus, t+nodes[-1]+1e-10, self.bus_period_s)
        relative = send_times-t
        active = np.clip(np.searchsorted(relative_starts, relative+1e-10, side="right")-1, 0, len(nodes)-1)
        partial = relative-relative_starts[active]
        integral = np.vstack((np.zeros(2), np.cumsum(h[:, None]*nominal[:, 3:], axis=0)))
        reference = r_now+integral[active]+partial[:, None]*nominal[active, 3:]
        q_encoded = quantize_half_away(reference, POSITION_STEP_RAD)
        v_encoded = quantize_half_away(nominal[active, 3:], VELOCITY_STEP_RAD_S)
        P, V = None, None
        if sensitivities:
            diagonal = np.diag(limits)
            prefix = np.zeros((len(nodes)+1, 2, n))
            for i, step in enumerate(h):
                prefix[i+1] = prefix[i]
                prefix[i+1, :, i*5+3:i*5+5] = step*diagonal
            # Batch storage gives each packet its own disjoint slice while
            # avoiding hundreds of Python allocations in every control tick.
            count = sent_count+len(send_times)
            P = np.zeros((count, 2, n))
            V = np.zeros((count, 2, n))
            rows = np.arange(sent_count, count)
            P[sent_count:] = prefix[active]
            for joint in range(2):
                columns = active*5+3+joint
                P[rows, joint, columns] += partial*limits[joint]
                V[rows, joint, columns] = limits[joint]
        return (send_times,q_encoded,v_encoded),P,V

    @staticmethod
    def _packet_arrays(packets):
        return (np.array([p["send_time_s"] for p in packets]),
                *[np.asarray([p[field] for p in packets]).reshape(-1,2)
                  for field in ("q_ref","dq_ref","tau_ff")])

    def _future_packed(self,t,nodes,nominal,limits,r_now,sent,phase,*,sensitivities=True):
        sends,positions,velocities,torques = sent
        data,P,V = self._future_packet_data(t,nodes,nominal,limits,r_now,
            sends[-1] if len(sends) else -np.inf,len(sends),phase,sensitivities=sensitivities)
        future,q,dq = data
        packets = (np.concatenate((sends,future)),np.concatenate((positions,q)),
                   np.concatenate((velocities,dq)),np.concatenate((torques,np.zeros((len(future),2)))))
        return packets,P,V

    def predict(self, t, node_times, intervals, state, history, velocity_limits, nominal,
                *, reference_state, packet_history, bus_origin_s=0., exact_local=False):
        return self._predict(t,node_times,intervals,state,history,velocity_limits,nominal,
            reference_state=reference_state,packet_history=packet_history,
            bus_origin_s=bus_origin_s,exact_local=exact_local,packed=False)

    def _predict_packed(self,t,node_times,intervals,state,history,velocity_limits,nominal,
                        *,reference_state,packet_history,bus_origin_s=0.,exact_local=False):
        # The rare exact-local relinearization continues through the complete
        # existing representation and event finite-difference implementation.
        return self._predict(t,node_times,intervals,state,history,velocity_limits,nominal,
            reference_state=reference_state,packet_history=packet_history,
            bus_origin_s=bus_origin_s,exact_local=exact_local,packed=not exact_local)

    def _predict(self, t, node_times, intervals, state, history, velocity_limits, nominal,
                 *, reference_state, packet_history, bus_origin_s, exact_local, packed):
        """Return legacy-chain compatible affine q/dq nodes/events and r state.

        All command/packet/reference times are task-relative. state timestamps
        may be absolute, their difference maps the measured q/dq to t-age.
        ``nominal`` is N-by-5 in physical velocity units. Q/D columns multiply
        z=flatten(plan)/tile(v_limits_5); base columns are exactly zero.
        Existing command history must contain only commands actually sent.
        """
        t, phase = _scalar(t, "task time"), _scalar(bus_origin_s, "bus origin")
        nodes, h = _finite(node_times, "node times"), _finite(intervals, "intervals")
        limits = _finite(velocity_limits, "joint velocity limits", (2,))
        if (nodes.ndim != 1 or h.shape != nodes.shape or not 1 <= len(nodes) <= 60
                or np.any(h <= 0) or np.any(limits <= 0)
                or not np.allclose(nodes, np.cumsum(h), rtol=0, atol=1e-9)
                or nodes[-1] > 2.):
            raise ValueError("Positive complete horizon <=2 s and joint limits required")
        nominal = _finite(nominal, "nominal plan", (len(nodes), 5))
        if np.any(np.abs(nominal[:, 3:]) > limits+1e-10):
            raise ValueError("Nominal joint plan exceeds velocity limits")
        now = _scalar(state.control_time_s, "control source time")
        stamp = _scalar(state.joint_stamp_s, "joint source time")
        age = now-stamp
        if not 0 <= age <= .25+1e-10:
            raise ValueError("Joint source timestamp is future or stale")
        start = t-age
        q0, dq0 = _finite(state.q, "measured q", (2,)), _finite(state.dq, "measured dq", (2,))
        checked = [(_scalar(when, "history time"), _finite(u, "history command", (2,))) for when, u in history]
        if any(when > t+1e-10 or np.any(np.abs(u) > limits+1e-10) for when, u in checked):
            raise ValueError("History contains future or out-of-range unsent commands")
        r_now = self._reference(t, reference_state, checked)
        packets = [self._packet(p) for p in packet_history]
        if any(p["send_time_s"] > t+1e-10 for p in packets):
            raise ValueError("Actually sent MIT history contains a future packet")
        n = len(nodes)*5
        z_nom = nominal.reshape(-1)/np.tile(np.concatenate((np.ones(3), limits)), len(nodes))
        # At t, an already sent packet cannot be rewritten by this new solve.
        sent_packets = packets.copy()
        if packed:
            sent_arrays = self._packet_arrays(packets)
            packet_arrays,packet_sensitivities,velocity_sensitivities = self._future_packed(
                t,nodes,nominal,limits,r_now,sent_arrays,phase)
            actual_sends = packet_arrays[0]
        else:
            packets, packet_sensitivities, velocity_sensitivities = self._future_packets(
                t, nodes, nominal, limits, r_now, packets, phase)
            actual_sends = np.array([p["send_time_s"] for p in packets])
        queries = np.array(sorted(set([t, *[t+x for x in nodes],
                            *[send+self.delay_s for send in actual_sends
                              if t <= send+self.delay_s <= t+nodes[-1]]])))
        # Only external actual packets need revalidation; the future bus
        # packets are produced by our already validated plan/adapter here.
        if packed:
            # Identical fixed-bus/coverage conditions to _check_packet_clock;
            # packet field and gain validation has already run above.
            if (not len(actual_sends) or np.any(np.diff(actual_sends)<=0)
                    or actual_sends[0]+self.delay_s>start+1e-10
                    or np.any(np.abs((actual_sends-actual_sends[0])/self.bus_period_s
                                     -np.rint((actual_sends-actual_sends[0])/self.bus_period_s))>1e-7)):
                raise ValueError("Ordered fixed-grid MIT history must cover the initial delayed packet")
            nominal_run = self._rollout_native_arrays(start,q0.copy(),dq0.copy(),*packet_arrays,
                                                      queries,materialize_events=False)
        else:
            self._check_packet_clock(packets, start)
            nominal_run = self._rollout_known(start, q0.copy(), dq0.copy(), packets, queries)
        # Sensitivity recurrence follows the same actual packet timing. Internal
        # PD acts continuously, unlike the independently held q_ref/dq_ref.
        sends = actual_sends+self.delay_s
        boundaries = nominal_run["_boundaries"] if packed else sorted(set(
            [start,*queries,*sends[(sends>start)&(sends<=queries[-1])]]))
        if exact_local:
            sensitivities = self._event_jacobian(start, q0, dq0, packets, queries,
                                                 packet_sensitivities, velocity_sensitivities)
        elif self._affine_kernel is not None:
            sensitivities = self._affine_kernel.joint_sensitivities(
                self,start,boundaries,queries,sends,packet_sensitivities,velocity_sensitivities)
        else:
            S = np.zeros((2, 2, n)); cursor = start
            index = max(0, int(np.searchsorted(sends, start+1e-11, side="right")-1))
            sensitivities = []
            for end in boundaries:
                duration = end-cursor
                if duration > 0:
                    A, B = self._transition(duration)
                    force_sensitivity = self.kp[:, None]*packet_sensitivities[index]+self.kd[:, None]*velocity_sensitivities[index]
                    S = np.einsum("jik,jkn->jin", A, S)+B[:, :, None]*force_sensitivity[:, None, :]
                if len(sensitivities) < len(queries) and abs(queries[len(sensitivities)]-end) < 1e-9:
                    sensitivities.append(S.copy())
                cursor = end
                index = max(0, int(np.searchsorted(sends, end+1e-11, side="right")-1))
        sensitivities = np.asarray(sensitivities)
        Q, D = sensitivities[:, :, 0], sensitivities[:, :, 1]
        q_affine, dq_affine = nominal_run["q"]-Q@z_nom, nominal_run["dq"]-D@z_nom
        relative_times = queries-t
        # Preserve the reference's first record satisfying the exact floating
        # subtraction/tolerance expression, using a batched comparison.
        lookup = np.concatenate(([0.], nodes))
        matches = np.abs(relative_times[:,None]-lookup[None,:])<=1e-9
        if not np.all(np.any(matches,axis=0)):
            raise ValueError("Joint prediction query coverage")
        indices = np.argmax(matches,axis=0)
        context = {"t":t,"node_times":nodes.copy(),"initial_time_s":start,
                   "initial_q":q0.copy(),"initial_dq":dq0.copy(),"sent_packets":sent_packets,
                   "limits":limits.copy(),"r_current":r_now.copy(),"bus_origin_s":phase}
        if packed:
            context["_sent_arrays"] = sent_arrays
            joint_rows = np.empty((len(nodes),4+4*n))
            selected = indices[1:]
            joint_rows[:,:2],joint_rows[:,2:4] = q_affine[selected],dq_affine[selected]
            joint_rows[:,4:4+2*n] = Q[selected].reshape(len(nodes),2*n)
            joint_rows[:,4+2*n:] = D[selected].reshape(len(nodes),2*n)
            reference_rows = np.empty((len(nodes),2+2*n))
        else:
            records = [(float(when),q,dq,qj.copy(),dj.copy()) for when,q,dq,qj,dj in
                       zip(relative_times,q_affine,dq_affine,Q,D)]
        current_Q,current_D = (Q[indices[0]].copy(),D[indices[0]].copy()) if packed else (
            records[indices[0]][3],records[indices[0]][4])
        # Add analytic continuous adapter r constraints at every horizon node;
        # endpoints suffice for the piecewise-linear unquantized reference.
        references = []
        diagonal = np.diag(limits)
        R, r = np.zeros((2, n)), r_now.copy()
        for i, elapsed in enumerate(nodes):
            # Each prefix has the same ordered additions as the reference's
            # inner loop; reuse it instead of rebuilding the first i terms.
            R[:, i*5+3:i*5+5] = h[i]*diagonal
            r += h[i]*nominal[i, 3:]
            if packed:
                reference_rows[i,:2],reference_rows[i,2:] = r-R@z_nom,R.ravel()
            else:
                references.append((float(elapsed), r-R@z_nom, R.copy()))
        result = {"q_current":q_affine[indices[0]]+current_Q@z_nom,
                "dq_current":dq_affine[indices[0]]+current_D@z_nom,
                "r_current": r_now, "reference_nodes": references,
                "measurement_age_s": age, "nominal_z": z_nom,
                "nominal_q": nominal_run["q"], "nominal_dq": nominal_run["dq"],
                "query_times_s": queries,
                "nominal_modes": nominal_run["modes"],
                "_context":context,
                "metadata": {**self.metadata(), "event_local_jacobian": bool(exact_local),
                    "event_fd_normalized_step": .0001 if exact_local else None}}
        if packed:
            result.update(_native_packed=True,native_joint_rows=joint_rows,
                          native_reference_rows=reference_rows)
        else:
            result.update(nodes=[records[index] for index in indices[1:]],events=records,
                          nominal_packets=packets)
        return result

    def _candidate_packed(self,prediction,candidate):
        """Private controller check; all extrema and safety-envelope data retained."""
        if not prediction.get("_native_packed"):
            return self.candidate_rollout(prediction,candidate)
        context = prediction["_context"]
        t,nodes,limits = context["t"],context["node_times"],context["limits"]
        candidate = _finite(candidate,"candidate plan",(len(nodes),5))
        if np.any(np.abs(candidate[:,3:])>limits+1e-10):
            raise ValueError("Candidate joint plan exceeds velocity limits")
        packets,_,_ = self._future_packed(t,nodes,candidate,limits,context["r_current"],
            context["_sent_arrays"],context["bus_origin_s"],sensitivities=False)
        out = self._rollout_native_arrays(context["initial_time_s"],context["initial_q"].copy(),
            context["initial_dq"].copy(),*packets,prediction["query_times_s"],
            track_extrema=True,materialize_events=False)
        out["reference_nodes"] = context["r_current"]+np.cumsum(
            np.diff(np.r_[0.,nodes])[:,None]*candidate[:,3:],axis=0)
        return out
