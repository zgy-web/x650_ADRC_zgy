"""Conditional TS20 interface-state prediction for upper-level velocity MPC.

The mechanical state is q/dq, the command adapter additionally owns an
*unquantized* integrated position reference r.  Actual MIT packets are sampled
on a fixed bus grid and delayed once by the published combined interface delay.
These are normalized interface coefficients, never URDF inertias or shaft Nm.

Nominal propagation uses event stiction/release and exponential-midpoint
Stribeck substeps.  The QP uses a frozen-friction sliding-branch
sensitivity, including while nominal motion is stuck.  This is deliberately a
surrogate (not the derivative of a discontinuous quantizer/stiction map): it
preserves the ability to plan breakaway.  A trust region and nonlinear plan
validation are mandatory at the caller.  This module does no device I/O.
"""
from __future__ import annotations

from dataclasses import dataclass
import ctypes
import hashlib
import math
from pathlib import Path
import re

import numpy as np

from .ts20 import POSITION_STEP_RAD, VELOCITY_STEP_RAD_S, quantize_half_away


KIND = "ts20_event_mit_reference_prediction_v1"


def _finite(value, name, shape=None):
    raw = np.asarray(value)
    if raw.dtype.kind not in "fiu" or not np.isfinite(raw).all():
        raise ValueError(name + " must be finite real numbers")
    out = np.asarray(raw, dtype=float)
    if shape is not None and out.shape != shape:
        raise ValueError(name + " has invalid shape")
    return out.copy()


def _scalar(value, name):
    return float(_finite(value, name, ()))


def _two(value, name):
    out = _finite(value, name)
    if out.shape == ():
        out = np.repeat(out, 2)
    if out.shape != (2,):
        raise ValueError(name + " must be scalar or two-joint values")
    return out


@dataclass(frozen=True)
class ReferenceState:
    """r at task-relative time_s; r is never reset to each measured q."""
    time_s: float
    r: np.ndarray


class Ts20PredictionModel:
    """Frozen TS20-50 model with explicit timing, gain and discretization.

    ``motion_model`` is the hash-pinned loader's metadata()['motion_model'].
    Supply/load qualifications remain false even when the hardware is 24 V.
    """
    def __init__(self, motion_model, *, kp=6., kd=3276/32768,
                 bus_period_s=.005, max_step_s=.001,
                 trust_radius_normalized=.15):
        if (not isinstance(motion_model, dict)
                or motion_model.get("motor") != "TS20-50"
                or motion_model.get("schema") != "ts20_public_motion_v1"
                or motion_model.get("status") != "frozen_conditional_interface_model"):
            raise ValueError("Frozen conditional TS20-50 motion metadata required")
        fixed, friction = motion_model["fixed_parameters"], motion_model["friction_parameters"]
        self.J = _scalar(fixed["J_total_over_ka"], "normalized J")
        self.b = _scalar(fixed["b_over_ka"], "normalized b")
        self.Fc = _scalar(fixed["Fc_over_ka"], "normalized Fc")
        self.offset = _scalar(fixed["offset_over_ka"], "normalized offset")
        self.delay_s = _scalar(fixed["combined_interface_delay_s"], "combined delay")
        self.Fs = _scalar(friction["static_threshold"], "static threshold")
        self.delta = _scalar(friction["delta_static"], "Stribeck delta")
        self.vs = _scalar(friction["stribeck_velocity"], "Stribeck velocity")
        self.kp, self.kd = _two(kp, "kp"), _two(kd, "kd")
        self.bus_period_s = _scalar(bus_period_s, "bus period")
        self.max_step_s = _scalar(max_step_s, "integration maximum step")
        self.trust_radius_normalized = _scalar(trust_radius_normalized, "trust radius")
        if (self.J <= 0 or min(self.b, self.Fc, self.delay_s, self.delta) < 0
                or self.vs <= 0 or self.Fs < self.Fc+self.delta
                or np.any(self.kp < 0) or np.any(self.kd < 0)
                or not .001 <= self.bus_period_s <= .01
                or not 1e-5 <= self.max_step_s <= .001
                or not 0 < self.trust_radius_normalized <= .25
                or self.delay_s > .25):
            raise ValueError("Invalid conditional TS20 model, timing, gains or trust region")
        self._transition_cache = {}
        self._native = None
        self._native_identity = None

    def enable_native(self, library_path, *, expected_sha256):
        """Load a caller-built, hash-pinned numerical accelerator explicitly.

        No compiler, downloader or implicit fallback runs here. A caller that
        requires real-time native execution must fail if this method fails.
        """
        if not isinstance(expected_sha256, str) or re.fullmatch(r"[0-9a-f]{64}", expected_sha256) is None:
            raise ValueError("Explicit lowercase native library SHA-256 required")
        path = Path(library_path).resolve(strict=True)
        if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected_sha256:
            raise ValueError("TS20 numerical accelerator identity mismatch")
        library = ctypes.CDLL(str(path))
        library.ts20_prediction_abi.restype = ctypes.c_int
        if library.ts20_prediction_abi() != 1:
            raise ValueError("Unsupported TS20 numerical accelerator ABI")
        dp, ip = ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int)
        function = library.ts20_prediction_batch
        function.restype = ctypes.c_int
        function.argtypes = [dp, dp, ctypes.c_double, ctypes.c_double, dp, dp, dp, ctypes.c_int,
            dp, ctypes.c_int, ctypes.c_int, dp, ip, ip, dp, dp, ctypes.c_int, ip, ip, ctypes.c_char_p, ctypes.c_int]
        self._native, self._native_identity = library, {"path": str(path), "sha256": expected_sha256, "abi": 1}
        return self._native_identity.copy()

    def metadata(self):
        return {"kind": KIND, "bus_period_s": self.bus_period_s,
                "combined_interface_delay_s": self.delay_s,
                "max_step_s": self.max_step_s,
                "trust_radius_normalized": self.trust_radius_normalized,
                "linearization": "frozen-friction sliding sensitivity calibrated at event nominal",
                "quantization_derivative": "unquantized surrogate; nominal packets quantized",
                "q_ref_quantization_half_step_rad": POSITION_STEP_RAD/2,
                "dq_ref_quantization_half_step_rad_s": VELOCITY_STEP_RAD_S/2,
                "nonlinear_candidate_check_required": True,
                "native_accelerator": None if self._native_identity is None else self._native_identity.copy(),
                "hardware_accessed": False, "supply_24v_qualified": False,
                "aerial_arm_load_qualified": False}

    def _transition(self, h):
        """Exact damped-oscillator exp(A h), integral exp(A s) B ds."""
        key = round(float(h), 14)
        if key in self._transition_cache:
            return self._transition_cache[key]
        A, B = np.empty((2, 2, 2)), np.empty((2, 2))
        for j in range(2):
            k, damping = self.kp[j]/self.J, (self.b+self.kd[j])/self.J
            half = -damping/2
            discriminant = half*half-k
            if abs(discriminant)*h*h < 1e-14:
                c, s = 1., h
            elif discriminant > 0:
                w = math.sqrt(discriminant)
                c, s = math.cosh(w*h), math.sinh(w*h)/w
            else:
                w = math.sqrt(-discriminant)
                c, s = math.cos(w*h), math.sin(w*h)/w
            A[j] = math.exp(half*h)*np.array([[c-half*s, s], [-k*s, c+half*s]])
            if self.kp[j] > 0:
                B[j] = [(1-A[j, 0, 0])/self.kp[j], -A[j, 1, 0]/self.kp[j]]
            elif damping > 0:
                gain = -math.expm1(-damping*h)/damping
                B[j] = [(h-gain)/(self.b+self.kd[j]), gain/self.J]
            else:
                B[j] = [h*h/(2*self.J), h/self.J]
        if len(self._transition_cache) >= 512:
            self._transition_cache.clear()
        self._transition_cache[key] = (A, B)
        return A, B

    def _packet(self, raw):
        required = {"send_time_s", "q_ref", "dq_ref", "tau_ff", "kp", "kd"}
        if not isinstance(raw, dict) or set(raw) != required:
            raise ValueError("MIT packet must have explicit send time and all five fields")
        packet = {"send_time_s": _scalar(raw["send_time_s"], "packet send time")}
        packet.update({k: _finite(raw[k], "packet "+k, (2,)) for k in required-{"send_time_s"}})
        if not np.array_equal(packet["kp"], self.kp) or not np.array_equal(packet["kd"], self.kd):
            raise ValueError("Packet gains differ from frozen predictor gains")
        return packet

    def advance(self, q, dq, command, duration_s):
        """One already-delayed held packet, no measurement reset or bus logic.

        Streaming consumers must split at the *delayed* packet activation time.
        Returns actual conditional-model q/dq, mode and zero-event count.
        """
        q, dq = _finite(q, "q", (2,)), _finite(dq, "dq", (2,))
        packet = self._packet({"send_time_s": 0., **command})
        duration = _scalar(duration_s, "duration")
        if not 0 <= duration <= 2.:
            raise ValueError("Streaming duration must be between zero and 2 seconds")
        forcing = self.kp*packet["q_ref"]+self.kd*packet["dq_ref"]+packet["tau_ff"]
        return self._advance(q, dq, forcing, duration)

    def _advance(self, q, dq, forcing, duration, *, track_extrema=False):
        """Validated internal path; two scalars avoid repeated tiny matmuls."""
        mode, zero_count = np.zeros(2, dtype=int), 0
        steps = max(1, int(math.ceil(duration/self.max_step_s-1e-12)))
        h = duration/steps
        if not h:
            return {"q": q, "dq": dq, "modes": np.sign(dq).astype(int), "zero_events": 0}
        A, B = self._transition(h)
        half_A, half_B = self._transition(h/2)
        coefficients = [tuple(map(float, A[j].reshape(-1)))+tuple(map(float, B[j])) for j in range(2)]
        qlo, qhi, vlo, vhi = q.copy(), q.copy(), dq.copy(), dq.copy()
        extrema = []
        for j in range(2):
            position, velocity = float(q[j]), float(dq[j])
            kp, force = float(self.kp[j]), float(forcing[j])
            damping = float(self.b+self.kd[j])
            half_a10, half_a11, half_b1 = map(float, (half_A[j, 1, 0], half_A[j, 1, 1], half_B[j, 1]))
            for substep in range(steps):
                remaining, restarts = h, 0
                while remaining > 1e-14:
                    restarts += 1
                    if restarts > 12:
                        raise ValueError("TS20 zero-event iteration budget exceeded")
                    drive = force-kp*position-self.offset
                    if velocity == 0 and abs(drive) <= self.Fs:
                        mode[j] = 0
                        break
                    sign = math.copysign(1., velocity if velocity else drive)
                    friction = self.Fc+self.delta*math.exp(-min(28., abs(velocity)/self.vs)**2)
                    effort = force-self.offset-sign*friction
                    # Exponential midpoint of the nonlinear friction, keeping
                    # the continuous PD dynamics exact. Freezing at the left
                    # endpoint accumulates visible breakaway/stop phase error.
                    if remaining == h:
                        ha10, ha11, hb1 = half_a10, half_a11, half_b1
                    else:
                        ha, hb = self._transition(remaining/2)
                        ha10, ha11, hb1 = map(float, (ha[j, 1, 0], ha[j, 1, 1], hb[j, 1]))
                    midpoint_velocity = ha10*position+ha11*velocity+hb1*effort
                    friction = self.Fc+self.delta*math.exp(-min(28., abs(midpoint_velocity)/self.vs)**2)
                    effort = force-self.offset-sign*friction
                    if remaining == h:
                        a00, a01, a10, a11, b0, b1 = coefficients[j]
                    else:
                        trans, gain = self._transition(remaining)
                        a00, a01, a10, a11 = map(float, trans[j].reshape(-1))
                        b0, b1 = map(float, gain[j])
                    end_q = a00*position+a01*velocity+b0*effort
                    end_v = a10*position+a11*velocity+b1*effort
                    mode[j] = int(sign)
                    accepted = remaining
                    if sign*end_v >= 0:
                        zero = False
                    else:
                        # First exact zero of this frozen-force substep.
                        lo, hi = 0., remaining
                        for _root in range(36):
                            mid = (lo+hi)/2
                            root_A, root_B = self._transition(mid)
                            root_velocity = root_A[j, 1, 0]*position+root_A[j, 1, 1]*velocity+root_B[j, 1]*effort
                            if sign*root_velocity > 0:
                                lo = mid
                            else:
                                hi = mid
                        root_A, root_B = self._transition(hi)
                        end_q = float(root_A[j, 0, 0])*position+float(root_A[j, 0, 1])*velocity+float(root_B[j, 0])*effort
                        end_v, accepted, zero = 0., hi, True
                        zero_count += 1
                    if track_extrema:
                        qlo[j], qhi[j] = min(qlo[j], end_q), max(qhi[j], end_q)
                        vlo[j], vhi[j] = min(vlo[j], end_v), max(vhi[j], end_v)
                        acceleration0 = (effort-kp*position-damping*velocity)/self.J
                        acceleration1 = (effort-kp*end_q-damping*end_v)/self.J
                        if acceleration0*acceleration1 < 0:
                            lo, hi = 0., accepted
                            for _root in range(30):
                                mid = (lo+hi)/2
                                ea, eb = self._transition(mid)
                                eq = float(ea[j, 0, 0])*position+float(ea[j, 0, 1])*velocity+float(eb[j, 0])*effort
                                ev = float(ea[j, 1, 0])*position+float(ea[j, 1, 1])*velocity+float(eb[j, 1])*effort
                                acceleration = (effort-kp*eq-damping*ev)/self.J
                                if acceleration*acceleration0 > 0:
                                    lo = mid
                                else:
                                    hi = mid
                            vlo[j], vhi[j] = min(vlo[j], ev), max(vhi[j], ev)
                            extrema.append({"relative_time_s": substep*h+h-remaining+mid,
                                "joint": j, "kind": "acceleration_zero", "q": eq, "dq": ev})
                        if zero:
                            extrema.append({"relative_time_s": substep*h+h-remaining+accepted,
                                "joint": j, "kind": "velocity_zero", "q": end_q, "dq": 0.})
                    position, velocity = end_q, end_v
                    remaining -= accepted
            q[j], dq[j] = position, velocity
        if not np.isfinite([q, dq]).all() or np.max(np.abs([q, dq])) > 1e6:
            raise ValueError("Conditional TS20 motion diverged")
        return {"q": q, "dq": dq, "modes": mode, "zero_events": zero_count,
                "envelope": {"q_lower": qlo, "q_upper": qhi, "dq_lower": vlo,
                    "dq_upper": vhi, "events": extrema, "substeps": steps,
                    "collected": track_extrema}}

    def rollout(self, initial_time_s, initial_q, initial_dq, packet_history, observation_times):
        """Free event nominal from actual packets, source stamp and first q/dq.

        Packet history must include the active packet at initial_time-delay;
        no unknown prehistory is invented. Packet time/order and fixed bus-grid
        membership are validated; bus origin may be an arbitrary phase.
        """
        start = _scalar(initial_time_s, "initial time")
        times = _finite(observation_times, "observation times")
        if (times.ndim != 1 or not len(times) or len(times) > 20000
                or times[0] < start-1e-10 or np.any(np.diff(times) <= 0)
                or times[-1]-start > 2.):
            raise ValueError("Finite increasing queries within two seconds required")
        packets = [self._packet(p) for p in packet_history]
        self._check_packet_clock(packets, start)
        q, dq = _finite(initial_q, "initial q", (2,)), _finite(initial_dq, "initial dq", (2,))
        return self._rollout_known(start, q, dq, packets, times, track_extrema=True)

    def _check_packet_clock(self, packets, start):
        sends = np.array([p["send_time_s"] for p in packets])
        if (not len(sends) or np.any(np.diff(sends) <= 0)
                or sends[0]+self.delay_s > start+1e-10
                or np.any(np.abs((sends-sends[0])/self.bus_period_s
                                 -np.rint((sends-sends[0])/self.bus_period_s)) > 1e-7)):
            raise ValueError("Ordered fixed-grid MIT history must cover the initial delayed packet")

    def _rollout_known(self, start, q, dq, packets, times, *, track_extrema=False):
        """Internal validated-packet path; never exported as a bypass."""
        if self._native is not None:
            return self._rollout_native(start, q, dq, packets, times, track_extrema=track_extrema)
        sends = np.array([p["send_time_s"] for p in packets])
        knots = sends+self.delay_s
        boundaries = sorted(set([start, *times, *knots[(knots > start) & (knots <= times[-1])]]))
        qs, dqs, modes, events = [], [], [], []
        cursor = start
        qlo, qhi, vlo, vhi = q.copy(), q.copy(), dq.copy(), dq.copy()
        extrema, substeps = [], 0
        index = max(0, int(np.searchsorted(knots, start+1e-11, side="right")-1))
        forcing = [self.kp*p["q_ref"]+self.kd*p["dq_ref"]+p["tau_ff"] for p in packets]
        for end in boundaries:
            previous_cursor = cursor
            result = self._advance(q, dq, forcing[index], end-cursor, track_extrema=track_extrema)
            q, dq, cursor = result["q"], result["dq"], end
            if track_extrema and end > previous_cursor:
                envelope = result["envelope"]
                qlo, qhi = np.minimum(qlo, envelope["q_lower"]), np.maximum(qhi, envelope["q_upper"])
                vlo, vhi = np.minimum(vlo, envelope["dq_lower"]), np.maximum(vhi, envelope["dq_upper"])
                substeps += envelope["substeps"]
                for extremum in envelope["events"]:
                    extrema.append({**extremum, "time_s": previous_cursor+extremum["relative_time_s"]})
            events.append({"time_s": end, "q": q.copy(), "dq": dq.copy(),
                           "modes": result["modes"].copy(), "zero_events": result["zero_events"]})
            if len(qs) < len(times) and abs(times[len(qs)]-end) < 1e-9:
                qs.append(q.copy()); dqs.append(dq.copy()); modes.append(result["modes"].copy())
            index = max(0, int(np.searchsorted(knots, end+1e-11, side="right")-1))
        return {"time_s": times, "q": np.asarray(qs), "dq": np.asarray(dqs),
                "modes": np.asarray(modes), "events": events, "metadata": self.metadata(),
                "envelope": {"q_lower": qlo, "q_upper": qhi, "dq_lower": vlo, "dq_upper": vhi,
                    "events": extrema, "substeps": substeps, "collected": track_extrema,
                    "method": "all substep endpoints plus frozen-substep velocity/acceleration roots",
                    "continuous_published_model_bound_claimed": False,
                    "max_step_s": self.max_step_s}}

    def _rollout_native(self, start, q, dq, packets, times, *, track_extrema=False):
        sends = np.array([p["send_time_s"] for p in packets])
        knots = np.ascontiguousarray(sends+self.delay_s)
        boundaries = np.array(sorted(set([start, *times, *knots[(knots > start) & (knots <= times[-1])]])))
        forcing = np.ascontiguousarray([self.kp*p["q_ref"]+self.kd*p["dq_ref"]+p["tau_ff"] for p in packets])
        parameters = np.array([self.J, self.b, self.Fc, self.offset, self.delay_s, self.Fs, self.delta, self.vs])
        gains, initial = np.r_[self.kp, self.kd], np.r_[q, dq]
        states = np.empty((len(boundaries), 4)); modes = np.empty((len(boundaries), 2), dtype=np.int32)
        zeros = np.empty(len(boundaries), dtype=np.int32); envelope = np.empty(8)
        extrema = np.empty((4096, 5)); event_count, substeps = ctypes.c_int(), ctypes.c_int()
        error = ctypes.create_string_buffer(512)
        dp, ip = ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int)
        double = lambda a: a.ctypes.data_as(dp)
        code = self._native.ts20_prediction_batch(double(parameters), double(gains), self.max_step_s, start,
            double(initial), double(knots), double(forcing), len(packets), double(boundaries), len(boundaries),
            int(track_extrema), double(states), modes.ctypes.data_as(ip), zeros.ctypes.data_as(ip),
            double(envelope), double(extrema), len(extrema), ctypes.byref(event_count), ctypes.byref(substeps), error, len(error))
        if code:
            raise ValueError("TS20 numerical accelerator rejected propagation: "+error.value.decode(errors="replace"))
        indices = np.searchsorted(boundaries, times)
        if np.any(np.abs(boundaries[indices]-times) > 1e-9):
            raise ValueError("TS20 native query coverage failure")
        events = [{"time_s": float(when), "q": value[:2].copy(), "dq": value[2:].copy(),
            "modes": mode.copy(), "zero_events": int(zero)}
            for when, value, mode, zero in zip(boundaries, states, modes, zeros)]
        detailed = [{"time_s": float(row[0]), "joint": int(row[1]),
            "kind": "velocity_zero" if row[2] == 1 else "acceleration_zero", "q": float(row[3]), "dq": float(row[4])}
            for row in extrema[:event_count.value]]
        return {"time_s": times, "q": states[indices, :2], "dq": states[indices, 2:],
            "modes": modes[indices], "events": events, "metadata": self.metadata(),
            "envelope": {"q_lower": envelope[:2], "q_upper": envelope[2:4],
                "dq_lower": envelope[4:6], "dq_upper": envelope[6:8], "events": detailed,
                "substeps": substeps.value, "collected": track_extrema,
                "method": "all substep endpoints plus frozen-substep velocity/acceleration roots",
                "continuous_published_model_bound_claimed": False, "max_step_s": self.max_step_s}}

    def _reference(self, end, anchor, history):
        if isinstance(anchor, ReferenceState):
            at, r = anchor.time_s, anchor.r
        elif isinstance(anchor, dict) and set(anchor) == {"time_s", "r"}:
            at, r = anchor["time_s"], anchor["r"]
        else:
            raise ValueError("Explicit reference_state={time_s,r} required")
        at, r = _scalar(at, "reference source time"), _finite(r, "reference r", (2,))
        if at > end+1e-10:
            raise ValueError("Reference source is in the future")
        records = [(_scalar(when, "command send time"), _finite(value, "velocity command", (2,)))
                   for when, value in history]
        if (not records or records[0][0] > at+1e-10
                or any(records[i][0] >= records[i+1][0] for i in range(len(records)-1))):
            raise ValueError("Sent command history must cover the explicit reference state")
        active = records[0][1]
        for when, value in records:
            if when <= at+1e-10:
                active = value
                continue
            if when > end:
                break
            r += (when-at)*active
            at, active = when, value
        return r+(end-at)*active

    def _future_packets(self, t, nodes, nominal, limits, r_now, packets, phase,
                        *, sensitivities=True):
        """Generate fixed-bus packets in one quantization call per field."""
        n, h = len(nodes)*5, np.diff(np.r_[0., nodes])
        relative_starts = np.r_[0., nodes[:-1]]
        last_send = packets[-1]["send_time_s"] if packets else -np.inf
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
            prefix = np.zeros((len(nodes)+1, 2, n))
            for i, step in enumerate(h):
                prefix[i+1] = prefix[i]
                prefix[i+1, :, i*5+3:i*5+5] = step*np.diag(limits)
            P = [np.zeros((2, n)) for _ in packets]
            V = [np.zeros((2, n)) for _ in packets]
        for b, send in enumerate(send_times):
            i = active[b]
            packets.append({"send_time_s": float(send), "q_ref": q_encoded[b], "dq_ref": v_encoded[b],
                            "tau_ff": np.zeros(2), "kp": self.kp.copy(), "kd": self.kd.copy()})
            if sensitivities:
                p = prefix[i].copy()
                p[:, i*5+3:i*5+5] += partial[b]*np.diag(limits)
                v = np.zeros((2, n)); v[:, i*5+3:i*5+5] = np.diag(limits)
                P.append(p); V.append(v)
        return packets, P, V

    def predict(self, t, node_times, intervals, state, history, velocity_limits, nominal,
                *, reference_state, packet_history, bus_origin_s=0., exact_local=False):
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
        z_nom = nominal.reshape(-1)/np.tile(np.r_[np.ones(3), limits], len(nodes))
        # At t, an already sent packet cannot be rewritten by this new solve.
        sent_packets = packets.copy()
        packets, packet_sensitivities, velocity_sensitivities = self._future_packets(
            t, nodes, nominal, limits, r_now, packets, phase)
        queries = np.array(sorted(set([t, *[t+x for x in nodes],
                            *[p["send_time_s"]+self.delay_s for p in packets
                              if t <= p["send_time_s"]+self.delay_s <= t+nodes[-1]]])))
        # Only external actual packets need revalidation; the future bus
        # packets are produced by our already validated plan/adapter here.
        self._check_packet_clock(packets, start)
        nominal_run = self._rollout_known(start, q0.copy(), dq0.copy(), packets, queries)
        # Sensitivity recurrence follows the same actual packet timing. Internal
        # PD acts continuously, unlike the independently held q_ref/dq_ref.
        sends = np.array([p["send_time_s"] for p in packets])+self.delay_s
        boundaries = sorted(set([start, *queries, *sends[(sends > start) & (sends <= queries[-1])]]))
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
        if exact_local:
            sensitivities = self._event_jacobian(start, q0, dq0, packets, queries,
                                                 packet_sensitivities, velocity_sensitivities)
        records = []
        for when, q, dq, S in zip(queries, nominal_run["q"], nominal_run["dq"], sensitivities):
            Q, D = S[:, 0], S[:, 1]
            records.append((float(when-t), q-Q@z_nom, dq-D@z_nom, Q.copy(), D.copy()))
        def at_time(relative):
            return next(record for record in records if abs(record[0]-relative) <= 1e-9)
        current = at_time(0.)
        # Add analytic continuous adapter r constraints at every horizon node;
        # endpoints suffice for the piecewise-linear unquantized reference.
        references = []
        for i, elapsed in enumerate(nodes):
            R = np.zeros((2, n)); r = r_now.copy()
            for j in range(i+1):
                R[:, j*5+3:j*5+5] = h[j]*np.diag(limits)
                r += h[j]*nominal[j, 3:]
            references.append((float(elapsed), r-R@z_nom, R))
        return {"nodes": [at_time(x) for x in nodes], "events": records,
                "q_current": current[1]+current[3]@z_nom,
                "dq_current": current[2]+current[4]@z_nom,
                "r_current": r_now, "reference_nodes": references,
                "measurement_age_s": age, "nominal_z": z_nom,
                "nominal_q": nominal_run["q"], "nominal_dq": nominal_run["dq"],
                "query_times_s": queries, "nominal_packets": packets,
                "nominal_modes": nominal_run["modes"],
                "_context": {"t": t, "node_times": nodes.copy(), "initial_time_s": start,
                    "initial_q": q0.copy(), "initial_dq": dq0.copy(), "sent_packets": sent_packets,
                    "limits": limits.copy(), "r_current": r_now.copy(), "bus_origin_s": phase},
                "metadata": {**self.metadata(), "event_local_jacobian": bool(exact_local),
                    "event_fd_normalized_step": .0001 if exact_local else None}}

    def _event_jacobian(self, start, q, dq, packets, queries, P, V):
        """Central event-model derivative, no re-quantization of tiny deltas.

        The nominal encoded forcing remains exact. Derivatives perturb the
        continuous forcing around that nominal, so protocol rounding cannot
        turn every column into zero. Static segments retain their true zero
        derivative; this is used only after the breakaway proxy has proposed
        a bounded candidate, never as the sole startup planner.
        """
        if self._native is None:
            raise ValueError("Event-local relinearization requires the explicit native backend")
        sends = np.array([p["send_time_s"] for p in packets])
        knots = np.ascontiguousarray(sends+self.delay_s)
        boundaries = np.array(sorted(set([start, *queries, *knots[(knots > start) & (knots <= queries[-1])]])))
        query_index = np.searchsorted(boundaries, queries)
        forcing = np.ascontiguousarray([self.kp*p["q_ref"]+self.kd*p["dq_ref"]+p["tau_ff"] for p in packets])
        forcing_jacobian = np.array([self.kp[:, None]*p+self.kd[:, None]*v for p, v in zip(P, V)])
        n = forcing_jacobian.shape[2]
        result = np.zeros((len(queries), 2, 2, n))
        parameters = np.array([self.J, self.b, self.Fc, self.offset, self.delay_s, self.Fs, self.delta, self.vs])
        gains, initial = np.r_[self.kp, self.kd], np.r_[q, dq]
        states = np.empty((len(boundaries), 4)); modes = np.empty((len(boundaries), 2), dtype=np.int32)
        zeros = np.empty(len(boundaries), dtype=np.int32); envelope = np.empty(8)
        extrema = np.empty((4096, 5)); event_count, substeps = ctypes.c_int(), ctypes.c_int()
        error = ctypes.create_string_buffer(512)
        dp, ip = ctypes.POINTER(ctypes.c_double), ctypes.POINTER(ctypes.c_int)
        double = lambda a: a.ctypes.data_as(dp)
        epsilon = .0001
        for column in np.flatnonzero(np.any(forcing_jacobian != 0., axis=(0, 1))):
            pair = []
            for sign in (1., -1.):
                perturbed = np.ascontiguousarray(forcing+sign*epsilon*forcing_jacobian[:, :, column])
                code = self._native.ts20_prediction_batch(double(parameters), double(gains), self.max_step_s,
                    start, double(initial), double(knots), double(perturbed), len(packets), double(boundaries),
                    len(boundaries), 0, double(states), modes.ctypes.data_as(ip), zeros.ctypes.data_as(ip),
                    double(envelope), double(extrema), len(extrema), ctypes.byref(event_count),
                    ctypes.byref(substeps), error, len(error))
                if code:
                    raise ValueError("TS20 native event derivative failed: "+error.value.decode(errors="replace"))
                pair.append(states[query_index].copy())
            derivative = (pair[0]-pair[1])/(2*epsilon)
            result[:, :, 0, column] = derivative[:, :2]
            result[:, :, 1, column] = derivative[:, 2:]
        return list(result)

    def candidate_rollout(self, prediction, candidate):
        """Nonlinear candidate check without rebuilding QP sensitivities.

        Uses the same frozen measurement, r and actually sent packets as the
        solve.  It cannot update a running adapter or condition on future q.
        """
        context = prediction["_context"]
        t, nodes, limits = context["t"], context["node_times"], context["limits"]
        candidate = _finite(candidate, "candidate plan", (len(nodes), 5))
        if np.any(np.abs(candidate[:, 3:]) > limits+1e-10):
            raise ValueError("Candidate joint plan exceeds velocity limits")
        packets, _, _ = self._future_packets(t, nodes, candidate, limits,
            context["r_current"], context["sent_packets"].copy(), context["bus_origin_s"], sensitivities=False)
        out = self._rollout_known(context["initial_time_s"], context["initial_q"].copy(),
            context["initial_dq"].copy(), packets, prediction["query_times_s"], track_extrema=True)
        r = context["r_current"]+np.cumsum(np.diff(np.r_[0., nodes])[:, None]*candidate[:, 3:], axis=0)
        out.update(reference_nodes=r, packets=packets)
        return out
