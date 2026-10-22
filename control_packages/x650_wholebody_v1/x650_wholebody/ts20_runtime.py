"""Causal reader for the conditional Gazebo TS20 encoder and MIT bus logs.

Only past complete CSV rows enter control. Native interval-average velocity is
kept for audit; endpoint velocity models the servo's reported motor-speed
feedback. No future q or native end-effector position is read by this adapter.
"""
from __future__ import annotations

import csv
from dataclasses import replace
from pathlib import Path
import numpy as np


class _CsvTail:
    def __init__(self, path):
        self.path = Path(path)
        self.stream, self.header, self.partial = None, None, ""

    def read(self):
        if self.stream is None:
            if not self.path.is_file():
                raise RuntimeError("TS20 plugin log does not exist: "+str(self.path))
            self.stream = self.path.open(encoding="utf-8", newline="")
        data = self.partial+self.stream.read()
        lines = data.split("\n")
        self.partial = lines.pop()
        if self.header is None:
            if not lines:
                return []
            self.header = next(csv.reader([lines.pop(0)]))
        out = []
        for line in lines:
            if not line.strip():
                continue
            fields = next(csv.reader([line]))
            if len(fields) != len(self.header):
                raise RuntimeError("Malformed complete TS20 CSV row")
            row = {k: float(v) for k, v in zip(self.header, fields)}
            if not np.isfinite(list(row.values())).all():
                raise RuntimeError("Nonfinite TS20 plugin observation")
            out.append(row)
        return out

    def close(self):
        if self.stream is not None:
            self.stream.close()


class Ts20Runtime:
    def __init__(self, state_csv, packet_csv, predictor, *, max_age_s=.025,
                 physical_position_tolerance_rad=.0001):
        self.states, self.packets = _CsvTail(state_csv), _CsvTail(packet_csv)
        self.predictor = predictor
        self.max_age_s = float(max_age_s)
        self.physical_position_tolerance_rad = float(physical_position_tolerance_rad)
        self._rows, self._packets = [], []
        self._command_events = []
        self._last_sequences, self._last_u = None, np.zeros(2)
        self._last_row_time, self._last_packet_time = -np.inf, -np.inf
        self._last_packet_sequence = None
        self.bus_origin_s = None
        self.observation_count = 0
        self.max_position_disagreement_rad = 0.
        self._measured_snapshot = None

    def _update(self):
        for row in self.states.read():
            at = row["sim_s"]
            if at <= self._last_row_time:
                raise RuntimeError("TS20 state source clock did not increase")
            if self._last_row_time != -np.inf and at-self._last_row_time > .02000001:
                raise RuntimeError("TS20 state log has a physics interval gap")
            self._last_row_time = at
            self._rows.append(row)
            seq = [int(row[f"upper_sequence{j}"]) for j in (1, 2)]
            if (any(seq[j] != row[f"upper_sequence{j+1}"] or seq[j] < 0 for j in range(2))
                    or (self._last_sequences is not None
                        and any(seq[j] < self._last_sequences[j] for j in range(2)))):
                raise RuntimeError("TS20 consumed-command sequence is invalid or regressed")
            u = np.array([row[f"u{j}"] for j in (1, 2)])
            if self._last_sequences is None:
                self._command_events.append((row["interval_begin_s"], u.copy(), at))
            else:
                changes = sorted(set(row[f"receipt{j+1}_s"] for j in range(2)
                                     if seq[j] != self._last_sequences[j]))
                for when in changes:
                    value = self._last_u.copy()
                    for j in range(2):
                        if seq[j] != self._last_sequences[j] and row[f"receipt{j+1}_s"] <= when:
                            value[j] = u[j]
                    if when < self._command_events[-1][0]-1e-10:
                        raise RuntimeError("TS20 consumed-command clock regressed")
                    if abs(when-self._command_events[-1][0]) < 1e-10:
                        self._command_events[-1] = (when, value.copy(), at)
                    else:
                        self._command_events.append((when, value.copy(), at))
            self._last_sequences, self._last_u = seq, u.copy()
        for row in self.packets.read():
            send = row["send_s"]
            if send <= self._last_packet_time:
                raise RuntimeError("TS20 actual MIT send clock did not increase")
            sequence = int(row["sequence"])
            if sequence != row["sequence"] or sequence < 0:
                raise RuntimeError("TS20 actual MIT sequence is invalid")
            if self._last_packet_sequence is not None and (
                    sequence != self._last_packet_sequence+1
                    or abs(send-self._last_packet_time-self.predictor.bus_period_s) > 1e-9):
                raise RuntimeError("TS20 actually sent MIT packet stream has a missing interval")
            if abs(row["apply_s"]-send-self.predictor.delay_s) > 1e-9:
                raise RuntimeError("TS20 plant/predictor combined delay differs")
            if self.bus_origin_s is None:
                self.bus_origin_s = send
            if abs((send-self.bus_origin_s)/self.predictor.bus_period_s
                   -round((send-self.bus_origin_s)/self.predictor.bus_period_s)) > 1e-6:
                raise RuntimeError("TS20 actual MIT send is off the frozen bus grid")
            self._last_packet_time = send
            self._last_packet_sequence = sequence
            self._packets.append(row)

    def _past_row(self, now):
        self._update()
        eligible = [r for r in self._rows if r["sim_s"] <= now]
        if not eligible:
            raise RuntimeError("No causal TS20 encoder sample at the control clock")
        row = eligible[-1]
        age = now-row["sim_s"]
        if not 0 <= age+1e-10 <= self.max_age_s+1e-10:
            raise RuntimeError("TS20 encoder source age exceeds the frozen limit")
        disagreement = max(abs(row[f"q{j}_physical"]-row[f"q{j}_internal"]) for j in (1, 2))
        self.max_position_disagreement_rad = max(self.max_position_disagreement_rad, disagreement)
        if disagreement > self.physical_position_tolerance_rad:
            raise RuntimeError("Conditional TS20 state diverges from actual Gazebo joint position")
        packets = {int(p["sequence"]): p for p in self._packets}
        for prefix, sequence_key in (("last_", "last_send_sequence"), ("active_", "active_sequence")):
            sequence = int(row[sequence_key])
            # The plugin bootstrap hold precedes its very first delayed packet.
            if sequence == 0 and prefix == "active_":
                continue
            packet = packets.get(sequence)
            if packet is None:
                raise RuntimeError("TS20 state declares a missing actually sent MIT packet")
            send_key = "last_send_s" if prefix == "last_" else "active_send_s"
            apply_key = "last_send_apply_s" if prefix == "last_" else "active_apply_s"
            pairs = [(row[send_key], packet["send_s"]), (row[apply_key], packet["apply_s"])]
            for j in (1, 2):
                pairs.extend([(row[f"{prefix}qref{j}"], packet[f"qref{j}"]),
                              (row[f"{prefix}dqref{j}"], packet[f"dqref{j}"])])
            if any(abs(a-b) > 1e-10 for a, b in pairs):
                raise RuntimeError("TS20 state and actual MIT packet contents disagree")
        if not 0 <= row["sim_s"]-row["last_send_s"]+1e-10 <= self.predictor.bus_period_s+2e-10:
            raise RuntimeError("TS20 actually sent MIT packet stream is stale")
        return row

    def measured_state(self, state, sim_s=None):
        # A failed new read must not leave an old solve snapshot eligible.
        self._measured_snapshot = None
        now = state.control_time_s if sim_s is None else float(sim_s)
        if now is None or not np.isfinite(now):
            raise ValueError("TS20 measured state requires the actual control source clock")
        row = self._past_row(now)
        q = np.array([row[f"q{j}_physical"] for j in (1, 2)])
        dq = np.array([row[f"dq{j}_internal"] for j in (1, 2)])
        observed = replace(state, q=q, dq=dq, joint_stamp_s=row["sim_s"], control_time_s=now)
        # The CSV writer can flush a later-but-still-past row between this
        # observation and context(). Keep the exact q/dq/r observation and
        # already-read histories together; do not select another row there.
        self._measured_snapshot = (float(now), dict(row))
        self.observation_count += 1
        return observed, row["sim_s"]

    def context(self, state, t):
        now = float(state.control_time_s)
        if self._measured_snapshot is None:
            raise RuntimeError("TS20 MPC context requires a successful measured snapshot")
        observed_now, row = self._measured_snapshot
        if (not np.isfinite(now) or abs(now-observed_now) > 1e-9
                or state.joint_stamp_s is None or abs(row["sim_s"]-state.joint_stamp_s) > 1e-9
                or not np.array_equal(state.q, [row[f"q{j}_physical"] for j in (1, 2)])
                or not np.array_equal(state.dq, [row[f"dq{j}_internal"] for j in (1, 2)])):
            raise RuntimeError("TS20 MPC context differs from the measured joint snapshot")
        # No CSV read here: new complete rows become eligible on the next
        # measured_state call, including their actual command/packet evidence.
        origin = now-float(t)
        cutoff = min(row["sim_s"], now)-.75
        # Retain one predecessor, necessary to reconstruct the active packet or
        # consumed velocity at the beginning of a source interval.
        def trim(records, key):
            past = [i for i, r in enumerate(records) if key(r) < cutoff]
            if len(past) > 1:
                del records[:past[-1]]
        trim(self._rows, lambda r: r["sim_s"])
        trim(self._packets, lambda r: r["send_s"])
        trim(self._command_events, lambda r: r[0])
        packet_history = []
        past_packets = [r for r in self._packets if r["send_s"] <= now+1e-10]
        active = [i for i, r in enumerate(past_packets)
                  if r["apply_s"] <= state.joint_stamp_s+1e-10]
        if active:
            past_packets = past_packets[active[-1]:]
        for raw in past_packets:
            if raw["send_s"] > now+1e-10:
                continue
            packet = {"send_time_s": raw["send_s"]-origin}
            for name, short in (("q_ref", "qref"), ("dq_ref", "dqref"),
                                ("tau_ff", "tau"), ("kp", "kp"), ("kd", "kd")):
                packet[name] = np.array([raw[f"{short}{j}"] for j in (1, 2)])
            packet_history.append(packet)
        causal_commands = [(when, value) for when, value, available in self._command_events
                           if when <= now+1e-10 and available <= now+1e-10]
        prior = [i for i, (when, _) in enumerate(causal_commands) if when <= row["sim_s"]+1e-10]
        if prior:
            causal_commands = causal_commands[prior[-1]:]
        history = [(when-origin, value.copy()) for when, value in causal_commands]
        if not history or not packet_history or self.bus_origin_s is None:
            raise RuntimeError("TS20 actual command or MIT packet history is missing")
        return {"reference_state": {"time_s": row["sim_s"]-origin,
                    "r": np.array([row["r1"], row["r2"]])},
                "packet_history": packet_history, "command_history": history,
                "bus_origin_s": self.bus_origin_s-origin}

    def metadata(self):
        return {"state_csv": str(self.states.path), "packet_csv": str(self.packets.path),
            "joint_position_source": "same-stamp physical Gazebo joint encoder",
            "joint_velocity_source": "conditional simulated servo endpoint speed report",
            "physical_interval_average_velocity_used_for_prediction": False,
            "solve_snapshot_policy": "exact measured q/dq/r and already-read causal histories; refresh only on next observation",
            "future_rows_used_for_control": False, "source_age_limit_s": self.max_age_s,
            "position_consistency_limit_rad": self.physical_position_tolerance_rad,
            "max_position_disagreement_rad": self.max_position_disagreement_rad,
            "observations": self.observation_count, "supply_24v_qualified": False,
            "hardware_sensor_model_qualified": False}

    def close(self):
        self.states.close()
        self.packets.close()
        return self.metadata()
