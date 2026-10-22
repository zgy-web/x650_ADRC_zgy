"""Causal sensor/command time contracts, including unread future CSV rows."""
from dataclasses import replace
from pathlib import Path
import csv
import sys
import tempfile
import unittest
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from x650_wholebody.api import ControlState
from x650_wholebody.ts20_runtime import Ts20Runtime
from x650_wholebody.ts20_prediction import Ts20PredictionModel
from test_ts20_prediction import metadata


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.folder = Path(self.tmp.name)
        self.predictor = Ts20PredictionModel(metadata())
        self.state = ControlState([0., 0., 2.], np.eye(3), [0.]*3, [0.]*3,
            [0.]*2, [0.]*2, 10.012, control_time_s=10.012, joint_stamp_s=10.012)

    def write(self, future=True, disagree=False):
        rows = []
        for i in range(1, 5 if future else 4):
            at = 10.+i*.004
            row = {"sim_s": at, "interval_begin_s": at-.004, "step_dt_s": .004}
            for j in (1, 2):
                row.update({f"q{j}_physical": .001 if disagree else 0.,
                    f"q{j}_internal": 0., f"dq{j}_physical": .123,
                    f"dq{j}_internal": .234, f"r{j}": .01*i,
                    f"u{j}": .2 if i == 4 else 0.,
                    f"receipt{j}_s": at-.004 if i == 4 else 10.,
                    f"upper_sequence{j}": 1 if i == 4 else 0.})
            send_index = int(np.floor((at-10.)/.005+1e-8))
            active_index = int(np.floor((at-10.-self.predictor.delay_s)/.005+1e-8))
            for prefix, index in (("last_", send_index), ("active_", active_index)):
                send = 10.+index*.005
                row["last_send_sequence" if prefix == "last_" else "active_sequence"] = index
                row["last_send_s" if prefix == "last_" else "active_send_s"] = send
                row["last_send_apply_s" if prefix == "last_" else "active_apply_s"] = send+self.predictor.delay_s
                for j in (1, 2):
                    row[f"{prefix}qref{j}"] = row[f"{prefix}dqref{j}"] = 0.
            rows.append(row)
        with (self.folder/"s.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
            writer.writeheader(); writer.writerows(rows)
        fields = ["send_s", "apply_s", "sequence"]
        fields += [f"{name}{j}" for j in (1, 2) for name in ("qref", "dqref", "tau", "kp", "kd")]
        with (self.folder/"p.csv").open("w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields); writer.writeheader()
            for i in range(4):
                at = 10.+i*.005
                row = dict(send_s=at, apply_s=at+self.predictor.delay_s, sequence=i)
                for j in (1, 2):
                    row.update({f"qref{j}": 0., f"dqref{j}": 0., f"tau{j}": 0.,
                                f"kp{j}": 6., f"kd{j}": 3276/32768})
                writer.writerow(row)
        runtime = Ts20Runtime(self.folder/"s.csv", self.folder/"p.csv", self.predictor)
        self.addCleanup(runtime.close)
        return runtime

    def test_future_rows_do_not_enter_encoder_or_consumed_history(self):
        runtime = self.write()
        state, stamp = runtime.measured_state(self.state)
        self.assertAlmostEqual(stamp, 10.012)
        np.testing.assert_array_equal(state.dq, [.234, .234])
        context = runtime.context(state, .012)
        np.testing.assert_array_equal(context["reference_state"]["r"], [.03, .03])
        self.assertTrue(all(p["send_time_s"] <= .012+1e-10 for p in context["packet_history"]))
        self.assertTrue(all(np.all(value == 0.) for _, value in context["command_history"]))
        # The next causal observation now admits the consumed command exactly.
        later, _ = runtime.measured_state(replace(self.state, control_time_s=10.016))
        newer = runtime.context(later, .016)
        np.testing.assert_array_equal(newer["command_history"][-1][1], [.2, .2])

    def test_physical_position_consistency_and_stale_source_refuse(self):
        runtime = self.write(future=False, disagree=True)
        runtime.physical_position_tolerance_rad = .0009
        with self.assertRaisesRegex(RuntimeError, "diverges"):
            runtime.measured_state(self.state)
        runtime.physical_position_tolerance_rad = .002
        with self.assertRaisesRegex(RuntimeError, "source age"):
            runtime.measured_state(replace(self.state, control_time_s=10.10))

    def test_context_cannot_silently_replace_joint_snapshot(self):
        runtime = self.write(future=False)
        state, _ = runtime.measured_state(self.state)
        with self.assertRaisesRegex(RuntimeError, "differs"):
            runtime.context(replace(state, joint_stamp_s=10.008), .012)

    def test_late_past_row_is_admitted_only_by_the_next_measurement(self):
        runtime = self.write(future=False)
        path = self.folder/"s.csv"
        lines = path.read_text().splitlines(keepends=True)
        # At observation time the 10.012 row is still buffered by the writer.
        path.write_text("".join(lines[:-1]))
        state, stamp = runtime.measured_state(self.state)
        self.assertAlmostEqual(stamp, 10.008)
        with path.open("a") as stream:
            stream.write(lines[-1])
        context = runtime.context(state, .012)
        self.assertAlmostEqual(context["reference_state"]["time_s"], .008)
        np.testing.assert_array_equal(context["reference_state"]["r"], [.02, .02])
        # A fresh observation can now use it; the prior state cannot borrow
        # the newer context even though the control clock has not changed.
        later, newer_stamp = runtime.measured_state(self.state)
        self.assertAlmostEqual(newer_stamp, 10.012)
        newer = runtime.context(later, .012)
        np.testing.assert_array_equal(newer["reference_state"]["r"], [.03, .03])
        with self.assertRaisesRegex(RuntimeError, "differs"):
            runtime.context(state, .012)

    def test_context_rejects_unobserved_mutated_or_failed_snapshot(self):
        runtime = self.write(future=False)
        with self.assertRaisesRegex(RuntimeError, "successful measured snapshot"):
            runtime.context(self.state, .012)
        state, _ = runtime.measured_state(self.state)
        for changed in (replace(state, control_time_s=10.016),
                        replace(state, q=[.001, 0.]), replace(state, dq=[.235, .234])):
            with self.assertRaisesRegex(RuntimeError, "differs"):
                runtime.context(changed, .012)
        with self.assertRaisesRegex(RuntimeError, "source age"):
            runtime.measured_state(replace(self.state, control_time_s=10.10))
        with self.assertRaisesRegex(RuntimeError, "successful measured snapshot"):
            runtime.context(state, .012)

    def test_prestream_history_is_bounded_before_any_mpc_context(self):
        runtime = self.write(future=False)
        state_path, packet_path = self.folder/"s.csv", self.folder/"p.csv"
        with state_path.open(newline="") as stream:
            old_states = list(csv.DictReader(stream))
        with packet_path.open(newline="") as stream:
            old_packets = list(csv.DictReader(stream))
        state_template = {key: float(value) for key, value in old_states[-1].items()}
        packet_template = {key: float(value) for key, value in old_packets[-1].items()}
        runtime.measured_state(self.state)
        last_packet = 3
        with state_path.open("a", newline="") as state_stream, packet_path.open("a", newline="") as packet_stream:
            state_writer = csv.DictWriter(state_stream, fieldnames=list(state_template))
            packet_writer = csv.DictWriter(packet_stream, fieldnames=list(packet_template))
            for i in range(4, 1004):
                at = 10.+i*.004
                send_index = int(np.floor((at-10.)/.005+1e-8))
                active_index = int(np.floor((at-10.-self.predictor.delay_s)/.005+1e-8))
                for index in range(last_packet+1, send_index+1):
                    packet = dict(packet_template, send_s=10.+index*.005,
                                  apply_s=10.+index*.005+self.predictor.delay_s, sequence=index)
                    packet_writer.writerow(packet)
                last_packet = max(last_packet, send_index)
                packet_stream.flush()
                row = dict(state_template, sim_s=at, interval_begin_s=at-.004)
                for j in (1, 2):
                    row[f"upper_sequence{j}"] = i-3
                    row[f"receipt{j}_s"] = at-.004
                for prefix, index in (("last_", send_index), ("active_", active_index)):
                    send = 10.+index*.005
                    row["last_send_sequence" if prefix == "last_" else "active_sequence"] = index
                    row["last_send_s" if prefix == "last_" else "active_send_s"] = send
                    row["last_send_apply_s" if prefix == "last_" else "active_apply_s"] = send+self.predictor.delay_s
                state_writer.writerow(row)
                state_stream.flush()
                observed, _ = runtime.measured_state(replace(self.state, control_time_s=at))
                # No MPC context has run throughout these four seconds.
                self.assertLessEqual(len(runtime._rows), 190)
                self.assertLessEqual(len(runtime._packets), 152)
                self.assertLessEqual(len(runtime._command_events), 190)
        context = runtime.context(observed, at-10.)
        np.testing.assert_array_equal(context["reference_state"]["r"], [.03, .03])
        self.assertGreaterEqual(len(context["packet_history"]), 1)
        self.assertTrue(all(np.all(value == 0.) for _, value in context["command_history"]))

    def test_truncated_static_packet_stream_cannot_pass_from_held_zero(self):
        runtime = self.write(future=False)
        path = self.folder/"p.csv"
        lines = path.read_text().splitlines(keepends=True)
        path.write_text("".join(lines[:2]))
        with self.assertRaisesRegex(RuntimeError, "missing actually sent"):
            runtime.measured_state(self.state)


if __name__ == "__main__":
    unittest.main()
