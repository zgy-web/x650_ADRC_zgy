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

    def test_truncated_static_packet_stream_cannot_pass_from_held_zero(self):
        runtime = self.write(future=False)
        path = self.folder/"p.csv"
        lines = path.read_text().splitlines(keepends=True)
        path.write_text("".join(lines[:2]))
        with self.assertRaisesRegex(RuntimeError, "missing actually sent"):
            runtime.measured_state(self.state)


if __name__ == "__main__":
    unittest.main()
