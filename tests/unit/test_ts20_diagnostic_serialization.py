"""Exact TS20 diagnostic JSON semantics and explicit shared-loop dispatch."""
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
for directory in (ROOT, ROOT / "control_packages/x650_wholebody_v1"):
    sys.path.insert(0, str(directory))
from simulations.x650_ts20_hold_v1 import diagnostic_serialization as serializer
from simulations.x650_ts20_hold_v1.run_case import Ts20Extension
from simulations.x650_wholebody_mpc_v1 import run_case as shared
from x650_wholebody.api import ControlState


def reference(value):
    return json.dumps(shared.json_value(value), allow_nan=False) + "\n"


def result(operation):
    try:
        return ("value", operation())
    except Exception as error:
        return ("error", type(error), str(error))


class DiagnosticSerializationTests(unittest.TestCase):
    def check_exact(self, value, *, fast):
        self.assertEqual(serializer.supports_fast_tree(value), fast)
        convert = Mock(wraps=shared.json_value)
        expected = result(lambda: reference(value))
        actual = result(lambda: serializer.encode_diagnostic_row(value, reference_converter=convert))
        self.assertEqual(actual, expected)
        if fast:
            convert.assert_not_called()
        else:
            convert.assert_called_once_with(value)

    def test_supported_tree_preserves_bytes_order_unicode_tuple_and_shared_children(self):
        child = {"error": (-0., np.float32(.1), None, True), "unicode": "关节\n位置"}
        self.check_exact({"z": child, "a": [child, {"exponent": 1e-200, "large": 10**80}]}, fast=True)

    def test_numeric_array_dtypes_shapes_layout_and_byte_order_are_exact(self):
        for dtype in (np.bool_, np.int8, np.int16, np.int32, np.int64,
                      np.uint8, np.uint16, np.uint32, np.uint64,
                      np.float16, np.float32, np.float64, ">f8", ">i4"):
            base = np.arange(12).reshape(3, 4).astype(dtype)
            for value in (base, base[:, ::2], base.T, base[0, 0].reshape(()), base[:0]):
                with self.subTest(dtype=str(dtype), shape=value.shape):
                    self.check_exact({"value": value}, fast=True)

    def test_numeric_scalars_are_exact(self):
        self.check_exact([kind(1) for kind in serializer._NUMERIC_SCALARS], fast=True)
        self.check_exact({"uint64": np.uint64(2**64-1), "int64": np.int64(-(2**63))}, fast=True)

    def test_fast_path_passes_original_container_to_encoder(self):
        row = {"events": [{"q": np.array([.1, -.2]), "kind": "acceleration_zero"}]}
        original = json.dumps
        seen = []
        def observe(value, **kwargs):
            seen.append(value)
            return original(value, **kwargs)
        with patch.object(serializer.json, "dumps", side_effect=observe):
            encoded = serializer.encode_diagnostic_row(row, reference_converter=Mock())
        self.assertIs(seen[0], row)
        self.assertIs(seen[0]["events"], row["events"])
        self.assertEqual(encoded, reference(row))

    def test_original_key_coercion_and_collision_counterexamples_fall_back(self):
        values = [
            {True: "yes", None: "missing", False: "no"},
            {(1, 2): "tuple"},
            {np.int64(2): "numpy"},
            {True: "first", "True": "second", None: "first", "None": "second"},
            {float("nan"): "key"},
        ]
        for value in values:
            with self.subTest(value=repr(value)):
                self.check_exact(value, fast=False)

    def test_object_array_numpy_and_longdouble_counterexamples_fall_back(self):
        # The original converter does not recurse into ndarray.tolist output.
        self.check_exact(np.array([np.int64(3)], dtype=object), fast=False)
        self.check_exact(np.array([{"x": [1, 2]}], dtype=object), fast=False)
        self.check_exact(np.longdouble(.1), fast=False)
        self.check_exact(np.array([.1], dtype=np.longdouble), fast=False)

    def test_nan_and_infinity_keep_original_rejection(self):
        for value in (float("nan"), float("inf"), -float("inf"), np.array([float("nan")])):
            with self.subTest(value=repr(value)):
                self.check_exact(value, fast=True)
        for value in (np.float16("inf"), np.float32("nan"), {"value": np.float64("inf")}):
            with self.subTest(value=repr(value)):
                self.check_exact(value, fast=False)

    def test_unsupported_domains_preserve_original_values_or_errors(self):
        for value in (np.str_("x"), np.array(["x"]), np.bytes_(b"x"),
                      np.complex128(1+2j), np.array([1+2j]),
                      b"unsupported", {1, 2}, np.datetime64("2026-10-23"),
                      np.array([(1, 2.)], dtype=[("a", "i4"), ("b", "f8")]), object()):
            with self.subTest(value=repr(value)):
                self.check_exact(value, fast=False)

    def test_custom_container_or_key_behavior_is_not_bypassed(self):
        class DictSubclass(dict):
            pass
        class ListSubclass(list):
            pass
        class StringSubclass(str):
            pass
        for value in (DictSubclass(x=np.int64(2)), ListSubclass([np.int64(2)]),
                      {StringSubclass("x"): 2}, np.ma.array([1, 2], mask=[False, True])):
            with self.subTest(value=repr(value)):
                self.check_exact(value, fast=False)

    def test_cycles_and_deep_inputs_are_left_to_original_converter(self):
        value = []
        value.append(value)
        self.assertFalse(serializer.supports_fast_tree(value))
        convert = Mock(side_effect=RuntimeError("original fallback called"))
        with self.assertRaisesRegex(RuntimeError, "original fallback"):
            serializer.encode_diagnostic_row(value, reference_converter=convert)
        convert.assert_called_once_with(value)
        value.clear()
        deep = None
        for _ in range(80):
            deep = [deep]
        self.check_exact(deep, fast=False)

    def test_no_gc_policy_is_read_or_changed(self):
        import gc
        enabled, thresholds = gc.isenabled(), gc.get_threshold()
        self.check_exact({"q": np.array([1., 2.])}, fast=True)
        self.assertEqual(gc.isenabled(), enabled)
        self.assertEqual(gc.get_threshold(), thresholds)


class DiagnosticDispatchTests(unittest.TestCase):
    def test_legacy_and_non_opted_extensions_keep_original_expression(self):
        row = {True: "first", "True": np.array([1., -0.])}
        for extension in (None, object(), NS(encode_diagnostic_row=None)):
            loop = shared.ControlLoop.__new__(shared.ControlLoop)
            loop.extension = extension
            with patch.object(shared, "json_value", wraps=shared.json_value) as convert:
                encoded = loop.encode_diagnostic_row(row)
                convert.assert_called_once_with(row)
            self.assertEqual(encoded, reference(row))

    def test_explicit_hook_is_called_once_and_errors_are_not_swallowed(self):
        row = {"diagnostics": np.array([1., 2.])}
        hook = Mock(return_value="explicit\n")
        loop = shared.ControlLoop.__new__(shared.ControlLoop)
        loop.extension = NS(encode_diagnostic_row=hook)
        self.assertEqual(loop.encode_diagnostic_row(row), "explicit\n")
        hook.assert_called_once_with(row)
        hook.side_effect = ValueError("diagnostic failure")
        with self.assertRaisesRegex(ValueError, "diagnostic failure"):
            loop.encode_diagnostic_row(row)

    def test_ts20_extension_uses_explicit_numerical_serializer(self):
        extension = Ts20Extension({}, {})
        row = {"diagnostics": {"q": np.array([1., 2.]), "events": [{"time": .1}]}}
        with patch.object(shared, "json_value", side_effect=AssertionError("unnecessary conversion")):
            self.assertEqual(extension.encode_diagnostic_row(row), reference_without_converter(row))
        # Unsupported input must still use the shared helper and its exact key
        # conversion, rather than the direct encoder's different key behavior.
        self.assertEqual(extension.encode_diagnostic_row({None: "key"}), reference({None: "key"}))

    def test_actual_pump_dispatches_then_encodes_original_diagnostic_row(self):
        loop = shared.ControlLoop.__new__(shared.ControlLoop)
        sent = []
        state = ControlState([0., 0., 1.], np.eye(3), [0., 0., 0.], [0., 0., 0.], [0., 0.], [0., 0.], 10.)
        loop.original_pump = lambda: None
        loop.upper_research = False
        loop.ctx = NS(node=NS(spin_once=lambda **kwargs: None, data={"position": NS(x=0., y=0., z=1., vx=0., vy=0., vz=0.)}),
                      arm=NS(set_velocity=lambda name, value: sent.append(name)),
                      vehicle=NS(target=(1, 1), link=NS(mav=NS(set_position_target_local_ned_send=lambda *args: sent.append("pva")))))
        loop.control_now = lambda: 10.
        loop.monitor_now = lambda: 10.
        loop.land_watchdog = NS(observe=lambda *args: None, diagnostics=lambda *args: {})
        loop.status_watchdog, loop.feedback_adapter = None, None
        loop.read = lambda: (10., state, 10.)
        loop.recorder = NS(phase="measure")
        loop.next_control, loop.last, loop.strict = 0., None, False
        loop.cfg = {"control_period_s": .02, "stage": "tip_hold", "duration_s": 30.}
        loop.task_start, loop.controller, loop.recovery = 9., object(), False
        loop.trajectory = NS(sample=lambda t: (np.zeros(3), np.zeros(3), np.zeros(3)))
        loop.feedback_diagnostics = None
        loop.evaluation_state = state
        loop.anchor, loop.fixed_reference, loop.send_reference = None, None, True
        loop.max_joint_cmd, loop.yaw_ned = np.zeros(2), 0.
        loop.sent_command_history = []
        loop.control, loop.diagnostic, loop.feedback_stream = io.StringIO(), io.StringIO(), None
        loop.writer, loop.rows = None, 0
        diagnostic = {"q": np.array([.1, -.2]), "events": [{"kind": "acceleration_zero"}]}
        sp = NS(type_mask=1, position=[0., 0., -1.], velocity=[0., 0., 0.], acceleration=[0., 0., 0.])
        command = NS(joint_velocity=np.zeros(2), base_velocity_target=np.zeros(3),
                     acceleration_feedforward=np.zeros(3), diagnostics=diagnostic,
                     px4_setpoint=lambda *args: sp)
        def publish(channel, payload, operation, **kwargs):
            return operation()
        loop.execution_trace = NS(controller_step=lambda *args, **kwargs: command, publish=publish)
        captured = []
        def encode(row):
            self.assertEqual(sent, ["arm_joint_1", "arm_joint_2", "pva"])
            self.assertIs(row["diagnostics"], diagnostic)
            captured.append(row)
            return serializer.encode_diagnostic_row(row, reference_converter=shared.json_value)
        loop.extension = NS(on_pump=lambda *args: None, before_publish=lambda *args: {}, encode_diagnostic_row=encode)
        clock = {"wall_monotonic_s": 20., "processed_estimator_clock_s": 10., "joint_header_stamp_s": 10.}
        with patch.dict(sys.modules, {"pymavlink": NS(mavutil=NS(mavlink=NS(MAV_FRAME_LOCAL_NED=1)))}), \
                patch.object(shared, "callback_clock_snapshot", return_value=clock):
            loop.pump()
        self.assertEqual(len(captured), 1)
        self.assertEqual(loop.diagnostic.getvalue(), reference(captured[0]))
        self.assertEqual(loop.last, 10.)
        self.assertEqual(loop.next_control, 10.02)


def reference_without_converter(row):
    """Independent fixed expectation for the supported extension fixture."""
    return '{"diagnostics": {"q": [1.0, 2.0], "events": [{"time": 0.1}]}}\n'


if __name__ == "__main__":
    unittest.main()
