"""Exercise the actual entry-point function with a ROS-free shutdown harness."""
import ast
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock


class ExternalShutdown(Exception):
    pass


class NodeLifecycleTests(unittest.TestCase):
    def entrypoint(self, spin_error, context_ok):
        source = Path(__file__).resolve().parents[1]/"flyarm_control_recorder"/"recorder_node.py"
        tree = ast.parse(source.read_text(encoding="utf-8"), filename=str(source))
        main = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == "main")
        node = SimpleNamespace(core=SimpleNamespace(close=Mock()), destroy_node=Mock())
        ros = SimpleNamespace(init=Mock(), spin=Mock(side_effect=spin_error),
                              ok=Mock(return_value=context_ok), shutdown=Mock())
        namespace = {"rclpy": ros, "RecorderNode": Mock(return_value=node),
                     "ExternalShutdownException": ExternalShutdown}
        exec(compile(ast.Module(body=[main], type_ignores=[]), str(source), "exec"), namespace)
        return namespace["main"], node, ros

    def test_sigint_and_external_shutdown_close_cleanly(self):
        for signal in (KeyboardInterrupt(), ExternalShutdown()):
            for context_ok in (True, False):
                with self.subTest(signal=type(signal).__name__, context_ok=context_ok):
                    main, node, ros = self.entrypoint(signal, context_ok)
                    main(args=["--ros-args"])
                    node.core.close.assert_called_once_with()
                    node.destroy_node.assert_called_once_with()
                    self.assertEqual(ros.shutdown.call_count, int(context_ok))

    def test_unexpected_spin_failure_is_not_masked(self):
        main, node, ros = self.entrypoint(RuntimeError("unexpected executor failure"), True)
        with self.assertRaisesRegex(RuntimeError, "unexpected executor failure"):
            main()
        node.core.close.assert_called_once_with()
        node.destroy_node.assert_called_once_with()
        ros.shutdown.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
