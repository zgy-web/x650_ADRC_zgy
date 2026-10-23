"""Process cleanup when ROS SIGINT already closed its context, without ROS."""
from pathlib import Path
import sys
import unittest
from unittest.mock import Mock

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from flyarm_mpc_reference.node import _shutdown_ros


class CleanupTests(unittest.TestCase):
    def test_external_shutdown_is_not_repeated(self):
        node, executor, ros = Mock(), Mock(), Mock()
        ros.ok.return_value = False
        _shutdown_ros(node, executor, ros)
        executor.shutdown.assert_called_once_with()
        node.destroy_node.assert_called_once_with()
        ros.shutdown.assert_not_called()

    def test_constructor_failure_still_shuts_down_owned_context(self):
        ros = Mock()
        ros.ok.return_value = True
        _shutdown_ros(None, None, ros)
        ros.shutdown.assert_called_once_with()

    def test_executor_failure_still_destroys_node_and_closes_context(self):
        node, executor, ros = Mock(), Mock(), Mock()
        executor.shutdown.side_effect = RuntimeError("executor cleanup failed")
        ros.ok.return_value = True
        with self.assertRaisesRegex(RuntimeError, "executor cleanup"):
            _shutdown_ros(node, executor, ros)
        node.destroy_node.assert_called_once_with()
        ros.shutdown.assert_called_once_with()


if __name__ == "__main__":
    unittest.main()
