import json
from pathlib import Path
import pickle
import socket
import sys
import tempfile
import unittest
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "ros2_ws/src/flyarm_mpc_reference"))
from flyarm_mpc_reference.diagnostic_channel import DiagnosticSender, HEADER, MAX_PACKET


@unittest.skipUnless(sys.platform.startswith("linux"), "Private Linux/Orin IPC contract")
class DiagnosticIsolationTest(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix="farm-unit-")
        self.path = Path(self.temp.name)
        self.receiver = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.receiver.bind(str(self.path/"s"))
        self.sender = DiagnosticSender(self.path/"s")

    def tearDown(self):
        self.sender.socket.close()
        self.receiver.close()
        self.temp.cleanup()

    def test_mutation_after_offer_cannot_rewrite_recorded_state(self):
        state = np.arange(15., dtype=float)
        self.assertTrue(self.sender.offer(1, {"state": state}))
        state[:] = -100
        raw = self.receiver.recv(MAX_PACKET)
        saved = pickle.loads(raw[HEADER.size:])
        np.testing.assert_array_equal(saved["state"], np.arange(15.))

    def test_full_recorder_queue_is_visible_and_never_waits(self):
        import time
        results = []
        start = time.monotonic()
        for seq in range(1, 101):
            results.append(self.sender.offer(seq, {"matrix": np.ones(12000)}))
        self.assertLess(time.monotonic()-start, 1.)
        self.assertIn(False, results)
        result = self.sender.close(self.path/"sender.json")
        self.assertEqual(result["attempts"], 100)
        self.assertEqual(result["sent"]+result["dropped"], 100)
        self.assertFalse(result["complete"])
        self.assertIsNotNone(result["first_error"])

    def test_oversized_row_cannot_silently_truncate(self):
        self.assertFalse(self.sender.offer(1, {"data": bytes(MAX_PACKET)}))
        self.assertEqual(self.sender.dropped, 1)
        self.receiver.settimeout(.01)
        with self.assertRaises(socket.timeout): self.receiver.recv(MAX_PACKET)

    def test_receiver_disappearance_is_evidence_failure(self):
        self.receiver.close()
        self.assertFalse(self.sender.offer(1, {"state": [0., 1.]}))
        self.assertEqual(self.sender.dropped, 1)


if __name__ == "__main__": unittest.main()
