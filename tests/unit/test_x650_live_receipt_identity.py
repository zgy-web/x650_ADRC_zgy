"""Exercise receipt ownership checks without loading ROS/native dependencies."""
import ast
from copy import deepcopy
from pathlib import Path
import unittest

ROOT = Path(__file__).resolve().parents[2]
tree = ast.parse((ROOT / 'scripts/audit_x650_isolated_live.py').read_text(encoding='utf-8'))
function = next(n for n in tree.body if isinstance(n, ast.FunctionDef) and n.name == 'check_receipt_identity')


def require(condition, message):
    if not condition:
        raise ValueError(message)


namespace = {'require': require}
exec(compile(ast.Module(body=[function], type_ignores=[]), '<receipt-audit>', 'exec'), namespace)
check = namespace['check_receipt_identity']


class ReceiptIdentity(unittest.TestCase):
    def setUp(self):
        self.start = {'session_id': 12, 'epoch': 2}
        self.sends = [{**self.start, 'dispatch_sequence': 8, 'command_sequence': 5}]
        self.receipts = deepcopy(self.sends)

    def test_complete_same_session(self):
        self.assertEqual(set(check(self.sends, self.receipts, self.start)), {8})

    def test_missing_extra_and_duplicate_receipts(self):
        for records in ([], self.receipts * 2,
                        self.receipts + [{**self.receipts[0], 'dispatch_sequence': 9}]):
            with self.subTest(records=records), self.assertRaises(ValueError):
                check(self.sends, records, self.start)

    def test_cross_session_epoch_and_candidate(self):
        for field in ('session_id', 'epoch', 'command_sequence'):
            records = deepcopy(self.receipts)
            records[0][field] += 1
            with self.subTest(field=field), self.assertRaises(ValueError):
                check(self.sends, records, self.start)

    def test_matched_foreign_session_rejected(self):
        self.sends[0]['session_id'] += 1
        self.receipts[0]['session_id'] += 1
        with self.assertRaises(ValueError):
            check(self.sends, self.receipts, self.start)


if __name__ == '__main__':
    unittest.main()
