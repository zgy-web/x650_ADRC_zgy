import sys
from pathlib import Path
from unittest import TestCase,main
from unittest.mock import patch

sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'src'))
from amsim.runtime.process import _alive


class ProcessStatusTests(TestCase):
    def test_zombie_is_not_a_live_supervisor(self):
        with patch('os.kill'),patch.object(Path,'read_text',return_value='123 (name with ) paren) Z 1 2 3'):
            self.assertFalse(_alive(123))

    def test_running_and_missing_proc(self):
        with patch('os.kill'),patch.object(Path,'read_text',return_value='123 (python) S 1 2 3'):
            self.assertTrue(_alive(123))
        with patch('os.kill'),patch.object(Path,'read_text',side_effect=FileNotFoundError):
            self.assertTrue(_alive(123))

    def test_gone_process(self):
        with patch('os.kill',side_effect=ProcessLookupError):self.assertFalse(_alive(123))


if __name__=='__main__':main()
