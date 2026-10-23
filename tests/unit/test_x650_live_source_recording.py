"""Source recorder loss/corruption cannot turn into a physical certificate."""
import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT))
from simulations.x650_isolated_live_v1.source_recording import materialize


class SourceRecordingTest(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.folder=Path(self.temp.name)
        (self.folder/'source_recording').mkdir()
        raw=b'{"sequence":1,"kind":"feedback","value":{"sequence":1,"sim_s":12.5,"feedback":{"q":[0.1,0.2]}}}\n'
        (self.folder/'source_recording/diagnostics.jsonl').write_bytes(raw)
        self.write(self.folder/'source_sender.json',{'complete':True,'dropped':0,'attempts':1,'sent':1})
        self.write(self.folder/'source_recording/manifest.json',{'rows':1,'last_sequence':1,'gaps':[],
            'error':None,'drained_and_fsynced':True,'records_sha256':hashlib.sha256(raw).hexdigest()})
        self.write(self.folder/'source_timing.json',{'rows':[[1,10,11,12.5,12.5]]})

    def tearDown(self):self.temp.cleanup()
    @staticmethod
    def write(path,value):path.write_text(json.dumps(value))
    def run_materialize(self):return materialize(self.folder,{},None,None,self.write)

    def test_complete_original_observations_preserved(self):
        report=self.run_materialize()
        self.assertTrue(report['complete'])
        self.assertEqual(json.loads((self.folder/'feedback_sources.json').read_text())[0]['feedback']['q'],[.1,.2])

    def test_loss_count_cannot_be_hidden(self):
        self.write(self.folder/'source_sender.json',{'complete':False,'dropped':1,'attempts':2,'sent':1})
        with self.assertRaisesRegex(RuntimeError,'reconciliation'):self.run_materialize()

    def test_tampered_payload_rejected(self):
        p=self.folder/'source_recording/diagnostics.jsonl'
        p.write_bytes(p.read_bytes().replace(b'0.1',b'0.9'))
        with self.assertRaisesRegex(RuntimeError,'reconciliation'):self.run_materialize()

    def test_missing_published_snapshot_rejected(self):
        self.write(self.folder/'source_timing.json',{'rows':[[1],[2]]})
        with self.assertRaisesRegex(RuntimeError,'every published'):self.run_materialize()


if __name__=='__main__':unittest.main()
