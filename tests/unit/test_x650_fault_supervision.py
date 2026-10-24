from types import SimpleNamespace
import unittest

from simulations.x650_isolated_live_v1.supervision import FaultLatch


def tick(status=3, session=7, epoch=2, reason=1, sequence=42):
    return SimpleNamespace(session_id=session, epoch=epoch, sequence=sequence,
                           status=status, reason=reason, actual_monotonic_ns=900)


class FaultLatchTests(unittest.TestCase):
    def test_first_fault_survives_stopped_and_later_fault(self):
        latch=FaultLatch(7,2,clock=lambda:1000)
        latch.observe(tick(),"dispatch")
        original=dict(latch.first)
        latch.observe(tick(status=4),"dispatch")
        latch.observe(tick(reason=9,sequence=43),"dispatch")
        self.assertEqual(latch.first,original)
        self.assertEqual(latch.first['producer_to_observation_ns'],100)
        self.assertIn('reason 1',latch.error())

    def test_unrelated_session_or_epoch_cannot_poison(self):
        latch=FaultLatch(7,2)
        latch.observe(tick(session=8),"dispatch")
        latch.observe(tick(epoch=3),"dispatch")
        for status in (0,1,2,4):latch.observe(tick(status=status),"dispatch")
        self.assertIsNone(latch.error())
        self.assertEqual(latch.report()['counts']['ignored_identity'],2)

    def test_candidate_rejection_latches_original_time(self):
        latch=FaultLatch(7,2,clock=lambda:1234)
        event=SimpleNamespace(session_id=7,epoch=2,sequence=8,accepted=True,
                              reason=5,received_monotonic_ns=1200)
        latch.observe(event,'events')
        self.assertIsNone(latch.error())
        event.accepted=False
        latch.observe(event,'events')
        self.assertEqual(latch.first['producer_monotonic_ns'],1200)
        self.assertIn('candidate gate rejected reason 5',latch.error())


if __name__=='__main__':unittest.main()
