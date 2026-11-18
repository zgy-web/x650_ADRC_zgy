"""Wall time gaps between advances of the ROS simulation clock."""
from __future__ import annotations


class ClockObservation:
    def __init__(self):
        self.stamp = None
        self.received_at = None
        self.advanced_at = None
        self.closed_gap_max = 0.

    def receive(self, stamp, now):
        """Record each callback, including repeated simulation timestamps."""
        if self.stamp is None or stamp > self.stamp:
            if self.advanced_at is not None:
                self.closed_gap_max = max(self.closed_gap_max, now - self.advanced_at)
            self.advanced_at = now
        self.stamp = stamp
        self.received_at = now

    def snapshot(self, now):
        return {
            'clock_receipt_age_s': None if self.received_at is None else now - self.received_at,
            'clock_no_progress_max_wall_s': (
                None if self.advanced_at is None else max(self.closed_gap_max, now - self.advanced_at)),
        }

    def reset_window(self):
        # Keep the last advance: an open gap can span multiple command cycles.
        self.closed_gap_max = 0.
