#!/usr/bin/env python3
"""
The two things the library fills rather than asks about.

Both are conventions for the same reason — being wrong is silent — so both are
tested for the property that makes them safe rather than for their values
alone.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from spatialdds_ros import qos                               # noqa: E402
from spatialdds_ros.tempo import Tempo                       # noqa: E402


class TheProfileTableIsReadableWithoutADdsBuild(unittest.TestCase):
    """
    The table has to be assertable where there is no CycloneDDS, because that
    is where this library's unit tests run — a laptop with no ROS and no DDS.
    `to_qos()` needs the binding; reading the row must not.
    """

    def test_the_three_lanes_are_present(self):
        self.assertEqual(set(qos.PROFILES),
                         {"MODEL_LATCHED", "MODEL_FAST", "MODEL_COMMAND"})

    def test_the_latched_lane_is_reliable_and_transient_local(self):
        """What makes a late joiner work without asking anyone anything."""
        p = qos.MODEL_LATCHED
        self.assertTrue(p.reliable)
        self.assertTrue(p.latched)
        self.assertEqual(p.keep_last, 1)

    def test_the_fast_lane_is_best_effort_and_volatile(self):
        """A pose that missed its moment is worth less than the next one."""
        p = qos.MODEL_FAST
        self.assertFalse(p.reliable)
        self.assertFalse(p.latched)
        self.assertEqual(p.keep_last, 1)

    def test_the_command_lane_is_reliable_and_volatile(self):
        """A request must not be lost, and must not be replayed tomorrow."""
        p = qos.MODEL_COMMAND
        self.assertTrue(p.reliable)
        self.assertFalse(p.latched)
        self.assertEqual(p.keep_last, 16)

    def test_no_lane_requests_a_deadline(self):
        """
        **The lesson this profile table exists to carry.** Deadline is
        request/offered in DDS: a reader asking for one does not match a writer
        offering none, and the failure is silent. A dash in the spec's column
        means no deadline, which is a different statement from an unstated one.
        If a future lane needs one it must be added on both sides deliberately,
        so this asserts the current answer rather than leaving it to drift.
        """
        for name, profile in qos.PROFILES.items():
            with self.subTest(lane=name):
                self.assertIsNone(profile.deadline_ms)

    def test_every_profile_explains_itself(self):
        for name, profile in qos.PROFILES.items():
            with self.subTest(lane=name):
                self.assertTrue(profile.note,
                                f"{name} should carry its §3.3.3 row in prose")


class TheCadenceIsMeasuredNotAssumed(unittest.TestCase):
    """
    The 94-flush lesson, as behaviour.

    A fixed threshold shorter than the update interval makes every gap look
    like a stop. These assert that the threshold follows the feed instead.
    """

    def test_the_latched_record_is_due_every_nth_update(self):
        tempo = Tempo(latch_every=5)
        due = [tempo.observed("r", now=i * 0.1) for i in range(10)]
        self.assertEqual(due.count(True), 2,
                         "five updates per latch, so ten updates latch twice")

    def test_one_update_is_not_a_cadence(self):
        """Until there are two there is no interval, so the floor is all there
        is to go on."""
        tempo = Tempo(idle_floor=1.0)
        tempo.observed("r", now=100.0)
        self.assertEqual(tempo.threshold("r"), 1.0)

    def test_a_slow_feed_gets_a_long_threshold(self):
        """
        The actual defect from the prototype: a 1.5 s feed against a 1 s fixed
        threshold was declared stopped between every pair of updates. Here the
        threshold has to exceed the feed's own interval.
        """
        tempo = Tempo(idle_floor=1.0, idle_factor=3.0)
        for i in range(8):
            tempo.observed("duck", now=100.0 + i * 1.5)
        threshold = tempo.threshold("duck")
        self.assertGreater(threshold, 1.5,
                           "a feed updating every 1.5 s must not be called "
                           "stopped after 1.5 s of quiet")
        self.assertAlmostEqual(threshold, 4.5, places=1)

    def test_a_fast_feed_is_not_given_a_needlessly_long_threshold(self):
        """The floor, not the factor, governs a 10 Hz feed."""
        tempo = Tempo(idle_floor=1.0, idle_factor=3.0)
        for i in range(10):
            tempo.observed("r", now=100.0 + i * 0.1)
        self.assertEqual(tempo.threshold("r"), 1.0)

    def test_a_still_world_is_republished_once_not_every_tick(self):
        tempo = Tempo(latch_every=5, idle_floor=1.0)
        tempo.observed("r", now=100.0)
        self.assertEqual(tempo.idle(now=100.5), [],
                         "not yet quiet for long enough")
        self.assertEqual(tempo.idle(now=102.0), ["r"], "quiet: flush it")
        self.assertEqual(tempo.idle(now=103.0), [],
                         "and exactly once — the flush marked it clean")

    def test_the_cadence_follows_a_change_within_a_few_updates(self):
        """Smoothing low enough to track, high enough that one late sample does
        not convince it everything stopped."""
        tempo = Tempo()
        for i in range(10):
            tempo.observed("r", now=100.0 + i * 0.1)
        fast = tempo.threshold("r")
        for i in range(10):
            tempo.observed("r", now=101.0 + i * 2.0)
        slow = tempo.threshold("r")
        self.assertGreater(slow, fast * 2,
                           "the threshold should have followed the slowdown")

    def test_forgetting_a_key_clears_it(self):
        tempo = Tempo()
        tempo.observed("r", now=100.0)
        tempo.forget("r")
        self.assertEqual(tempo.idle(now=200.0), [])
        self.assertEqual(tempo.threshold("r"), tempo.idle_floor)


if __name__ == "__main__":
    unittest.main()
