#!/usr/bin/env python3
"""The no-hardware source: straight lines, and a late refusal."""
from __future__ import annotations

import math
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from spatialdds_ros.kinematic import (                        # noqa: E402
    ARRIVAL_M, Bounds, KinematicNavigator, KinematicSource)


class ItDrivesStraightLines(unittest.TestCase):
    def test_idle_is_still(self):
        """
        A sim that drifted while idle would publish motion nobody asked for,
        and the fast lane would carry it as though the robot were moving.
        """
        sim = KinematicSource(start=(1.0, 2.0), yaw=0.5)
        for _ in range(20):
            x, y, yaw = sim.step(0.1)
        self.assertEqual((x, y, yaw), (1.0, 2.0, 0.5))

    def test_it_reaches_a_goal(self):
        sim = KinematicSource(start=(0.0, 0.0), speed=1.0)
        sim.goal = (3.0, 0.0)
        for _ in range(100):
            x, y, _ = sim.step(0.1)
            if sim.goal is None:
                break
        self.assertAlmostEqual(x, 3.0, delta=ARRIVAL_M)
        self.assertAlmostEqual(y, 0.0, delta=ARRIVAL_M)
        self.assertIsNone(sim.goal, "arriving should clear the goal")

    def test_it_faces_where_it_is_going(self):
        sim = KinematicSource(start=(0.0, 0.0), speed=1.0)
        sim.goal = (0.0, 5.0)
        _, _, yaw = sim.step(0.1)
        self.assertAlmostEqual(yaw, math.pi / 2, places=6)

    def test_speed_is_respected(self):
        sim = KinematicSource(start=(0.0, 0.0), speed=2.0)
        sim.goal = (100.0, 0.0)
        x, _, _ = sim.step(0.5)
        self.assertAlmostEqual(x, 1.0, places=6)

    def test_it_does_not_overshoot(self):
        sim = KinematicSource(start=(0.0, 0.0), speed=100.0)
        sim.goal = (1.0, 0.0)
        x, y, _ = sim.step(1.0)
        self.assertAlmostEqual(x, 1.0, delta=ARRIVAL_M)

    def test_nothing_is_abandoned_without_bounds(self):
        """Unbounded is the default, so by default nothing is refused."""
        sim = KinematicSource(speed=10.0)
        sim.goal = (1000.0, -1000.0)
        for _ in range(50):
            sim.step(0.5)
        self.assertIsNone(sim.abandoned)


class TheLateRefusal(unittest.TestCase):
    """
    The shape worth reproducing: the goal is accepted honestly and given up on
    later, because whether it was drivable depended on where the robot got to.
    """

    def test_a_goal_outside_the_bounds_is_abandoned_at_the_boundary(self):
        sim = KinematicSource(start=(0.0, 0.0), speed=1.0,
                              bounds=Bounds(-5.0, -5.0, 5.0, 5.0))
        sim.goal = (20.0, 0.0)
        for _ in range(200):
            sim.step(0.1)
            if sim.abandoned:
                break
        self.assertIsNotNone(sim.abandoned, "it should have given up")
        self.assertIn("abandoned", sim.abandoned)
        self.assertIn("bounds", sim.abandoned)
        self.assertIsNone(sim.goal, "and dropped the goal")
        self.assertLessEqual(sim.x, 5.0, "and stopped inside the bounds")

    def test_the_reason_names_where_it_stopped(self):
        """Where it stopped is part of the reason — a refusal that named only
        the goal would not say what was in the way."""
        sim = KinematicSource(start=(0.0, 0.0), speed=1.0,
                              bounds=Bounds(-5.0, -5.0, 5.0, 5.0))
        sim.goal = (20.0, 0.0)
        for _ in range(200):
            sim.step(0.1)
            if sim.abandoned:
                break
        self.assertIn("(20.00, 0.00)", sim.abandoned, "the goal")
        self.assertRegex(sim.abandoned, r"at \(\d\.\d\d, 0\.00\)",
                         "and the position it gave up at")

    def test_a_goal_inside_the_bounds_is_reached(self):
        """The control: bounds must not refuse everything."""
        sim = KinematicSource(start=(0.0, 0.0), speed=1.0,
                              bounds=Bounds(-5.0, -5.0, 5.0, 5.0))
        sim.goal = (3.0, 3.0)
        for _ in range(200):
            sim.step(0.1)
            if sim.goal is None:
                break
        self.assertIsNone(sim.abandoned)
        self.assertAlmostEqual(sim.x, 3.0, delta=ARRIVAL_M)

    def test_it_keeps_driving_after_a_refusal(self):
        """A refused goal is not a terminal state."""
        sim = KinematicSource(start=(0.0, 0.0), speed=1.0,
                              bounds=Bounds(-5.0, -5.0, 5.0, 5.0))
        sim.goal = (20.0, 0.0)
        for _ in range(200):
            sim.step(0.1)
            if sim.abandoned:
                break
        sim.abandoned = None
        sim.goal = (0.0, 2.0)
        for _ in range(200):
            sim.step(0.1)
            if sim.goal is None:
                break
        self.assertAlmostEqual(sim.y, 2.0, delta=ARRIVAL_M)
        self.assertIsNone(sim.abandoned)


class BoundsAreValidated(unittest.TestCase):
    def test_empty_bounds_are_rejected(self):
        with self.assertRaises(ValueError):
            Bounds(5.0, 0.0, -5.0, 10.0)

    def test_a_start_outside_the_bounds_is_rejected(self):
        """It would refuse from its first step, which is a configuration
        mistake rather than a robot behaviour."""
        with self.assertRaises(ValueError) as caught:
            KinematicSource(start=(99.0, 0.0),
                            bounds=Bounds(-5.0, -5.0, 5.0, 5.0))
        self.assertIn("outside the configured bounds", str(caught.exception))


class TheNavigator(unittest.TestCase):
    def test_it_is_always_available(self):
        """There is nothing for a straight-line sim to be unavailable for, and
        claiming otherwise would make the adapter decline goals it can take."""
        self.assertIsNone(KinematicNavigator(KinematicSource()).available())

    def test_sending_sets_the_goal_and_reports_it(self):
        sim = KinematicSource()
        note = KinematicNavigator(sim).send(4.0, 5.0)
        self.assertEqual(sim.goal, (4.0, 5.0))
        self.assertIn("(4.00, 5.00)", note)


if __name__ == "__main__":
    unittest.main()
