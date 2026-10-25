"""The nav2 node says only what it saw: findings 10 and 11, without ROS.

The node itself needs rclpy, so its two decisions are module-level and are
checked here directly; the ROS run that shows them end to end is SpatialDDS-
demo's `robot-captures.spec.ts`, "the sim dies...", against the nav2 tier.
"""
from types import SimpleNamespace

from spatialdds_ros.robot_node import (
    GOAL_ABORTED, GOAL_CANCELED, GOAL_SUCCEEDED, TfFreshness, goal_outcome)


def stamp(sec, nanosec=0):
    return SimpleNamespace(sec=sec, nanosec=nanosec)


def test_a_transform_already_answered_is_not_a_new_observation():
    """Finding 10: the buffer's last transform, asked for again after its
    publisher died, must not count as a sighting."""
    fresh = TfFreshness()
    assert fresh.new(stamp(100, 5)) is True
    assert fresh.new(stamp(100, 5)) is False      # the sim died here
    assert fresh.new(stamp(100, 5)) is False
    assert fresh.new(stamp(100, 6)) is True       # and came back


def test_freshness_is_stamp_to_stamp_not_against_a_clock():
    """A sim on its own time and a robot on wall time are treated alike."""
    fresh = TfFreshness()
    assert fresh.new(stamp(0, 1)) is True
    assert fresh.new(stamp(1792950000, 0)) is True


def test_a_goal_that_arrived_owes_no_refusal():
    assert goal_outcome(True, GOAL_SUCCEEDED, 1, 2) is None


def test_every_other_ending_is_a_refusal_naming_the_goal():
    """Finding 11: rejected, aborted and cancelled goals reach the requester."""
    for accepted, status, word in ((False, None, "rejected"),
                                   (True, GOAL_ABORTED, "abandoned"),
                                   (True, GOAL_CANCELED, "cancelled"),
                                   (True, 2, "status 2")):
        why = goal_outcome(accepted, status, 10.0, -12.0)
        assert why and "(10.00, -12.00)" in why and word in why, why
