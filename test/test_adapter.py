#!/usr/bin/env python3
"""
The adapter, against a real bus and no ROS.

Every test here runs on a private DDS domain with no robot, no ROS install and
no simulator — which is the property the adapter was shaped for. If these
needed a robot they would not run in CI, and the behaviours they cover
(cadence, going quiet, declining, refusing) are exactly the ones nobody
exercises by hand.

One domain per class, because these lanes are keyed and two classes sharing a
domain would read each other's robot.
"""
from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path
from typing import Optional

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from spatialdds_idl18.builtin import Time                     # noqa: E402
from spatialdds_idl18.spatial.core import (                   # noqa: E402
    Aabb3, FrameTransform, PoseSE3)
from spatialdds_idl18.spatial.owm import (                    # noqa: E402
    Entity, ModelCommand, ModelPose)

from spatialdds_ros import transport as tt                    # noqa: E402
from spatialdds_ros.adapter import (                          # noqa: E402
    NAV_STATUS_KEY, RobotAdapter, already_published)
from spatialdds_ros.config import Anchor, Config              # noqa: E402
from spatialdds_ros.qos import (                              # noqa: E402
    MODEL_COMMAND, MODEL_FAST, MODEL_LATCHED)
from spatialdds_ros.tempo import LATCH_EVERY_N                # noqa: E402
from spatialdds_ros.topics import Topics                      # noqa: E402

ENTITY = "ent:robot:r1"
SCENE = "bare-scene"
ANCHOR = Anchor(30.28396857092828, -97.73974829101985, 145.083)


def config(**overrides) -> Config:
    base = dict(scene=SCENE, entity_id=ENTITY, frame_id="map/bare-scene")
    base.update(overrides)
    return Config(**base)


def participant(domain: int):
    try:
        from cyclonedds.domain import DomainParticipant
        return DomainParticipant(domain)
    except Exception as exc:                                  # noqa: BLE001
        raise unittest.SkipTest(f"DDS-UNAVAILABLE: {exc}")


class _OnABus(unittest.TestCase):
    DOMAIN = 90
    CONFIG_KW: dict = {}

    def setUp(self):
        self.participant = participant(self.DOMAIN)
        self.config = config(**self.CONFIG_KW)
        self.adapter = RobotAdapter(self.participant, self.config)
        topics = Topics(SCENE)
        self.entities = tt.make_reader(
            self.participant, topics.entity, Entity, MODEL_LATCHED.name)
        self.poses = tt.make_reader(
            self.participant, topics.pose, ModelPose, MODEL_FAST.name)
        time.sleep(0.5)
        tt.take_samples(self.entities)
        tt.take_samples(self.poses)

    def drain(self, reader, seconds: float = 0.6):
        got, deadline = [], time.time() + seconds
        while time.time() < deadline:
            got.extend(tt.take_samples(reader))
            time.sleep(0.02)
        return got

    def mine(self, reader, seconds: float = 0.6):
        return [s for s in self.drain(reader, seconds)
                if s.entity_id == ENTITY]

    @staticmethod
    def nav_status(entity) -> Optional[str]:
        for kv in entity.properties or []:
            if kv.key == NAV_STATUS_KEY:
                return kv.value
        return None


class WhatItPublishes(_OnABus):
    DOMAIN = 90

    def test_the_first_pose_publishes_a_complete_entity(self):
        self.adapter.observed(1.0, 2.0, 0.0)
        seen = self.mine(self.entities)
        self.assertTrue(seen, "the robot should appear on its first pose")
        robot = seen[-1]
        self.assertEqual(robot.basis.name, "OBSERVED")
        self.assertEqual(robot.layer.name, "FAST")
        self.assertEqual(robot.state.name, "ACTIVE")
        self.assertEqual(robot.source_id, self.config.effective_source_id)
        self.assertEqual(list(robot.type_uris), [self.config.type_uri])
        self.assertEqual(robot.frame_ref.fqn, self.config.frame_id)

    def test_it_appears_at_once_not_on_the_latch_cadence(self):
        """Appearing is a change of state, not of position."""
        self.adapter.observed(1.0, 2.0, 0.0)
        self.assertTrue(self.mine(self.entities, 0.4))

    def test_the_frame_declares_metres_on_the_wire(self):
        self.adapter.observed(1.0, 2.0, 0.0)
        ref = self.mine(self.entities)[-1].frame_ref
        self.assertTrue(ref.has_scale)
        self.assertEqual(ref.scale_status.name, "SCALE_DECLARED")
        self.assertEqual(ref.meters_per_unit, 1.0)

    def test_it_claims_no_extent(self):
        """A robot locates itself; it does not legislate a region. An extent
        would be read as keep-out by anything applying the DECLARED rule."""
        self.adapter.observed(1.0, 2.0, 0.0)
        self.assertFalse(self.mine(self.entities)[-1].has_extent)

    def test_every_pose_goes_out_on_the_fast_lane(self):
        """
        Drained between writes, because the lane is BEST_EFFORT KEEP_LAST(1):
        four poses written in a tight loop reach a reader as one, and that is
        the profile working as chosen. Asserting "four arrive" would be
        asserting against the QoS.
        """
        got = []
        for i in range(4):
            self.adapter.observed(1.0 + i * 0.1, 2.0, 0.0)
            got.extend(self.mine(self.poses, 0.25))
        self.assertGreaterEqual(len(got), 3)
        self.assertAlmostEqual(got[-1].pose.t[0], 1.3, places=3)

    def test_the_latched_record_is_not_rewritten_for_every_pose(self):
        self.adapter.observed(1.0, 2.0, 0.0)       # the first, always sent
        self.drain(self.entities, 0.4)
        for i in range(LATCH_EVERY_N - 2):
            self.adapter.observed(1.0 + i * 0.1, 2.0, 0.0)
        self.assertEqual(self.mine(self.entities, 0.4), [],
                         "the expensive record should not follow every pose")

    def test_the_ground_height_comes_from_configuration(self):
        adapter = RobotAdapter(self.participant, config(ground_z=-0.84))
        adapter.observed(1.0, 2.0, 0.0)
        self.assertAlmostEqual(self.mine(self.entities)[-1].pose.t[2], -0.84)


class GoingQuiet(_OnABus):
    DOMAIN = 91

    def test_silence_is_reported_as_unobserved_with_a_reason(self):
        """
        Not disposed. Disposal says the robot is gone; silence only licenses
        "this is where it was last seen, not where it is".
        """
        self.adapter.observed(1.0, 2.0, 0.0)
        self.drain(self.entities, 0.4)
        note = self.adapter.tick(now=time.time() + 60.0)
        self.assertIsNotNone(note)
        self.assertIn("unobserved", note)
        seen = self.mine(self.entities)
        self.assertTrue(seen)
        self.assertEqual(seen[-1].state.name, "UNOBSERVED")
        self.assertIn("last seen", seen[-1].state_reason)
        self.assertEqual(self.adapter.state, "UNOBSERVED")

    def test_the_last_pose_is_kept_not_zeroed(self):
        """Where it was last seen is the only thing we know; a zeroed pose
        would be a claim that it is at the frame origin."""
        self.adapter.observed(7.0, 8.0, 0.0)
        self.drain(self.entities, 0.4)
        self.adapter.tick(now=time.time() + 60.0)
        seen = self.mine(self.entities)
        self.assertAlmostEqual(seen[-1].pose.t[0], 7.0)
        self.assertAlmostEqual(seen[-1].pose.t[1], 8.0)

    def test_coming_back_republishes_at_once(self):
        self.adapter.observed(1.0, 2.0, 0.0)
        self.adapter.tick(now=time.time() + 60.0)
        self.drain(self.entities, 0.4)
        self.adapter.observed(1.1, 2.0, 0.0)
        seen = self.mine(self.entities, 0.4)
        self.assertTrue(seen, "coming back is a change of state, sent at once")
        self.assertEqual(seen[-1].state.name, "ACTIVE")


class ARefusalReachesWhoeverAsked(_OnABus):
    DOMAIN = 92

    def test_a_refusal_rides_the_latched_record(self):
        self.adapter.observed(1.0, 2.0, 0.0)
        self.drain(self.entities, 0.4)
        self.adapter.refused("goal (9, 9) abandoned: it enters declared water")
        seen = self.mine(self.entities)
        self.assertTrue(seen, "the refusal republishes at once, off the "
                              "cadence — the asker should not wait four poses")
        self.assertIn("abandoned", self.nav_status(seen[-1]) or "")

    def test_the_refusal_keeps_the_pose_it_was_refused_at(self):
        """Where it stopped is part of the reason."""
        self.adapter.observed(12.5, -3.25, 0.0)
        self.drain(self.entities, 0.4)
        self.adapter.refused("blocked")
        seen = self.mine(self.entities)
        self.assertAlmostEqual(seen[-1].pose.t[0], 12.5)
        self.assertAlmostEqual(seen[-1].pose.t[1], -3.25)

    def test_an_ordinary_record_carries_no_refusal(self):
        """The field appears because something was refused, not by default."""
        self.adapter.observed(1.0, 2.0, 0.0)
        self.assertIsNone(self.nav_status(self.mine(self.entities)[-1]))

    def test_a_refusal_before_the_first_pose_publishes_nothing(self):
        """Announcing a robot by complaining about it is not an introduction."""
        self.adapter.refused("blocked before any pose was reported")
        self.assertEqual(self.mine(self.entities, 0.4), [])


class _Navigator:
    def __init__(self, unavailable: Optional[str] = None):
        self.unavailable = unavailable
        self.sent: list = []

    def available(self) -> Optional[str]:
        return self.unavailable

    def send(self, x: float, y: float) -> str:
        self.sent.append((x, y))
        return f"the navigator accepted ({x:.2f}, {y:.2f})"


def goto(subject: str = ENTITY, *, has_pose: bool = True,
         x: float = 5.0, y: float = 6.0, verb: str = "goto",
         requester: str = "test") -> ModelCommand:
    now = time.time()
    return ModelCommand(
        command_id=f"cmd:test:{int(now * 1000)}", verb=verb,
        subject_id=subject, reason="", requester_id=requester,
        has_pose=has_pose, pose=PoseSE3(t=[x, y, 0.0], q=[0.0, 0.0, 0.0, 1.0]),
        has_extent=False,
        extent=Aabb3(min_xyz=[0.0, 0.0, 0.0], max_xyz=[0.0, 0.0, 0.0]),
        stamp=Time(sec=int(now), nanosec=0))


class WhatItWillAndWillNotAccept(_OnABus):
    DOMAIN = 93

    def test_somebody_elses_subject_gets_silence(self):
        """A refusal from the wrong owner reads as the system disagreeing with
        itself."""
        self.adapter.observed(1.0, 2.0, 0.0)
        self.assertIsNone(self.adapter.handle_command(goto("ent:duck:east")))

    def test_an_unknown_verb_gets_silence(self):
        self.adapter.observed(1.0, 2.0, 0.0)
        self.assertIsNone(self.adapter.handle_command(goto(verb="pirouette")))

    def test_a_goto_with_no_pose_is_declined_with_a_reason(self):
        """A zeroed pose is a well-formed request to drive to the origin, so
        declining is the only reading that cannot be mistaken for obedience."""
        self.adapter.observed(1.0, 2.0, 0.0)
        note = self.adapter.handle_command(goto(has_pose=False))
        self.assertIn("declined", note)
        self.assertIn("no pose", note)

    def test_a_goto_before_any_pose_is_declined(self):
        note = self.adapter.handle_command(goto())
        self.assertIn("nothing to navigate from", note)

    def test_a_goto_while_unobserved_is_declined(self):
        self.adapter.observed(1.0, 2.0, 0.0)
        self.adapter.tick(now=time.time() + 60.0)
        note = self.adapter.handle_command(goto())
        self.assertIn("nothing to navigate from", note)

    def test_a_goto_is_declined_when_the_navigator_will_not_take_it(self):
        """
        The other half of "nothing to navigate from": the robot may be
        reporting its pose perfectly while the thing that would drive it is
        not accepting goals. Accepting here would leave a requester watching a
        still robot with no way to learn why.
        """
        navigator = _Navigator(unavailable="nav2 is not accepting goals")
        adapter = RobotAdapter(self.participant, self.config, navigator)
        adapter.observed(1.0, 2.0, 0.0)
        note = adapter.handle_command(goto())
        self.assertIn("declined", note)
        self.assertIn("not accepting goals", note)
        self.assertEqual(navigator.sent, [], "nothing should have been sent")

    def test_an_accepted_goto_reaches_the_navigator(self):
        navigator = _Navigator()
        adapter = RobotAdapter(self.participant, self.config, navigator)
        adapter.observed(1.0, 2.0, 0.0)
        note = adapter.handle_command(goto(x=5.0, y=6.0))
        self.assertEqual(navigator.sent, [(5.0, 6.0)])
        self.assertIn("asked by test", note)
        self.assertEqual(adapter.goal, (5.0, 6.0))

    def test_what_is_reported_is_what_the_navigator_accepted(self):
        """If it rounds, clamps or refuses, that is what belongs in the log."""
        class Rounding(_Navigator):
            def send(self, x, y):
                return f"the navigator accepted ({round(x)}, {round(y)})"
        adapter = RobotAdapter(self.participant, self.config, Rounding())
        adapter.observed(1.0, 2.0, 0.0)
        note = adapter.handle_command(goto(x=5.4, y=6.6))
        self.assertIn("(5, 7)", note)

    def test_commands_arrive_over_the_bus(self):
        """The lane itself, not just the handler."""
        writer = tt.make_writer(self.participant, Topics(SCENE).command,
                                ModelCommand, MODEL_COMMAND.name)
        navigator = _Navigator()
        adapter = RobotAdapter(self.participant, self.config, navigator)
        adapter.observed(1.0, 2.0, 0.0)
        time.sleep(0.4)
        writer.write(goto(x=3.0, y=4.0))
        notes = []
        deadline = time.time() + 2.0
        while time.time() < deadline and not notes:
            notes = adapter.poll_commands()
            time.sleep(0.05)
        self.assertTrue(notes, "the command should arrive over the lane")
        self.assertEqual(navigator.sent, [(3.0, 4.0)])


class TheAnchorsSilence(unittest.TestCase):
    """
    Acceptance item 4: no anchor configured means no `FrameTransform`
    published, asserted.

    Read off the bus rather than from the adapter's own state, because the
    claim is about what a consumer sees. A reader on the frames lane that
    receives nothing is the whole assertion.
    """

    DOMAIN = 94

    def _frames_reader(self, p):
        return tt.make_reader(p, Topics(SCENE).frame_transform,
                              FrameTransform, MODEL_LATCHED.name)

    def test_without_an_anchor_nothing_is_published_on_the_frames_lane(self):
        p = participant(self.DOMAIN)
        adapter = RobotAdapter(p, config())
        reader = self._frames_reader(p)
        time.sleep(0.5)
        tt.take_samples(reader)

        self.assertIsNone(adapter.publish_georeference(),
                          "with no anchor there is nothing to report")
        adapter.observed(1.0, 2.0, 0.0)

        got, deadline = [], time.time() + 1.5
        while time.time() < deadline:
            got.extend(tt.take_samples(reader))
            time.sleep(0.05)
        self.assertEqual(
            [t.child_ref.fqn for t in got], [],
            "a consumer must see no georeference at all — not an empty one, "
            "and not one at (0, 0)")

    def test_with_an_anchor_the_transform_is_published_and_correct(self):
        """
        The control. Without it the assertion above is satisfied by an adapter
        that never publishes a transform under any circumstances.
        """
        p = participant(self.DOMAIN + 1)
        adapter = RobotAdapter(p, config(anchor=ANCHOR))
        reader = self._frames_reader(p)
        time.sleep(0.5)
        tt.take_samples(reader)

        note = adapter.publish_georeference()
        self.assertIsNotNone(note)
        self.assertIn("georeference", note)

        got, deadline = [], time.time() + 2.0
        while time.time() < deadline and not got:
            got.extend(tt.take_samples(reader))
            time.sleep(0.05)
        self.assertTrue(got, "the transform should be on the lane")
        transform = got[-1]
        self.assertEqual(transform.child_ref.fqn, "map/bare-scene")
        self.assertEqual(transform.parent_ref.fqn, "earth-fixed")

        from spatialdds_ros.frames import geodetic_of
        lat, lon, _ = geodetic_of(transform.T_parent_child.t)
        self.assertAlmostEqual(lat, ANCHOR.lat_deg, places=6)
        self.assertAlmostEqual(lon, ANCHOR.lon_deg, places=6)


class OneWriterPerKey(unittest.TestCase):
    DOMAIN = 96

    def test_an_unclaimed_key_is_free(self):
        p = participant(self.DOMAIN)
        self.assertIsNone(already_published(p, config(), settle=1.0))

    def test_an_existing_publisher_is_found_and_named(self):
        """
        Two adapters on one robot would put it in two places with nothing
        reporting a conflict — a reader would just see it teleport. So the
        guard reports *who*, which is the thing an operator needs.
        """
        p = participant(self.DOMAIN + 1)
        adapter = RobotAdapter(p, config())
        adapter.observed(1.0, 2.0, 0.0)
        time.sleep(0.5)
        found = already_published(p, config(), settle=2.5)
        self.assertEqual(found, config().effective_source_id)


if __name__ == "__main__":
    unittest.main()
