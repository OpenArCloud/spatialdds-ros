#!/usr/bin/env python3
"""
Acceptance item 1: a kinematic robot through the library on a bare scene.

    entities on the bus, announce present, a goto accepted and acted on,
    the refusal path exercised

**A bare scene, not the demo's venue.** `bare-scene` has no catalogue, no
pond, no ducks, no georeference and no venue model of any kind — nothing but
this robot. That is the point of the item: the prototype's behaviour is being
shown to survive separation from the venue it was written in. A run against
`ut-austin-littlefield` would prove the library works where it was extracted
from, which is the one place it was already known to work.

**It drives `Runner`, which is what the README's command drives.** Not a
test-only harness: a scaffold that assembled the pieces itself could pass
while the documented path was broken, and the documented path is the deliverable.
"""
from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from spatialdds_idl18.builtin import Time                     # noqa: E402
from spatialdds_idl18.spatial.core import Aabb3, PoseSE3      # noqa: E402
from spatialdds_idl18.spatial.disco import Announce           # noqa: E402
from spatialdds_idl18.spatial.owm import (                    # noqa: E402
    Entity, ModelCommand, ModelPose)

from spatialdds_ros import transport as tt                    # noqa: E402
from spatialdds_ros.adapter import NAV_STATUS_KEY             # noqa: E402
from spatialdds_ros.announce import ANNOUNCE_TOPIC            # noqa: E402
from spatialdds_ros.config import Config                      # noqa: E402
from spatialdds_ros.kinematic import Bounds                   # noqa: E402
from spatialdds_ros.qos import (                              # noqa: E402
    MODEL_COMMAND, MODEL_FAST, MODEL_LATCHED)
from spatialdds_ros.standalone import Runner                  # noqa: E402
from spatialdds_ros.topics import Topics                      # noqa: E402

SCENE = "bare-scene"
ENTITY = "ent:robot:r1"
DOMAIN = 120


def goto(x: float, y: float, requester: str = "acceptance") -> ModelCommand:
    now = time.time()
    return ModelCommand(
        command_id=f"cmd:acceptance:{int(now * 1e6)}", verb="goto",
        subject_id=ENTITY, reason="acceptance run", requester_id=requester,
        has_pose=True, pose=PoseSE3(t=[x, y, 0.0], q=[0.0, 0.0, 0.0, 1.0]),
        has_extent=False,
        extent=Aabb3(min_xyz=[0.0, 0.0, 0.0], max_xyz=[0.0, 0.0, 0.0]),
        stamp=Time(sec=int(now), nanosec=0))


class ABareScene(unittest.TestCase):
    """
    One run, asserted in order, because the items are a sequence: the robot has
    to exist before a goal means anything, and has to have accepted one before
    a refusal does.
    """

    @classmethod
    def setUpClass(cls):
        try:
            from cyclonedds.domain import DomainParticipant
        except Exception as exc:                              # noqa: BLE001
            raise unittest.SkipTest(f"DDS-UNAVAILABLE: {exc}")

        cls.config = Config(scene=SCENE, entity_id=ENTITY,
                            frame_id="map/bare-scene", domain=DOMAIN)
        # Bounds so the refusal path exists without hardware. Small, and the
        # robot starts inside them.
        cls.runner = Runner(cls.config, rate_hz=20.0, start=(0.0, 0.0),
                            speed=2.0,
                            bounds=Bounds(-10.0, -10.0, 10.0, 10.0))

        # A consumer, built the way any other participant would build one —
        # from the profile names the announce advertises.
        cls.observer = DomainParticipant(DOMAIN)
        names = Topics(SCENE)
        cls.entities = tt.make_reader(cls.observer, names.entity, Entity,
                                      MODEL_LATCHED.name)
        cls.poses = tt.make_reader(cls.observer, names.pose, ModelPose,
                                   MODEL_FAST.name)
        cls.announces = tt.make_reader(cls.observer, ANNOUNCE_TOPIC, Announce,
                                       MODEL_LATCHED.name)
        cls.commands = tt.make_writer(cls.observer, names.command,
                                      ModelCommand, MODEL_COMMAND.name)
        time.sleep(0.6)

        cls.start_notes = cls.runner.start()
        cls.log: list[str] = list(cls.start_notes)
        cls.seen_entities: list = []
        cls.seen_poses: list = []

    @classmethod
    def _pump(cls, seconds: float):
        deadline = time.time() + seconds
        while time.time() < deadline:
            cls.log.extend(cls.runner.step())
            cls.seen_entities.extend(
                e for e in tt.take_samples(cls.entities)
                if e.entity_id == ENTITY)
            cls.seen_poses.extend(
                m for m in tt.take_samples(cls.poses) if m.entity_id == ENTITY)
            time.sleep(0.02)

    @staticmethod
    def _nav_status(entity):
        for kv in entity.properties or []:
            if kv.key == NAV_STATUS_KEY:
                return kv.value
        return None

    def test_1_the_robot_is_on_the_bus(self):
        self._pump(1.0)
        self.assertTrue(self.seen_entities,
                        "the robot's entity should be on the entity lane")
        robot = self.seen_entities[-1]
        self.assertEqual(robot.basis.name, "OBSERVED")
        self.assertEqual(robot.layer.name, "FAST")
        self.assertEqual(robot.state.name, "ACTIVE")
        self.assertEqual(robot.frame_ref.fqn, "map/bare-scene")
        self.assertEqual(robot.frame_ref.scale_status.name, "SCALE_DECLARED")
        self.assertTrue(self.seen_poses, "and its poses on the fast lane")
        print(f"\n  entity on the bus: {robot.entity_id} "
              f"{robot.basis.name}/{robot.layer.name} in {robot.frame_ref.fqn}")

    def test_2_the_announce_is_present_and_describes_the_lanes(self):
        got, deadline = [], time.time() + 2.0
        while time.time() < deadline and not got:
            got = [a for a in tt.take_samples(self.announces)
                   if a.service_id == self.config.effective_source_id]
            time.sleep(0.05)
        self.assertTrue(got, "the robot should be discoverable")
        announce = got[-1]
        advertised = {t.name: t.qos_profile for t in announce.topics}
        names = Topics(SCENE)
        # The advertisement has to be *true*: the profile named for each lane
        # must be the one the adapter actually used, or a consumer that builds
        # from the announce gets endpoints that never match.
        self.assertEqual(advertised.get(names.entity), MODEL_LATCHED.name)
        self.assertEqual(advertised.get(names.pose), MODEL_FAST.name)
        self.assertEqual(advertised.get(names.command), MODEL_COMMAND.name)
        self.assertEqual(announce.version, "1.8")
        print(f"  announce: {announce.service_id} as {announce.kind.name}, "
              f"{len(announce.topics)} lanes, ttl {announce.ttl_sec}s")

    def test_3_a_goto_is_accepted_and_acted_on(self):
        """Accepted *and* acted on: the robot has to actually arrive."""
        before = (self.runner.source.x, self.runner.source.y)
        self.commands.write(goto(4.0, 3.0))
        self._pump(6.0)
        accepted = [n for n in self.log if "heading for" in n]
        self.assertTrue(accepted, f"the goto should be accepted: {self.log[-6:]}")
        self.assertIn("asked by acceptance", " ".join(self.log))

        x, y = self.runner.source.x, self.runner.source.y
        self.assertAlmostEqual(x, 4.0, delta=0.2)
        self.assertAlmostEqual(y, 3.0, delta=0.2)
        self.assertNotEqual((x, y), before, "and it should have moved")

        # And the movement is visible to the consumer, not just internal.
        self.assertGreater(len(self.seen_poses), 5)
        last = self.seen_poses[-1]
        self.assertAlmostEqual(last.pose.t[0], x, delta=0.5)
        print(f"  goto (4, 3) accepted and driven to "
              f"({x:.2f}, {y:.2f}); {len(self.seen_poses)} poses observed")

    def test_4_the_refusal_path_reaches_the_asker(self):
        """
        A goal outside the bounds: accepted honestly, then given up on, with
        the reason arriving on the latched record where whoever asked can read
        it. The *consumer* reads it — that is the whole point.
        """
        self.seen_entities.clear()
        self.commands.write(goto(50.0, 0.0, requester="acceptance-refusal"))
        self._pump(8.0)

        abandoned = [n for n in self.log if "abandoned" in n]
        self.assertTrue(abandoned,
                        f"the goal should have been given up on: "
                        f"{self.log[-6:]}")

        with_status = [e for e in self.seen_entities
                       if self._nav_status(e) is not None]
        self.assertTrue(
            with_status,
            "the refusal must reach a consumer on the entity lane, not only "
            "the runner's log")
        status = self._nav_status(with_status[-1])
        self.assertIn("abandoned", status)
        self.assertIn("bounds", status)
        self.assertLessEqual(self.runner.source.x, 10.0,
                             "and it stopped at the boundary")
        print(f"  refusal reached the consumer: {status[:72]}...")

    def test_5_no_anchor_means_no_georeference_on_the_bus(self):
        """
        The ruled silence, observed from the consumer's side on a real run.
        `bare-scene` has no anchor, so a client that wants to place this robot
        on a globe cannot — and sees nothing rather than something wrong.
        """
        reader = tt.make_reader(
            self.observer, Topics(SCENE).frame_transform,
            __import__("spatialdds_idl18.spatial.core", fromlist=["x"]
                       ).FrameTransform, MODEL_LATCHED.name)
        got, deadline = [], time.time() + 1.5
        while time.time() < deadline:
            got.extend(tt.take_samples(reader))
            time.sleep(0.05)
        self.assertEqual([t.child_ref.fqn for t in got], [])
        self.assertFalse(self.config.publishes_georeference)
        notes = " ".join(self.start_notes)
        self.assertIn("publishes no FrameTransform", notes)
        # And the note must not over-claim on another participant's behalf.
        # It said "nothing earth-references them" until 2026-10-19, which was
        # false in the first real deployment — a venue owned that claim, and
        # anchorless was correct *because* it did.
        self.assertNotIn("nothing earth-references", notes)
        self.assertIn("Another participant may own that claim", notes)
        print("  no anchor: nothing on the frames lane, and the mode was "
              "stated at startup without claiming what others publish")

    def test_6_the_whole_run_reported_itself(self):
        """
        The log is part of the deliverable: an operator watching this should be
        able to tell what happened without reading the source.
        """
        joined = " ".join(self.log)
        for expected in ("scene bare-scene", "announce:", "heading for",
                         "abandoned"):
            self.assertIn(expected, joined,
                          f"the run should have reported {expected!r}")
        print(f"  {len(self.log)} log lines across the run")


if __name__ == "__main__":
    unittest.main()
