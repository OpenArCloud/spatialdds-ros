#!/usr/bin/env python3
"""
Awareness: what becomes an obstacle, when it stops, and in which frame.

Every peer here is real traffic on a real bus — another `RobotAdapter` on its
own participant, or a writer of raw records — read by `Awareness` exactly as
the node reads it. Time is driven through `now`, which is the receive time the
consumer measures its windows on, so the staleness rules are exercised without
sleeping through them.

**What is asserted is what the consumer emits, not a costmap.** For
self-exclusion in particular a costmap is the wrong witness: nav2's
`footprint_clearing_enabled` clears the cells under the robot, which is
exactly where a self-obstacle would land, so a costmap-level check passes with
the exclusion removed (ruled 2026-10-21, `docs/awareness-census.md` §2). The
emitted picture has no such blind spot.
"""
from __future__ import annotations

import gc
import math
import sys
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from spatialdds_idl18.spatial.common import KV                # noqa: E402
from spatialdds_idl18.spatial.core import (                   # noqa: E402
    Aabb3, FrameTransform, PoseSE3)
from spatialdds_idl18.spatial.owm import (                    # noqa: E402
    Basis, Entity, LifecycleState, ModelLayer, ModelPose)

from spatialdds_ros import transport as tt                    # noqa: E402
from spatialdds_ros.adapter import RobotAdapter               # noqa: E402
from spatialdds_ros.config import Anchor, Config              # noqa: E402
from spatialdds_ros.frames import (                           # noqa: E402
    georeference, now_time, robot_frame)
from spatialdds_ros.qos import MODEL_FAST, MODEL_LATCHED      # noqa: E402
from spatialdds_ros.topics import Topics                      # noqa: E402

SCENE = "bare-scene"
FRAME = "map/bare-scene"
ME = "ent:robot:b"
PEER = "ent:robot:a"
ANCHOR = Anchor(30.28396857092828, -97.73974829101985, 145.083)
#: 100 m east of ANCHOR, by the small-angle rule.
EAST = Anchor(ANCHOR.lat_deg, ANCHOR.lon_deg + 100.0 / (
    111_320.0 * math.cos(math.radians(ANCHOR.lat_deg))), ANCHOR.height_m)


def participant(domain: int):
    try:
        from cyclonedds.domain import DomainParticipant
        return DomainParticipant(domain)
    except Exception as exc:                                  # noqa: BLE001
        raise unittest.SkipTest(f"DDS-UNAVAILABLE: {exc}")


def me(**overrides) -> Config:
    base = dict(scene=SCENE, entity_id=ME, frame_id=FRAME, awareness=True,
                awareness_range_m=8.0)
    base.update(overrides)
    return Config(**base)


def record(entity_id: str, *, frame: str = FRAME, layer=ModelLayer.FAST,
           state=LifecycleState.ACTIVE, xy=(3.0, 0.0), extent=None,
           type_uri="http://www.wikidata.org/entity/Q4810574",
           reason="") -> Entity:
    box = extent or ((0.0, 0.0), (0.0, 0.0))
    return Entity(
        entity_id=entity_id, basis=Basis.OBSERVED, type_uris=[type_uri],
        layer=layer, frame_ref=robot_frame(frame), has_pose=True,
        pose=PoseSE3(t=[xy[0], xy[1], 0.0], q=[0.0, 0.0, 0.0, 1.0]),
        has_extent=extent is not None,
        extent=Aabb3(min_xyz=[box[0][0], box[0][1], 0.0],
                     max_xyz=[box[1][0], box[1][1], 1.0]),
        properties=[KV(key="label", value=entity_id)], external_refs=[],
        content_refs=[], state=state, state_reason=reason,
        source_id="svc:test", stamp=now_time())


class _Scene(unittest.TestCase):
    """This robot (B) with awareness, and a second participant for peers."""
    DOMAIN = 100
    CONFIG_KW: dict = {}

    def setUp(self):
        from spatialdds_ros.awareness import Awareness
        self.mine = participant(self.DOMAIN)
        self.config = me(**self.CONFIG_KW)
        self.adapter = RobotAdapter(self.mine, self.config)
        self.awareness = Awareness(self.mine, self.config)
        self.theirs = participant(self.DOMAIN)
        topics = Topics(SCENE)
        self.records = tt.make_writer(
            self.theirs, topics.entity, Entity, MODEL_LATCHED.name)
        self.poses = tt.make_writer(
            self.theirs, topics.pose, ModelPose, MODEL_FAST.name)
        self.frames = tt.make_writer(
            self.theirs, topics.frame_transform, FrameTransform,
            MODEL_LATCHED.name)
        time.sleep(0.5)
        self.t = time.time()

    def pose(self, entity_id: str, xy, when: float) -> None:
        """A pose from a peer, received at `when`."""
        self.poses.write(ModelPose(
            entity_id=entity_id,
            pose=PoseSE3(t=[xy[0], xy[1], 0.0], q=[0.0, 0.0, 0.0, 1.0]),
            source_id="svc:test", stamp=now_time()))
        self.settle(when)

    def settle(self, when: float) -> None:
        time.sleep(0.05)
        self.awareness.poll(now=when)

    def drawn(self, when: float, robot=(0.0, 0.0)):
        return self.awareness.picture(robot, now=when)

    def ids(self, picture):
        return [o.entity_id for o in picture.obstacles]

    def heard_at_ten_hertz(self, entity_id: str, xy, seconds: float = 2.0):
        """A peer's record, then its poses at 10 Hz. Returns the last time."""
        self.records.write(record(entity_id, xy=xy))
        when = self.t
        for k in range(int(seconds * 10)):
            when = self.t + k * 0.1
            self.pose(entity_id, xy, when)
        return when


class ItNeverAvoidsItself(_Scene):
    DOMAIN = 100

    def test_its_own_entity_is_heard_and_not_drawn(self):
        """
        The robot's own record reaches its own reader — the census measured
        that — so the exclusion has something to exclude. The peer beside it
        is drawn, which is what makes the empty-of-self result mean
        something.
        """
        when = self.t
        for k in range(10):
            when = self.t + k * 0.1
            self.adapter.observed(0.0, 0.0, 0.0, now=when)
            self.pose(PEER, (3.0, 0.0), when)
        self.records.write(record(PEER))
        self.settle(when)
        picture = self.drawn(when)
        self.assertEqual(picture.counts["self"], 1,
                         "this robot's own record should have arrived")
        self.assertNotIn(ME, self.ids(picture))
        self.assertIn(PEER, self.ids(picture))
        for obstacle in picture.obstacles:
            for x, y in obstacle.points:
                self.assertGreater(math.hypot(x, y), 1.0,
                                   "nothing is drawn where this robot is")


class WhatIsDrawn(_Scene):
    DOMAIN = 101

    def test_a_peer_is_a_disc_at_its_pose(self):
        when = self.heard_at_ten_hertz(PEER, (3.0, 1.0))
        (obstacle,) = self.drawn(when).obstacles
        self.assertAlmostEqual(obstacle.x, 3.0, places=6)
        self.assertAlmostEqual(obstacle.y, 1.0, places=6)
        radius = max(math.dist(p, (3.0, 1.0)) for p in obstacle.points)
        self.assertAlmostEqual(radius, self.config.peer_radius_m, delta=0.05)

    def test_an_extent_wins_over_the_default_radius(self):
        self.records.write(record(PEER, xy=(3.0, 0.0),
                                  extent=((2.0, -1.0), (4.0, 1.0))))
        for k in range(10):
            self.pose(PEER, (3.0, 0.0), self.t + k * 0.1)
        (obstacle,) = self.drawn(self.t + 0.9).obstacles
        xs = [p[0] for p in obstacle.points]
        ys = [p[1] for p in obstacle.points]
        self.assertAlmostEqual(min(xs), 2.0, places=6)
        self.assertAlmostEqual(max(ys), 1.0, places=6)

    def test_beyond_the_range_is_not_drawn(self):
        when = self.heard_at_ten_hertz(PEER, (20.0, 0.0))
        picture = self.drawn(when)
        self.assertEqual(picture.obstacles, [])
        self.assertEqual(picture.counts["beyond_range"], 1)

    def test_nothing_drawn_reaches_the_clearing_ring(self):
        """A mark on the ring is in the cell a clearing ray ends in, which
        nav2 does not clear."""
        from spatialdds_ros.awareness import RIM_M
        when = self.heard_at_ten_hertz(PEER, (7.9, 0.0))
        for obstacle in self.drawn(when).obstacles:
            for p in obstacle.points:
                self.assertLessEqual(math.hypot(*p),
                                     self.config.awareness_range_m - RIM_M)

    def test_scenery_is_not_drawn(self):
        """A DECLARED region is the keep-out filter's, and a SLOW thing is
        not moving: only FAST entities are peers."""
        self.records.write(record("ent:pond", layer=ModelLayer.SLOW,
                                  extent=((1.0, 1.0), (2.0, 2.0))))
        when = self.heard_at_ten_hertz(PEER, (3.0, 0.0))
        self.pose("ent:pond", (1.5, 1.5), when)
        picture = self.drawn(when)
        self.assertEqual(self.ids(picture), [PEER])
        self.assertEqual(picture.counts["not_fast"], 1)


class TheTypeAllowlist(_Scene):
    DOMAIN = 102
    CONFIG_KW = {"awareness_types": ("urn:x:forklift",)}

    def test_only_listed_types_are_drawn(self):
        self.records.write(record("ent:forklift", type_uri="urn:x:forklift",
                                  xy=(2.0, 2.0)))
        when = self.heard_at_ten_hertz(PEER, (3.0, 0.0))
        self.pose("ent:forklift", (2.0, 2.0), when)
        picture = self.drawn(when)
        self.assertEqual(self.ids(picture), ["ent:forklift"])
        self.assertEqual(picture.counts["not_allowed_type"], 1)


class WhenAPeerStopsBeingAnObstacle(_Scene):
    """
    The learned-rate rule, and the two kinds of going away it has to tell
    apart.
    """
    DOMAIN = 103

    def test_a_quiet_peer_fades_after_its_window_and_is_kept(self):
        last = self.heard_at_ten_hertz(PEER, (3.0, 0.0))
        self.assertEqual(self.ids(self.drawn(last + 0.5)), [PEER],
                         "inside the window it is still drawn")
        picture = self.drawn(last + 1.5)
        self.assertEqual(picture.obstacles, [])
        self.assertIn("past its 1.0s window", picture.skipped[PEER])
        self.assertIn("record is kept", picture.skipped[PEER])
        # And it comes back on its next pose, with no rediscovery: the
        # record was never purged (§2.14).
        self.pose(PEER, (3.0, 0.0), last + 2.0)
        self.assertEqual(self.ids(self.drawn(last + 2.0)), [PEER])

    def test_a_slow_peer_earns_a_longer_window(self):
        """The window follows the peer's own cadence: a thing heard every
        two seconds is not gone after one."""
        self.records.write(record(PEER))
        for k in range(6):
            self.pose(PEER, (3.0, 0.0), self.t + 2.0 * k)
        last = self.t + 10.0
        self.assertEqual(self.ids(self.drawn(last + 3.0)), [PEER])
        self.assertEqual(self.drawn(last + 7.0).obstacles, [])

    def test_an_unobserved_record_is_dropped_at_once(self):
        last = self.heard_at_ten_hertz(PEER, (3.0, 0.0))
        self.records.write(record(PEER, state=LifecycleState.UNOBSERVED,
                                  reason="no pose for 3.0s"))
        self.settle(last)
        picture = self.drawn(last)
        self.assertEqual(picture.obstacles, [])
        self.assertIn("UNOBSERVED", picture.skipped[PEER])

    def test_a_disposed_peer_is_dropped(self):
        last = self.heard_at_ten_hertz(PEER, (3.0, 0.0))
        self.records.dispose(record(PEER))
        self.settle(last)
        picture = self.drawn(last)
        self.assertEqual(picture.obstacles, [])
        self.assertIn("disposed", picture.skipped[PEER])


class APeerThatDies(_Scene):
    """
    Died, and quiet-but-latched, told apart (census §3).

    A crashed peer stops sending, so it drops out on its learned window as
    *quiet*, long before the middleware notices; when the lease does expire
    the reason becomes *writer gone*. It is never *disposed*: that would be
    the middleware calling a crash a removal, which §2.14 forbids, and which
    CycloneDDS's default writer QoS did until `qos.py` turned it off.
    """
    DOMAIN = 104

    def test_a_killed_peer_fades_on_its_window_then_reads_as_gone(self):
        import signal
        import subprocess
        proc = subprocess.Popen(
            [sys.executable, str(REPO / "test" / "_peer_robot.py"),
             str(self.DOMAIN), PEER, "3.0", "0.0"],
            stdout=subprocess.PIPE, text=True)
        try:
            self.assertEqual(proc.stdout.readline().strip(), "up")
            deadline = time.time() + 5.0
            while time.time() < deadline:
                self.awareness.poll()
                if self.ids(self.drawn(time.time())) == [PEER]:
                    break
                time.sleep(0.1)
            # Its cadence is learned at the rate this consumer polls — the
            # pose lane is KEEP_LAST(1), so one poll sees at most one pose —
            # so poll the way the node does, at 10 Hz, while it is learned.
            for _ in range(10):
                time.sleep(0.1)
                self.awareness.poll()
            self.assertEqual(self.ids(self.drawn(time.time())), [PEER])
        finally:
            proc.send_signal(signal.SIGKILL)
            proc.wait()
        killed = time.time()

        faded = None
        while time.time() - killed < 3.0 and faded is None:
            self.awareness.poll()
            picture = self.drawn(time.time())
            if not picture.obstacles:
                faded = time.time() - killed
                self.assertIn("window", picture.skipped[PEER])
            time.sleep(0.05)
        self.assertIsNotNone(faded, "a dead peer should fade on its window")
        self.assertLess(faded, 2.0)

        reason = ""
        while time.time() - killed < 15.0:
            self.awareness.poll()
            reason = self.drawn(time.time()).skipped.get(PEER, "")
            if "writer has gone" in reason or "disposed" in reason:
                break
            time.sleep(0.1)
        self.assertIn("writer has gone", reason,
                      "a crash is liveliness loss, not a removal")
        print(f"\n  killed peer: faded at {faded:.2f}s, read as gone at "
              f"{time.time() - killed:.1f}s")

    def test_a_peer_that_leaves_cleanly_is_gone_at_once(self):
        peer = participant(self.DOMAIN)
        robot = RobotAdapter(peer, Config(scene=SCENE, entity_id=PEER,
                                          frame_id=FRAME))
        time.sleep(0.5)
        when = self.t
        for k in range(10):
            when = self.t + k * 0.1
            robot.observed(3.0, 0.0, 0.0, now=when)
            self.settle(when)
        self.assertEqual(self.ids(self.drawn(when)), [PEER])
        del robot
        del peer
        gc.collect()
        deadline = time.time() + 5.0
        picture = self.drawn(when)
        while time.time() < deadline and picture.obstacles:
            self.settle(when)
            picture = self.drawn(when)
        self.assertEqual(picture.obstacles, [])
        self.assertIn("writer has gone", picture.skipped[PEER])


class AStationaryPeerStays(_Scene):
    """
    The AMCL gap, shown closed (ruled 2026-10-21). A peer whose pose source
    has gone quiet because it stopped — not because it died — is held by its
    own adapter and kept fresh by the floor, so it stays in the picture for
    as long as it stands there.
    """
    DOMAIN = 105

    def test_held_and_floored_it_is_drawn_throughout(self):
        peer = participant(self.DOMAIN)
        robot = RobotAdapter(peer, Config(scene=SCENE, entity_id=PEER,
                                          frame_id=FRAME))
        time.sleep(0.5)
        robot.observed(3.0, 0.0, 0.0, now=self.t)
        self.settle(self.t)
        step = 1.0 / robot.config.pose_floor_hz
        for k in range(1, 41):                       # twenty seconds still
            when = self.t + k * step
            robot.holding(now=when)
            robot.tick(now=when + 0.001)
            self.settle(when)
            self.assertEqual(self.ids(self.drawn(when)), [PEER],
                             f"dropped after {k * step:.1f}s standing still")
        self.assertEqual(robot.state, "ACTIVE")


class APeerThatCannotBePlaced(_Scene):
    """Degraded modes: skipped with the reason and counted, never placed."""
    DOMAIN = 106

    def test_an_unanchored_frame_is_skipped_with_its_reason(self):
        self.records.write(record(PEER, frame="map/elsewhere"))
        for k in range(10):
            self.pose(PEER, (3.0, 0.0), self.t + k * 0.1)
        picture = self.drawn(self.t + 0.9)
        self.assertEqual(picture.obstacles, [])
        self.assertEqual(picture.counts["unplaceable"], 1)
        self.assertIn("georeference", picture.skipped[PEER])

    def test_once_both_frames_are_georeferenced_it_is_placed(self):
        """And placed where the geography says: 100 m east."""
        self.frames.write(georeference(FRAME, ANCHOR))
        self.frames.write(georeference("map/elsewhere", EAST))
        self.records.write(record(PEER, frame="map/elsewhere",
                                  xy=(0.0, 0.0)))
        for k in range(10):
            self.pose(PEER, (0.0, 0.0), self.t + k * 0.1)
        picture = self.drawn(self.t + 0.9, robot=(98.0, 0.0))
        (obstacle,) = picture.obstacles
        self.assertAlmostEqual(obstacle.x, 100.0, delta=0.3)
        self.assertAlmostEqual(obstacle.y, 0.0, delta=0.3)

    def test_two_georeferences_that_disagree_place_nothing(self):
        self.frames.write(georeference(FRAME, ANCHOR))
        self.frames.write(georeference("map/elsewhere", EAST))
        rival = georeference("map/elsewhere", ANCHOR)
        rival.transform_id = "xf:rival"
        self.frames.write(rival)
        self.records.write(record(PEER, frame="map/elsewhere"))
        for k in range(10):
            self.pose(PEER, (0.0, 0.0), self.t + k * 0.1)
        picture = self.drawn(self.t + 0.9)
        self.assertEqual(picture.obstacles, [])
        self.assertIn("disagree", picture.skipped[PEER])


class TheCloudsGeometry(unittest.TestCase):
    def test_the_ring_is_at_the_range_with_no_gap_wider_than_a_step(self):
        from spatialdds_ros.awareness import RING_STEP_M, clearing_ring
        ring = clearing_ring(8.0)
        for p in ring:
            self.assertAlmostEqual(math.hypot(*p), 8.0, places=9)
        gap = max(math.dist(ring[i], ring[i - 1]) for i in range(len(ring)))
        self.assertLessEqual(gap, RING_STEP_M + 1e-9)

    def test_the_cloud_bytes_are_twelve_per_point_little_endian(self):
        import struct
        from spatialdds_ros.awareness import POINT_Z_M, pack_xyz
        data = pack_xyz([(1.5, -2.0), (0.25, 4.0)])
        self.assertEqual(len(data), 24)
        self.assertEqual(data, struct.pack(
            "<6f", 1.5, -2.0, POINT_Z_M, 0.25, 4.0, POINT_Z_M))

    def test_base_frame_points_follow_the_robots_heading(self):
        """A peer due east of a robot facing north is on its right."""
        from spatialdds_ros.awareness import to_base
        (x, y), = to_base([(5.0, 2.0)], 2.0, 2.0, math.pi / 2)
        self.assertAlmostEqual(x, 0.0, places=9)
        self.assertAlmostEqual(y, -3.0, places=9)


if __name__ == "__main__":
    unittest.main()
