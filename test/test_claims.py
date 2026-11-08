#!/usr/bin/env python3
"""
The claims layer: whose claims are drawn, until when, and where.

Every claim here is real traffic on a real bus: an `ObstaclePublisher` on its
own participant, a raw relationship writer, or a hazard asserter in its own
process that the test SIGKILLs. What is asserted is what the layer emits,
its picture and its mask, which is what `robot_node` hands nav2; the nav2
half is SpatialDDS-demo's tier, Stage C of the claims batch.

Three rules carry the whole layer, and each has a test that fails when the
rule is removed (`docs/failability.md` §6 lists the plants):

* trust: an untrusted asserter, or a trusted one on a basis or predicate its
  rule does not name, changes nothing;
* freshness: an OBSERVED claim whose asserter's liveliness ends leaves the
  picture, is kept, and is not retired; a DECLARED one stands;
* geometry: the subject's oriented box where it carries one, else its Aabb3.
"""
from __future__ import annotations

import itertools
import math
import signal
import subprocess
import sys
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from spatialdds_idl18.oarc_owm import (                        # noqa: E402
    Relationship, RelationshipLifecycle)
from spatialdds_idl18.spatial.common import KV                 # noqa: E402
from spatialdds_idl18.spatial.core import Aabb3, PoseSE3       # noqa: E402
from spatialdds_idl18.spatial.owm import (                     # noqa: E402
    Basis, Entity, LifecycleState, ModelLayer)

from spatialdds_ros import transport as tt                     # noqa: E402
from spatialdds_ros.claims import (                            # noqa: E402
    ClaimsLayer, MaskCanvas, Region, oriented_box)
from spatialdds_ros.config import (                            # noqa: E402
    Config, ConfigError, TrustRule)
from spatialdds_ros.frames import now_time, robot_frame        # noqa: E402
from spatialdds_ros.perception import ObstaclePublisher        # noqa: E402
from spatialdds_ros.qos import MODEL_LATCHED                   # noqa: E402
from spatialdds_ros.topics import Topics                       # noqa: E402

SCENE = "bare-scene"
FRAME = "map/bare-scene"
ME = "ent:robot:b"
SPOTTER = "svc:hazard:spotter"
STRANGER = "svc:hazard:stranger"
INTERSECTS = "http://www.opengis.net/ont/geosparql#sfIntersects"
WALKWAY = "ent:venue:walkway"


def participant(domain: int):
    try:
        from cyclonedds.domain import DomainParticipant
        return DomainParticipant(domain)
    except Exception as exc:                                  # noqa: BLE001
        raise unittest.SkipTest(f"DDS-UNAVAILABLE: {exc}")


def me(**overrides) -> Config:
    base = dict(scene=SCENE, entity_id=ME, frame_id=FRAME, claims=True,
                claims_trust=[f"{SPOTTER} OBSERVED",
                              f"svc:gate:ctl DECLARED"],
                claims_shoulder_m=0.3)
    base.update(overrides)
    return Config(**base)


def asserter(source: str) -> Config:
    return Config(scene=SCENE, entity_id=f"ent:hazard:{source.split(':')[-1]}",
                  frame_id=FRAME, source_id=source)


#: **One domain per test.** A latched claim written by a raw writer outlived
#: its test and reached the next test's layer on a shared domain: invisible
#: while the rule under test held, and measured when a `valid_until` plant
#: let the stale claim draw, failing a test about retirement instead of the
#: one about `valid_until`. Deleting the participants in a teardown did not
#: release it. So no two tests here share a domain.
_DOMAINS = itertools.count(160)


class _Scene(unittest.TestCase):
    CONFIG_KW: dict = {}

    def setUp(self):
        self.domain = next(_DOMAINS)
        self.mine = participant(self.domain)
        self.layer = ClaimsLayer(self.mine, me(**self.CONFIG_KW))
        self.theirs = participant(self.domain)
        time.sleep(0.3)

    def settle(self, until=None, timeout: float = 3.0):
        """Poll until `until(picture)` holds, or the timeout."""
        deadline = time.time() + timeout
        picture = None
        while time.time() < deadline:
            self.layer.poll()
            picture = self.layer.picture(time.time())
            if until is None or until(picture):
                break
            time.sleep(0.05)
        return picture

    def drawn(self, picture):
        return sorted(r.claim_id for r in picture.regions)


class WhoseClaimsAreDrawn(_Scene):
    """The filter, demonstrated on traffic and not on a config object."""

    def test_a_trusted_observed_claim_is_drawn_and_a_strangers_is_not(self):
        spotter = ObstaclePublisher(self.theirs, asserter(SPOTTER))
        stranger = ObstaclePublisher(self.theirs, asserter(STRANGER))
        spotter.observed("box", 3.0, 0.0, length=2.0, width=1.0)
        stranger.observed("box", -3.0, 0.0, length=2.0, width=1.0)
        trusted = spotter.claim("box", INTERSECTS, WALKWAY)
        untrusted = stranger.claim("box", INTERSECTS, WALKWAY)
        picture = self.settle(lambda p: p.regions and untrusted in p.skipped)
        self.assertEqual(self.drawn(picture), [trusted])
        self.assertIn("no trust rule admits it", picture.skipped[untrusted])
        self.assertEqual(picture.counts["untrusted"], 1)

    def test_a_trusted_asserter_on_a_basis_its_rule_does_not_name(self):
        """The spotter is trusted for OBSERVED; a DECLARED claim from it is
        not admitted, and the layer says which rule failed."""
        writer = tt.make_writer(self.theirs, Topics(SCENE).relationship,
                                Relationship, MODEL_LATCHED.name)
        records = tt.make_writer(self.theirs, Topics(SCENE).entity, Entity,
                                 MODEL_LATCHED.name)
        records.write(_box_record("ent:spotter:box", 3.0, 0.0))
        writer.write(_edge("rel:x:1", "ent:spotter:box", Basis.DECLARED,
                           SPOTTER))
        picture = self.settle(lambda p: "rel:x:1" in p.skipped)
        self.assertEqual(picture.regions, [])
        self.assertIn("DECLARED", picture.skipped["rel:x:1"])

    def test_a_rule_that_names_predicates_admits_only_those(self):
        self.layer = ClaimsLayer(self.mine, me(claims_trust=[
            f"{SPOTTER} OBSERVED {INTERSECTS}"]))
        spotter = ObstaclePublisher(self.theirs, asserter(SPOTTER))
        spotter.observed("box", 3.0, 0.0, length=2.0, width=1.0)
        yes = spotter.claim("box", INTERSECTS, WALKWAY)
        no = spotter.claim("box", "https://example.org/near", WALKWAY)
        picture = self.settle(lambda p: p.regions and no in p.skipped)
        self.assertEqual(self.drawn(picture), [yes])


class WhenAClaimStopsCounting(_Scene):

    def test_a_retired_claim_leaves_and_the_log_says_retired(self):
        spotter = ObstaclePublisher(self.theirs, asserter(SPOTTER))
        spotter.observed("box", 3.0, 0.0, length=2.0, width=1.0)
        rid = spotter.claim("box", INTERSECTS, WALKWAY)
        self.settle(lambda p: p.regions)
        spotter.withdraw(rid, "the box was moved off the walkway")
        notes = []
        deadline = time.time() + 3.0
        while time.time() < deadline:
            notes += self.layer.poll()
            if not self.layer.picture(time.time()).regions:
                break
            time.sleep(0.05)
        self.assertEqual(self.layer.picture(time.time()).regions, [])
        self.assertTrue(any("retired" in n and "no longer holds" in n
                            for n in notes), notes)

    def test_a_claim_past_its_valid_until_is_not_drawn(self):
        writer = tt.make_writer(self.theirs, Topics(SCENE).relationship,
                                Relationship, MODEL_LATCHED.name)
        records = tt.make_writer(self.theirs, Topics(SCENE).entity, Entity,
                                 MODEL_LATCHED.name)
        records.write(_box_record("ent:spotter:box", 3.0, 0.0))
        until = time.time() + 0.5
        writer.write(_edge("rel:x:2", "ent:spotter:box", Basis.OBSERVED,
                           SPOTTER, valid_until=until))
        self.assertTrue(self.settle(lambda p: p.regions).regions)
        late = self.layer.picture(until + 0.1)
        self.assertEqual(late.regions, [])
        self.assertIn("valid_until", late.skipped["rel:x:2"])


class AnAsserterThatDies(_Scene):
    """
    The silenced-asserter arm, in the library: SIGKILL, liveliness ends, the
    OBSERVED claim leaves the picture, and nothing anywhere reads it as a
    retirement. Measured: the lease is CycloneDDS's default, 10 s.
    """

    def test_a_killed_asserters_observed_claim_ages_out_and_is_kept(self):
        proc = subprocess.Popen(
            [sys.executable, str(REPO / "test" / "_hazard_asserter.py"),
             str(self.domain), SCENE, FRAME, SPOTTER, "3.0", "0.0", "2.0",
             "1.0", INTERSECTS, WALKWAY], stdout=subprocess.PIPE, text=True)
        try:
            line = proc.stdout.readline().split()
            self.assertEqual(line[0], "up")
            rid = line[1]
            picture = self.settle(lambda p: p.regions, timeout=5.0)
            self.assertEqual(self.drawn(picture), [rid])
        finally:
            proc.send_signal(signal.SIGKILL)
            proc.wait()
        killed = time.time()
        notes = []
        aged = None
        while time.time() - killed < 20.0:
            notes += self.layer.poll()
            picture = self.layer.picture(time.time())
            if not picture.regions:
                aged = time.time() - killed
                break
            time.sleep(0.1)
        self.assertIsNotNone(aged, "a killed asserter's OBSERVED claim must "
                                   "leave the costmap on liveliness loss")
        self.assertIn("liveliness ended", picture.skipped[rid])
        self.assertIn("Kept, not retired", picture.skipped[rid])
        self.assertEqual(picture.counts["aged_out"], 1)
        self.assertFalse(any("retired by" in n or "withdrawn" in n
                             for n in notes), notes)
        self.assertIn(rid, self.layer._claims, "the record is kept")
        print(f"\n  killed asserter: claim aged out {aged:.1f}s after the "
              f"kill, on liveliness")

    def test_a_declared_claim_stands_when_its_declarer_goes(self):
        declarer = participant(self.domain)
        writer = tt.make_writer(declarer, Topics(SCENE).relationship,
                                Relationship, MODEL_LATCHED.name)
        records = tt.make_writer(self.theirs, Topics(SCENE).entity, Entity,
                                 MODEL_LATCHED.name)
        records.write(_box_record("ent:gate:zone", 3.0, 0.0))
        writer.write(_edge("rel:gate:1", "ent:gate:zone", Basis.DECLARED,
                           "svc:gate:ctl"))
        self.assertTrue(self.settle(lambda p: p.regions).regions)
        del writer
        del declarer
        import gc
        gc.collect()
        picture = self.settle(
            lambda p: p.counts.get("drawn_unasserted"), timeout=5.0)
        self.assertEqual([r.claim_id for r in picture.regions], ["rel:gate:1"])
        self.assertTrue(picture.regions[0].unasserted)


class WhereItIsDrawn(unittest.TestCase):
    def test_an_oriented_box_is_centred_and_turned(self):
        corners = oriented_box(10.0, 5.0, math.pi / 2, 4.0, 2.0)
        xs = sorted(round(c[0], 6) for c in corners)
        ys = sorted(round(c[1], 6) for c in corners)
        self.assertEqual((xs[0], xs[-1]), (9.0, 11.0))
        self.assertEqual((ys[0], ys[-1]), (3.0, 7.0))

    def test_the_mask_marks_the_region_and_its_shoulder_only(self):
        canvas = MaskCanvas(shoulder_m=0.5, cost=50)
        box = Region("r", "s", "p", "a", "OBSERVED",
                     ((0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (0.0, 1.0)))
        mask = canvas.render([box])
        self.assertEqual(mask.value_at(1.0, 0.5), 50)
        self.assertEqual(mask.value_at(2.4, 0.5), 50)     # in the shoulder
        self.assertEqual(mask.value_at(2.53, 0.5), 0)     # beyond it
        self.assertNotIn(100, mask.data, "advice is never lethal")

    def test_a_dropped_region_is_drawn_as_gone_on_the_same_canvas(self):
        canvas = MaskCanvas(shoulder_m=0.2, cost=50)
        box = Region("r", "s", "p", "a", "OBSERVED",
                     ((0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0)))
        first = canvas.render([box])
        after = canvas.render([])
        self.assertEqual((after.origin_x, after.origin_y, after.width,
                          after.height),
                         (first.origin_x, first.origin_y, first.width,
                          first.height))
        self.assertEqual(after.marked, 0)

    def test_nothing_ever_claimed_publishes_no_mask(self):
        self.assertIsNone(MaskCanvas(shoulder_m=0.2, cost=50).render([]))

    def test_aligned_it_is_exactly_the_costmaps_grid(self):
        """nav2 paints a mask only on its costmap's grid (finding 14)."""
        from spatialdds_ros.claims import Grid
        canvas = MaskCanvas(shoulder_m=0.2, cost=50)
        grid = Grid(-30.5, -49.0, 0.1, 850, 733)
        canvas.align(grid)
        self.assertIsNone(canvas.render([]), "nothing claimed yet")
        box = Region("r", "s", "p", "a", "OBSERVED",
                     ((23.0, -23.8), (25.0, -23.8), (25.0, -22.2),
                      (23.0, -22.2)))
        mask = canvas.render([box])
        self.assertEqual((mask.origin_x, mask.origin_y, mask.resolution,
                          mask.width, mask.height),
                         (-30.5, -49.0, 0.1, 850, 733))
        self.assertEqual(mask.value_at(24.0, -23.0), 50)
        self.assertEqual(mask.value_at(24.0, -27.0), 0)
        after = canvas.render([])
        self.assertEqual((after.width, after.height), (850, 733))
        self.assertEqual(after.marked, 0, "a dropped claim is drawn as gone")


class FromTheWire(_Scene):

    def test_the_oriented_box_wins_over_the_enclosing_aabb(self):
        spotter = ObstaclePublisher(self.theirs, asserter(SPOTTER))
        spotter.observed("box", 3.0, 0.0, yaw=math.pi / 4, length=2.0,
                         width=0.4)
        spotter.claim("box", INTERSECTS, WALKWAY)
        picture = self.settle(lambda p: p.regions)
        polygon = picture.regions[0].polygon
        # The enclosing Aabb3 of a 2 x 0.4 box at 45 degrees is 1.13 m
        # square; the oriented box's corners reach 0.85 m along each axis
        # but its own width is 0.4, so its centre-to-corner span is not a
        # square's.
        sides = sorted(round(math.dist(polygon[i], polygon[(i + 1) % 4]), 3)
                       for i in range(4))
        self.assertEqual(sides, [0.4, 0.4, 2.0, 2.0])

    def test_a_subject_with_no_extent_is_skipped_with_its_reason(self):
        spotter = ObstaclePublisher(self.theirs, asserter(SPOTTER))
        spotter.observed("dot", 3.0, 0.0)
        rid = spotter.claim("dot", INTERSECTS, WALKWAY)
        picture = self.settle(lambda p: rid in p.skipped)
        self.assertIn("publishes no extent", picture.skipped[rid])


class TheTrustRule(unittest.TestCase):
    def test_parse_and_admit(self):
        rule = TrustRule.parse(f"{SPOTTER} OBSERVED,DERIVED {INTERSECTS}")
        self.assertTrue(rule.admits(SPOTTER, "DERIVED", INTERSECTS))
        self.assertFalse(rule.admits(SPOTTER, "DECLARED", INTERSECTS))
        self.assertFalse(rule.admits(STRANGER, "OBSERVED", INTERSECTS))
        self.assertFalse(rule.admits(SPOTTER, "OBSERVED", "other"))

    def test_a_basis_that_is_not_one_is_refused(self):
        with self.assertRaises(ConfigError):
            TrustRule.parse(f"{SPOTTER} OBSERVD")

    def test_claims_on_needs_trust_and_a_shoulder(self):
        with self.assertRaises(ConfigError) as caught:
            Config(scene=SCENE, entity_id=ME, frame_id=FRAME, claims=True)
        self.assertIn("claims_trust is required", str(caught.exception))
        self.assertIn("claims_shoulder_m is required", str(caught.exception))

    def test_a_lethal_cost_is_refused(self):
        with self.assertRaises(ConfigError) as caught:
            me(claims_cost=100)
        self.assertIn("lethal", str(caught.exception))


def _box_record(entity_id: str, x: float, y: float) -> Entity:
    return Entity(
        entity_id=entity_id, basis=Basis.OBSERVED,
        type_uris=["http://www.wikidata.org/entity/Q223557"],
        layer=ModelLayer.FAST, frame_ref=robot_frame(FRAME), has_pose=True,
        pose=PoseSE3(t=[x, y, 0.0], q=[0.0, 0.0, 0.0, 1.0]), has_extent=True,
        extent=Aabb3(min_xyz=[x - 1.0, y - 0.5, 0.0],
                     max_xyz=[x + 1.0, y + 0.5, 1.0]),
        properties=[KV(key="label", value=entity_id)], external_refs=[],
        content_refs=[], state=LifecycleState.ACTIVE, state_reason="",
        source_id="svc:test", stamp=now_time())


def _edge(rid: str, subject: str, basis, asserter_id: str,
          valid_until=None) -> Relationship:
    from spatialdds_idl18.builtin import Time
    zero = Time(sec=0, nanosec=0)
    until = zero
    if valid_until is not None:
        until = Time(sec=int(valid_until),
                     nanosec=int((valid_until % 1) * 1e9))
    return Relationship(
        id=rid, subject_ref=subject, predicate=INTERSECTS, object_ref=WALKWAY,
        basis=basis, asserter=asserter_id, has_confidence=False,
        confidence=0.0, derived_from=[], stamp=now_time(),
        has_valid_from=False, valid_from=zero,
        has_valid_until=valid_until is not None, valid_until=until, meta=[],
        state=RelationshipLifecycle.CREATED, state_reason="")


if __name__ == "__main__":
    unittest.main()
