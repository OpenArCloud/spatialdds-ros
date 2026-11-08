#!/usr/bin/env python3
"""
Perception publishing: this robot as an asserter, and what its records say.

`ObstaclePublisher` against a raw reader on a second participant, so what is
asserted is what reaches the wire: records, states, reasons, the order of a
terminal sample and its dispose, and what a reader sees when the publisher's
process is killed. The source of tracks is this file, scripted; the API is
the deliverable, not a detector.

Also here, because it is the same question asked of every writer: **the
autodispose audit.** Every writer this library opens is read back off the
wire objects, not off the profile table, and must carry
`WriterDataLifecycle(autodispose=False)`.
"""
from __future__ import annotations

import gc
import signal
import subprocess
import sys
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from cyclonedds.core import InstanceState                       # noqa: E402

from spatialdds_idl18.oarc_owm import (                         # noqa: E402
    Relationship, RelationshipLifecycle)
from spatialdds_idl18.spatial.owm import (                      # noqa: E402
    Basis, Entity, LifecycleState, ModelLayer)

from spatialdds_ros import transport as tt                      # noqa: E402
from spatialdds_ros.adapter import RobotAdapter                 # noqa: E402
from spatialdds_ros.config import Anchor, Config                # noqa: E402
from spatialdds_ros.perception import (                         # noqa: E402
    ASSERTER_STOPPED, NO_LONGER_HOLDS, SUBJECT_UNOBSERVED,
    ObstaclePublisher, edge_id)
from spatialdds_ros.qos import MODEL_LATCHED                    # noqa: E402
from spatialdds_ros.topics import Topics                        # noqa: E402

SCENE = "bare-scene"
FRAME = "map/bare-scene"
ROBOT = "ent:robot:b"
INTERSECTS = "http://www.opengis.net/ont/geosparql#sfIntersects"
WALKWAY = "ent:venue:walkway"


def participant(domain: int):
    try:
        from cyclonedds.domain import DomainParticipant
        return DomainParticipant(domain)
    except Exception as exc:                                  # noqa: BLE001
        raise unittest.SkipTest(f"DDS-UNAVAILABLE: {exc}")


def config(**kw) -> Config:
    return Config(scene=SCENE, entity_id=ROBOT, frame_id=FRAME, **kw)


class _Wire(unittest.TestCase):
    DOMAIN = 155

    def setUp(self):
        self.ours = participant(self.DOMAIN)
        self.publisher = ObstaclePublisher(self.ours, config())
        self.theirs = participant(self.DOMAIN)
        topics = Topics(SCENE)
        self.records = tt.make_reader(self.theirs, topics.entity, Entity,
                                      MODEL_LATCHED.name)
        self.edges = tt.make_reader(self.theirs, topics.relationship,
                                    Relationship, MODEL_LATCHED.name)
        time.sleep(0.3)

    def drain(self, reader, seconds: float = 0.5):
        out, deadline = [], time.time() + seconds
        while time.time() < deadline:
            out += tt.take_with_state(reader)
            time.sleep(0.05)
        return out


class ATrack(_Wire):
    DOMAIN = 155

    def test_a_sighting_is_an_observed_fast_record_in_this_publishers_name(
            self):
        eid = self.publisher.observed("cone-1", 2.0, 1.0, length=1.0,
                                      width=0.5, height=0.8)
        self.assertEqual(eid, "ent:b:cone-1")
        record = [s.data for s in self.drain(self.records) if s.data][-1]
        self.assertEqual(record.basis, Basis.OBSERVED)
        self.assertEqual(record.layer, ModelLayer.FAST)
        self.assertEqual(record.source_id, "svc:robot:b")
        self.assertTrue(record.has_extent)
        props = {kv.key: kv.value for kv in record.properties}
        self.assertEqual(props["demo.obb.length_m"], "1")
        self.assertEqual(props["demo.obb.width_m"], "0.5")

    def test_without_dimensions_it_claims_no_size(self):
        self.publisher.observed("dot", 2.0, 1.0)
        record = [s.data for s in self.drain(self.records) if s.data][-1]
        self.assertFalse(record.has_extent)

    def test_a_silent_track_is_unobserved_with_a_reason_and_its_claims_retire(
            self):
        t = time.time()
        for k in range(5):
            self.publisher.observed("cone", 2.0, 1.0, length=1.0, width=1.0,
                                    now=t + 0.1 * k)
        rid = self.publisher.claim("cone", INTERSECTS, WALKWAY)
        notes = self.publisher.tick(now=t + 5.0)
        self.assertTrue(any("unobserved" in n for n in notes), notes)
        records = [s.data for s in self.drain(self.records) if s.data]
        self.assertEqual(records[-1].state, LifecycleState.UNOBSERVED)
        self.assertIn("last seen", records[-1].state_reason)
        edges = self.drain(self.edges)
        terminal = [s.data for s in edges if s.data
                    and s.data.state == RelationshipLifecycle.RETIRED]
        self.assertEqual([e.id for e in terminal], [rid])
        self.assertTrue(terminal[0].state_reason.startswith(
            SUBJECT_UNOBSERVED))
        self.assertTrue(terminal[0].has_valid_until)

    def test_a_lost_track_retires_with_its_reason_then_is_disposed(self):
        self.publisher.observed("cone", 2.0, 1.0)
        self.drain(self.records)
        self.publisher.lost("cone", "tracker gave it up")
        samples = self.drain(self.records)
        retired = [s for s in samples if s.data is not None]
        self.assertEqual(retired[-1].data.state, LifecycleState.RETIRED)
        self.assertEqual(retired[-1].data.state_reason, "tracker gave it up")
        self.assertTrue(any(int(s.instance_state)
                            == int(InstanceState.NotAliveDisposed)
                            for s in samples))


class AClaim(_Wire):
    DOMAIN = 156

    def test_the_id_is_the_demos_rule(self):
        self.publisher.observed("cone", 2.0, 1.0)
        rid = self.publisher.claim("cone", INTERSECTS, WALKWAY)
        self.assertEqual(rid, edge_id("svc:robot:b", "ent:b:cone",
                                      INTERSECTS, WALKWAY))
        # SpatialDDS-demo's spelling of the same id, computed there for this
        # proposition, so the two rules cannot drift apart unnoticed.
        self.assertEqual(
            edge_id("svc:crossing:mmwave-ut-04", "ent:mmwave-ut-04:ped-17",
                    "http://www.opengis.net/ont/geosparql#sfWithin",
                    "ent:map:crosswalk"),
            "rel:svc:crossing:mmwave-ut-04:dd530c4a474956c8")

    def test_an_unchanged_opinion_writes_nothing_and_a_new_one_revises(self):
        self.publisher.observed("cone", 2.0, 1.0)
        rid = self.publisher.claim("cone", INTERSECTS, WALKWAY,
                                   confidence=0.8)
        first = [s.data for s in self.drain(self.edges) if s.data]
        self.assertEqual([e.state for e in first],
                         [RelationshipLifecycle.CREATED])
        self.publisher.claim("cone", INTERSECTS, WALKWAY, confidence=0.8)
        self.assertEqual([s for s in self.drain(self.edges) if s.data], [])
        self.publisher.claim("cone", INTERSECTS, WALKWAY, confidence=0.9)
        revised = [s.data for s in self.drain(self.edges) if s.data]
        self.assertEqual([(e.id, e.state) for e in revised],
                         [(rid, RelationshipLifecycle.REVISED)])
        self.assertEqual(revised[0].basis, Basis.OBSERVED)
        self.assertEqual(revised[0].derived_from[0].key, "ent:b:cone")

    def test_withdrawal_names_its_cause_and_sets_valid_until_by_it(self):
        self.publisher.observed("a", 2.0, 1.0)
        self.publisher.observed("c", 2.0, 1.0)
        held = self.publisher.claim("a", INTERSECTS, WALKWAY)
        stop = self.publisher.claim("c", INTERSECTS, WALKWAY)
        self.drain(self.edges)
        self.publisher.withdraw(held, "moved off the walkway")
        self.publisher.close()
        terminal = {s.data.id: s.data for s in self.drain(self.edges)
                    if s.data and s.data.state
                    == RelationshipLifecycle.RETIRED}
        self.assertTrue(terminal[held].state_reason.startswith(
            NO_LONGER_HOLDS))
        self.assertTrue(terminal[held].has_valid_until)
        self.assertTrue(terminal[stop].state_reason.startswith(
            ASSERTER_STOPPED))
        self.assertFalse(terminal[stop].has_valid_until,
                         "a clean stop did not consult the world")

    def test_no_claim_about_a_track_it_is_not_seeing(self):
        with self.assertRaises(ValueError):
            self.publisher.claim("never-seen", INTERSECTS, WALKWAY)


class AKilledPublisherDisposesNothing(_Wire):
    """The crash half of the audit: liveliness ends, nothing is disposed."""
    DOMAIN = 157

    def test_sigkill_reads_as_writers_gone_never_as_disposed(self):
        proc = subprocess.Popen(
            [sys.executable, str(REPO / "test" / "_hazard_asserter.py"),
             str(self.DOMAIN), SCENE, FRAME, "svc:hazard:k", "3.0", "0.0",
             "2.0", "1.0", INTERSECTS, WALKWAY], stdout=subprocess.PIPE,
            text=True)
        try:
            self.assertEqual(proc.stdout.readline().split()[0], "up")
            time.sleep(1.0)
        finally:
            proc.send_signal(signal.SIGKILL)
            proc.wait()
        states = {"entity": set(), "relationship": set()}
        deadline = time.time() + 15.0
        while time.time() < deadline:
            for lane, reader in (("entity", self.records),
                                 ("relationship", self.edges)):
                for s in tt.take_with_state(reader):
                    states[lane].add(s.state_name)
                    if s.data is not None:
                        self.assertNotEqual(
                            getattr(s.data, "state", None),
                            RelationshipLifecycle.RETIRED)
            if all("NOT_ALIVE_NO_WRITERS" in v for v in states.values()):
                break
            time.sleep(0.1)
        for lane, seen in states.items():
            with self.subTest(lane=lane):
                self.assertIn("NOT_ALIVE_NO_WRITERS", seen)
                self.assertNotIn("NOT_ALIVE_DISPOSED", seen)


class TheAutodisposeAudit(unittest.TestCase):
    """Every writer this library opens, read off the wire object."""
    DOMAIN = 158

    def test_no_writer_disposes_on_exit(self):
        from cyclonedds.qos import Policy
        p = participant(self.DOMAIN)
        adapter = RobotAdapter(p, config(anchor=Anchor(30.0, -97.0, 0.0)))
        writers = {
            "adapter.entity": adapter._entities,
            "adapter.pose": adapter._poses,
            "adapter.frame_transform": adapter._frames,
            **{f"perception.{k}": w for k, w in
               ObstaclePublisher(p, config()).writers.items()},
        }
        for name, writer in writers.items():
            with self.subTest(writer=name):
                policies = [x for x in writer.get_qos()
                            if isinstance(x, Policy.WriterDataLifecycle)]
                self.assertEqual(
                    [str(x) for x in policies],
                    ["Policy.WriterDataLifecycle(autodispose=False)"])
        print("\n  autodispose audit: " + ", ".join(sorted(writers)))
        del adapter
        gc.collect()


if __name__ == "__main__":
    unittest.main()
