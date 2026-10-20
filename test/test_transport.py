#!/usr/bin/env python3
"""
Endpoints and reads: the two defects that only appear with a second party.

**The pair.** The announce reader was opened first, then the announce writer
with its `ttl_sec` mapped onto Lifespan, and the *writer* failed with
`DDS_RETCODE_INCONSISTENT_POLICY`. Each endpoint was fine alone; the pair was
not, because both created a `Topic` for the same name and the Lifespan made
their topic QoS disagree. The failure lands on whichever endpoint was
constructed second, which makes it look like a problem with that endpoint
rather than with the pair — so a test that built them in one order proved
nothing. Both orders are here.

**The second library.** The same mechanism, one policy wider, found by the
first real consumer: this library put the whole profile on its topics, which
silently made topic QoS a contract every other participant in the process had
to match. `TwoConventionsOnOneTopic` is that case, both orders again.

**The peer with a lifetime.** `take()` yields lifecycle notifications as well
as payloads, and until 2026-10-19 this library's reads filtered only `None` —
so the first requester whose participant went away killed the robot reading
the lane. No test here had ever produced one, because every writer in the
suite outlived every reader. `APeerThatDies` is the writer that does not.
"""
from __future__ import annotations

import gc
import sys
import threading
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from spatialdds_idl18.builtin import Time                      # noqa: E402
from spatialdds_idl18.spatial.core import Aabb3, PoseSE3        # noqa: E402
from spatialdds_idl18.spatial.disco import Announce            # noqa: E402
from spatialdds_idl18.spatial.owm import (                      # noqa: E402
    Entity, ModelCommand)

from spatialdds_ros import transport as tt                     # noqa: E402
from spatialdds_ros.config import Config                        # noqa: E402
from spatialdds_ros.qos import MODEL_COMMAND, MODEL_LATCHED     # noqa: E402
from spatialdds_ros.topics import Topics                        # noqa: E402
from spatialdds_ros.transport import UnknownQosProfile         # noqa: E402

TOPIC = "spatialdds/transport-test/world/entity/v1"


def participant(domain: int):
    try:
        from cyclonedds.domain import DomainParticipant
        return DomainParticipant(domain)
    except Exception as exc:                                   # noqa: BLE001
        raise unittest.SkipTest(f"DDS-UNAVAILABLE: {exc}")


class ALifespanDoesNotBreakThePair(unittest.TestCase):
    def test_reader_first_then_a_writer_with_a_lifespan(self):
        """The order that failed. Lifespan is a writer concern, so the topic's
        QoS must not carry it."""
        p = participant(110)
        tt.make_reader(p, TOPIC, Entity, MODEL_LATCHED.name)
        tt.make_writer(p, TOPIC, Entity, MODEL_LATCHED.name,
                       lifespan_sec=300.0)

    def test_writer_with_a_lifespan_first_then_a_reader(self):
        """The other order, which passed even before the fix — which is why
        testing one order would have proved nothing."""
        p = participant(111)
        tt.make_writer(p, TOPIC, Entity, MODEL_LATCHED.name,
                       lifespan_sec=300.0)
        tt.make_reader(p, TOPIC, Entity, MODEL_LATCHED.name)

    def test_two_writers_with_different_lifespans_coexist(self):
        """
        The general form: a topic's QoS has to be a function of the profile
        alone, so two publishers advertising different TTLs on one topic is a
        legitimate thing rather than a conflict.
        """
        p = participant(112)
        tt.make_writer(p, TOPIC, Entity, MODEL_LATCHED.name, lifespan_sec=60.0)
        tt.make_writer(p, TOPIC, Entity, MODEL_LATCHED.name, lifespan_sec=600.0)

    def test_a_lifespan_still_reaches_the_wire(self):
        """
        And it is not simply being dropped. Moving Lifespan off the topic would
        be no fix at all if it stopped being applied anywhere — the announce
        depends on the middleware expiring a stale sample.
        """
        p = participant(113)
        writer = tt.make_writer(p, "spatialdds/discovery/announce/v1", Announce,
                                MODEL_LATCHED.name, lifespan_sec=2.0)
        policies = repr(writer.get_qos())
        self.assertIn("Lifespan", policies,
                      f"the writer's QoS should carry Lifespan: {policies}")

    def test_the_reader_has_no_lifespan(self):
        reader = tt.make_reader(participant(114), TOPIC, Entity,
                                MODEL_LATCHED.name)
        self.assertNotIn("Lifespan", repr(reader.get_qos()))


class AnUnknownProfileIsRefused(unittest.TestCase):
    def test_a_name_outside_the_table_raises_with_the_options(self):
        """
        Rather than silently falling back to a default. A profile this library
        does not carry is a mistake with a silent failure mode — endpoints that
        never match — so it is refused at construction with the three names
        that do exist.
        """
        p = participant(115)
        with self.assertRaises(UnknownQosProfile) as caught:
            tt.make_writer(p, TOPIC, Entity, "MODEL_WISHFUL")
        self.assertIn("MODEL_LATCHED", str(caught.exception))


class SamplesComeBack(unittest.TestCase):
    def test_a_write_reaches_a_reader_on_the_same_topic(self):
        """The floor: that these helpers produce endpoints which actually
        match. Every other test in the suite rests on it."""
        from spatialdds_ros.adapter import RobotAdapter
        from spatialdds_ros.config import Config
        p = participant(116)
        config = Config(scene="transport-test", entity_id="ent:robot:t1",
                        frame_id="map/transport-test")
        reader = tt.make_reader(p, "spatialdds/transport-test/world/entity/v1",
                                Entity, MODEL_LATCHED.name)
        time.sleep(0.4)
        tt.take_samples(reader)
        RobotAdapter(p, config).observed(1.0, 2.0, 0.0)
        got, deadline = [], time.time() + 2.0
        while time.time() < deadline and not got:
            got = [s for s in tt.take_samples(reader)
                   if s.entity_id == "ent:robot:t1"]
            time.sleep(0.05)
        self.assertTrue(got, "a written entity should reach a reader")


class TwoConventionsOnOneTopic(unittest.TestCase):
    """
    What the first real consumer hit, and why topic QoS is now the default.

    SpatialDDS-demo applies the same §3.3.3 profiles — its `to_qos()` results
    are identical to this library's — but puts them on **endpoints only** and
    creates its `Topic` objects with no QoS at all. When this library created
    its topics carrying the profile, the two could not share a participant:
    the demo's entity reader was already up, the adapter asked for the same
    name with the profile, and CycloneDDS refused with
    `DDS_RETCODE_INCONSISTENT_POLICY`.

    Both conventions are legal DDS. The library had quietly made itself the
    one that must be matched, with nothing in its API saying so.
    """

    @staticmethod
    def _topic_with(participant, qos, name=TOPIC):
        from cyclonedds.topic import Topic
        return (Topic(participant, name, Entity, qos=qos) if qos
                else Topic(participant, name, Entity))

    def test_topic_qos_must_agree_among_live_topics(self):
        """
        The behaviour the convention exists for, pinned so it is not folklore.
        If CycloneDDS ever stopped refusing, the convention would be moot and
        this is where that news arrives.

        **Every topic here is held in a local.** A `Topic` whose last
        reference drops is collected, and the name becomes free again — so a
        probe that discards its first topic sees every combination accepted
        and concludes, wrongly, that nothing is enforced. That mistake is why
        the finding filed during the adoption named the construction *order*
        as the cause. The cause is disagreement between live topics; order
        only decides which call is the one refused.
        """
        profile = MODEL_LATCHED.to_qos()
        # One participant per direction, bound once: a topic name is unique
        # *within a participant*, so two participants would agree trivially.
        first = participant(119)
        held = self._topic_with(first, profile)                  # noqa: F841
        with self.assertRaises(Exception) as caught:
            self._topic_with(first, None)
        self.assertIn("INCONSISTENT_POLICY", str(caught.exception).upper())

        # And the other way round, which is the shape the adoption met: the
        # consumer was up first, holding a default-QoS topic in its cache.
        second = participant(120)
        other = self._topic_with(second, None)                   # noqa: F841
        with self.assertRaises(Exception) as caught:
            self._topic_with(second, profile)
        self.assertIn("INCONSISTENT_POLICY", str(caught.exception).upper())

        # Agreement is all that is asked. Two topics, same name, same QoS.
        third = participant(121)
        agreed = self._topic_with(third, None)                   # noqa: F841
        self._topic_with(third, None)
        print("\n  topic QoS must agree among live topics, either order")

    def test_a_held_foreign_topic_then_this_library(self):
        """
        The failing order, with the consumer's topic **held** the way the
        demo's cache holds it — without that this test passes against a
        collected topic and proves nothing.
        """
        p = participant(122)
        self.foreign = self._topic_with(p, None)     # held for the test's life
        tt.make_reader(p, TOPIC, Entity, MODEL_LATCHED.name)
        tt.make_writer(p, TOPIC, Entity, MODEL_LATCHED.name)

    def test_this_library_first_then_a_held_foreign_topic(self):
        """The other order. Separate, because the refusal lands on whoever is
        second and one order would have proved half of it."""
        p = participant(123)
        self.writer = tt.make_writer(p, TOPIC, Entity, MODEL_LATCHED.name)
        self.reader = tt.make_reader(p, TOPIC, Entity, MODEL_LATCHED.name)
        self.foreign = self._topic_with(p, None)

    def test_the_profile_still_reaches_both_endpoints(self):
        """
        The fix must not be "stopped applying QoS". §3.3.3 is load-bearing:
        Deadline alone is request/offered, and a mismatch is silence.
        """
        p = participant(124)
        writer = tt.make_writer(p, TOPIC, Entity, MODEL_LATCHED.name)
        reader = tt.make_reader(p, TOPIC, Entity, MODEL_LATCHED.name)
        for what, qos in (("writer", repr(writer.get_qos())),
                          ("reader", repr(reader.get_qos()))):
            for policy in ("Durability", "Reliability", "History"):
                self.assertIn(policy, qos,
                              f"the {what} should carry {policy}: {qos}")

    def test_a_library_writer_exchanges_with_a_foreign_reader(self):
        """
        The floor that matters: matching is an *endpoint* negotiation. A reader
        built the other convention's way — default topic QoS, profile on the
        endpoint — must still receive, or the fix traded a crash for silence.
        """
        from cyclonedds.sub import DataReader, Subscriber
        from spatialdds_ros.adapter import RobotAdapter
        p = participant(125)
        name = Topics("conventions").entity
        self.foreign = self._topic_with(p, None, name=name)
        foreign_reader = DataReader(Subscriber(p), self.foreign,
                                    qos=MODEL_LATCHED.to_qos())
        time.sleep(0.4)
        # The adapter's own writer, built by this library on that same name —
        # no substitution, so what is proved is the real pairing.
        adapter = RobotAdapter(p, Config(
            scene="conventions", entity_id="ent:robot:x1",
            frame_id="map/conventions"))
        adapter.observed(1.0, 2.0, 0.0)
        got, deadline = [], time.time() + 3.0
        while time.time() < deadline and not got:
            got = [s for s in tt.take_samples(foreign_reader)
                   if s.entity_id == "ent:robot:x1"]
            time.sleep(0.05)
        self.assertTrue(got, "a library writer must reach a reader built the "
                             "other convention's way")
        print(f"  across conventions: {got[-1].entity_id} received")


def a_goto(subject: str, x: float, y: float,
           requester: str = "short-lived-peer") -> ModelCommand:
    now = time.time()
    return ModelCommand(
        command_id=f"cmd:peer:{int(now * 1e6)}", verb="goto",
        subject_id=subject, reason="injection", requester_id=requester,
        has_pose=True, pose=PoseSE3(t=[x, y, 0.0], q=[0.0, 0.0, 0.0, 1.0]),
        has_extent=False,
        extent=Aabb3(min_xyz=[0.0, 0.0, 0.0], max_xyz=[0.0, 0.0, 0.0]),
        stamp=Time(sec=int(now), nanosec=0))


class Recording:
    """A navigator that accepts and remembers."""

    def __init__(self):
        self.goals = []

    def available(self):
        return None

    def send(self, x, y):
        self.goals.append((x, y))
        return f"heading for ({x:.2f}, {y:.2f})"


class APeerThatDies(unittest.TestCase):
    """
    The one writer in this suite that does not outlive the reader.

    This is the shape every unit test avoids and every deployment supplies: a
    requester that joins a lane, asks for something, and goes away. The
    middleware then delivers a notification about the instance it left behind
    — an `InvalidSample`, carrying state and **no payload**. A read that
    assumes every sample is a payload takes a field off one and raises:

        AttributeError: 'InvalidSample' object has no attribute 'subject_id'

    Found a minute into the demo's adoption, by exactly this sequence: the
    capture tool starts its own robot, that robot is correctly refused by the
    one-writer-per-key guard, and its participant then goes away.

    Two details the first version of this test got wrong, both of which made
    it pass while proving nothing:

    * The peer gets its **own participant**, deleted outright, because that is
      what a requester is. A writer dropped inside the reader's own
      participant is a weaker reproduction of a stronger event.
    * The payload must be **taken before the peer dies**. A dispose with an
      unread sample still queued rides on that sample's `sample_info` and no
      separate notification is generated — so leaving it unread suppresses the
      very thing being injected. Taking it first is also what a running robot
      does, which is why the deployment hit this and the test did not.
    """

    SCENE = "peer-dies"
    ENTITY = "ent:robot:p1"
    DOMAIN = 126

    def setUp(self):
        self.p = participant(self.DOMAIN)
        self.names = Topics(self.SCENE)
        # A second reader on the same lane, so the assertion that a lifecycle
        # event was really delivered does not rest on the adapter — whose job
        # here is precisely to not see it.
        self.watcher = tt.make_reader(self.p, self.names.command, ModelCommand,
                                      MODEL_COMMAND.name)

    def _drain(self, reader, seconds=1.5):
        out, deadline = [], time.time() + seconds
        while time.time() < deadline:
            out.extend(tt.take_with_state(reader))
            time.sleep(0.05)
        return out

    def test_the_lifecycle_event_is_real_and_not_a_payload(self):
        """
        Both halves. The injection has to actually produce an invalid sample —
        otherwise everything below it passes while testing nothing — and the
        adapter has to come through it having driven the goal exactly once.
        """
        from cyclonedds.domain import DomainParticipant
        from spatialdds_ros.adapter import RobotAdapter
        navigator = Recording()
        adapter = RobotAdapter(self.p, Config(
            scene=self.SCENE, entity_id=self.ENTITY,
            frame_id="map/peer-dies"), navigator=navigator)
        adapter.observed(0.0, 0.0, 0.0)

        peer = DomainParticipant(self.DOMAIN)
        writer = tt.make_writer(peer, self.names.command, ModelCommand,
                                MODEL_COMMAND.name)
        time.sleep(0.5)                       # let it match before it speaks
        writer.write(a_goto(self.ENTITY, 4.0, 3.0))

        # Consumed while the peer is still alive, by both readers.
        payloads, deadline = [], time.time() + 3.0
        while time.time() < deadline and not navigator.goals:
            payloads.extend(t for t in tt.take_with_state(self.watcher)
                            if t.data is not None)
            adapter.poll_commands()
            time.sleep(0.05)
        self.assertTrue(payloads, "the peer's goto should have arrived")
        self.assertEqual(navigator.goals, [(4.0, 3.0)],
                         "and should have been driven once")

        del writer
        del peer
        gc.collect()
        time.sleep(0.5)

        # What the middleware delivered after the death, read independently.
        taken = self._drain(self.watcher)
        events = [t for t in taken if t.data is None]
        self.assertTrue(
            events,
            "the injection produced no lifecycle event, so this test proves "
            "nothing — a peer that dies must leave one behind")
        self.assertFalse(events[0].alive)
        self.assertEqual(events[0].state_name, "NOT_ALIVE_DISPOSED")
        self.assertIsNone(events[0].data,
                          "a lifecycle notification carries no payload")
        print(f"\n  the peer's death delivered {len(events)} lifecycle "
              f"event(s): {events[0].state_name}")

        # And the adapter, whose crash this was, reads the lane after it.
        notes = adapter.poll_commands() + adapter.poll_commands()
        self.assertEqual([n for n in notes if "read failed" in n], [],
                         f"the lane read should not have failed: {notes}")
        self.assertEqual(
            navigator.goals, [(4.0, 3.0)],
            f"and the goal stays handled exactly once: {navigator.goals}")
        print(f"  adapter read the lane after it and drove once: "
              f"{navigator.goals}")

    def test_a_predecessors_tombstone_does_not_stop_a_robot_starting(self):
        """
        The second call site, which nothing had ever reached.

        A scene with one robot on it; a second robot starts; the first exits
        during the startup check. `already_published` scans the latched entity
        lane for *its own* key, so a record belonging to somebody else does
        not end the loop — and when that robot's tombstone arrives, the scan
        took `.entity_id` off it. An adapter could therefore crash at startup
        because an unrelated robot happened to exit while it was looking.

        The timer is the point: the predecessor has to die **inside** the
        settle window, because `already_published` builds its own reader and a
        reader that joins after the writer is gone is never told anything.
        """
        from cyclonedds.domain import DomainParticipant
        from spatialdds_ros.adapter import RobotAdapter, already_published
        ghost_participant = DomainParticipant(self.DOMAIN)
        holder = [RobotAdapter(ghost_participant, Config(
            scene=self.SCENE, entity_id="ent:robot:ghost",
            frame_id="map/peer-dies")), ghost_participant]
        holder[0].observed(7.0, 8.0, 0.0)
        time.sleep(0.5)

        def exit_mid_scan():
            holder.clear()
            gc.collect()

        timer = threading.Timer(0.8, exit_mid_scan)
        timer.start()
        try:
            verdict = already_published(self.p, Config(
                scene=self.SCENE, entity_id="ent:robot:newcomer",
                frame_id="map/peer-dies"), settle=2.5)
        finally:
            timer.cancel()

        self.assertIsNone(
            verdict,
            "nobody is publishing ent:robot:newcomer, so the newcomer starts")
        print("  a predecessor exiting mid-scan: read without crashing, and "
              "not counted as a conflict")


if __name__ == "__main__":
    unittest.main()
