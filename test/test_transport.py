#!/usr/bin/env python3
"""
Endpoints, and the one ordering bug that only appears with a pair.

Found by using it: the announce reader was opened first, then the announce
writer with its `ttl_sec` mapped onto Lifespan, and the *writer* failed with
`DDS_RETCODE_INCONSISTENT_POLICY`. Each endpoint was fine alone; the pair was
not, because both created a `Topic` for the same name and the Lifespan made
their topic QoS disagree.

The failure lands on whichever endpoint was constructed second, which makes it
look like a problem with that endpoint rather than with the pair — so a test
that built them in the other order would have passed.
"""
from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from spatialdds_idl18.spatial.disco import Announce            # noqa: E402
from spatialdds_idl18.spatial.owm import Entity                # noqa: E402

from spatialdds_ros import transport as tt                     # noqa: E402
from spatialdds_ros.qos import MODEL_LATCHED                   # noqa: E402
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


if __name__ == "__main__":
    unittest.main()
