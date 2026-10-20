#!/usr/bin/env python3
"""
The ruled parameter surface, asserted.

`docs/extraction-census.md` §12 is the ruling; these are the behaviours it
ruled. The two halves worth testing are the ones where being wrong is silent:
what refuses to start, and what a silence means.
"""
from __future__ import annotations

import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from spatialdds_ros.config import (                           # noqa: E402
    DEFAULT_NAV2_ACTION, DEFAULT_TYPE_URI, MAX_CONTENT_REFS,
    MAX_EXTERNAL_REFS, Anchor, Config, ConfigError)
from spatialdds_ros.topics import Topics                      # noqa: E402

GOOD = dict(scene="bare-scene", entity_id="ent:robot:r1",
            frame_id="map/bare-scene")


class WhatRefusesToStart(unittest.TestCase):
    """
    The three with no honest default.

    Each is tested for *refusing*, not for a message, because the behaviour is
    the contract — but the message is checked to contain the reason, since a
    bare "required" would leave an operator to guess why no default exists.
    """

    def test_a_complete_configuration_starts(self):
        """The control. Without it the three below pass by refusing everything."""
        config = Config(**GOOD)
        self.assertEqual(config.scene, "bare-scene")

    def test_no_scene_refuses(self):
        with self.assertRaises(ConfigError) as caught:
            Config(**{**GOOD, "scene": ""})
        self.assertIn("scene is required", str(caught.exception))
        self.assertIn("empty bus", str(caught.exception),
                      "the reason the default cannot exist should travel with "
                      "the refusal")

    def test_no_entity_id_refuses(self):
        with self.assertRaises(ConfigError) as caught:
            Config(**{**GOOD, "entity_id": ""})
        self.assertIn("entity_id is required", str(caught.exception))

    def test_no_frame_id_refuses(self):
        with self.assertRaises(ConfigError) as caught:
            Config(**{**GOOD, "frame_id": ""})
        self.assertIn("frame_id is required", str(caught.exception))

    def test_all_three_problems_are_reported_at_once(self):
        """
        One pass, not three.

        An operator editing a launch file should learn about every missing
        parameter together; reporting the first and exiting makes them
        rediscover the same class of problem once per restart.
        """
        with self.assertRaises(ConfigError) as caught:
            Config(scene="", entity_id="", frame_id="")
        message = str(caught.exception)
        for expected in ("scene is required", "entity_id is required",
                         "frame_id is required"):
            self.assertIn(expected, message)

    def test_a_scene_that_is_not_one_topic_segment_refuses(self):
        """
        A slash in the scene silently produces a six-segment topic, which is a
        name nothing else looks for — the empty-bus failure, not an error.
        """
        with self.assertRaises(ConfigError) as caught:
            Config(**{**GOOD, "scene": "two/segments"})
        self.assertIn("§3.3.1", str(caught.exception))

    def test_a_domain_outside_the_dds_range_refuses(self):
        for bad in (-1, 233):
            with self.subTest(domain=bad), self.assertRaises(ConfigError):
                Config(**{**GOOD, "domain": bad})

    def test_an_unknown_pose_source_refuses(self):
        with self.assertRaises(ConfigError):
            Config(**{**GOOD, "pose_source": "wishful"})


class TheReferenceFieldsAreValidated(unittest.TestCase):
    """
    Bounded sequences, checked where they enter rather than at serialize time.

    `content_refs` and `external_refs` are both `sequence[..., 8]` in the IDL.
    Exceeding a bound fails inside CycloneDDS with a message that names
    neither the field nor the limit, so the refusal belongs here.
    """

    def test_empty_by_default(self):
        config = Config(**GOOD)
        self.assertEqual(config.content_refs, ())
        self.assertEqual(config.external_refs, ())

    def test_lists_are_normalised_to_tuples(self):
        """
        A launch file hands over lists; the dataclass is frozen. A mutable
        value on a frozen "value" is a bug waiting for a second robot.
        """
        config = Config(**GOOD, content_refs=["catalog:a", "catalog:b"],
                        external_refs=[["wikidata", "Q1"]])
        self.assertIsInstance(config.content_refs, tuple)
        self.assertIsInstance(config.external_refs, tuple)
        self.assertIsInstance(config.external_refs[0], tuple)
        self.assertEqual(config.content_refs, ("catalog:a", "catalog:b"))

    def test_too_many_content_refs_refuses_and_cites_the_bound(self):
        with self.assertRaises(ConfigError) as caught:
            Config(**GOOD, content_refs=[f"catalog:{i}" for i in range(9)])
        self.assertIn(str(MAX_CONTENT_REFS), str(caught.exception))
        self.assertIn("IDL", str(caught.exception))

    def test_exactly_the_bound_is_accepted(self):
        """The boundary, not just past it — an off-by-one here would refuse a
        legitimate configuration."""
        config = Config(**GOOD,
                        content_refs=[f"catalog:{i}" for i in
                                      range(MAX_CONTENT_REFS)])
        self.assertEqual(len(config.content_refs), MAX_CONTENT_REFS)

    def test_too_many_external_refs_refuses(self):
        with self.assertRaises(ConfigError) as caught:
            Config(**GOOD,
                   external_refs=[("k", str(i)) for i in range(9)])
        self.assertIn(str(MAX_EXTERNAL_REFS), str(caught.exception))

    def test_an_empty_content_ref_refuses(self):
        """An empty reference resolves to nothing, which is different from
        publishing none."""
        with self.assertRaises(ConfigError) as caught:
            Config(**GOOD, content_refs=["catalog:a", "  "])
        self.assertIn("resolves to nothing", str(caught.exception))

    def test_a_malformed_external_ref_refuses(self):
        for bad in ([("only-one",)], [("k", "")], [("", "v")]):
            with self.subTest(bad=bad), self.assertRaises(ConfigError):
                Config(**GOOD, external_refs=bad)


class WhatASilenceMeans(unittest.TestCase):
    def test_no_anchor_means_no_georeference_and_says_so(self):
        """
        The one legitimate silence. The adapter is fully functional; what is
        absent is the earth-referencing, and the note says what that costs a
        *consumer* rather than only naming the unset parameter.
        """
        config = Config(**GOOD)
        self.assertFalse(config.publishes_georeference)
        notes = " ".join(config.startup_notes())
        self.assertIn("publishes no FrameTransform", notes)
        self.assertIn("consumer", notes,
                      "the degraded mode is documented from the consumer's "
                      "side, per the ruling")
        # And it must not claim to know what *other* participants publish.
        # The first wording said "nothing earth-references them", which was
        # false in the first real deployment — a venue owned that claim.
        self.assertNotIn("nothing earth-references", notes)
        self.assertIn("Another participant may own that claim", notes)

    def test_an_anchor_means_a_georeference(self):
        config = Config(**GOOD, anchor=Anchor(30.28396857, -97.73974829, 145.083))
        self.assertTrue(config.publishes_georeference)
        notes = " ".join(config.startup_notes())
        self.assertIn("earth-fixed", notes)
        self.assertNotIn("publishes no FrameTransform", notes)

    def test_odom_is_selectable_but_never_silent(self):
        """
        Ruled: `/odom` stays available and must announce its own consequence.
        It is map-frame only where `map -> odom` is static and identity.
        """
        notes = " ".join(Config(**GOOD, pose_source="odom").startup_notes())
        self.assertIn("drifts", notes)
        self.assertIn("simulator", notes)

    def test_the_default_pose_source_is_tf_and_says_nothing(self):
        notes = " ".join(Config(**GOOD).startup_notes())
        self.assertNotIn("drifts", notes,
                         "the default should not warn about itself")
        self.assertEqual(Config(**GOOD).pose_source, "tf")


class AnchorsAreValidated(unittest.TestCase):
    def test_a_plausible_anchor_is_accepted(self):
        Anchor(30.28396857, -97.73974829, 145.083)

    def test_a_latitude_beyond_the_poles_refuses_and_says_why(self):
        """
        The transposition case. (-97.7, 30.3) is a well-formed point in the
        South Atlantic and nothing downstream could detect it, so the only
        place to catch it is here.
        """
        with self.assertRaises(ConfigError) as caught:
            Anchor(-97.73974829, 30.28396857, 145.083)
        self.assertIn("swapped", str(caught.exception))

    def test_a_longitude_out_of_range_refuses(self):
        with self.assertRaises(ConfigError):
            Anchor(30.0, 200.0, 0.0)


class DerivedRatherThanAsked(unittest.TestCase):
    def test_source_id_follows_the_entity_id(self):
        self.assertEqual(Config(**GOOD).effective_source_id, "svc:robot:r1")

    def test_an_explicit_source_id_wins(self):
        config = Config(**GOOD, source_id="svc:robot:fleet/r1")
        self.assertEqual(config.effective_source_id, "svc:robot:fleet/r1")

    def test_the_label_falls_back_to_the_id_tail(self):
        self.assertEqual(Config(**GOOD).effective_label, "r1")

    def test_the_documented_defaults_are_what_the_census_says(self):
        config = Config(**GOOD)
        self.assertEqual(config.domain, 0)
        self.assertEqual(config.ground_z, 0.0)
        self.assertEqual(config.nav2_action, DEFAULT_NAV2_ACTION)
        self.assertEqual(config.type_uri, DEFAULT_TYPE_URI)
        self.assertIn("Q4810574", config.type_uri)


class TopicsFollowTheGrammar(unittest.TestCase):
    def test_the_four_names_are_the_scene_applied_to_the_grammar(self):
        topics = Topics("bare-scene")
        self.assertEqual(topics.entity, "spatialdds/bare-scene/world/entity/v1")
        self.assertEqual(topics.pose,
                         "spatialdds/bare-scene/world/model_pose/v1")
        self.assertEqual(topics.command,
                         "spatialdds/bare-scene/world/model_command/v1")
        self.assertEqual(topics.frame_transform,
                         "spatialdds/bare-scene/frames/frame_transform/v1")

    def test_every_name_has_five_segments(self):
        """§3.3.1. A scene that broke this would be caught by Config, but the
        grammar is asserted here too so a future lane cannot quietly add a
        sixth."""
        for lane, name in Topics("bare-scene").all().items():
            with self.subTest(lane=lane):
                self.assertEqual(len(name.split("/")), 5, name)
                self.assertTrue(name.startswith("spatialdds/"))
                self.assertTrue(name.endswith("/v1"))

    def test_the_scene_is_the_only_thing_that_varies(self):
        a = Topics("scene-a").all()
        b = Topics("scene-b").all()
        self.assertEqual(set(a), set(b))
        for lane in a:
            self.assertEqual(a[lane].replace("scene-a", "X"),
                             b[lane].replace("scene-b", "X"))


if __name__ == "__main__":
    unittest.main()
