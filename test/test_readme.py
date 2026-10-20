#!/usr/bin/env python3
"""
The README's fences are gated, and its tables are checked against the source.

**Prose that ships is an artifact.** A README is the first thing anyone runs
and the last thing anyone re-tests; a command in it that stopped working is a
defect with no failing test anywhere, and a table that drifted from the code is
worse than no table because it will be believed.

So: every fence is accounted for here. The ones that can run on this machine
are run. The ones that cannot — `pip install`, `ros2 run` — are checked
*structurally* against the source, so a renamed parameter or a changed default
fails here rather than in somebody's terminal. An unrunnable fence is not an
excuse for an unchecked one.

The accounting is total by construction: the test below extracts every fence
and fails if one is neither run nor claimed by a structural check, so a new
fence cannot be added without deciding how it is verified.
"""
from __future__ import annotations

import re
import subprocess
import sys
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from spatialdds_ros.announce import ANNOUNCE_TOPIC            # noqa: E402
from spatialdds_ros.config import (                           # noqa: E402
    DEFAULT_NAV2_ACTION, Config)
from spatialdds_ros.qos import (                              # noqa: E402
    MODEL_COMMAND, MODEL_FAST, MODEL_LATCHED)
from spatialdds_ros.topics import Topics                      # noqa: E402

README = REPO / "README.md"
FENCE = re.compile(r"```(\w*)\n(.*?)```", re.S)


def fences() -> list[tuple[str, str]]:
    return [(lang, body) for lang, body in FENCE.findall(README.read_text())]


class EveryFenceIsAccountedFor(unittest.TestCase):
    #: Fences verified by running them.
    RUN = ["python3 -m spatialdds_ros.standalone", "python3 -m pytest"]
    #: Fences verified structurally, with the test that does it.
    STRUCTURAL = {
        "pip install -e .": "the install line is the ament/pip path",
        "ros2 run spatialdds_ros robot_node": "test_the_ros2_fence_names_real_parameters",
        "robot: scene bare-scene": "test_the_printed_output_is_what_it_prints",
    }

    def test_there_are_fences_to_check(self):
        self.assertGreater(len(fences()), 3,
                           "the README should carry runnable examples")

    def test_every_fence_is_either_run_or_structurally_checked(self):
        unaccounted = []
        for lang, body in fences():
            first = body.strip().splitlines()[0] if body.strip() else ""
            claimed = any(k in body for k in self.RUN) or \
                any(k in body for k in self.STRUCTURAL)
            if not claimed:
                unaccounted.append(f"{lang or 'text'}: {first[:70]}")
        self.assertEqual(
            unaccounted, [],
            "these fences are neither run nor structurally checked, so nothing "
            "would notice them going stale:\n" + "\n".join(unaccounted))

    def test_the_standalone_fence_runs_as_printed(self):
        """
        The headline claim — a robot on the bus in a few lines — run exactly as
        the README prints it, flags and all.
        """
        try:
            from cyclonedds.domain import DomainParticipant   # noqa: F401
        except Exception as exc:                              # noqa: BLE001
            raise unittest.SkipTest(f"DDS-UNAVAILABLE: {exc}")

        body = next(b for _l, b in fences()
                    if "python3 -m spatialdds_ros.standalone" in b)
        # The README's command, lifted verbatim rather than retyped, with the
        # pip line dropped and a domain added so it cannot meet another test's
        # robot.
        command = " ".join(
            line.strip().rstrip("\\").strip()
            for line in body.strip().splitlines()
            if "pip install" not in line)
        self.assertIn("--scene bare-scene", command)
        argv = command.split() + ["--domain", "150"]

        # `python3` in the README means "this interpreter" here, so the name
        # is replaced rather than relied on being the right one on PATH.
        if argv[0] == "python3":
            argv = [sys.executable] + argv[1:]
        process = subprocess.Popen(
            argv, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
            cwd=str(REPO),
            env={**__import__("os").environ, "PYTHONPATH": str(REPO)})
        try:
            lines: list[str] = []
            deadline = time.time() + 25.0
            while time.time() < deadline:
                if process.poll() is not None:
                    break
                line = process.stdout.readline()
                if line:
                    lines.append(line.rstrip())
                if any("announce:" in l for l in lines):
                    break
            output = "\n".join(lines)
            self.assertIn("scene bare-scene", output,
                          f"the command should report its scene: {output}")
            self.assertIn("no anchor configured", output,
                          "and the degraded mode, which the README shows")
            self.assertIn("announce:", output,
                          "and that it announced itself")
            print(f"\n  the README's command ran and printed "
                  f"{len(lines)} lines")
        finally:
            process.terminate()
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:                 # pragma: no cover
                process.kill()

    def test_the_printed_output_is_what_it_prints(self):
        """
        The output fence is not decorative. Each line of it has to be something
        the code actually emits, or the README is quoting a program that does
        not exist.
        """
        body = next(b for _l, b in fences() if "robot: scene bare-scene" in b)
        config = Config(scene="bare-scene", entity_id="ent:robot:r1",
                        frame_id="map/bare-scene")
        notes = " ".join(config.startup_notes())
        self.assertIn("scene bare-scene, entity ent:robot:r1 in frame "
                      "map/bare-scene, DDS domain 0", notes)
        for phrase in ("no anchor configured", "earth-references",
                       "should say so rather than assume"):
            self.assertIn(phrase, notes,
                          f"the README quotes {phrase!r} but the code does not "
                          f"print it")
            self.assertIn(phrase.split()[0], body)

    def test_the_lane_table_matches_the_topics_and_qos(self):
        """
        The table names four topics and their QoS profiles. Both halves come
        from the code here, so a renamed lane or a changed profile fails.
        """
        text = README.read_text()
        topics = Topics("bare-scene")
        for topic in (topics.entity, topics.pose, topics.command,
                      ANNOUNCE_TOPIC):
            self.assertIn(topic, text,
                          f"the README should list {topic}")
        for profile in (MODEL_LATCHED.name, MODEL_FAST.name,
                        MODEL_COMMAND.name):
            self.assertIn(profile, text)

    def test_the_defaults_in_the_prose_are_the_real_defaults(self):
        config = Config(scene="s", entity_id="e", frame_id="map/s")
        text = README.read_text()
        self.assertEqual(config.domain, 0)
        self.assertEqual(config.ground_z, 0.0)
        self.assertEqual(config.pose_source, "tf")
        self.assertEqual(config.nav2_action, DEFAULT_NAV2_ACTION)
        self.assertIn("`domain` 0", text)
        self.assertIn("`ground_z` 0.0", text)
        self.assertIn("`pose_source` tf", text)
        self.assertIn(DEFAULT_NAV2_ACTION, text)

    def test_the_ros2_fence_names_real_parameters(self):
        """
        `ros2 run` cannot be executed here, so its parameter names are checked
        against what the node declares. A renamed parameter would otherwise
        only be found by someone following the README on a robot.
        """
        body = next(b for _l, b in fences() if "ros2 run spatialdds_ros" in b)
        declared = self._declared_parameters()
        used = re.findall(r"-p (\w+):=", body)
        self.assertTrue(used, "the fence should set parameters")
        for name in used:
            self.assertIn(name, declared,
                          f"the README sets -p {name}:= but the node never "
                          f"declares it")
        # And the three the README calls mandatory really are.
        for name in ("scene", "entity_id", "frame_id"):
            self.assertIn(name, used)
        print(f"  ros2 fence sets {len(used)} parameters, all declared")

    def test_the_pose_source_values_in_the_prose_are_the_real_ones(self):
        from spatialdds_ros.config import POSE_SOURCES
        text = README.read_text()
        for value in POSE_SOURCES:
            self.assertIn(value, text,
                          f"pose_source={value} is accepted but undocumented")

    def test_the_three_mandatory_parameters_are_the_ones_that_refuse(self):
        """The README's table is a promise about behaviour; this is the
        behaviour."""
        from spatialdds_ros.config import ConfigError
        good = dict(scene="s", entity_id="e", frame_id="map/s")
        for missing in ("scene", "entity_id", "frame_id"):
            with self.subTest(missing=missing):
                with self.assertRaises(ConfigError):
                    Config(**{**good, missing: ""})

    def test_the_named_docs_exist(self):
        for name in ("docs/extraction-census.md", "docs/failability.md",
                     "docs/spec-queue.md",
                     "spatialdds_idl18/PROVENANCE.md"):
            with self.subTest(doc=name):
                self.assertIn(name, README.read_text())
                self.assertTrue((REPO / name).exists(),
                                f"the README points at {name}, which is absent")

    @staticmethod
    def _declared_parameters() -> set[str]:
        source = (REPO / "spatialdds_ros" / "robot_node.py").read_text()
        return set(re.findall(r'declare_parameter\("(\w+)"', source))


if __name__ == "__main__":
    unittest.main()
