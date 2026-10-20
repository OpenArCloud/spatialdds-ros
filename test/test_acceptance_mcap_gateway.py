#!/usr/bin/env python3
"""
Acceptance item 2: recorded to typed MCAP, served by the unmodified pip
gateway, every payload validated against published schemas with counts.

**Why the gateway is the check and not an assertion of mine.** The recording is
written from the bytes the library put on the bus — `sample.serialize()` and
nothing else. If `spatialdds serve --mcap` can read that file, resolve its
embedded IDL, build resources from it and serve payloads that validate against
the schemas *it* publishes, then the bytes were conformant. A test of my own
devising could agree with my own encoder and prove very little; this one is
checked by a tool I did not write and cannot adjust.

The chain asserted here is: library → CDR on the bus → typed MCAP →
unmodified gateway → JSON over HTTP → the gateway's own published JSON Schema.
Every hop is a place a type error could hide, and the last hop is the one that
would catch a payload that merely *looked* right.

Nothing in this repository is on the gateway's path: it is the PyPI
distribution, run as a subprocess, reading a file.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

from spatialdds_ros.announce import ANNOUNCE_TOPIC, announce_for  # noqa: E402
from spatialdds_ros.config import Anchor, Config                 # noqa: E402
from spatialdds_ros.frames import georeference                    # noqa: E402
from spatialdds_ros.kinematic import Bounds                       # noqa: E402
from spatialdds_ros.recording import Recorder                     # noqa: E402
from spatialdds_ros.standalone import Runner                      # noqa: E402
from spatialdds_ros.topics import Topics                          # noqa: E402

SCENE = "bare-scene"
TOKEN = "acceptance-token"
PORT = 8412
BASE = f"http://127.0.0.1:{PORT}"
ANCHOR = Anchor(30.28396857092828, -97.73974829101985, 145.083)


def _get(path: str, token: str | None = None):
    request = urllib.request.Request(f"{BASE}{path}")
    if token:
        request.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(request, timeout=10) as response:
        return response.status, json.load(response)


class RecordedThenServed(unittest.TestCase):
    gateway: subprocess.Popen | None = None
    tmp: Path | None = None

    @classmethod
    def setUpClass(cls):
        try:
            import jsonschema                                   # noqa: F401
        except ImportError:                                     # pragma: no cover
            raise unittest.SkipTest("jsonschema is needed to validate payloads")
        if shutil.which("spatialdds-serve") is None:
            raise unittest.SkipTest(
                "spatialdds-serve is not on PATH; the acceptance needs the "
                "PyPI gateway (pip install 'spatialdds[server]==0.5.4')")
        try:
            from cyclonedds.domain import DomainParticipant     # noqa: F401
        except Exception as exc:                                # noqa: BLE001
            raise unittest.SkipTest(f"DDS-UNAVAILABLE: {exc}")

        cls.tmp = Path(tempfile.mkdtemp(prefix="spatialdds-ros-acceptance-"))
        cls.mcap = cls.tmp / f"{SCENE}.mcap"
        cls.counts = cls._record(cls.mcap)
        cls.gateway = cls._serve(cls.mcap)

    @classmethod
    def tearDownClass(cls):
        if cls.gateway is not None:
            cls.gateway.terminate()
            try:
                cls.gateway.wait(timeout=10)
            except subprocess.TimeoutExpired:                   # pragma: no cover
                cls.gateway.kill()
        if cls.tmp is not None and cls.tmp.exists():
            shutil.rmtree(cls.tmp, ignore_errors=True)

    # --- the recording ----------------------------------------------------

    @classmethod
    def _record(cls, path: Path) -> dict:
        """
        A real run, recorded. Anchored, so the frames lane has something on it
        — the no-anchor case is acceptance item 4's and is asserted there.
        """
        config = Config(scene=SCENE, entity_id="ent:robot:r1",
                        frame_id="map/bare-scene", domain=132, anchor=ANCHOR)
        runner = Runner(config, rate_hz=20.0, start=(0.0, 0.0), speed=2.0,
                        bounds=Bounds(-10.0, -10.0, 10.0, 10.0))
        runner.start()
        names = Topics(SCENE)
        with Recorder(path, config) as recorder:
            recorder.write(ANNOUNCE_TOPIC, announce_for(config))
            recorder.write(names.frame_transform,
                           georeference(config.frame_id, config.anchor))
            runner.source.goal = (4.0, 3.0)
            for _ in range(60):
                runner.step(0.05)
                recorder.write(names.entity, runner.adapter._entity)
                if runner.source.goal is None:
                    break

            # And a refusal, so the recording carries the path that matters.
            # Driven until it actually happens rather than for a fixed number
            # of steps: the first version of this ran 60 steps at 0.1 m each
            # and stopped about a metre short of the boundary, so the recording
            # contained no refusal and the assertion downstream failed for a
            # reason that had nothing to do with the thing under test.
            runner.source.goal = (50.0, 0.0)
            refused = False
            for _ in range(400):
                notes = runner.step(0.05)
                recorder.write(names.entity, runner.adapter._entity)
                if any("abandoned" in n for n in notes):
                    refused = True
                if refused and runner.source.goal is None:
                    # A few more samples after it, so the recording shows the
                    # refusal *persisting* rather than only the instant of it.
                    for _ in range(10):
                        runner.step(0.05)
                        recorder.write(names.entity, runner.adapter._entity)
                    break
            if not refused:
                raise AssertionError(
                    "the run never produced a refusal, so the recording cannot "
                    "demonstrate one — the acceptance would pass or fail on "
                    "the wrong thing")
            cls.refused = True
            return recorder.close()

    @classmethod
    def _serve(cls, path: Path) -> subprocess.Popen:
        process = subprocess.Popen(
            ["spatialdds-serve", "--mcap", str(path), "--host", "127.0.0.1",
             "--port", str(PORT), "--token", TOKEN],
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
        deadline = time.time() + 30.0
        while time.time() < deadline:
            try:
                _get("/.well-known/spatialdds/schemas")
                return process
            except Exception:                                   # noqa: BLE001
                if process.poll() is not None:                  # pragma: no cover
                    raise AssertionError(
                        f"the gateway exited: {process.stdout.read()[:800]}")
                time.sleep(0.4)
        raise AssertionError("the gateway did not come up")      # pragma: no cover

    # --- the assertions ---------------------------------------------------

    def test_1_the_recording_has_every_lane_that_was_used(self):
        self.assertIn(Topics(SCENE).entity, self.counts)
        self.assertIn(Topics(SCENE).frame_transform, self.counts)
        self.assertIn(ANNOUNCE_TOPIC, self.counts)
        self.assertGreater(self.counts[Topics(SCENE).entity], 50)
        print("\n  recorded:")
        for topic, n in sorted(self.counts.items()):
            print(f"    {n:>4}  {topic}")

    def test_2_the_gateway_read_the_recordings_own_idl(self):
        """
        Not the gateway's vendored types. `spatial.owm` is provisional and the
        package does not vendor it — so if the gateway can serve an owm
        resource at all, it got the type from the schema embedded in this
        recording, which is this repository's vendored IDL.
        """
        _status, listing = _get("/.well-known/spatialdds/schemas")
        types = {entry["type"] for entry in listing}
        self.assertIn("spatial.owm.Entity", types,
                      "the gateway should know spatial.owm.Entity, and the "
                      "only place it could have come from is the recording")
        print(f"  gateway publishes {len(types)} schemas, including "
              f"spatial.owm.Entity")

    def test_3_the_resources_are_classified_as_n4_requires(self):
        """
        Read off the gateway, not asserted from my own table. An entity is an
        observation and so operator-scoped; a frame transform is configured
        geometry and so public-cacheable. The gateway decides this from the
        type, which is the point of checking it here.
        """
        status, _body = 0, None
        try:
            status, _body = _get(f"/{SCENE}/world/entity/v1")
        except urllib.error.HTTPError as error:
            status = error.code
        self.assertEqual(status, 401,
                         "an observation lane must require a credential")

        status, _frames = _get(f"/{SCENE}/frames/frame_transform/v1")
        self.assertEqual(status, 200,
                         "a georeference must be readable without one — it is "
                         "configured geometry, and a credential would force a "
                         "CORS preflight a browser cannot complete")
        print("  entity lane 401 without a token; frames lane 200 without one")

    def test_4_every_payload_validates_against_the_published_schema(self):
        """
        The item's core, with counts.

        Every instance of every resource, fetched over HTTP, validated against
        the JSON Schema the gateway publishes for that resource's type. Counts
        printed, and a floor asserted — a validation pass over zero payloads
        is the instrument-blindness shape and would otherwise read as success.
        """
        import jsonschema

        lanes = [(f"/{SCENE}/world/entity/v1", "spatial.owm.Entity"),
                 (f"/{SCENE}/frames/frame_transform/v1",
                  "spatial.core.FrameTransform")]

        schemas: dict[str, dict] = {}
        for _path, fqn in lanes:
            _s, schema = _get(
                f"/.well-known/spatialdds/schemas/{fqn.replace('.', '_')}")
            schemas[fqn] = schema

        validated = 0
        failures: list[str] = []
        per_lane: dict[str, int] = {}
        for path, fqn in lanes:
            _status, collection = _get(path, TOKEN)
            items = (collection if isinstance(collection, list)
                     else collection.get("items")
                     or collection.get("results") or [collection])
            for item in items:
                payload = item.get("value", item) if isinstance(item, dict) \
                    else item
                try:
                    jsonschema.validate(payload, schemas[fqn])
                    validated += 1
                    per_lane[fqn] = per_lane.get(fqn, 0) + 1
                except jsonschema.ValidationError as error:
                    failures.append(
                        f"{fqn} at {path}: {error.message} "
                        f"(at {'/'.join(str(p) for p in error.absolute_path)})")

        print("  validated against the gateway's published schemas:")
        for fqn, n in sorted(per_lane.items()):
            print(f"    {n:>4}  {fqn}")
        self.assertEqual(failures, [],
                         f"{len(failures)} payload(s) failed validation:\n"
                         + "\n".join(failures[:8]))
        self.assertGreater(validated, 0,
                           "a validation pass over zero payloads is not a pass")
        self.assertEqual(sorted(per_lane), sorted(fqn for _p, fqn in lanes),
                         "every lane should have contributed payloads")

    def test_5_the_refusal_survived_the_round_trip(self):
        """
        The thing most likely to be dropped by an encoding mistake is the thing
        that was added last. The refusal rides a `KV` property, so it exercises
        a variable-length sequence of nested structs — and it is read back here
        through the gateway rather than from the recorder.
        """
        _status, collection = _get(f"/{SCENE}/world/entity/v1", TOKEN)
        items = (collection if isinstance(collection, list)
                 else collection.get("items") or [collection])
        statuses = []
        for item in items:
            payload = item.get("value", item) if isinstance(item, dict) else item
            for kv in (payload.get("properties") or []):
                if kv.get("key") == "nav.status":
                    statuses.append(kv.get("value"))
        self.assertTrue(
            statuses,
            "the refusal should have survived bus -> MCAP -> gateway -> JSON")
        self.assertIn("abandoned", statuses[-1])
        print(f"  refusal read back through the gateway: "
              f"{statuses[-1][:64]}...")


if __name__ == "__main__":
    unittest.main()
