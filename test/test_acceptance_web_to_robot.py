#!/usr/bin/env python3
"""
Acceptance item 3: end to end through the web.

    N.6 POST to the gateway, command onto the bus, the library's robot moves
    — the demo's proven chain, now with the library in the robot's seat.

Nothing here reaches into the library to make the robot move. The only input is
an HTTP POST to the gateway, shaped the way a browser would shape it; everything
after that is the chain under test:

    HTTP POST  ->  gateway validates against its published schema
               ->  CommandLane writes a ModelCommand to DDS
               ->  RobotAdapter reads the lane and accepts the goal
               ->  KinematicSource drives
               ->  the robot's pose changes on the bus

The robot's arrival is read back from the *bus*, not from the source's
attributes, so what is asserted is what another participant would see.
"""
from __future__ import annotations

import json
import math
import sys
import time
import unittest
import urllib.error
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from spatialdds_idl18.spatial.owm import Entity               # noqa: E402

from spatialdds_ros import transport as tt                    # noqa: E402
from spatialdds_ros.config import Config                      # noqa: E402
from spatialdds_ros.kinematic import Bounds                   # noqa: E402
from spatialdds_ros.qos import MODEL_LATCHED                  # noqa: E402
from spatialdds_ros.standalone import Runner                  # noqa: E402
from spatialdds_ros.topics import Topics                      # noqa: E402

SCENE = "bare-scene"
ENTITY = "ent:robot:r1"
DOMAIN = 140
PORT = 8414
TOKEN = "web-acceptance-token"


def _post(base: str, path: str, body: dict, token: str):
    request = urllib.request.Request(
        f"{base}{path}", data=json.dumps(body).encode(),
        headers={"Content-Type": "application/json",
                 "Authorization": f"Bearer {token}"},
        method="POST")
    try:
        with urllib.request.urlopen(request, timeout=15) as response:
            return response.status, json.load(response)
    except urllib.error.HTTPError as error:
        raw = error.read()
        try:
            return error.code, json.loads(raw)
        except Exception:                                     # noqa: BLE001
            return error.code, {"raw": raw.decode(errors="replace")[:400]}


def n6_goto(x: float, y: float, requester: str = "web-acceptance") -> dict:
    """
    A `goto` as JSON, the way the web binding carries it.

    `stamp.sec` is a *decimal string*: §N.1 maps int64 onto a string so a
    JavaScript consumer cannot silently lose precision. Writing it as a number
    here would be the mistake the gateway's schema exists to catch, and the
    gateway would reject it — which is worth knowing is the case.
    """
    now = time.time()
    return {
        "command_id": f"cmd:web:{int(now * 1e6)}",
        "verb": "goto",
        "subject_id": ENTITY,
        "reason": "acceptance item 3",
        "requester_id": requester,
        "has_pose": True,
        "pose": {"t": [x, y, 0.0], "q": [0.0, 0.0, 0.0, 1.0]},
        "has_extent": False,
        "extent": {"min_xyz": [0.0, 0.0, 0.0], "max_xyz": [0.0, 0.0, 0.0]},
        "stamp": {"sec": str(int(now)), "nanosec": 0},
    }


class FromAnHttpPostToAMovingRobot(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        try:
            import aiohttp                                    # noqa: F401
            from cyclonedds.domain import DomainParticipant
        except Exception as exc:                              # noqa: BLE001
            raise unittest.SkipTest(f"UNAVAILABLE: {exc}")
        try:
            from _live_gateway import LiveGateway
        except Exception as exc:                              # noqa: BLE001
            raise unittest.SkipTest(f"gateway harness unavailable: {exc}")

        cls.config = Config(scene=SCENE, entity_id=ENTITY,
                            frame_id="map/bare-scene", domain=DOMAIN)
        cls.runner = Runner(cls.config, rate_hz=20.0, start=(0.0, 0.0),
                            speed=2.0,
                            bounds=Bounds(-20.0, -20.0, 20.0, 20.0))
        cls.runner.start()

        # The gateway shares the robot's participant: one process, one bus.
        cls.gateway = LiveGateway(cls.config, cls.runner.participant, PORT,
                                  TOKEN)
        cls.gateway.start()

        # And an independent observer, so arrival is read off the bus.
        cls.observer = DomainParticipant(DOMAIN)
        cls.entities = tt.make_reader(cls.observer, Topics(SCENE).entity,
                                      Entity, MODEL_LATCHED.name)
        time.sleep(0.6)
        tt.take_samples(cls.entities)
        cls.log: list[str] = []

    @classmethod
    def tearDownClass(cls):
        if getattr(cls, "gateway", None) is not None:
            cls.gateway.stop()

    @classmethod
    def _pump(cls, seconds: float) -> list:
        """Step the robot and the gateway's bus-side pump together."""
        seen = []
        deadline = time.time() + seconds
        while time.time() < deadline:
            cls.log.extend(cls.runner.step(0.05))
            cls.gateway.pump.drain()
            seen.extend(e for e in tt.take_samples(cls.entities)
                        if e.entity_id == ENTITY)
            time.sleep(0.02)
        return seen

    def test_1_the_gateway_advertises_the_command_surface(self):
        """
        It should advertise only what it serves. A surface a client is told
        about and cannot use is worse than one it has to discover.
        """
        with urllib.request.urlopen(
                f"{self.gateway.base}/.well-known/spatialdds/search",
                timeout=10) as response:
            body = json.load(response)
        text = json.dumps(body)
        self.assertIn("spatial.owm", text)
        print(f"\n  gateway up at {self.gateway.base}")

    def test_2_an_n6_post_is_accepted_and_reaches_the_bus(self):
        self._pump(0.8)                      # let the robot exist first
        status, body = _post(self.gateway.base,
                             f"/{SCENE}/world/model_command/v1",
                             n6_goto(6.0, 4.0), TOKEN)
        self.assertIn(status, (200, 202),
                      f"the gateway should accept the command: {status} {body}")
        self.assertEqual(self.gateway.lane.sent, 1,
                         "and it should have reached the bus")
        print(f"  POST /{SCENE}/world/model_command/v1 -> {status} "
              f"{body.get('outcome') or body.get('detail', '')}")

    def test_3_the_robot_accepted_the_goal_and_moved(self):
        seen = self._pump(7.0)
        accepted = [n for n in self.log if "heading for" in n]
        self.assertTrue(accepted,
                        f"the adapter should have taken the goal: "
                        f"{self.log[-6:]}")
        self.assertIn("asked by web-acceptance", " ".join(self.log))

        self.assertTrue(seen, "the robot's record should be on the bus")
        last = seen[-1]
        x, y = last.pose.t[0], last.pose.t[1]
        self.assertAlmostEqual(x, 6.0, delta=0.3)
        self.assertAlmostEqual(y, 4.0, delta=0.3)
        print(f"  the robot arrived at ({x:.2f}, {y:.2f}), read off the bus "
              f"by an independent observer")

    def test_4_the_whole_chain_was_exercised_once(self):
        """
        Stated so the run cannot be misread: exactly one command went in, and
        it is the one the robot acted on.
        """
        self.assertEqual(self.gateway.lane.sent, 1)
        self.assertGreater(
            sum(self.gateway.pump.counts.values()), 0,
            "the gateway should also have pumped the robot's own records")
        print(f"  1 command in; {sum(self.gateway.pump.counts.values())} "
              f"records pumped back out through the gateway")

    def test_5_a_malformed_command_is_declined_with_a_reason(self):
        """
        The other half of M.1: a refusal is an outcome, not an exception. And
        `stamp.sec` as a *number* rather than a decimal string is exactly the
        §N.1 mistake a JavaScript client would make.
        """
        bad = n6_goto(1.0, 1.0)
        bad["stamp"]["sec"] = int(time.time())        # a number, not a string
        status, body = _post(self.gateway.base,
                             f"/{SCENE}/world/model_command/v1", bad, TOKEN)
        self.assertGreaterEqual(status, 400,
                                f"an int64 as a JSON number should be refused "
                                f"by the schema: {status} {body}")
        detail = json.dumps(body)
        self.assertTrue(len(detail) > 10, "and the refusal should say why")
        print(f"  a numeric stamp.sec -> {status}, refused by the published "
              f"schema")
        self.assertEqual(self.gateway.lane.sent, 1,
                         "and nothing extra reached the bus")


if __name__ == "__main__":
    unittest.main()
