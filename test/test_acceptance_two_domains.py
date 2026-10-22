#!/usr/bin/env python3
"""
The awareness acceptance: two robots in two ROS domains, one SpatialDDS bus.

    robot A   ROS_DOMAIN_ID 10   localized by an AMCL that goes quiet at (3, 0)
    robot B   ROS_DOMAIN_ID 20   at the origin, aware, with nav2's planner
    the bus   DDS domain 30      the only place the two meet

A's entity reaches B's costmap through the bus, and B's plan to the same goal
bends around A — measured as B's path with A absent against A present. No ROS
topic crosses between the domains, asserted by listing both graphs **from one
container**, so the separation shown is the domain id's and not the network's.

Then the ways A stops being an obstacle, each told apart in B's own log:

* **standing still** with a localizer that has stopped publishing: A is held
  by its adapter and kept fresh by the floor, so it stays — the AMCL gap,
  shown closed (ruled 2026-10-21);
* **quiet but latched**: AMCL interrupted, A's writer still alive, A goes
  UNOBSERVED and B drops it on that record;
* **killed**: SIGKILL, and B drops it on its learned window, long before the
  lease would say anything.

Each of those is checked in B's **costmap** as well as in its cloud, because
that is where a ghost would live: the clearing ring is the census's measured
row, rerun here through the library rather than a probe. Self-exclusion is
asserted on the cloud, not the costmap — nav2 clears under the robot, so a
costmap check would pass with the exclusion removed.

The session is recorded to typed MCAP on the bus and served by the unmodified
PyPI gateway, with every entity payload validated against its schema.

**Skips with a reason until its image exists**, and runs by itself once it
does — the blocker is probed, so the skip expires on its own:

    docker build -t spatialdds-ros-nav2 -f test/ros/Dockerfile .
"""
from __future__ import annotations

import json
import re
import shutil
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

IMAGE = "spatialdds-ros-nav2"
NETWORK = "sdds-two-domains"
A, B, RECORDER = "sdds-robot-a", "sdds-robot-b", "sdds-recorder"
BUS = 30
GOAL = (6.0, 0.0)
PEER_AT = (3.0, 0.0)
#: OccupancyGrid's lethal value, as nav2 publishes the costmap.
LETHAL = 99


def _docker(*args, check=True, timeout=120) -> str:
    out = subprocess.run(["docker", *args], capture_output=True, text=True,
                         timeout=timeout)
    if check and out.returncode != 0:
        raise AssertionError(f"docker {' '.join(args[:3])}: {out.stderr}")
    return out.stdout


def _ros(container: str, domain: int, command: str, timeout=60) -> str:
    return _docker("exec", container, "bash", "-lc",
                   f"source /opt/ros/humble/setup.bash; "
                   f"export ROS_DOMAIN_ID={domain}; {command}",
                   timeout=timeout)


def _probe(*args) -> dict:
    out = _ros(B, 20, "python3 /work/test/ros/probe.py "
               + " ".join(str(a) for a in args))
    return json.loads(out.strip().splitlines()[-1])


def _logs(container: str) -> str:
    """Both streams: ROS logs to stderr, the probes and notes to stdout."""
    return subprocess.run(["docker", "logs", container], text=True,
                          stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT).stdout


def _until(what, seconds: float, every: float = 0.5):
    deadline = time.time() + seconds
    result = what()
    while not result and time.time() < deadline:
        time.sleep(every)
        result = what()
    return result


def _image_missing() -> str | None:
    if shutil.which("docker") is None:
        return "docker is not on PATH"
    probe = subprocess.run(["docker", "image", "inspect", IMAGE],
                           capture_output=True)
    if probe.returncode != 0:
        return (f"image {IMAGE} is not built; build it with "
                f"`docker build -t {IMAGE} -f test/ros/Dockerfile .`")
    return None


class TwoRobotsTwoDomainsOneBus(unittest.TestCase):
    out: Path

    @classmethod
    def setUpClass(cls):
        missing = _image_missing()
        if missing:
            raise unittest.SkipTest(missing)
        cls._teardown()
        try:
            cls._bring_up()
        except BaseException:
            # tearDownClass does not run when setUpClass raises, and a
            # container left running is a port and a bus somebody else meets.
            cls._teardown()
            if getattr(cls, "out", None):
                shutil.rmtree(cls.out, ignore_errors=True)
            raise

    @classmethod
    def _bring_up(cls):
        cls.out = Path(tempfile.mkdtemp(prefix="spatialdds-two-domains-"))
        _docker("network", "create", NETWORK)
        mount = ["-v", f"{REPO}:/work:ro"]
        _docker("run", "-d", "--name", RECORDER, "--network", NETWORK,
                *mount, "-v", f"{cls.out}:/out", IMAGE,
                "python3", "/work/test/ros/record.py", str(BUS),
                "/out/session.mcap")
        _docker("run", "-d", "--name", B, "--network", NETWORK, *mount,
                *cls.extra_b_args(), IMAGE, "bash", "/work/test/ros/robot_b.sh")
        # Up means the node owns its entity, not anything awareness decided:
        # the bring-up must not depend on the behaviour under test, or a
        # broken exclusion fails here rather than in the step that checks it.
        if not _until(lambda: "owns ent:robot:b" in _logs(B), 40):
            raise AssertionError(f"robot B never came up:\n{_logs(B)[-2000:]}")
        cls.absent = _until(lambda: (lambda p: p if "n" in p else None)(
            _probe("path", *GOAL)), 40, 2.0)

    @classmethod
    def extra_b_args(cls) -> list:
        return []

    @classmethod
    def tearDownClass(cls):
        cls._teardown()
        shutil.rmtree(cls.out, ignore_errors=True)

    @staticmethod
    def _teardown():
        for name in (A, B, RECORDER):
            subprocess.run(["docker", "rm", "-f", name], capture_output=True)
        subprocess.run(["docker", "network", "rm", NETWORK],
                       capture_output=True)

    # --- 1. B alone -------------------------------------------------------

    def test_1_alone_b_plans_straight_and_draws_nothing_not_even_itself(self):
        """
        B's own record is on the bus and reaches its own reader ("self 1" in
        its log), and its cloud is empty. Asserted on the cloud rather than
        the costmap: nav2 clears under the robot, so a costmap check would
        pass with the exclusion removed.
        """
        cloud = _probe("cloud")
        self.assertEqual(cloud["width"], 0, f"B drew something: {cloud}")
        self.assertEqual(cloud["frame"], "base_link")
        # And the empty cloud means something: B's own record did arrive.
        self.assertIn("awareness: self 1", _logs(B))
        self.assertLess(self.absent["max_abs_y"], 0.05, self.absent)
        self.assertEqual(_probe("cost", *PEER_AT)["max_cost"], 0)
        print(f"\n  A absent: path {self.absent['length']:.2f} m, "
              f"max |y| {self.absent['max_abs_y']:.3f}; cloud empty")

    # --- 2. A arrives -----------------------------------------------------

    def test_2_a_reaches_bs_costmap_and_bs_plan_bends_around_it(self):
        _docker("run", "-d", "--name", A, "--network", NETWORK,
                "-v", f"{REPO}:/work:ro", IMAGE,
                "bash", "/work/test/ros/robot_a.sh")
        cloud = _until(lambda: (lambda c: c if c.get("near_a") else None)(
            _probe("cloud")), 40, 1.0)
        self.assertTrue(cloud, "A never reached B's cloud")
        self.assertGreater(cloud["min_r"], 1.0,
                           "nothing may be drawn where B itself is")
        cost = _until(lambda: (lambda c: c if c["max_cost"] >= LETHAL
                               else None)(_probe("cost", *PEER_AT)), 20, 1.0)
        self.assertTrue(cost, "A's cells never became lethal")
        present = _probe("path", *GOAL)
        self.assertGreater(present["max_abs_y"], 0.4, present)
        self.assertGreater(present["length"], self.absent["length"] + 0.1)
        type(self).present = present
        print(f"  A present: cloud {cloud['width']} points near (3, 0), cost "
              f"{cost['max_cost']}; path {present['length']:.2f} m, max |y| "
              f"{present['max_abs_y']:.2f}")

    # --- 3. the graphs ----------------------------------------------------

    def test_3_no_ros_topic_crosses_between_the_domains(self):
        """Both graphs, listed from B's container: the network reaches both,
        so what separates them is the domain id alone."""
        listing = {}
        for domain in (10, 20):
            nodes = _ros(B, domain, "ros2 node list --no-daemon --spin-time 3")
            topics = _ros(B, domain, "ros2 topic list --no-daemon --spin-time 3")
            listing[domain] = (set(nodes.split()), set(topics.split()))
        nodes10, topics10 = listing[10]
        nodes20, topics20 = listing[20]
        self.assertIn("/robot_a", nodes10)
        self.assertIn("/robot_b", nodes20)
        for name in ("/robot_b", "/planner_server", "/canvas"):
            self.assertNotIn(name, nodes10)
        for name in ("/robot_a", "/fake_amcl"):
            self.assertNotIn(name, nodes20)
        self.assertFalse({t for t in topics10 if t.startswith("/robot_b")})
        self.assertNotIn("/amcl_pose", topics20)
        self.assertNotIn("/global_costmap/costmap", topics10)
        print(f"  domain 10: {sorted(nodes10)}\n  domain 20: {sorted(nodes20)}")

    # --- 4. standing still ------------------------------------------------

    def test_4_a_standing_still_with_a_silent_localizer_stays(self):
        """Twenty seconds with no new AMCL estimate. Before the floor, A's
        adapter would have gone UNOBSERVED and B would have dropped it."""
        self.assertTrue(_until(lambda: "now silent" in _logs(A), 20))
        silent_since = time.time()
        samples = 0
        while time.time() - silent_since < 20.0:
            cloud = _probe("cloud")
            self.assertGreater(cloud.get("near_a", 0), 0,
                               f"A dropped after {time.time() - silent_since:.1f}s "
                               f"standing still: {cloud}")
            samples += 1
        self.assertNotIn("unobserved", _logs(A))
        print(f"  A still for 20 s: drawn in all {samples} samples")

    # --- 5. quiet but latched ---------------------------------------------

    def test_5_quiet_but_latched_is_dropped_on_its_unobserved_record(self):
        _docker("exec", A, "pkill", "-INT", "-f", "fake_amcl")
        said = _until(lambda: re.search(
            r"not drawing ent:robot:a: UNOBSERVED[^\n]*", _logs(B)), 15)
        self.assertTrue(said, "B never said why A stopped being drawn")
        self.assertTrue(_until(
            lambda: _probe("cloud")["width"] == 0, 10, 0.5))
        self.assertTrue(_until(
            lambda: _probe("cost", *PEER_AT)["max_cost"] == 0, 10, 0.5),
            "the clearing ring should have cleared A's cells")
        print(f"  quiet but latched: {said.group(0)[:90]}")

    def test_6_it_comes_back_on_its_next_pose(self):
        _docker("exec", "-d", A, "bash", "-lc",
                "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=10; "
                "python3 /work/test/ros/fake_amcl.py 3.0 0.0")
        self.assertTrue(_until(lambda: _probe("cloud").get("near_a", 0) > 0,
                               30, 1.0), "A should be drawn again")

    # --- 7. killed --------------------------------------------------------

    def test_7_killed_it_fades_on_its_window_and_the_ring_clears_it(self):
        """
        The ghost test: the clearing-ring row of the census's table, rerun
        through the library. A is SIGKILLed; B's cloud drops it on A's
        learned window, and B's costmap cell goes from lethal to free.
        """
        self.assertGreaterEqual(_probe("cost", *PEER_AT)["max_cost"], LETHAL)
        before = len(re.findall(r"not drawing ent:robot:a", _logs(B)))
        killed = time.time()
        _docker("kill", A)
        cleared = _until(
            lambda: _probe("cost", *PEER_AT)["max_cost"] == 0, 15, 0.2)
        took = time.time() - killed
        self.assertTrue(cleared, "A's cells stayed lethal: a ghost")
        reasons = re.findall(r"not drawing ent:robot:a: ([^\n]*)",
                             _logs(B))[before:]
        self.assertTrue(reasons, "B should have said why A went")
        self.assertIn("window", reasons[0],
                      "a killed peer goes on its window, not its lease")
        self.assertFalse([r for r in reasons if "disposed" in r],
                         "a crash is not a removal")
        after = _probe("path", *GOAL)
        self.assertLess(after["max_abs_y"], 0.05, after)
        print(f"  killed: cost at A back to 0 within {took:.1f}s (probe "
              f"overhead included); B said: {reasons[0][:70]}")

    # --- 8. the session ---------------------------------------------------

    def test_8_the_session_is_typed_mcap_the_gateway_serves(self):
        try:
            import jsonschema
        except ImportError:                                 # pragma: no cover
            self.skipTest("jsonschema is needed to validate payloads")
        if shutil.which("spatialdds-serve") is None:
            self.skipTest("spatialdds-serve is not on PATH")
        import test_acceptance_mcap_gateway as gw
        _docker("kill", "-s", "INT", RECORDER)
        _docker("wait", RECORDER, timeout=30)
        counts = json.loads(_logs(RECORDER).strip().splitlines()[-1])
        mcap = self.out / "session.mcap"
        self.assertTrue(mcap.exists() and mcap.stat().st_size > 0)
        print("  recorded:")
        for topic, n in sorted(counts.items()):
            print(f"    {n:>5}  {topic}")
        server = gw.RecordedThenServed._serve(mcap)
        try:
            _s, schema = gw._get(
                "/.well-known/spatialdds/schemas/spatial_owm_Entity")
            _s, collection = gw._get("/bare-scene/world/entity/v1", gw.TOKEN)
            items = (collection if isinstance(collection, list)
                     else collection.get("items")
                     or collection.get("results") or [collection])
            payloads = [i.get("value", i) for i in items]
            for payload in payloads:
                jsonschema.validate(payload, schema)
            ids = sorted(p["entity_id"] for p in payloads)
        finally:
            server.terminate()
            server.wait(timeout=10)
        self.assertEqual(ids, ["ent:robot:a", "ent:robot:b"])
        self.assertGreater(counts.get("spatialdds/bare-scene/world/model_pose/v1",
                                      0), 50)
        print(f"  gateway served and validated {len(payloads)} entities: "
              f"{ids}")


if __name__ == "__main__":
    unittest.main()
