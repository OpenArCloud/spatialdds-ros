#!/usr/bin/env python3
"""
The library imports, and is testable, on a machine with no ROS.

**This is a gate, not a habit.** Every other test in this suite depends on it:
they run on a laptop with no ROS install, and they only can because no module
reachable from `import spatialdds_ros` pulls in rclpy. One top-level
`import rclpy` anywhere in the package would take the whole suite offline, and
the symptom would be a CI failure in a module that looks unrelated.

So this asserts the discipline directly — by source inspection, which catches
it whether or not rclpy happens to be installed on the machine running the
tests. A test that only checked "the import worked" would pass on a developer's
ROS workstation and fail in CI.
"""
from __future__ import annotations

import ast
import importlib
import pkgutil
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import spatialdds_ros                                         # noqa: E402

#: Modules that may name ROS packages, and only inside a function body.
ROS_PACKAGES = {"rclpy", "tf2_ros", "geometry_msgs", "nav_msgs", "nav2_msgs",
                "builtin_interfaces", "std_msgs", "action_msgs"}

#: The one module whose job is to be the ROS entry point. Everything in it is
#: still function-scoped; it is listed so a reader knows where to look.
ENTRY_POINT = "spatialdds_ros.robot_node"


def _package_modules() -> list[str]:
    return sorted(m.name for m in pkgutil.walk_packages(
        spatialdds_ros.__path__, spatialdds_ros.__name__ + "."))


def _module_path(name: str) -> Path:
    return REPO / Path(*name.split(".")).with_suffix(".py")


def _top_level_ros_imports(source: str) -> list[str]:
    """ROS imports at module scope — the ones that run on import."""
    tree = ast.parse(source)
    found = []
    for node in tree.body:                      # module scope only
        if isinstance(node, ast.Import):
            for alias in node.names:
                if alias.name.split(".")[0] in ROS_PACKAGES:
                    found.append(alias.name)
        elif isinstance(node, ast.ImportFrom) and node.module:
            if node.module.split(".")[0] in ROS_PACKAGES:
                found.append(node.module)
    return found


class NothingPullsInRosOnImport(unittest.TestCase):
    def test_the_package_has_modules_to_check(self):
        """The guard on the guard: an empty walk would pass everything below."""
        self.assertGreater(len(_package_modules()), 4,
                           "the package walk found almost nothing; this gate "
                           "is pointed at the wrong place")

    def test_no_module_imports_ros_at_module_scope(self):
        offenders = []
        for name in _package_modules():
            path = _module_path(name)
            if not path.exists():
                continue
            for imported in _top_level_ros_imports(path.read_text()):
                offenders.append(f"{name} imports {imported} at module scope")
        self.assertEqual(
            offenders, [],
            "a module-scope ROS import takes the whole unit suite offline on "
            "any machine without ROS:\n" + "\n".join(offenders))

    def test_every_module_actually_imports_here(self):
        """
        The behavioural half. The source check above is the one that holds on a
        ROS workstation; this one proves the claim on *this* machine, which has
        no ROS at all.
        """
        self.assertNotIn("rclpy", sys.modules,
                         "something imported rclpy before this test ran")
        for name in _package_modules():
            with self.subTest(module=name):
                importlib.import_module(name)
        self.assertNotIn(
            "rclpy", sys.modules,
            "importing the package pulled in rclpy, which is the thing this "
            "gate exists to prevent")

    def test_the_entry_point_is_importable_without_ros(self):
        """
        `robot_node` is the ROS module and still must import cleanly: setup.py
        names `robot_node:main` as a console script, so a module-scope ROS
        import there would break `pip install` introspection and any tool that
        loads entry points.
        """
        module = importlib.import_module(ENTRY_POINT)
        self.assertTrue(callable(module.main))

    def test_the_config_reader_can_be_exercised_without_ros(self):
        """
        `_declare` takes a node and asks it for parameters, so a stand-in with
        the same two methods exercises the whole parameter-collection path —
        including the partial-anchor refusal — with no rclpy anywhere.
        """
        from spatialdds_ros.config import ConfigError
        from spatialdds_ros.robot_node import _declare

        class FakeParameter:
            def __init__(self, value):
                self.value = value

        class FakeNode:
            def __init__(self, values):
                self._values = dict(values)
            def declare_parameter(self, name, default):
                self._values.setdefault(name, default)
            def get_parameter(self, name):
                return FakeParameter(self._values[name])

        good = FakeNode({"scene": "bare-scene", "entity_id": "ent:robot:r1",
                         "frame_id": "map/bare-scene"})
        config = _declare(good)
        self.assertEqual(config.scene, "bare-scene")
        self.assertIsNone(config.anchor)
        self.assertFalse(config.publishes_georeference)

        partial = FakeNode({"scene": "bare-scene", "entity_id": "ent:robot:r1",
                            "frame_id": "map/bare-scene",
                            "anchor_lat_deg": 30.28, "anchor_lon_deg": -97.74})
        with self.assertRaises(ConfigError) as caught:
            _declare(partial)
        self.assertIn("all three", str(caught.exception))
        self.assertIn("none is a supported mode", str(caught.exception),
                      "the refusal should point at the supported alternative")

        whole = FakeNode({"scene": "bare-scene", "entity_id": "ent:robot:r1",
                          "frame_id": "map/bare-scene",
                          "anchor_lat_deg": 30.28396857,
                          "anchor_lon_deg": -97.73974829,
                          "anchor_height_m": 145.083})
        self.assertTrue(_declare(whole).publishes_georeference)


if __name__ == "__main__":
    unittest.main()
