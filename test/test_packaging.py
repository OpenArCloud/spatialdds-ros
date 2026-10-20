#!/usr/bin/env python3
"""
The distribution, checked as the thing that ships rather than as source.

Three classes of defect this exists for, each of which has cost a release
somewhere in this programme:

* **The version in three places.** `setup.py`, `package.xml` and
  `__init__.py` all carry it, the release tag carries it a fourth time, and
  any pair disagreeing means a tag that points at something never released.
* **A README that renders as damage on PyPI.** A relative link has no
  repository around it to resolve against, and a table whose header row is
  empty renders as a header of blank columns. Both look fine in a GitHub
  preview, which is where they get checked and where they are not broken.
* **A dependency split that is only a comment.** `rclpy` being absent from
  `install_requires` is load-bearing: it is what makes `pip install
  spatialdds-ros` a thing you can do without ROS. An absence is not
  self-enforcing, so it is asserted here from both sides.

**The rendered page is read out of a built wheel's `METADATA`, not out of
`README.md`.** Those are different artifacts: the source file is what the
author edits, the METADATA body is what PyPI renders, and everything between
them -- the read, the content-type declaration, the packaging tool -- is
exactly where a long description gets truncated, re-encoded or silently
dropped. Checking the source file would be checking the wrong one.
"""
from __future__ import annotations

import re
import subprocess
import sys
import tempfile
import unittest
import zipfile
from email import message_from_string
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import spatialdds_ros                                          # noqa: E402

README = REPO / "README.md"
SETUP = REPO / "setup.py"
PACKAGE_XML = REPO / "package.xml"
DIST_NAME = "spatialdds-ros"
IMPORT_NAME = "spatialdds_ros"

#: Built once for the whole module: a wheel build is seconds, and every
#: METADATA assertion below wants the same one.
_WHEEL: list = []


def built_wheel() -> Path:
    if _WHEEL:
        return _WHEEL[0]
    out = Path(tempfile.mkdtemp(prefix="spatialdds-ros-wheel-"))
    # `pip wheel`, not `python -m build`: no network and no build isolation,
    # so this runs in CI and on a laptop with the same inputs. `--no-deps`
    # because the dependency set is asserted from METADATA, not resolved.
    result = subprocess.run(
        [sys.executable, "-m", "pip", "wheel", ".", "--no-deps",
         "--no-build-isolation", "-w", str(out)],
        cwd=str(REPO), capture_output=True, text=True)
    if result.returncode != 0:
        raise unittest.SkipTest(
            "BUILD-UNAVAILABLE: pip wheel failed, so the shipped artifact "
            f"cannot be inspected:\n{result.stdout[-2000:]}\n"
            f"{result.stderr[-2000:]}")
    wheels = sorted(out.glob("*.whl"))
    assert wheels, f"pip wheel reported success and produced nothing: {out}"
    _WHEEL.append(wheels[0])
    return wheels[0]


def metadata():
    with zipfile.ZipFile(built_wheel()) as zf:
        name = next(n for n in zf.namelist()
                    if n.endswith(".dist-info/METADATA"))
        return message_from_string(zf.read(name).decode("utf-8"))


def payload_names() -> list[str]:
    """Everything in the wheel that is not `.dist-info` bookkeeping."""
    with zipfile.ZipFile(built_wheel()) as zf:
        return sorted(n for n in zf.namelist()
                      if ".dist-info/" not in n and not n.endswith("/"))


class TheVersionIsOneNumber(unittest.TestCase):
    def test_all_three_declarations_agree(self):
        setup_version = re.search(r'version="([^"]+)"',
                                  SETUP.read_text()).group(1)
        xml_version = re.search(r"<version>([^<]+)</version>",
                                PACKAGE_XML.read_text()).group(1)
        self.assertEqual(setup_version, spatialdds_ros.__version__)
        self.assertEqual(xml_version, spatialdds_ros.__version__)
        print(f"\n  version {spatialdds_ros.__version__} in all three places")

    def test_the_built_wheel_carries_that_version(self):
        self.assertEqual(metadata()["Version"], spatialdds_ros.__version__)


class TheReadmeSurvivesLeavingTheRepository(unittest.TestCase):
    """
    What PyPI does to prose that assumed a repository around it.

    Both of these were real in this README until 0.3.0: four references to
    files by bare path, and two tables whose header row was entirely empty.
    """

    LINK = re.compile(r"\]\(([^)]+)\)")

    def test_no_relative_links(self):
        bad = [t for t in self.LINK.findall(README.read_text())
               if not t.startswith(("https://", "http://", "#"))]
        self.assertEqual(
            bad, [],
            "PyPI has no repository to resolve these against, so each one "
            f"renders as a dead link: {bad}")

    def test_no_empty_table_header_cells(self):
        lines = README.read_text().splitlines()
        bad = []
        for i, line in enumerate(lines):
            nxt = lines[i + 1].strip() if i + 1 < len(lines) else ""
            if not re.match(r"^\|[\s|:-]*\|$", nxt) or "-" not in nxt:
                continue
            cells = [c.strip() for c in line.strip().strip("|").split("|")]
            if any(c == "" for c in cells):
                bad.append(f"line {i + 1}: {line}")
        self.assertEqual(
            bad, [],
            "a header row with an empty cell renders as a blank column "
            f"heading:\n" + "\n".join(bad))

    def test_the_page_pypi_renders_is_the_readme(self):
        """
        Read out of the wheel, which is the artifact PyPI is handed.
        """
        meta = metadata()
        self.assertEqual(meta.get("Description-Content-Type"), "text/markdown",
                         "without this PyPI renders the markdown as plain text")
        body = meta.get_payload()
        source = README.read_text()
        self.assertEqual(
            body.strip(), source.strip(),
            "the shipped description is not the README; something between "
            "the file and the metadata changed it")
        self.assertIn("# spatialdds_ros", body, "including its first heading")
        print(f"  METADATA carries {len(body.splitlines())} lines of "
              f"markdown, identical to README.md")

    def test_the_shipped_page_has_no_relative_links_either(self):
        """
        The same rule, applied to the artifact rather than the source. Separate
        from the test above on purpose: that one compares two things, and a
        check that only compares cannot notice both of them being wrong.
        """
        bad = [t for t in self.LINK.findall(metadata().get_payload())
               if not t.startswith(("https://", "http://", "#"))]
        self.assertEqual(bad, [], f"relative links in the shipped page: {bad}")


class TheDependencySplitIsReal(unittest.TestCase):
    """
    Asserted from the built metadata and from `package.xml`, because the claim
    has two sides and an absence on one of them proves nothing on its own.
    """

    ROS_ONLY = ("rclpy", "tf2_ros", "nav2_msgs", "geometry_msgs", "nav_msgs",
                "ament")

    def test_the_wheel_requires_cyclonedds(self):
        requires = metadata().get_all("Requires-Dist") or []
        self.assertTrue(
            any(r.split()[0].split(">")[0].split("=")[0].strip() == "cyclonedds"
                for r in requires),
            f"the bus is not optional: {requires}")
        print(f"  Requires-Dist: {requires}")

    def test_the_wheel_requires_nothing_from_ros(self):
        requires = " ".join(metadata().get_all("Requires-Dist") or []).lower()
        found = [n for n in self.ROS_ONLY if n in requires]
        self.assertEqual(
            found, [],
            "pip must not be asked to install ROS -- that is what makes this "
            f"installable without it: {found}")

    def test_package_xml_still_declares_the_ros_side(self):
        """
        The other half. If these ever moved into `install_requires` the split
        would have collapsed in the direction the absence above cannot see.
        """
        xml = PACKAGE_XML.read_text()
        for name in ("rclpy", "tf2_ros", "nav2_msgs"):
            self.assertIn(f"<exec_depend>{name}</exec_depend>", xml)

    def test_the_readme_install_line_names_the_distribution(self):
        """
        Claims the `pip install spatialdds-ros` fence for
        `test_readme.py`'s accounting, and checks the name it prints is the
        name that is published -- a rename would otherwise leave the README
        telling people to install something that does not exist.
        """
        self.assertIn(f"pip install {DIST_NAME}", README.read_text())
        self.assertEqual(metadata()["Name"], DIST_NAME)
        self.assertEqual(spatialdds_ros.__name__, IMPORT_NAME,
                         "the import name does not follow the dist name")


class TheWheelCarriesWhatItShould(unittest.TestCase):
    def test_both_packages_ship(self):
        names = payload_names()
        self.assertTrue(any(n.startswith("spatialdds_ros/") for n in names))
        self.assertTrue(any(n.startswith("spatialdds_idl18/") for n in names),
                         "the generated bindings ship alongside; see "
                         "spatialdds_idl18/PROVENANCE.md")

    def test_provenance_ships_with_the_bindings_it_describes(self):
        """The README's table points at it, so it has to be in the wheel."""
        self.assertIn("spatialdds_idl18/PROVENANCE.md", payload_names())

    def test_no_bytecode_and_no_build_leavings(self):
        bad = [n for n in payload_names()
               if "__pycache__" in n or n.endswith((".pyc", ".idlpy_manifest"))]
        self.assertEqual(bad, [], f"build leavings in the payload: {bad}")

    def test_the_payload_file_count_is_what_the_tree_predicts(self):
        """
        Predicted from the source tree, not read off the build. A count that
        is only ever read back from the artifact cannot notice the artifact
        losing a file.
        """
        def count(root: Path, suffix: str) -> int:
            return len([p for p in (REPO / root).rglob(f"*{suffix}")
                        if "__pycache__" not in p.parts])
        predicted = (count(Path("spatialdds_ros"), ".py")
                     + count(Path("spatialdds_idl18"), ".py")
                     + 1                       # PROVENANCE.md, as package_data
                     + 2)                      # data_files: resource, package.xml
        actual = len(payload_names())
        self.assertEqual(
            predicted, actual,
            f"predicted {predicted} payload files from the tree, the wheel "
            f"has {actual}:\n  " + "\n  ".join(payload_names()))
        print(f"  payload: {actual} files, as predicted from the tree")


if __name__ == "__main__":
    unittest.main()
