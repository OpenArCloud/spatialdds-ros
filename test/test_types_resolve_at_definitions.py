#!/usr/bin/env python3
"""
Every generated type this library imports is imported from where it is defined.

**This gate exists because the same trap has cost four separate debugging
rounds in this programme.** The generated trees re-export types across module
boundaries as *annotated typedef aliases*. An alias looks like the real thing
to a reader and to an IDE: `from spatialdds_idl18.spatial.disco import Aabb3`
imports successfully, and `spatial.disco` is exactly where you would look for
it when building an `Announce`. But the alias reports zero dataclass members
and raises

    TypeError: 'ForwardRef' object is not callable

the moment you try to construct it. `spatial.disco` alone re-exports six such
aliases — `Aabb3`, `FrameRef`, `Time`, `PoseSE3`, `SpatialUri`, `MetaKV` —
alongside eight names that *are* real there, so "it is in this module" is no
guide at all.

The four rounds it cost: a 1.8 census that read zero members off
`spatial.argeo` and nearly concluded the types were empty; a cross-era test
harness that failed on `GeoPose`; the same harness again on a second name in
the same import block; and this library's announce, on `Aabb3` and then `Time`
and `PoseSE3` in turn. Three of those were fixed one name at a time, which is
why this is a gate and not a note: the lesson was written down after the
second round and did not prevent the third or fourth.

So the rule is enforced rather than remembered — **resolve a type at its
definition** — and enforced over the library's whole source rather than over
the names somebody thought to list.
"""
from __future__ import annotations

import ast
import dataclasses
import importlib
import pkgutil
import sys
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import spatialdds_ros                                         # noqa: E402

GENERATED_ROOT = "spatialdds_idl18"


def _library_sources() -> list[Path]:
    paths = []
    for module in pkgutil.walk_packages(spatialdds_ros.__path__,
                                        spatialdds_ros.__name__ + "."):
        path = REPO / Path(*module.name.split(".")).with_suffix(".py")
        if path.exists():
            paths.append(path)
    return sorted(paths)


def _generated_imports(path: Path) -> list[tuple[str, str, int]]:
    """Every `from spatialdds_idl18.* import Name` in a file, at any scope."""
    found = []
    for node in ast.walk(ast.parse(path.read_text())):
        if not isinstance(node, ast.ImportFrom) or not node.module:
            continue
        if node.module.split(".")[0] != GENERATED_ROOT:
            continue
        for alias in node.names:
            found.append((node.module, alias.name, node.lineno))
    return found


def _constructible(obj) -> bool:
    """
    Can this be called to make a value?

    A real generated type is a dataclass or an enum class. An annotated typedef
    alias is a `typing` construct: it passes `is not None`, it has a `__name__`
    through the annotation, and it raises on call. `isinstance(obj, type)` is
    what actually separates them.
    """
    return isinstance(obj, type) or dataclasses.is_dataclass(obj)


class EveryImportedTypeIsConstructible(unittest.TestCase):
    def test_there_are_sources_to_check(self):
        """The guard on the guard: an empty walk passes everything below."""
        self.assertGreater(len(_library_sources()), 5,
                           "the source walk found almost nothing")

    def test_the_library_imports_generated_types_at_all(self):
        """And that those imports are being seen — a parser that silently
        matched nothing would make this suite vacuous."""
        total = sum(len(_generated_imports(p)) for p in _library_sources())
        self.assertGreater(total, 10,
                           f"only {total} generated imports found across the "
                           f"library; the AST walk is probably wrong")

    def test_no_imported_name_is_an_alias(self):
        offenders = []
        for path in _library_sources():
            for module_name, name, lineno in _generated_imports(path):
                module = importlib.import_module(module_name)
                obj = getattr(module, name, None)
                if obj is None:
                    offenders.append(
                        f"{path.relative_to(REPO)}:{lineno}: {module_name} has "
                        f"no attribute {name}")
                elif not _constructible(obj):
                    offenders.append(
                        f"{path.relative_to(REPO)}:{lineno}: {name} imported "
                        f"from {module_name} is a re-export alias, not the "
                        f"definition — constructing it raises "
                        f"TypeError: 'ForwardRef' object is not callable")
        self.assertEqual(
            offenders, [],
            "resolve each of these at its definition:\n" + "\n".join(offenders))

    def test_the_gate_can_tell_an_alias_from_a_definition(self):
        """
        The instrument's own injection test.

        If `_constructible` returned True for everything, the check above would
        pass on any source at all. So this pins a known alias and a known
        definition from the generated tree, which also documents the trap by
        example for whoever reads this next.
        """
        disco = importlib.import_module(f"{GENERATED_ROOT}.spatial.disco")
        core = importlib.import_module(f"{GENERATED_ROOT}.spatial.core")

        self.assertFalse(
            _constructible(getattr(disco, "Aabb3")),
            "spatial.disco.Aabb3 is a re-export alias; if this now reports "
            "constructible, either the generator changed or the gate is broken")
        self.assertTrue(
            _constructible(getattr(core, "Aabb3")),
            "spatial.core.Aabb3 is the definition")

        # And the alias really does raise when called, which is the symptom the
        # whole gate is about.
        with self.assertRaises(TypeError):
            getattr(disco, "Aabb3")(min_xyz=[0.0, 0.0, 0.0],
                                    max_xyz=[0.0, 0.0, 0.0])

    def test_every_type_the_library_imports_can_be_introspected(self):
        """
        Belt and braces against the other half of the same family: a name can
        be a real class and still be unusable if its annotations do not
        resolve. The vendored tree's own gate covers the whole tree; this
        covers what the library actually touches.
        """
        import typing
        failures = []
        for path in _library_sources():
            for module_name, name, lineno in _generated_imports(path):
                obj = getattr(importlib.import_module(module_name), name, None)
                if obj is None or not dataclasses.is_dataclass(obj):
                    continue
                try:
                    typing.get_type_hints(obj)
                except Exception as error:                    # noqa: BLE001
                    failures.append(f"{name} ({module_name}): {error}")
        self.assertEqual(failures, [], "\n".join(failures))


if __name__ == "__main__":
    unittest.main()
