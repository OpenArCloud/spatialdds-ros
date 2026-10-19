#!/usr/bin/env python3
"""
Every vendored dataclass answers `typing.get_type_hints()`.

**The gate the vendored tree exists behind.** idlc annotates a nested member
with a fully-qualified *string* — `cov_pos:
'spatialdds_idl18.spatial.common.Mat3x3'` — while importing only
`from spatialdds_idl18 import spatial`. The root name in those annotations is
then unbound, and `get_type_hints` raises `NameError` on every generated
dataclass with a nested member. `generate_types.py` patches each module to
bind the root; this asserts the patch is present and effective on the tree
that actually ships.

CycloneDDS resolves annotations with its own machinery and never calls
`get_type_hints`, so a tree missing the patch puts correct bytes on the wire
and is simultaneously unreadable to anything that introspects types. That is
the shape worth guarding: it cannot be caught by sending a message and
getting it back.

It matters concretely here. This library validates its MCAP payloads against
published schemas, and schema generation is exactly dataclass introspection —
so a regeneration that dropped the patch would break the acceptance run with a
`NameError` from somewhere that looks unrelated, rather than here with the
reason.

**The tree is walked from disk, not from a list.** A written-down list of
modules checks only the ones somebody remembered, which is the failure the
upstream version of this gate was written to avoid.
"""
from __future__ import annotations

import dataclasses
import importlib
import pkgutil
import sys
import typing
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
if str(REPO) not in sys.path:
    sys.path.insert(0, str(REPO))

import spatialdds_idl18 as vendored                            # noqa: E402


def _modules() -> list[str]:
    return sorted(m.name for m in pkgutil.walk_packages(
        vendored.__path__, vendored.__name__ + "."))


def _dataclasses_in(module) -> list[tuple[str, type]]:
    """Only the ones this module defines, so a re-export is not counted twice."""
    return [(name, obj) for name, obj in vars(module).items()
            if dataclasses.is_dataclass(obj)
            and getattr(obj, "__module__", "") == module.__name__]


class TheVendoredTreeIsIntrospectable(unittest.TestCase):
    def test_the_tree_is_not_empty(self):
        """
        The guard on the guard.

        `walk_packages` over a missing or empty tree returns nothing, and every
        other test here would then pass while checking no types at all — the
        instrument-blindness shape. 64 files were vendored; the floor is set
        well below that so a legitimate trim does not fail, and far above zero.
        """
        modules = _modules()
        self.assertGreater(len(modules), 20,
                           f"only {len(modules)} modules found under "
                           f"{vendored.__path__}; the vendored tree is missing "
                           f"or this gate is pointed at the wrong place")

    def test_every_module_imports(self):
        failures = []
        for name in _modules():
            try:
                importlib.import_module(name)
            except Exception as error:                 # noqa: BLE001
                failures.append(f"{name}: {type(error).__name__}: {error}")
        self.assertEqual(failures, [], "\n".join(failures))

    def test_every_dataclass_resolves_its_hints(self):
        """The patch's actual effect, asserted where it is visible."""
        checked = 0
        failures = []
        for name in _modules():
            module = importlib.import_module(name)
            for cls_name, cls in _dataclasses_in(module):
                try:
                    typing.get_type_hints(cls)
                    checked += 1
                except Exception as error:             # noqa: BLE001
                    failures.append(
                        f"{name}.{cls_name}: {type(error).__name__}: {error}")
        self.assertEqual(failures, [],
                         f"{len(failures)} of {checked + len(failures)} "
                         f"generated dataclasses cannot be introspected:\n"
                         + "\n".join(failures[:12]))
        self.assertGreater(checked, 50,
                           f"only {checked} dataclasses were checked; this gate "
                           f"should be seeing the whole tree")
        print(f"\n  {checked} generated dataclasses resolve their type hints")

    def test_the_root_binding_patch_is_present_where_it_is_needed(self):
        """
        Belt and braces, and not redundant with the test above.

        `get_type_hints` succeeding proves the *current* tree resolves. This
        asserts the mechanism — that a module carrying dotted annotations also
        imports the root — so a regeneration that resolved by luck (say, every
        nested member happening to be local) still reports the missing patch
        rather than passing quietly until the next type is added.
        """
        root = vendored.__name__
        missing = []
        for path in sorted(Path(vendored.__path__[0]).rglob("*.py")):
            source = path.read_text()
            needs = f"'{root}." in source or f'"{root}.' in source
            if needs and f"\nimport {root}\n" not in source:
                missing.append(str(path.relative_to(REPO)))
        self.assertEqual(missing, [],
                         "these modules carry fully-qualified string "
                         "annotations but never bind the root package, so "
                         "`get_type_hints` will fail on them as soon as one is "
                         "introspected:\n" + "\n".join(missing))


if __name__ == "__main__":
    unittest.main()
