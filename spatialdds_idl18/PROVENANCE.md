# Where these types came from

Generated SpatialDDS **1.8** Python bindings, vendored. Do not edit by hand —
every file here is idlc output plus two mechanical patches, and an edit would
be lost the next time they are regenerated.

| | |
|---|---|
| IDL source | `idl/v1.8/` in `OpenArCloud/SpatialDDS-spec`, at `9c3e1f2` (2026-10-15) |
| generator | `scripts/generate_types.py` in `OpenArCloud/SpatialDDS-demo` |
| vendored from | that repo's `spatialdds_idl18/` at `cf4ec23` |
| files | 64 |

The spec's `idl/v1.8` and the demo's copy were verified byte-identical at the
time of vendoring, so these bindings correspond to the specification's IDL and
not to a demo-local variant.

## Why vendored rather than depended on

The `spatialdds` distribution on PyPI (0.5.4, this library's toolchain) ships
the gateway, canonical JSON and the record helpers — it does **not** ship IDL
bindings. There is no published package to depend on, and generating at
install time would need `idlc` and a CycloneDDS build on every machine that
installs this library, including CI runners and a robot. So they are vendored,
with the provenance above standing in for a version constraint.

## The two patches, and why they are the generator's job

Both are applied by `generate_types.py` after idlc runs, and both are
mechanical consequences of idlc's output rather than choices:

**1. The root package is bound.** idlc annotates a nested member with a
fully-qualified *string* — `cov_pos: 'spatialdds_idl18.spatial.common.Mat3x3'`
— while importing only `from spatialdds_idl18 import spatial`. The root name in
those annotations is therefore unbound and `typing.get_type_hints()` raises
`NameError` on every generated dataclass with a nested member. So each such
module also does `import spatialdds_idl18`.

CycloneDDS resolves annotations with its own machinery and never calls
`get_type_hints`, which is why this defect sat unnoticed for a year while
everything on the wire worked. It still mattered: `get_type_hints` is how
dataclass-based tooling reads a type, so schema generators and validators
could not introspect these bindings at all. Fixed upstream in SpatialDDS-demo
`b856788`, and in openvps `26c7182`.

**2. Union discriminators are imported.** idlc emits some discriminators as
bare names it never imports; the generator resolves each against the module
that defines it and writes the import.

## The gate

`test/test_generated_types.py` imports every module in this tree and calls
`typing.get_type_hints()` on every generated dataclass. It walks the tree from
disk rather than from a written-down list, because a hand-kept list checks only
the modules somebody remembered.

That gate is the reason this directory can be trusted by anything that
introspects types — which, in this library, is the MCAP schema validation in
the acceptance run.

## Regenerating

Not from this repository. The generator and the IDL both live elsewhere; to
move to a newer 1.8 revision, regenerate in SpatialDDS-demo and re-vendor,
updating the table above. The gate will tell you immediately if a regeneration
dropped either patch.
