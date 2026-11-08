# Where these types came from

Generated SpatialDDS **1.8** Python bindings, vendored. Do not edit by hand —
every file here is idlc output plus two mechanical patches, and an edit would
be lost the next time they are regenerated.

| | |
|---|---|
| IDL source | `idl/v1.8/` in `OpenArCloud/SpatialDDS-spec`, at `9c3e1f2` (2026-10-15) |
| generator | `scripts/generate_types.py` in `OpenArCloud/SpatialDDS-demo` |
| vendored from | that repo's `spatialdds_idl18/` at `cf4ec23` |
| files | 64 (spec), plus `oarc_owm/` below |

The spec's `idl/v1.8` and the demo's copy were verified byte-identical at the
time of vendoring, so these bindings correspond to the specification's IDL and
not to a demo-local variant.

## `oarc_owm/`, added for 0.5.0

| | |
|---|---|
| IDL source | `idl/demo18/oarc_owm02.idl` in `OpenArCloud/SpatialDDS-demo`, last changed at `1ec36ca` |
| spec IDL beside it | the same `idl/v1.8` at `9c3e1f2` as the table above |
| generator | the same `scripts/generate_types.py`, its `generate()` with the package `spatialdds_idl18` |
| files | 3 (two modules and idlc's `.idlpy_manifest`, as every package here carries), and one import in this tree's `__init__.py` |

**Not a spec type.** `oarc.relationship` is the OWM 0.2 relationship
prototype, a deployment extension (§3.3.2) that SpatialDDS-demo registers and
its asserters publish; this library reads that lane for its claims layer and
writes it for perception publishing. It is vendored beside the spec's types
because there is nowhere else to get it, and it carries nothing the spec is
asked to adopt.

**Generated, not copied, and the run is its own check.** The generator was
run over the spec IDL at `9c3e1f2` together with `oarc_owm02.idl`, into a
scratch `spatialdds_idl18`. Every spec file it produced was byte-identical to
the 64 already vendored here (`diff -r`, `__pycache__` and `oarc_owm`
excluded), and `_field_aliases.py` too; the only differences were the new
`oarc_owm/` and its import in `__init__.py`. That one run reproduces this
tree and adds the module, so the new module comes from the same generator
and the same spec bytes as everything beside it. Its minimal XTypes
identifiers for `Relationship` and `EvidenceRef` equal those of
SpatialDDS-demo's own `spatialdds_demo_idl18.oarc_owm`, which is what DDS
matches on; the demo's `tests/test_idl_trees_agree.py` now holds them there.

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
