# Releasing `spatialdds-ros` to PyPI

The distribution is `spatialdds-ros`; the import stays `spatialdds_ros`. Both
spellings resolve to the same project on PyPI, which normalises `_` and `-`,
and the name was unclaimed when 0.3.0 was prepared (2026-10-20).

Published by trusted publishing from a tag, by
`.github/workflows/publish-spatialdds-ros.yml`. The workflow holds no
credential.

## One-time, and the only reason the workflow is inert

PyPI needs a trusted publisher pointing at this exact triple. Until it exists,
a tag push builds the distributions and the publish job fails at the upload:

| field | value |
|---|---|
| PyPI project | `spatialdds-ros` |
| Owner / repository | `OpenArCloud` / `spatialdds-ros` |
| Workflow filename | `publish-spatialdds-ros.yml` |
| Environment | `pypi` |

Because the project does not exist yet, this is a **pending publisher**, added
under *Your projects → Publishing* rather than on a project's settings page.
The first successful upload creates the project and converts it.

## The drill

A release is cut from a commit whose gates already passed, which is why the
workflow does not re-run them.

1. `python3 -m pytest test/ -q` — green, with `test_packaging.py` among them.
2. Bump the version in **three** places, to the same number:
   `spatialdds_ros/__init__.py`, `setup.py`, `package.xml`.
3. `python3 -m pytest test/test_packaging.py -q` — it asserts those three
   agree, builds a wheel, and reads the rendered page out of the wheel's
   `METADATA` rather than out of `README.md`. Those are different artifacts
   and only one of them is what PyPI shows.
4. Commit, then `git tag spatialdds-ros-v<version> && git push origin
   spatialdds-ros-v<version>`. The workflow's build job re-checks the tag
   against all three declarations, so a mistyped tag does not reach PyPI.
5. Approve the `pypi` environment if the repository is set to require it,
   then confirm: `pip download spatialdds-ros==<version> --no-deps` into an
   empty directory.
6. **Check that what was published is what the pushed commit builds.**

## Step 6, in full

Name and size are not enough, and whole-file hashes are worse than nothing: a
wheel is not bit-reproducible across build-tool versions, so comparing whole
files reports a difference for every release built on a different machine and
trains you to ignore it. Compare the *payload* file by file.

```sh
git archive <pushed commit> | (mkdir -p /tmp/t && cd /tmp/t && tar -xf -)
(cd /tmp/t && python3 -m pip wheel . --no-deps --no-build-isolation -w /tmp/t/dist)
python3 - <<'PY'
import glob, hashlib, sys, zipfile
def payload(whl):
    with zipfile.ZipFile(whl) as z:
        return {n: hashlib.sha256(z.read(n)).hexdigest()
                for n in z.namelist()
                if ".dist-info/" not in n and not n.endswith("/")}
a = payload(sys.argv[1] if len(sys.argv) > 1 else glob.glob("*.whl")[0])
b = payload(glob.glob("/tmp/t/dist/*.whl")[0])
differing = [n for n in sorted(set(a) | set(b)) if a.get(n) != b.get(n)]
print(f"{len(a)} payload files, {len(differing)} differing")
for n in differing:
    print("  ", n)
PY
```

**Any payload difference is a different artifact**, and the answer to one is a
new patch version cut from the pushed tree, not an explanation. `dist-info`
lines are expected to differ — `Metadata-Version` and the `setuptools (...)`
line in `WHEEL` move with the build machine — and are excluded above for that
reason rather than ignored silently.

### The payload, predicted

The count is derived from the source tree, not read off a build, so a build
that loses a file is a failure rather than a new number to write down.
`test_packaging.py::test_the_payload_file_count_is_what_the_tree_predicts`
holds the arithmetic:

| source | files |
|---|---|
| `spatialdds_ros/**/*.py` | 13 |
| `spatialdds_idl18/**/*.py` | 64 |
| `spatialdds_idl18/PROVENANCE.md`, as `package_data` | 1 |
| `data_files`: `resource/spatialdds_ros`, `package.xml` | 2 |
| **total payload** | **80** |

Measured on the 0.3.0 build: 80 files. `__pycache__`, `.pyc` and the idlc
`.idlpy_manifest` leavings are asserted absent — they are build artifacts of
the bindings' generation and nothing reads them at runtime.

## What this package declares, and what it refuses to

`install_requires` is `cyclonedds` and nothing else. `rclpy` is absent on
purpose: only `spatialdds_ros.robot_node` needs a ROS 2 environment, and it
imports rclpy inside its entry point rather than at module scope.
`test/test_no_ros_needed.py` enforces that by source inspection — so the
split is a gate rather than a convention, and `pip install spatialdds-ros` on
a machine that has never had ROS gets a working library.

The ROS side is declared in `package.xml`, which `rosdep` and `colcon` read
and pip never does. Two packaging systems, each declaring what it is
responsible for, neither a subset of the other.

## Why the metadata lives in `setup.py`

`ament_python` builds read `setup.py`, so that is the single source for every
value. A `pyproject.toml` holding `[project]` would fight it for ownership,
and one holding only `[build-system]` would add a file without moving any
metadata — which is why the PEP 639 `License-Expression` field is not used
and the deprecated `License ::` classifier is kept deliberately. `python -m
build` and `pip wheel` both work against `setup.py` alone; the 0.3.0
distributions were built that way.
