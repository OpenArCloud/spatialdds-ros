# spatialdds_ros, findings

Defects in this library found from outside it. Each says what was observed,
how it was reproduced, and the smallest change that would close it. A filing is
not a fix: each is closed by its own commit, which names the finding.

## 1. The recorder cannot run from an installed wheel

*Filed 2026-10-24, from the spatialdds-web native-SDK census. Affects every
release: 0.3.0 and 0.4.0 as published on PyPI.*

`spatialdds_ros.recording` finds the IDL it embeds in recordings relative to
its own file:

```python
IDL_DIR = Path(__file__).resolve().parent.parent / "idl" / "v1.8"   # recording.py:53
```

In a source checkout, that is the repository's `idl/v1.8`. In an installed wheel
it is `site-packages/idl/v1.8`, which does not exist: `idl/` is in no
`packages`, `package_data` or `data_files` entry (`setup.py:50-59`), so no
wheel has ever carried it.

**Reproduced against the published artifact**, not a local build. The 0.4.0
wheel was downloaded from PyPI and installed into a fresh venv with
`cyclonedds==11.0.1` and `spatialdds[server]==0.6.0`. Its 88 entries include
no `.idl` file. Then:

```python
cfg = Config(scene="probe", entity_id="ent:probe", frame_id="map")
with recording.Recorder(out, cfg) as rec:
    rec.write(topic, sample)
```

```
FileNotFoundError: [Errno 2] No such file or directory:
  '…/venv/lib/python3.12/site-packages/idl/v1.8/types.idl'
```

The failure is on the first `write()`, when the first schema is flattened, so
opening a `Recorder` succeeds and only recording fails. 0.3.0, installed from
PyPI, resolves the same missing path.

**Why the suite never saw it.** Every test that records imports the library
from the checkout, where `../idl/v1.8` exists. `test_packaging.py` checks what
the wheel contains, but not that the recorder can find its IDL in it. The
environment the check ran in picked which defects were reachable.

**A second gap on the same path.** `Recorder` imports `spatialdds.server.pin`
(`recording.py:108`), but `spatialdds` is not in `install_requires`. It is
present in the test toolchain only (`requirements.txt`). An install of
`spatialdds_ros` alone fails one step earlier, with `ModuleNotFoundError`.

**Smallest change.**

1. Ship the IDL as package data, under the package that reads it, for example
   `spatialdds_ros/idl/v1.8/…`.
2. Resolve `IDL_DIR` from the package rather than from the repository layout.
3. Declare the recorder's dependency as an extra, `spatialdds_ros[record]`,
   pulling `spatialdds[server]` and `mcap`. The core install stays light.
4. Add a gate that builds the wheel, installs it into a clean venv, and records
   one sample, so the subject is the shipped artifact.

**Where this goes next.** The native SDK in spatialdds-web takes over the
recording tap, with its schema closures computed from IDL that ships inside
that package. Adopting it closes this finding for ROS, but the published
releases are defective until a fixed release replaces them.
