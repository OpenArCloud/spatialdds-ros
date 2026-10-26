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

## 2–9. From the native-SDK census

*Filed 2026-10-24, from spatialdds-web's `docs/NATIVE-SDK-CENSUS.md`, which
inventoried this library beside three others and then built
`spatialdds.dds`. This library was the model for several of the SDK's
defaults: autodispose off, a reader that keeps a record on no-writers, the
pose floor, and the one copy of the frame arithmetic. These are what the
census found besides. Read at f36958b. Each was checked at its cited line;
none was run on a bus except where it says so.*

**2. The ROS node never announces.** `robot_node.py` has no `Announcer`,
publishes no `Announce` and never departs. `package.xml:9` describes the
node's "presence announced", and `announce.py:4-6` says a robot should be
discoverable. Only the standalone participant announces.

**3. The standalone announce omits a lane it writes.** The adapter writes the
`FrameTransform` lane when an anchor is set (`adapter.py:84-87`), but
`announce._topics` lists only entity, pose and command (`announce.py:74-86`).
A consumer that discovers lanes from the announce never subscribes to the
georeference.

**4. `TopicMeta.type` carries the IDL name, not the registered type.** The
announce's rows say `type="spatial.owm.Entity"`, `"spatial.owm.ModelPose"` and
`"spatial.owm.ModelCommand"` (`announce.py:76-84`). §3.3.2 makes `type` a
registered value, which at v1.8 is `model_entity`, `model_pose` and
`model_command`, or a documented extension. `qos_profile="MODEL_LATCHED"` is
not a §3.3.3 profile either. Its naming half is already queued
(SpatialDDS-demo's `owm-1.8-spec-queue.md`). The native SDK announces
`model_entity` with a registered profile, so one robot announced each way
would describe the same lane in two vocabularies.

**5. `frames.py`'s docstring has the direction backwards.** `frames.py:9-10`
says `T_parent_child` maps "parent into child". The code, the georeference's
own docstring (`:110-111`) and the composition note (`:230-232`) all treat it
as the child's pose in the parent, mapping child into parent. The code is
right: the SDK copied it and composes it in both hemispheres.

**6. The bindings say 1.7.** `spatialdds_idl18/__init__.py:4` reads "Generated
SpatialDDS 1.7 types" over the 1.8 bindings. The label came from the demo's
generator, which wrote it unconditionally. The SDK's generator reads the label
from the pin.

**7. The ROS install path has no Python cyclonedds.** `package.xml` lists rclpy
and the message packages, but no Python cyclonedds binding.
`rmw_cyclonedds_cpp` ships the C library, and `robot_node.py:10-16` says itself
that the binding is installed separately. The test image works around it with a
pip install (`test/ros/Dockerfile:34-42`). Whether a rosdep key exists was not
checked.

**8. SIGTERM is unhandled in the ROS node.** `robot_node.py:409-415` catches
`KeyboardInterrupt` only, so a SIGTERM (systemd, `docker stop`) skips
`destroy_node`. With autodispose off this is harmless to readers today: the
record stands and liveliness ends. It becomes a gap once the node announces
(item 2), because the announce would then never depart.

**9. The declared binding is broader than the tested one.**
`install_requires=["cyclonedds>=0.10"]` (`setup.py:60`), while the only binding
any test here runs is 11.0.1. The SDK's `[dds]` extra declares
`cyclonedds>=11.0.1,<12`.

**Unverified, recorded so it is not lost.** `already_published` uses
`take_samples`, which keeps valid samples whose instance is not alive
(`transport.py:103-107`). A same-key predecessor exiting during the 2.5 s
settle could then be counted as a live conflict. The existing test
(`test_transport.py:488`) covers a different key only.

**One of this library's spec-queue entries is closed by the stamp.** Entry 5
("the registered entity topic is not the one in use") is resolved at v1.8,
which registers `model_entity` on `spatialdds/<scene>/world/entity/v1`, the
name this library already uses.

## 10. A dead sim stays ACTIVE: the latest transform is read as a new sighting

*Filed and fixed 2026-10-25, found by SpatialDDS-demo's acceptance run on
0.3.0 against its nav2 tier. Affects 0.3.0 and 0.4.0.*

With `pose_source: tf` (the default), the node takes the robot's pose with
`lookup_transform(map, base, Time())`, "the latest available", every tick,
and calls `adapter.observed()` with it. When the pose's publisher dies, the
buffer keeps answering with the last transform it heard, so the adapter is
told of a sighting every tick and its silence rule never fires. Measured:
40 s after `pkill plaza_sim_node.py` the record was still `ACTIVE`,
restamped every 5 s, at the sim's last pose.

With 0.4.0's pose floor it is worse than a stale record: the floor
republishes that pose as fresh, so a dead robot is an actively published
ghost. A consumer's staleness rule (this library's own awareness node
among them) cannot catch it, because the publisher is alive and chatty.
That is the failure the awareness design exists to prevent, inverted on
the publishing side.

**Fixed in 0.4.1:** a transform whose stamp is the one the buffer answered
with last time is not an observation (`TfFreshness`). The comparison is
stamp to stamp, never to a clock, so sim time and wall time are treated
alike. The adapter's own cadence-learned threshold then declares the robot
`UNOBSERVED`, and the floor stops, as it already does for a silent robot.

## 11. nav2's answer to a goal went nowhere

*Filed and fixed 2026-10-25, from the same run. Affects 0.3.0 and 0.4.0.*

The nav2 navigator sent each goal with `send_goal_async` and read nothing
back, and logged "nav2 accepted a goal" before nav2 had answered. A goal
nav2 rejected, or accepted and then aborted ("Goal failed": no plan into a
declared keep-out), reached the robot's log and no client. The kinematic
fallback has always reported its refusals on the robot's latched record
(`nav.status`), so a client saw a refusal from one navigator and silence
from the other.

**Fixed in 0.4.1:** the goal's response and its result are read, and a
rejected, aborted or cancelled goal becomes `adapter.refused()`, on the
record. Only the latest goal reports: a goal that ended because a newer one
superseded it is not news about the robot. The log line now says "sent".

Acceptance for both, against SpatialDDS-demo's nav2 tier with this tree in
the robot image: a goal into the declared pond came back on the robot's
record through the web gateway as "goal (10.00, -12.00) abandoned: nav2
could not reach it"; and the demo's `robot-captures.spec.ts`, "the sim
dies, and both the watching tab and a late one are told", which is marked
`test.fail` for finding 10, reported "Expected to fail, but passed".
`test/test_node_honesty.py` holds both decisions without ROS; a plant of
each fails it.

## 12. A preempting goal can be reported succeeded where it was never driven

*Filed 2026-10-26 from SpatialDDS-demo's OWM 0.2 batch, on the nav2 tier,
against 0.4.1. Not fixed: nav2's behaviour, reported faithfully, with a
check the node could add.*

Since 0.4.1 the node reports what nav2 decides about each goal. A goal sent
while nav2 is still finishing the last one is a preemption, and measured on
the demo's tier nav2 then reports the *new* goal succeeded at the old goal's
position, in the same tick:

```
[bt_navigator]: Received goal preemption request
[spatialdds_robot]: sent a goal at (17.00, -23.00) to nav2 (asked by playwright)
[controller_server]: Reached the goal!
[bt_navigator]: Goal succeeded
```

The robot was at (12.3, -2.1), 0.33 m from the goal being preempted and
21 m from the new one, and it never moved. SUCCEEDED owes no refusal, so
nothing reached the robot's record: a client that asked to go to (17, -23)
was told nothing, which reads as "on its way" for as long as it waits.
Seen three times while writing the demo's `claims-stack.spec.ts`, each
time as a return goal sent within seconds of arriving near the first, and
never driven, with nothing on the robot's record. nav2's own log, above,
was read for the third; for the first two, only the robot node's log, which
shows the goal sent and no outcome.

**The check the node could add:** on SUCCEEDED, compare the robot's pose
with the goal, and if it is further than the goal tolerance, report a
refusal ("nav2 reported success 21.0 m from the goal"), which is what the
requester needs to hear. `goal_outcome` would take the distance.
