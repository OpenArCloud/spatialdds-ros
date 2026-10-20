# Shown failable

Acceptance item 4, kept up as the library grows. Four claims this library
makes are asserted by tests, and an assertion is worth nothing until it has
been seen to fail. Each was broken deliberately and the failures recorded
here, with the exact output.

The first two are from the extraction; §3 and §4 are 0.2.1's, and both came
from a real consumer rather than from reading the code.

Why this is written down rather than just done: a green suite is evidence that
the tests ran, not that they *can* distinguish. These two in particular guard
behaviours whose failure mode is silence — a georeference that should not
exist, and a refusal nobody receives — so a test that could never go red would
look identical to one doing its job.

---

## 1. No anchor configured means no `FrameTransform` published

**The claim.** With no anchor, the adapter publishes nothing on the frames
lane. Not a transform at the frame origin, not an empty one — nothing. A
consumer sees no georeference and should say so rather than assume a position.

**What was injected.** The alternative the ruling rejected: default the anchor
to the frame origin so a transform is always published.

```python
transform = georeference(self.config.frame_id,
                         self.config.anchor or Anchor(0.0, 0.0, 0.0))
```

**What failed.** Two tests, at two levels — the unit assertion on the
adapter's own report, and the acceptance run reading the bus as a consumer:

```
test/test_adapter.py::TheAnchorsSilence
  AssertionError: 'georeference: map/bare-scene -> earth-fixed' is not None
                : with no anchor there is nothing to report

test/test_acceptance_bare_scene.py::test_5_no_anchor_means_no_georeference_on_the_bus
  AssertionError: Lists differ: ['map/bare-scene'] != []
```

The second is the one that matters. `['map/bare-scene'] != []` is a consumer
receiving a georeference that should not exist — and under the injected
default, it would place every unconfigured robot in the Gulf of Guinea with a
pose that looks entirely plausible.

The anchored case is a control in both tests, so neither can pass by never
publishing a transform at all.

---

## 2. A refused `goto` reaches the asker

**The claim.** When a goal is accepted and then given up on, the reason
reaches whoever asked — on the robot's latched record, and it *stays* there
until a new goal is accepted.

**What was injected.** The obvious implementation, and the one this library
had first: publish the refusal once and drop it on the next pose.

```python
if self._nav_status:
    properties.append(KV(key=NAV_STATUS_KEY, value=self._nav_status))
    self._nav_status = None   # injected: event, not state
```

**What failed.** Three tests across two levels:

```
test/test_adapter.py::ARefusalReachesWhoeverAsked
  test_a_refusal_persists_until_a_new_goal_is_accepted            FAILED
  test_a_declined_command_leaves_the_previous_outcome_standing    FAILED

test/test_acceptance_mcap_gateway.py
  test_5_the_refusal_survived_the_round_trip                      FAILED
```

That third failure is how the design changed. The refusal was *published*
correctly and still could not be found through the gateway, because a
collection GET serves the current latched record and the refusal had been
superseded by the very next pose. A client polling a moment later saw a robot
sitting still with nothing to explain it — the exact silence the whole path
exists to end.

So the lesson is not only that the test can fail. It is that the test found a
real gap: "what became of the goal I sent" is **state**, and a latched record
is where state belongs. Cleared on *acceptance* rather than on arrival, so a
declined command leaves the previous outcome standing — the requester still
needs to see it.

---

## 3. A read sees the lifecycle, and does not mistake it for a payload

**The claim.** `transport.take_samples` returns payloads only, and a dispose
or writer-gone notification is dropped rather than handed to a caller as a
record.

**What was injected.** What this library actually shipped until 0.2.1 — a
filter for `None` and nothing else:

```python
return [s for s in (reader.take(N=n) or []) if s is not None]
```

**What failed.** Both of `APeerThatDies`, and the value is *where*:

```
test/test_transport.py::APeerThatDies::test_the_lifecycle_event_is_real_and_not_a_payload
  AttributeError: 'InvalidSample' object has no attribute 'subject_id'
  spatialdds_ros/adapter.py:218

test/test_transport.py::APeerThatDies::test_a_predecessors_tombstone_does_not_stop_a_robot_starting
  AttributeError: 'InvalidSample' object has no attribute 'entity_id'
  spatialdds_ros/adapter.py:333
```

Line 218 is the crash SpatialDDS-demo hit a minute into its adoption: any
requester whose participant goes away kills the robot reading the command
lane. Line 333 is `already_published`, which nothing had ever reached — an
adapter crashing *at startup* because an unrelated robot happened to exit
while it was checking whether its own key was taken.

**Why 153 tests never produced one.** Every writer in the suite outlived
every reader. The defect needs a peer with a lifetime, which is the first
thing a deployment supplies and the last thing a unit test does.

Two things had to be right for the new test to be able to fail at all, and
the first version of it got both wrong:

* The peer needs its **own participant**, deleted outright. A writer dropped
  inside the reader's participant is a weaker event.
* The payload must be **taken before the peer dies**. A dispose with an unread
  sample still queued rides on that sample's own `sample_info` and generates
  no separate notification — so leaving it unread suppresses exactly what is
  being injected. Taking it first is also what a running robot does, which is
  why the deployment met this and the suite did not.

The guard against that is in the test itself: it asserts a lifecycle event was
delivered before asserting anything about how it was handled, and says so —
*"the injection produced no lifecycle event, so this test proves nothing"*.

---

## 4. The library's topics ask for the least, so a second library can share them

**The claim.** `transport` creates `Topic` objects with default QoS, and
§3.3.3 rides on the endpoints — where matching is actually decided.

**What was injected.** Again what shipped until 0.2.1: the profile on the
topic as well.

```python
return Topic(participant, topic_name, datatype, qos=profile.to_qos())
```

**What failed.** All three of the cross-convention tests, in both
construction orders:

```
test/test_transport.py::TwoConventionsOnOneTopic
  test_a_held_foreign_topic_then_this_library              FAILED
  test_this_library_first_then_a_held_foreign_topic        FAILED
  test_a_library_writer_exchanges_with_a_foreign_reader    FAILED
  cyclonedds.core.DDSException: [DDS_RETCODE_INCONSISTENT_POLICY]
    Occurred upon initialisation of a cyclonedds.topic.Topic
```

**And the measurement that corrected the filing.** The finding written during
the adoption named the construction *order* as the cause. It is not. A
`Topic` name is unique within a participant and its QoS must agree among
**live** topic objects for that name — in either direction:

| first (held) | second | |
|---|---|---|
| profile | default | `INCONSISTENT_POLICY` |
| default | profile | `INCONSISTENT_POLICY` |
| profile | profile | accepted |
| default | default | accepted |

Order only decides which call is the one refused, and *retention* decides
which that is in practice: SpatialDDS-demo caches its topic objects
deliberately, so its convention persisted and the library's was the call that
failed. The first probe written for this missed all of it by discarding its
first topic, which let the name go free and made every combination look
accepted — so the tests now hold every topic in a named local and say why.

The fix must also not be "stopped applying QoS", so two controls sit beside
it: `test_the_profile_still_reaches_both_endpoints`, and a library writer
exchanging real samples with a reader built the other convention's way.
Deadline alone is request/offered, and a mismatch there is silence.

---

## Reproducing

§1 and §2 are three-line edits to `spatialdds_ros/adapter.py`, described
above exactly as applied. §3 and §4 were run by restoring the pre-0.2.1
functions over `spatialdds_ros.transport` from a `conftest.py`, which is the
faithful way to break them: the shipped code is quoted above and goes back
verbatim. Restore and the suite returns to green:

```sh
python3 -m pytest test/ -q
```

Two further gates in this repository carry their own injection tests rather
than a note here, because they check *instruments* and an instrument that
cannot fail is the thing they are about:
`test/test_types_resolve_at_definitions.py` pins a known alias against a known
definition, and `test/test_generated_types.py` asserts the tree it walks is
not empty.
