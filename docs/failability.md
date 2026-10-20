# Shown failable

Acceptance item 4. Two claims this library makes are asserted by tests, and an
assertion is worth nothing until it has been seen to fail. Both were broken
deliberately and the failures recorded here, with the exact output.

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

## Reproducing

Both injections are three-line edits to `spatialdds_ros/adapter.py` and both
are described above exactly as applied. Restore the file afterwards and the
suite returns to green:

```sh
python3 -m pytest test/ -q
# 130 passed, 32 subtests passed
```

Two further gates in this repository carry their own injection tests rather
than a note here, because they check *instruments* and an instrument that
cannot fail is the thing they are about:
`test/test_types_resolve_at_definitions.py` pins a known alias against a known
definition, and `test/test_generated_types.py` asserts the tree it walks is
not empty.
