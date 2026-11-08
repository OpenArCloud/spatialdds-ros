# The claims census: what a robot plans by, and what it says it sees

0.5.0's two additions, and the reasoning behind every rule in them. The
claims layer (`spatialdds_ros/claims.py`, drawn by `robot_node` as a nav2
filter mask) is the robot reading the world model; perception publishing
(`spatialdds_ros/perception.py`) is the robot adding to it. Both were built
for SpatialDDS-demo's claims batch (2026-11-08), whose brief set the doctrine
and whose nav2 tier is where the layer was measured; the demo's
`docs/ros-claims.md` carries that run.

The pattern for the consuming half already existed demo-side, in
`robot_tier/keepout_node.py`: the venue's declared bounds drawn into a nav2
keep-out filter. This is that node generalized from authored bounds to
claims, and its consumption rules are the crossing scene's
(`docs/crossing-claims.md` there): controllers that trust named asserters on
named bases and ignore everything else on the lane.

## 1. Doctrine

**Claims are advisory.** A drawn claim raises the planner's cost of passing
through a region, and the robot's own stack decides. So:

- the mask value is `claims_cost`, 1..99, and 100 is refused: a lethal region
  would make a stranger's statement a wall;
- the filter belongs on the global costmap only (the README's stanza), so a
  claim bends the plan and the local costmap the controller drives by is
  left alone;
- nothing in either module commands anything.

**Consumers filter by asserter and basis, per explicit configuration.**
`claims_trust` is required with claims on, and has no wildcard. A trust rule
names an asserter, the bases accepted from it, and optionally predicates.

**Observed claims age out when unrefreshed, and a crash is never a
retirement.** §2 below.

## 2. Freshness: the term the wire carries

The brief asked that an OBSERVED claim past its refresh interval leave the
costmap exactly as it leaves the bus's live view. **The wire has no refresh
interval on a claim**, and the asserters in this family say why there should
not be a republish clock either: an asserter holding an unchanged opinion
writes nothing ("the revision is of the opinion, not of the lineage",
SpatialDDS-demo `spatialdds_demo/claims.py`). A claim's age is therefore no
evidence about it.

What the wire does carry is the asserter's **liveliness**, and that is the
freshness term the bus's live view uses: SpatialDDS-demo's `ClaimStore`
(`web/src/claims.ts`) keeps a killed asserter's edges, marks them
`writerGone`, and draws them dashed, "nobody asserts it now". The layer takes
the same event and, because a costmap has no dashed line, its analogue:

| | live view | claims layer |
|---|---|---|
| asserter alive | drawn | drawn |
| asserter's writer gone, OBSERVED | kept, drawn dashed | **kept, not drawn** |
| asserter's writer gone, DECLARED | kept, drawn dashed | kept, drawn, reported unasserted |
| RETIRED terminal sample | removed, with its reason | removed, logged "retired" |
| dispose with no terminal sample | removed | removed, logged "withdrawn" |
| past its own `valid_until` | (not checked) | not drawn |

**Why OBSERVED and not DECLARED.** An observation nobody is still making is
not about the present, and a costmap is a decision about the present (the
awareness census's argument for peers, §2 there). A declaration is not
perception: a gate controller that crashed after declaring the crossing
closed has not reopened it, and §2.14 does not let its liveliness loss
withdraw the claim. DERIVED is treated as DECLARED for now, and the question
is filed for the 0.2 register (SpatialDDS-demo `docs/ros-claims.md` §6).

**Kept, not retired.** The claim's record stays in the layer, so the next
sample from its asserter brings it back with no rediscovery, and nothing on
the robot's log or in its picture calls it retired. Measured in
`test/test_claims.py::AnAsserterThatDies`: a hazard asserter SIGKILLed in its
own process, its OBSERVED claim aged out 10.0 s after the kill, which is
CycloneDDS's default participant lease, with no retirement in the notes. On
the demo's tier the same event took 10.0 s, and the lane showed
`NOT_ALIVE_NO_WRITERS`, never `NOT_ALIVE_DISPOSED`, never a RETIRED sample.

## 3. Geometry

- **The subject's footprint, not the object's.** A claim's subject is the
  thing; its object is what the claim relates it to. The crossing's
  controllers stop short of the zone a pedestrian is *in*, which is the
  object; a robot asked to plan round a hazard plans round the hazard, which
  is the subject. A consumer that wants the object drawn writes a trust rule
  for claims whose subject is the region.
- **The oriented box first.** 0.1's `Entity` has no oriented extent, and the
  crossing census ruled that moving things carry the enclosing `Aabb3` plus
  `demo.obb.length_m` / `demo.obb.width_m` meta for the true box. The layer
  reads the meta when present, centred on the subject's latest pose and
  turned by its yaw, because the enclosing box over-claims a long object
  turned across the axes: `test_the_oriented_box_wins_over_the_enclosing_aabb`'s
  2 x 0.4 m box at 45° has an enclosing box of (2.4/√2)² = 2.88 m², 3.6 times
  its own 0.8 m². Otherwise the `Aabb3`, read as `awareness`
  reads one: in the entity's frame, not about its pose.
- **No extent, no region.** A subject with neither is skipped with its
  reason. A disc of guessed radius is what awareness does for a peer, which
  is a statement about drawing a moving robot; for a claim, a guessed size
  would be the layer inventing the claim's content.
- **One georeference book.** A subject in another frame composes through
  both frames' georeferences, read by `georeferences.GeoreferenceBook`, which
  is awareness's book moved out of `awareness.py` so the two consumers on one
  robot cannot place one thing in two places.

## 4. Perception publishing

`ObstaclePublisher` keeps the rules this library and the demo's asserters
already keep, each with a test in `test/test_perception.py`:

- publisher-namespaced ids, `ent:<robot>:<track>`, and the demo's
  `claims.edge_id` for claims, byte for byte (a test computes one of the
  crossing's committed ids);
- OBSERVED, `FAST`, with the box as the enclosing `Aabb3` plus `demo.obb.*`,
  or no extent at all when the caller gives no size;
- silence within the track's own cadence (`Tempo`) is `UNOBSERVED` with a
  reason, and its claims retire `subject unobserved` with `valid_until` the
  last evidence that showed them;
- `lost()` and `withdraw()` write the terminal RETIRED sample with the reason,
  then dispose; the reason's prefix is one of the three causes;
- `claim()` on a track it is not seeing is refused: that is not an
  observation;
- an unchanged opinion writes nothing; a changed one revises in place, and
  keeps the lineage that established it.

## 5. The autodispose audit

Every writer the library opens is made by `transport.make_writer`, which
gives it `WriterDataLifecycle(autodispose=False)` (`qos.QosProfile.to_qos`,
since 0.4.0). The audit reads the policy back off each wire object, not off
the profile table (`test/test_perception.py::TheAutodisposeAudit`):

```
adapter.entity           Policy.WriterDataLifecycle(autodispose=False)
adapter.pose             Policy.WriterDataLifecycle(autodispose=False)
adapter.frame_transform  Policy.WriterDataLifecycle(autodispose=False)
perception.entity        Policy.WriterDataLifecycle(autodispose=False)
perception.pose          Policy.WriterDataLifecycle(autodispose=False)
perception.relationship  Policy.WriterDataLifecycle(autodispose=False)
```

and `AKilledPublisherDisposesNothing` SIGKILLs a publisher and reads both of
its latched lanes until `NOT_ALIVE_NO_WRITERS`: no `NOT_ALIVE_DISPOSED`, no
RETIRED sample. The claims layer and the ROS node add no DDS writer; the node
publishes ROS topics only.

## 6. What nav2 needed that the census did not predict

**A mask is painted only on its costmap's grid** (`FINDINGS.md`, finding 14).
The first live run's mask was received by the filter and the global
costmap's cost at the hazard stayed 0; values 50, 99 and 100 all left it at
0. nav2 Humble's keep-out filter repaints a window from the costmap's origin
as wide as the mask, so a mask with an origin of its own repaints somewhere
else. Drawn on the costmap's grid, read off `/global_costmap/costmap`, 50
became cost 127, 99 became 251, and 0 cleared it. The node now waits for that
grid before publishing anything.
