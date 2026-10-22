# `spatialdds_ros` — awareness census

What the demo's keep-out lane and robot tier hard-code or assume about
consuming bus state into a planner, each with a verdict, and the design for a
consumer that puts the scene's *other* entities into this robot's costmap.

Same three verdicts as `extraction-census.md`: **parameter** (an operator must
set it, because the library cannot know it), **convention** (one right answer,
the library fills it), **out** (the demo's, not ours).

**Read for this census** (SpatialDDS-demo, read-only):
`robot_tier/keepout_node.py`, `spatialdds_demo/keepout.py`,
`robot_tier/nav2_params.yaml`, `robot_tier/_bringup.sh`,
`robot_tier/plaza_sim_node.py`, `spatialdds_demo/typed_transport.py`,
`model_service.py`, `tests/test_keepout.py`, `web/tests/robot-captures.spec.ts`.
And from the spec at the pinned commit `9c3e1f2`: §2.14, the Announce
lifecycle, the `model_entity` registration, and `spatial.owm/0.1`.

**Measured for this census:** nav2's `ObstacleLayer` against four virtual
sensors (§1). The probe and its configs are in
[`probes/virtual-sensor-clearing/`](probes/virtual-sensor-clearing/) and run in
SpatialDDS-demo's robot-tier image (ROS Humble, `nav2_costmap_2d` 1.1.20,
image `sha256:c6ed664e9a4c`):
`docker run --rm -v $PWD/docs/probes/virtual-sensor-clearing:/probe:ro spatialdds-robot-tier bash /probe/run.sh <variant>`.

---

## 1. Output form

What the demo does: **a costmap keep-out filter**. `keepout_node` rasterises
every DECLARED extent into one `OccupancyGrid` on `/keepout_filter_mask`, with
a `CostmapFilterInfo` and an all-free `/map` canvas, all latched
(`keepout_node.py:71-73,102-106`). Both costmaps run
`static_layer + inflation_layer` and `filters: [keepout_filter]`, and **no
obstacle layer**, deliberately (`nav2_params.yaml:3-6,80,89-93`).

That is the right form for what it carries — law, which changes rarely — and
the wrong one for peers. Weighed for this job:

| form | carries | cannot express | verdict |
|---|---|---|---|
| keep-out filter mask | arbitrary regions, any distance, latched | per-entity identity (one union grid); motion at any useful rate (the whole grid plus `/map` republished per change, `keepout_node.py:194-196`; a repaint of "about three seconds" by the capture spec's own account, `robot-captures.spec.ts:219-222`); anything but lethal-forever; coexistence with the demo's own mask on the same filter topic | **out** for peers — stays the DECLARED lane's job |
| `LaserScan`, one source, marking + clearing, `inf_is_valid` | the nearest peer on each bearing, cleared by the next scan | **a peer behind a nearer peer** (one range per bearing), anything beyond `range_max`, height | measured to work; rejected for occlusion |
| `PointCloud2`, one source, marking + clearing | every peer | **its own expiry**: an empty cloud raytraces nothing, so the last marks stay | **measured ghost** — rejected |
| `PointCloud2` marking source **+** a clearing-only ring at the awareness range | every peer, each with its own footprint, expired on the next cycle | anything beyond the ring; height | **recommended** |
| `MarkerArray` | labels, ids, states, skip reasons | anything a planner reads | **convention**, alongside, for operators |

**Measured** (cost read as the maximum within 0.4 m of peer A at (3, 0);
phases of 3 s each):

```
RESULT scan:       cost at A present=100 absent=0   present-again=100
RESULT cloud:      cost at A present=100 absent=100 present-again=100
RESULT cloud_ring: cost at A present=100 absent=0   present-again=100 ring-cell=0
RESULT scan_lidar: cost at A present=100 absent=100 present-again=100
```

The probe can fail both ways — `scan` and `cloud` disagree on the same
absence — so a clean `absent=0` is the layer's behaviour, not the probe's
blindness. Its first run read 0 in every phase because it sampled A's centre
while a laser marks A's near surface; that was the probe, and was fixed before
any row above counted.

**`scan_lidar` corrects a claim both repos carry.** `keepout_node`'s docstring
— which `extraction-census.md` §9 said "should survive the extraction
verbatim" — argues that the obstacle layer raytraces away a virtual obstacle a
real sensor sees through. Measured with a real-lidar source in the *same*
layer, every ray at max range straight through A: A stayed at 100. Each cycle
clears and then marks, so a virtual source that keeps publishing re-marks
every cycle. The claim holds only for a mark nobody refreshes — which is
exactly the `cloud` ghost. It remains a good reason for the demo's choice (a
keep-out published once and latched *would* be eroded) and is no reason
against a live virtual sensor. Filed for both queues (§7).

**What the recommended form costs the operator:** a dedicated obstacle layer
in their nav2 params with two observation sources, and that layer's
`raytrace_max_range` at or beyond the ring. The library cannot write nav2's
parameters, so the README carries the stanza, and §5 is where a mismatch is
refused rather than discovered. The cloud is stamped in the robot's base
frame, because the layer raytraces from the cloud's frame origin: a cloud in
`map` would clear from the map origin.

## 2. What becomes an obstacle

What the demo does (`keepout.py:115-135`): **DECLARED, ACTIVE, `has_extent`,
not `ent:robot:tb3`.** No kind filter; scene scoping by topic name alone;
extent from the `Aabb3`'s x/y only, the pose ignored. The robot's own entity
is excluded twice over — by id, and because it is OBSERVED with no extent.

The division of labour carries over and is the census's first call:
**DECLARED regions stay the keep-out filter's; awareness is for things that
move.**

| what | demo | proposed | verdict |
|---|---|---|---|
| layer | not read | `FAST` only | **convention** — FAST is the spec's word for things that move |
| state | `ACTIVE` | `ACTIVE` only | **convention** |
| basis | `DECLARED` only | any; DECLARED regions are already in the mask | **convention** |
| `has_pose` | not read | required | **convention** — a pose-less entity has nowhere to be |
| kind | not read | optional `type_uris` allowlist, empty = every FAST entity | **parameter** |
| scene | topic name | topic name | **convention**, unchanged |
| self | `ROBOT_ID` constant | this node's own `entity_id`, from `Config` | **convention**, not configurable |
| extent | `Aabb3` x/y | `Aabb3` x/y when `has_extent`; otherwise a disc of `peer_radius_m` | **parameter** (the radius) |
| covariance | — | none to read | see below |

**Self-exclusion is load-bearing, not redundant.** Measured: a reader opened
on the same participant as this library's entity writer receives that
writer's samples (`['ent:robot:b']`; no `ignore_local` anywhere in
`transport.py`). An awareness mode sharing the robot's participant *will* hear
itself, and without the filter the robot would avoid its own position.

**And it must be asserted at the consumer's output, not in the costmap.**
nav2's `footprint_clearing_enabled` clears the cells under the robot, which
would hide a self-obstacle at exactly the place it would land. A costmap-level
test would pass with the filter removed. Acceptance §4.3 asserts on the
emitted cloud and marker set.

**Every robot this library publishes is a point, so every one gets the default
radius.** The adapter publishes `has_extent=False` (`adapter.py:126`). Its comment (`:123-125`)
says an extent "would be read as keep-out by anything applying the
DECLARED rule", but the robot's basis is OBSERVED, and the demo's rule
(`keepout.py:132`) excludes it on basis before extent is read. The conclusion
may still be right — no consumer should read a footprint as a region — but its
stated reason is not. That is a library finding for this batch's build
(§7), not a reason to change what is published.

**Covariance contributes nothing because there is none.** `Entity` carries a
`PoseSE3` and `ModelPose` carries a `PoseSE3`: no covariance on either
(`owm.idl:71-109`). An OBSERVED pose with no uncertainty is a claim of
precision the publisher never made. So there is nothing to inflate by; filed
for the spec queue (§7).

## 3. Staleness

**The brief's "2×ttl rule" does not apply to entities.** At `9c3e1f2` it reads
"Consumers SHOULD discard **Announce** samples where `now - stamp > 2 *
ttl_sec`" (1.8-full:1326), and `Entity` has no ttl. Applied by analogy, it
would use the announce's ttl (`DEFAULT_TTL_SEC = 300` in `announce.py`): a
ten-minute ghost.

**And §2.14 constrains what "gone" may mean:** "Consumers MUST NOT treat a
liveliness change as a deliberate removal, and MUST NOT purge a durable keyed
instance on liveliness loss alone" (1.8-full:473). That is about the *record*.
A costmap is not the record; it is a planning decision about the present. So
the design keeps every record — §2.14 honoured, a returning peer resumes with
no rediscovery — and *projects* only what has current evidence. "Not an
obstacle now" is not "removed".

What the demo does: **never evicts.** Disposed and no-writer samples arrive
with `data=None` and are skipped (`keepout_node.py:169-172`), so a dispose
without a tombstone leaves the last ACTIVE copy in force. Separately, when the
*last* contributor retires the mask is not cleared at all
(`keepout_node.py:176-177`, "no mask is not an empty mask"). Both filed for
the demo's queue (§7).

Proposed:

| event | projected? | why |
|---|---|---|
| `ModelPose` or ACTIVE `Entity` received | yes, at that pose | evidence; resets the window |
| no evidence for longer than the window | **no** | quiet; record kept |
| `UNOBSERVED` / `RETIRED` / `SUPERSEDED` record | **no**, at once | the publisher has said so |
| disposed (`NOT_ALIVE_DISPOSED`) | **no**, at once | removal, §2.14 |
| writer gone (`NOT_ALIVE_NO_WRITERS`) | **no**, at once; record kept | a peer that died is not where it was, and is not removed either |

**The window is the library's `Tempo` threshold, learned per key from the
peer's own pose cadence**: `max(1.0 s, 3 × smoothed gap)` (`tempo.py`). It is
the same derivation the adapter uses to declare its *own* robot UNOBSERVED, so
a peer from this library and a consumer of it agree on what quiet means. It is
measured on **receive time**, never the sample's `stamp`: the window has to
survive clock skew between two machines, and a stamp from a skewed clock
would make a peer look permanently fresh or permanently stale.

**Died versus quiet-but-latched** is then two different paths, which
acceptance §4.2 shows separately. A killed publisher arrives as
`NOT_ALIVE_NO_WRITERS` and drops out on that sample. A live publisher that
stops sending poses drops out when its window passes — and, if it is this
library, also republishes UNOBSERVED on its own threshold, whichever comes
first.

**The cost, stated rather than discovered:** a publisher that sends poses
only when something *moves* will vanish from the costmap when its entity stops,
while it is still physically there. **This library's own robot is one, in
one of its three pose modes.** With `pose_source=tf` (the default) the node
reads TF every tick and publishes whether the robot moves or not
(`robot_node.py` `_tick` → `_pose_from_tf`); `odom` arrives at the odometry
rate. With `amcl` it publishes only when `/amcl_pose` does (`_on_amcl`), and
AMCL publishes on filter updates, which by default need motion — so a stopped
AMCL robot would drop out of its peers' costmaps. Read, not measured; the
build either measures it or makes the adapter repeat its last pose on the
tick, and filed for this library in §7. A generic FAST publisher may behave
the same way. The window cannot tell stopped from gone,
because nothing on the wire says how often a FAST entity promises to speak:
`TopicMeta.target_rate_hz` is 0 on this library's own pose lane
(`announce.py`). This is the spec gap underneath the whole item, filed in §7.
The brief's ruling — a ghost is worse than none — is what picks this side of
the trade.

## 4. Frame composition

What the demo does: **nothing, by construction.** `MAP_FRAME = "map"`, "the
venue frame is the map frame here … nothing has to be transformed"
(`keepout_node.py:81-84`); no FrameRef member is read anywhere in the lane;
the ROS side's `map → odom` is a static identity (`plaza_sim_node.py:54-61`).
So the demo never meets the question.

The library has half of the machinery: `frames.py` writes a georeference
(ENU→ECEF) and inverts its translation for tests. It has no composition. That
goes in `frames.py`, beside the projection, as the one place:

| case | composition | anchor needed |
|---|---|---|
| peer's `frame_ref.fqn` == this robot's `frame_id` (uuid agreeing) | identity | **none** — the bare-scene case |
| different fqn | `T(earth→mine)⁻¹ · T(earth→theirs)` | **both**: ours from `Config.anchor`, theirs from the frames lane |
| different fqn, either missing | — | skipped: reason logged once per key, counter incremented, **never placed** |
| fqn agrees, uuid does not | — | skipped: two frames sharing a name (`frames.frame_uuid`) |
| `has_scale` with `meters_per_unit ≠ 1` | — | skipped in v1, with the reason |

This robot's ROS `map` frame *is* its `frame_id` — the adapter already treats
them as one (it reads TF `map → base_link` and publishes the result as a pose in `frame_id`) — so
the composed pose lands in `map`. It then goes to the base frame through TF
for the cloud's header (§1).

**What the two-domain proof needs from this:** if both robots run with the
same `frame_id`, they are claiming one shared map, and identity is correct
only if their ROS `map` frames really coincide. The bare-scene acceptance
constructs them to. A deployment where two robots use one `frame_id` over two
independent SLAM maps would be placing peers wrongly with every number looking
plausible. That is an operator's false claim, not something the consumer can
detect, and the README says so.

## 5. Parameter surface — for ruling

**Shape: a mode of the existing node, not a second node.** Same argument as
`extraction-census.md` §10a, and stronger here. Self-exclusion needs the
robot's own `entity_id`; composition needs its `frame_id` and anchor; the
cloud needs its TF; and the two-domain proof needs robot B to publish itself
*and* consume. All of that is already in `Config`, and in the same process
the exclusion cannot be misconfigured, because there is nothing to type. A
second node would need the `entity_id` passed twice, and the failure if the
two disagree is a robot avoiding itself. A standalone consumer with no robot
(a camera, a dashboard) is not a v1 case.

| parameter | required | silence means |
|---|---|---|
| `awareness` | no | **off**. No reader opened, nothing published — the node exactly as today. |
| `awareness_range_m` | **yes, when `awareness` is on** | **refuse to start.** It is the clearing ring's radius, and it must not exceed the operator's nav2 `raytrace_max_range`. The library cannot read nav2's parameters, and a mismatch produces ghosts beyond the overlap with no error anywhere. A default would be a guess about someone else's configuration. Peers beyond it are not emitted, which is what keeps every mark clearable. |
| `peer_radius_m` | no | **0.3 m**, documented: the disc for an entity with no extent. A TurtleBot 3 Waffle is 0.22 m (the demo's `robot_radius`), and nav2's inflation adds to it. This is plumbing about *how big to draw* a point — the one claim it makes, that a point peer is no larger than 0.3 m, is printed at startup. |
| `awareness_types` | no | empty: every FAST entity. A `type_uris` allowlist otherwise. |
| ROS topics | — | **convention**: `~/peers/cloud`, `~/peers/clearing`, `~/peers/markers`, remappable the ROS way rather than by parameter |
| window | — | **convention**: `Tempo`, §3. Not configurable, for the reason `tempo.py` gives: a fixed threshold shorter than a peer's cadence makes every gap look like a departure |
| `base_frame`, `map_frame`, `rate_hz`, `scene`, `domain`, `entity_id`, `frame_id`, anchor | — | shared with the robot, unchanged |

**Degraded modes, stated as configuration** (logged once, counted, asserted
in acceptance §4.4):

- **No anchor here:** peers in this robot's frame are projected; peers in any
  other frame are skipped with "no anchor on this robot to compose through".
- **No FrameTransform from the peer's frame:** that peer is skipped with "no
  georeference for `<fqn>`", and it starts projecting the moment one arrives.
- **`pose_source=odom`** with awareness on: allowed, with the same warning the
  robot already prints — the peers are placed in a frame that drifts.

**The one row this census most wants ruled is `awareness_range_m`
refusing to start**, against the alternative of a default (say 8 m) plus a
README stanza. Refusal is consistent with the library's rule — the value is
a claim about a configuration the library cannot see — but it is the first
parameter whose correct value lives in somebody else's YAML.

## 6. The build, after the ruling

One consumer within `robot_node`, following the established shape:

- `awareness.py`, no ROS: entity and pose readers via `transport.make_reader`
  with the lanes' own profiles, `take_with_state` for every read, the
  selection and projection rules above as pure functions, and `Tempo` per key.
  Emits a list of (entity_id, x, y, radius) plus skip reasons, so all of §2–§4
  is testable with no ROS installation.
- `frames.py` gains the composition (§4), round-trip tested the way the
  georeference is.
- `robot_node.py` turns that list into the two clouds and the markers.
- Vendored types untouched; `test_no_ros_needed` still green; one participant
  per robot.

## 7. Filed

**For the spec queue** (to `spec-queue.md`, with these origins):

1. **A FAST entity has no freshness contract.** No ttl on `Entity`; the 2×ttl
   rule is Announce-only (1.8-full:1326); `target_rate_hz` 0 means
   unspecified. So no consumer can tell a stopped entity from a gone one
   without a cadence it learned or guessed (§3).
2. **No pose uncertainty on `Entity` or `ModelPose`** (`owm.idl:71-109`): an
   OBSERVED pose is published with an implied precision nobody claimed (§2).
3. **The registered entity topic is not the one in use.** 1.8-full:1484
   registers `model_entity` as `spatialdds/<scene>/model/entity/v0`; this
   library and SpatialDDS-demo both use `spatialdds/<scene>/world/entity/v1`
   (`topics.py`). A publisher following the registration and a consumer
   following the implementations would never meet, with an empty bus as the
   only symptom. Which one moves is a ruling, not this batch's call.
4. **§2.14 says what a consumer may not purge, not what it may stop acting
   on.** A non-normative note distinguishing the record from a projection of
   it (§3) would stop the next implementer reading "MUST NOT purge" as "must
   keep planning around".

**For the demo's queue** (SpatialDDS-demo, read-only here):

1. **The keep-out mask is never cleared when the last contributor retires**
   (`keepout_node.py:176-177`). nav2 keeps planning around water that is no
   longer declared.
2. **A dispose without a tombstone, and a writer that dies, leave the last
   ACTIVE copy in force** (`keepout_node.py:169-172`), because samples with no
   payload are skipped rather than read for their state.
3. **`robot-captures.spec.ts` looks for the process the tier no longer runs.**
   `tierRunning()` greps for `spatialdds_demo.robot_bridge`
   (`robot-captures.spec.ts:80`), while `_bringup.sh:48` starts
   `spatialdds_ros.robot_node`. If nothing else matches, the spec skips
   whenever the tier is up — a self-re-arming skip. Read, not run.
4. **`keepout_node`'s obstacle-layer argument is broader than what was
   measured** (§1, `scan_lidar`). It is right for a latched one-shot mark and
   wrong for a live virtual sensor.
5. Minor: `sendGoto` uses z = −1.15 (`robot-captures.spec.ts:99`) against
   `GROUND_Z = −0.84` (`plaza.py:31`); the header of
   `tools/capture_robot_session.sh` still gives the start as (17.0, −21.5)
   (`:17`), against `plaza.py:69`'s (17, −23).

**For this library:**

1. **`extraction-census.md` §9's verbatim-survival note is withdrawn** for
   the obstacle-layer argument, on §1's measurement.
2. **`adapter.py:123-125`'s reason for `has_extent=False` names a rule that
   would not read it** (§2). The comment gets corrected in the build; what is
   published does not change.
3. **With `pose_source=amcl`, a stopped robot stops publishing poses** (§3),
   and to any consumer applying a freshness window it reads as gone. Read
   from the code and AMCL's documented update rule, not measured.
