# spatialdds_ros

Make a ROS 2 robot a SpatialDDS participant: its pose published as a
`spatial.owm` entity on scene-scoped topics, `goto` commands consumed from
`ModelCommand` and dispatched to nav2, its presence announced, and — when
asked — the scene's other moving entities put into its own costmap. All on the
same DDS bus ROS already runs on: one pip install makes a robot seen,
steerable and aware, including of robots in other ROS domains it shares no
middleware with.

**A type adapter on a shared bus, not a protocol gateway.** Nothing is
relayed. ROS and SpatialDDS meet on the wire because both are DDS, so this is
one process holding both stacks, publishing the model as model and reading ROS
as ROS.

## Installing, and what you are not installing

```sh
pip install spatialdds-ros
```

That gets `cyclonedds` and nothing else. **No ROS.** The library puts a robot
on a SpatialDDS bus, reads commands off it and drives a navigator, and none of
that needs a ROS install — so `rclpy` is not a dependency, and the absence is
deliberate rather than an omission waiting to be tidied up.

The split runs through the whole package:

| dependency | declared in | what it is for |
|---|---|---|
| `cyclonedds` | `install_requires` | the bus. Everything here needs it |
| `rclpy`, `tf2_ros`, `nav2_msgs` | `package.xml` | the ROS node alone, resolved by `rosdep`, never by pip |

Only `spatialdds_ros.robot_node` requires a ROS 2 environment. It imports
`rclpy` inside its entry point rather than at module scope, which is what
lets the other thirteen modules — and the entire test suite — run on a machine
that has never had ROS installed.

**That claim is a gate, not a promise.**
[`test/test_no_ros_needed.py`](https://github.com/OpenArCloud/spatialdds-ros/blob/main/test/test_no_ros_needed.py)
walks every module reachable from `import spatialdds_ros` and fails on a
module-scope `import rclpy` anywhere in it. By source inspection, so it
catches the violation on a ROS workstation where the import would have
succeeded — a test that only checked "the import worked" would pass there and
fail in CI. If the split ever stops being real, that is where it is reported.

## A robot on the bus, with no robot

No ROS, no nav2, no hardware:

```sh
pip install -e .
python3 -m spatialdds_ros.standalone \
    --scene bare-scene \
    --entity-id ent:robot:r1 \
    --frame-id map/bare-scene
```

```
robot: scene bare-scene, entity ent:robot:r1 in frame map/bare-scene, DDS domain 0
robot: no anchor configured, so this adapter publishes no FrameTransform: its
       entity and poses are on the bus and complete, and it makes no claim
       about where that frame sits on the earth. Another participant may own
       that claim — a venue publishing its own georeference is the ordinary
       case, and then this is the correct configuration. If nobody publishes
       one, a consumer cannot place this robot on a globe and should say so
       rather than assume a position
robot: announce: svc:robot:r1 as OTHER, 3 topics, ttl 300s
```

It is now on three lanes, and discoverable on a fourth:

| lane | topic | QoS |
|---|---|---|
| what exists | `spatialdds/bare-scene/world/entity/v1` | `MODEL_LATCHED` |
| where it is | `spatialdds/bare-scene/world/model_pose/v1` | `MODEL_FAST` |
| what to do | `spatialdds/bare-scene/world/model_command/v1` | `MODEL_COMMAND` |
| who it is | `spatialdds/discovery/announce/v1` | `MODEL_LATCHED` |

Add `--bounds -10,-10,10,10` and it will refuse a goal that leaves the box, so
the refusal path can be exercised without hardware.

## On a real robot

```sh
ros2 run spatialdds_ros robot_node --ros-args \
    -p scene:=bare-scene \
    -p entity_id:=ent:robot:r1 \
    -p frame_id:=map/bare-scene
```

The pose input defaults to the TF chain `map -> base_link`. That default
matters: it is what nav2 itself consumes, it works whatever the localizer is,
and it assumes no topic name. `pose_source:=amcl` and `pose_source:=odom` are
available, and `odom` logs its own consequence — odom drifts against map, so
publishing it as a georeferenced pose is correct only where `map -> odom` is
static and identity, which is a simulator and not a localizing robot.

`ROS 2 Humble` and `Jazzy` are the named targets. **The ROS buildfarm is
deferred** — there is no released binary package; install from source or with
pip.

## A robot that sees the scene

```sh
ros2 run spatialdds_ros robot_node --ros-args \
    -p scene:=bare-scene \
    -p entity_id:=ent:robot:b \
    -p frame_id:=map/bare-scene \
    -p awareness:=true \
    -p awareness_range_m:=8.0
```

The node then reads the scene's other moving entities off the bus and
publishes them where nav2 can plan around them: `~/peers/cloud` marks them,
`~/peers/clearing` is a ring at `awareness_range_m` that clears whatever has
gone, and `~/peers/markers` shows an operator what is being avoided. nav2
needs one obstacle layer reading both clouds:

```yaml
peers_layer:
  plugin: "nav2_costmap_2d::ObstacleLayer"
  enabled: true
  observation_sources: peers clearing
  peers:
    topic: /robot_b/peers/cloud
    data_type: PointCloud2
    marking: true
    clearing: false
    obstacle_max_range: 8.0
    raytrace_max_range: 8.5
    min_obstacle_height: 0.0
    max_obstacle_height: 2.0
  clearing:
    topic: /robot_b/peers/clearing
    data_type: PointCloud2
    marking: false
    clearing: true
    obstacle_max_range: 8.0
    raytrace_max_range: 8.5
    min_obstacle_height: 0.0
    max_obstacle_height: 2.0
```

Add `peers_layer` to the costmap's `plugins`, before the inflation layer, in
every costmap that should avoid peers — the global one, for the planner to
route around them. The topics carry the node's name (`robot_b` here).

**Set `awareness_range_m` to your costmap's obstacle range or less** — both
`obstacle_max_range` and `raytrace_max_range` above. The node refuses to start
with awareness on and no range, because the right value lives in your nav2
YAML, which it cannot read, and a guess beyond your raytrace range draws marks
nothing will ever clear.

**Why two clouds.** A marking cloud that empties raytraces nothing, so the
last place a peer was seen stays lethal for good. Measured, with the ring
beside it and without, in
[`docs/awareness-census.md`](https://github.com/OpenArCloud/spatialdds-ros/blob/main/docs/awareness-census.md)
§1.

### What is drawn, and when it stops

- **Moving things only:** `FAST`, `ACTIVE`, with a pose, within range, and of
  an `awareness_types` URI if you set any. A declared region is not a peer;
  the keep-out filter is the mechanism for those.
- **Never this robot.** Its own record does reach its own reader, so this is
  a filter that does work, on its own `entity_id`.
- **Its own size, or `peer_radius_m`** (0.3 m) for a peer that publishes no
  extent — which includes every robot this library publishes.
- **Dropped when it goes quiet, and remembered.** A peer is drawn while its
  poses keep arriving, within a window learned from its own cadence — at
  least one second, three times its interval, measured on this robot's clock.
  The record is kept, so the next pose brings it back with no rediscovery. An
  `UNOBSERVED`, retired or disposed record, or a writer that has gone, drops
  it at once.
- **Never at a guessed position.** A peer in another frame is placed through
  both frames' georeferences; with either one missing it is skipped, and the
  log says which.

### Two robots, two ROS domains, one bus

The walkthrough is the acceptance test. Robot A runs in `ROS_DOMAIN_ID` 10,
localized by an AMCL that converges at (3, 0) and then says nothing; robot B
runs in 20 at the origin with nav2's planner; the SpatialDDS bus is DDS domain
30. Nothing else connects them.

```sh
docker build -t spatialdds-ros-nav2 -f test/ros/Dockerfile .
python3 -m pytest test/test_acceptance_two_domains.py -q -s
```

```
  A absent: path 6.00 m, max |y| 0.000; cloud empty
  A present: cloud 109 points near (3, 0), cost 100; path 6.30 m, max |y| 0.84
  A still for 20 s: drawn in all 31 samples
  quiet but latched: not drawing ent:robot:a: UNOBSERVED: no pose from svc:robot:a for 1.6s
  killed: cost at A back to 0 within 2.8s (probe overhead included)
  gateway served and validated 2 entities: ['ent:robot:a', 'ent:robot:b']
```

B's plan to (6, 0) bends around A, and straightens again once A is gone. Both
ROS graphs are listed from one container, and neither holds a node or topic of
the other: what separates them is the domain id, not the network.

**What it does not do.** A peer beyond the range is not drawn. A mark left
behind when this robot drives away from it stays until the robot comes back
within range, which is how nav2 treats anything a sensor saw. A peer whose
publisher sends a pose only when it moves will drop out when it stops — every
`spatialdds_ros` robot sends at least `pose_floor_hz` for exactly this reason,
but the wire gives a consumer no way to know what a foreign one promises
([`docs/spec-queue.md`](https://github.com/OpenArCloud/spatialdds-ros/blob/main/docs/spec-queue.md)
§3).

## Configuration

Three parameters have no default and the node refuses to start without them,
because every possible default would be a false statement about the world:

| parameter | why there is no default |
|---|---|
| `scene` | which world this robot is in. A wrong scene is an empty bus, not an error |
| `entity_id` | which robot it is. A default collides the moment a second one runs |
| `frame_id` | which frame its pose is in. A pose without its frame is meaningless |

Everything else has one: `domain` 0, `ground_z` 0.0, `pose_source` tf,
`nav2_action` navigate_to_pose, `source_id` derived from `entity_id`,
`content_refs` and `external_refs` empty, `pose_floor_hz` 2.0, `awareness`
off, `peer_radius_m` 0.3, `awareness_types` empty. One more has none, and is
needed only with awareness on: `awareness_range_m`, for the reason above.

### What the robot *is*, and what it is *also known as*

```sh
python3 -m spatialdds_ros.standalone \
    --scene bare-scene --entity-id ent:robot:r1 --frame-id map/bare-scene \
    --content-ref catalog:dcef7c31-1d5f-54aa-9344-899912f9d34a \
    --external-ref wikidata=Q4810574
```

`content_refs` is how a consumer finds out what to *draw*. A client that reads
a `catalog:` reference can resolve the model behind it; without one it has
nothing to resolve and will render a marker. That is not a bug in the client —
it is the honest rendering of a robot that has not said what it looks like.

`external_refs` is what it is also known as: a wikidata id, an
asset-management number, a fleet number.

**Both are empty unless configured, and empty is a statement.** The adapter
publishes what the operator told it and nothing else — it invents no reference
and guesses no asset, because a reference nobody authorised is a claim on
nobody's authority. An empty sequence says "this robot names no content"; it
is not an accident of omission, and `test/test_adapter.py` asserts both
directions.

On a ROS robot the same two are string-array parameters, with `external_refs`
as `key=value` entries:

```sh
ros2 run spatialdds_ros robot_node --ros-args \
    -p scene:=bare-scene -p entity_id:=ent:robot:r1 \
    -p frame_id:=map/bare-scene \
    -p content_refs:='[catalog:dcef7c31-1d5f-54aa-9344-899912f9d34a]' \
    -p external_refs:='[wikidata=Q4810574]'
```

### Property keys

The adapter publishes two entity properties, and these names are **the
convention** rather than this library's private spelling — a consumer should
be able to read any `spatialdds_ros` robot without diffing two records to
discover what they are called:

| key | meaning |
|---|---|
| `label` | the human-facing name, from `label` or the `entity_id`'s tail |
| `nav.status` | what became of the last goal, held until a new one is accepted |

Unprefixed on purpose. An earlier consumer had published `demo.label` and
`demo.nav_status`, which scoped a general fact to one deployment; a second
consumer reading that record would have had to know whose demo it was.

### The anchor, and what its absence means for a consumer

`anchor_lat_deg`, `anchor_lon_deg` and `anchor_height_m` say where the robot's
frame sits on the earth. Set all three and the node publishes a
`FrameTransform` from that frame to `earth-fixed`. Set none and it publishes
none — a supported mode, not a misconfiguration. Set one or two and it refuses
to start, because a partial anchor is not a weaker claim about where the frame
is, it is an unusable one.

**What a reader of the scene sees in that mode.** The robot's entity and its
poses are present, complete and moving, and *this adapter* makes no claim
about where that frame sits on the earth.

Whether anybody else does is not something it can see. **A venue publishing
its own georeference is the ordinary case**, and then anchorless is the
correct configuration — the scene keeps the single claim and the robot does
not contradict it. That is how SpatialDDS-demo runs. If nobody publishes one,
a client that wants to place the robot on a globe cannot, and should say so
rather than assume a position.

The note said "nothing earth-references them" until 2026-10-19, which assumed
this adapter was the only thing that could. In the first real deployment it
was not, and anchorless was correct *because* somebody else owned the claim —
the opposite of the absence the wording described.

The degradation is deliberate either way. The alternative, defaulting the
anchor to the frame origin, would have every unconfigured robot claiming to be
in the Gulf of Guinea with a pose that looks entirely plausible. A default is
not neutral when the thing being defaulted is a claim.

## What it claims, and what it will not

The adapter publishes `basis = OBSERVED` and offers no parameter to change it:
a self-localizing robot is reporting where it has worked out that it is, which
is a sighting rather than an assertion. It claims no extent — a robot locates
itself, and its footprint is not a region anybody should plan by. (Until 0.4.0
this said an extent would be read as keep-out by the `DECLARED` rule; the
robot is `OBSERVED`, so that rule excludes it before it looks at extent. The
conclusion stands; the reason was wrong. Publishing a true footprint is a
follow-up.)

**Its pose goes out at least `pose_floor_hz` (2 Hz), moving or not.** A
consumer stops drawing a peer it has not heard from within that peer's
cadence, and some localizers fall silent when the robot stops — AMCL updates
on motion — so without a floor a stationary robot would vanish from its
peers' costmaps while standing in their way. In `pose_source=amcl` a live
AMCL publisher counts as holding the estimate; a robot whose localizer has
actually gone still goes `UNOBSERVED` on time. Anything at or below 1 Hz is
refused, as slower than a consumer's shortest window.

Three things it refuses to do quietly:

- **It does not dispose its entity when the feed goes quiet.** Disposal says
  the robot is gone; silence only licenses "this is where it was last seen,
  not where it is". The record is republished `UNOBSERVED` with a reason and
  the last pose kept.
- **It does not accept a goal nothing will act on.** No pose publishing, or
  nav2 not accepting goals, is declined *with the reason* — a declined goal
  must not look like an ignored one.
- **It does not let an accepted goal fail silently.** Whether a goal is
  drivable can depend on where the robot is when it meets a boundary, so the
  refusal arrives after the acceptance, on the latched record, and stays there
  until a new goal is accepted.

QoS and topic names are not configurable. Both are interop contracts whose
failure mode is silence: a mismatched QoS means endpoints that never meet with
nothing reporting it, and a hand-written topic name means a robot publishing
where nobody is listening.

### Sharing a participant with another SpatialDDS library

The §3.3.3 profiles go on the **endpoints**. The `Topic` objects this library
creates carry default QoS, and that is deliberate rather than an omission.

A topic name is unique within a participant, and its QoS has to agree among
every live `Topic` object for that name — in either direction, with the
refusal (`DDS_RETCODE_INCONSISTENT_POLICY`) landing on whichever call came
second. Putting the profile on the topic therefore made topic QoS a contract
every *other* participant in the process had to match, and nothing in this
API said so. Version 0.2.0 did that, and SpatialDDS-demo was refused an entity
reader on a lane this adapter had already opened — both conventions legal DDS,
neither legal beside the other.

So this library now asks for the least it can: the default, which is what any
participant that passes no QoS gets, including every ROS and DDS example.
Nothing leaves the wire — matching is a reader/writer negotiation over
endpoint QoS, and a topic's QoS enters it only as a default for endpoints that
ask for one, which these never do.

If you run another SpatialDDS stack in the same process and it creates its
topics with an explicit QoS, it is the one imposing the contract, and the two
cannot share that name. `test/test_transport.py::TwoConventionsOnOneTopic`
pins all of it, including a library writer exchanging real samples with a
reader built the other convention's way.

### Reads see the lifecycle

`transport.take_samples` returns payloads; `transport.take_with_state` returns
them alongside instance state for a caller that wants the lifecycle. Both
matter because CycloneDDS's `take()` does not return payloads only — it also
yields `InvalidSample` notifications for a dispose or a departed writer, and a
read that assumes otherwise raises on the first peer that joins a lane and
leaves. Until 0.2.1 this library made that assumption in one place, which
reached two call sites, and the first real deployment found it within a
minute. [`docs/failability.md`](https://github.com/OpenArCloud/spatialdds-ros/blob/main/docs/failability.md) §3 has the tracebacks.

This adapter still does not *act* on disposal: it owns one key, never disposes
it, and reports going quiet as `UNOBSERVED` — a weaker statement than "this
robot is gone". Not acting on an event is not the same as not receiving one.

**And nothing it writes is disposed for it.** CycloneDDS's default writer QoS
disposes a writer's instances when the writer goes, so until 0.4.0 a robot
that crashed reached every reader as `NOT_ALIVE_DISPOSED` — a deliberate
removal, by §2.14's reading — ten seconds after the crash. Every writer here
now turns that off, and the same events arrive as `NOT_ALIVE_NO_WRITERS`:
liveliness lost, which is what happened.

## Tests

```sh
python3 -m pytest test/ -q
```

Everything runs with no ROS installed — that is a gate
(`test/test_no_ros_needed.py`), not a habit, since one module-scope
`import rclpy` would take the whole suite offline. The DDS-backed tests need
`cyclonedds` and skip with a reason without it.

| document | what is in it |
|---|---|
| [`docs/extraction-census.md`](https://github.com/OpenArCloud/spatialdds-ros/blob/main/docs/extraction-census.md) | what became a parameter, what the library fills, and why |
| [`docs/awareness-census.md`](https://github.com/OpenArCloud/spatialdds-ros/blob/main/docs/awareness-census.md) | what becomes an obstacle, when it stops, in which frame — measured, and ruled |
| [`docs/failability.md`](https://github.com/OpenArCloud/spatialdds-ros/blob/main/docs/failability.md) | the two claims broken deliberately, with the failures |
| [`docs/spec-queue.md`](https://github.com/OpenArCloud/spatialdds-ros/blob/main/docs/spec-queue.md) | things for the specification rather than worked around |
| [`spatialdds_idl18/PROVENANCE.md`](https://github.com/OpenArCloud/spatialdds-ros/blob/main/spatialdds_idl18/PROVENANCE.md) | where the vendored 1.8 bindings came from |
| [`docs/RELEASING.md`](https://github.com/OpenArCloud/spatialdds-ros/blob/main/docs/RELEASING.md) | how a version reaches PyPI, and how the artifact is checked against the commit |

## Licence

MIT. See [`LICENSE`](https://github.com/OpenArCloud/spatialdds-ros/blob/main/LICENSE).
