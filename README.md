# spatialdds_ros

Make a ROS 2 robot a SpatialDDS participant: its pose published as a
`spatial.owm` entity on scene-scoped topics, `goto` commands consumed from
`ModelCommand` and dispatched to nav2, and its presence announced — all on the
same DDS bus ROS already runs on.

**A type adapter on a shared bus, not a protocol gateway.** Nothing is
relayed. ROS and SpatialDDS meet on the wire because both are DDS, so this is
one process holding both stacks, publishing the model as model and reading ROS
as ROS.

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
robot: no anchor configured, so no FrameTransform is published: this robot's
       entity and poses are on the bus and complete, and nothing
       earth-references them — a consumer that wants to place it on a globe
       cannot, and should say so rather than assume a position
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

## Configuration

Three parameters have no default and the node refuses to start without them,
because every possible default would be a false statement about the world:

| | why there is no default |
|---|---|
| `scene` | which world this robot is in. A wrong scene is an empty bus, not an error |
| `entity_id` | which robot it is. A default collides the moment a second one runs |
| `frame_id` | which frame its pose is in. A pose without its frame is meaningless |

Everything else has one: `domain` 0, `ground_z` 0.0, `pose_source` tf,
`nav2_action` navigate_to_pose, `source_id` derived from `entity_id`.

### The anchor, and what its absence means for a consumer

`anchor_lat_deg`, `anchor_lon_deg` and `anchor_height_m` say where the robot's
frame sits on the earth. Set all three and the node publishes a
`FrameTransform` from that frame to `earth-fixed`. Set none and it publishes
none — a supported mode, not a misconfiguration. Set one or two and it refuses
to start, because a partial anchor is not a weaker claim about where the frame
is, it is an unusable one.

**What a reader of the scene sees in that mode.** The robot's entity and its
poses are present, complete and moving. Nothing earth-references them: there
is no `FrameTransform` for its frame, so a client that wants to place the
robot on a globe *cannot*, and should say so rather than assume a position.
That is the whole of the degradation, and it is deliberate — the alternative,
defaulting the anchor to the frame origin, would have every unconfigured robot
claiming to be in the Gulf of Guinea with a pose that looks entirely
plausible. A default is not neutral when the thing being defaulted is a claim.

## What it claims, and what it will not

The adapter publishes `basis = OBSERVED` and offers no parameter to change it:
a self-localizing robot is reporting where it has worked out that it is, which
is a sighting rather than an assertion. It claims no extent — a robot locates
itself, it does not legislate a region, and an extent would be read as
keep-out by anything applying the `DECLARED` rule.

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

## Tests

```sh
python3 -m pytest test/ -q
```

Everything runs with no ROS installed — that is a gate
(`test/test_no_ros_needed.py`), not a habit, since one module-scope
`import rclpy` would take the whole suite offline. The DDS-backed tests need
`cyclonedds` and skip with a reason without it.

| | |
|---|---|
| `docs/extraction-census.md` | what became a parameter, what the library fills, and why |
| `docs/failability.md` | the two claims broken deliberately, with the failures |
| `docs/spec-queue.md` | two things for the specification rather than worked around |
| `spatialdds_idl18/PROVENANCE.md` | where the vendored 1.8 bindings came from |

## Licence

MIT. See `LICENSE`.
