# `spatialdds_ros` — extraction census

What `robot_bridge` and its neighbours in SpatialDDS-demo hard-code, field by
field, each with a verdict. The library generalizes that prototype; this is
the inventory of what has to become configurable, what the library should
simply fill in, and what belongs to the Littlefield plaza and does not travel.

**Three verdicts, and the rule separating them.** The program convention is
that *conventions fill plumbing and never claims*. So:

- **parameter** — an operator must be able to set it, because the library
  cannot know it and getting it wrong is a false statement about the world.
- **convention** — the library fills it and does not offer a knob, because
  there is one right answer and a knob would only let someone be wrong. QoS
  and topic grammar are the clearest cases: both are interop contracts.
- **out** — specific to the demo's venue, catalogue or container, and not the
  library's business.

**Read for this census** (all read-only): `spatialdds_demo/robot_bridge.py`,
`topics.py`, `qos_profiles.py`, `tempo.py`, `plaza.py`, `keepout.py`,
`typed_transport.py`, `dds_transport.py`, `model_service.py`,
`ar_demo/spatialdds_catalog_server.py`, and the ROS-side tier in
`robot_tier/` — `plaza_sim_node.py`, `keepout_node.py`, `nav2_params.yaml`.

---

## 1. Scene and topics

| what | where | current | verdict |
|---|---|---|---|
| scene name | `topics.py:21` | `ut-austin-littlefield` | **parameter** |
| entity topic | `topics.py` | `spatialdds/{SCENE}/world/entity/v1` | **convention** |
| pose topic | `topics.py` | `spatialdds/{SCENE}/world/model_pose/v1` | **convention** |
| command topic | `topics.py` | `spatialdds/{SCENE}/world/model_command/v1` | **convention** |
| frames topic | `topics.py` | `spatialdds/{SCENE}/frames/frame_transform/v1` | **convention** |

The scene is the only free variable; the four topic names are §3.3.1's
five-segment grammar applied to it. Exposing the topic strings themselves
would let an operator produce a name that no other participant will look for,
which is a silent non-interop — the failure mode is an empty bus, not an
error. So: one `scene` parameter, four names derived.

## 2. Domain and environment

| what | where | current | verdict |
|---|---|---|---|
| DDS domain | `dds_transport.py:228`, `--domain` | `SPATIALDDS_DDS_DOMAIN`, default 0 | **parameter** |
| transport gate | `dds_transport.py:218` | requires `SPATIALDDS_TRANSPORT=dds` | **out** |
| config path gate | `dds_transport.py:224` | requires `CYCLONEDDS_URI == file:///etc/cyclonedds.xml` | **out** |

The domain is a parameter and matters more here than in the demo: a ROS robot
already has a domain, and the library's whole claim is that it joins the bus
ROS is on rather than standing up a second one.

Both gates are out, and the second is worth a sentence because it cost real
time. `require_dds_env` refuses any `CYCLONEDDS_URI` other than that exact
path — reasonable as a container's own policy, wrong as a library's. A ROS
install has its own Cyclone configuration and the library has no business
dictating where it lives. (During the AWS deployment this guard was satisfied
by mounting a file *at* the required path rather than by loosening the check,
which was right for the demo and is exactly the constraint a library must not
inherit.)

## 3. Identity

| what | where | current | verdict |
|---|---|---|---|
| entity id | `robot_bridge.py:74` | `ent:robot:tb3` | **parameter** |
| source id | `robot_bridge.py:75` | `svc:robot:demo/tb3` | **parameter** |
| type URI | `robot_bridge.py:81` | wikidata `Q4810574` (mobile robot) | **convention**, overridable |
| display label | `robot_bridge.py` property | `TurtleBot` | **parameter** |
| prose note | `robot_bridge.py` property | the OBSERVED explanation | **out** |
| catalogue ref | `robot_bridge.py:85` | `catalog:dcef7c31-…` | **out** |

`entity_id` and `source_id` are the two an operator must set, and a fleet of
more than one makes that unavoidable. The type URI has a defensible default —
Q4810574 was verified against the entity data rather than taken from a search
result, and it is right for any mobile robot — but a drone or an AMR may want
its own, so it is a default and not a fixture. The catalogue reference is a
row in the demo's content catalogue and means nothing elsewhere.

## 4. The claims — what the adapter asserts about the robot

| what | where | current | verdict |
|---|---|---|---|
| basis | `robot_bridge.py` `_record` | `OBSERVED` | **convention**, not configurable |
| layer | `robot_bridge.py` `_record` | `FAST` | **convention** |
| extent | `robot_bridge.py` `_record` | `has_extent=False` | **convention**, parameter later |
| lifecycle on silence | `robot_bridge.py` `tick` | `UNOBSERVED` + reason | **convention** |
| refusal surfacing | `robot_bridge.py` `refused` | `demo.nav_status` on the latched record | **convention** |

These are the library's statements, not the operator's, and that is the point
of the category. A self-localizing robot reporting where it has worked out
that it is, is `OBSERVED` — a sighting, not an assertion, and not `DERIVED`
because nothing here fuses anyone else's claims. Offering a `basis` parameter
would let an operator mislabel the one thing the adapter actually knows.

Two behaviours belong here rather than in configuration because they were
each learned the hard way in the demo, and both are about not lying by
omission:

- **UNOBSERVED is not absence.** When the feed goes quiet the entity is
  republished with `state = UNOBSERVED` and a reason, rather than disposed
  (which says the robot is gone) or left claiming its last pose as current.
- **A refusal reaches whoever asked.** A `goto` is accepted before anyone can
  know whether it is drivable, so the refusal arrives later, on the latched
  record. Demo commit `b2b826a`; before it, the reason reached only the
  robot's stdout and a tap looked ignored.

`has_extent=False` is right for now — the robot locates itself and does not
legislate a region — but a footprint is a plausible later parameter for a
robot whose size matters to other participants.

## 5. The frame, and its scale declaration

| what | where | current | verdict |
|---|---|---|---|
| frame fqn | `model_service.py:69` | `map/ut-littlefield-fountain` | **parameter** |
| frame uuid | `model_service.py` `venue_frame` | `uuid5(NAMESPACE_URL, fqn)` | **convention** |
| coord convention | `venue_frame` | `ENU` | **convention** |
| scale declaration | `venue_frame` | `SCALE_DECLARED`, `1.0`, `'m'` | **convention** |
| ground height | `plaza.py:31` | `GROUND_Z = -0.84` | **parameter** |
| venue anchor | `plaza.py:73-75` | 30.28396857…, -97.73974829…, 145.083 | **parameter**, see §12 |
| ENU↔ECEF | `plaza.py` `enu_to_ecef_pose` | shared, one implementation | **convention** |

The uuid derivation is a convention and load-bearing: the demo derives it from
the fqn so that the scene, the catalogue and the announce name *one* frame
rather than three. Any library doing otherwise would fragment the identity it
is supposed to join.

The scale quartet is a convention the library can fill honestly for ROS
specifically: REP-103 makes ROS metric, so `SCALE_DECLARED` with
`meters_per_unit = 1.0` is a true statement about any conformant ROS robot
rather than a guess. Worth stating as the reason, because it is the one place
the library asserts a number it did not measure — and it is the member a 1.7
reader truncates away, verified on the deployed service this week (8 members
to a 1.8 reader, 4 to a 1.7 one).

## 6. QoS and durability

| lane | profile | QoS | verdict |
|---|---|---|---|
| entity | `MODEL_LATCHED` | RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1) per key | **convention** |
| pose | `MODEL_FAST` | BEST_EFFORT + VOLATILE + KEEP_LAST(1) per key | **convention** |
| command | `MODEL_COMMAND` | RELIABLE + VOLATILE + KEEP_LAST(16), unkeyed | **convention** |

Not parameters, and the demo has the scar that proves it. Deadline is a
request/offered QoS in DDS: a reader requesting 33 ms does not match a writer
offering none, **and the failure is silent**. An independently-built probe
transcribed the spec's old table, read its "Typical" heading as advisory,
omitted the deadlines and exchanged nothing;
`tests/test_interop.py::DeadlineIsLoadBearing` pins it. A QoS knob on a
library is a way to produce two endpoints that will never meet and no error
to say so.

## 7. Cadence and rates

| what | where | current | verdict |
|---|---|---|---|
| latch cadence | `tempo.py:36` | `LATCH_EVERY_N = 5` | **convention** |
| idle floor / factor | `tempo.py:42-43` | `1.0 s`, `3.0×` | **convention** |
| silence threshold | `tempo.py` `threshold()` | derived from observed cadence | **convention** |
| sim report rate | `robot_bridge.py:96` | `SIM_HZ = 5.0`, `--hz` | **parameter** (sim only) |

The derivation is the convention, not any particular number. `tempo.py`
exists because a fixed threshold shorter than the update interval makes every
gap look like a stop — measured in the demo on the duck mover, where a
one-second threshold against a mover giving each duck a turn every 1.5 s
produced 94 "it stopped moving" flushes while nothing had stopped, and "the
fast tier quietly became the slow one". A robot's rate is whatever ROS
publishes at, and a library cannot know it, so it must derive and must not
ask.

## 8. The ROS input and the nav2 output

| what | where | current | verdict |
|---|---|---|---|
| pose input topic | `robot_bridge.py:637` | `/odom` (`nav_msgs/Odometry`) | **parameter**, and see the finding below |
| nav2 action | `robot_bridge.py` `Nav2Navigator` | `navigate_to_pose` | **parameter** |
| goal frame | `Nav2Navigator.send` | `header.frame_id = "map"` | **parameter** |
| goal z | `Nav2Navigator.send` | `GROUND_Z` | **parameter** |
| yaw convention | `robot_bridge.py:104` | identity faces east | **convention** |
| mask/info topics | `keepout_node.py:71-73` | `/keepout_filter_mask`, `/costmap_filter_info`, `/map` | **parameter** |
| map frame | `keepout_node.py:84` | `map` | **parameter** |

**The pose input is the one substantive design defect the extraction found,
and it is not a naming question.** `robot_bridge`'s own module docstring says
it subscribes to what a navigating robot already publishes, `/amcl_pose`
(line 9); the code subscribes to `Odometry` on `/odom` (line 637). In the demo
that works, and `plaza_sim_node` says exactly why in its own comment:

> `map -> odom`, identity and static: this sim knows exactly where the robot
> is, so there is no drift for a localizer to correct.

So `/odom` *is* map-frame — in the simulator, by construction. On a real robot
it is not: odom drifts relative to map, which is the entire reason AMCL
publishes a correction. A library that read raw `/odom` and published it as an
`OBSERVED` pose in a georeferenced frame would emit a slowly-drifting claim
about where a robot is on the earth, with every number looking plausible —
the same silent-wrongness shape as an inverted frame transform.

The library's input must therefore be a **map-frame** pose. The honest default
is the TF chain `map → base_link` (which is what nav2 itself consumes, works
whatever the localizer is, and needs no assumption about topic names), with
`/amcl_pose` accepted as an alternative for robots that publish it. Raw
`/odom` should be available only with the frame stated, never as the default.

Filed for the demo repo's queue rather than fixed here, per the brief.

## 9. The keep-out coupling

| what | where | current | verdict |
|---|---|---|---|
| mask from declarations | `keepout.py` `build_mask` | `KEEPOUT_BASES = ("DECLARED",)` | **convention**, optional capability |
| clearance | `robot_bridge.py:350` | `CLEARANCE_M = 1.2` | **out** |
| shoulder | `keepout_node.py:100` | `SHOULDER_M = 0.9` | **parameter** |
| costmap-filter path | `keepout_node.py` | OccupancyGrid + CostmapFilterInfo, latched | **convention** |

The division here is the interesting one. `CLEARANCE_M` is explicitly tied to
what is **drawn**, not to a real robot: the comment says the asset is scaled
up for legibility, 0.14 m across the plates becoming about 2 m on screen, and
"a demo whose robot visibly overlaps water it is refusing to enter has lost
the argument regardless of what the arithmetic says". That is a presentation
decision about a GLB and does not travel.

For a real robot the clearance is nav2's footprint and inflation radius, which
nav2 already has and the library must not duplicate. So the library's keep-out
job is the `keepout_node` path — turn declared extents into a costmap filter
and let nav2 plan — and *not* the kinematic sim's own mask test. The two must
not both be offered as if equivalent: one is the mechanism, the other is a dev
fallback's refusal to move.

`keepout_node`'s reasoning for a filter rather than an obstacle layer should
survive the extraction verbatim, because it is a mechanical argument and not a
preference: the obstacle layer raytraces to clear cells its sensors see
through, so a virtual obstacle producing no real returns is marked and then
promptly cleared. Filters are nav2's mechanism for policy-derived exclusion.

## 10. Process shape

| what | where | current | verdict |
|---|---|---|---|
| dual-stack participant | `robot_bridge.py` `_run_with_ros`, `keepout_node.py` | rclpy + cyclonedds in one process | **convention** |
| deferred rclpy import | `robot_bridge.py:597` | imported inside `_run_with_ros` | **convention** |
| one writer per key | `robot_bridge.py` `already_published` | 2.5 s settle, then refuse to start | **convention** |
| kinematic source | `robot_bridge.py` `PlazaSim` | ships as the no-hardware mode | **convention** (per brief §3) |
| sim start / ground | `plaza.py:52`, `:31` | `(17.0, -21.5)`, `-0.84` | **parameter** |

The dual-stack pattern is the library's foundation and is better evidenced
than it sounds: ROS Humble's middleware is CycloneDDS 0.10.5 while the Python
binding here is 11.0.1, which looks impossible and works because the binding
loads its own `libddsc` and shares no symbols with `rmw_cyclonedds_cpp`'s.
`bridges/ros2_bridge` has held both in one process since long before the robot
tier.

The deferred `rclpy` import is not tidiness — it is what keeps the module
importable, and therefore testable, on a machine with no ROS. That is the CI
story, and it should be a gate rather than a habit.

`already_published` is a real safety property and should be kept: one writer
per key is the rule the whole model rests on, and two adapters publishing one
robot would put it in two places with no error anywhere.

---

## 11. What the extraction revealed about the demo

Filed, not fixed — cross-repo fixes are out of scope per the brief. For the
demo repo's queue:

1. **`robot_bridge` reads `/odom` where its docstring says `/amcl_pose`.**
   §8 above. Correct in the sim because `map→odom` is identity there; wrong
   on any real robot. The docstring describes the right design and the code
   implements the one that works in simulation.

2. **`model_service.py` assigns seven module-level constants twice**, lines
   69–78 duplicated at 89–98 plus 85–87 at 105/129–130. Four pairs agree;
   three re-assign as literals what the first assignment imported from
   `tempo.py` — `LATCH_EVERY_N_MOVES = 5`, `IDLE_FLUSH_S = 1.0`,
   `IDLE_FLUSH_FACTOR = 3.0`. The literals currently match `tempo`'s values,
   so nothing is broken today; the later assignment wins, so changing
   `tempo.py` would silently not reach `model_service`. That defeats the
   single source of truth `tempo.py` exists to be.

3. **`CLEARANCE_M` is documented as tied to the drawn asset** rather than the
   robot. Correct and well-argued for the demo; worth a note that the real
   robot path should take it from nav2's footprint, so nobody later reads
   1.2 m as a physical fact.

---

## 12. The design question, for ruling

**What the adapter claims versus what the operator configures.**

The venue-georeference batch is the pattern to follow: the scene publishes its
own `FrameTransform` from a configured anchor, the ENU↔ECEF arithmetic lives
in exactly one place, and **nothing is defaulted that is a claim**. No anchor
meant no georeference published, stated plainly, rather than a venue at (0,0).

Proposed parameter surface, with what each silence means:

| parameter | required | silence means |
|---|---|---|
| `scene` | **yes** | refuse to start. There is no sensible default scene; joining the wrong one is an invisible non-interop, and a made-up name puts the robot in a world nobody is watching. |
| `entity_id` | **yes** | refuse to start. A default would collide the moment two robots run, and the one-writer-per-key guard would then stop the second with a confusing reason. |
| `source_id` | no | derive from `entity_id`. It identifies the publisher, not a claim about the world. |
| `frame_id` (fqn) | **yes** | refuse to start. A pose is meaningless without the frame it is in, and a default frame name is a claim that the robot is somewhere it may not be. |
| `anchor` (lat, lon, height) | no | **publish no `FrameTransform`.** Say so once, plainly. The robot still publishes its pose in its frame; the scene simply has no georeference from this adapter. This is the venue-georeference rule applied exactly. |
| `domain` | no | 0, matching DDS's own default and the demo's. |
| `ground_z` | no | 0.0 — a plane the robot drives on is a reasonable neutral, and it is a frame-local offset rather than a claim about the earth. |
| `pose_source` | no | TF `map → base_link`. See §8: this is the one default that must not be `/odom`. |
| `nav2_action` | no | `navigate_to_pose`, nav2's own name. |
| `type_uri` | no | Q4810574, verified, right for any mobile robot. |
| `label` | no | the `entity_id`'s tail. Cosmetic. |

**The three that refuse to start** are the ones where every possible default
is a false statement: which world this robot is in, which robot it is, and
which frame its pose is expressed in. The brief's rule is that nothing
defaulted may be a claim, and those three are nothing but claims.

**The anchor is the interesting one** and the reason this question needs a
ruling rather than a decision. It is the only parameter whose absence has a
*visible, useful* degraded mode: without it the adapter is still completely
functional on the bus — entities, poses, commands, refusals all work — and
only the earth-referencing is absent. So silence there is a legitimate
operating mode rather than a misconfiguration, and the library should say so
once at startup and carry on, exactly as the venue does.

**The rejected alternative, kept here as the cautionary example.** Let the
anchor default to the frame origin at (0, 0, 0) so a `FrameTransform` is
always published. Every unconfigured robot would then claim to be in the Gulf
of Guinea, and its pose would still look entirely plausible — no exception, no
malformed record, nothing a schema check would catch. That is the
silent-wrongness class wearing a default's clothing, and it is the same
failure the venue-georeference batch was built to prevent. A default is not
neutral when the thing being defaulted is a claim.

**Ruled as proposed, all three rows (2026-10-19).** Refuse-to-start for
`scene`, `entity_id` and `frame_id`; the documented-defaults set as plumbing;
the anchor approved as the one legitimate silence, because its degradation is
honest and visible — fully functional on the bus, not placeable on the earth,
said once plainly.

With one addendum carried into the build: **the README states the degraded
mode's visible consequence from the consumer's side**, not only the
operator's. A reader of the scene sees the robot's entity and its poses
present and moving, and nothing earth-referenced — no `FrameTransform` for the
robot's frame, so a client that wants to place it on a globe cannot, and
should say so rather than assume. Documenting it only as "the operator did not
set an anchor" would leave the consumer to discover the absence.
