# For the SpatialDDS spec queue

Things this library ran into that the specification, rather than this
implementation, would have to settle. Filed here rather than worked around
silently, and each one says what the library does in the meantime.

---

## 1. `ServiceKind` has no member for an agent

**Raised 2026-10-19, building `announce.py`.**

A robot that publishes its own pose and accepts `ModelCommand` goals is
discoverable in exactly the way a VPS or a content catalogue is: something
that wants to drive it should be able to find it without being told its topic
names. So it belongs on the discovery lane, and `Announce.kind` has to say
what it is.

1.8's `ServiceKind` is:

    VPS, MAPPING, RELOCAL, SEMANTICS, STORAGE, CONTENT,
    ANCHOR_REGISTRY, SENSING, INFRASTRUCTURE, FUSION, OTHER

None of them is an agent. The nearest candidates and why each is wrong:

- **`SENSING`** reads as a service offering sensor feeds — point clouds,
  images, detections. A robot announcing `SENSING` invites a consumer to look
  for data lanes it does not publish. It is close enough to be tempting and
  that is exactly the problem: it would be wrong in a way nobody notices until
  a client asks for something.
- **`INFRASTRUCTURE`** suggests fixed plant — the opposite of the one property
  that matters about a robot.
- **`FUSION`** is about combining other people's claims, which is precisely
  what the robot's `OBSERVED` basis says it is *not* doing.

**What the library does meanwhile.** Defaults to `OTHER`, which is accurate,
and exposes `kind` as a parameter so an operator whose robot genuinely is one
of the listed kinds can say so. `OTHER` is chosen deliberately over the
nearest-fit: stretching an enum member makes every robot on the bus advertise
a capability it does not have, and leaves the next reader no way to know the
enum had been stretched.

**What the spec might do.** An `AGENT` member — something that occupies space,
reports where it is, and may accept commands about where to go. Note that this
is not only robots: a tracked person, a vehicle, a drone and an AMR all share
the shape. `spatial.owm` already models them as entities with a `FAST` layer;
discovery has no corresponding vocabulary.

A smaller alternative: say in prose that `OTHER` is the correct answer for an
agent, so that implementations converge rather than each picking a different
nearest-fit. That would be worth having even if `AGENT` is added later, since
1.8 is already published.

---

## 2. `BBox2D`'s element order lives only in a comment

**Noticed 2026-10-19, building the announce's coverage.**

`BBox2D` is `typedef double BBox2D[4]` (`idl/v1.8/types.idl:16`) — four
unnamed doubles. The order is stated in a comment at one use site:

    spatial::common::BBox2D bbox;  // [west, south, east, north]
                                   -- idl/v1.8/discovery.idl:96

and a *different* order at another:

    spatial::common::BBox2D bbox;  // [u_min,v_min,u_max,v_max] (px)
                                   -- idl/v1.8/semantics.idl:51

Both are reasonable for their contexts, and together they mean the type
carries no order of its own — it is whatever the field's comment says. A
transposed pair produces a well-formed box in the wrong place, which nothing
downstream can detect.

This is not a defect so much as a hazard worth knowing about. Filed because
getting it wrong is silent and because the two orders in one specification
version invite exactly the mistake. A reader who learned `[west, south, east,
north]` from the discovery module and applied it in the semantics module would
be wrong, with no error anywhere.

**What the library does meanwhile.** Writes the order out at the construction
site with the IDL line cited, rather than passing coordinates through a helper
whose argument order could be read either way.

**What the spec might do.** Name the members — a small struct instead of an
array — or, far cheaper, state the order once on the typedef and have each use
site refer to it rather than restate it.

---

## 3. A FAST entity has no freshness contract

**Raised 2026-10-21, the awareness census (`awareness-census.md` §3).**

A consumer planning around another robot has to decide when that robot stops
being where it was last seen. Nothing on the wire says. `Entity` has no ttl;
the spec's one staleness rule, "Consumers SHOULD discard Announce samples
where `now - stamp > 2 * ttl_sec`" (1.8-full:1326 at `9c3e1f2`), is about
Announce; and `TopicMeta.target_rate_hz = 0` means *unspecified*. So a
publisher that speaks only when its entity moves and one that has died look
the same until something else says otherwise.

**What the library does meanwhile.** Learns each peer's cadence from its own
poses (`Tempo`, the derivation the adapter uses for its own silence) and
stops projecting a peer once that cadence is broken. The record is kept, per
§2.14.

**What the spec might do.** Let a FAST entity's publisher state its cadence —
a non-zero `target_rate_hz` given meaning, or a heartbeat promise on the pose
lane — so that silence beyond it is evidence rather than a guess.

---

## 4. `Entity` and `ModelPose` carry no pose uncertainty

**Raised 2026-10-21, the awareness census §2.**

Both carry a bare `PoseSE3` (`owm.idl:71-109`). An OBSERVED pose is a
sighting, and every sighting has error; without a covariance the wire implies
a precision no publisher claimed, and a consumer has nothing to inflate an
obstacle by. `core` already has `CovMatrix`, and `FrameTransform` uses it.

**What the library does meanwhile.** Draws a point peer as a disc of a
configured radius, documented as plumbing and not as a measurement.

---

## 5. The registered entity topic is not the one in use

**Raised 2026-10-21, the awareness census §7.**

1.8-full:1484 registers `model_entity` as
`spatialdds/<scene>/model/entity/v0`. This library and SpatialDDS-demo both
publish and read `spatialdds/<scene>/world/entity/v1` (`topics.py`). A
publisher following the registration and a consumer following either
implementation would never meet, and the only symptom is an empty bus.

**What the library does meanwhile.** Nothing different. Which side moves —
the registration or the implementations — is a ruling, and moving one
library's topic alone would split the bus it is meant to join.

---

## 6. §2.14 says what may not be purged, not what may stop being acted on

**Raised 2026-10-21, the awareness census §3.**

"Consumers MUST NOT treat a liveliness change as a deliberate removal, and
MUST NOT purge a durable keyed instance on liveliness loss alone"
(1.8-full:473). This is about the record, and it is right. Read literally by
a planner, it says to keep avoiding a robot that died ten minutes ago. A
consumer that keeps the record but stops *projecting* a peer with no current
evidence breaks neither sentence, but the text does not say so.

**What the spec might do.** A non-normative line distinguishing the record
from what a consumer derives from it.
