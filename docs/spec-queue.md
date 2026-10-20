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
