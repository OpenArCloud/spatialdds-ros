"""
The parameter surface, and what each silence means.

Ruled 2026-10-19; `docs/extraction-census.md` §12 is the reasoning and this is
the implementation of it. The rule the surface is built on is the programme's:
**conventions fill plumbing and never claims.** So a parameter exists where an
operator could be wrong *about the world*, and a default exists only where
being wrong is impossible.

Three parameters have no honest default and therefore refuse to start:

* `scene` — which world this robot is in. A made-up default puts it somewhere
  nobody is watching, and the failure is an empty bus rather than an error.
* `entity_id` — which robot it is. A default collides the moment two run, and
  the one-writer-per-key guard would then stop the second with a reason that
  points at the wrong thing.
* `frame_id` — which frame its pose is expressed in. A pose without its frame
  is not a weak claim, it is a meaningless one.

One silence is a legitimate operating mode rather than a misconfiguration:

* `anchor` absent — **no `FrameTransform` is published**, said once plainly.
  The adapter is still fully functional on the bus; the scene simply has no
  georeference from it. The alternative, defaulting the anchor to the frame
  origin, would have every unconfigured robot claiming to be in the Gulf of
  Guinea with a pose that looks entirely plausible. A default is not neutral
  when the thing being defaulted is a claim.
"""
from __future__ import annotations

import re
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

#: "mobile robot" — an automatic machine capable of movement in any given
#: environment, `subclass of` (P279) Q11012 (robot). Carried from the
#: prototype, where it was verified against the entity data rather than taken
#: from a search result: several US patents are also labelled "Mobile robot".
#: A default rather than a fixture — a drone or an AMR may want its own.
DEFAULT_TYPE_URI = "http://www.wikidata.org/entity/Q4810574"

#: nav2's own action name. A parameter because a robot may remap it; this is
#: what nav2 calls it unremapped.
DEFAULT_NAV2_ACTION = "navigate_to_pose"

#: How a map-frame pose is obtained. `tf` is the default for a reason the
#: census documents at length: the prototype read `nav_msgs/Odometry` on
#: `/odom`, which is map-frame *only* in the simulator, where `map -> odom` is
#: static and identity. On a real robot odom drifts against map, so publishing
#: it as a georeferenced OBSERVED pose emits a slowly-drifting claim about
#: where a robot sits on the earth with every number looking plausible.
POSE_SOURCES = ("tf", "amcl", "odom")

#: The IDL's bounds on the two reference sequences
#: (`sequence[str, 8]` and `sequence[KV, 8]` on `spatial.owm.Entity`).
#: Named here so a refusal can cite the limit rather than just report a
#: serialize failure.
MAX_CONTENT_REFS = 8
MAX_EXTERNAL_REFS = 8

#: One path segment of a SpatialDDS topic name (§3.3.1). Deliberately strict:
#: a scene name carrying a slash would silently produce a six-segment topic,
#: which is a name no other participant looks for.
_SEGMENT = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._-]*$")


class ConfigError(ValueError):
    """
    Raised for a configuration that cannot be started.

    Carries *every* problem rather than the first, because a robot operator
    editing a launch file should learn about three missing parameters in one
    pass instead of three.
    """


@dataclass(frozen=True)
class Anchor:
    """
    Where a local frame's origin sits on the earth.

    Validated rather than taken on trust: these three numbers are the library's
    only statement about the planet, and a transposed pair is not detectable
    downstream — an anchor at (-97.7, 30.3) instead of (30.3, -97.7) is a
    perfectly well-formed point in the South Atlantic.
    """

    lat_deg: float
    lon_deg: float
    height_m: float

    def __post_init__(self) -> None:
        problems = []
        if not -90.0 <= self.lat_deg <= 90.0:
            problems.append(
                f"lat_deg {self.lat_deg} is outside [-90, 90]; a latitude "
                f"beyond the poles usually means lat and lon were swapped")
        if not -180.0 <= self.lon_deg <= 180.0:
            problems.append(
                f"lon_deg {self.lon_deg} is outside [-180, 180]")
        if problems:
            raise ConfigError("; ".join(problems))


@dataclass(frozen=True)
class Config:
    """
    Everything the adapter needs, validated at construction.

    Frozen: the parameter surface is read at startup and a value that changed
    underneath a running adapter would mean the entity it has been publishing
    all along was a different claim than the one it is making now.
    """

    scene: str
    entity_id: str
    frame_id: str

    anchor: Optional[Anchor] = None
    source_id: Optional[str] = None
    domain: int = 0
    ground_z: float = 0.0
    pose_source: str = "tf"
    nav2_action: str = DEFAULT_NAV2_ACTION
    type_uri: str = DEFAULT_TYPE_URI
    label: Optional[str] = None

    #: What this robot *is*, for a consumer that wants to draw it.
    #:
    #: Added 2026-10-19 because their absence blocked the first real adoption.
    #: SpatialDDS-demo's robot carries `catalog:dcef7c31-…`, and a consumer
    #: reading that reference is what makes it render a robot rather than a
    #: pin: no reference, no asset, no model. An adapter that cannot say what
    #: a robot looks like cannot be a consumer's only publisher of that robot,
    #: and there was no way for a consumer to supply it from outside — the
    #: record simply had nowhere to put it.
    #:
    #: **Empty by default, and empty means empty.** The adapter states what the
    #: operator configured and nothing else; it invents no reference and
    #: guesses no asset. An empty sequence is a published fact — "this robot
    #: names no content" — not an accident of omission, and the gates assert
    #: both directions.
    content_refs: Tuple[str, ...] = ()

    #: What this robot is *also known as*: wikidata, an asset-management id, a
    #: fleet number. Same rule — only what was configured.
    external_refs: Tuple[Tuple[str, str], ...] = ()

    def __post_init__(self) -> None:
        # A launch file or a dict will hand these over as lists. The dataclass
        # is frozen, so they are normalised to tuples here rather than stored
        # mutable — a shared mutable default on a frozen "value" is a bug
        # waiting for a second robot.
        object.__setattr__(self, "content_refs",
                           tuple(self.content_refs or ()))
        object.__setattr__(self, "external_refs",
                           tuple(tuple(pair) for pair in
                                 (self.external_refs or ())))
        problems: list[str] = []

        # The three with no honest default. Reported together, and each with
        # what the silence would have meant rather than just "required".
        if not self.scene:
            problems.append(
                "scene is required: it names the world this robot joins, and "
                "there is no default that is not a guess about where it is. "
                "A wrong scene is an empty bus, not an error")
        elif not _SEGMENT.match(self.scene):
            problems.append(
                f"scene {self.scene!r} is not one topic segment (§3.3.1): "
                f"expected letters, digits, dot, dash or underscore. A scene "
                f"containing a slash silently produces a topic name no other "
                f"participant looks for")

        if not self.entity_id:
            problems.append(
                "entity_id is required: it names which robot this is. A "
                "default collides as soon as a second robot runs, and the "
                "one-writer-per-key guard would then refuse the second for a "
                "reason that points at the wrong thing")

        if not self.frame_id:
            problems.append(
                "frame_id is required: a pose without the frame it is "
                "expressed in is not a weak claim, it is a meaningless one")

        if self.pose_source not in POSE_SOURCES:
            problems.append(
                f"pose_source {self.pose_source!r} is not one of "
                f"{', '.join(POSE_SOURCES)}")

        if self.domain < 0 or self.domain > 232:
            problems.append(
                f"domain {self.domain} is outside the DDS range [0, 232]")

        # Bounded sequences in the IDL: `content_refs` and `external_refs` are
        # both `sequence[..., 8]`. Checked here rather than left to serialize,
        # which fails late and says nothing about which field or why.
        if len(self.content_refs) > MAX_CONTENT_REFS:
            problems.append(
                f"content_refs has {len(self.content_refs)} entries; the IDL "
                f"bounds it at {MAX_CONTENT_REFS}")
        for ref in self.content_refs:
            if not ref or not str(ref).strip():
                problems.append(
                    "a content_ref is empty; an empty reference resolves to "
                    "nothing and is not the same as publishing none")
        if len(self.external_refs) > MAX_EXTERNAL_REFS:
            problems.append(
                f"external_refs has {len(self.external_refs)} entries; the IDL "
                f"bounds it at {MAX_EXTERNAL_REFS}")
        for pair in self.external_refs:
            if len(tuple(pair)) != 2 or not all(str(v).strip() for v in pair):
                problems.append(
                    f"external_refs entries are (key, value) pairs and both "
                    f"halves must be non-empty; got {pair!r}")

        if problems:
            raise ConfigError(
                "this configuration cannot be started:\n  - "
                + "\n  - ".join(problems))

    # --- derived, never asked for -----------------------------------------

    @property
    def effective_source_id(self) -> str:
        """
        Who is publishing, which is not a claim about the world.

        Derived from `entity_id` when unset: the publisher's identity follows
        the thing it publishes, and asking an operator for both invites them
        to disagree.
        """
        if self.source_id:
            return self.source_id
        tail = self.entity_id.split(":")[-1] or self.entity_id
        return f"svc:robot:{tail}"

    @property
    def effective_label(self) -> str:
        """A human-facing name. Cosmetic, so the id's tail will do."""
        return self.label or (self.entity_id.split(":")[-1] or self.entity_id)

    @property
    def publishes_georeference(self) -> bool:
        return self.anchor is not None

    def startup_notes(self) -> list[str]:
        """
        What to say once, at startup, about the modes this configuration is in.

        Returned rather than logged so the caller owns the logging and the
        tests can read it. Each line states a *consequence*, not a setting:
        "no anchor configured" leaves a reader to work out what follows, and
        the whole point of the degraded mode being legitimate is that it is
        legible.
        """
        notes = [
            f"scene {self.scene}, entity {self.entity_id} in frame "
            f"{self.frame_id}, DDS domain {self.domain}",
        ]
        if self.anchor is None:
            # **Reworded 2026-10-19.** This used to say "nothing
            # earth-references them", which assumed this adapter is the only
            # thing that could — and in the first real deployment it was not:
            # the venue publishes the scene's own FrameTransform, and running
            # the robot anchorless is correct *because* somebody else owns
            # that claim, which is the opposite of the absence the old wording
            # described. So the note now says what it actually knows: this
            # adapter is not publishing one. Whether anyone else is, it cannot
            # see from here.
            notes.append(
                "no anchor configured, so this adapter publishes no "
                "FrameTransform: its entity and poses are on the bus and "
                "complete, and it makes no claim about where that frame sits "
                "on the earth. Another participant may own that claim — a "
                "venue publishing its own georeference is the ordinary case, "
                "and then this is the correct configuration. If nobody "
                "publishes one, a consumer cannot place this robot on a globe "
                "and should say so rather than assume a position")
        else:
            notes.append(
                f"anchor {self.anchor.lat_deg:.8f}, {self.anchor.lon_deg:.8f}, "
                f"{self.anchor.height_m:.3f} m — publishing {self.frame_id} -> "
                f"earth-fixed")
        if self.pose_source == "odom":
            notes.append(
                "pose_source is odom: odom drifts against map, so this "
                "publishes a pose that will slowly diverge from where the "
                "robot actually is. Correct only where map -> odom is static "
                "and identity, which is true of a simulator and false of a "
                "localizing robot. Prefer pose_source=tf")
        return notes
