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
from typing import Optional

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

    def __post_init__(self) -> None:
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
            notes.append(
                "no anchor configured, so no FrameTransform is published: this "
                "robot's entity and poses are on the bus and complete, and "
                "nothing earth-references them — a consumer that wants to "
                "place it on a globe cannot, and should say so rather than "
                "assume a position")
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
