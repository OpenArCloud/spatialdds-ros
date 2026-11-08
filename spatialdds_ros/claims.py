"""
Claims: the scene's relationship lane, as advice to this robot's planner.

**No ROS in here,** for the adapter's reason: what is drawn, what is ignored
and when a claim stops counting are all decided with no ROS installation.
`robot_node` publishes the result as a nav2 costmap-filter mask; this produces
the numbers. `docs/claims-census.md` is the reasoning behind every rule below.

The pattern is SpatialDDS-demo's keep-out node (`robot_tier/keepout_node.py`),
which draws a venue's declared bounds into a keep-out filter, generalized from
authored bounds to claims, and the consumption rules are the crossing scene's
(`docs/crossing-claims.md` there): its controllers trust named asserters on
named bases, and everything else on the lane is ignored.

**Claims are advisory.** A drawn claim raises the planner's cost of passing
through a region; it never makes the region lethal, and nothing here moves the
robot. The robot's own stack decides. What is drawn: a claim on the
relationship lane that

* a `TrustRule` admits, by asserter, basis and (if the rule says) predicate;
* is in force: not RETIRED, not disposed, and not past its own `valid_until`;
* if OBSERVED, is still being asserted (below);
* has a subject on the entity lane with a region this robot can place: the
  oriented box when the subject carries `demo.obb.*` (the crossing census's
  ruling, read centred on the pose and turned by its yaw), else its `Aabb3`,
  read as `awareness` reads one, in the entity's frame; and composes into
  this robot's frame through the shared georeference book.

Everything else is counted, and the ones an operator could be surprised by
carry a reason.

**An observation nobody is still making is not about the present.** The wire
carries no refresh interval on a claim: an asserter that holds an unchanged
opinion does not republish it (the demo's writer, "the revision is of the
opinion, not of the lineage"), so a claim's age is no evidence either way.
What the wire does carry is the asserter's liveliness, and that is the
freshness term the bus's live view already uses: when an asserter's writer
goes, its edges stand and the view marks them "nobody asserts it now"
(SpatialDDS-demo `web/src/claims.ts`, `writerGone`). This layer stops
drawing an OBSERVED claim at that same event. It does not forget it, and it
does not treat it as retired: a crash is never a retirement (§2.14), the
record is kept, and the claim is drawn again the moment its asserter is heard
from. A DECLARED claim is not perception and does not age out this way: a
declaration stands until its declarer withdraws it, and is drawn, with the
asserter's absence reported.
"""
from __future__ import annotations

import math
from collections import Counter
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

from cyclonedds.core import InstanceState
from cyclonedds.domain import DomainParticipant

from spatialdds_idl18.oarc_owm import Relationship, RelationshipLifecycle
from spatialdds_idl18.spatial.core import PoseSE3
from spatialdds_idl18.spatial.owm import (
    Basis, Entity, LifecycleState, ModelPose)

from spatialdds_ros import transport as tt
from spatialdds_ros.config import Config, ConfigError
from spatialdds_ros.frames import compose_point, frame_problem
from spatialdds_ros.georeferences import GeoreferenceBook
from spatialdds_ros.qos import MODEL_FAST, MODEL_LATCHED
from spatialdds_ros.topics import Topics

#: The mask's cell. Half the 0.1 m costmap SpatialDDS-demo runs, so a
#: region's edge lands inside the cell nav2 samples rather than on its border.
MASK_RESOLUTION_M = 0.05

#: The oriented box's dimensions, as SpatialDDS-demo's crossing scene carries
#: them (`scenarios/crossing/scene.py`, `OBB_KEYS`; ruled in its census §4).
#: Namespaced to the demo because 0.1's `Entity` has no oriented extent; the
#: 0.2 register and the 1.9 register both hold "an oriented extent" open, and
#: until one of them lands these keys are the only place the true box is.
OBB_LENGTH_KEY = "demo.obb.length_m"
OBB_WIDTH_KEY = "demo.obb.width_m"


@dataclass(frozen=True)
class Region:
    """One drawn claim: its subject's footprint, in this robot's frame."""
    claim_id: str
    subject_ref: str
    predicate: str
    asserter: str
    basis: str
    #: Corners, (x, y), in order round the footprint.
    polygon: Tuple[Tuple[float, float], ...]
    #: True when it is drawn although nobody asserts it now (DECLARED only).
    unasserted: bool = False


@dataclass
class ClaimsPicture:
    """What the layer would draw now, and why everything else is not."""
    regions: List[Region] = field(default_factory=list)
    #: claim id -> why it is not drawn, for the skips a reader needs told.
    skipped: Dict[str, str] = field(default_factory=dict)
    counts: Counter = field(default_factory=Counter)


@dataclass
class _Claim:
    edge: Relationship
    writer_gone: bool = False


@dataclass
class _Subject:
    record: Optional[Entity] = None
    pose: Optional[PoseSE3] = None
    disposed: bool = False


class ClaimsLayer:
    """Reads the scene's entity, pose and relationship lanes, and says what
    to draw."""

    def __init__(self, participant: DomainParticipant, config: Config) -> None:
        if not config.claims_trust:
            raise ConfigError(
                "a claims layer needs claims_trust: one that trusts nobody "
                "draws nothing, and looks exactly like one with nothing to "
                "draw")
        if config.claims_shoulder_m is None:
            raise ConfigError("a claims layer needs claims_shoulder_m")
        self.config = config
        topics = Topics(config.scene)
        self._edges = tt.make_reader(
            participant, topics.relationship, Relationship,
            MODEL_LATCHED.name)
        self._entities = tt.make_reader(
            participant, topics.entity, Entity, MODEL_LATCHED.name)
        self._poses = tt.make_reader(
            participant, topics.pose, ModelPose, MODEL_FAST.name)
        self._georeferences = GeoreferenceBook(participant, config)
        self._claims: Dict[str, _Claim] = {}
        self._subjects: Dict[str, _Subject] = {}
        self._handles: Dict[Tuple[str, int], str] = {}

    # --- reading ----------------------------------------------------------

    def poll(self) -> List[str]:
        """
        Drain the lanes. Returns one line per lifecycle change of a claim
        this layer could draw, for the log: a crash and a retirement must
        never read alike there either.
        """
        notes: List[str] = []
        for sample in tt.take_with_state(self._entities):
            self._on_entity(sample)
        for sample in tt.take_with_state(self._poses):
            self._on_pose(sample)
        for sample in tt.take_with_state(self._edges):
            note = self._on_edge(sample)
            if note:
                notes.append(note)
        self._georeferences.poll()
        return notes

    def _key(self, lane: str, sample) -> Optional[str]:
        handle = sample.instance_handle
        if sample.data is not None:
            key = (sample.data.id if lane == "edge"
                   else sample.data.entity_id)
            if handle is not None:
                self._handles[(lane, int(handle))] = key
            return key
        # A lifecycle notification carries no payload, so its key is
        # recovered from the instance it names.
        return (self._handles.get((lane, int(handle)))
                if handle is not None else None)

    def _on_entity(self, sample) -> None:
        key = self._key("entity", sample)
        if key is None:
            return
        subject = self._subjects.setdefault(key, _Subject())
        if sample.data is not None:
            subject.record = sample.data
            if sample.alive:
                subject.disposed = False
        if (not sample.alive and int(sample.instance_state)
                == int(InstanceState.NotAliveDisposed)):
            subject.disposed = True

    def _on_pose(self, sample) -> None:
        key = self._key("pose", sample)
        if key is None or sample.data is None or not sample.alive:
            return
        self._subjects.setdefault(key, _Subject()).pose = sample.data.pose

    def _on_edge(self, sample) -> Optional[str]:
        key = self._key("edge", sample)
        if key is None:
            return None
        held = self._claims.get(key)
        if sample.data is not None:
            edge = sample.data
            if edge.state == RelationshipLifecycle.RETIRED:
                # The terminal sample: the asserter withdrew it, and said
                # why. The dispose that follows has nothing left to remove.
                self._claims.pop(key, None)
                return (f"claim {key} retired by {edge.asserter}: "
                        f"{edge.state_reason or 'no reason given'}"
                        if self._admitted(edge) else None)
            came_back = held is not None and held.writer_gone
            self._claims[key] = _Claim(edge)
            if not sample.alive:
                self._lifecycle(key, sample)
            if came_back and self._admitted(edge):
                return f"claim {key}: its asserter {edge.asserter} is back"
            return None
        if held is None:
            return None
        return self._lifecycle(key, sample)

    def _lifecycle(self, key: str, sample) -> Optional[str]:
        held = self._claims.get(key)
        if held is None:
            return None
        state = int(sample.instance_state)
        admitted = self._admitted(held.edge)
        if state == int(InstanceState.NotAliveDisposed):
            # Disposed without a terminal sample: withdrawn, reason unknown.
            self._claims.pop(key, None)
            return (f"claim {key} withdrawn by {held.edge.asserter} (a "
                    f"dispose with no terminal sample)" if admitted else None)
        if state == int(InstanceState.NotAliveNoWriters):
            if held.writer_gone:
                return None
            held.writer_gone = True
            if not admitted:
                return None
            if held.edge.basis == Basis.OBSERVED:
                return (f"claim {key}: {held.edge.asserter}'s liveliness "
                        f"ended; an OBSERVED claim nobody is still making "
                        f"leaves the costmap. Kept, not retired")
            return (f"claim {key}: {held.edge.asserter}'s liveliness ended; "
                    f"a {held.edge.basis.name} claim stands until withdrawn")
        return None

    def _admitted(self, edge: Relationship) -> bool:
        basis = getattr(edge.basis, "name", str(edge.basis))
        return any(rule.admits(edge.asserter, basis, edge.predicate)
                   for rule in self.config.claims_trust)

    # --- deciding ---------------------------------------------------------

    def picture(self, now: float) -> ClaimsPicture:
        """What to draw now. `now` is this host's wall clock, which is the
        clock the prototype's writers stamp `valid_until` on."""
        out = ClaimsPicture()
        book, conflicts = self._georeferences.book()
        own = self.config.frame_id
        for claim_id, held in sorted(self._claims.items()):
            reason = self._why_not(held, now, out.counts)
            if reason is not None:
                if reason:
                    out.skipped[claim_id] = reason
                continue
            edge = held.edge
            subject = self._subjects.get(edge.subject_ref)
            footprint, z, why = self._footprint(edge.subject_ref, subject)
            if footprint is None:
                out.counts["no_region"] += 1
                out.skipped[claim_id] = why
                continue
            fqn = subject.record.frame_ref.fqn
            problem = frame_problem(subject.record.frame_ref) \
                or conflicts.get(fqn) \
                or (conflicts.get(own) if fqn != own else None)
            if problem:
                out.counts["unplaceable"] += 1
                out.skipped[claim_id] = problem
                continue
            polygon = []
            for x, y in footprint:
                p, why = compose_point([x, y, z], fqn, own, book)
                if p is None:
                    break
                polygon.append((p[0], p[1]))
            if len(polygon) != len(footprint):
                out.counts["unplaceable"] += 1
                out.skipped[claim_id] = why
                continue
            unasserted = held.writer_gone
            out.counts["drawn_unasserted" if unasserted else "drawn"] += 1
            out.regions.append(Region(
                claim_id, edge.subject_ref, edge.predicate, edge.asserter,
                edge.basis.name, tuple(polygon), unasserted))
        return out

    def _why_not(self, held: _Claim, now: float,
                 counts: Counter) -> Optional[str]:
        """`None` to draw; otherwise a reason. An empty one is counted only."""
        edge = held.edge
        if not self._admitted(edge):
            counts["untrusted"] += 1
            return (f"{edge.asserter}, {edge.basis.name}, {edge.predicate}: "
                    f"no trust rule admits it")
        if edge.has_valid_until and _seconds(edge.valid_until) < now:
            counts["expired"] += 1
            return (f"past its valid_until "
                    f"({now - _seconds(edge.valid_until):.1f}s ago)")
        if held.writer_gone and edge.basis == Basis.OBSERVED:
            counts["aged_out"] += 1
            return (f"{edge.asserter}'s liveliness ended: an observation "
                    f"nobody is still making is not about the present. "
                    f"Kept, not retired")
        subject = self._subjects.get(edge.subject_ref)
        if subject is None or subject.record is None:
            counts["no_subject"] += 1
            return f"its subject {edge.subject_ref} is not on the entity lane"
        if subject.disposed:
            counts["no_subject"] += 1
            return f"its subject {edge.subject_ref} was removed by its owner"
        if subject.record.state == LifecycleState.RETIRED:
            counts["no_subject"] += 1
            return f"its subject {edge.subject_ref} is RETIRED"
        return None

    @staticmethod
    def _footprint(subject_ref: str, subject: _Subject
                   ) -> Tuple[Optional[List[Tuple[float, float]]], float, str]:
        """
        The subject's footprint corners in its own frame, the height they sit
        at (which matters only when composing across frames), or why not.
        """
        record = subject.record
        props = {kv.key: kv.value for kv in record.properties or []}
        pose = subject.pose or (record.pose if record.has_pose else None)
        if OBB_LENGTH_KEY in props and OBB_WIDTH_KEY in props and pose:
            try:
                length = float(props[OBB_LENGTH_KEY])
                width = float(props[OBB_WIDTH_KEY])
            except ValueError:
                return None, 0.0, (
                    f"{subject_ref}'s oriented box is not two numbers: "
                    f"{props[OBB_LENGTH_KEY]!r}, {props[OBB_WIDTH_KEY]!r}")
            return oriented_box(pose.t[0], pose.t[1], _yaw(pose.q),
                                length, width), pose.t[2], ""
        if record.has_extent:
            lo, hi = record.extent.min_xyz, record.extent.max_xyz
            return [(lo[0], lo[1]), (hi[0], lo[1]),
                    (hi[0], hi[1]), (lo[0], hi[1])], lo[2], ""
        return None, 0.0, (f"its subject {subject_ref} publishes no extent: there "
                      f"is no region to draw, and none is guessed")


# --- the mask ---------------------------------------------------------------


@dataclass(frozen=True)
class Mask:
    """An OccupancyGrid's numbers: row-major from the origin corner."""
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int
    data: Tuple[int, ...]

    def value_at(self, x: float, y: float) -> Optional[int]:
        col = int(math.floor((x - self.origin_x) / self.resolution))
        row = int(math.floor((y - self.origin_y) / self.resolution))
        if 0 <= col < self.width and 0 <= row < self.height:
            return self.data[row * self.width + col]
        return None

    @property
    def marked(self) -> int:
        return sum(1 for v in self.data if v)


@dataclass(frozen=True)
class Grid:
    """A costmap's cells: where its origin is, how big, how fine."""
    origin_x: float
    origin_y: float
    resolution: float
    width: int
    height: int


class MaskCanvas:
    """
    Renders regions into a mask, on the costmap's own grid when it has one.

    **On the costmap's grid, or nav2 paints none of it** (`FINDINGS.md`,
    finding 14, measured on SpatialDDS-demo's tier). nav2 Humble's keep-out
    filter receives any mask, and repaints the window from the *costmap's*
    origin as wide as the *mask* is: it assumes the two are one grid, as a
    mask served beside a map is. A mask elsewhere, at any value, 100
    included, was received and never painted. So the node hands this canvas
    the costmap's grid (`align`), and every mask is exactly that grid.

    **A region that has gone must be drawn as gone.** The filter repaints
    only inside a mask it has been given, so a mask that shrank to the
    regions still standing would leave a dropped claim's cells where the last
    mask put them. On the costmap's grid that holds by construction. Without
    one (a consumer that is not nav2, and this library's tests) the canvas
    covers every region ever drawn, so a dropped one still goes back to free.
    No mask at all is rendered until the first region exists: "nothing has
    ever been claimed" needs no filter.
    """

    def __init__(self, shoulder_m: float, cost: int,
                 resolution: float = MASK_RESOLUTION_M) -> None:
        self.shoulder_m = float(shoulder_m)
        self.cost = int(cost)
        self.resolution = resolution
        self._bounds: Optional[Tuple[float, float, float, float]] = None
        self._grid: Optional[Grid] = None
        self._drawn = False

    def align(self, grid: Grid) -> bool:
        """Render on this grid from now on. Returns whether it changed."""
        if grid == self._grid:
            return False
        self._grid = grid
        return True

    def render(self, regions: Sequence[Region]) -> Optional[Mask]:
        self._drawn = self._drawn or bool(regions)
        if self._grid is not None:
            if not self._drawn:
                return None
            g = self._grid
            data = [0] * (g.width * g.height)
            for region in regions:
                self._paint(region.polygon, data, g.origin_x, g.origin_y,
                            g.width, g.height, g.resolution)
            return Mask(g.resolution, g.origin_x, g.origin_y, g.width,
                        g.height, tuple(data))
        for region in regions:
            self._grow(region.polygon)
        if self._bounds is None:
            return None
        r = self.resolution
        x0, y0, x1, y1 = self._bounds
        width = max(1, int(round((x1 - x0) / r)))
        height = max(1, int(round((y1 - y0) / r)))
        data = [0] * (width * height)
        for region in regions:
            self._paint(region.polygon, data, x0, y0, width, height, r)
        return Mask(r, x0, y0, width, height, tuple(data))

    def _grow(self, polygon) -> None:
        pad = self.shoulder_m + self.resolution
        r = self.resolution
        xs = [p[0] for p in polygon]
        ys = [p[1] for p in polygon]
        # Snapped to the cell, so the grid of a grown canvas still lines up
        # with every earlier one.
        lo_x = math.floor((min(xs) - pad) / r) * r
        lo_y = math.floor((min(ys) - pad) / r) * r
        hi_x = math.ceil((max(xs) + pad) / r) * r
        hi_y = math.ceil((max(ys) + pad) / r) * r
        if self._bounds is None:
            self._bounds = (lo_x, lo_y, hi_x, hi_y)
        else:
            a, b, c, d = self._bounds
            self._bounds = (min(a, lo_x), min(b, lo_y),
                            max(c, hi_x), max(d, hi_y))

    def _paint(self, polygon, data, x0, y0, width, height, r) -> None:
        pad = self.shoulder_m
        xs = [p[0] for p in polygon]
        ys = [p[1] for p in polygon]
        c0 = max(0, int(math.floor((min(xs) - pad - x0) / r)))
        c1 = min(width - 1, int(math.ceil((max(xs) + pad - x0) / r)))
        r0 = max(0, int(math.floor((min(ys) - pad - y0) / r)))
        r1 = min(height - 1, int(math.ceil((max(ys) + pad - y0) / r)))
        for row in range(r0, r1 + 1):
            cy = y0 + (row + 0.5) * r
            for col in range(c0, c1 + 1):
                cx = x0 + (col + 0.5) * r
                if (inside(polygon, cx, cy)
                        or edge_distance(polygon, cx, cy) <= pad):
                    data[row * width + col] = self.cost


# --- geometry ---------------------------------------------------------------


def oriented_box(cx: float, cy: float, yaw: float, length: float,
                 width: float) -> List[Tuple[float, float]]:
    """A box centred on (cx, cy), `length` along the heading `yaw`."""
    c, s = math.cos(yaw), math.sin(yaw)
    hl, hw = length / 2.0, width / 2.0
    return [(cx + c * dx - s * dy, cy + s * dx + c * dy)
            for dx, dy in ((-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw))]


def inside(polygon, x: float, y: float) -> bool:
    """Even-odd rule; a point on an edge counts as inside."""
    hit = False
    n = len(polygon)
    for i in range(n):
        (ax, ay), (bx, by) = polygon[i], polygon[(i + 1) % n]
        if (ay > y) != (by > y):
            if x < ax + (y - ay) * (bx - ax) / (by - ay):
                hit = not hit
    return hit


def edge_distance(polygon, x: float, y: float) -> float:
    best = math.inf
    n = len(polygon)
    for i in range(n):
        (ax, ay), (bx, by) = polygon[i], polygon[(i + 1) % n]
        dx, dy = bx - ax, by - ay
        span = dx * dx + dy * dy
        t = 0.0 if span == 0 else max(0.0, min(1.0, (
            (x - ax) * dx + (y - ay) * dy) / span))
        best = min(best, math.hypot(x - (ax + t * dx), y - (ay + t * dy)))
    return best


def _yaw(q) -> float:
    return math.atan2(2.0 * (q[3] * q[2] + q[0] * q[1]),
                      1.0 - 2.0 * (q[1] * q[1] + q[2] * q[2]))


def _seconds(t) -> float:
    return t.sec + t.nanosec / 1e9
