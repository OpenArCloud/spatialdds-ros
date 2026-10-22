"""
Awareness: the scene's other moving entities, placed in this robot's frame.

**No ROS in here,** for the adapter's reason: everything that decides what
becomes an obstacle, when it stops being one and where it goes is testable
with no ROS installation. `robot_node` turns the result into two point clouds
and a marker array; this produces the numbers. `docs/awareness-census.md` is
the reasoning behind every rule below, and it was ruled on 2026-10-21.

What is drawn: an entity on the scene's lanes that is `FAST`, `ACTIVE`, has a
pose, is not this robot, matches the type allowlist if there is one, has been
heard from within its own cadence, sits in a frame that composes into this
robot's, and is within range. Everything else is counted, and the ones a
reader could be surprised by carry a reason.

Three rules, each because the alternative is a false statement:

* **This robot never avoids itself.** A reader on the same participant as the
  adapter's writer receives that writer's samples (measured for the census),
  so the exclusion is a filter that does work, not a formality. It is on this
  robot's own `entity_id`, from the same `Config` — there is nothing to type,
  so nothing to get wrong.
* **A peer with no recent evidence is not drawn, and is not forgotten.** §2.14
  forbids purging a durable instance on liveliness loss, and that is about the
  record; a costmap is a decision about the present. So every record is kept
  and only projected while its poses keep arriving. Evidence is the pose lane
  and nothing else — a latched record says what a thing is, not that it is
  still there — and the window is `Tempo`'s, learned per peer from its own
  cadence and measured on receive time, so another machine's clock cannot make
  a peer look fresh.
* **A peer that cannot be placed is skipped, never defaulted.** Composition is
  `frames.compose_point`, the one place; when it has no answer, the peer is
  counted and its reason reported, and nothing is drawn at a guessed position.
"""
from __future__ import annotations

import math
import struct
import time
from collections import Counter
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

from cyclonedds.core import InstanceState
from cyclonedds.domain import DomainParticipant

from spatialdds_idl18.spatial.core import FrameTransform, PoseSE3
from spatialdds_idl18.spatial.owm import (
    Entity, LifecycleState, ModelLayer, ModelPose)

from spatialdds_ros import transport as tt
from spatialdds_ros.config import Config, ConfigError
from spatialdds_ros.frames import (
    compose_point, frame_problem, georeference)
from spatialdds_ros.qos import MODEL_FAST, MODEL_LATCHED
from spatialdds_ros.tempo import Tempo
from spatialdds_ros.topics import EARTH_FIXED_FQN, Topics

#: Spacing of the points that fill a footprint. Half nav2's usual 0.05 m cell
#: would be finer than any costmap reads; 0.05 m marks every cell of one.
FILL_STEP_M = 0.05

#: Spacing of the clearing ring. A ray must reach every cell inside the range,
#: so at the rim the gap between neighbouring rays has to stay under a cell.
RING_STEP_M = 0.05

#: How far inside the ring a drawn point must stay. nav2 clears the cells a
#: clearing ray passes through but not the cell it ends in, so a mark on the
#: ring itself would never clear. A quarter metre is several cells at any
#: resolution nav2 is normally run at.
RIM_M = 0.25

#: Two georeferences for one frame that disagree by more than this are a
#: conflict, and nothing in that frame is placed.
GEOREFERENCE_AGREEMENT_M = 0.01


@dataclass(frozen=True)
class Obstacle:
    """One peer, in this robot's frame."""
    entity_id: str
    x: float
    y: float
    #: The points that fill its footprint, (x, y) in this robot's frame.
    points: Tuple[Tuple[float, float], ...]
    label: str = ""


@dataclass
class Picture:
    """What the consumer would draw now, and why everything else is not."""
    obstacles: List[Obstacle] = field(default_factory=list)
    #: entity_id -> why it is not drawn, for the skips a reader needs told.
    skipped: Dict[str, str] = field(default_factory=dict)
    counts: Counter = field(default_factory=Counter)


@dataclass
class _Peer:
    record: Optional[Entity] = None
    pose: Optional[PoseSE3] = None
    heard: float = 0.0              # receive time of the last pose
    disposed: bool = False
    gone: str = ""                  # which lane's writer went away, if any


class Awareness:
    """Reads the scene's lanes and says what to draw."""

    def __init__(self, participant: DomainParticipant, config: Config) -> None:
        if not config.awareness_range_m or config.awareness_range_m <= RIM_M:
            raise ConfigError(
                f"awareness_range_m {config.awareness_range_m} leaves nothing "
                f"inside the clearing ring: drawn points stay {RIM_M} m short "
                f"of it, so the range must be larger than that")
        self.config = config
        topics = Topics(config.scene)
        self._entities = tt.make_reader(
            participant, topics.entity, Entity, MODEL_LATCHED.name)
        self._poses = tt.make_reader(
            participant, topics.pose, ModelPose, MODEL_FAST.name)
        self._frames = tt.make_reader(
            participant, topics.frame_transform, FrameTransform,
            MODEL_LATCHED.name)
        self._peers: Dict[str, _Peer] = {}
        self._handles: Dict[Tuple[str, int], str] = {}
        #: child fqn -> {transform_id: T_parent_child}
        self._georeferences: Dict[str, Dict[str, PoseSE3]] = {}
        self._frame_handles: Dict[int, Tuple[str, str]] = {}
        self._tempo = Tempo()

    # --- reading ----------------------------------------------------------

    def poll(self, now: Optional[float] = None) -> None:
        """Drain all three lanes. Receive time is `now`."""
        now = time.time() if now is None else now
        for sample in tt.take_with_state(self._entities):
            self._on_entity(sample)
        for sample in tt.take_with_state(self._poses):
            self._on_pose(sample, now)
        for sample in tt.take_with_state(self._frames):
            self._on_frame(sample)

    def _peer_for(self, lane: str, sample) -> Optional[Tuple[str, _Peer]]:
        handle = sample.instance_handle
        if sample.data is not None:
            entity_id = sample.data.entity_id
            if handle is not None:
                self._handles[(lane, int(handle))] = entity_id
        else:
            # A lifecycle notification carries no payload, so its key is
            # recovered from the instance it names.
            entity_id = (self._handles.get((lane, int(handle)))
                         if handle is not None else None)
            if entity_id is None:
                return None
        return entity_id, self._peers.setdefault(entity_id, _Peer())

    def _on_entity(self, sample) -> None:
        found = self._peer_for("entity", sample)
        if found is None:
            return
        _id, peer = found
        if sample.data is not None:
            peer.record = sample.data
            if sample.alive:
                peer.disposed, peer.gone = False, ""
        if not sample.alive:
            self._lifecycle(peer, sample, "entity")

    def _on_pose(self, sample, now: float) -> None:
        found = self._peer_for("pose", sample)
        if found is None:
            return
        entity_id, peer = found
        if sample.data is not None and sample.alive:
            peer.pose = sample.data.pose
            peer.heard = now
            if peer.gone == "pose":
                peer.gone = ""
            self._tempo.observed(entity_id, now)
        if not sample.alive:
            self._lifecycle(peer, sample, "pose")

    @staticmethod
    def _lifecycle(peer: _Peer, sample, lane: str) -> None:
        state = int(sample.instance_state)
        if state == int(InstanceState.NotAliveDisposed) and lane == "entity":
            peer.disposed = True
        elif state == int(InstanceState.NotAliveNoWriters):
            peer.gone = lane

    def _on_frame(self, sample) -> None:
        handle = sample.instance_handle
        if sample.data is not None:
            ft = sample.data
            if ft.parent_ref.fqn != EARTH_FIXED_FQN:
                return                  # only georeferences compose here
            child = ft.child_ref.fqn
            self._georeferences.setdefault(child, {})[ft.transform_id] = \
                ft.T_parent_child
            if handle is not None:
                self._frame_handles[int(handle)] = (child, ft.transform_id)
        if (not sample.alive and handle is not None
                and int(sample.instance_state)
                == int(InstanceState.NotAliveDisposed)):
            # A retracted georeference is a removal, §2.14. A writer that
            # merely left is not: a venue's claim about its frame stands.
            child, tid = self._frame_handles.get(int(handle), (None, None))
            if child is not None:
                self._georeferences.get(child, {}).pop(tid, None)

    # --- deciding ---------------------------------------------------------

    def georeferences(self) -> Tuple[Dict[str, PoseSE3], Dict[str, str]]:
        """
        One georeference per frame, and the frames that have a conflict.

        This robot's own anchor counts for its own frame when nothing on the
        bus does — normally its own adapter's transform arrives here anyway.
        """
        book: Dict[str, PoseSE3] = {}
        conflicts: Dict[str, str] = {}
        for child, claims in self._georeferences.items():
            poses = list(claims.values())
            if not poses:
                continue
            spread = max(math.dist(poses[0].t, p.t) for p in poses)
            if spread > GEOREFERENCE_AGREEMENT_M:
                conflicts[child] = (
                    f"{len(poses)} georeferences for {child!r} disagree by "
                    f"{spread:.2f} m")
            else:
                book[child] = poses[0]
        own = self.config.frame_id
        if own not in book and own not in conflicts and self.config.anchor:
            book[own] = georeference(own, self.config.anchor).T_parent_child
        return book, conflicts

    def picture(self, robot_xy: Optional[Tuple[float, float]],
                now: Optional[float] = None) -> Picture:
        """
        What to draw now. `robot_xy` is this robot's position in its frame,
        or `None` before it has one — in which case nothing is in range.
        """
        now = time.time() if now is None else now
        out = Picture()
        book, conflicts = self.georeferences()
        own = self.config.frame_id
        allow = set(self.config.awareness_types)
        for entity_id, peer in sorted(self._peers.items()):
            reason = self._why_not(entity_id, peer, now, allow, out.counts)
            if reason is not None:
                if reason:
                    out.skipped[entity_id] = reason
                continue
            record = peer.record
            fqn = record.frame_ref.fqn
            problem = frame_problem(record.frame_ref) or conflicts.get(fqn) \
                or (conflicts.get(own) if fqn != own else None)
            if problem:
                out.counts["unplaceable"] += 1
                out.skipped[entity_id] = problem
                continue
            centre, why = compose_point(peer.pose.t, fqn, own, book)
            if centre is None:
                out.counts["unplaceable"] += 1
                out.skipped[entity_id] = why
                continue
            points = self._footprint(record, peer.pose, fqn, own, book)
            if points is None:
                out.counts["unplaceable"] += 1
                out.skipped[entity_id] = why or "extent could not be placed"
                continue
            if robot_xy is None:
                out.counts["beyond_range"] += 1
                continue
            reach = self.config.awareness_range_m - RIM_M
            kept = tuple(p for p in points
                         if math.dist(p, robot_xy) <= reach)
            if not kept:
                out.counts["beyond_range"] += 1
                continue
            out.counts["drawn"] += 1
            out.obstacles.append(Obstacle(
                entity_id, centre[0], centre[1], kept,
                label=_label(record)))
        return out

    def _why_not(self, entity_id: str, peer: _Peer, now: float, allow: set,
                 counts: Counter) -> Optional[str]:
        """
        `None` to draw; otherwise a reason. An empty reason is counted but not
        reported — static scenery is not news.
        """
        if entity_id == self.config.entity_id:
            counts["self"] += 1
            return ""
        record = peer.record
        if record is None:
            counts["no_record"] += 1
            return ""
        if record.layer != ModelLayer.FAST:
            counts["not_fast"] += 1
            return ""
        if allow and not allow.intersection(record.type_uris or []):
            counts["not_allowed_type"] += 1
            return ""
        if peer.disposed:
            counts["disposed"] += 1
            return "disposed: removed by its owner"
        if peer.gone:
            counts["writer_gone"] += 1
            return (f"its {peer.gone} writer has gone; the record is kept, "
                    f"but a peer that died is not where it was")
        if record.state != LifecycleState.ACTIVE:
            counts["not_active"] += 1
            name = getattr(record.state, "name", str(record.state))
            return f"{name}: {record.state_reason or 'no reason given'}"
        if not record.has_pose or peer.pose is None:
            counts["no_pose"] += 1
            return "no pose heard yet"
        window = self._tempo.threshold(entity_id)
        quiet = now - peer.heard
        if quiet > window:
            counts["quiet"] += 1
            return (f"no pose for {quiet:.1f}s, past its {window:.1f}s "
                    f"window; the record is kept")
        return None

    def _footprint(self, record: Entity, pose: PoseSE3, fqn: str, own: str,
                   book) -> Optional[Tuple[Tuple[float, float], ...]]:
        """
        Points filling the peer's footprint, in this robot's frame.

        With an extent, its box — read as `SpatialZone` bounds are, in the
        entity's frame rather than about its pose, the same reading
        SpatialDDS-demo's consumer makes (the spec does not say; filed). With
        none, a disc of `peer_radius_m` about the pose.
        """
        if record.has_extent:
            lo, hi = record.extent.min_xyz, record.extent.max_xyz
            local = _fill_box(lo[0], lo[1], hi[0], hi[1])
            z = pose.t[2]
        else:
            r = self.config.peer_radius_m
            local = _fill_disc(pose.t[0], pose.t[1], r)
            z = pose.t[2]
        out = []
        for x, y in local:
            p, _why = compose_point([x, y, z], fqn, own, book)
            if p is None:
                return None
            out.append((p[0], p[1]))
        return tuple(out)


def _fill_disc(cx: float, cy: float, r: float) -> List[Tuple[float, float]]:
    k = max(1, int(math.ceil(r / FILL_STEP_M)))
    return [(cx + i * FILL_STEP_M, cy + j * FILL_STEP_M)
            for i in range(-k, k + 1) for j in range(-k, k + 1)
            if (i * FILL_STEP_M) ** 2 + (j * FILL_STEP_M) ** 2 <= r * r]


def _fill_box(x0: float, y0: float, x1: float,
              y1: float) -> List[Tuple[float, float]]:
    nx = max(1, int(math.ceil((x1 - x0) / FILL_STEP_M)))
    ny = max(1, int(math.ceil((y1 - y0) / FILL_STEP_M)))
    return [(x0 + (x1 - x0) * i / nx, y0 + (y1 - y0) * j / ny)
            for i in range(nx + 1) for j in range(ny + 1)]


def _label(record: Entity) -> str:
    for kv in record.properties or []:
        if kv.key == "label":
            return kv.value
    return record.entity_id


def to_base(points, robot_x: float, robot_y: float,
            robot_yaw: float) -> List[Tuple[float, float]]:
    """
    Frame points into the robot's base frame.

    The cloud is stamped in the base frame because nav2's obstacle layer
    raytraces from the cloud's frame origin: stamped in `map`, the clearing
    ring would clear from the map origin instead of from the robot.
    """
    c, s = math.cos(robot_yaw), math.sin(robot_yaw)
    out = []
    for x, y in points:
        dx, dy = x - robot_x, y - robot_y
        out.append((c * dx + s * dy, -s * dx + c * dy))
    return out


def clearing_ring(radius: float) -> List[Tuple[float, float]]:
    """
    The ring that clears what has gone, in the base frame.

    Measured for the census: a marking cloud that empties raytraces nothing,
    so its last marks stay forever; a clearing-only ring at the range beside
    it clears them on the next cycle and marks nothing itself.
    """
    n = max(8, int(math.ceil(2 * math.pi * radius / RING_STEP_M)))
    return [(radius * math.cos(2 * math.pi * k / n),
             radius * math.sin(2 * math.pi * k / n)) for k in range(n)]


#: Height the cloud's points sit at in the base frame: inside the obstacle
#: layer's default `min_obstacle_height` (0.0) to `max_obstacle_height` (2.0).
POINT_Z_M = 0.1


def pack_xyz(points, z: float = POINT_Z_M) -> bytes:
    """
    `PointCloud2` data for (x, y) points: little-endian float32 x, y, z, 12
    bytes a point. Packed here rather than through `sensor_msgs_py` so the
    bytes are testable with no ROS installed.
    """
    return b"".join(struct.pack("<fff", x, y, z) for x, y in points)
