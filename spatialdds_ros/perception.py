"""
Perception: this robot as an asserter of what it sees.

The adapter publishes the robot's own record; this publishes the things the
robot tracks, as OBSERVED entities, and claims about them where a predicate
applies. **No ROS in here** and no detector: the caller hands it tracks, from
whatever the robot's own perception is, and in this library's tests from a
scripted source. The API is the deliverable (`docs/claims-census.md` §4).

The rules are the ones this library and the demo's asserters already keep,
each because the alternative is a false statement:

* **Ids are this publisher's.** A track is `ent:<robot>:<track>`, so two
  robots seeing one chair publish two records and neither overwrites the
  other; fusing them is a consumer's business. A claim's id is one asserter,
  one (subject, predicate, object), the demo's `claims.edge_id` rule, so a
  revised opinion lands on the same instance.
* **Liveliness is the writer's, and a crash is never a retirement.** Every
  writer here is made by `transport.make_writer`, so none disposes on crash
  (`qos.QosProfile.to_qos`, autodispose off). A killed robot's tracks and
  claims stand, and their readers see the writer go.
* **Silence is UNOBSERVED, not gone.** A track not refreshed within its own
  cadence (`Tempo`, as the adapter learns its robot's) is republished
  UNOBSERVED with a reason, and its claims retire, `subject unobserved`.
* **Retirement is an act, and says why.** `lost()` and `withdraw()` write a
  terminal RETIRED sample carrying the reason, then dispose. A claim's reason
  starts with one of the demo's three causes, so a consumer can tell "the
  world changed" from "somebody stopped saying it".
"""
from __future__ import annotations

import math
import time
import uuid
from dataclasses import dataclass, field
from typing import Dict, Iterable, List, Optional, Tuple

from cyclonedds.domain import DomainParticipant

from spatialdds_idl18.builtin import Time
from spatialdds_idl18.oarc_owm import (
    EvidenceRef, Relationship, RelationshipLifecycle)
from spatialdds_idl18.spatial.common import KV
from spatialdds_idl18.spatial.core import Aabb3, PoseSE3
from spatialdds_idl18.spatial.owm import (
    Basis, Entity, LifecycleState, ModelLayer, ModelPose)

from spatialdds_ros import transport as tt
from spatialdds_ros.claims import OBB_LENGTH_KEY, OBB_WIDTH_KEY
from spatialdds_ros.config import Config
from spatialdds_ros.frames import now_time, robot_frame, yaw_quaternion
from spatialdds_ros.qos import MODEL_FAST, MODEL_LATCHED
from spatialdds_ros.tempo import Tempo
from spatialdds_ros.topics import Topics

#: "physical object" (Q223557), for a track the caller does not type. It
#: says only that something occupies space there, which is all a tracker
#: that has not classified it knows.
DEFAULT_OBSTACLE_TYPE = "http://www.wikidata.org/entity/Q223557"

#: The three retirement causes, spelled as SpatialDDS-demo's asserters spell
#: them (`spatialdds_demo/claims.py`), because a consumer tells them apart by
#: the reason's prefix.
NO_LONGER_HOLDS = "no longer holds"
SUBJECT_UNOBSERVED = "subject unobserved"
ASSERTER_STOPPED = "asserter stopped"

_ZERO = Time(sec=0, nanosec=0)
_OBB_HEIGHT_KEY = "demo.obb.height_m"


def edge_id(asserter: str, subject: str, predicate: str, obj: str) -> str:
    """
    One asserter's opinion about one proposition. SpatialDDS-demo's
    `claims.edge_id`, byte for byte, so an id minted here and one minted
    there for the same proposition agree.
    """
    digest = uuid.uuid5(uuid.NAMESPACE_URL,
                        "\x1f".join((asserter, subject, predicate, obj)))
    return f"rel:{asserter.replace('/', '.')}:{digest.hex[:16]}"


@dataclass
class _Track:
    record: Entity
    last_seen: float
    last_pose_write: float
    pose_stamp: Time
    silent: bool = False


@dataclass
class _Edge:
    edge: Relationship
    subject: str
    #: When the evidence last showed it true, for `subject unobserved`.
    last_true: Time = field(default_factory=lambda: _ZERO)


class ObstaclePublisher:
    """Publishes this robot's tracks and its claims about them."""

    def __init__(self, participant: DomainParticipant, config: Config) -> None:
        self.config = config
        self.asserter = config.effective_source_id
        self.topics = Topics(config.scene)
        self._entities = tt.make_writer(
            participant, self.topics.entity, Entity, MODEL_LATCHED.name)
        self._poses = tt.make_writer(
            participant, self.topics.pose, ModelPose, MODEL_FAST.name)
        self._edges = tt.make_writer(
            participant, self.topics.relationship, Relationship,
            MODEL_LATCHED.name)
        self._tracks: Dict[str, _Track] = {}
        self._held: Dict[str, _Edge] = {}
        self._tempo = Tempo()
        tail = config.entity_id.split(":")[-1] or config.entity_id
        self._prefix = f"ent:{tail}:"

    @property
    def writers(self):
        """Every writer this publisher owns, for the autodispose audit."""
        return {"entity": self._entities, "pose": self._poses,
                "relationship": self._edges}

    def entity_id(self, track_id: str) -> str:
        return f"{self._prefix}{track_id}"

    # --- tracks -----------------------------------------------------------

    def observed(self, track_id: str, x: float, y: float, *,
                 yaw: float = 0.0, length: Optional[float] = None,
                 width: Optional[float] = None,
                 height: Optional[float] = None,
                 type_uri: str = DEFAULT_OBSTACLE_TYPE,
                 label: Optional[str] = None,
                 now: Optional[float] = None) -> str:
        """
        A sighting, in this robot's frame. Returns the track's entity id.

        With `length` and `width`, the record carries the box: the native
        `Aabb3` enclosing the rotated footprint (axis-aligned, so honestly
        inflated), and the oriented dimensions as the crossing scene's
        `demo.obb.*` meta, which this library's claims layer reads. Without
        them it carries no extent, and says nothing about its size.
        """
        now = time.time() if now is None else now
        eid = self.entity_id(track_id)
        z = self.config.ground_z
        pose = PoseSE3(t=[x, y, z], q=yaw_quaternion(yaw))
        props = [KV(key="label", value=label or track_id)]
        extent, has_extent = Aabb3(min_xyz=[0.0] * 3, max_xyz=[0.0] * 3), False
        if length is not None and width is not None:
            h = 0.0 if height is None else float(height)
            c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
            hx = c * length / 2.0 + s * width / 2.0
            hy = s * length / 2.0 + c * width / 2.0
            extent = Aabb3(min_xyz=[x - hx, y - hy, z],
                           max_xyz=[x + hx, y + hy, z + h])
            has_extent = True
            props += [KV(key=OBB_LENGTH_KEY, value=f"{length:g}"),
                      KV(key=OBB_WIDTH_KEY, value=f"{width:g}")]
            if height is not None:
                props.append(KV(key=_OBB_HEIGHT_KEY, value=f"{h:g}"))
        stamp = now_time()
        record = Entity(
            entity_id=eid, basis=Basis.OBSERVED, type_uris=[type_uri],
            layer=ModelLayer.FAST, frame_ref=robot_frame(self.config.frame_id),
            has_pose=True, pose=pose, has_extent=has_extent, extent=extent,
            properties=props, external_refs=[], content_refs=[],
            state=LifecycleState.ACTIVE, state_reason="",
            source_id=self.asserter, stamp=stamp)
        held = self._tracks.get(eid)
        changed = (held is None or held.silent
                   or held.record.has_extent != has_extent
                   or list(held.record.properties) != props
                   or list(held.record.type_uris) != [type_uri])
        self._poses.write(ModelPose(entity_id=eid, pose=pose,
                                    source_id=self.asserter, stamp=stamp))
        due = self._tempo.observed(eid, now)
        if held is None:
            held = _Track(record, now, now, stamp)
            self._tracks[eid] = held
        held.record, held.last_seen, held.silent = record, now, False
        held.last_pose_write, held.pose_stamp = now, stamp
        if changed or due:
            self._entities.write(record)
        for edge in self._held.values():
            if edge.subject == eid:
                edge.last_true = stamp
        return eid

    def lost(self, track_id: str, reason: str) -> int:
        """
        The tracker has given the track up. Its claims retire, `subject
        unobserved`, then the record does: a terminal RETIRED sample with
        `reason`, then the dispose. Returns how many claims retired.
        """
        eid = self.entity_id(track_id)
        held = self._tracks.pop(eid, None)
        if held is None:
            return 0
        n = self._retire_involving(eid, SUBJECT_UNOBSERVED, reason)
        self._terminal_entity(held.record, reason)
        self._tempo.forget(eid)
        return n

    def tick(self, now: Optional[float] = None) -> List[str]:
        """
        Housekeeping: a pose floor for live tracks, and UNOBSERVED for a track
        not refreshed within its own cadence. Returns lines for the log.
        """
        now = time.time() if now is None else now
        notes = []
        floor = 1.0 / self.config.pose_floor_hz
        for eid, held in self._tracks.items():
            if held.silent:
                continue
            quiet = now - held.last_seen
            if quiet > self._tempo.threshold(eid):
                held.silent = True
                reason = (f"no sighting by {self.asserter} for {quiet:.1f}s; "
                          f"this is where it was last seen, not where it is")
                held.record = _with_state(held.record,
                                          LifecycleState.UNOBSERVED, reason)
                self._entities.write(held.record)
                n = self._retire_involving(
                    eid, SUBJECT_UNOBSERVED, f"{eid} not seen for "
                    f"{quiet:.1f}s")
                self._tempo.forget(eid)
                notes.append(f"{eid} unobserved after {quiet:.1f}s"
                             + (f", {n} claim(s) retired" if n else ""))
                continue
            if now - held.last_pose_write >= floor:
                # Last pose again, stamped now: true for as long as the track
                # is not silent, and touching neither silence nor cadence.
                self._poses.write(ModelPose(
                    entity_id=eid, pose=held.record.pose,
                    source_id=self.asserter, stamp=now_time()))
                held.last_pose_write = now
        for eid in self._tempo.idle(now):
            held = self._tracks.get(eid)
            if held is not None and not held.silent:
                self._entities.write(held.record)
        return notes

    # --- claims -----------------------------------------------------------

    def claim(self, track_id: str, predicate: str, object_ref: str, *,
              confidence: Optional[float] = None,
              meta: Iterable[Tuple[str, str]] = ()) -> str:
        """
        Hold an OBSERVED claim about a live track. Returns its id.

        `derived_from` names the track's latest pose sample: the record that
        showed it, which is the reading of an OBSERVED edge's lineage the 0.2
        register's E13 leaves open, and the one that names what observed it.
        Holding an unchanged opinion again writes nothing.
        """
        eid = self.entity_id(track_id)
        held_track = self._tracks.get(eid)
        if held_track is None or held_track.silent:
            raise ValueError(
                f"{eid} is not a live track: a claim about something this "
                f"robot is not seeing is not an observation")
        rid = edge_id(self.asserter, eid, predicate, object_ref)
        edge = Relationship(
            id=rid, subject_ref=eid, predicate=predicate,
            object_ref=object_ref, basis=Basis.OBSERVED,
            asserter=self.asserter,
            has_confidence=confidence is not None,
            confidence=0.0 if confidence is None else float(confidence),
            derived_from=[EvidenceRef(topic=self.topics.pose, key=eid,
                                      stamp=held_track.pose_stamp)],
            stamp=now_time(), has_valid_from=False, valid_from=_ZERO,
            has_valid_until=False, valid_until=_ZERO,
            meta=[KV(key=k, value=v) for k, v in meta],
            state=RelationshipLifecycle.CREATED, state_reason="")
        prior = self._held.get(rid)
        if prior is not None:
            if (prior.edge.has_confidence == edge.has_confidence
                    and prior.edge.confidence == edge.confidence
                    and list(prior.edge.meta) == list(edge.meta)):
                return rid
            edge = _replace(edge, state=RelationshipLifecycle.REVISED,
                            derived_from=list(prior.edge.derived_from))
        self._edges.write(edge)
        self._held[rid] = _Edge(edge, eid, held_track.pose_stamp)
        return rid

    def withdraw(self, claim_id: str, detail: str,
                 cause: str = NO_LONGER_HOLDS) -> bool:
        """
        Retire a held claim: RETIRED, with `<cause>: <detail>`, then dispose.

        `no longer holds` sets `valid_until` to now, when the evidence showed
        it; `subject unobserved` to the last evidence that showed it true; and
        `asserter stopped` leaves it unset, because the world was not
        consulted.
        """
        held = self._held.pop(claim_id, None)
        if held is None:
            return False
        until = {NO_LONGER_HOLDS: now_time(),
                 SUBJECT_UNOBSERVED: held.last_true}.get(cause)
        terminal = _replace(
            held.edge, state=RelationshipLifecycle.RETIRED,
            state_reason=f"{cause}: {detail}", stamp=now_time(),
            has_valid_until=until is not None,
            valid_until=until if until is not None else _ZERO)
        self._edges.write(terminal)
        self._edges.dispose(terminal)
        return True

    def close(self, why: str = "clean stop") -> None:
        """
        A clean stop withdraws everything held, as the demo's asserters'
        `on_exit="withdraw"` does: claims `asserter stopped`, then the tracks.
        A killed process never gets here, and that is the point: its records
        stand, and its readers see the writer go.
        """
        for rid in list(self._held):
            self.withdraw(rid, why, ASSERTER_STOPPED)
        for eid, held in list(self._tracks.items()):
            self._terminal_entity(held.record, f"{ASSERTER_STOPPED}: {why}")
        self._tracks.clear()

    def _retire_involving(self, eid: str, cause: str, detail: str) -> int:
        n = 0
        for rid, held in list(self._held.items()):
            if eid in (held.edge.subject_ref, held.edge.object_ref):
                self.withdraw(rid, detail, cause)
                n += 1
        return n

    def _terminal_entity(self, record: Entity, reason: str) -> None:
        terminal = _with_state(record, LifecycleState.RETIRED, reason)
        self._entities.write(terminal)
        self._entities.dispose(terminal)


def _with_state(record: Entity, state: LifecycleState, reason: str) -> Entity:
    return _replace(record, state=state, state_reason=reason,
                    stamp=now_time())


def _replace(sample, **changes):
    fields = {name: getattr(sample, name) for name in sample.__dataclass_fields__}
    fields.update(changes)
    return type(sample)(**fields)
