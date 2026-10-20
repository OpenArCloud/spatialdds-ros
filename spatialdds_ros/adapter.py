"""
The adapter: one robot, published as a `spatial.owm` entity.

**No ROS in here.** It is handed poses and handed a navigator. On a robot the
caller feeds it the map-frame pose and gives it a nav2 action client; in the
no-hardware mode the caller feeds it a kinematic sim and gives it that sim's
goal setter. Both call the same methods, which keeps the interesting behaviour
— cadence, latching, going quiet, declining, refusing — testable with no ROS
installation and no robot. That is this library's CI story, and the prototype's
`RobotBridge` is the shape it follows.

It owns exactly one key and writes nothing else. The pose-lane rule is **one
writer per key**, and `already_published` is the one place two processes could
plausibly break it.

Three things it will not do, each because the alternative is a false
statement:

* **It does not dispose its entity when the feed goes quiet.** Disposal says
  the robot is gone; silence only licenses "this is where it was last seen,
  not where it is". So the record is republished `UNOBSERVED` with a reason.
* **It does not accept a goal it knows nothing will act on.** A `goto` with no
  pose publishing, or with the navigator refusing goals, is declined with the
  reason — because a requester watching a motionless robot cannot tell a
  declined goal from an ignored one.
* **It does not let an accepted goal fail silently.** Whether a goal is
  drivable can depend on where the robot is when it meets a boundary, so the
  refusal arrives after the acceptance — on the latched record, where whoever
  asked will see it.
"""
from __future__ import annotations

import time
from typing import List, Optional, Protocol, Tuple

from cyclonedds.domain import DomainParticipant

from spatialdds_idl18.spatial.common import KV
from spatialdds_idl18.spatial.core import Aabb3, PoseSE3
from spatialdds_idl18.spatial.owm import (
    Basis, Entity, LifecycleState, ModelCommand, ModelLayer, ModelPose)
from spatialdds_idl18.spatial.core import FrameTransform

from spatialdds_ros import transport as tt
from spatialdds_ros.config import Config
from spatialdds_ros.frames import georeference, now_time, robot_frame
from spatialdds_ros.qos import MODEL_COMMAND, MODEL_FAST, MODEL_LATCHED
from spatialdds_ros.tempo import Tempo
from spatialdds_ros.topics import Topics

#: The property key a late refusal travels on. `demo.nav_status` in the
#: prototype; `nav.status` here, because this is no longer a demo's private
#: note but the library's own documented surface.
NAV_STATUS_KEY = "nav.status"


class Navigator(Protocol):
    """Whatever actually drives the robot."""

    def available(self) -> Optional[str]:
        """`None` when it will accept goals; otherwise one plain reason."""

    def send(self, x: float, y: float) -> str:
        """Ask for a goal. Returns what was *accepted*, for the log."""


class RobotAdapter:
    """Owns one entity on the scene's lanes."""

    def __init__(self, participant: DomainParticipant, config: Config,
                 navigator: Optional[Navigator] = None) -> None:
        self.config = config
        self.topics = Topics(config.scene)
        self._entities = tt.make_writer(
            participant, self.topics.entity, Entity, MODEL_LATCHED.name)
        self._poses = tt.make_writer(
            participant, self.topics.pose, ModelPose, MODEL_FAST.name)
        self._commands = tt.make_reader(
            participant, self.topics.command, ModelCommand, MODEL_COMMAND.name)
        # Created only when there is something to say. A writer on the frames
        # lane that never writes is indistinguishable, to a reader, from one
        # about to — so with no anchor there is no writer at all.
        self._frames = (
            tt.make_writer(participant, self.topics.frame_transform,
                           FrameTransform, MODEL_LATCHED.name)
            if config.publishes_georeference else None)

        self._tempo = Tempo()
        self._entity: Optional[Entity] = None
        self._goal: Optional[Tuple[float, float]] = None
        self._silent = False
        self._last_seen = 0.0
        self._navigator = navigator
        # **The last thing that happened to a goal, held until something
        # changes it.** A refusal published once and then superseded by the
        # next pose is an *event*, and an event can be missed: a client that
        # polls the latched record a moment later sees a robot sitting still
        # with nothing to explain it, which is the exact silence this whole
        # path exists to end. "What became of the goal I sent" is state, and a
        # latched record is where state belongs. Cleared when a new goal is
        # accepted, because that supersedes the old outcome.
        self._nav_status: Optional[str] = None

    # --- what it publishes ------------------------------------------------

    def _record(self, pose: PoseSE3, state: LifecycleState,
                reason: str = "") -> Entity:
        properties = [KV(key="label", value=self.config.effective_label)]
        if self._nav_status:
            properties.append(KV(key=NAV_STATUS_KEY, value=self._nav_status))
        return Entity(
            entity_id=self.config.entity_id,
            # OBSERVED, and not configurable. A self-localizing robot is
            # reporting where it has worked out that it is, from sensors —
            # a sighting rather than an assertion, and not DERIVED because
            # nothing here fuses anyone else's claims.
            basis=Basis.OBSERVED,
            type_uris=[self.config.type_uri],
            layer=ModelLayer.FAST,
            frame_ref=robot_frame(self.config.frame_id),
            has_pose=True,
            pose=pose,
            # A robot locates itself; it does not legislate a region. An
            # extent here would be read as keep-out by anything applying the
            # DECLARED rule, which is not what a robot's footprint means.
            has_extent=False,
            extent=Aabb3(min_xyz=[0.0, 0.0, 0.0], max_xyz=[0.0, 0.0, 0.0]),
            properties=properties,
            # Only what the operator configured. An adapter that invented a
            # reference would be asserting something about the world on
            # nobody's authority; an adapter with nowhere to put one cannot be
            # a consumer's only publisher of its robot, which is what blocked
            # the first adoption. Empty is a published fact, not an omission.
            external_refs=[KV(key=k, value=v)
                           for k, v in self.config.external_refs],
            content_refs=list(self.config.content_refs),
            state=state,
            state_reason=reason,
            source_id=self.config.effective_source_id,
            stamp=now_time(),
        )

    def publish_georeference(self) -> Optional[str]:
        """
        Where this robot's frame sits on the earth, if an anchor was given.

        Returns the line to log, or `None` when there is no anchor — which is
        a documented mode and not a failure. `Config.startup_notes()` is where
        that absence is explained to an operator; this just does not write.
        """
        if self._frames is None or self.config.anchor is None:
            return None
        transform = georeference(self.config.frame_id, self.config.anchor)
        self._frames.write(transform)
        return (f"georeference: {self.config.frame_id} -> "
                f"{transform.parent_ref.fqn}")

    def observed(self, x: float, y: float, yaw: float,
                 now: Optional[float] = None) -> None:
        """
        A new map-frame pose from upstream. Fast lane always; latch on cadence.

        `z` comes from the configured ground height rather than from the
        caller: a planar robot's upstream pose often carries a z of zero in its
        own frame, and the scene's floor is a property of the scene.
        """
        now = time.time() if now is None else now
        pose = PoseSE3(t=[x, y, self.config.ground_z],
                       q=_yaw_quaternion(yaw))
        first = self._entity is None
        was_silent = self._silent
        self._entity = self._record(pose, LifecycleState.ACTIVE)
        self._last_seen = now
        self._silent = False

        self._poses.write(ModelPose(
            entity_id=self.config.entity_id, pose=pose,
            source_id=self.config.effective_source_id,
            stamp=self._entity.stamp))

        # The expensive record goes on the cadence — and unconditionally when
        # the robot first appears or comes back, because those are changes of
        # *state* rather than of position, and a client waiting four more poses
        # to learn the robot is alive again would be told late.
        if (self._tempo.observed(self.config.entity_id, now)
                or first or was_silent):
            self._entities.write(self._entity)

    def refused(self, why: str) -> None:
        """
        Republish the latched record carrying a refusal, now.

        Off the cadence deliberately: this is a change of state, and the point
        is that the client which asked learns promptly rather than up to four
        poses later. It then *stays* on the record until a new goal is
        accepted — see `_nav_status`. A no-op before the first pose: there is
        no record to amend, and announcing a robot by complaining about it is
        not an introduction.
        """
        if self._entity is None:
            return
        self._nav_status = why
        self._entity = self._record(self._entity.pose, LifecycleState.ACTIVE)
        self._entities.write(self._entity)

    # --- what it is asked to do -------------------------------------------

    def handle_command(self, command: ModelCommand) -> Optional[str]:
        """
        `goto`, and nothing else. A line for the log, or `None` when the
        command was not ours to answer.

        Silence for somebody else's subject: the lane has several owners and a
        refusal from the wrong one reads as the system disagreeing with itself.
        A malformed command *about this robot* is a different matter and gets
        a reason.
        """
        if command.subject_id != self.config.entity_id:
            return None
        if command.verb != "goto":
            return None
        if not command.has_pose:
            # A zeroed pose is a well-formed request to drive to the frame
            # origin, so declining is the only reading that cannot be mistaken
            # for obedience.
            return (f"declined: goto for {self.config.entity_id} carried no "
                    f"pose")
        if self._entity is None or self._silent:
            return (f"declined: goto for {self.config.entity_id} — no pose "
                    f"from {self.config.effective_source_id}, so there is "
                    f"nothing to navigate from")
        if self._navigator is not None:
            unavailable = self._navigator.available()
            if unavailable:
                return (f"declined: goto for {self.config.entity_id} — "
                        f"{unavailable}")

        x, y = command.pose.t[0], command.pose.t[1]
        self._goal = (x, y)
        # A new goal supersedes whatever became of the last one, so the record
        # should stop reporting it. Cleared here rather than on arrival, so a
        # *declined* command leaves the previous outcome standing — the
        # requester still needs to see it.
        self._nav_status = None
        if self._navigator is None:
            return (f"accepted goto ({x:.2f}, {y:.2f}) "
                    f"(asked by {command.requester_id})")
        # What the navigator accepted, not what was sent. If it rounds, clamps
        # or refuses, that is what belongs in the log.
        outcome = self._navigator.send(x, y)
        return f"{outcome} (asked by {command.requester_id})"

    def poll_commands(self) -> List[str]:
        """Drain the command lane. The goal is already set; the caller decides
        what to do with the notes."""
        notes = []
        try:
            batch = tt.take_samples(self._commands)
        except Exception as error:                            # noqa: BLE001
            # Untrusted traffic, like every other reader here.
            return [f"command lane read failed: {error!r}"]
        for command in batch:
            note = self.handle_command(command)
            if note:
                notes.append(note)
        return notes

    def tick(self, now: Optional[float] = None) -> Optional[str]:
        """
        Housekeeping between poses: flush the latch when the robot stops
        moving, and declare it UNOBSERVED when we stop hearing from it at all.
        """
        now = time.time() if now is None else now
        if self._entity is None:
            return None

        if not self._silent and now - self._last_seen > self.silence_threshold():
            self._silent = True
            quiet_for = now - self._last_seen
            self._entity = self._record(
                self._entity.pose, LifecycleState.UNOBSERVED,
                f"no pose from {self.config.effective_source_id} for "
                f"{quiet_for:.1f}s; this is where it was last seen, not where "
                f"it is")
            self._entities.write(self._entity)
            self._tempo.forget(self.config.entity_id)
            return f"unobserved after {quiet_for:.1f}s of silence"

        if not self._silent and self._tempo.idle(now):
            self._entities.write(self._entity)
            return "latch flushed — it stopped moving"
        return None

    def silence_threshold(self) -> float:
        return self._tempo.threshold(self.config.entity_id)

    @property
    def goal(self) -> Optional[Tuple[float, float]]:
        return self._goal

    def clear_goal(self) -> None:
        self._goal = None

    @property
    def state(self) -> str:
        return "UNOBSERVED" if self._silent else "ACTIVE"


def _yaw_quaternion(yaw: float) -> list:
    from spatialdds_ros.frames import yaw_quaternion
    return yaw_quaternion(yaw)


def already_published(participant: DomainParticipant, config: Config,
                      settle: float = 2.5) -> Optional[str]:
    """
    Is somebody else already publishing this entity? Returns their source id.

    One writer per key is the rule the whole model rests on, and two adapters
    on one robot would put it in two places with nothing anywhere reporting a
    conflict — the readers would simply see a robot that teleports. The caller
    is expected to refuse to start.

    `settle` is how long to wait for a latched record to arrive. TRANSIENT_LOCAL
    means an existing publisher's record is delivered on connect, so this is
    bounded discovery rather than a poll of an unknown duration.
    """
    reader = tt.make_reader(participant, Topics(config.scene).entity, Entity,
                            MODEL_LATCHED.name)
    deadline = time.time() + settle
    while time.time() < deadline:
        for sample in tt.take_samples(reader):
            if sample.entity_id == config.entity_id:
                return sample.source_id
        time.sleep(0.05)
    return None
