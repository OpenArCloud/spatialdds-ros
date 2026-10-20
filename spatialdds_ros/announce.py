"""
The robot's presence on the discovery lane.

A robot that publishes its pose and accepts goals is discoverable in exactly
the way a VPS or a catalogue is: something that wants to drive it should be
able to find it without being told its topic names. The prototype's robot does
*not* announce — discovery there belongs to the VPS and catalogue servers — so
this capability is new rather than extracted, and the catalogue server's
announce is the shape it follows.

**On `kind`.** 1.8's `ServiceKind` is VPS, MAPPING, RELOCAL, SEMANTICS,
STORAGE, CONTENT, ANCHOR_REGISTRY, SENSING, INFRASTRUCTURE, FUSION, OTHER.
None of them is an agent that reports where it is and accepts commands about
where to go. `SENSING` is the nearest and it overstates — it reads as a service
offering sensor feeds, which this is not. So the default is `OTHER`, which is
accurate, and `kind` is a parameter so an operator whose robot genuinely is one
of the others can say so.

That gap is filed rather than papered over: see `docs/spec-queue.md`. Choosing
`SENSING` because it is closest would make every robot on the bus advertise a
capability it does not have, and the next reader would have no way to know the
enum had been stretched.
"""
from __future__ import annotations

from typing import Optional

# **Resolved at their definitions, not through `spatial.disco`.** That module
# re-exports most of what an announce needs, and six of the names it re-exports
# are annotated typedef aliases that report zero members and raise
# `TypeError: 'ForwardRef' object is not callable` when constructed — Aabb3,
# FrameRef, Time, PoseSE3, SpatialUri and MetaKV among them. The convenient
# import is the wrong one. `test/test_types_resolve_at_definitions.py` is the
# gate; this trap has cost four separate debugging rounds across this
# programme, so it is enforced rather than remembered.
from spatialdds_idl18.builtin import Time
from spatialdds_idl18.spatial.common import FrameRef
from spatialdds_idl18.spatial.core import Aabb3, PoseSE3
from spatialdds_idl18.spatial.disco import (
    Announce, Capabilities, CoverageElement, ProfileSupport, ServiceKind,
    TopicMeta, Transform, ValidityWindow)

from spatialdds_ros.config import Config
from spatialdds_ros.frames import (
    earth_fixed_frame, enu_to_ecef_pose, now_time, robot_frame)
from spatialdds_ros.qos import MODEL_COMMAND, MODEL_FAST, MODEL_LATCHED
from spatialdds_ros.topics import Topics

#: An absent bbox still needs four numbers, because `BBox2D` is a fixed
#: `double[4]`. Paired with `has_bbox=False`, so nothing reads them.
_NO_BBOX = [0.0, 0.0, 0.0, 0.0]

#: How long an announce stands before a consumer should stop believing it.
#: Mapped onto DDS Lifespan by the writer, so the middleware expires it rather
#: than every consumer re-deriving staleness.
DEFAULT_TTL_SEC = 300

#: What this library speaks. Stated as a range because that is what
#: `ProfileSupport` is for, and narrow because claiming 1.7 would invite a 1.7
#: client to assume the four members 1.8 appended are absent.
PROFILES = (("spatial.core", 1, 8, 8), ("spatial.owm", 0, 1, 1))


def _topics(config: Config) -> list[TopicMeta]:
    """
    The lanes this robot reads and writes, with their QoS profiles named.

    `qos_profile` being advertised is what lets a consumer build a matching
    endpoint without guessing — and the names here are the same ones
    `qos.PROFILES` resolves, so the advertisement is true on the wire rather
    than decorative. `target_rate_hz` is 0.0 where there is no steady rate:
    0 means unspecified, which is a different statement from a slow rate.
    """
    names = Topics(config.scene)
    return [
        TopicMeta(name=names.entity, type="spatial.owm.Entity", version="v1",
                  qos_profile=MODEL_LATCHED.name, target_rate_hz=0.0,
                  max_chunk_bytes=0, durability="TRANSIENT_LOCAL"),
        TopicMeta(name=names.pose, type="spatial.owm.ModelPose", version="v1",
                  qos_profile=MODEL_FAST.name, target_rate_hz=0.0,
                  max_chunk_bytes=0, durability="VOLATILE"),
        TopicMeta(name=names.command, type="spatial.owm.ModelCommand",
                  version="v1", qos_profile=MODEL_COMMAND.name,
                  target_rate_hz=0.0, max_chunk_bytes=0,
                  durability="VOLATILE"),
    ]


def _coverage(config: Config) -> list[CoverageElement]:
    """
    Where this robot is useful.

    **With no anchor there is no geographic coverage to claim**, and a
    `_global` element would be a loud lie — a robot in one building
    advertising the planet. So the coverage is expressed in the robot's own
    frame instead: a frame-referenced element with no bbox, which says "within
    this frame" and nothing about the earth. That is the same honesty the
    missing `FrameTransform` observes, carried onto the discovery lane.

    With an anchor, a degenerate bbox at the anchor point: the robot moves, so
    any extent would be a guess about how far, and a point is the one claim
    that is true.
    """
    if config.anchor is None:
        return [CoverageElement(
            has_crs=False, crs="", has_bbox=False,
            bbox=_NO_BBOX,
            has_aabb=False,
            aabb=Aabb3(min_xyz=[0.0, 0.0, 0.0], max_xyz=[0.0, 0.0, 0.0]),
            _global=False,
            has_frame_ref=True, frame_ref=robot_frame(config.frame_id),
            has_coverage_window=False,
            coverage_window_start=Time(sec=0, nanosec=0),
            coverage_window_end=Time(sec=0, nanosec=0),
            has_circle=False, circle_center=[0.0, 0.0, 0.0],
            circle_radius_m=0.0)]
    a = config.anchor
    return [CoverageElement(
        has_crs=True, crs="EPSG:4326", has_bbox=True,
        # **[west, south, east, north]** — `BBox2D` is `double[4]`, not a
        # struct with named members, and the order is stated only in a comment
        # in the IDL (`discovery.idl:96`). Longitude first. Getting it wrong
        # produces a well-formed box somewhere else entirely, which is why the
        # order is written out here rather than inferred at the call site.
        bbox=[a.lon_deg, a.lat_deg, a.lon_deg, a.lat_deg],
        has_aabb=False,
        aabb=Aabb3(min_xyz=[0.0, 0.0, 0.0], max_xyz=[0.0, 0.0, 0.0]),
        _global=False,
        has_frame_ref=True, frame_ref=robot_frame(config.frame_id),
        has_coverage_window=False,
        coverage_window_start=Time(sec=0, nanosec=0),
        coverage_window_end=Time(sec=0, nanosec=0),
        has_circle=False, circle_center=[0.0, 0.0, 0.0],
        circle_radius_m=0.0)]


def _transforms(config: Config) -> list[Transform]:
    """
    The announce's own copy of the georeference, when there is one.

    Empty without an anchor, for the reason the frames lane is silent: there is
    nothing true to say. Note the spelling — `_from` is the *local* frame and
    `to` is earth-fixed, which is the opposite of `FrameTransform`'s
    parent/child roles while carrying identical numbers. Mapping by name
    rather than by role inverts it, which is why both ends are written out
    here rather than passed through a helper that could be read either way.
    """
    if config.anchor is None:
        return []
    t, q = enu_to_ecef_pose(config.anchor.lat_deg, config.anchor.lon_deg,
                            config.anchor.height_m)
    return [Transform(
        _from=robot_frame(config.frame_id),      # the local frame
        to=earth_fixed_frame(),                  # the global one
        pose=PoseSE3(t=t, q=q),
        stamp=now_time(),
        has_validity=False,
        # `_from` and `seconds`, not start/end — a window with a duration
        # rather than two instants. Unread while `has_validity` is false; a
        # georeference that expires would say the frame moved, which is not
        # what an anchor from a launch file means.
        validity=ValidityWindow(_from=Time(sec=0, nanosec=0), seconds=0))]


def announce_for(config: Config, kind: str = "OTHER",
                 ttl_sec: int = DEFAULT_TTL_SEC) -> Announce:
    """This robot, as something another participant can discover."""
    return Announce(
        service_id=config.effective_source_id,
        name=config.effective_label,
        kind=getattr(ServiceKind, kind),
        version="1.8",
        org="",
        hints=[],
        caps=Capabilities(
            supported_profiles=[
                ProfileSupport(name=n, major=major, min_minor=lo,
                               max_minor=hi)
                for n, major, lo, hi in PROFILES],
            preferred_profiles=["spatial.owm/0.1"],
            features=[]),
        topics=_topics(config),
        coverage=_coverage(config),
        coverage_frame_ref=(earth_fixed_frame() if config.anchor
                            else robot_frame(config.frame_id)),
        has_coverage_eval_time=False,
        coverage_eval_time=Time(sec=0, nanosec=0),
        transforms=_transforms(config),
        manifest_uri="",
        auth_hint="",
        stamp=now_time(),
        ttl_sec=ttl_sec,
        coverage_source_ids=[],
    )


#: The discovery lane is not scene-scoped — it is the one place a participant
#: looks before it knows what scenes exist.
ANNOUNCE_TOPIC = "spatialdds/discovery/announce/v1"


class Announcer:
    """
    Publishes the announce, and re-stamps it on a cadence.

    Re-stamped rather than written once: `ttl_sec` is mapped onto DDS Lifespan,
    so a sample that is never refreshed expires and the robot silently stops
    being discoverable while it is still running. The prototype learned the
    same thing — "Re-stamp the announce on refresh, not just the Lifespan".
    """

    def __init__(self, participant, config: Config, kind: str = "OTHER",
                 ttl_sec: int = DEFAULT_TTL_SEC) -> None:
        from spatialdds_ros import transport as tt
        self.config = config
        self.kind = kind
        self.ttl_sec = ttl_sec
        self._writer = tt.make_writer(
            participant, ANNOUNCE_TOPIC, Announce, MODEL_LATCHED.name,
            lifespan_sec=float(ttl_sec))

    def publish(self) -> str:
        announce = announce_for(self.config, self.kind, self.ttl_sec)
        self._writer.write(announce)
        return (f"announce: {announce.service_id} as {announce.kind.name}, "
                f"{len(announce.topics)} topics, ttl {announce.ttl_sec}s")

    @property
    def refresh_interval_s(self) -> float:
        """Comfortably inside the TTL, so one missed refresh is not an outage."""
        return max(1.0, self.ttl_sec / 3.0)
