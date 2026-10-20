"""
Frames, and the one piece of arithmetic that fails silently.

Two things live here: the `FrameRef` the robot stamps on every pose, and the
`FrameTransform` that says where that frame sits on the earth — published only
when an anchor is configured.

**The direction of the transform is the thing to get right.** `parent_ref` is
the *global* frame and `child_ref` the *local* one, with `T_parent_child`
mapping parent into child. An inverted transform still produces a well-formed
pose with plausible-looking numbers and places everything near the centre of
the earth: no exception, no malformed record, nothing a schema check would
catch. The prototype's `venue_georeference` carries the same warning, and for
the same reason — the 1.7 announce spells the two ends `from` and `to` with
`from` being the *local* frame, so anyone mapping by name rather than by role
inverts it.

The ENU↔ECEF projection is written once, here. Two copies would make a drift
between consumers look like a disagreement about where the venue is.
"""
from __future__ import annotations

import math
import time
import uuid
from typing import Optional, Tuple

from spatialdds_idl18.builtin import Time
from spatialdds_idl18.spatial.common import (
    CoordConvention, FrameRef, ScaleStatus)
from spatialdds_idl18.spatial.core import CovMatrix, FrameTransform, PoseSE3

from spatialdds_ros.config import Anchor
from spatialdds_ros.topics import EARTH_FIXED_FQN


def now_time() -> Time:
    now = time.time()
    return Time(sec=int(now), nanosec=int((now % 1) * 1e9))


def frame_uuid(fqn: str) -> str:
    """
    A frame's uuid, derived from its name.

    UUIDv5 of the fqn, which is how every other participant in this family
    derives it. Load-bearing rather than tidy: the scene, a catalogue and an
    announce all naming the same frame must produce the same uuid, or they
    describe three frames that happen to share a name.
    """
    return str(uuid.uuid5(uuid.NAMESPACE_URL, fqn))


def robot_frame(fqn: str) -> FrameRef:
    """
    The frame the robot's pose is expressed in.

    **The scale quartet is filled, and it is the one number this library
    asserts without measuring it.** REP-103 makes ROS metric, so
    `SCALE_DECLARED` with `meters_per_unit = 1.0` is a true statement about any
    conformant ROS robot rather than a guess — and `SCALE_DECLARED` rather than
    `SCALE_DERIVED` because nobody recovered it from a similarity transform;
    the units were metres to begin with.

    `has_scale = false` would also mean metric — the spec makes every pre-1.8
    publisher correct by construction — so these four members change nothing a
    consumer computes. They change what it has to assume. They are also
    exactly the four a 1.7 reader truncates away, which is the era boundary
    working as designed rather than a fault.
    """
    return FrameRef(
        uuid=frame_uuid(fqn),
        fqn=fqn,
        has_coord_convention=True,
        coord_convention=CoordConvention.ENU,
        has_scale=True,
        scale_status=ScaleStatus.SCALE_DECLARED,
        meters_per_unit=1.0,
        display_unit="m",
    )


def earth_fixed_frame() -> FrameRef:
    """
    The global frame a georeference is relative to.

    No coordinate convention: ECEF is not one of the conventions
    `CoordConvention` can name, so claiming one would be false. Saying nothing
    is the accurate answer, and `has_coord_convention = False` is how the type
    says it.
    """
    return FrameRef(
        uuid=frame_uuid(EARTH_FIXED_FQN),
        fqn=EARTH_FIXED_FQN,
        has_coord_convention=False,
        coord_convention=CoordConvention.ENU,     # unread while the flag is off
        has_scale=True,
        scale_status=ScaleStatus.SCALE_DECLARED,
        meters_per_unit=1.0,
        display_unit="m",
    )


def enu_to_ecef_pose(lat_deg: float, lon_deg: float,
                     height_m: float) -> Tuple[list, list]:
    """
    A local ENU frame's origin and basis, in earth-fixed metres.

    Returns ``(t, q)``: the translation is the frame origin's ECEF position and
    the quaternion (x, y, z, w) is the ENU basis at that origin, so the pose
    maps **from** the local frame **into** earth-fixed.

    WGS84. One implementation, because a difference between two consumers
    should come from their anchors and never from two copies of this drifting
    apart.
    """
    lat, lon = math.radians(lat_deg), math.radians(lon_deg)
    a, f = 6378137.0, 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    n = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
    t = [(n + height_m) * math.cos(lat) * math.cos(lon),
         (n + height_m) * math.cos(lat) * math.sin(lon),
         (n * (1.0 - e2) + height_m) * math.sin(lat)]

    # The ENU basis at (lat, lon), as columns: east, north, up.
    sl, cl = math.sin(lon), math.cos(lon)
    sp, cp = math.sin(lat), math.cos(lat)
    east = (-sl, cl, 0.0)
    north = (-sp * cl, -sp * sl, cp)
    up = (cp * cl, cp * sl, sp)
    m = ((east[0], north[0], up[0]),
         (east[1], north[1], up[1]),
         (east[2], north[2], up[2]))

    # Matrix to quaternion, branching on the trace for numerical conditioning.
    trace = m[0][0] + m[1][1] + m[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (m[2][1] - m[1][2]) / s
        y = (m[0][2] - m[2][0]) / s
        z = (m[1][0] - m[0][1]) / s
    elif m[0][0] > m[1][1] and m[0][0] > m[2][2]:
        s = math.sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2.0
        w = (m[2][1] - m[1][2]) / s
        x = 0.25 * s
        y = (m[0][1] + m[1][0]) / s
        z = (m[0][2] + m[2][0]) / s
    elif m[1][1] > m[2][2]:
        s = math.sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2.0
        w = (m[0][2] - m[2][0]) / s
        x = (m[0][1] + m[1][0]) / s
        y = 0.25 * s
        z = (m[1][2] + m[2][1]) / s
    else:
        s = math.sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2.0
        w = (m[1][0] - m[0][1]) / s
        x = (m[0][2] + m[2][0]) / s
        y = (m[1][2] + m[2][1]) / s
        z = 0.25 * s
    return t, [x, y, z, w]


def georeference(frame_id: str, anchor: Anchor) -> FrameTransform:
    """
    Where `frame_id`'s origin sits on the earth — the robot's own claim.

    Published only when an anchor is configured; `Config.publishes_georeference`
    is the gate, and its absence is a documented mode rather than a failure.

    **Direction, stated at the construction site because this is where it goes
    wrong.** `parent_ref` is earth-fixed, the global frame. `child_ref` is the
    local frame. Following the 1.7 announce's `from`/`to` spelling instead
    would invert it and place the frame near the centre of the earth with every
    number still looking plausible.

    `cov` is a union and has to say which arm it is using. `COV_NONE` is the
    honest answer for an anchor somebody typed into a launch file: there are no
    error bars on it, and there is no implicit absent value to leave it at.
    """
    t, q = enu_to_ecef_pose(anchor.lat_deg, anchor.lon_deg, anchor.height_m)
    return FrameTransform(
        transform_id=f"xf:{frame_uuid(frame_id)}",
        parent_ref=earth_fixed_frame(),
        child_ref=robot_frame(frame_id),
        T_parent_child=PoseSE3(t=t, q=q),
        stamp=now_time(),
        cov=CovMatrix(none=0),
    )


def yaw_quaternion(yaw: float) -> list:
    """A rotation about the frame's up axis, as (x, y, z, w)."""
    return [0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)]


def pose_at(x: float, y: float, z: float, yaw: float) -> PoseSE3:
    return PoseSE3(t=[x, y, z], q=yaw_quaternion(yaw))


def geodetic_of(t) -> Tuple[float, float, float]:
    """
    ECEF metres back to (lat, lon, height). The inverse of the translation
    above, provided so a test can compose the two and check the round trip —
    an inverted or mis-scaled transform is caught by composing it, not by
    reading it.
    """
    x, y, z = t[0], t[1], t[2]
    a, f = 6378137.0, 1.0 / 298.257223563
    e2 = f * (2.0 - f)
    lon = math.atan2(y, x)
    p = math.hypot(x, y)
    lat = math.atan2(z, p)
    for _ in range(8):
        n = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
        h = p / math.cos(lat) - n
        lat = math.atan2(z, p * (1.0 - e2 * n / (n + h)))
    n = a / math.sqrt(1.0 - e2 * math.sin(lat) ** 2)
    return math.degrees(lat), math.degrees(lon), p / math.cos(lat) - n
