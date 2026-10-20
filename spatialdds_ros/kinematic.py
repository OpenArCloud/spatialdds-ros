"""
The no-hardware mode: a robot that drives in straight lines.

This is the library's CI story and its try-it story, the same role the
prototype's `PlazaSim` played. It exists so the whole chain — entity on the
bus, a command accepted, a refusal reaching the asker — can be exercised with
no ROS, no nav2 and no robot.

**What it deliberately does not carry.** The prototype's sim tested every step
against a mask built from the venue's declared extents, and refused to enter.
That coupling does not travel: for a real robot the clearance is nav2's
footprint and inflation radius, which nav2 already owns, and the prototype's
own clearance constant is documented as tied to a GLB scaled up for on-screen
legibility rather than to a robot. The census rules the keep-out job as the
costmap-filter path, and this source is not it.

**What it carries instead, and why.** Optional `bounds`: an axis-aligned box
this sim will not leave. Default unbounded, so by default nothing is refused.
Bounds are not a world model and make no claim about a venue — they are a
property of the sim, the way a treadmill is a property of a test rig. They are
here because the *shape* of a real late refusal needs reproducing: whether a
goal is drivable depends on where the robot is when it meets the boundary, so
the refusal necessarily arrives after the acceptance. A source that could only
accept would leave that path unexercised in the one mode anybody can run
without hardware.
"""
from __future__ import annotations

import math
from typing import Optional, Tuple

#: Metres per second. Fast enough that a test does not wait, slow enough that
#: a person watching can see it move.
DEFAULT_SPEED_MPS = 0.6

#: How close counts as arrived.
ARRIVAL_M = 0.05


class Bounds:
    """An axis-aligned box the sim will not leave."""

    def __init__(self, min_x: float, min_y: float,
                 max_x: float, max_y: float) -> None:
        if min_x >= max_x or min_y >= max_y:
            raise ValueError(
                f"bounds are empty: x [{min_x}, {max_x}], y [{min_y}, {max_y}]")
        self.min_x, self.min_y = min_x, min_y
        self.max_x, self.max_y = max_x, max_y

    def contains(self, x: float, y: float) -> bool:
        return (self.min_x <= x <= self.max_x
                and self.min_y <= y <= self.max_y)

    def __repr__(self) -> str:                                # pragma: no cover
        return (f"Bounds(x [{self.min_x}, {self.max_x}], "
                f"y [{self.min_y}, {self.max_y}])")


class KinematicSource:
    """
    A pose that walks towards a goal in a straight line.

    No physics, no sensors, no world model. `step` returns the new pose; when
    it has had to give up on a goal it sets `abandoned` to the reason, which
    the caller is expected to pass to `RobotAdapter.refused` so whoever asked
    finds out.
    """

    def __init__(self, start: Tuple[float, float] = (0.0, 0.0),
                 yaw: float = 0.0, speed: float = DEFAULT_SPEED_MPS,
                 bounds: Optional[Bounds] = None) -> None:
        self.x, self.y = start
        self.yaw = yaw
        self.speed = speed
        self.bounds = bounds
        self.goal: Optional[Tuple[float, float]] = None
        #: Set when a goal was dropped, cleared by the caller once reported.
        self.abandoned: Optional[str] = None
        if bounds is not None and not bounds.contains(self.x, self.y):
            raise ValueError(
                f"the start position ({self.x}, {self.y}) is outside the "
                f"configured bounds, so this sim would be refusing from its "
                f"first step")

    def step(self, dt: float) -> Tuple[float, float, float]:
        """Advance by `dt` seconds. Returns (x, y, yaw)."""
        if self.goal is None:
            # Idle is *still*. A sim that drifted while idle would publish
            # motion nobody asked for, and the fast lane would carry it.
            return self.x, self.y, self.yaw

        gx, gy = self.goal
        dx, dy = gx - self.x, gy - self.y
        distance = math.hypot(dx, dy)
        if distance <= ARRIVAL_M:
            self.x, self.y, self.goal = gx, gy, None
            return self.x, self.y, self.yaw

        yaw = math.atan2(dy, dx)
        step = min(self.speed * dt, distance)
        x = self.x + step * math.cos(yaw)
        y = self.y + step * math.sin(yaw)

        if self.bounds is not None and not self.bounds.contains(x, y):
            # **The reason cannot be known at accept time**, which is the whole
            # point of this branch: it depends on where the robot is when it
            # meets the boundary. So the goal was honestly accepted and is
            # honestly given up on here, with a reason the asker can read.
            self.abandoned = (
                f"goal ({gx:.2f}, {gy:.2f}) abandoned at "
                f"({self.x:.2f}, {self.y:.2f}) — a straight line from here "
                f"leaves this source's configured bounds, and it drives "
                f"straight lines only rather than planning around them. Send "
                f"a waypoint inside the bounds, or run a real planner")
            self.goal = None
            return self.x, self.y, self.yaw

        self.yaw, self.x, self.y = yaw, x, y
        return self.x, self.y, self.yaw


class KinematicNavigator:
    """
    `RobotAdapter`'s navigator, backed by the sim above.

    `available()` is always `None`: there is nothing for a straight-line sim to
    be unavailable for, and claiming otherwise would make the adapter decline
    goals it can in fact take.
    """

    def __init__(self, source: KinematicSource) -> None:
        self._source = source

    def available(self) -> Optional[str]:
        return None

    def send(self, x: float, y: float) -> str:
        self._source.goal = (x, y)
        return f"the kinematic source is heading for ({x:.2f}, {y:.2f})"
