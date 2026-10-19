"""
When to refresh a latched record, and when to call a feed silent.

**The derivation is the convention, not any of the numbers.** A fixed silence
threshold shorter than the update interval makes every gap between updates
look like a stop. Measured in the prototype: a one-second threshold against a
mover giving each duck a turn every 1.5 s produced 94 "it stopped moving"
flushes while nothing had stopped, and "the fast tier quietly became the slow
one" (`spatialdds_demo/tempo.py`).

That is why this is not a parameter. A robot's publish rate is whatever ROS
publishes at — 20 Hz from a simulator, 5 Hz from an AMCL, something else on
hardware under load — and a library cannot know it. So it is measured per key
and the threshold follows it.

No I/O and no DDS: this answers questions and the caller decides what to
write, which keeps the single-writer rule with whoever owns the instance and
keeps this testable with no bus.
"""
from __future__ import annotations

import time
from typing import Dict, List, Optional

#: A FAST entity's every update goes out on the pose lane; its latched record
#: is refreshed every Nth, so a late joiner is at most N-1 updates stale and
#: converges on the next pose.
LATCH_EVERY_N = 5

#: The floor under the idle threshold, and the multiple of an entity's own
#: observed interval taken as evidence it has stopped. A thing updating every
#: 100 ms is idle after a beat; a thing updating every ten seconds is not idle
#: after eleven.
IDLE_FLOOR_S = 1.0
IDLE_FACTOR = 3.0

#: How much of the observed interval survives each new measurement. Low enough
#: to follow a cadence change within a few updates, high enough that one late
#: sample does not convince it everything stopped.
GAP_SMOOTHING = 0.7


class Tempo:
    """Per-key bookkeeping for a fast lane."""

    def __init__(self, latch_every: int = LATCH_EVERY_N,
                 idle_floor: float = IDLE_FLOOR_S,
                 idle_factor: float = IDLE_FACTOR,
                 smoothing: float = GAP_SMOOTHING) -> None:
        self.latch_every = latch_every
        self.idle_floor = idle_floor
        self.idle_factor = idle_factor
        self.smoothing = smoothing
        self._pending: Dict[str, int] = {}
        self._last: Dict[str, float] = {}
        self._gap: Dict[str, float] = {}

    def observed(self, key: str, now: Optional[float] = None) -> bool:
        """
        Record an update. Returns whether the latched record is now due.

        The caller publishes the fast sample either way; this decides only
        about the expensive one.
        """
        now = time.time() if now is None else now
        previous = self._last.get(key)
        if previous is not None:
            gap = now - previous
            known = self._gap.get(key)
            self._gap[key] = (gap if known is None
                              else self.smoothing * known
                              + (1 - self.smoothing) * gap)
        self._last[key] = now

        pending = self._pending.get(key, 0) + 1
        if pending >= self.latch_every:
            self._pending[key] = 0
            return True
        self._pending[key] = pending
        return False

    def threshold(self, key: str) -> float:
        """How long without an update means this key has stopped."""
        gap = self._gap.get(key)
        if gap is None:
            # One update is not a cadence. Until there are two, the floor is
            # all there is to go on — which costs a slow feed one uneconomical
            # but correct flush while its cadence is learned.
            return self.idle_floor
        return max(self.idle_floor, self.idle_factor * gap)

    def idle(self, now: Optional[float] = None) -> List[str]:
        """
        Keys with pending changes that have gone quiet, marked clean.

        So calling it every tick republishes a still world exactly once rather
        than every tick.
        """
        now = time.time() if now is None else now
        due = []
        for key, pending in list(self._pending.items()):
            if pending == 0:
                continue
            if now - self._last.get(key, 0.0) < self.threshold(key):
                continue
            self._pending[key] = 0
            due.append(key)
        return due

    def forget(self, key: str) -> None:
        """Stop tracking a key — it retired, or was disposed."""
        self._pending.pop(key, None)
        self._last.pop(key, None)
        self._gap.pop(key, None)
