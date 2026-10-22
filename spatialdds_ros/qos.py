"""
The three QoS profiles this library's lanes use, from spec §3.3.3.

**Not configurable, and this is the clearest case in the library.** QoS in DDS
is a request/offered contract: endpoints match only when the reader's request
is compatible with the writer's offer, and when they do not match *nothing
happens*. No error, no warning, no mismatch reported — the reader simply never
receives, which is indistinguishable from a writer that is not publishing.

The prototype has the scar. Deadline is one of those request/offered policies,
so a reader requesting 33 ms does not match a writer offering none. An
independently-built probe transcribed the spec's older table, read its
"Typical" column heading as advisory, omitted the deadlines, and exchanged
nothing at all; `tests/test_interop.py::DeadlineIsLoadBearing` in
SpatialDDS-demo pins the behaviour. A dash in the spec's deadline column means
*no deadline*, which is a different statement from an unstated one.

So a QoS knob on this library would be a way for an operator to build a robot
that is invisible to the scene, with nothing anywhere saying why. There is one
right answer per lane and the library fills it.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

try:                                    # pragma: no cover - import shape
    from cyclonedds import qos as _qos, util as _util
    HAVE_CYCLONEDDS = True
except Exception:                       # pragma: no cover - import shape
    # Importable without CycloneDDS so the profile *table* can be read and
    # asserted on a machine with no DDS build — which is where this library's
    # unit tests run. `to_qos` is the only thing that needs the binding.
    _qos = _util = None                                     # type: ignore
    HAVE_CYCLONEDDS = False


@dataclass(frozen=True)
class QosProfile:
    """One row of §3.3.3, transcribed field for field."""

    name: str
    reliable: bool
    deadline_ms: Optional[int]
    latched: bool = False
    keep_last: Optional[int] = None          # None means KEEP_ALL
    note: str = ""

    def to_qos(self, lifespan_sec: Optional[float] = None, *,
               writer: bool = False):
        """This row as a CycloneDDS `Qos`."""
        if not HAVE_CYCLONEDDS:             # pragma: no cover - import shape
            raise RuntimeError(
                "cyclonedds is not importable, so no QoS can be built. The "
                "profile table itself is readable without it")
        policies = [
            _qos.Policy.Reliability.Reliable(_util.duration(seconds=1))
            if self.reliable else _qos.Policy.Reliability.BestEffort,
            _qos.Policy.Durability.TransientLocal if self.latched
            else _qos.Policy.Durability.Volatile,
            _qos.Policy.History.KeepLast(self.keep_last) if self.keep_last
            else _qos.Policy.History.KeepAll,
        ]
        if self.deadline_ms is not None:
            policies.append(
                _qos.Policy.Deadline(
                    _util.duration(milliseconds=self.deadline_ms)))
        if lifespan_sec:
            # A payload TTL (an announce's `ttl_sec`) mapped onto DDS Lifespan,
            # so the middleware expires a stale sample rather than every
            # consumer re-deriving staleness.
            policies.append(
                _qos.Policy.Lifespan(_util.duration(seconds=lifespan_sec)))
        if writer:
            # **A writer that goes away does not dispose what it wrote.**
            # CycloneDDS's default is to dispose a writer's instances when it
            # is deleted or its lease expires, so a crashed robot's record
            # reached every reader as NOT_ALIVE_DISPOSED — a deliberate
            # removal — ten seconds after the crash. Measured 2026-10-21 for
            # the awareness build, SIGKILL and SIGTERM alike. §2.14 is
            # explicit that liveliness loss is not removal, and here the
            # middleware was manufacturing the one from the other on our
            # behalf. Off, the same events arrive as NOT_ALIVE_NO_WRITERS,
            # and disposal means what it says: somebody chose it. Not a
            # request/offered policy, so no reader's matching changes.
            policies.append(_qos.Policy.WriterDataLifecycle(autodispose=False))
        return _qos.Qos(*policies)


#: What exists and what it is. TRANSIENT_LOCAL so a client that connects after
#: the robot has been running still learns the robot is there — the property
#: the whole model layer rests on, and the reason a late joiner needs no query.
MODEL_LATCHED = QosProfile(
    "MODEL_LATCHED", reliable=True, deadline_ms=None, latched=True,
    keep_last=1,
    note="RELIABLE + TRANSIENT_LOCAL + KEEP_LAST(1) per key",
)

#: Where it is now. BEST_EFFORT because a pose that missed its moment is worth
#: less than the next one; VOLATILE because a late joiner should read the
#: latched record rather than a pose from before it existed.
MODEL_FAST = QosProfile(
    "MODEL_FAST", reliable=False, deadline_ms=None, latched=False, keep_last=1,
    note="BEST_EFFORT + VOLATILE + KEEP_LAST(1) per key -- the tempo lane",
)

#: Requests. RELIABLE so one is not lost in transit, VOLATILE so a restarting
#: participant does not replay yesterday's goals at a robot that has moved.
MODEL_COMMAND = QosProfile(
    "MODEL_COMMAND", reliable=True, deadline_ms=None, latched=False,
    keep_last=16,
    note="RELIABLE + VOLATILE + KEEP_LAST(16), unkeyed -- a request, not a claim",
)

PROFILES = {p.name: p for p in (MODEL_LATCHED, MODEL_FAST, MODEL_COMMAND)}
