"""
The no-hardware mode, wired up and runnable.

    python3 -m spatialdds_ros.standalone --scene bare-scene \
        --entity-id ent:robot:r1 --frame-id map/bare-scene

A robot on the bus in one command, with no ROS, no nav2 and no hardware. It is
the try-it story, and it is also the harness the acceptance run drives — the
same code path a person follows from the README, rather than a test-only
scaffold that could pass while the documented path was broken.

Everything here is wiring. The behaviour lives in `RobotAdapter` (what is
published, what is declined, what is refused), `KinematicSource` (how it
moves) and `Announcer` (how it is found).
"""
from __future__ import annotations

import argparse
import signal
import sys
import time
from typing import Optional

from cyclonedds.domain import DomainParticipant

from spatialdds_ros.adapter import RobotAdapter, already_published
from spatialdds_ros.announce import Announcer
from spatialdds_ros.config import Anchor, Config, ConfigError
from spatialdds_ros.kinematic import (
    Bounds, KinematicNavigator, KinematicSource)

DEFAULT_RATE_HZ = 10.0


class Runner:
    """
    One robot, driven by the kinematic source, for as long as it is stepped.

    Split from `main` so the acceptance test drives exactly what a person
    running the command drives — `main` only parses arguments and loops.
    """

    def __init__(self, config: Config, *, rate_hz: float = DEFAULT_RATE_HZ,
                 start: tuple = (0.0, 0.0), speed: Optional[float] = None,
                 bounds: Optional[Bounds] = None,
                 announce_kind: str = "OTHER") -> None:
        self.config = config
        self.rate_hz = rate_hz
        self.interval = 1.0 / rate_hz
        self.participant = DomainParticipant(config.domain)

        existing = already_published(self.participant, config)
        if existing is not None:
            raise SystemExit(
                f"{config.entity_id} is already being published by "
                f"{existing}. Refusing to start: one writer per key, and two "
                f"of us would put one robot in two places with nothing "
                f"reporting the conflict")

        self.source = KinematicSource(
            start=start, speed=speed or 0.6, bounds=bounds)
        self.adapter = RobotAdapter(
            self.participant, config,
            navigator=KinematicNavigator(self.source))
        self.announcer = Announcer(self.participant, config, announce_kind)
        self._announced_at = 0.0
        self.notes: list[str] = []

    def start(self) -> list[str]:
        """Say what mode this is, announce, and publish the georeference."""
        notes = list(self.config.startup_notes())
        notes.append(self.announcer.publish())
        self._announced_at = time.monotonic()
        georeference = self.adapter.publish_georeference()
        if georeference:
            notes.append(georeference)
        self.notes.extend(notes)
        return notes

    def step(self, dt: Optional[float] = None) -> list[str]:
        """One iteration. Returns whatever is worth logging."""
        dt = self.interval if dt is None else dt
        notes = list(self.adapter.poll_commands())

        x, y, yaw = self.source.step(dt)
        abandoned = self.source.abandoned
        if abandoned:
            self.source.abandoned = None
        self.adapter.observed(x, y, yaw)
        if abandoned:
            # After `observed`, so the amended record carries the current pose
            # rather than being overwritten by it on the next line.
            self.adapter.refused(abandoned)
            notes.append(abandoned)

        note = self.adapter.tick()
        if note:
            notes.append(note)

        # Re-announce well inside the TTL, so one missed refresh is not an
        # outage.
        if (time.monotonic() - self._announced_at
                > self.announcer.refresh_interval_s):
            notes.append(self.announcer.publish())
            self._announced_at = time.monotonic()

        self.notes.extend(notes)
        return notes

    def run(self) -> int:                                     # pragma: no cover
        stop = False

        def _stop(_signum, _frame):
            nonlocal stop
            stop = True

        signal.signal(signal.SIGINT, _stop)
        signal.signal(signal.SIGTERM, _stop)

        for note in self.start():
            print(f"robot: {note}", flush=True)
        reported = 0
        while not stop:
            for note in self.step():
                print(f"robot: {note}", flush=True)
            reported += 1
            if reported % int(self.rate_hz * 20) == 0:
                print(f"robot: {reported} poses; at "
                      f"({self.source.x:.2f}, {self.source.y:.2f})", flush=True)
            time.sleep(self.interval)
        print("robot: stopping.", flush=True)
        note = self.stop()
        if note:
            print(f"robot: {note}", flush=True)
        return 0

    def stop(self) -> Optional[str]:
        """Leave gracefully: the announce is disposed, the entity is not —
        the robot is not gone, this process is."""
        return self.announcer.depart()


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Put a kinematic robot on a SpatialDDS bus, with no ROS")
    # The three with no default, per the ruling. argparse does not mark them
    # required: `Config` is the single place that decides, and its refusal
    # explains *why* each has no default, which `argparse` cannot.
    p.add_argument("--scene", default="")
    p.add_argument("--entity-id", default="")
    p.add_argument("--frame-id", default="")
    p.add_argument("--source-id", default="")
    p.add_argument("--label", default="")
    p.add_argument("--domain", type=int, default=0)
    p.add_argument("--ground-z", type=float, default=0.0)
    p.add_argument("--rate-hz", type=float, default=DEFAULT_RATE_HZ)
    p.add_argument("--speed", type=float, default=0.6)
    p.add_argument("--start", default="0,0", metavar="X,Y")
    p.add_argument("--anchor", default="", metavar="LAT,LON,HEIGHT",
                   help="publish a FrameTransform for this frame. Omitted, "
                        "none is published and the robot is simply not "
                        "earth-referenced — a supported mode")
    p.add_argument("--bounds", default="", metavar="MINX,MINY,MAXX,MAXY",
                   help="a box this source will not leave, so the late-refusal "
                        "path can be exercised without hardware")
    p.add_argument("--announce-kind", default="OTHER")
    p.add_argument("--content-ref", action="append", default=[],
                   metavar="catalog:ID",
                   help="what this robot is, for a consumer that wants to "
                        "draw it. Repeatable. Omitted, the record carries an "
                        "empty sequence and a consumer renders a marker")
    p.add_argument("--external-ref", action="append", default=[],
                   metavar="KEY=VALUE",
                   help="what it is also known as, e.g. wikidata=Q4810574. "
                        "Repeatable")
    return p


def _config_from(args) -> tuple:
    anchor = None
    if args.anchor:
        parts = [float(v) for v in args.anchor.split(",")]
        if len(parts) != 3:
            raise ConfigError(
                "--anchor needs LAT,LON,HEIGHT — all three. A partial anchor "
                "is not a weaker claim about where the frame is, it is an "
                "unusable one; omit it entirely for the no-georeference mode")
        anchor = Anchor(*parts)
    bounds = None
    if args.bounds:
        parts = [float(v) for v in args.bounds.split(",")]
        if len(parts) != 4:
            raise ConfigError("--bounds needs MINX,MINY,MAXX,MAXY")
        bounds = Bounds(*parts)
    start = tuple(float(v) for v in args.start.split(","))
    external = []
    for item in (args.external_ref or []):
        if "=" not in item:
            raise ConfigError(
                f"--external-ref takes KEY=VALUE; got {item!r}. A reference "
                f"without a namespace cannot be resolved by anyone, which is "
                f"worse than publishing none")
        key, _, value = item.partition("=")
        external.append((key.strip(), value.strip()))
    kwargs = dict(scene=args.scene, entity_id=args.entity_id,
                  frame_id=args.frame_id, anchor=anchor, domain=args.domain,
                  ground_z=args.ground_z,
                  content_refs=tuple(args.content_ref or ()),
                  external_refs=tuple(external))
    if args.source_id:
        kwargs["source_id"] = args.source_id
    if args.label:
        kwargs["label"] = args.label
    return Config(**kwargs), start, bounds


def main(argv=None) -> int:                                   # pragma: no cover
    args = _parser().parse_args(argv)
    try:
        config, start, bounds = _config_from(args)
    except ConfigError as error:
        print(str(error), file=sys.stderr)
        return 2
    return Runner(config, rate_hz=args.rate_hz, start=start, speed=args.speed,
                  bounds=bounds, announce_kind=args.announce_kind).run()


if __name__ == "__main__":                                    # pragma: no cover
    sys.exit(main())
