#!/usr/bin/env python3
"""
The unmodified pip gateway, over a live DDS bus, for the acceptance run.

Test tooling rather than library code: `spatialdds_ros` is a type adapter on a
shared bus and has no business shipping an HTTP endpoint. But acceptance item 3
needs the whole chain — an N.6 POST reaching a robot — and that needs a
gateway in front of the bus, so one is assembled here from the parts the
`spatialdds` distribution already exposes.

Nothing in it is novel. It is the shape `bridges/web_bridge/binding.py` has in
the prototype, reduced to the one lane this run needs (commands in) plus the
resources the gateway serves from the bus.
"""
from __future__ import annotations

import asyncio
import threading
import time
from pathlib import Path
from typing import Any, Optional

from spatialdds.server import commands, gateway, gw_http, pin, publish

from spatialdds_idl18.builtin import Time
from spatialdds_idl18.spatial.core import Aabb3, FrameTransform, PoseSE3
from spatialdds_idl18.spatial.owm import Entity, ModelCommand, ModelPose

from spatialdds_ros import transport as tt
from spatialdds_ros.config import Config
from spatialdds_ros.qos import MODEL_COMMAND, MODEL_FAST, MODEL_LATCHED
from spatialdds_ros.recording import CLOSURE, IDL_DIR, SPEC_COMMIT
from spatialdds_ros.topics import Topics

#: Topic, class, QoS, colon fqn, dotted fqn.
def _lanes(config: Config) -> list:
    names = Topics(config.scene)
    return [
        (names.entity, Entity, MODEL_LATCHED,
         "spatial::owm::Entity", "spatial.owm.Entity"),
        (names.pose, ModelPose, MODEL_FAST,
         "spatial::owm::ModelPose", "spatial.owm.ModelPose"),
        (names.command, ModelCommand, MODEL_COMMAND,
         "spatial::owm::ModelCommand", "spatial.owm.ModelCommand"),
        (names.frame_transform, FrameTransform, MODEL_LATCHED,
         "spatial::core::FrameTransform", "spatial.core.FrameTransform"),
    ]


def build_state(config: Config) -> gateway.State:
    """
    A live registry, types from this repository's vendored IDL.

    The same closure the recorder uses, from the same directory, so a payload
    served here and a payload recorded there are the same type — a gateway
    whose types came from a different spec revision than the bytes it serves
    would be two type systems in one process.
    """
    lanes = _lanes(config)
    flat = pin.flatten(IDL_DIR, CLOSURE).encode()
    model, idl = pin.model_from_records(
        [(colons, flat) for _t, _c, _q, colons, _fqn in lanes],
        whence=f"spatialdds-ros idl/v1.8 (SpatialDDS-spec {SPEC_COMMIT})")
    state = gateway.State(
        source="live DDS",
        model=model,
        idl=idl,
        type_source=f"spatialdds-ros vendored IDL (SpatialDDS-spec "
                    f"{SPEC_COMMIT})")
    for topic, _cls, _qos, _colons, fqn in lanes:
        _scene, stream, type_seg, version = topic.split("/")[1:]
        latched, access = gateway.DURABILITY.get(
            fqn, (False, gateway.OPERATOR))
        agg = model.get(fqn)
        keys = [m.name for m in (agg.members if agg else []) if m.is_key]
        res = gateway.Resource(config.scene, stream, type_seg, version, topic,
                               fqn, keys[0] if keys else "", access, latched)
        state.resources[res.path] = res
        state.latched.setdefault(topic, {})
        state.live.setdefault(topic, [])
    state.scenes = sorted({r.scene for r in state.resources.values()})
    return state


class CommandLane:
    """
    An N.6 POST becomes a `ModelCommand` on the bus.

    The gateway has already validated the body against the published schema
    and lifted it to canonical JSON by the time this sees it, so the only job
    left is writing it. It *declines* rather than accepts when it cannot
    write, because a swallowed command is the silence M.1 forbids.
    """

    def __init__(self, participant, config: Config) -> None:
        self.topic = Topics(config.scene).command
        self.writer = tt.make_writer(participant, self.topic, ModelCommand,
                                     MODEL_COMMAND.name)
        self.sent = 0

    def handle(self, record: dict) -> "commands.Outcome":
        try:
            pose = record.get("pose") or {}
            extent = record.get("extent") or {}
            stamp = record.get("stamp") or {}
            command = ModelCommand(
                command_id=record["command_id"],
                verb=record["verb"],
                subject_id=record.get("subject_id", ""),
                reason=record.get("reason", ""),
                requester_id=record.get("requester_id", ""),
                has_pose=bool(record.get("has_pose")),
                pose=PoseSE3(
                    t=[float(v) for v in pose.get("t", [0, 0, 0])],
                    q=[float(v) for v in pose.get("q", [0, 0, 0, 1])]),
                has_extent=bool(record.get("has_extent")),
                extent=Aabb3(
                    min_xyz=[float(v) for v in extent.get("min_xyz", [0, 0, 0])],
                    max_xyz=[float(v) for v in extent.get("max_xyz", [0, 0, 0])]),
                # §N.1: an int64 is a decimal string on the wire, so `sec`
                # arrives as a string and has to go back to an integer here.
                stamp=Time(sec=int(stamp.get("sec", 0)),
                           nanosec=int(stamp.get("nanosec", 0))),
            )
        except (KeyError, TypeError, ValueError) as error:
            return commands.Outcome.decline(
                f"the command could not be built for the bus: "
                f"{type(error).__name__}: {error}")
        try:
            self.writer.write(command)
        except Exception as error:                            # noqa: BLE001
            return commands.Outcome.decline(
                f"the bus would not take it: {type(error).__name__}: {error}")
        self.sent += 1
        return commands.Outcome.accept(
            f"published {command.verb!r} for {command.subject_id!r} on "
            f"{self.topic}")


class Pump:
    """
    The bus into the gateway: every sample becomes a served resource.

    Only what the acceptance reads — entities and frame transforms. The
    command lane runs the other way and is not pumped.
    """

    def __init__(self, state: gateway.State, app, participant,
                 config: Config) -> None:
        self.state = state
        self.app = app
        self.config = config
        self.readers = [
            (topic, tt.make_reader(participant, topic, cls, qos.name))
            for topic, cls, qos, _colons, _fqn in _lanes(config)
            if cls in (Entity, FrameTransform)]
        self.counts: dict[str, int] = {}

    def drain(self) -> int:
        """Pull what is on the bus into the gateway's state. Returns how many."""
        from spatialdds.server import xcdr2
        decoder = xcdr2.Decoder(self.state.model)
        moved = 0
        for topic, reader in self.readers:
            resource = next(r for r in self.state.resources.values()
                            if r.topic == topic)
            for sample in tt.take_samples(reader):
                # The payload as the gateway's own decoder sees it, from the
                # bytes that were on the wire. Decoding with the package's
                # reader rather than projecting the dataclass myself is the
                # point: it is the same path the recording takes, so a type
                # error cannot pass here and fail there.
                doc = decoder.decode(resource.fqn, sample.serialize())
                # The key comes from the *decoded doc* keyed by the member the
                # resource names, not from the dataclass — `resource.key` is a
                # member name taken from the IDL's `@key`.
                key = str(doc.get(resource.key, "")) if resource.key else ""
                if resource.latched:
                    publish.apply_sample(self.state, topic, resource, key, doc)
                else:
                    self.state.counts[topic] = (
                        self.state.counts.get(topic, 0) + 1)
                gw_http.notify(self.app, topic,
                               {"event": "sample", "topic": topic,
                                "key": key, "data": doc})
                self.counts[topic] = self.counts.get(topic, 0) + 1
                moved += 1
        return moved


class LiveGateway:
    """The gateway, on its own thread, over a live bus."""

    def __init__(self, config: Config, participant, port: int,
                 token: str) -> None:
        self.config = config
        self.port = port
        self.token = token
        self.base = f"http://127.0.0.1:{port}"
        self.state = build_state(config)
        self.registry = commands.Registry()
        self.app = gw_http.make_app(
            self.state, self.base, token, publish_enabled=False,
            live_subscriptions=True, command_registry=self.registry)
        self.lane = CommandLane(participant, config)
        self.registry.register("spatial.owm.ModelCommand", self.lane.handle)
        self.pump = Pump(self.state, self.app, participant, config)
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._thread: Optional[threading.Thread] = None
        self._runner = None

    def start(self, timeout: float = 20.0) -> None:
        ready = threading.Event()

        def serve() -> None:
            from aiohttp import web
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._loop = loop

            async def up() -> None:
                self._runner = web.AppRunner(self.app)
                await self._runner.setup()
                site = web.TCPSite(self._runner, "127.0.0.1", self.port)
                await site.start()
                ready.set()

            loop.run_until_complete(up())
            loop.run_forever()

        self._thread = threading.Thread(target=serve, daemon=True)
        self._thread.start()
        if not ready.wait(timeout):                           # pragma: no cover
            raise AssertionError("the gateway did not come up")

    def stop(self) -> None:
        if self._loop is not None:
            self._loop.call_soon_threadsafe(self._loop.stop)
        if self._thread is not None:
            self._thread.join(timeout=5)
