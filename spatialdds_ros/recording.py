"""
Record this library's lanes to a typed SpatialDDS MCAP.

The file this writes is the one `spatialdds serve --mcap` reads — the
unmodified gateway from PyPI, with no adapter in between. That is the point:
if the gateway can serve it, the bytes on the bus were conformant, and the
check is the gateway rather than an assertion of mine.

**The format is defined by what the gateway reads, not chosen here.** Matching
the packaged `fountain-anchors.mcap`: one schema per type, named in *colon*
form (`spatial::owm::Entity`), `encoding="omgidl"`, data being the flattened
IDL text; one channel per topic with `message_encoding="cdr"`; and each
message the sample's own CycloneDDS serialization.

That last part is worth stating because it is the load-bearing coincidence,
and it was verified rather than assumed: `Entity.serialize()` from
cyclonedds-python decodes with `spatialdds.server.xcdr2.Decoder` against the
model built from this IDL, 1.8's appended scale members included. Nothing
re-encodes anything — the bytes recorded are the bytes that were on the wire.

Note the contrast with the prototype's `bridges/mcap_bridge`, which writes
`message_encoding="json"` with `encoding="jsonschema"`. That is a 1.7 lane for
a different consumer and is not what the gateway reads; a recording in that
format would be a perfectly good MCAP that `serve --mcap` could not use.
"""
from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Dict, Optional

from spatialdds_idl18.spatial.core import FrameTransform
from spatialdds_idl18.spatial.disco import Announce
from spatialdds_idl18.spatial.owm import Entity, ModelCommand, ModelPose

from spatialdds_ros.announce import ANNOUNCE_TOPIC
from spatialdds_ros.config import Config
from spatialdds_ros.topics import Topics

#: The IDL closure the four model types need, relative to `idl/v1.8`.
#: `provisional/owm.idl` is here because `spatial.owm` is provisional and
#: independently versioned, so the `spatialdds` package does not vendor it —
#: and should not. This repository does.
CLOSURE = ["types.idl", "core.idl", "provisional/owm.idl"]

#: And the closure for the discovery lane, which `Announce` needs.
DISCOVERY_CLOSURE = ["types.idl", "core.idl", "discovery.idl"]

#: Where the vendored IDL lives, and where it came from. Kept beside the
#: generated bindings' provenance deliberately: a recording whose schemas came
#: from a different spec revision than the bindings that produced the bytes
#: would be two type systems in one file.
IDL_DIR = Path(__file__).resolve().parent.parent / "idl" / "v1.8"
SPEC_COMMIT = "9c3e1f2"


def lanes(config: Config) -> Dict[str, tuple]:
    """
    Topic -> (python class, colon fqn, dotted fqn, closure).

    The four model lanes plus discovery. Discovery is not scene-scoped — it is
    the one place a participant looks before it knows what scenes exist.
    """
    names = Topics(config.scene)
    return {
        names.entity: (Entity, "spatial::owm::Entity",
                       "spatial.owm.Entity", CLOSURE),
        names.pose: (ModelPose, "spatial::owm::ModelPose",
                     "spatial.owm.ModelPose", CLOSURE),
        names.command: (ModelCommand, "spatial::owm::ModelCommand",
                        "spatial.owm.ModelCommand", CLOSURE),
        names.frame_transform: (FrameTransform,
                                "spatial::core::FrameTransform",
                                "spatial.core.FrameTransform", CLOSURE),
        ANNOUNCE_TOPIC: (Announce, "spatial::disco::Announce",
                         "spatial.disco.Announce", DISCOVERY_CLOSURE),
    }


class Recorder:
    """
    One MCAP writer, schemas and channels registered on first sight.

    Lazily, because a lane with nothing on it should not appear in the
    recording: an empty channel tells a reader a lane exists and said nothing,
    which is a claim this recorder is not in a position to make — it only knows
    what it saw.
    """

    def __init__(self, path: Path, config: Config) -> None:
        from mcap.writer import Writer

        self.path = Path(path)
        self.config = config
        self._lanes = lanes(config)
        self._handle = self.path.open("wb")
        self._writer = Writer(self._handle)
        self._writer.start()
        self._schema_ids: Dict[str, int] = {}
        self._channel_ids: Dict[str, int] = {}
        self._flat: Dict[str, bytes] = {}
        self.counts: Dict[str, int] = {}

    # --- schemas and channels --------------------------------------------

    def _flattened(self, closure_key: str, closure: list) -> bytes:
        if closure_key not in self._flat:
            from spatialdds.server import pin
            self._flat[closure_key] = pin.flatten(IDL_DIR, closure).encode()
        return self._flat[closure_key]

    def _schema_id(self, colon_fqn: str, closure: list) -> int:
        if colon_fqn not in self._schema_ids:
            closure_key = ",".join(closure)
            self._schema_ids[colon_fqn] = self._writer.register_schema(
                name=colon_fqn, encoding="omgidl",
                data=self._flattened(closure_key, closure))
        return self._schema_ids[colon_fqn]

    def _channel_id(self, topic: str) -> int:
        if topic not in self._channel_ids:
            _cls, colon_fqn, dotted, closure = self._lanes[topic]
            self._channel_ids[topic] = self._writer.register_channel(
                schema_id=self._schema_id(colon_fqn, closure),
                topic=topic,
                message_encoding="cdr",
                metadata={
                    "spatialdds_type": dotted,
                    "spatialdds_version": "1.8",
                    "spatialdds_idl": f"SpatialDDS-spec {SPEC_COMMIT}",
                })
            self.counts.setdefault(topic, 0)
        return self._channel_ids[topic]

    # --- writing ----------------------------------------------------------

    def write(self, topic: str, sample: Any,
              log_time_ns: Optional[int] = None) -> None:
        """
        One sample, as the bytes it was on the wire.

        `sample.serialize()` and nothing else: re-encoding would make this a
        recording of what the recorder thinks the sample means rather than of
        what was published.
        """
        if topic not in self._lanes:
            raise KeyError(
                f"{topic!r} is not a lane this recorder knows; expected one of "
                f"{sorted(self._lanes)}")
        now = time.time_ns() if log_time_ns is None else log_time_ns
        self._writer.add_message(
            channel_id=self._channel_id(topic),
            log_time=now, publish_time=now, data=sample.serialize())
        self.counts[topic] = self.counts.get(topic, 0) + 1

    def close(self) -> Dict[str, int]:
        """Finish the file and return what was recorded, per lane."""
        self._writer.finish()
        self._handle.close()
        return dict(self.counts)

    def __enter__(self) -> "Recorder":
        return self

    def __exit__(self, *exc) -> None:
        if not self._handle.closed:
            self.close()
