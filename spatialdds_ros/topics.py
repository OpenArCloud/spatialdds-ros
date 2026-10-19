"""
Topic names, derived from the scene and never configured.

§3.3.1's five-segment grammar: `spatialdds/{scene}/{group}/{type}/{version}`.
The scene is the only free variable; the names below are that grammar applied
to it.

**Why these are not parameters.** A topic name is the whole of interop — two
participants meet because they chose the same string and for no other reason.
Exposing the strings would let an operator produce a name nothing else looks
for, and the symptom is an empty bus: no error, no warning, no mismatch
reported anywhere, just a robot publishing into a topic with no readers and a
scene that never sees it. There is one right answer per scene, so the library
fills it.

The prototype had the same four names with the scene baked into module-level
constants (`spatialdds_demo/topics.py`), which is the same decision made for a
single scene.
"""
from __future__ import annotations

from dataclasses import dataclass

#: The frame that `earth-fixed` names, shared by every participant that
#: georeferences anything. Not a parameter: it is the one frame name that must
#: agree across the whole bus, and the prototype, the catalogue and the spec
#: all spell it this way.
EARTH_FIXED_FQN = "earth-fixed"


@dataclass(frozen=True)
class Topics:
    """The four lanes this library uses, for one scene."""

    scene: str

    @property
    def entity(self) -> str:
        """Latched records: what exists, and what it is."""
        return f"spatialdds/{self.scene}/world/entity/v1"

    @property
    def pose(self) -> str:
        """The fast lane: where it is now."""
        return f"spatialdds/{self.scene}/world/model_pose/v1"

    @property
    def command(self) -> str:
        """Requests addressed to something in the scene."""
        return f"spatialdds/{self.scene}/world/model_command/v1"

    @property
    def frame_transform(self) -> str:
        """Where the scene's frames sit relative to one another."""
        return f"spatialdds/{self.scene}/frames/frame_transform/v1"

    def all(self) -> dict[str, str]:
        """Every name, for a startup banner and for the tests to check."""
        return {
            "entity": self.entity,
            "pose": self.pose,
            "command": self.command,
            "frame_transform": self.frame_transform,
        }
