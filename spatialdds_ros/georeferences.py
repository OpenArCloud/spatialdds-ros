"""
The scene's georeferences, read once for every consumer in this library.

Awareness had this to itself until 0.5.0; the claims layer needs the same
book, and two copies of "which georeference counts" would let two consumers on
one robot place one thing in two places. So it lives here, and both read it.

The rules are awareness's, unchanged (`docs/awareness-census.md`):

* Only transforms whose parent is `earth-fixed` compose here.
* A retracted georeference is a removal (§2.14). A writer that merely left is
  not: a venue's claim about its frame stands.
* Two georeferences for one frame that disagree by more than
  `GEOREFERENCE_AGREEMENT_M` are a conflict, and nothing in that frame is
  placed.
* This robot's own anchor counts for its own frame when nothing on the bus
  does.
"""
from __future__ import annotations

import math
from typing import Dict, Tuple

from cyclonedds.core import InstanceState
from cyclonedds.domain import DomainParticipant

from spatialdds_idl18.spatial.core import FrameTransform, PoseSE3

from spatialdds_ros import transport as tt
from spatialdds_ros.config import Config
from spatialdds_ros.frames import georeference
from spatialdds_ros.qos import MODEL_LATCHED
from spatialdds_ros.topics import EARTH_FIXED_FQN, Topics

#: Two georeferences for one frame that disagree by more than this are a
#: conflict, and nothing in that frame is placed.
GEOREFERENCE_AGREEMENT_M = 0.01


class GeoreferenceBook:
    """The frames lane, kept as one georeference per frame."""

    def __init__(self, participant: DomainParticipant, config: Config) -> None:
        self.config = config
        self._reader = tt.make_reader(
            participant, Topics(config.scene).frame_transform, FrameTransform,
            MODEL_LATCHED.name)
        #: child fqn -> {transform_id: T_parent_child}
        self._claims: Dict[str, Dict[str, PoseSE3]] = {}
        self._handles: Dict[int, Tuple[str, str]] = {}

    def poll(self) -> None:
        for sample in tt.take_with_state(self._reader):
            self.on_sample(sample)

    def on_sample(self, sample) -> None:
        handle = sample.instance_handle
        if sample.data is not None:
            ft = sample.data
            if ft.parent_ref.fqn != EARTH_FIXED_FQN:
                return                  # only georeferences compose here
            child = ft.child_ref.fqn
            self._claims.setdefault(child, {})[ft.transform_id] = \
                ft.T_parent_child
            if handle is not None:
                self._handles[int(handle)] = (child, ft.transform_id)
        if (not sample.alive and handle is not None
                and int(sample.instance_state)
                == int(InstanceState.NotAliveDisposed)):
            child, tid = self._handles.get(int(handle), (None, None))
            if child is not None:
                self._claims.get(child, {}).pop(tid, None)

    def book(self) -> Tuple[Dict[str, PoseSE3], Dict[str, str]]:
        """One georeference per frame, and the frames that have a conflict."""
        book: Dict[str, PoseSE3] = {}
        conflicts: Dict[str, str] = {}
        for child, claims in self._claims.items():
            poses = list(claims.values())
            if not poses:
                continue
            spread = max(math.dist(poses[0].t, p.t) for p in poses)
            if spread > GEOREFERENCE_AGREEMENT_M:
                conflicts[child] = (
                    f"{len(poses)} georeferences for {child!r} disagree by "
                    f"{spread:.2f} m")
            else:
                book[child] = poses[0]
        own = self.config.frame_id
        if own not in book and own not in conflicts and self.config.anchor:
            book[own] = georeference(own, self.config.anchor).T_parent_child
        return book, conflicts
