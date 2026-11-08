#!/usr/bin/env python3
"""
A hazard asserter in its own process, for the tests that need one to die.

Test tooling, the claims layer's `_peer_robot.py`: a scripted obstacle
source. It tracks one box through `ObstaclePublisher` and holds one OBSERVED
claim about it, until it is killed or its stdin closes.

    python3 test/_hazard_asserter.py DOMAIN SCENE FRAME SOURCE_ID X Y L W \
        PREDICATE OBJECT

Prints "up <claim id>" once both are written.
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cyclonedds.domain import DomainParticipant               # noqa: E402

from spatialdds_ros.config import Config                       # noqa: E402
from spatialdds_ros.perception import ObstaclePublisher        # noqa: E402

(domain, scene, frame, source, x, y, length, width, predicate,
 obj) = sys.argv[1:11]
config = Config(scene=scene, entity_id=f"ent:hazard:{source.split(':')[-1]}",
                frame_id=frame, source_id=source, domain=int(domain))
publisher = ObstaclePublisher(DomainParticipant(int(domain)), config)
publisher.observed("hazard-1", float(x), float(y), length=float(length),
                   width=float(width), label="hazard")
claim = publisher.claim("hazard-1", predicate, obj)
print(f"up {claim}", flush=True)
while True:
    publisher.observed("hazard-1", float(x), float(y), length=float(length),
                       width=float(width), label="hazard")
    publisher.tick()
    time.sleep(0.1)
