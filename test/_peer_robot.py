#!/usr/bin/env python3
"""
A peer robot in its own process, for the tests that need one to die.

Test tooling. A participant deleted inside the test process leaves through
the front door; a robot that crashes does not, and only a process the test
can SIGKILL shows what a reader sees then.

    python3 test/_peer_robot.py DOMAIN ENTITY_ID X Y
"""
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from cyclonedds.domain import DomainParticipant           # noqa: E402

from spatialdds_ros.adapter import RobotAdapter            # noqa: E402
from spatialdds_ros.config import Config                   # noqa: E402

domain, entity_id, x, y = (int(sys.argv[1]), sys.argv[2],
                           float(sys.argv[3]), float(sys.argv[4]))
robot = RobotAdapter(DomainParticipant(domain),
                     Config(scene="bare-scene", entity_id=entity_id,
                            frame_id="map/bare-scene"))
print("up", flush=True)
while True:
    robot.observed(x, y, 0.0)
    robot.tick()
    time.sleep(0.1)
