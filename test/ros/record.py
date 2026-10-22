"""
Record the session's SpatialDDS lanes to a typed MCAP, until interrupted.

    record.py DOMAIN OUT.mcap

The library's own `Recorder`, fed from the bus: every valid sample on every
lane it knows, as the bytes it was. Prints the per-lane counts as JSON on
exit. Test tooling for test_acceptance_two_domains.py.
"""
import json
import signal
import sys
import time

from cyclonedds.domain import DomainParticipant

from spatialdds_ros import transport as tt
from spatialdds_ros.config import Config
from spatialdds_ros.qos import MODEL_COMMAND, MODEL_FAST, MODEL_LATCHED
from spatialdds_ros.recording import Recorder, lanes
from spatialdds_ros.topics import Topics

config = Config(scene="bare-scene", entity_id="ent:recorder",
                frame_id="map/bare-scene", domain=int(sys.argv[1]))
participant = DomainParticipant(config.domain)
names = Topics(config.scene)
profile = {names.pose: MODEL_FAST.name, names.command: MODEL_COMMAND.name}
readers = {topic: tt.make_reader(participant, topic, cls,
                                 profile.get(topic, MODEL_LATCHED.name))
           for topic, (cls, *_rest) in lanes(config).items()}
stop = False


def _stop(*_):
    global stop
    stop = True


signal.signal(signal.SIGINT, _stop)
signal.signal(signal.SIGTERM, _stop)
print("recording", flush=True)
with Recorder(sys.argv[2], config) as recorder:
    while not stop:
        for topic, reader in readers.items():
            for sample in tt.take_samples(reader):
                recorder.write(topic, sample)
        time.sleep(0.02)
    print(json.dumps(recorder.close()), flush=True)
