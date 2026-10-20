"""
Endpoints, with the §3.3.3 QoS actually applied.

Thin on purpose. Its whole job is that every endpoint this library creates
goes through one place that resolves a profile name to a `Qos`, so the
profiles in `qos.py` are true on the wire rather than decorative. The
prototype's `typed_transport` exists for the same reason and this follows its
shape.
"""
from __future__ import annotations

from typing import Any, List, Optional, Type

from cyclonedds.domain import DomainParticipant
from cyclonedds.pub import DataWriter, Publisher
from cyclonedds.sub import DataReader, Subscriber
from cyclonedds.topic import Topic

from spatialdds_ros.qos import PROFILES, QosProfile


class UnknownQosProfile(KeyError):
    """A profile name that is not one of the three this library uses."""


def _profile(name: str) -> QosProfile:
    try:
        return PROFILES[name]
    except KeyError:
        raise UnknownQosProfile(
            f"{name!r} is not a profile this library carries; expected one of "
            f"{', '.join(sorted(PROFILES))}") from None


def _topic(participant: DomainParticipant, topic_name: str, datatype: Type,
           profile: QosProfile) -> Topic:
    """
    The topic, with QoS derived from the profile **and nothing else**.

    Lifespan is deliberately excluded here even though DDS lists it as a topic
    policy. A topic's QoS has to be the same whoever creates the object, and
    this library creates one per endpoint: a reader built without a lifespan
    and a writer built with one would disagree about the same topic name, and
    DDS rejects the second with `DDS_RETCODE_INCONSISTENT_POLICY` — on whichever
    endpoint happened to be constructed second, which makes it look like a
    problem with that endpoint rather than with the pair.

    Found exactly that way: an announce reader opened first, then the announce
    writer with its `ttl_sec` mapped onto Lifespan, and the writer failed. So
    the topic is a function of the profile alone and Lifespan rides on the
    writer, which is where a payload TTL belongs anyway — it is the publisher's
    statement about how long its own sample is worth keeping.
    """
    return Topic(participant, topic_name, datatype, qos=profile.to_qos())


def make_writer(participant: DomainParticipant, topic_name: str,
                datatype: Type, qos_profile: str, *,
                lifespan_sec: Optional[float] = None) -> DataWriter:
    profile = _profile(qos_profile)
    topic = _topic(participant, topic_name, datatype, profile)
    writer_qos = profile.to_qos(lifespan_sec=lifespan_sec)
    return DataWriter(Publisher(participant, qos=profile.to_qos()), topic,
                      qos=writer_qos)


def make_reader(participant: DomainParticipant, topic_name: str,
                datatype: Type, qos_profile: str) -> DataReader:
    profile = _profile(qos_profile)
    topic = _topic(participant, topic_name, datatype, profile)
    return DataReader(Subscriber(participant, qos=profile.to_qos()), topic,
                      qos=profile.to_qos())


def take_samples(reader: DataReader, *, n: int = 100) -> List[Any]:
    """
    Valid samples only.

    Disposal is not a signal this library acts on: the adapter owns exactly one
    key and never disposes it — going quiet is reported as `UNOBSERVED`, which
    is a different and weaker statement than "this robot is gone".
    """
    return [s for s in (reader.take(N=n) or []) if s is not None]
