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


def make_writer(participant: DomainParticipant, topic_name: str,
                datatype: Type, qos_profile: str, *,
                lifespan_sec: Optional[float] = None) -> DataWriter:
    profile = _profile(qos_profile)
    qos = profile.to_qos(lifespan_sec=lifespan_sec)
    topic = Topic(participant, topic_name, datatype, qos=qos)
    return DataWriter(Publisher(participant, qos=qos), topic, qos=qos)


def make_reader(participant: DomainParticipant, topic_name: str,
                datatype: Type, qos_profile: str, *,
                lifespan_sec: Optional[float] = None) -> DataReader:
    profile = _profile(qos_profile)
    qos = profile.to_qos(lifespan_sec=lifespan_sec)
    topic = Topic(participant, topic_name, datatype, qos=qos)
    return DataReader(Subscriber(participant, qos=qos), topic, qos=qos)


def take_samples(reader: DataReader, *, n: int = 100) -> List[Any]:
    """
    Valid samples only.

    Disposal is not a signal this library acts on: the adapter owns exactly one
    key and never disposes it — going quiet is reported as `UNOBSERVED`, which
    is a different and weaker statement than "this robot is gone".
    """
    return [s for s in (reader.take(N=n) or []) if s is not None]
