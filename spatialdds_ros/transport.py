"""
Endpoints, with the §3.3.3 QoS actually applied.

Thin on purpose, with two jobs. Every endpoint this library creates goes
through one place that resolves a profile name to a `Qos`, so the profiles in
`qos.py` are true on the wire rather than decorative; and every read goes
through one place that knows `take()` returns lifecycle notifications as well
as payloads. Both are the kind of rule that is wrong in one spot or wrong
everywhere, and one enforcement point is the difference. The prototype's
`typed_transport` exists for the same reasons and this follows its shape.
"""
from __future__ import annotations

from typing import Any, List, NamedTuple, Optional, Type

from cyclonedds.core import InstanceState
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


def _topic(participant: DomainParticipant, topic_name: str,
           datatype: Type) -> Topic:
    """
    The topic, with **default QoS**. The profile rides on the endpoints.

    A `Topic` object's QoS must be identical for every creation of the same
    name within a participant: DDS refuses the second with
    `DDS_RETCODE_INCONSISTENT_POLICY`, and the refusal lands on whichever
    endpoint happened to be constructed later, which makes it look like a
    problem with that endpoint rather than with the pair.

    That makes topic QoS a contract between **every participant in the
    process**, not a property of this library's lanes — and nothing in this
    API said so. The first real consumer paid for it. SpatialDDS-demo applies
    the same §3.3.3 profiles to endpoints only; it asked for an entity reader
    on a topic this library had already created carrying the profile, and was
    refused. Both conventions are legal DDS. They are not legal together, and
    the library had quietly made itself the one that must be matched.

    So it now asks for the least it can: the default, which is what any
    participant that passes no QoS gets, including every ROS and DDS example.

    Nothing leaves the wire. Matching is a reader/writer negotiation over
    *endpoint* QoS — a topic's QoS enters it only as a default for endpoints
    that ask for one, and these never do; both are always given the profile
    explicitly. `test_transport.py` pins both halves: that the policies still
    reach the endpoints, and that a library writer still exchanges with a
    reader built the other convention's way.

    Lifespan was excluded here for this exact reason on 2026-10-18, one policy
    at a time, when an announce writer with a `ttl_sec` failed against a reader
    opened first. This is that lesson finished.
    """
    return Topic(participant, topic_name, datatype)


def make_writer(participant: DomainParticipant, topic_name: str,
                datatype: Type, qos_profile: str, *,
                lifespan_sec: Optional[float] = None) -> DataWriter:
    profile = _profile(qos_profile)
    topic = _topic(participant, topic_name, datatype)
    writer_qos = profile.to_qos(lifespan_sec=lifespan_sec, writer=True)
    return DataWriter(Publisher(participant, qos=profile.to_qos()), topic,
                      qos=writer_qos)


def make_reader(participant: DomainParticipant, topic_name: str,
                datatype: Type, qos_profile: str) -> DataReader:
    profile = _profile(qos_profile)
    topic = _topic(participant, topic_name, datatype)
    return DataReader(Subscriber(participant, qos=profile.to_qos()), topic,
                      qos=profile.to_qos())


STATE_NAMES = {int(InstanceState.Alive): "ALIVE",
               int(InstanceState.NotAliveDisposed): "NOT_ALIVE_DISPOSED",
               int(InstanceState.NotAliveNoWriters): "NOT_ALIVE_NO_WRITERS"}


class TakenSample(NamedTuple):
    """
    A sample and what the middleware said about its instance.

    `data` is `None` for a lifecycle notification — a dispose or a
    writer-gone — which carries instance state and no payload. It is *not*
    the only way a not-alive state arrives: a valid sample that has not been
    read when its writer disposes carries the state on its own `sample_info`
    instead, and no separate notification is generated. So `data` and `alive`
    are independent, and a caller that wants the lifecycle must read `alive`
    rather than infer it from a missing payload.
    """
    data: Optional[Any]
    alive: bool
    instance_handle: Optional[int]
    instance_state: Any

    @property
    def state_name(self) -> str:
        """For logs. `instance_state` is a plain int, not an enum member."""
        return STATE_NAMES.get(int(self.instance_state),
                               f"UNKNOWN({int(self.instance_state)})")


def take_with_state(reader: DataReader, *, n: int = 100) -> List[TakenSample]:
    """
    Samples **and their instance state**, so a caller can act on disposal.

    CycloneDDS's `take()` does not return payloads only. It also yields
    `InvalidSample` objects — dispose and no-writer notifications, carrying
    `sample_info` with `valid_data == False` and no fields of the datatype at
    all. A read that assumes every sample is a payload reads an attribute off
    one and raises.
    """
    out: List[TakenSample] = []
    for sample in (reader.take(N=n) or []):
        if sample is None:
            continue
        info = getattr(sample, "sample_info", None)
        valid = bool(getattr(info, "valid_data", True)) if info else True
        state = (getattr(info, "instance_state", InstanceState.Alive)
                 if info else InstanceState.Alive)
        out.append(TakenSample(
            data=sample if valid else None,
            alive=valid and state == InstanceState.Alive,
            instance_handle=getattr(info, "instance_handle", None) if info
            else None,
            instance_state=state))
    return out


def take_samples(reader: DataReader, *, n: int = 100) -> List[Any]:
    """
    Payloads only — lifecycle notifications dropped, not mistaken for one.

    Disposal is not a signal this library *acts* on: the adapter owns exactly
    one key and never disposes it, and going quiet is reported as `UNOBSERVED`,
    which is a weaker statement than "this robot is gone". But not acting on
    an event is not the same as not receiving it, and this filtered only
    `None` until 2026-10-19 — so the first peer that joined a lane and left
    killed the robot reading it:

        AttributeError: 'InvalidSample' object has no attribute 'subject_id'

    Two call sites, both reached by an ordinary deployment. `poll_commands`
    crashed when any requester's participant went away; `already_published`
    would have crashed at startup on the tombstone of the robot that ran
    before it — and a tombstone must not count as a live conflict either,
    which it now cannot, because it never arrives as a record.

    Nothing in this library's 153 tests produced one: every writer outlived
    every reader. `test_transport.py` now keeps a peer that dies.
    """
    return [s.data for s in take_with_state(reader, n=n) if s.data is not None]
