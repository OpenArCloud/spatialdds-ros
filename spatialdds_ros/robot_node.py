"""
The ROS 2 node: one process holding both stacks.

**rclpy is imported inside `main`, not at module scope.** That is not
tidiness. It keeps every other module in this package importable — and
therefore testable — on a machine with no ROS, which is where this library's
unit tests run and where its CI runs. The prototype does the same thing for
the same reason.

**Both stacks in one process is the proven pattern, not a gamble.** ROS
Humble's middleware is CycloneDDS 0.10.5 while the Python binding here is
11.0.1, which sounds like it cannot work and does: the binding loads its own
`libddsc` and shares no symbols with `rmw_cyclonedds_cpp`'s. SpatialDDS and
ROS meet on the wire because both are DDS — here, in one process, with the
model read as model and ROS read as ROS. Nothing is relayed and nothing is
translated into an envelope type and back.
"""
from __future__ import annotations

import math
import sys
from typing import Optional

from spatialdds_ros.config import (
    DEFAULT_PEER_RADIUS_M, DEFAULT_POSE_FLOOR_HZ, Anchor, Config, ConfigError)


def _declare(node) -> Config:
    """
    Read the parameter surface off the node, and let `Config` judge it.

    Every parameter is declared with a *sentinel* default rather than a real
    one for the three that must refuse to start, so "not set" is
    distinguishable from "set to the empty string". `Config` is the single
    place that decides what is acceptable; this function only collects.
    """
    node.declare_parameter("scene", "")
    node.declare_parameter("entity_id", "")
    node.declare_parameter("frame_id", "")
    node.declare_parameter("source_id", "")
    node.declare_parameter("label", "")
    node.declare_parameter("domain", 0)
    node.declare_parameter("ground_z", 0.0)
    node.declare_parameter("pose_source", "tf")
    node.declare_parameter("nav2_action", "navigate_to_pose")
    node.declare_parameter("type_uri", "")
    # String arrays, because that is what a launch file can express. Declared
    # with empty defaults: an adapter states only what the operator
    # configured, so "not set" has to be distinguishable from "set to
    # nothing" — and both mean the same published empty sequence.
    node.declare_parameter("content_refs", [""])
    node.declare_parameter("external_refs", [""])
    node.declare_parameter("base_frame", "base_link")
    node.declare_parameter("map_frame", "map")
    node.declare_parameter("odom_topic", "/odom")
    node.declare_parameter("amcl_topic", "/amcl_pose")
    node.declare_parameter("rate_hz", 10.0)
    node.declare_parameter("pose_floor_hz", DEFAULT_POSE_FLOOR_HZ)
    # Awareness. Off unless asked for; the range has a NaN sentinel because
    # "not set" must refuse to start when awareness is on (`Config`).
    node.declare_parameter("awareness", False)
    node.declare_parameter("awareness_range_m", float("nan"))
    node.declare_parameter("peer_radius_m", DEFAULT_PEER_RADIUS_M)
    node.declare_parameter("awareness_types", [""])
    # Three separate parameters rather than one array, so a launch file that
    # sets two of them fails on the third instead of silently shipping a
    # two-element anchor.
    node.declare_parameter("anchor_lat_deg", float("nan"))
    node.declare_parameter("anchor_lon_deg", float("nan"))
    node.declare_parameter("anchor_height_m", float("nan"))

    def text(name: str) -> str:
        return str(node.get_parameter(name).value or "")

    def number(name: str) -> float:
        return float(node.get_parameter(name).value)

    lat, lon, height = (number("anchor_lat_deg"), number("anchor_lon_deg"),
                        number("anchor_height_m"))
    given = [not math.isnan(v) for v in (lat, lon, height)]
    if any(given) and not all(given):
        raise ConfigError(
            "an anchor needs all three of anchor_lat_deg, anchor_lon_deg and "
            "anchor_height_m; a partial anchor is not a weaker claim about "
            "where the frame is, it is an unusable one. Set all three, or none "
            "— none is a supported mode and publishes no FrameTransform")
    anchor = Anchor(lat, lon, height) if all(given) else None

    kwargs = dict(
        scene=text("scene"),
        entity_id=text("entity_id"),
        frame_id=text("frame_id"),
        anchor=anchor,
        domain=int(node.get_parameter("domain").value),
        ground_z=number("ground_z"),
        pose_source=text("pose_source") or "tf",
        nav2_action=text("nav2_action") or "navigate_to_pose",
        pose_floor_hz=number("pose_floor_hz"),
        awareness=bool(node.get_parameter("awareness").value),
        peer_radius_m=number("peer_radius_m"),
    )
    reach = number("awareness_range_m")
    if not math.isnan(reach):
        kwargs["awareness_range_m"] = reach
    if text("source_id"):
        kwargs["source_id"] = text("source_id")
    if text("label"):
        kwargs["label"] = text("label")
    if text("type_uri"):
        kwargs["type_uri"] = text("type_uri")

    def strings(name: str) -> list:
        raw = node.get_parameter(name).value or []
        return [str(v).strip() for v in raw if str(v).strip()]

    types = strings("awareness_types")
    if types:
        kwargs["awareness_types"] = types
    refs = strings("content_refs")
    if refs:
        kwargs["content_refs"] = refs
    external = []
    for item in strings("external_refs"):
        if "=" not in item:
            raise ConfigError(
                f"external_refs entries are key=value; got {item!r}. A "
                f"reference without a namespace cannot be resolved by "
                f"anyone, which is worse than publishing none")
        key, _, value = item.partition("=")
        external.append((key.strip(), value.strip()))
    if external:
        kwargs["external_refs"] = external
    return Config(**kwargs)


def main(argv=None) -> int:                                   # pragma: no cover
    import rclpy
    from rclpy.node import Node

    from cyclonedds.domain import DomainParticipant

    from spatialdds_ros.adapter import RobotAdapter, already_published

    rclpy.init(args=argv)

    class SpatialDdsRobot(Node):
        def __init__(self) -> None:
            super().__init__("spatialdds_robot")
            self.config = _declare(self)
            for note in self.config.startup_notes():
                self.get_logger().info(note)

            self.participant = DomainParticipant(self.config.domain)
            existing = already_published(self.participant, self.config)
            if existing is not None:
                raise SystemExit(
                    f"{self.config.entity_id} is already being published by "
                    f"{existing}. Refusing to start: one writer per key, and "
                    f"two of us would put one robot in two places with "
                    f"nothing reporting the conflict")

            self.adapter = RobotAdapter(
                self.participant, self.config,
                navigator=_Nav2Navigator(self, self.config.nav2_action))
            note = self.adapter.publish_georeference()
            if note:
                self.get_logger().info(note)

            self._subscribe_to_the_pose()
            self._awareness = (_AwarenessOutput(self)
                               if self.config.awareness else None)
            period = 1.0 / float(self.get_parameter("rate_hz").value)
            self.create_timer(period, self._tick)
            self.get_logger().info(
                f"owns {self.config.entity_id} on DDS domain "
                f"{self.config.domain}; driving through "
                f"{self.config.nav2_action}")

        # --- the pose in ------------------------------------------------

        def _subscribe_to_the_pose(self) -> None:
            """
            **A map-frame pose, and that is the whole of this method's care.**

            The prototype read `nav_msgs/Odometry` on `/odom`, which is
            map-frame *only* in its simulator, where `map -> odom` is static
            and identity. On a localizing robot odom drifts against map — that
            drift is the entire reason AMCL publishes a correction — so reading
            it would publish a slowly-diverging claim about where the robot is
            on the earth, with every number looking plausible. The census files
            that against the demo; here the default simply is not `/odom`.

            `tf` is the default because it is what nav2 itself consumes, works
            whatever the localizer is, and assumes no topic name.
            """
            source = self.config.pose_source
            if source == "tf":
                from tf2_ros import Buffer, TransformListener
                self._tf_buffer = Buffer()
                self._tf_listener = TransformListener(self._tf_buffer, self)
                self._tf_freshness = TfFreshness()
                self._read_pose = self._pose_from_tf
            elif source == "amcl":
                from geometry_msgs.msg import PoseWithCovarianceStamped
                self._amcl_topic = str(self.get_parameter("amcl_topic").value)
                self.create_subscription(
                    PoseWithCovarianceStamped, self._amcl_topic,
                    self._on_amcl, 10)
                self._read_pose = self._amcl_holding
            else:                                   # odom, warned about above
                from nav_msgs.msg import Odometry
                self.create_subscription(
                    Odometry, str(self.get_parameter("odom_topic").value),
                    self._on_odom, 20)
                self._read_pose = lambda: None

        def _pose_from_tf(self) -> None:
            from rclpy.time import Time as RosTime
            map_frame = str(self.get_parameter("map_frame").value)
            base_frame = str(self.get_parameter("base_frame").value)
            try:
                tf = self._tf_buffer.lookup_transform(
                    map_frame, base_frame, RosTime())
            except Exception:                                 # noqa: BLE001
                # Not an error worth logging every tick: a localizer that has
                # not converged yet simply has no transform, and the adapter's
                # own silence handling already describes that state correctly.
                return
            # A transform the buffer has already answered with is not a new
            # observation. Asked for "the latest", the buffer keeps returning
            # the last transform it heard after its publisher has died, and
            # calling that a sighting every tick kept a dead robot ACTIVE and
            # freshly stamped forever -- the floor republishing a ghost the
            # adapter's silence rule could never catch (finding 10).
            if not self._tf_freshness.new(tf.header.stamp):
                return
            t = tf.transform.translation
            self.adapter.observed(t.x, t.y, _yaw_of(tf.transform.rotation))

        def _on_amcl(self, msg) -> None:
            p = msg.pose.pose
            self.adapter.observed(p.position.x, p.position.y,
                                  _yaw_of(p.orientation))

        def _amcl_holding(self) -> None:
            """
            AMCL publishes on filter updates, and those need motion, so a
            stopped robot hears nothing — which is not the same as a localizer
            that has gone. While something still publishes on the AMCL topic
            its last estimate stands, and the adapter is told so; the floor
            then keeps the robot fresh on the bus. When AMCL goes away, so does
            this, and the robot goes UNOBSERVED on its own threshold.
            """
            if self.count_publishers(self._amcl_topic) > 0:
                self.adapter.holding()

        def _on_odom(self, msg) -> None:
            p = msg.pose.pose
            self.adapter.observed(p.position.x, p.position.y,
                                  _yaw_of(p.orientation))

        # --- housekeeping ------------------------------------------------

        def _tick(self) -> None:
            self._read_pose()
            for note in self.adapter.poll_commands():
                self.get_logger().info(note)
            note = self.adapter.tick()
            if note:
                self.get_logger().info(note)
            if self._awareness is not None:
                self._awareness.tick()

    class _AwarenessOutput:
        """
        The awareness picture as nav2 reads it: two clouds and the markers.

        `~/peers/cloud` marks the peers; `~/peers/clearing` is a clearing-only
        ring at `awareness_range_m` that clears whatever has gone — measured
        for the census, a marking cloud that empties clears nothing on its
        own. Both are stamped in the base frame, because the obstacle layer
        raytraces from the cloud's frame origin. The README carries the
        obstacle-layer stanza they need.
        """

        def __init__(self, node) -> None:
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import PointCloud2, PointField
            from visualization_msgs.msg import MarkerArray

            from spatialdds_ros.awareness import Awareness, clearing_ring

            self._node = node
            self._awareness = Awareness(node.participant, node.config)
            self._PointCloud2, self._PointField = PointCloud2, PointField
            self._cloud = node.create_publisher(
                PointCloud2, "~/peers/cloud", qos_profile_sensor_data)
            self._clearing = node.create_publisher(
                PointCloud2, "~/peers/clearing", qos_profile_sensor_data)
            self._markers = node.create_publisher(
                MarkerArray, "~/peers/markers", 10)
            self._ring = clearing_ring(node.config.awareness_range_m)
            self._reported: dict = {}
            self._counts: dict = {}

        def _cloud_msg(self, points, stamp, frame):
            from spatialdds_ros.awareness import pack_xyz
            msg = self._PointCloud2()
            msg.header.stamp, msg.header.frame_id = stamp, frame
            msg.height, msg.width = 1, len(points)
            msg.fields = [self._PointField(
                name=n, offset=4 * i, datatype=self._PointField.FLOAT32,
                count=1) for i, n in enumerate("xyz")]
            msg.is_bigendian, msg.point_step = False, 12
            msg.row_step, msg.is_dense = 12 * len(points), True
            msg.data = pack_xyz(points)
            return msg

        def tick(self) -> None:
            from spatialdds_ros.awareness import to_base
            self._awareness.poll()
            robot = self._node.adapter.pose
            picture = self._awareness.picture(
                None if robot is None else robot[:2])
            self._report(picture)
            if robot is None:
                return              # no pose, no base frame to draw in
            stamp = self._node.get_clock().now().to_msg()
            base = str(self._node.get_parameter("base_frame").value)
            points = [p for o in picture.obstacles for p in o.points]
            self._cloud.publish(self._cloud_msg(
                to_base(points, *robot), stamp, base))
            self._clearing.publish(self._cloud_msg(self._ring, stamp, base))
            self._publish_markers(picture, stamp)

        def _report(self, picture) -> None:
            """Each skip reason once, when it changes; counts when they do."""
            log = self._node.get_logger()
            for entity_id, why in picture.skipped.items():
                if self._reported.get(entity_id) != why:
                    log.info(f"awareness: not drawing {entity_id}: {why}")
                    self._reported[entity_id] = why
            for entity_id in [k for k in self._reported
                              if k not in picture.skipped]:
                del self._reported[entity_id]
            counts = dict(picture.counts)
            if counts != self._counts:
                log.info("awareness: " + ", ".join(
                    f"{k} {v}" for k, v in sorted(counts.items())))
                self._counts = counts

        def _publish_markers(self, picture, stamp) -> None:
            from visualization_msgs.msg import Marker, MarkerArray
            frame = str(self._node.get_parameter("map_frame").value)
            out = MarkerArray()
            clear = Marker()
            clear.action = Marker.DELETEALL
            out.markers.append(clear)
            for index, obstacle in enumerate(picture.obstacles):
                for kind, offset in ((Marker.CYLINDER, 0), (Marker.TEXT_VIEW_FACING, 1)):
                    m = Marker()
                    m.header.stamp, m.header.frame_id = stamp, frame
                    m.ns, m.id, m.type = "peers", 2 * index + offset, kind
                    m.pose.position.x, m.pose.position.y = obstacle.x, obstacle.y
                    m.pose.orientation.w = 1.0
                    if kind == Marker.CYLINDER:
                        radius = max(math.dist(p, (obstacle.x, obstacle.y))
                                     for p in obstacle.points) or 0.05
                        m.scale.x = m.scale.y = 2 * radius
                        m.scale.z = 0.2
                        m.color.r, m.color.g, m.color.a = 1.0, 0.5, 0.6
                    else:
                        m.pose.position.z = 0.6
                        m.scale.z = 0.3
                        m.text = obstacle.label
                        m.color.r = m.color.g = m.color.b = m.color.a = 1.0
                    out.markers.append(m)
            self._markers.publish(out)

    class _Nav2Navigator:
        """nav2, asked properly and reported honestly."""

        def __init__(self, node, action_name: str) -> None:
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            self._node = node
            self._goal_type = NavigateToPose
            self._client = ActionClient(node, NavigateToPose, action_name)
            self._action_name = action_name
            # Only the latest goal may report. A goal superseded by a newer
            # one ends too, and its ending is not news about the robot.
            self._seq = 0

        def available(self) -> Optional[str]:
            if not self._client.server_is_ready():
                # One reason, said plainly. "nav2 is down" and "nav2 is up but
                # inactive" look identical from here and lead to the same
                # place: nothing will drive.
                return (f"nav2 is not accepting goals (no {self._action_name} "
                        f"server)")
            return None

        def send(self, x: float, y: float) -> str:
            goal = self._goal_type.Goal()
            goal.pose.header.frame_id = str(
                self._node.get_parameter("map_frame").value)
            goal.pose.header.stamp = self._node.get_clock().now().to_msg()
            goal.pose.pose.position.x = x
            goal.pose.pose.position.y = y
            goal.pose.pose.position.z = self._node.config.ground_z
            goal.pose.pose.orientation.w = 1.0
            self._seq += 1
            seq = self._seq
            future = self._client.send_goal_async(goal)
            future.add_done_callback(
                lambda f: self._on_response(f, seq, x, y))
            # Sent, not accepted: nav2 has not answered yet. Until 0.4.1 this
            # said "accepted", and whatever nav2 then decided went nowhere: a
            # rejected or aborted goal reached the robot's log and no client
            # (finding 11).
            return f"sent a goal at ({x:.2f}, {y:.2f}) to nav2"

        def _on_response(self, future, seq: int, x: float, y: float) -> None:
            if seq != self._seq:
                return
            try:
                handle = future.result()
            except Exception as error:                        # noqa: BLE001
                self._refuse(f"goal ({x:.2f}, {y:.2f}) could not be sent to "
                             f"nav2: {error}")
                return
            if not handle.accepted:
                self._refuse(goal_outcome(False, None, x, y))
                return
            handle.get_result_async().add_done_callback(
                lambda f: self._on_result(f, seq, x, y))

        def _on_result(self, future, seq: int, x: float, y: float) -> None:
            if seq != self._seq:
                return
            try:
                status = future.result().status
            except Exception as error:                        # noqa: BLE001
                self._refuse(f"goal ({x:.2f}, {y:.2f}): nav2 gave no result: "
                             f"{error}")
                return
            robot = self._node.adapter.pose
            distance = (None if robot is None
                        else math.hypot(robot[0] - x, robot[1] - y))
            why = goal_outcome(True, status, x, y, distance)
            if why:
                self._refuse(why)

        def _refuse(self, why: str) -> None:
            self._node.adapter.refused(why)
            self._node.get_logger().info(why)

    try:
        node = SpatialDdsRobot()
    except ConfigError as error:
        print(str(error), file=sys.stderr)
        rclpy.shutdown()
        return 2
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


class TfFreshness:
    """Whether a looked-up transform is a new observation, by its stamp.

    Compared stamp to stamp, never to a clock, so a sim on its own time and a
    robot on wall time are treated alike: a transform is new if its stamp is
    not the one the buffer answered with last time.
    """

    def __init__(self) -> None:
        self._last: Optional[tuple] = None

    def new(self, stamp) -> bool:
        key = (int(stamp.sec), int(stamp.nanosec))
        if key == self._last:
            return False
        self._last = key
        return True


#: action_msgs/GoalStatus, the three endings.
GOAL_SUCCEEDED, GOAL_CANCELED, GOAL_ABORTED = 4, 5, 6

#: How far from its goal a robot may be when nav2 reports success, before the
#: node says it was not reached (finding 12). nav2's goal checker decides
#: arrival by its own `xy_goal_tolerance`: 0.25 m by default
#: (`nav2_controller::SimpleGoalChecker`), 0.35 m on SpatialDDS-demo's tier
#: (`robot_tier/nav2_params.yaml`). The node cannot read the controller's
#: parameter, and its own pose arrives a tick later and, with
#: `pose_source odom`, in a frame that drifts from the map the goal is in. So
#: this is deliberately looser than any goal checker, and catches what was
#: measured: a preempting goal reported succeeded 21 m from where it was.
ARRIVAL_TOLERANCE_M = 1.0


def goal_outcome(accepted: bool, status: Optional[int],
                 x: float, y: float,
                 distance: Optional[float] = None) -> Optional[str]:
    """The refusal a goal's ending owes its requester, or None if it arrived.

    A goal nav2 would not take, or gave up on, travels to whoever asked on the
    robot's latched record (`RobotAdapter.refused`), as the kinematic fallback's
    refusals always have.

    So does a success reported where the robot is not. `distance` is the
    robot's own, from its last pose to the goal. A goal sent while nav2 is
    finishing the last one is a preemption, and nav2 has reported the *new*
    goal succeeded at the old goal's position in the same tick, the robot never
    moving (finding 12). With no pose there is nothing to compare, and nav2's
    word stands: the robot has no record yet for a refusal to amend.
    """
    if not accepted:
        return f"goal ({x:.2f}, {y:.2f}) rejected by nav2"
    if status == GOAL_SUCCEEDED:
        if distance is not None and distance > ARRIVAL_TOLERANCE_M:
            return (f"goal ({x:.2f}, {y:.2f}) not reached: nav2 reported "
                    f"success {distance:.1f} m from the goal")
        return None
    if status == GOAL_ABORTED:
        return (f"goal ({x:.2f}, {y:.2f}) abandoned: nav2 could not reach it "
                f"(no plan, or the controller gave up)")
    if status == GOAL_CANCELED:
        return f"goal ({x:.2f}, {y:.2f}) cancelled before it was reached"
    return f"goal ({x:.2f}, {y:.2f}) ended in nav2 status {status}"


def _yaw_of(q) -> float:
    """Yaw from a ROS quaternion message."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


if __name__ == "__main__":                                    # pragma: no cover
    sys.exit(main())
