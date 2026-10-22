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
    DEFAULT_POSE_FLOOR_HZ, Anchor, Config, ConfigError)


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
    )
    if text("source_id"):
        kwargs["source_id"] = text("source_id")
    if text("label"):
        kwargs["label"] = text("label")
    if text("type_uri"):
        kwargs["type_uri"] = text("type_uri")

    def strings(name: str) -> list:
        raw = node.get_parameter(name).value or []
        return [str(v).strip() for v in raw if str(v).strip()]

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

    class _Nav2Navigator:
        """nav2, asked properly and reported honestly."""

        def __init__(self, node, action_name: str) -> None:
            from nav2_msgs.action import NavigateToPose
            from rclpy.action import ActionClient
            self._node = node
            self._goal_type = NavigateToPose
            self._client = ActionClient(node, NavigateToPose, action_name)
            self._action_name = action_name

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
            self._client.send_goal_async(goal)
            return f"nav2 accepted a goal at ({x:.2f}, {y:.2f})"

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


def _yaw_of(q) -> float:
    """Yaw from a ROS quaternion message."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


if __name__ == "__main__":                                    # pragma: no cover
    sys.exit(main())
