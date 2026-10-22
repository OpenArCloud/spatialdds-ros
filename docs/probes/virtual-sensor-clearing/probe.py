"""
Census probe: how nav2's ObstacleLayer treats a *virtual* sensor.

One costmap (standalone nav2_costmap_2d, global frame `map`, static map->base_link
identity), one layer under test, and a peer "robot A" that is present, then
absent, then present again. Reads the published master grid at A's cell.

Variants (argv[1]):
  cloud        PointCloud2, one source marking+clearing; absent = empty cloud
  cloud_ring   PointCloud2 marking source + separate clearing-only ring at 8 m
  scan         LaserScan, 360 rays, marking+clearing, inf_is_valid; absent = all inf
  scan_lidar   scan as above for A, plus a "real" lidar source in the SAME layer
               whose ray passes through A's cell (A present throughout) — does a
               real sensor clear the virtual obstacle?
"""
import math
import struct
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_msgs.msg import Header

VARIANT = sys.argv[1]
A = (3.0, 0.0)          # peer robot position, map frame (robot at origin)
RES, ORIGIN, SIZE = 0.1, -10.0, 200


def cloud(points, stamp):
    msg = PointCloud2()
    msg.header = Header(frame_id="base_link", stamp=stamp)
    msg.height, msg.width = 1, len(points)
    msg.fields = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32,
                             count=1) for i, n in enumerate("xyz")]
    msg.is_bigendian, msg.point_step = False, 12
    msg.row_step, msg.is_dense = 12 * len(points), True
    msg.data = b"".join(struct.pack("<fff", *p) for p in points)
    return msg


def scan(obstacle, stamp, frame="base_link"):
    msg = LaserScan()
    msg.header = Header(frame_id=frame, stamp=stamp)
    n = 360
    msg.angle_min, msg.angle_max = -math.pi, math.pi - 2 * math.pi / n
    msg.angle_increment = 2 * math.pi / n
    msg.range_min, msg.range_max = 0.05, 8.0
    ranges = [float("inf")] * n
    if obstacle:
        bearing = math.atan2(obstacle[1], obstacle[0])
        rng = math.hypot(*obstacle)
        for k in range(n):
            ang = msg.angle_min + k * msg.angle_increment
            # A disc of radius 0.3 m around the peer, nearest surface per ray
            d = abs((ang - bearing + math.pi) % (2 * math.pi) - math.pi)
            if d < math.atan2(0.3, rng):
                ranges[k] = rng - 0.3 * math.cos(d / math.atan2(0.3, rng) * math.pi / 2)
    msg.ranges = ranges
    return msg


def disc(c, r=0.3, step=0.05):
    pts = []
    k = int(r / step)
    for i in range(-k, k + 1):
        for j in range(-k, k + 1):
            if (i * step) ** 2 + (j * step) ** 2 <= r * r:
                pts.append((c[0] + i * step, c[1] + j * step, 0.2))
    return pts


def ring(r=8.0, n=720):
    return [(r * math.cos(2 * math.pi * k / n), r * math.sin(2 * math.pi * k / n),
             0.2) for k in range(n)]


class Probe(Node):
    def __init__(self):
        super().__init__("probe")
        self.grid = None
        q = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                       reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(OccupancyGrid, "/costmap/costmap",
                                 self.on_grid, q)
        self.cloud_pub = self.create_publisher(PointCloud2, "/bus_cloud", 10)
        self.ring_pub = self.create_publisher(PointCloud2, "/bus_ring", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/bus_scan", 10)
        self.lidar_pub = self.create_publisher(LaserScan, "/lidar", 10)

    def on_grid(self, msg):
        self.grid = msg

    def cost_at(self, x, y):
        if self.grid is None:
            return None
        # The highest cost within 0.4 m of the point: a laser marks the
        # peer's near surface, not its centre.
        g = self.grid
        res = g.info.resolution
        ci = int((x - g.info.origin.position.x) / res)
        cj = int((y - g.info.origin.position.y) / res)
        k = int(0.4 / res)
        return max(g.data[j * g.info.width + i]
                   for i in range(ci - k, ci + k + 1)
                   for j in range(cj - k, cj + k + 1))

    def publish(self, present):
        stamp = self.get_clock().now().to_msg()
        if VARIANT in ("cloud", "cloud_ring"):
            self.cloud_pub.publish(cloud(disc(A) if present else [], stamp))
            if VARIANT == "cloud_ring":
                self.ring_pub.publish(cloud(ring(), stamp))
        elif VARIANT == "scan":
            self.scan_pub.publish(scan(A if present else None, stamp))
        if VARIANT == "scan_lidar":
            # The virtual peer is present in every phase; only the lidar is
            # in question.
            self.scan_pub.publish(scan(A, stamp))
            # A real lidar that sees nothing in A's direction out to 8 m:
            # i.e. the peer is in another room / another domain, not physically
            # in front of this sensor. All rays max range.
            self.lidar_pub.publish(scan(None, stamp))


def phase(node, present, seconds):
    end = time.time() + seconds
    while time.time() < end:
        node.publish(present)
        rclpy.spin_once(node, timeout_sec=0.1)
    return node.cost_at(*A)


def main():
    rclpy.init()
    node = Probe()
    # wait for the costmap to come up
    end = time.time() + 30
    while node.grid is None and time.time() < end:
        node.publish(False)
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.grid is None:
        print(f"{VARIANT}: NO COSTMAP")
        return 1
    present = phase(node, True, 3.0)
    absent = phase(node, False, 3.0)
    back = phase(node, True, 3.0)
    ring_cost = node.cost_at(7.9, 0.0) if VARIANT == "cloud_ring" else None
    print(f"RESULT {VARIANT}: cost at A present={present} absent={absent} "
          f"present-again={back}" +
          (f" ring-cell={ring_cost}" if ring_cost is not None else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
