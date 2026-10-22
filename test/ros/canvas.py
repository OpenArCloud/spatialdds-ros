"""A free 20 x 20 m map, latched, so the static layer has a world to hold."""
import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

rclpy.init()
node = rclpy.create_node("canvas")
pub = node.create_publisher(OccupancyGrid, "/map", QoSProfile(
    depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
    reliability=ReliabilityPolicy.RELIABLE))
grid = OccupancyGrid()
grid.header.frame_id = "map"
grid.info.resolution, grid.info.width, grid.info.height = 0.05, 400, 400
grid.info.origin.position.x = grid.info.origin.position.y = -10.0
grid.info.origin.orientation.w = 1.0
grid.data = [0] * (400 * 400)
grid.header.stamp = node.get_clock().now().to_msg()
pub.publish(grid)
rclpy.spin(node)
