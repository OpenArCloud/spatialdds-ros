"""
A localizer that has converged and then says nothing while the robot stands
still — what AMCL does, since its filter updates need motion. Publishes a
handful of estimates once the robot node is listening, then stays alive and
silent until interrupted.

    python3 fake_amcl.py X Y
"""
import sys
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped

rclpy.init()
node = rclpy.create_node("fake_amcl")
pub = node.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", 10)
while pub.get_subscription_count() == 0:
    rclpy.spin_once(node, timeout_sec=0.2)
for _ in range(5):
    msg = PoseWithCovarianceStamped()
    msg.header.frame_id = "map"
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.pose.pose.position.x = float(sys.argv[1])
    msg.pose.pose.position.y = float(sys.argv[2])
    msg.pose.pose.orientation.w = 1.0
    pub.publish(msg)
    time.sleep(0.5)
print("fake_amcl: converged, now silent", flush=True)
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
