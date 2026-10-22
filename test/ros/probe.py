"""
Ask robot B's ROS graph a question, print the answer as one JSON line.

    probe.py path GX GY     nav2's plan from B to (GX, GY)
    probe.py cloud          what B's awareness put on ~/peers/cloud
    probe.py cost X Y       the global costmap's highest cost within 0.4 m

Run with ROS_DOMAIN_ID=20. Test tooling for test_acceptance_two_domains.py.
"""
import json
import math
import struct
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.qos import (DurabilityPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)


def _wait(node, done, seconds):
    deadline = time.time() + seconds
    while time.time() < deadline and not done():
        rclpy.spin_once(node, timeout_sec=0.1)


def path(node, gx, gy):
    from nav2_msgs.action import ComputePathToPose
    client = ActionClient(node, ComputePathToPose, "compute_path_to_pose")
    if not client.wait_for_server(timeout_sec=20.0):
        return {"error": "no compute_path_to_pose server"}
    goal = ComputePathToPose.Goal()
    goal.goal.header.frame_id = "map"
    goal.goal.pose.position.x, goal.goal.pose.position.y = gx, gy
    goal.goal.pose.orientation.w = 1.0
    goal.planner_id = "GridBased"
    future = client.send_goal_async(goal)
    _wait(node, future.done, 20.0)
    handle = future.result()
    if handle is None or not handle.accepted:
        return {"error": "goal not accepted"}
    result = handle.get_result_async()
    _wait(node, result.done, 20.0)
    poses = result.result().result.path.poses
    xy = [(p.pose.position.x, p.pose.position.y) for p in poses]
    if not xy:
        return {"error": "empty path"}
    return {"n": len(xy), "max_abs_y": max(abs(y) for _x, y in xy),
            "length": sum(math.dist(xy[i], xy[i - 1])
                          for i in range(1, len(xy)))}


def cloud(node):
    from sensor_msgs.msg import PointCloud2
    got = []
    node.create_subscription(PointCloud2, "/robot_b/peers/cloud",
                             got.append, qos_profile_sensor_data)
    _wait(node, lambda: len(got) >= 3, 10.0)
    if not got:
        return {"error": "no cloud"}
    msg = got[-1]
    pts = [struct.unpack_from("<fff", msg.data, i * 12)[:2]
           for i in range(msg.width)]
    r = [math.hypot(x, y) for x, y in pts]
    return {"frame": msg.header.frame_id, "width": msg.width,
            "min_r": min(r) if r else None, "max_r": max(r) if r else None,
            "near_a": sum(1 for p in pts if math.dist(p, (3.0, 0.0)) < 0.5)}


def cost(node, x, y):
    from nav_msgs.msg import OccupancyGrid
    got = []
    node.create_subscription(OccupancyGrid, "/global_costmap/costmap",
                             got.append, QoSProfile(
                                 depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL))
    _wait(node, lambda: len(got) >= 2, 10.0)
    if not got:
        return {"error": "no costmap"}
    g = got[-1]
    res = g.info.resolution
    ci = int((x - g.info.origin.position.x) / res)
    cj = int((y - g.info.origin.position.y) / res)
    k = int(0.4 / res)
    return {"max_cost": max(g.data[j * g.info.width + i]
                            for i in range(ci - k, ci + k + 1)
                            for j in range(cj - k, cj + k + 1))}


def main():
    rclpy.init()
    node = rclpy.create_node("probe")
    what = sys.argv[1]
    if what == "path":
        out = path(node, float(sys.argv[2]), float(sys.argv[3]))
    elif what == "cloud":
        out = cloud(node)
    else:
        out = cost(node, float(sys.argv[2]), float(sys.argv[3]))
    print(json.dumps(out), flush=True)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
