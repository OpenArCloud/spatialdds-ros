#!/bin/bash
# Robot B: ROS domain 20, at the origin, aware, with nav2's planner.
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=20
PARAMS=${NAV2_PARAMS:-/work/test/ros/nav2_b.yaml}
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link >/dev/null 2>&1 &
python3 /work/test/ros/canvas.py &
python3 -m spatialdds_ros.robot_node --ros-args -r __node:=robot_b \
  -p scene:=bare-scene -p entity_id:=ent:robot:b -p frame_id:=map/bare-scene \
  -p domain:=30 -p awareness:=true -p awareness_range_m:=8.0 &
ros2 run nav2_planner planner_server --ros-args --params-file "$PARAMS" >/tmp/planner.log 2>&1 &
sleep 3
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  -p autostart:=true -p "node_names:=[planner_server]" >/tmp/lifecycle.log 2>&1 &
wait
