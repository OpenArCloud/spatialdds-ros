#!/bin/bash
# Robot A: ROS domain 10, localized by a fake AMCL that goes quiet at (3, 0).
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=10
ros2 run tf2_ros static_transform_publisher --x 3 --frame-id map --child-frame-id base_link >/dev/null 2>&1 &
python3 -m spatialdds_ros.robot_node --ros-args -r __node:=robot_a \
  -p scene:=bare-scene -p entity_id:=ent:robot:a -p frame_id:=map/bare-scene \
  -p domain:=30 -p pose_source:=amcl &
sleep 2
python3 /work/test/ros/fake_amcl.py 3.0 0.0 &
wait
