#!/bin/bash
source /opt/ros/humble/setup.bash
v=$1
ros2 run tf2_ros static_transform_publisher --frame-id map --child-frame-id base_link >/dev/null 2>&1 &
ros2 run nav2_costmap_2d nav2_costmap_2d --ros-args --params-file /probe/$v.yaml >/tmp/costmap.log 2>&1 &
sleep 3
ros2 lifecycle set /costmap/costmap configure >/dev/null && ros2 lifecycle set /costmap/costmap activate >/dev/null
python3 /probe/probe.py $v
grep -iE "error|warn" /tmp/costmap.log | head -5
