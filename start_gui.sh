#!/bin/bash
# G1 控制台启动脚本

echo "=== 启动 G1 控制台 ==="

source /opt/ros/foxy/setup.bash

if [ -f ~/unitree_g1/unitree_ros2/cyclonedds_ws/install/setup.bash ]; then
    source ~/unitree_g1/unitree_ros2/cyclonedds_ws/install/setup.bash
fi

source ~/unitree_g1/unitree_g1_ws/install/setup.bash

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"lo\" priority=\"default\" multicast=\"default\" /></Interfaces></General></Domain></CycloneDDS>"

echo "启动 launcher..."
ros2 run g1_gui launcher
