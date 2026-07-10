#!/bin/bash
rm -f /dev/shm/unitree_*.json

echo "Setup G1 workspace (simulation)"
source /opt/ros/jazzy/setup.bash
source ~/unitree_g1/unitree_ros2/cyclonedds_ws/install/setup.bash
source ~/unitree_g1/unitree_g1_ws/install/setup.bash
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
