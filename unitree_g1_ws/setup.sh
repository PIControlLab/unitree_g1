#!/bin/bash
rm -f /dev/shm/unitree_*.json

SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
echo "Setup G1 controller workspace (simulation)"
source /opt/ros/jazzy/setup.bash
source "$SCRIPT_DIR/../unitree_ros2/cyclonedds_ws/install/setup.bash"
source "$SCRIPT_DIR/install/setup.bash"
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:$LD_LIBRARY_PATH
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
