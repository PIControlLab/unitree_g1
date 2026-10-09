#!/bin/bash
SCRIPT_DIR="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"
echo "Setup G1 controller workspace (REAL ROBOT - WiFi)"
echo "Usage: source setup_real.sh [interface_name]  (default: eth0)"
echo "Robot IP: 192.168.12.103"

IFACE=${1:-eth0}

source /opt/ros/jazzy/setup.bash
source /home/ningwt/unitree_g1/unitree_ros2/cyclonedds_ws/install/setup.bash
source $SCRIPT_DIR/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export ROS_DOMAIN_ID=0
export CYCLONEDDS_URI='<CycloneDDS><Domain>
				<General>
					<AllowMulticast>false</AllowMulticast>
					<Interfaces>
						<NetworkInterface name="'$IFACE'" />
					</Interfaces>
				</General>
				<Discovery>
					<Peers>
						<Peer Address="192.168.12.103"/>
					</Peers>
				</Discovery>
			</Domain></CycloneDDS>'

echo "  Interface : $IFACE"
echo "  Robot IP  : 192.168.12.103"
echo "  Domain ID : 0"
