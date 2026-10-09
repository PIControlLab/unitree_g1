#!/bin/bash
# 以本脚本所在目录作为项目根目录，避免依赖用户的 clone 路径。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 删除共享内存中的旧 JSON 文件，用于清理旧的摇杆、策略或手臂控制数据。
rm -f /dev/shm/unitree_*.json

echo "Setup G1 workspace (simulation)"
# 加载 ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash
# 加载官方 unitree_ros2 工作区
source "$SCRIPT_DIR/unitree_ros2/cyclonedds_ws/install/setup.bash"
# 加载本项目的 ROS 2 工作区
source "$SCRIPT_DIR/unitree_g1_ws/install/setup.bash"
# 添加 Unitree SDK2 动态库路径
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:$LD_LIBRARY_PATH
# 指定 ROS 2 使用 CycloneDDS
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# 将 CycloneDDS 网络接口限制为本机回环接口 lo
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="lo" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
