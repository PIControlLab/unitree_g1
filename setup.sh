#!/bin/bash
# 以本脚本所在目录作为项目根目录，避免依赖用户的 clone 路径。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

# 删除共享内存中的旧 JSON 文件，用于清理摇杆、FSM 状态和策略选择数据。
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
# WSLg 默认可能回退到 llvmpipe 软件 OpenGL。强制 Mesa 使用 WSL 的
# D3D12 Gallium 驱动，并优先选择本机 NVIDIA 显卡用于 MuJoCo/GLFW 渲染。
# 这只加速可视化渲染；MuJoCo 动力学计算仍然运行在 CPU 上。
unset LIBGL_ALWAYS_SOFTWARE
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
# 将 CycloneDDS 网络接口限制为本机 WSL 的 eth0；lo 不支持 DDS 发现所需的组播。
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="eth0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
