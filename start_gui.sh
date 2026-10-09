#!/bin/bash
# G1 控制台启动脚本

# 以本脚本所在目录作为项目根目录，避免依赖用户的 clone 路径。
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"

echo "=== 启动 G1 控制台 ==="

# 加载项目统一的 ROS 2 Jazzy 和 DDS 环境。
source "$SCRIPT_DIR/setup.sh"

echo "启动 launcher..."
ros2 run g1_gui launcher
# g1_gui 启动后，会通过 unitree_g1_ws/src/g1_gui/g1_gui/launcher.py 启动 g1_ctrl，写入共享内存中的 JSON 文件。GUI 发送 Fixstand, Velocity, WBC, Dance 等状态切换指令，g1_ctrl 会读取这些 JSON 文件来控制 G1 机器人。
# 这份代码不会自动启动unitree_mujoco，仿真器仍需要单独启动
