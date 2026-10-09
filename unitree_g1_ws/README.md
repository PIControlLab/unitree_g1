# G1 GUI Workspace

这个 ROS 2 Jazzy 工作空间只包含 `g1_gui`，用于启动和操作
`unitree_g1_ws2` 中的 `g1_ctrl`。

核心控制器通过 Unitree SDK2/DDS 与 MuJoCo 或机器人通信；GUI 不发布 ROS 2
控制话题，而是通过 `/dev/shm/unitree_joystick_bridge.json` 向 `g1_ctrl`
提供虚拟手柄输入。

## 目录结构

```text
unitree_g1_ws/
├── src/g1_gui/
│   ├── g1_gui/launcher.py
│   ├── resource/g1_gui
│   ├── package.xml
│   ├── setup.cfg
│   └── setup.py
├── compile.sh
└── setup.sh
```

## 构建

```bash
cd unitree_g1_ws
source /opt/ros/jazzy/setup.bash
./compile.sh
```

## 启动

```bash
source setup.sh
ros2 run g1_gui launcher
```

GUI 提供 `g1_ctrl` 进程启停、FSM 操作和虚拟摇杆控制。核心命令行
Sim-to-Sim 链路不依赖这个工作空间。
