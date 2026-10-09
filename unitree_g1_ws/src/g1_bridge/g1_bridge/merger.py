#!/usr/bin/env python3
"""
G1 统一合并器 — 系统唯一拥有 /lowcmd 的节点

整个控制架构的数据流：
  g1_ctrl (C++)   ──policy JSON──▶ /dev/shm/unitree_policy_bridge.json ──┐
  arm_ctrl_json    ──arm JSON─────▶ /dev/shm/unitree_arm_bridge.json    ──┤
                                                                          ├──▶ g1_merger ──LowCmd──▶ /lowcmd → 机器人
  (未来) head_ctrl ──head JSON───▶ /dev/shm/unitree_head_bridge.json   ──┘

合并逻辑:
  - 策略 JSON 提供所有 35 个电机的默认指令（来自 RL 策略或 FSM）
  - 各控制器 JSON 覆盖各自管辖的电机
  - 控制器 JSON 超时 (0.3s) 后自动回退到策略控制

用法:
  ros2 run g1_bridge g1_merger
"""

import json
import os
import time
import math

import rclpy
from rclpy.node import Node
from unitree_hg.msg import LowCmd, MotorCmd

POLICY_PATH = "/dev/shm/unitree_policy_bridge.json"
ARM_PATH    = "/dev/shm/unitree_arm_bridge.json"

RATE      = 500.0       # Hz
TIMEOUT   = 0.3         # arm JSON 超时 (秒)
PosStopF  = 2.146e9
VelStopF  = 16000.0
NUM_MOTORS = 35


class G1Merger(Node):
    def __init__(self):
        super().__init__("g1_merger")

        self.pub = self.create_publisher(LowCmd, "lowcmd", 10)
        self.dt = 1.0 / RATE
        self.timer = self.create_timer(self.dt, self._tick)

        # 最新的策略数据
        self.policy_data = None
        self.policy_seq  = -1

        # 最新的手臂数据 + 时间戳
        self.arm_data = None
        self.arm_seq  = -1
        self.arm_time = 0.0

        # 手臂覆盖的电机列表
        self.arm_motors = set()

        self.get_logger().info(
            f"G1 Merger ready | {RATE:.0f}Hz | "
            f"reading policy JSON + arm JSON → /lowcmd")

    def _read_json(self, path):
        """读取 JSON 文件，不存在或格式错返回 None"""
        try:
            with open(path, "r") as f:
                return json.load(f)
        except (FileNotFoundError, json.JSONDecodeError, OSError):
            return None

    def _tick(self):
        # 1. 读策略 JSON
        policy = self._read_json(POLICY_PATH)
        if policy is not None:
            self.policy_data = policy
            self.policy_seq = policy.get("seq", 0)

        # 2. 读手臂 JSON
        arm = self._read_json(ARM_PATH)
        if arm is not None:
            self.arm_data = arm
            self.arm_seq  = arm.get("seq", 0)
            self.arm_time = time.time()
            self.arm_motors = set(int(k) for k in arm.get("joints", {}).keys())

        # 3. 如果没有策略数据，不发指令
        if self.policy_data is None:
            return

        # 4. 组装 LowCmd
        msg = LowCmd()
        msg.mode_pr = self.policy_data.get("mode_pr", 0)
        msg.mode_machine = self.policy_data.get("mode_machine", 5)

        motors = self.policy_data.get("motors", {})

        # 检查手臂指令是否超时
        arm_active = (self.arm_data is not None and
                      time.time() - self.arm_time < TIMEOUT)

        for i in range(NUM_MOTORS):
            m = msg.motor_cmd[i]
            key = str(i)

            if arm_active and key in self.arm_data.get("joints", {}):
                # 手臂指令覆盖
                ac = self.arm_data["joints"][key]
                m.mode = 0x01
                m.q    = float(ac.get("q", 0.0))
                m.dq   = float(ac.get("dq", 0.0))
                m.kp   = float(ac.get("kp", 0.0))
                m.kd   = float(ac.get("kd", 0.0))
                m.tau  = float(ac.get("tau", 0.0))
            elif key in motors:
                # 策略指令
                mc = motors[key]
                m.mode = int(mc.get("mode", 0))
                m.q    = float(mc.get("q", 0.0))
                m.dq   = float(mc.get("dq", 0.0))
                m.kp   = float(mc.get("kp", 0.0))
                m.kd   = float(mc.get("kd", 0.0))
                m.tau  = float(mc.get("tau", 0.0))
            else:
                # 占位
                m.mode = 0
                m.q    = PosStopF
                m.dq   = VelStopF
                m.kp   = 0.0
                m.kd   = 0.0
                m.tau  = 0.0

        self.pub.publish(msg)

        # 心跳 1Hz
        if self.policy_seq % 500 == 0 and self.policy_seq > 0:
            arm_info = f" | arm active: {len(self.arm_motors)} joints" if arm_active else ""
            self.get_logger().info(
                f"policy seq={self.policy_seq}" + arm_info,
                throttle_duration_sec=1.0)


def main():
    rclpy.init() # 初始化 ROS 2
    node = G1Merger() # 创建节点
    try:
        rclpy.spin(node) # 进入ROS event loop，触发timer
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
