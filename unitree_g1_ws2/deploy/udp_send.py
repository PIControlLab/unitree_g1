#!/usr/bin/env python3
"""
UDP 动作触发器 — 在另一台电脑上运行。
输入数字 1-9，通过 UDP 发送到 G1 控制台触发对应动作。

用法:
  python3 udp_send.py <目标IP> [端口]
  python3 udp_send.py 192.168.1.100       # 默认端口 9876
  python3 udp_send.py 192.168.1.100 9999  # 自定义端口

动作映射:
  1 = 前进      5 = 左转弯
  2 = 后退      6 = 右转弯
  3 = 左移      7 = WBC
  4 = 右移      8 = 跳舞
               9 = UFO
  0 / q = 退出
"""

import socket
import sys


def main():
    if len(sys.argv) < 2:
        print("用法: python3 udp_send.py <目标IP> [端口]")
        print("示例: python3 udp_send.py 192.168.1.100")
        sys.exit(1)

    target_ip = sys.argv[1]
    target_port = int(sys.argv[2]) if len(sys.argv) > 2 else 9876

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print(f"UDP 发送端已就绪 → {target_ip}:{target_port}")
    print("─" * 40)
    print("  1=前进  2=后退  3=左移  4=右移")
    print("  5=左转弯 6=右转弯 7=WBC   8=跳舞  9=UFO")
    print("  0 / q = 退出")
    print("─" * 40)

    while True:
        try:
            cmd = input("输入数字 (1-9): ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n退出。")
            break

        if cmd.lower() in ("0", "q", "quit", "exit"):
            print("退出。")
            break

        if cmd not in ("1", "2", "3", "4", "5", "6", "7", "8", "9"):
            print("  无效输入，请输入 1-9")
            continue

        sock.sendto(cmd.encode(), (target_ip, target_port))
        actions = {
            "1": "前进", "2": "后退", "3": "左移", "4": "右移",
            "5": "左转弯", "6": "右转弯", "7": "WBC", "8": "跳舞", "9": "UFO",
        }
        print(f"  → 已发送: {actions[cmd]}")

    sock.close()


if __name__ == "__main__":
    main()
