# -*- coding: utf-8 -*-
"""只关闭 1~6 号舵机扭矩，不发送任何位置目标。"""

from __future__ import annotations

import argparse

import serial

import common
from common import arm_lib


def main() -> int:
    parser = argparse.ArgumentParser(description="关闭全部舵机扭矩")
    parser.add_argument("--port", default=None)
    parser.add_argument("--confirm", action="store_true")
    args = parser.parse_args()
    if not args.confirm:
        parser.error("请先托住机械臂，再添加 --confirm。")
    port = common.find_robot_port(args.port)
    if port is None:
        print("没有找到机械臂 USB 串口。")
        return 2
    with serial.Serial(port, arm_lib.BAUDRATE, timeout=0.2) as ser:
        for sid in range(1, 7):
            if not common._ping_servo(ser, sid):
                print(f"ID {sid} 无应答，停止。")
                return 1
        for sid in range(1, 7):
            arm_lib.set_torque(ser, sid, False)
        print("已对 1~6 号舵机发送扭矩关闭命令。")
        print("机械臂现在会变松，请始终托住，直到重新使能或断电。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
