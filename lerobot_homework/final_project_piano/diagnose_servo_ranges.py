# -*- coding: utf-8 -*-
"""只读检查 1~6 号舵机的内部限位、当前位置、扭矩和电压。

本脚本不会使能扭矩、不会改变目标位置、不会写入 EEPROM。
"""

from __future__ import annotations

import argparse

import serial

import common
from common import arm_lib


def read_register(ser, servo_id: int, address: int, length: int):
    ser.reset_input_buffer()
    ser.write(arm_lib.make_frame(servo_id, 0x02, [address, length]))
    reply = ser.read(length + 6)
    if (
        len(reply) != length + 6
        or reply[:2] != b"\xff\xff"
        or reply[2] != servo_id
        or reply[3] != length + 2
        or reply[4] != 0
        or sum(reply[2:]) & 0xFF != 0xFF
    ):
        return None
    return reply[5:-1]


def read_internal_limits(ser, servo_id: int):
    data = read_register(ser, servo_id, 0x09, 4)
    if data is None:
        return None
    return (data[0] << 8) | data[1], (data[2] << 8) | data[3]


def main() -> int:
    parser = argparse.ArgumentParser(description="只读检查舵机量程")
    parser.add_argument("--port", default=None, help="机械臂串口，例如 COM5；不填则自动查找 USB 串口")
    parser.add_argument("--ranges-file", default=str(common.DEFAULT_RANGES))
    args = parser.parse_args()

    port = common.find_robot_port(args.port)
    if port is None:
        print("没有找到机械臂 USB 串口。当前无法在线检查 4/5/6 号舵机。")
        print("请接上机械臂控制板，再用设备管理器确认端口号。")
        return 2

    ranges = common.load_json(common.Path(args.ranges_file))
    print(f"使用 {port}，只读检查，不发送运动指令。")
    try:
        ser = serial.Serial(port, arm_lib.BAUDRATE, timeout=0.25)
    except serial.SerialException as error:
        print(f"打不开串口 {port}：{error}")
        return 2

    with ser:
        print(
            f"{'ID':<4}{'内部限位':<16}{'当前位置':<10}"
            f"{'软件范围':<16}{'扭矩':<6}{'电压':<6}{'检查'}"
        )
        print("-" * 76)
        problems = []
        warnings = []
        for sid in range(1, 7):
            if not common._ping_servo(ser, sid):
                print(f"{sid:<4}{'无应答':<16}")
                problems.append(f"ID {sid} 无应答")
                continue
            try:
                position = arm_lib.read_position(ser, sid)
            except Exception:
                position = None
            limits = read_internal_limits(ser, sid)
            torque = arm_lib.read_torque(ser, sid)
            voltage = arm_lib.read_voltage(ser, sid)
            low = int(ranges[str(sid)]["min_step"])
            high = int(ranges[str(sid)]["max_step"])
            checks = []
            if position is None:
                checks.append("位置读不到")
            elif not low <= position <= high:
                checks.append("当前位置超出软件范围")
            if limits is not None and not (limits[0] <= low < high <= limits[1]):
                checks.append("软件范围超出舵机内部限位")
            if sid in (4, 5, 6) and position is not None:
                edge_gap = min(position - low, high - position)
                if edge_gap < 20:
                    warnings.append(f"ID {sid}: 距软件边界不足20步（请确认不是异常偏位）")
            if torque == 0:
                warnings.append(f"ID {sid}: 扭矩关闭（手动标定后正常）")
            if voltage is not None and voltage < 5.8:
                checks.append("电压低")
            status = "正常" if not checks else "；".join(checks)
            if checks:
                problems.append(f"ID {sid}: {status}")
            internal = "-" if limits is None else f"{limits[0]}~{limits[1]}"
            print(
                f"{sid:<4}{internal:<16}{str(position):<10}"
                f"{f'{low}~{high}':<16}{str(torque):<6}"
                f"{'-' if voltage is None else f'{voltage:.1f}V':<6}{status}"
            )

    print("\n结论：")
    if warnings:
        print("提示：")
        for item in warnings:
            print(f"  - {item}")
    if problems:
        print("发现以下阻止运动的问题：")
        for item in problems:
            print(f"  - {item}")
        print("\n请先把当前位置移回软件范围，并确认新端点留有安全余量。")
        return 1
    print("六颗舵机当前位置都在软件范围内。扭矩关闭是安全待机状态。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
