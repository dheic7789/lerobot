# -*- coding: utf-8 -*-
"""
扫描总线，看看现在挂了哪些舵机、分别是什么状态。

用途：
  1. 给舵机编号时，一次只接一颗，用这个脚本确认它现在的 ID
  2. 多颗舵机串在一起以后，用这个脚本确认总线上一共几颗、ID 有没有冲突

运行： python scan_bus.py
"""

import argparse

import serial
from serial.tools import list_ports

PORT = "COM5"
BAUDRATE = 1_000_000
SCAN_FROM = 1
SCAN_TO = 20

# 与 step3_control.py 保持一致：把机械零点附近的原始反馈换算为相对位置。
ZERO_OFFSETS = {
    1: 3,
    2: 2,
}


def make_frame(servo_id, instruction, params):
    body = [servo_id, len(params) + 2, instruction] + params
    return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])


def read(ser, servo_id, address, length):
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, 0x02, [address, length]))
    r = ser.read(length + 6)
    if len(r) == length + 6 and r[0] == 0xFF and r[1] == 0xFF:
        return r[5:5 + length]
    return None


def main():
    parser = argparse.ArgumentParser(description="扫描舵机总线并读取舵机状态")
    parser.add_argument("--port", default=PORT, help=f"串口号，例如 COM6（默认：{PORT}）")
    parser.add_argument("--list-ports", action="store_true", help="列出 Windows 当前识别到的串口并退出")
    args = parser.parse_args()

    available_ports = list(list_ports.comports())
    if args.list_ports:
        if available_ports:
            for port in available_ports:
                print(f"{port.device}: {port.description}")
        else:
            print("Windows 当前没有识别到串口设备。")
        return

    try:
        ser = serial.Serial(args.port, BAUDRATE, timeout=0.3)
    except serial.SerialException as error:
        detected = ", ".join(port.device for port in available_ports) or "无"
        print(f"无法打开串口 {args.port}：{error}")
        print(f"当前识别到的串口：{detected}")
        print("请连接机械臂控制板，检查 USB 数据线和驱动，在设备管理器的“端口 (COM 和 LPT)”中确认端口号。")
        print("确认后运行：python scan_bus.py --port COM端口号")
        return

    with ser:
        found = []
        for sid in range(SCAN_FROM, SCAN_TO + 1):
            ser.reset_input_buffer()
            ser.write(make_frame(sid, 0x01, []))      # PING
            r = ser.read(6)
            if len(r) == 6 and r[0] == 0xFF and r[1] == 0xFF and r[2] == sid:
                found.append(sid)

        print(f"扫描 ID {SCAN_FROM} ~ {SCAN_TO}，串口 {args.port}（波特率 {BAUDRATE}）")
        print("-" * 58)

        if not found:
            print("没有任何舵机应答。可能的原因：")
            print("  1. 舵机没接上驱动板，或者信号线接反了")
            print("  2. 舵机没供电（舵机要单独接电源，不能只靠 USB）")
            print("  3. 舵机的 ID 不在 1~20 这个范围里")
            print("  4. 串口号不对，或者驱动板没插好")
            return

        print(f"共找到 {len(found)} 颗舵机: {found}\n")
        print(f"{'ID':<5}{'角度限制(步)':<20}{'当前位置(原始/相对)':<20}{'位置环P/D'}")
        print("-" * 58)
        for sid in found:
            lim = read(ser, sid, 0x09, 4)
            lo = (lim[0] << 8) | lim[1] if lim else None
            hi = (lim[2] << 8) | lim[3] if lim else None
            pos = read(ser, sid, 0x38, 2)
            now = (pos[0] << 8) | pos[1] if pos else None
            relative = now - ZERO_OFFSETS.get(sid, 0) if now is not None else None
            p = read(ser, sid, 0x15, 2)
            pd = f"{p[0]}/{p[1]}" if p else "?"
            position = f"{now} / {relative}"
            print(f"{sid:<5}{str(lo) + ' ~ ' + str(hi):<20}{position:<20}{pd}")

        if len(found) > 1:
            print("\n提示：多颗舵机同时在总线上时，它们的 ID 必须是唯一的。")
            print("      如果要改其中一颗的 ID，请把它单独接上再改。")


if __name__ == "__main__":
    main()
