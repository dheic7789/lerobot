# -*- coding: utf-8 -*-
"""将六个舵机的内部位置限位设为完整的 10 位范围。"""

import time

import serial

PORT = "COM5"
BAUDRATE = 1_000_000

# 10 位位置值为 0~1023，共 1024 个离散位置；1024 本身不是有效位置。
# 此项只设置舵机内部限制，不会覆盖 joint_ranges.json 中的机械安全范围。
RANGES = {servo_id: (0, 1023) for servo_id in range(1, 7)}


def make_frame(servo_id, instruction, params):
    body = [servo_id, len(params) + 2, instruction] + params
    return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])


def send(ser, servo_id, instruction, params, expect=6):
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, instruction, params))
    return ser.read(expect) if expect else b""


def read_register(ser, servo_id, address, length):
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, 0x02, [address, length]))
    reply = ser.read(length + 6)
    if (len(reply) != length + 6 or reply[:2] != b"\xff\xff"
            or reply[2] != servo_id or reply[3] != length + 2
            or sum(reply[2:]) & 0xFF != 0xFF or reply[4] != 0):
        return None
    return reply[5:-1]


def read_range(ser, servo_id):
    """读回 0x09~0x0C 这 4 个字节"""
    data = read_register(ser, servo_id, 0x09, 4)
    if data is None:
        return None, None
    return (data[0] << 8) | data[1], (data[2] << 8) | data[3]


def set_range(ser, servo_id, lo, hi, torque_was_enabled):
    send(ser, servo_id, 0x03, [0x28, 0])                              # 关扭矩
    try:
        send(ser, servo_id, 0x03, [0x30, 0])                          # 解锁 EEPROM
        send(ser, servo_id, 0x03, [0x09,
                                   (lo >> 8) & 0xFF, lo & 0xFF,
                                   (hi >> 8) & 0xFF, hi & 0xFF])
        time.sleep(0.05)
    finally:
        send(ser, servo_id, 0x03, [0x30, 1])                          # 上锁 EEPROM
        time.sleep(0.05)
        if torque_was_enabled:
            send(ser, servo_id, 0x03, [0x28, 1])                     # 恢复原扭矩状态


def main():
    for servo_id, (lo, hi) in RANGES.items():
        if not 0 <= lo < hi <= 1023:
            raise ValueError(f"舵机 {servo_id} 的范围无效：{lo}~{hi}")

    print("即将写入以下舵机 EEPROM 内部限位：")
    for servo_id, (lo, hi) in RANGES.items():
        print(f"  ID {servo_id}: {lo}~{hi} 步")
    print("此操作不会主动转动舵机，但会允许之后命令覆盖更大行程；请支撑机械臂并清空周围空间。")
    print("机械臂实际安全范围仍由 joint_ranges.json 限定。")
    if input("确认写入以上范围？输入 YES：").strip() != "YES":
        print("已取消，未连接串口，也未修改舵机。")
        return

    with serial.Serial(PORT, BAUDRATE, timeout=0.3) as ser:
        torque_states = {}
        for servo_id in RANGES:
            torque = read_register(ser, servo_id, 0x28, 1)
            if torque is None or torque[0] not in (0, 1):
                print(f"无法读取 ID {servo_id} 的扭矩状态；为避免部分写入，尚未修改任何舵机。")
                return
            torque_states[servo_id] = bool(torque[0])

        for servo_id, (lo, hi) in RANGES.items():
            set_range(ser, servo_id, lo, hi, torque_states[servo_id])
            back_lo, back_hi = read_range(ser, servo_id)
            ok = "OK" if (back_lo, back_hi) == (lo, hi) else "!! 不一致"
            print(f"舵机 {servo_id}: 写入 {lo} ~ {hi}  ->  读回 {back_lo} ~ {back_hi}   {ok}")

        print("\n最后再读一遍确认（这是掉电后会保留的值）：")
        for servo_id in RANGES:
            back_lo, back_hi = read_range(ser, servo_id)
            print(f"  舵机 {servo_id}: {back_lo} ~ {back_hi} 步")


if __name__ == "__main__":
    main()
