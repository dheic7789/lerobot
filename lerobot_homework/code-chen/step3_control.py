# -*- coding: utf-8 -*-
"""
EmbodiedAI Week 3 课堂练习 - 第三步
用两个 0~1 的数字同时控制两个舵机（不使用 lerobot 库）

运行前确认三件事：
  1. 第一步已完成：两个舵机的 ID 分别是 1 和 2
    2. 第二步已完成：舵机1 的角度限制是 0~614（0~180 度）
                                        舵机2 的角度限制是 0~307（0~90 度）
  3. PORT 改成你自己的串口号（Windows 形如 "COM5"，Mac 形如 /dev/cu.usbmodemXXXX）
"""

import serial

PORT = "COM5"
BAUDRATE = 1_000_000          # SCS 系列默认 1M，不要改
USE_SYNC_WRITE = True         # 若舵机不支持同步写指令，改成 False

# 每个舵机的步数范围，必须和第二步 set_range() 里设的值一致
RANGES = {
    1: (0, 307),              # 舵机1：0~90 度
    2: (0, 154),              # 舵机2：0~45 度
}

# 实测舵机1在机械零点附近反馈为3步，舵机2为2步。
# 如果重新安装舵机或重新校准后数值变化，请按 scan_bus.py 的读数修改这里。
ZERO_OFFSETS = {
    1: 3,
    2: 2,
}


def make_frame(servo_id, instruction, params):
    """按 SCS 协议拼一帧： FF FF | ID | 长度 | 指令 | 参数... | 校验和"""
    body = [servo_id, len(params) + 2, instruction] + params
    checksum = (~sum(body)) & 0xFF
    return bytes([0xFF, 0xFF, *body, checksum])


def send(ser, servo_id, instruction, params, expect=6):
    """发一帧并读掉舵机的应答。广播指令（ID=0xFE）没有应答，用 expect=0"""
    ser.reset_input_buffer()          # 清掉上一次残留的应答，防止错位
    ser.write(make_frame(servo_id, instruction, params))
    return ser.read(expect) if expect else b""


def set_torque(ser, servo_id, on):
    """0x28 扭矩开关：1 打开，0 关闭"""
    send(ser, servo_id, 0x03, [0x28, 1 if on else 0])


def ratio_to_step(servo_id, ratio):
    """核心映射：把 0~1 的比例换算成舵机的目标位置（步）"""
    lo, hi = RANGES[servo_id]
    zero = lo + ZERO_OFFSETS.get(servo_id, 0)
    return round(zero + ratio * (hi - zero))


def move_sync(ser, ratios):
    """同步写指令 0x83：一条指令让所有舵机同时转到各自的目标位置"""
    params = [0x2A, 0x02]                     # 起始地址 0x2A、每个舵机的数据长度 2
    for servo_id, ratio in ratios.items():
        target = ratio_to_step(servo_id, ratio)
        params += [servo_id, target >> 8, target & 0xFF]
    send(ser, 0xFE, 0x83, params, expect=0)   # ID 用广播地址 0xFE，舵机不回应答


def move_one_by_one(ser, ratios):
    """备用方案：用两条独立的写指令，先后发给两个舵机"""
    for servo_id, ratio in ratios.items():
        target = ratio_to_step(servo_id, ratio)
        send(ser, servo_id, 0x03, [0x2A, target >> 8, target & 0xFF])


def move(ser, ratios):
    (move_sync if USE_SYNC_WRITE else move_one_by_one)(ser, ratios)


def main():
    with serial.Serial(PORT, BAUDRATE, timeout=0.2) as ser:
        for servo_id in RANGES:
            set_torque(ser, servo_id, True)
        print("已打开两个舵机的扭矩。")
        print("输入两个 0~1 的数字，用空格分隔（例如 0.5 0.5），输入 q 退出。\n")

        while True:
            try:
                text = input("比例 (舵机1 舵机2): ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            if text.lower() in ("q", "quit", "exit", ""):
                break

            parts = text.replace(",", " ").split()
            if len(parts) != 2:
                print("  请输两个数字，例如：0.5 0.5\n")
                continue
            try:
                r1, r2 = float(parts[0]), float(parts[1])
            except ValueError:
                print("  只能输入数字\n")
                continue
            if not (0.0 <= r1 <= 1.0 and 0.0 <= r2 <= 1.0):
                print("  两个数字都要在 0~1 之间\n")
                continue

            move(ser, {1: r1, 2: r2})
            t1, t2 = ratio_to_step(1, r1), ratio_to_step(2, r2)
            print(f"  舵机1 比例 {r1:.2f} -> {t1} 步（{r1 * 180:.1f} 度）")
            print(f"  舵机2 比例 {r2:.2f} -> {t2} 步（{r2 * 90:.1f} 度）\n")


if __name__ == "__main__":
    main()
