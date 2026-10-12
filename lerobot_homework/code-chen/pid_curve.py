# -*- coding: utf-8 -*-
"""
EmbodiedAI Week 3 - 第三部分末尾的实验

"编写 python 程序，让舵机转 90 度，并调整比例、微分、积分系数，
  观察舵机角度随时间变化的曲线。"

这个程序做三件事：
  1. 先读一遍舵机位置环当前的 P、D 系数，用它当基准
  2. 分别用几组不同的系数，让舵机从起点转到 90 度，
     在运动过程中高速读取当前位置，记录 (时间, 角度)
  3. 把几组曲线画在同一张图上，存成 PNG 和 CSV

注意：内存表里 0x15 是位置环P、0x16 是位置环D，
      通用 SCSCL 表里 0x17 写的是"无定义"，所以默认只调 P 和 D。
      如果你的 SCS215 的 0x17 确实有定义（有些型号是位置环I），
      把 ADDR_I 改成 0x17 即可一起调。
"""

import csv
import time

import matplotlib
matplotlib.use("Agg")            # 直接存图，不弹窗
import matplotlib.pyplot as plt
import serial

# ------------------------------------------------------------------
# 配置区
# ------------------------------------------------------------------
PORT = "COM5"
BAUDRATE = 1_000_000
SERVO_ID = 1

STEPS_PER_DEG = 1024 / 300       # 量程 300 度：每度约 3.41 步
START_DEG = 6                    # 每次实验的起点角度（出厂最小限制是 20 步≈6 度，起点取 0 会被截断）
TARGET_DEG = 90                  # 目标角度
DURATION = 2.0                   # 每次采样的时长（秒）

ADDR_P = 0x15                    # 位置环P比例系数
ADDR_D = 0x16                    # 位置环D微分系数
ADDR_I = None                    # 若你的舵机有位置环I，填它的地址，例如 0x17

UNLOCK_EEPROM = False            # False：改完掉电自动恢复默认值，实验更安全
OUTPUT_PNG = "pid_curve.png"
OUTPUT_CSV = "pid_curve.csv"


def deg_to_step(deg):
    return int(round(deg * STEPS_PER_DEG))


def make_frame(servo_id, instruction, params):
    body = [servo_id, len(params) + 2, instruction] + params
    return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])


def send(ser, servo_id, instruction, params, expect=6):
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, instruction, params))
    return ser.read(expect) if expect else b""


def read_reg(ser, servo_id, address, length=1):
    ser.write(make_frame(servo_id, 0x02, [address, length]))
    r = ser.read(length + 6)
    if len(r) == length + 6 and r[0] == 0xFF and r[1] == 0xFF:
        return r[5:5 + length]
    ser.reset_input_buffer()
    return None


def write_reg(ser, servo_id, address, value):
    send(ser, servo_id, 0x03, [address, value])


def read_position_step(ser, servo_id):
    """快速读一次当前位置（步）。读失败返回 None。"""
    ser.write(make_frame(servo_id, 0x02, [0x38, 2]))
    r = ser.read(8)
    if len(r) == 8 and r[0] == 0xFF and r[1] == 0xFF and r[2] == servo_id:
        return (r[5] << 8) | r[6]
    ser.reset_input_buffer()
    return None


def goto_and_settle(ser, servo_id, deg, timeout=8.0):
    """把舵机送到起点角度，一直等到它真的停稳（位置连续几次不变）再返回"""
    target = deg_to_step(deg)
    send(ser, servo_id, 0x03, [0x28, 1])                       # 打开扭矩
    send(ser, servo_id, 0x03, [0x2A, target >> 8, target & 0xFF])

    t0 = time.perf_counter()
    last, stable = None, 0
    while time.perf_counter() - t0 < timeout:
        time.sleep(0.05)
        pos = read_position_step(ser, servo_id)
        if pos is None:
            continue
        if last is not None and abs(pos - last) <= 2:
            stable += 1
            if stable >= 5:                    # 连续 5 次几乎不动，认为停稳了
                return pos
        else:
            stable = 0
        last = pos
    return last


def record_move(ser, servo_id, target_deg, duration):
    """发目标角度，然后按时间采样位置，返回 (时间(ms) 列表, 角度列表)"""
    target = deg_to_step(target_deg)
    t0 = time.perf_counter()
    ser.write(make_frame(servo_id, 0x03, [0x2A, target >> 8, target & 0xFF]))
    ser.read(6)

    times_ms, angles = [], []
    while True:
        now = time.perf_counter() - t0
        if now > duration:
            break
        pos = read_position_step(ser, servo_id)
        if pos is not None:
            times_ms.append(now * 1000.0)
            angles.append(pos / STEPS_PER_DEG)
    return times_ms, angles


def apply_coefficients(ser, servo_id, p, d, i=None):
    """写入 P / D（有需要时写 I）。为了写 EPROM，先解锁再上锁。"""
    if UNLOCK_EEPROM:
        write_reg(ser, servo_id, 0x30, 0)      # 解锁 EEPROM
    if p is not None:
        write_reg(ser, servo_id, ADDR_P, p)
    if d is not None:
        write_reg(ser, servo_id, ADDR_D, d)
    if i is not None and ADDR_I is not None:
        write_reg(ser, servo_id, ADDR_I, i)
    if UNLOCK_EEPROM:
        write_reg(ser, servo_id, 0x30, 1)      # 上锁


def main():
    with serial.Serial(PORT, BAUDRATE, timeout=0.15) as ser:
        base_p = read_reg(ser, SERVO_ID, ADDR_P)
        base_d = read_reg(ser, SERVO_ID, ADDR_D)
        if base_p is None or base_d is None:
            print("读不到舵机，请检查接线、供电和串口号。")
            return
        base_p, base_d = base_p[0], base_d[0]
        print(f"舵机 {SERVO_ID} 当前位置环系数: P = {base_p}, D = {base_d}")
        if ADDR_I is not None:
            base_i = read_reg(ser, SERVO_ID, ADDR_I)
            print("位置环 I =", base_i[0] if base_i else "读不到")

        # 用当前值当基准，避开展不开的情况
        ref_p = base_p if base_p > 0 else 32
        ref_d = base_d if base_d > 0 else 32

        trials = [
            ("default", base_p, base_d),
            ("P x 4", min(254, ref_p * 4), base_d),
            ("P / 4", max(1, ref_p // 4), base_d),
            ("D x 4", base_p, min(254, ref_d * 4)),
        ]

        all_data = []
        for name, p, d in trials:
            apply_coefficients(ser, SERVO_ID, p, d)
            back_p = read_reg(ser, SERVO_ID, ADDR_P)
            back_d = read_reg(ser, SERVO_ID, ADDR_D)
            shown = f"P={back_p[0] if back_p else '?'} D={back_d[0] if back_d else '?'}"

            goto_and_settle(ser, SERVO_ID, START_DEG)
            ts, angs = record_move(ser, SERVO_ID, TARGET_DEG, DURATION)
            rate = len(ts) / DURATION if ts else 0
            print(f"  {name:9s} 实际写入 {shown} | 采到 {len(ts)} 个点，约 {rate:.0f} Hz")
            all_data.append((name, shown, ts, angs))

        # 恢复原始系数
        apply_coefficients(ser, SERVO_ID, base_p, base_d)
        send(ser, SERVO_ID, 0x03, [0x28, 0])
        print("已恢复原始系数并关闭扭矩。")

    # ---------------- 画图 ----------------
    plt.figure(figsize=(9, 5))
    for name, shown, ts, angs in all_data:
        plt.plot(ts, angs, linewidth=1.4, label=f"{name} ({shown})")
    plt.axhline(TARGET_DEG, color="gray", linestyle="--", linewidth=1, label="target")
    plt.xlabel("Time (ms)")
    plt.ylabel("Position (deg)")
    plt.title(f"Servo step response to {TARGET_DEG} deg")
    plt.grid(alpha=0.3)
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(OUTPUT_PNG, dpi=150)
    print("曲线图已保存:", OUTPUT_PNG)

    with open(OUTPUT_CSV, "w", newline="", encoding="utf-8-sig") as f:
        writer = csv.writer(f)
        writer.writerow(["trial", "coefficients", "time_ms", "angle_deg"])
        for name, shown, ts, angs in all_data:
            for t, a in zip(ts, angs):
                writer.writerow([name, shown, f"{t:.3f}", f"{a:.3f}"])
    print("原始数据已保存:", OUTPUT_CSV)


if __name__ == "__main__":
    main()
