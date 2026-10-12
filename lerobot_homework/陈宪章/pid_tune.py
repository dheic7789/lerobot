# -*- coding: utf-8 -*-
"""
EmbodiedAI Week 3 - 手动调试舵机 P / D 系数的交互程序

用法：
    在「具身智能」文件夹里打开终端，运行
        python pid_tune.py
    然后按提示输入 P 和 D，程序会让舵机从起点转到目标角度，
    把整个过程的「角度 - 时间」曲线画出来，并算出关键指标。
    可以反复输入不同系数对比，输入 q 退出。

输入格式：
    P D               例：60 15
    P D 目标角度       例：60 15 45     （不填就用默认 90 度）
    直接回车           沿用上一轮的系数再跑一次
    r                 恢复最初的系数
    q                 退出（会自动把系数改回初始值）

内存表依据：
    0x15 = 位置环P比例系数，0x16 = 位置环D微分系数，取值范围都是 0~254。
    通用 SCSCL 内存表里 0x17 是"无定义"，你这颗读出来也是 0，
    所以这个实验只能调 P 和 D，没有可调的积分项。
"""

import csv
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import serial

# ------------------------- 配置区 -------------------------
PORT = "COM5"
BAUDRATE = 1_000_000
SERVO_ID = 1

STEPS_PER_DEG = 1024 / 300        # 量程按 300 度算：每度约 3.41 步
START_DEG = 6                     # 起点（出厂最小限制是 20 步≈6 度）
DEFAULT_TARGET_DEG = 90           # 默认目标角度
DURATION = 2.0                    # 每次采样时长（秒）

ADDR_P = 0x15
ADDR_D = 0x16

UNLOCK_EEPROM = False             # False：改完掉电自动恢复，实验更安全
COMBINED_PNG = "pid_tune_all.png"
COMBINED_CSV = "pid_tune_all.csv"
# ---------------------------------------------------------


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
    ser.write(make_frame(servo_id, 0x02, [0x38, 2]))
    r = ser.read(8)
    if len(r) == 8 and r[0] == 0xFF and r[1] == 0xFF and r[2] == servo_id:
        return (r[5] << 8) | r[6]
    ser.reset_input_buffer()
    return None


def set_coefficients(ser, servo_id, p, d):
    if UNLOCK_EEPROM:
        write_reg(ser, servo_id, 0x30, 0)
    write_reg(ser, servo_id, ADDR_P, p)
    write_reg(ser, servo_id, ADDR_D, d)
    if UNLOCK_EEPROM:
        write_reg(ser, servo_id, 0x30, 1)
    bp = read_reg(ser, servo_id, ADDR_P)
    bd = read_reg(ser, servo_id, ADDR_D)
    return (bp[0] if bp else None), (bd[0] if bd else None)


def goto_and_settle(ser, servo_id, deg, timeout=8.0):
    """送到起点并等它真的停稳"""
    target = deg_to_step(deg)
    send(ser, servo_id, 0x03, [0x28, 1])
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
            if stable >= 5:
                return pos
        else:
            stable = 0
        last = pos
    return last


def record_move(ser, servo_id, target_deg, duration):
    target = deg_to_step(target_deg)
    t0 = time.perf_counter()
    ser.write(make_frame(servo_id, 0x03, [0x2A, target >> 8, target & 0xFF]))
    ser.read(6)
    ts, angs = [], []
    while True:
        now = time.perf_counter() - t0
        if now > duration:
            break
        pos = read_position_step(ser, servo_id)
        if pos is not None:
            ts.append(now * 1000.0)
            angs.append(pos / STEPS_PER_DEG)
    return ts, angs


def analyze(ts, angs, target_deg, start_deg):
    if not angs:
        return {}
    span = target_deg - start_deg
    peak = max(angs)
    threshold = start_deg + 0.9 * span
    rise = next((t for t, a in zip(ts, angs) if a >= threshold), None)
    tail = angs[-max(1, len(angs) // 10):]
    steady = sum(tail) / len(tail)
    return {
        "peak": peak,
        "overshoot": (peak - target_deg) / span * 100 if span else 0.0,
        "rise_ms": rise,
        "steady": steady,
        "error": steady - target_deg,
    }


def draw(runs, target_deg):
    plt.figure(figsize=(9, 5))
    for label, ts, angs in runs:
        plt.plot(ts, angs, linewidth=1.4, label=label)
    plt.axhline(target_deg, color="gray", linestyle="--", linewidth=1, label="target")
    plt.xlabel("Time (ms)")
    plt.ylabel("Position (deg)")
    plt.title("Servo step response")
    plt.grid(alpha=0.3)
    plt.legend(fontsize=8)
    plt.tight_layout()
    plt.savefig(COMBINED_PNG, dpi=150)
    plt.close()


def main():
    with serial.Serial(PORT, BAUDRATE, timeout=0.15) as ser:
        base = read_reg(ser, SERVO_ID, ADDR_P), read_reg(ser, SERVO_ID, ADDR_D)
        if not base[0] or not base[1]:
            print("读不到舵机，检查接线、供电和串口号（当前 PORT =", PORT, "）")
            return
        base_p, base_d = base[0][0], base[1][0]

        print("=" * 52)
        print(f"舵机 {SERVO_ID}  初始系数: P = {base_p}, D = {base_d}")
        print(f"起点 {START_DEG} 度，默认目标 {DEFAULT_TARGET_DEG} 度，采样 {DURATION} 秒")
        print("输入 'P D' 或 'P D 目标角度'；回车重跑；'r' 恢复初始；'q' 退出")
        print("=" * 52)

        runs, rows = [], []
        cur_p, cur_d, cur_target = base_p, base_d, DEFAULT_TARGET_DEG

        while True:
            try:
                text = input(f"\n[P={cur_p} D={cur_d} 目标={cur_target}] 输入: ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if text.lower() in ("q", "quit", "exit"):
                break
            if text.lower() == "r":
                cur_p, cur_d = base_p, base_d
                print(f"  已恢复初始系数 P={cur_p} D={cur_d}")
                continue
            if text:
                parts = text.replace(",", " ").split()
                try:
                    if len(parts) >= 1:
                        cur_p = max(0, min(254, int(parts[0])))
                    if len(parts) >= 2:
                        cur_d = max(0, min(254, int(parts[1])))
                    if len(parts) >= 3:
                        cur_target = float(parts[2])
                except ValueError:
                    print("  格式不对，应该是：P D 或 P D 目标角度")
                    continue

            actual_p, actual_d = set_coefficients(ser, SERVO_ID, cur_p, cur_d)
            if actual_p != cur_p or actual_d != cur_d:
                print(f"  注意：写进去读回来是 P={actual_p} D={actual_d}")

            start_step = goto_and_settle(ser, SERVO_ID, START_DEG)
            start_deg = (start_step or 0) / STEPS_PER_DEG
            ts, angs = record_move(ser, SERVO_ID, cur_target, DURATION)
            if not angs:
                print("  没有采到数据，检查舵机连接")
                continue

            label = f"P={cur_p} D={cur_d}"
            runs.append((label, ts, angs))
            for t, a in zip(ts, angs):
                rows.append([label, f"{t:.3f}", f"{a:.3f}"])

            st = analyze(ts, angs, cur_target, start_deg)
            print(f"  起点 {start_deg:.1f}° -> 目标 {cur_target}° | 采到 {len(ts)} 点")
            print(f"  峰值 {st['peak']:.1f}°  超调 {st['overshoot']:+.1f}%  "
                  f"上升时间 {st['rise_ms']:.0f} ms  稳态 {st['steady']:.1f}°  "
                  f"稳态误差 {st['error']:+.1f}°")

            draw(runs, cur_target)
            print(f"  曲线已更新: {COMBINED_PNG}")

            # 每轮结束立刻把系数改回初始值。
            # 这样即使程序被强制中断（点了停止按钮），舵机也不会留在异常系数上。
            set_coefficients(ser, SERVO_ID, base_p, base_d)

        # 收尾：恢复初始系数、关扭矩
        set_coefficients(ser, SERVO_ID, base_p, base_d)
        send(ser, SERVO_ID, 0x03, [0x28, 0])
        print(f"\n已恢复初始系数 P={base_p} D={base_d}，并关闭扭矩。")

    if rows:
        with open(COMBINED_CSV, "w", newline="", encoding="utf-8-sig") as f:
            w = csv.writer(f)
            w.writerow(["run", "time_ms", "angle_deg"])
            w.writerows(rows)
        print("原始数据已保存:", COMBINED_CSV)


if __name__ == "__main__":
    main()
