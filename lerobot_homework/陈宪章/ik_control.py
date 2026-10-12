# -*- coding: utf-8 -*-
"""SCS215 六关节机械臂 · 逆运动学(IK)控制（第 4 周 1.2 任务）

运动学与角度换算按 lerobot 的 SO-101 方案实现：

1. 正运动学用 SO101/so101_new_calib.urdf 的完整连杆链，末端取
   `gripper_frame_link`，和 lerobot 的 RobotKinematics 目标坐标系一致。
2. 舵机步数 <-> 关节角换算：
       angle_deg = (step - midpoint) * 300 / (resolution - 1)
   其中 midpoint = (min_step + max_step) / 2，resolution = 1024。
   SCS215 是 10bit、300° 位置范围（不是 STS3215 的 12bit/360°）。
3. 逆运动学用数值雅可比（阻尼最小二乘），只解 shoulder_pan / shoulder_lift /
   elbow_flex 三个关节，wrist_flex / wrist_roll / gripper 保持不动。

这样末端走直线时三个关节会同时运动，不会出现“只有一个关节转、末端走斜”的问题。
"""

import argparse
import json
import math
import os
import re
from pathlib import Path


PORT = "COM5"
BAUDRATE = 1_000_000
RANGES_FILE = Path(__file__).with_name("joint_ranges.json")
DEFAULT_DURATION = 2.0
DEFAULT_SPEED = 100
DEFAULT_STEP = 0.01
POSITION_MODULUS = 1 << 10

RESOLUTION = 1024
MAX_RES = RESOLUTION - 1.0
SCS215_FULL_RANGE_DEG = 300.0
RAD_PER_STEP = math.radians(SCS215_FULL_RANGE_DEG) / MAX_RES

JOINT_BY_ID = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}
JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
ACTIVE_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex"]
ACTIVE_IDS = [1, 2, 3]
HELD_IDS = [4, 5, 6]

# SO101 URDF 连杆链（onshape-to-robot 生成，关节轴均为子连杆局部 z 轴）
CHAIN = [
    ("base_link", None, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), None),
    ("shoulder_link", "base_link",
     (0.0388353, -8.97657e-09, 0.0624), (3.14159, 4.18253e-17, -3.14159), "shoulder_pan"),
    ("upper_arm_link", "shoulder_link",
     (-0.0303992, -0.0182778, -0.0542), (-1.5708, -1.5708, 0.0), "shoulder_lift"),
    ("lower_arm_link", "upper_arm_link",
     (-0.11257, -0.028, 1.73763e-16), (-3.63608e-16, 8.74301e-16, 1.5708), "elbow_flex"),
    ("wrist_link", "lower_arm_link",
     (-0.1349, 0.0052, 3.62355e-17), (4.02456e-15, 8.67362e-16, -1.5708), "wrist_flex"),
    ("gripper_link", "wrist_link",
     (5.55112e-17, -0.0611, 0.0181), (1.5708, 0.0486795, 3.14159), "wrist_roll"),
    ("gripper_frame_link", "gripper_link",
     (-0.0079, -0.000218121, -0.0981274), (0.0, 3.14159, 0.0), None),
]


# ---------------------------------------------------------------------------
# SCS 协议基础（与 six_joint_control.py 一致）
# ---------------------------------------------------------------------------
def make_frame(servo_id, instruction, params):
    body = [servo_id, len(params) + 2, instruction] + params
    return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])


def send(ser, servo_id, instruction, params, expect=6):
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, instruction, params))
    return ser.read(expect) if expect else b""


def set_torque(ser, servo_id, enabled):
    send(ser, servo_id, 0x03, [0x28, 1 if enabled else 0])


def read_position(ser, servo_id):
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, 0x02, [0x38, 2]))
    reply = ser.read(8)
    if (len(reply) != 8 or reply[:2] != b"\xff\xff"
            or reply[2] != servo_id or reply[3] != 4
            or reply[4] != 0 or sum(reply[2:]) & 0xFF != 0xFF):
        raise RuntimeError(f"ID {servo_id} 读取当前位置失败。")
    return (reply[5] << 8) | reply[6]


def sync_move(ser, targets, duration, speed):
    if not math.isfinite(duration) or not 0 < duration <= 65.535:
        raise ValueError("切换时间必须大于 0 且不超过 65.535 秒。")
    if not isinstance(speed, int) or isinstance(speed, bool) or not 1 <= speed <= 0xFFFF:
        raise ValueError("速度寄存器值必须是 1~65535 的整数。")
    duration_ms = round(duration * 1000)
    if not 1 <= duration_ms <= 0xFFFF:
        raise ValueError("切换时间换算成毫秒后必须在 1~65535 之间。")

    params = [0x2A, 0x06]
    for servo_id in sorted(targets):
        pos = targets[servo_id]
        if (not isinstance(pos, int) or isinstance(pos, bool)
                or not 0 <= pos < POSITION_MODULUS):
            raise ValueError(f"ID {servo_id} 的目标位置无效。")
        params += [
            servo_id,
            pos >> 8, pos & 0xFF,
            duration_ms >> 8, duration_ms & 0xFF,
            speed >> 8, speed & 0xFF,
        ]
    send(ser, 0xFE, 0x83, params, expect=0)


# ---------------------------------------------------------------------------
# 校准范围与步数 <-> 关节角
# ---------------------------------------------------------------------------
def load_ranges(path=RANGES_FILE):
    if not path.exists():
        raise ValueError(f"未找到校准文件 {path.name}，请先完成关节范围标定。")
    data = json.loads(path.read_text(encoding="utf-8"))
    ranges = {}
    for sid in "123456":
        if sid not in data:
            raise ValueError(f"{path.name} 缺少 ID {sid} 的校准范围。")
        item = data[sid]
        low, high = item.get("min_step"), item.get("max_step")
        if (not isinstance(low, int) or isinstance(low, bool)
                or not isinstance(high, int) or isinstance(high, bool)
                or not 0 <= low < high < POSITION_MODULUS):
            raise ValueError(f"{path.name} 中 ID {sid} 的步数范围无效。")
        ranges[int(sid)] = (low, high)
    return ranges


def position_in_range(position, low, high):
    return low <= position <= high


def step_to_rad(step, mid):
    return (step - mid) * RAD_PER_STEP


def rad_to_step(rad, mid):
    return int(round(rad / RAD_PER_STEP + mid))


def ranges_midpoints(ranges):
    return {sid: (low + high) / 2.0 for sid, (low, high) in ranges.items()}


def ranges_limits_rad(ranges, mids):
    return {
        sid: (step_to_rad(low, mids[sid]), step_to_rad(high, mids[sid]))
        for sid, (low, high) in ranges.items()
    }


# ---------------------------------------------------------------------------
# 正运动学（SO101 URDF）
# ---------------------------------------------------------------------------
def _rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return ((1, 0, 0), (0, c, -s), (0, s, c))


def _rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, 0, s), (0, 1, 0), (-s, 0, c))


def _rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return ((c, -s, 0), (s, c, 0), (0, 0, 1))


def _matmul(a, b):
    return tuple(
        tuple(sum(a[i][k] * b[k][j] for k in range(3)) for j in range(3))
        for i in range(3)
    )


def _matvec(m, v):
    return tuple(sum(m[i][j] * v[j] for j in range(3)) for i in range(3))


def _rpy_to_rot(r, p, y):
    return _matmul(_matmul(_rot_z(y), _rot_y(p)), _rot_x(r))


def fk_position(joint_rad):
    """返回末端 gripper_frame_link 在 base_link 系下的位置 (x, y, z)，单位米。"""
    pose = {
        "base_link": ((0.0, 0.0, 0.0), ((1, 0, 0), (0, 1, 0), (0, 0, 1)))
    }
    for name, parent, xyz, rpy, joint in CHAIN[1:]:
        pp, pr = pose[parent]
        rot = _rpy_to_rot(*rpy)
        if joint is not None:
            rot = _matmul(rot, _rot_z(joint_rad[joint]))
        world_rot = _matmul(pr, rot)
        world_pos = tuple(_matvec(pr, xyz)[i] + pp[i] for i in range(3))
        pose[name] = (world_pos, world_rot)
    return pose["gripper_frame_link"][0]


def current_end_position(joint_rad):
    return fk_position(joint_rad)


# ---------------------------------------------------------------------------
# 逆运动学（数值雅可比 + 阻尼最小二乘）
# ---------------------------------------------------------------------------
def _solve3(a, b):
    m = [a[i][:] + [b[i]] for i in range(3)]
    n = 3
    for col in range(n):
        pivot = max(range(col, n), key=lambda r: abs(m[r][col]))
        m[col], m[pivot] = m[pivot], m[col]
        if abs(m[col][col]) < 1e-12:
            return [0.0, 0.0, 0.0]
        for r in range(col + 1, n):
            factor = m[r][col] / m[col][col]
            for c in range(col, n + 1):
                m[r][c] -= factor * m[col][c]
    x = [0.0] * n
    for r in range(n - 1, -1, -1):
        s = m[r][n]
        for c in range(r + 1, n):
            s -= m[r][c] * x[c]
        x[r] = s / m[r][r]
    return x


def _numeric_jacobian(joint_rad, active):
    eps = 1e-6
    base = fk_position(joint_rad)
    jac = [[0.0] * len(active) for _ in range(3)]
    for j, name in enumerate(active):
        joint_rad[name] += eps
        p = fk_position(joint_rad)
        joint_rad[name] -= eps
        for i in range(3):
            jac[i][j] = (p[i] - base[i]) / eps
    return jac


def inverse_kinematics(target, current_rad, limits):
    """对 shoulder_pan/shoulder_lift/elbow_flex 做位置 IK，其余关节保持 current_rad。"""
    q = dict(current_rad)
    damping = 1e-3
    max_step = 0.15
    for _ in range(120):
        pos = fk_position(q)
        err = [target[i] - pos[i] for i in range(3)]
        if math.sqrt(sum(e * e for e in err)) < 1e-5:
            break
        jac = _numeric_jacobian(q, ACTIVE_JOINTS)
        a = [[0.0] * 3 for _ in range(3)]
        for i in range(3):
            for j in range(3):
                a[i][j] = sum(jac[i][k] * jac[j][k] for k in range(3))
            a[i][i] += damping
        x = _solve3(a, err)
        dq = [sum(jac[i][j] * x[i] for i in range(3)) for j in range(3)]
        scale = max((abs(v) for v in dq), default=0.0)
        if scale > max_step:
            dq = [v * max_step / scale for v in dq]
        for j, name in enumerate(ACTIVE_JOINTS):
            sid = ACTIVE_IDS[j]
            lo, hi = limits[sid]
            q[name] = max(lo, min(hi, q[name] + dq[j]))
    pos = fk_position(q)
    err = [target[i] - pos[i] for i in range(3)]
    note = ""
    residual = math.sqrt(sum(e * e for e in err))
    if residual > 5e-3:
        note = (
            f"IK 无解：目标超出可达范围，已沿原方向移动到最近可达点"
            f"（本次还差 {residual * 100:.1f} cm）"
        )
    return q, note


# ---------------------------------------------------------------------------
# 命令解析
# ---------------------------------------------------------------------------
def direction_to_vector(direction, distance):
    unit = {
        "forward": (1.0, 0.0, 0.0),
        "f": (1.0, 0.0, 0.0),
        "back": (-1.0, 0.0, 0.0),
        "b": (-1.0, 0.0, 0.0),
        "left": (0.0, 1.0, 0.0),
        "l": (0.0, 1.0, 0.0),
        "right": (0.0, -1.0, 0.0),
        "r": (0.0, -1.0, 0.0),
        "up": (0.0, 0.0, 1.0),
        "u": (0.0, 0.0, 1.0),
        "down": (0.0, 0.0, -1.0),
        "d": (0.0, 0.0, -1.0),
    }.get(direction.lower())
    if unit is None:
        raise ValueError("方向必须是 forward/back/left/right/up/down 之一。")
    return tuple(distance * value for value in unit)


def show_text_help(step):
    print("输入运动命令（回车执行），q 退出：")
    print("  up / down / forward / back / left / right [距离]    例：up 0.05")
    print(f"  方向后面不写距离时，用 --step（默认 {step * 100:g} cm）")
    print("  move dx dy dz       相对位移（米），例：move 0 0.05 0")
    print("  goto x y z          绝对坐标（米，base 系），例：goto 0.39 0 0.23")
    print("  home / 回中          回到六个关节的校准中点")
    print("  也可用中文，例：向上移动5厘米 / 向前3厘米 / 左移2厘米")
    print("  i                   显示当前位置")
    print("  help                显示本帮助")


def parse_command(text, step):
    parts = text.strip().lower().split()
    if not parts:
        return ("error", "请输入命令，help 查看用法")
    cmd = parts[0]
    if cmd in ("q", "quit", "exit"):
        return ("quit",)
    if cmd in ("i", "info"):
        return ("info",)
    if cmd in ("h", "help", "?"):
        return ("help",)
    if cmd in ("home", "回中", "归零", "复位", "初始"):
        return ("home",)
    if cmd in ("up", "down", "forward", "back", "left", "right"):
        if len(parts) > 2:
            return ("error", "方向命令最多带一个距离，例如 up 0.05")
        dist = step
        if len(parts) == 2:
            try:
                dist = float(parts[1])
            except ValueError:
                return ("error", "距离必须是数字")
        if not math.isfinite(dist) or abs(dist) > 0.5:
            return ("error", "距离必须是 0~0.5 米之间的数字")
        return ("move_dir", cmd, dist)
    if cmd == "move":
        if len(parts) != 4:
            return ("error", "用法：move dx dy dz")
        try:
            vals = [float(p) for p in parts[1:4]]
        except ValueError:
            return ("error", "位移必须是数字")
        if any(not math.isfinite(v) or abs(v) > 0.5 for v in vals):
            return ("error", "每个位移都必须是 0~0.5 米之间的数字")
        return ("move_vec", vals[0], vals[1], vals[2])
    if cmd == "goto":
        if len(parts) != 4:
            return ("error", "用法：goto x y z")
        try:
            vals = [float(p) for p in parts[1:4]]
        except ValueError:
            return ("error", "坐标必须是数字")
        if any(not math.isfinite(v) for v in vals):
            return ("error", "坐标必须是数字")
        return ("goto", vals[0], vals[1], vals[2])
    return parse_chinese_motion(text, step)


def parse_chinese_motion(text, step):
    direction_map = {
        "向上": "up", "往上": "up", "上升": "up", "抬高": "up", "上": "up",
        "向下": "down", "往下": "down", "下降": "down", "降低": "down", "下": "down",
        "向前": "forward", "往前": "forward", "前进": "forward", "前": "forward",
        "向后": "back", "往后": "back", "后退": "back", "后": "back",
        "向左": "left", "往左": "left", "左": "left",
        "向右": "right", "往右": "right", "右": "right",
    }
    keyword = None
    for word, direction in direction_map.items():
        if word in text and (keyword is None or len(word) > len(keyword[0])):
            keyword = (word, direction)
    if keyword is None:
        return ("error", "未识别的命令，输入 help 查看用法")
    match = re.search(r"(-?\d+(?:\.\d+)?)\s*(厘米|公分|cm|米|m)?", text)
    if match is None:
        return ("move_dir", keyword[1], step)
    value = float(match.group(1))
    unit = (match.group(2) or "").lower()
    if unit in ("厘米", "公分", "cm"):
        value /= 100.0
    if not math.isfinite(value) or abs(value) > 0.5:
        return ("error", "距离必须是 0~0.5 米之间的数字")
    return ("move_dir", keyword[1], value)


def show_keyboard_help(step):
    print("键盘控制末端（关节角由 IK 计算）：")
    print("  w/s 前/后，a/d 左/右，r/f 上/下")
    print("  方向键 ↑↓←→ 也可用；q 退出")
    print(f"  每按一次移动 {step * 100:g} cm")


def read_key():
    if os.name != "nt":
        line = input("键盘> ").strip().lower()
        return line[:1] if line else ""
    import msvcrt
    key = msvcrt.getwch()
    if key in ("\x00", "\xe0"):
        code = msvcrt.getwch()
        return {"H": "up", "P": "down", "K": "left", "M": "right"}.get(code, "")
    return key.lower()


# ---------------------------------------------------------------------------
# 主流程
# ---------------------------------------------------------------------------
def home_steps(ranges):
    return {sid: round((low + high) / 2.0) for sid, (low, high) in ranges.items()}


def read_real_steps(ser, ranges):
    steps = {}
    for sid in sorted(ranges):
        position = read_position(ser, sid)
        low, high = ranges[sid]
        if not position_in_range(position, low, high):
            raise RuntimeError(
                f"ID {sid} 当前 {position} 步，超出标定范围 {low}~{high}；未使能任何舵机。"
            )
        steps[sid] = position
    return steps


def steps_to_joint_rad(steps, mids):
    return {
        JOINT_BY_ID[sid]: step_to_rad(steps[sid], mids[sid])
        for sid in range(1, 6)
    }


def report_plan(steps, joint_rad, achieved, duration, speed, dry_run):
    x, y, z = achieved
    print(f"  末端参考点位置: x={x:+.3f} m, y={y:+.3f} m, z={z:+.3f} m")
    print("  关节角: " + ", ".join(
        f"{JOINT_BY_ID[sid]}={math.degrees(joint_rad[JOINT_BY_ID[sid]]):+.1f}°"
        for sid in range(1, 6)
    ))
    for sid in sorted(steps):
        print(f"  {JOINT_BY_ID[sid]:<14} {steps[sid]} 步")
    if dry_run:
        print("  --dry-run：仅预览，未向舵机发送指令。")


def main():
    parser = argparse.ArgumentParser(
        description="SCS215 六关节机械臂逆运动学(IK)控制（1.2 任务）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="示例: python ik_control.py --dry-run --move up 0.05",
    )
    parser.add_argument("--port", default=PORT, help=f"串口号（默认 {PORT}）")
    parser.add_argument("--baudrate", type=int, default=BAUDRATE,
                        help=f"波特率（默认 {BAUDRATE}）")
    parser.add_argument("--dry-run", action="store_true",
                        help="不连接舵机，只预览 IK 计算结果")
    parser.add_argument("--selftest", action="store_true",
                        help="离线自测 IK 数学逻辑（无需硬件）")
    parser.add_argument("--step", type=float, default=DEFAULT_STEP,
                        help=f"方向命令省略距离时的默认步长，单位米（默认 {DEFAULT_STEP:g}）")
    parser.add_argument("--move", nargs=2, metavar=("方向", "距离"),
                        help="一次性让末端沿指定方向移动指定距离（米）")
    parser.add_argument("--goto", nargs=3, type=float, metavar=("x", "y", "z"),
                        help="一次性让末端直接移动到绝对坐标（米，base 系）")
    parser.add_argument("--mode", choices=["text", "keyboard"], default="text",
                        help="交互模式：text=文字/数字命令，keyboard=键盘按键（默认 text）")
    parser.add_argument("--duration", type=float, default=DEFAULT_DURATION,
                        help=f"目标运动时间，单位秒（默认 {DEFAULT_DURATION:g}）")
    parser.add_argument("--speed", type=int, default=DEFAULT_SPEED,
                        help=f"速度寄存器值（默认 {DEFAULT_SPEED}）")
    args = parser.parse_args()

    if args.selftest:
        selftest()
        return

    if not math.isfinite(args.step) or not 0 < args.step <= 0.5:
        parser.error("--step 必须是 0~0.5 米之间的正数。")
    if not 1 <= args.speed <= 0xFFFF:
        parser.error("--speed 必须是 1~65535 的整数。")
    if not math.isfinite(args.duration) or not 0 < args.duration <= 65.535:
        parser.error("--duration 必须大于 0 且不超过 65.535 秒。")

    try:
        ranges = load_ranges()
    except ValueError as error:
        parser.error(str(error))

    mids = ranges_midpoints(ranges)
    limits = ranges_limits_rad(ranges, mids)

    ser = None
    try:
        if args.dry_run:
            steps = home_steps(ranges)
        else:
            import serial
            ser = serial.Serial(args.port, args.baudrate, timeout=0.2)
            for sid in sorted(ranges):
                set_torque(ser, sid, True)
            steps = read_real_steps(ser, ranges)
            print(f"已连接 {args.port} 并使能六个舵机；退出后扭矩保持使能，请支撑机械臂。")
    except Exception as error:
        if ser is not None and getattr(ser, "is_open", False):
            ser.close()
        if isinstance(error, ModuleNotFoundError):
            parser.error("未安装 pyserial，请先执行 pip install pyserial")
        parser.error(str(error))

    try:
        held_steps = {sid: steps[sid] for sid in HELD_IDS}
        current_rad = steps_to_joint_rad(steps, mids)

        def apply_target(tx, ty, tz):
            nonlocal current_rad
            q, note = inverse_kinematics((tx, ty, tz), current_rad, limits)
            if note:
                print(f"  {note}")
            new_steps = dict(held_steps)
            for sid in ACTIVE_IDS:
                name = JOINT_BY_ID[sid]
                step = rad_to_step(q[name], mids[sid])
                lo, hi = ranges[sid]
                new_steps[sid] = max(lo, min(hi, step))
            achieved = fk_position(q)
            report_plan(new_steps, q, achieved, args.duration, args.speed, args.dry_run)
            if ser is not None:
                sync_move(ser, new_steps, args.duration, args.speed)
                print("  已发送目标位置。")
            current_rad = q

        def move_by(dx, dy, dz):
            x, y, z = current_end_position(current_rad)
            apply_target(x + dx, y + dy, z + dz)

        def go_home():
            nonlocal current_rad, held_steps
            target = home_steps(ranges)
            print("  回到中间位置（六个关节到校准中点）")
            if ser is not None:
                sync_move(ser, target, args.duration, args.speed)
                print("  已发送目标位置。")
            held_steps = {sid: target[sid] for sid in HELD_IDS}
            current_rad = steps_to_joint_rad(target, mids)
            report_plan(target, current_rad, fk_position(current_rad),
                        args.duration, args.speed, args.dry_run)

        def print_info():
            x, y, z = current_end_position(current_rad)
            print(f"  当前末端参考点: x={x:+.3f} m, y={y:+.3f} m, z={z:+.3f} m")
            print("  当前关节角: " + ", ".join(
                f"{JOINT_BY_ID[sid]}={math.degrees(current_rad[JOINT_BY_ID[sid]]):+.1f}°"
                for sid in range(1, 6)
            ))

        if args.move:
            direction, distance_text = args.move
            try:
                distance = float(distance_text)
                if not math.isfinite(distance) or not 0 <= abs(distance) <= 0.5:
                    raise ValueError
            except ValueError:
                parser.error("--move 的距离必须是 0~0.5 米之间的数字。")
            dx, dy, dz = direction_to_vector(direction, distance)
            move_by(dx, dy, dz)
            return

        if args.goto:
            if any(not math.isfinite(v) or abs(v) > 2.0 for v in args.goto):
                parser.error("--goto 的坐标必须是 -2~2 米之间的数字。")
            apply_target(*args.goto)
            return

        if args.mode == "keyboard":
            print_info()
            show_keyboard_help(args.step)
            while True:
                try:
                    key = read_key()
                except (EOFError, KeyboardInterrupt):
                    break
                if key in ("q", "quit", "exit"):
                    break
                if key == "h":
                    go_home()
                    continue
                if key in ("w", "s", "a", "d", "r", "f", "up", "down", "left", "right"):
                    direction = {
                        "w": "forward", "s": "back",
                        "a": "left", "d": "right",
                        "r": "up", "f": "down",
                        "up": "up", "down": "down",
                        "left": "left", "right": "right",
                    }[key]
                    dx, dy, dz = direction_to_vector(direction, args.step)
                    move_by(dx, dy, dz)
            return

        print_info()
        show_text_help(args.step)
        while True:
            try:
                line = input("运动> ").strip()
            except (EOFError, KeyboardInterrupt):
                break
            command = parse_command(line, args.step)
            if command[0] == "quit":
                break
            if command[0] == "info":
                print_info()
            elif command[0] == "help":
                show_text_help(args.step)
            elif command[0] == "error":
                print(f"  {command[1]}")
            elif command[0] == "move_dir":
                dx, dy, dz = direction_to_vector(command[1], command[2])
                move_by(dx, dy, dz)
            elif command[0] == "move_vec":
                move_by(command[1], command[2], command[3])
            elif command[0] == "goto":
                apply_target(command[1], command[2], command[3])
            elif command[0] == "home":
                go_home()
    finally:
        if ser is not None:
            ser.close()


def selftest():
    print("=== 逆运动学自测（离线，无需硬件） ===")
    ranges = load_ranges()
    mids = ranges_midpoints(ranges)
    limits = ranges_limits_rad(ranges, mids)

    # 零位 FK（所有关节角为 0，即每个舵机在校准中点）
    zero = {name: 0.0 for name in JOINT_ORDER}
    p0 = fk_position(zero)
    print(f"零位末端参考点: x={p0[0]:+.4f} m, y={p0[1]:+.4f} m, z={p0[2]:+.4f} m")

    # FK -> IK -> FK 往返
    random = __import__("random")
    random.seed(1)
    max_err = 0.0
    for _ in range(200):
        q = {}
        for sid in range(1, 6):
            lo, hi = limits[sid]
            q[JOINT_BY_ID[sid]] = random.uniform(lo, hi)
        target = fk_position(q)
        solved, note = inverse_kinematics(target, q, limits)
        achieved = fk_position(solved)
        max_err = max(max_err, math.dist(target, achieved))
    print(f"200 个随机点 IK 往返最大位置误差: {max_err * 1000:.3f} mm")
    assert max_err < 1e-3, "IK 往返误差过大"

    # 直线性：向上移动 5 cm，x/y 应基本不变
    q_solved, note = inverse_kinematics(
        (p0[0], p0[1], p0[2] + 0.05), zero, limits
    )
    p1 = fk_position(q_solved)
    dx = p1[0] - p0[0]
    dy = p1[1] - p0[1]
    dz = p1[2] - p0[2]
    print(f"向上 5cm 测试: 实际位移 x={dx * 100:+.2f}cm y={dy * 100:+.2f}cm z={dz * 100:+.2f}cm")
    assert abs(dx) < 0.01 and abs(dy) < 0.01 and abs(dz - 0.05) < 0.01

    print("\n全部自测通过（OK）")


if __name__ == "__main__":
    main()
