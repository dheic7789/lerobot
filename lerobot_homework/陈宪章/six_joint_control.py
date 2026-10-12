# -*- coding: utf-8 -*-
"""用六个 0~1 比例值控制六个舵机，不依赖 LeRobot。"""

import argparse
import json
import math
import os
from pathlib import Path
import time


PORT = "COM5"
BAUDRATE = 1_000_000
USE_SYNC_WRITE = True
RANGES_FILE = Path(__file__).with_name("joint_ranges.json")
POSES_FILE = Path(__file__).with_name("joint_poses.json")
DEFAULT_DURATION = 2.0
DEFAULT_SPEED = 100
POSITION_MODULUS = 1 << 10

# 按 scan_bus.py 的实际结果修改 ID；先逐关节标定安全端点再填写步数。
# min_step/max_step 必须是经过实测的安全位置，不能直接照抄其他机械臂的数据。
JOINTS = [
    {"name": "关节1", "servo_id": 1, "min_step": None, "max_step": None, "reverse": False},
    {"name": "关节2", "servo_id": 2, "min_step": None, "max_step": None, "reverse": False},
    {"name": "关节3", "servo_id": 3, "min_step": None, "max_step": None, "reverse": False},
    {"name": "关节4", "servo_id": 4, "min_step": None, "max_step": None, "reverse": False},
    {"name": "关节5", "servo_id": 5, "min_step": None, "max_step": None, "reverse": False},
    {"name": "关节6", "servo_id": 6, "min_step": None, "max_step": None, "reverse": False},
]


def load_saved_ranges(joints, path=RANGES_FILE):
    if not path.exists():
        return
    try:
        ranges = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"读取 {path.name} 失败：{error}") from error
    if not isinstance(ranges, dict):
        raise ValueError(f"{path.name} 格式错误，必须是 JSON 对象。")

    for joint in joints:
        saved = ranges.get(str(joint["servo_id"]))
        if saved is None:
            continue
        if not isinstance(saved, dict):
            raise ValueError(f"{path.name} 中 ID {joint['servo_id']} 的范围格式错误。")
        low, high = saved.get("min_step"), saved.get("max_step")
        if (not isinstance(low, int) or isinstance(low, bool)
            or not isinstance(high, int) or isinstance(high, bool)
            or not 0 <= low < POSITION_MODULUS
            or not 0 <= high < POSITION_MODULUS or low == high):
            raise ValueError(f"{path.name} 中 ID {joint['servo_id']} 的步数范围无效。")
        joint["min_step"], joint["max_step"] = low, high


def validate_configuration(joints):
    if len(joints) != 6:
        raise ValueError("JOINTS 必须恰好配置六个关节。")

    servo_ids = [joint["servo_id"] for joint in joints]
    if len(set(servo_ids)) != 6 or any(not 1 <= sid <= 0xFD for sid in servo_ids):
        raise ValueError("六个舵机 ID 必须是总线上实际存在且互不重复的 ID。")

    missing = [joint["name"] for joint in joints
               if joint["min_step"] is None or joint["max_step"] is None]
    if missing:
        raise ValueError(
            "请先为以下关节填写实测安全 min_step/max_step：" + ", ".join(missing)
        )

    for joint in joints:
        low, high = joint["min_step"], joint["max_step"]
        if not (isinstance(low, int) and isinstance(high, int)
                and 0 <= low < POSITION_MODULUS
                and 0 <= high < POSITION_MODULUS and low != high):
            raise ValueError(
                f"{joint['name']} 的范围端点需是不同的 0~1023 整数；"
                "min_step > max_step 表示范围跨过 1023/0。"
            )


def position_in_range(position, min_step, max_step):
    if min_step < max_step:
        return min_step <= position <= max_step
    return position >= min_step or position <= max_step


def ratio_to_step(ratio, min_step, max_step, reverse=False):
    """Map 0~1 across a calibrated interval, including one crossing 1023/0."""
    if not math.isfinite(ratio) or not 0.0 <= ratio <= 1.0:
        raise ValueError("每个比例值都必须是 0~1 之间的有限数字。")
    if reverse:
        ratio = 1.0 - ratio
    span = (max_step - min_step) % POSITION_MODULUS
    return (min_step + round(ratio * span)) % POSITION_MODULUS


def cross_zero_mapping(position, reference_position):
    """Map a 10-bit position to its continuous equivalent nearest a reference."""
    if (not isinstance(position, int) or isinstance(position, bool)
            or not 0 <= position < POSITION_MODULUS):
        raise ValueError("原始位置必须是 0~1023 的整数。")
    if not isinstance(reference_position, int) or isinstance(reference_position, bool):
        raise ValueError("参考位置必须是整数。")

    reference_raw = reference_position % POSITION_MODULUS
    delta = (position - reference_raw + POSITION_MODULUS // 2) % POSITION_MODULUS
    delta -= POSITION_MODULUS // 2
    return reference_position + delta


def parse_ratios(text, count=6):
    parts = text.replace(",", " ").split()
    if len(parts) not in (1, count):
        raise ValueError(f"请输入 1 个公共比例，或按舵机顺序输入 {count} 个比例。")
    try:
        ratios = [float(part) for part in parts]
    except ValueError as error:
        raise ValueError("输入只能包含数字。") from error
    if any(not math.isfinite(value) or not 0.0 <= value <= 1.0 for value in ratios):
        raise ValueError("六个数字都必须在 0~1 之间。")
    if len(ratios) == 1:
        ratios *= count
    return ratios


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


def move_sync(ser, joints, ratios, duration=DEFAULT_DURATION, speed=DEFAULT_SPEED):
    positions = {
        joint["servo_id"]: ratio_to_step(
            ratio, joint["min_step"], joint["max_step"], joint["reverse"]
        )
        for joint, ratio in zip(joints, ratios)
    }
    params = build_pose_params(joints, positions, duration, speed)
    send(ser, 0xFE, 0x83, params, expect=0)


def build_pose_params(joints, positions, duration, speed):
    """Build synchronous position/time/speed data for saved joint positions."""
    if not math.isfinite(duration) or not 0 < duration <= 65.535:
        raise ValueError("切换时间必须大于 0 且不超过 65.535 秒。")
    if not isinstance(speed, int) or isinstance(speed, bool) or not 1 <= speed <= 0xFFFF:
        raise ValueError("速度寄存器值必须是 1~65535 的整数。")
    duration_ms = round(duration * 1000)
    if not 1 <= duration_ms <= 0xFFFF:
        raise ValueError("切换时间换算成毫秒后必须在 1~65535 之间。")

    params = [0x2A, 0x06]
    for joint in joints:
        servo_id = joint["servo_id"]
        if servo_id not in positions:
            raise ValueError(f"姿态缺少 ID {servo_id} 的位置。")
        target = positions[servo_id]
        if (not isinstance(target, int) or isinstance(target, bool)
            or not 0 <= target < POSITION_MODULUS
            or not position_in_range(target, joint["min_step"], joint["max_step"])):
            raise ValueError(f"ID {servo_id} 的预设位置超出已标定安全范围。")
        params += [
            servo_id,
            target >> 8, target & 0xFF,
            duration_ms >> 8, duration_ms & 0xFF,
            speed >> 8, speed & 0xFF,
        ]
    return params


def move_pose(ser, joints, positions, duration, speed):
    params = build_pose_params(joints, positions, duration, speed)
    if USE_SYNC_WRITE:
        send(ser, 0xFE, 0x83, params, expect=0)
    else:
        duration_ms = round(duration * 1000)
        for joint in joints:
            target = positions[joint["servo_id"]]
            send(ser, joint["servo_id"], 0x03, [
                0x2A, target >> 8, target & 0xFF,
                duration_ms >> 8, duration_ms & 0xFF,
                speed >> 8, speed & 0xFF,
            ])


def load_poses(joints, path=POSES_FILE):
    if not path.exists():
        return {}
    try:
        poses = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise ValueError(f"读取 {path.name} 失败：{error}") from error
    if not isinstance(poses, dict):
        raise ValueError(f"{path.name} 格式错误，必须是 JSON 对象。")

    known_joints = {joint["servo_id"]: joint for joint in joints}
    for key, pose in poses.items():
        if len(key) != 1 or key not in "123456789" or not isinstance(pose, dict):
            raise ValueError(f"{path.name} 中的姿态格式错误。快捷键必须是 1~9。")
        positions = pose.get("positions")
        if not isinstance(positions, dict):
            raise ValueError(f"{path.name} 中快捷键 {key} 缺少 positions 对象。")
        for servo_id, target in positions.items():
            if not servo_id.isdigit() or int(servo_id) not in known_joints:
                continue
            joint = known_joints[int(servo_id)]
            if (not isinstance(target, int) or isinstance(target, bool)
                    or not 0 <= target < POSITION_MODULUS
                    or not position_in_range(target, joint["min_step"], joint["max_step"])):
                raise ValueError(f"快捷键 {key} 的 ID {servo_id} 位置超出安全范围。")
    return poses


def save_poses(poses, path=POSES_FILE):
    path.write_text(json.dumps(poses, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_current_pose(ser, joints):
    positions = {}
    for joint in joints:
        servo_id = joint["servo_id"]
        position = read_position(ser, servo_id)
        if not position_in_range(position, joint["min_step"], joint["max_step"]):
            raise RuntimeError(
                f"ID {servo_id} 当前 {position} 步，超出安全范围 "
                f"{joint['min_step']}~{joint['max_step']}；没有保存姿态。"
            )
        positions[str(servo_id)] = position
    return positions


def read_key():
    if os.name != "nt":
        return input("按键> ").strip()[:1].lower()
    import msvcrt

    key = msvcrt.getwch()
    if key in ("\x00", "\xe0"):
        msvcrt.getwch()
        return ""
    return key.lower()


def run_pose_sequence(ser, joints, poses, keys, duration, speed, dry_run=False):
    sequence = [key for key in keys if not key.isspace() and key not in ",->"]
    missing = [key for key in sequence if key not in poses]
    if not sequence or missing:
        raise ValueError("动作序列不能为空，且每个数字都必须已有对应预设姿态。")
    print(f"  执行动作序列（每段 {duration:g} 秒，速度寄存器值 {speed}）：")
    for key in sequence:
        pose = poses[key]
        positions = {int(sid): step for sid, step in pose["positions"].items()}
        print(f"  {key}: {pose['name']} -> " + ", ".join(
            f"ID{joint['servo_id']}={positions[joint['servo_id']]}"
            for joint in joints if joint["servo_id"] in positions
        ))
        if not dry_run:
            move_pose(ser, joints, positions, duration, speed)
            time.sleep(duration)
        print(f"    -> {key}（{pose['name']}）用时 {duration:g} 秒")


def capture_pose(ser, joints, poses, dry_run=False):
    if dry_run:
        print("  --dry-run 模式不能读取机械臂位置，未保存。")
        return
    key = input("分配快捷键 (1~9): ").strip()
    if key not in "123456789" or len(key) != 1:
        print("  快捷键必须是 1~9。")
        return
    name = input("姿态名称: ").strip()
    if not name:
        print("  姿态名称不能为空。")
        return
    if key in poses and input(f"快捷键 {key} 已存在，覆盖？(y/N): ").strip().lower() != "y":
        print("  未覆盖现有姿态。")
        return
    try:
        positions = read_current_pose(ser, joints)
    except RuntimeError as error:
        print(f"  {error}")
        return
    poses[key] = {"name": name, "positions": positions}
    save_poses(poses)
    print(f"  已保存“{name}”到快捷键 {key}。")


def show_controls(poses, duration, speed):
    print("快捷键：s 保存当前姿态；1~9 切换预设；a 执行动作序列；")
    print("        t 设置切换时长；v 设置速度；m 手动输入比例；q 退出。")
    if poses:
        print("已保存预设：" + "；".join(
            f"{key}={pose['name']}" for key, pose in sorted(poses.items())
        ))
    else:
        print("尚无预设姿态，先按 s 保存一个姿态。")
    print(f"当前参数：每段 {duration:g} 秒，速度寄存器值 {speed}。")


def move_one_by_one(ser, joints, ratios, duration=DEFAULT_DURATION, speed=DEFAULT_SPEED):
    duration_ms = round(duration * 1000)
    for joint, ratio in zip(joints, ratios):
        target = ratio_to_step(
            ratio, joint["min_step"], joint["max_step"], joint["reverse"]
        )
        send(ser, joint["servo_id"], 0x03, [
            0x2A, target >> 8, target & 0xFF,
            duration_ms >> 8, duration_ms & 0xFF,
            speed >> 8, speed & 0xFF,
        ])


def report_targets(joints, ratios):
    targets = []
    for joint, ratio in zip(joints, ratios):
        target = ratio_to_step(
            ratio, joint["min_step"], joint["max_step"], joint["reverse"]
        )
        targets.append(target)
        print(f"  {joint['name']} (ID {joint['servo_id']}): {ratio:.3f} -> {target} 步")
    return targets


def main():
    parser = argparse.ArgumentParser(description="六关节舵机比例控制")
    parser.add_argument(
        "--dry-run", action="store_true", help="只显示目标位置，不向舵机发送指令"
    )
    parser.add_argument(
        "--continuous", action="store_true",
        help="启动后直接进入逐行比例输入模式",
    )
    parser.add_argument(
        "--ratios", nargs="+", type=float,
        help="发送一次比例目标：输入 1 个公共比例，或按所选 ID 输入多个比例",
    )
    parser.add_argument(
        "--duration", type=float, default=DEFAULT_DURATION,
        help=f"目标运动时间（秒，默认：{DEFAULT_DURATION:g}）",
    )
    parser.add_argument(
        "--speed", type=int, default=DEFAULT_SPEED,
        help=f"舵机速度寄存器值（默认：{DEFAULT_SPEED}）",
    )
    parser.add_argument("--port", default=PORT, help=f"串口号（默认：{PORT}）")
    parser.add_argument(
        "--ids", nargs="+", type=int, default=[1, 2, 3, 4, 5, 6],
        help="要控制的舵机 ID；输入值按此顺序对应（默认：1 2 3 4 5 6）",
    )
    args = parser.parse_args()

    if not math.isfinite(args.duration) or not 0 < args.duration <= 65.535:
        parser.error("--duration 必须大于 0 且不超过 65.535 秒。")
    if not 1 <= args.speed <= 0xFFFF:
        parser.error("--speed 必须是 1~65535 的整数。")

    try:
        load_saved_ranges(JOINTS)
        validate_configuration(JOINTS)
    except ValueError as error:
        parser.error(str(error))

    joint_by_id = {joint["servo_id"]: joint for joint in JOINTS}
    if (not args.ids or len(set(args.ids)) != len(args.ids)
            or any(servo_id not in joint_by_id for servo_id in args.ids)):
        parser.error("--ids 必须是不重复且已配置范围的舵机 ID（1~6）。")
    selected_joints = [joint_by_id[servo_id] for servo_id in args.ids]

    one_shot_ratios = None
    if args.ratios is not None:
        try:
            one_shot_ratios = parse_ratios(
                " ".join(str(value) for value in args.ratios), len(selected_joints)
            )
        except ValueError as error:
            parser.error(str(error))

    try:
        poses = load_poses(selected_joints)
    except ValueError as error:
        parser.error(str(error))

    ser = None
    if not args.dry_run:
        import serial

        try:
            ser = serial.Serial(args.port, BAUDRATE, timeout=0.2)
            for joint in selected_joints:
                position = read_position(ser, joint["servo_id"])
                low, high = joint["min_step"], joint["max_step"]
                if not position_in_range(position, low, high):
                    print(
                        f"ID {joint['servo_id']} 当前 {position} 步，"
                        f"超出已标定范围 {low}~{high}；未使能任何舵机。"
                    )
                    ser.close()
                    return
        except (serial.SerialException, RuntimeError) as error:
            if ser is not None and ser.is_open:
                ser.close()
            parser.error(f"连接或读取舵机失败：{error}")

        for joint in selected_joints:
            set_torque(ser, joint["servo_id"], True)
        selected_ids = " ".join(str(joint["servo_id"]) for joint in selected_joints)
        print(f"已连接 {args.port}，仅使能 ID：{selected_ids}。")
        print("退出时所选舵机保持扭矩使能；请支撑机械臂并远离夹点。")

    print(f"姿态文件：{POSES_FILE.name}；默认每段 {DEFAULT_DURATION:g} 秒，速度寄存器值 {DEFAULT_SPEED}。")
    print("保存姿态前请先用示教器/手动方式摆到安全位置。")
    duration, speed = args.duration, args.speed
    show_controls(poses, duration, speed)
    try:
        if one_shot_ratios is not None:
            report_targets(selected_joints, one_shot_ratios)
            if ser is not None:
                mover = move_sync if USE_SYNC_WRITE else move_one_by_one
                mover(ser, selected_joints, one_shot_ratios, duration, speed)
            print("  仅预览，未发送指令。" if args.dry_run else "  已发送目标位置。")
            return

        if args.continuous:
            selected_ids = "、".join(str(joint["servo_id"]) for joint in selected_joints)
            print(f"逐行输入 1 个公共比例或按 ID 顺序输入 {len(selected_joints)} 个比例（ID：{selected_ids}）；q 退出。")
            if not args.dry_run:
                print("每次发出目标后，等机械臂停稳再输入下一行。")
            while True:
                try:
                    text = input("比例> ").strip()
                except (EOFError, KeyboardInterrupt):
                    break
                if text.lower() in ("q", "quit", "exit"):
                    break
                try:
                    ratios = parse_ratios(text, len(selected_joints))
                    report_targets(selected_joints, ratios)
                    if ser is not None:
                        mover = move_sync if USE_SYNC_WRITE else move_one_by_one
                        mover(ser, selected_joints, ratios, duration, speed)
                    print("  仅预览，未发送指令。" if args.dry_run else "  已发送目标位置。")
                except ValueError as error:
                    print(f"  {error}")
            return

        while True:
            try:
                print("\n姿态控制> ", end="", flush=True)
                key = read_key()
                if key:
                    print(key)
            except (EOFError, KeyboardInterrupt):
                break
            if key in ("q", "quit", "exit"):
                break
            if key in poses:
                try:
                    run_pose_sequence(ser, selected_joints, poses, key, duration, speed, args.dry_run)
                except (ValueError, KeyError) as error:
                    print(f"  {error}")
                continue
            if key == "s":
                capture_pose(ser, selected_joints, poses, args.dry_run)
            elif key == "t":
                try:
                    value = float(input("每段切换时长（秒，0~65.535）: ").strip())
                    if not math.isfinite(value) or not 0 < value <= 65.535:
                        raise ValueError
                    duration = value
                    print(f"  每段时长设为 {duration:g} 秒；当前参数：每段 {duration:g} 秒，速度寄存器值 {speed}。")
                except ValueError:
                    print("  时长无效，保持原设置。")
            elif key == "v":
                try:
                    value = int(input("速度寄存器值（1~65535）: ").strip())
                    if not 1 <= value <= 0xFFFF:
                        raise ValueError
                    speed = value
                    print(f"  速度寄存器值设为 {speed}；当前参数：每段 {duration:g} 秒，速度寄存器值 {speed}。")
                except ValueError:
                    print("  速度无效，保持原设置。")
            elif key == "a":
                sequence = input("输入预设快捷键序列（例如 123）: ").strip()
                try:
                    run_pose_sequence(ser, selected_joints, poses, sequence,
                                      duration, speed, args.dry_run)
                    if args.dry_run:
                        print("  仅预览，未发送指令。")
                except (ValueError, KeyError) as error:
                    print(f"  {error}")
            elif key == "m":
                selected_ids = "、".join(str(joint["servo_id"]) for joint in selected_joints)
                example = " ".join(["0.5"] * len(selected_joints))
                text = input(
                    f"输入 1 个公共比例，或按 ID 顺序 {selected_ids} 输入 "
                    f"{len(selected_joints)} 个比例（例如 0.5 或 {example}）: "
                )
                try:
                    ratios = parse_ratios(text, len(selected_joints))
                    report_targets(selected_joints, ratios)
                    if ser is not None:
                        mover = move_sync if USE_SYNC_WRITE else move_one_by_one
                        mover(ser, selected_joints, ratios, duration, speed)
                    print("  仅预览，未发送指令。" if args.dry_run else "  已发送目标位置。")
                except ValueError as error:
                    print(f"  {error}")
            elif key == "?":
                show_controls(poses, duration, speed)
            elif key:
                print("  未绑定该按键；按 ? 查看控制提示。")
    finally:
        if ser is not None:
            ser.close()


if __name__ == "__main__":
    main()
