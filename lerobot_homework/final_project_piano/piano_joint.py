# -*- coding: utf-8 -*-
"""按示教的关节位姿演奏，不使用逐点笛卡尔 IK。"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import common
from common import arm_lib
from songs import MELODIES, MELODY_BEATS, parse_notes


def main() -> int:
    parser = argparse.ArgumentParser(description="关节位姿演奏")
    parser.add_argument("--port", default=None)
    parser.add_argument("--poses", default=str(common.CONFIG_DIR / "piano_joint_poses.json"))
    parser.add_argument("--ranges-file", default=str(common.DEFAULT_RANGES))
    parser.add_argument("--note", default=None)
    parser.add_argument("--melody", choices=sorted(MELODIES), default=None)
    parser.add_argument("--echo", action="store_true")
    parser.add_argument("--move-seconds", type=float, default=0.32)
    parser.add_argument("--press-seconds", type=float, default=0.12)
    parser.add_argument(
        "--initial-seconds", type=float, default=1.8,
        help="从任意当前位置首次移动到统一抬起位的最短时间",
    )
    parser.add_argument(
        "--max-joint-speed", type=float, default=160.0,
        help="按每秒最多移动多少舵机步估算安全运动时间",
    )
    parser.add_argument("--dwell-ms", type=int, default=60)
    parser.add_argument("--beat-ms", type=float, default=650.0)
    parser.add_argument("--speed", type=int, default=260)
    parser.add_argument(
        "--approach-lift-steps", type=int, default=60,
        help="按键前先在 2 号关节上方抬起的步数，避免斜线擦到相邻琴键",
    )
    parser.add_argument("--tolerance-steps", type=int, default=18)
    parser.add_argument("--max-move-attempts", type=int, default=3)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--allow-low-voltage", action="store_true")
    parser.add_argument("--confirm", action="store_true")
    args = parser.parse_args()
    if not any((args.note, args.melody, args.echo)):
        parser.error("至少指定 --note、--melody 或 --echo。")
    if not args.dry_run and not args.confirm:
        parser.error("真实运动前请固定机械臂并加 --confirm。")

    data = common.load_json(Path(args.poses))
    keys = {
        note: {int(k): int(v) for k, v in steps.items()}
        for note, steps in data["keys"].items()
    }
    hover = {int(k): int(v) for k, v in data["hover"].items()}
    port = common.find_robot_port(args.port)
    if port is None and not args.dry_run:
        print("没有找到机械臂 USB 串口。")
        return 2
    ranges = arm_lib.load_ranges(args.ranges_file)
    if not args.dry_run:
        common.servo_state_preflight(port, ranges)
    arm = arm_lib.Arm(
        port=port or "DRY",
        ranges_file=Path(args.ranges_file),
        dry_run=args.dry_run,
        duration=args.move_seconds,
        speed=args.speed,
        frozen={6},
    )
    def move_steps(target: dict[int, int], duration: float, label: str) -> None:
        target = dict(target)
        target[6] = hover[6]
        for sid, position in target.items():
            low, high = ranges[sid]
            if not low <= position <= high:
                raise RuntimeError(
                    f"{label}: ID {sid} 目标 {position} 超出软件范围 {low}~{high}。"
                )
        before = arm.read_steps()
        current = dict(before)
        for attempt in range(args.max_move_attempts):
            max_delta = max(abs(target[sid] - current[sid]) for sid in range(1, 6))
            if max_delta <= args.tolerance_steps:
                return
            required_duration = max_delta / max(1.0, args.max_joint_speed) + 0.12
            move_duration = max(duration if attempt == 0 else 0.0, required_duration)
            arm.duration = move_duration
            arm.speed = args.speed
            arm.send_steps(target)
            if arm.dry_run:
                return
            time.sleep(move_duration + 0.08)
            after = arm.read_steps()
            error = max(abs(after[sid] - target[sid]) for sid in range(1, 6))
            progress = max(abs(after[sid] - current[sid]) for sid in range(1, 6))
            print(
                f"{label}: 第 {attempt + 1} 段误差 {error} 步，"
                f"本段实际移动 {progress} 步。"
            )
            current = dict(after)
            if error <= args.tolerance_steps:
                return
        hold = arm.read_steps()
        arm.send_steps(hold)
        print(f"{label}: 运动前 {before}")
        print(f"{label}: 最终位置 {current}")
        raise RuntimeError(
            f"关节连续 {args.max_move_attempts} 段仍未跟上目标，"
            "已把目标恢复到当前实际位置并停止。"
        )

    try:
        arm.connect()
        if args.note:
            sequence = parse_notes(args.note, list(keys))
            beats = [1] * len(sequence)
        elif args.melody:
            sequence = list(MELODIES[args.melody])
            beats = list(MELODY_BEATS.get(args.melody, [1] * len(sequence)))
        else:
            print("可用音键：", " ".join(keys))
            sequence = parse_notes(input("你弹："), list(keys))
            beats = [1] * len(sequence)
            print("JARVIS 接：", " ".join(sequence))

        move_steps(hover, args.initial_seconds, "移动到抬起位")
        for note, beat_count in zip(sequence, beats):
            if note not in keys:
                raise ValueError(f"没有示教音键 {note}")
            approach = dict(keys[note])
            approach[2] = approach[2] - args.approach_lift_steps
            note_started = time.monotonic()
            move_steps(approach, args.move_seconds, f"移动到 {note} 号键上方")
            move_steps(keys[note], args.press_seconds, f"按下 {note}")
            time.sleep(args.dwell_ms * beat_count / 1000.0)
            move_steps(approach, args.move_seconds, f"从 {note} 号键抬起")
            print(f"演奏 {note}")
            target_interval = beat_count * args.beat_ms / 1000.0
            remaining = target_interval - (time.monotonic() - note_started)
            if remaining > 0:
                time.sleep(remaining)
        move_steps(hover, args.move_seconds, "返回统一抬起位")
        return 0
    finally:
        arm.close()


if __name__ == "__main__":
    raise SystemExit(main())
