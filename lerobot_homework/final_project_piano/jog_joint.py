# -*- coding: utf-8 -*-
"""绕过 IK，直接让一颗关节移动少量步数并读取实际反馈。"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import common
from common import arm_lib


def main() -> int:
    parser = argparse.ArgumentParser(description="单关节反馈测试")
    parser.add_argument("--port", default=None)
    parser.add_argument("--ranges-file", default=str(common.DEFAULT_RANGES))
    parser.add_argument("--id", type=int, required=True, choices=range(1, 6))
    parser.add_argument("--steps", type=int, default=5, help="正负都可，绝对值建议不超过 10")
    parser.add_argument("--duration", type=float, default=0.6)
    parser.add_argument("--speed", type=int, default=220)
    parser.add_argument("--confirm", action="store_true")
    args = parser.parse_args()
    if not args.confirm:
        parser.error("请支撑机械臂、清空周围空间并加 --confirm。")
    if not 1 <= abs(args.steps) <= 10:
        parser.error("--steps 绝对值必须在 1~10 之间。")

    port = common.find_robot_port(args.port)
    if port is None:
        print("没有找到机械臂 USB 串口。")
        return 2
    ranges = arm_lib.load_ranges(args.ranges_file)
    common.servo_state_preflight(port, ranges)
    arm = arm_lib.Arm(
        port=port,
        ranges_file=Path(args.ranges_file),
        dry_run=False,
        duration=args.duration,
        speed=args.speed,
        frozen={6},
    )
    try:
        arm.connect()
        before_steps = arm.read_steps()
        before = before_steps[args.id]
        low, high = arm.ranges[args.id]
        target = max(low, min(high, before + args.steps))
        print(
            f"ID {args.id}: 当前 {before}，目标 {target}，软件范围 {low}~{high}。"
        )
        if target == before:
            print("目标与当前位置相同，未发送运动。")
            return 0
        arm_lib.sync_move(arm.ser, {args.id: target}, args.duration, args.speed)
        time.sleep(args.duration + 0.30)
        after_steps = arm.read_steps()
        after = after_steps[args.id]
        actual_delta = after - before
        print(f"实际反馈：{before} -> {after}，移动 {actual_delta:+d} 步。")
        if actual_delta == 0:
            print(
                "结论：舵机完全没有移动。优先检查负载、机械干涉和供电，"
                "不要继续重复发送目标。"
            )
            return 1
        if abs(actual_delta - args.steps) > max(2, abs(args.steps) // 2):
            print("结论：舵机能动，但跟随误差偏大；先排除卡线和带载不足。")
            return 1
        print("结论：该关节能够跟随单步命令。")
        return 0
    finally:
        arm.close()


if __name__ == "__main__":
    raise SystemExit(main())
