# -*- coding: utf-8 -*-
"""手动示教钢琴的抬起位和每个白键按下位（记录舵机步数）。

示教期间不使能扭矩。必须有人托住机械臂。每一步只读取当前位置，
不会发送运动命令。
"""

from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path

import serial

import common
from common import arm_lib

DEFAULT_NOTES = ["1", "2", "3", "4", "5", "6", "7"]


def read_steps(ser, ranges: dict) -> dict[int, int]:
    result = {}
    for sid in range(1, 7):
        position = arm_lib.read_position(ser, sid)
        low, high = ranges[sid]
        if not low <= position <= high:
            raise RuntimeError(
                f"ID {sid} 当前 {position} 不在软件范围 {low}~{high} 内。"
            )
        result[sid] = position
    return result


def main() -> int:
    parser = argparse.ArgumentParser(description="手动记录关节位姿")
    parser.add_argument("--port", default=None)
    parser.add_argument("--ranges-file", default=str(common.DEFAULT_RANGES))
    parser.add_argument("--output", default=str(common.CONFIG_DIR / "piano_joint_poses.json"))
    parser.add_argument("--notes", nargs="+", default=DEFAULT_NOTES)
    parser.add_argument(
        "--update-existing", action="store_true",
        help="读取已有配置，只覆盖 --notes 指定的琴键",
    )
    parser.add_argument("--confirm", action="store_true")
    args = parser.parse_args()
    if not args.confirm:
        parser.error("必须托住机械臂、关闭扭矩并清空运动区域；确认后加 --confirm。")
    if not 1 <= len(args.notes) <= 9:
        parser.error("--notes 需要 1~9 个音键。")

    port = common.find_robot_port(args.port)
    if port is None:
        print("没有找到机械臂 USB 串口。")
        return 2
    ranges = arm_lib.load_ranges(args.ranges_file)
    hover_steps = None
    key_steps: dict[str, dict[int, int]] = {}
    if args.update_existing:
        output_path = Path(args.output)
        if not output_path.exists():
            print(f"--update-existing 需要已有文件：{output_path}")
            return 2
        existing = common.load_json(output_path)
        hover_steps = {int(k): int(v) for k, v in existing["hover"].items()}
        key_steps = {
            note: {int(k): int(v) for k, v in steps.items()}
            for note, steps in existing["keys"].items()
        }
        print(f"已读取 {output_path}，保留已有抬起位和 {len(key_steps)} 个琴键。")
        print(f"本次只更新：{', '.join(args.notes)}")

    try:
        ser = serial.Serial(port, arm_lib.BAUDRATE, timeout=0.20)
    except serial.SerialException as error:
        print(f"打不开串口 {port}：{error}")
        return 2

    print("本程序只读位置，不会使能扭矩，也不会主动运动。")
    if hover_steps is None:
        print("用手把机械臂移到所有琴键上方一个安全的“抬起位”，输入 H 记录。")
    else:
        print("已有抬起位会保留；如需重新记录，可以输入 H 覆盖。")
    print(f"本次需要记录：{', '.join(args.notes)}")
    print("命令：p 读状态，h 记录抬起位，1~7 记录按键位，f 保存，q 退出。")
    with ser:
        torque = {
            sid: arm_lib.read_torque(ser, sid) for sid in range(1, 7)
        }
        if any(value not in (0, None) for value in torque.values()):
            print(f"检测到部分舵机扭矩仍开启：{torque}")
            print("若机械臂不能用手轻松移动，请先断电再重新上电，确保扭矩关闭。")
        while True:
            command = input("record> ").strip().lower()
            if command in ("q", "quit", "exit"):
                print("退出，未保存。")
                return 0
            if command == "p":
                try:
                    print(read_steps(ser, ranges))
                except RuntimeError as error:
                    print(error)
                continue
            if command == "f":
                if hover_steps is None:
                    print("还没有记录抬起位 H。")
                    continue
                missing = [note for note in args.notes if note not in key_steps]
                if missing:
                    print(f"还没有记录音键：{', '.join(missing)}")
                    continue
                payload = {
                    "hover": {str(k): v for k, v in hover_steps.items()},
                    "keys": {
                        note: {str(k): v for k, v in key_steps[note].items()}
                        for note in sorted(key_steps)
                    },
                    "created_at": datetime.now().isoformat(timespec="seconds"),
                    "instruction": "关节示教不需要 IK；移动机械臂、钢琴或改变装配后重新示教。",
                }
                common.save_json(Path(args.output), payload)
                print(f"已保存：{args.output}")
                return 0
            try:
                steps = read_steps(ser, ranges)
            except RuntimeError as error:
                print(error)
                continue
            if command == "h":
                hover_steps = steps
                print(f"抬起位：{steps}")
                continue
            if command in args.notes:
                if hover_steps is None:
                    print("先记录抬起位 H。")
                    continue
                key_steps[command] = steps
                print(f"音键 {command} 按下位：{steps}")
                continue
            print("无效命令。")


if __name__ == "__main__":
    raise SystemExit(main())
