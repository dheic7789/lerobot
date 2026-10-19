"""
六个舵机逐角度测试：每次只动一个舵机、停在指定角度，让你观察并记录
该角度对应的物理位置，形成"角度 ↔ 位置"对照表（之后设任何姿态都知道该用什么角度）。

用法（默认交互式：每个角度停住，按回车测下一个）：
    python lerobot_homework\\tools\\servo_angle_test.py            # 真机，交互
    python lerobot_homework\\tools\\servo_angle_test.py --auto 3   # 自动模式：每个角度停 3 秒
    python lerobot_homework\\tools\\servo_angle_test.py --dry-run  # 仿真验证流程
"""
import argparse
import json
import sys
import time
from pathlib import Path

WEEK7 = Path(r"D:\lerobot_workspace\lerobot_homework\week7_llm_agent")
sys.path.insert(0, str(WEEK7))
from arm_tools import ArmController  # noqa: E402

# 六个舵机（ID1~ID6）与测试角度序列（覆盖各自全范围）
MOTORS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
ANGLES = {
    "shoulder_pan":  [-90, -45, 0, 45, 90],          # 范围 [-110, 110]
    "shoulder_lift": [-90, -45, 0, 45, 90],          # 范围 [-100, 100]
    "elbow_flex":    [-90, -45, 0, 45, 90],          # 范围 [-97, 97]
    "wrist_flex":    [-90, -45, 0, 45, 90],          # 范围 [-95, 95]
    "wrist_roll":    [-90, -45, 0, 45, 90, 135],     # 范围 [-157, 163]
    "gripper":       [-5, 0, 25, 50, 75, 100],       # 范围 [-10, 100]
}

# 基准姿态：手臂抬起悬空，避免测试时碰桌/底座（被测试舵机会被覆盖为各角度）
BASE = {"shoulder_pan": 0, "shoulder_lift": 50, "elbow_flex": 30,
        "wrist_flex": 0, "wrist_roll": 0, "gripper": 50}


def main():
    p = argparse.ArgumentParser(description="六舵机逐角度位置测试")
    p.add_argument("--port", default="COM3")
    p.add_argument("--dry-run", action="store_true", help="仿真：不连机械臂")
    p.add_argument("--auto", type=float, default=0.0,
                   help="自动模式：每个角度停留 N 秒后自动继续（0=交互按回车）")
    p.add_argument("--only", type=int, default=0, choices=list(range(7)),
                   help="只测某个舵机 1~6（0=全部）")
    args = p.parse_args()

    arm = ArmController(port=args.port, dry_run=args.dry_run)
    try:
        for mi, name in enumerate(MOTORS, 1):
            if args.only and args.only != mi:
                continue
            print(f"\n===== 舵机 {mi}（{name}）测试开始 =====")
            print(f"     其余关节固定：{json.dumps(BASE, ensure_ascii=False)}")
            if name == "shoulder_lift":
                print("     注意：肩抬到负角度时手臂会大幅下垂，确认下方无遮挡物")
            for ang in ANGLES[name]:
                joints = dict(BASE)
                joints[name] = ang
                r = arm.move_to_joints(joints)
                time.sleep(1.2)  # 等到位
                line = (f">>> 舵机{mi}({name}) 角度 {ang:+d}° —— 观察该关节现在的物理位置，"
                        f"记录到对照表")
                if args.auto > 0:
                    print(line + f"（{args.auto} 秒后自动下一个）")
                    time.sleep(args.auto)
                else:
                    input(line + "，按回车测下一个角度")
        print("\n全部测试完成。把每行'角度→位置'记下来，就得到该舵机的角度对照表。")
    finally:
        arm.close()


if __name__ == "__main__":
    main()
