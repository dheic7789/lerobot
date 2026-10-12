#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""示教记录 + 重放：记忆机械臂的一段动作并重复执行。

用法：
  1) 按 Enter       开始记录（舵机力矩关闭，可手动掰动机械臂，每 50ms 记一帧）
  2) 再按 Enter     结束记录
  3) 按 Enter       重放一次刚才的动作
  4) 重放完按 Enter 再放一次，按 q 退出
"""
import msvcrt
import sys
import time
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

PORT = "COM3"
ROBOT_ID = "my_awesome_follower_arm"
DT = 0.05  # 采样/重放间隔（秒）


def wait_enter_or_q() -> str:
    """等待 Enter（返回 'enter'）或 q（返回 'q'）。"""
    while True:
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch in (b"\r", b"\n"):
                return "enter"
            if ch in (b"q", b"Q"):
                return "q"
        time.sleep(0.02)


def read_q(robot, names):
    obs = robot.get_observation()
    return {f"{m}.pos": float(obs[f"{m}.pos"]) for m in names}


def main() -> None:
    cfg = SO101FollowerConfig(port=PORT, id=ROBOT_ID, use_degrees=True)
    robot = SO101Follower(cfg)
    robot.connect()
    names = list(robot.bus.motors.keys())

    print("按 Enter 开始记录（力矩将关闭，可手动掰动机械臂；再按 Enter 结束）")
    if wait_enter_or_q() == "q":
        sys.exit(0)

    # 关闭力矩：让机械臂可自由手动掰动
    for m in names:
        robot.bus.write("Torque_Enable", m, 0)
    print("记录中...（再按 Enter 结束记录）")
    traj = []
    last = time.time()
    while True:
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            if ch in (b"\r", b"\n"):
                break
        now = time.time()
        if now - last >= DT:
            try:
                traj.append(read_q(robot, names))
            except Exception as e:
                print(f"[warn] 采样失败（跳过）: {type(e).__name__}")
            last = now
        time.sleep(0.01)
    print(f"记录结束，共 {len(traj)} 帧")

    # 恢复力矩
    for m in names:
        robot.bus.write("Torque_Enable", m, 1)
    time.sleep(0.5)
    print("按 Enter 重放动作（q 退出）")
    if wait_enter_or_q() == "q":
        sys.exit(0)

    while True:
        for q in traj:
            robot.send_action(q)
            time.sleep(DT)
        print(f"重放完成（{len(traj)} 帧）。按 Enter 再放一次，q 退出")
        if wait_enter_or_q() == "q":
            break

    try:
        robot.disconnect()
    except Exception:
        pass
    print("[done] 已断开机械臂")


if __name__ == "__main__":
    main()
