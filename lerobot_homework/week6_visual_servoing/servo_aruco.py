#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""视觉伺服（课堂练习 3/4，简单版）：ArUco 码区域步进追踪。

规则（文档简单版）：
  - ArUco 中心落在左/右/上/下 30% 区域 → 相机往对应方向移动 1cm
  - 落在中间区域 → 相机前移 1cm（靠近）
相机安装在末端：移动末端即可。

用法：
  python servo_aruco.py --camera-id 1
按 q / ESC 退出；Ctrl+C 紧急停止。
"""
import argparse
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

DEFAULT_URDF = Path("lerobot_homework/SO101/so101_kinematics.urdf")
STEP = 0.01  # 每步移动 1cm


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM3")
    p.add_argument("--camera-id", type=int, default=1)
    p.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    p.add_argument("--zone", type=float, default=0.3, help="边缘区域比例（默认 0.3=30%）")
    p.add_argument("--step", type=float, default=STEP, help="每步移动量，米（默认 0.01）")
    p.add_argument("--aruco-id", type=int, default=None, help="只追踪指定 ID 的码（默认任意）")
    return p.parse_args()


def load_joint_limits_deg(urdf_path: Path):
    tree = ET.parse(urdf_path)
    limits = {}
    for joint in tree.getroot().iter("joint"):
        name = joint.get("name")
        lim = joint.find("limit")
        if lim is None or name is None:
            continue
        limits[name] = (np.rad2deg(float(lim.get("lower", 0))), np.rad2deg(float(lim.get("upper", 0))))
    return limits


def main() -> None:
    args = parse_args()
    try:
        import placo  # noqa: F401
    except ImportError:
        sys.exit("缺少 placo，请先安装")

    cfg = SO101FollowerConfig(port=args.port, id="my_awesome_follower_arm", use_degrees=True)
    robot = SO101Follower(cfg)
    robot.connect()
    names = list(robot.bus.motors.keys())
    limits = load_joint_limits_deg(args.urdf)
    kin = RobotKinematics(urdf_path=str(args.urdf), target_frame_name="gripper_frame_link", joint_names=names)

    # ArUco 检测器
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        raise SystemExit(f"无法打开摄像头 #{args.camera_id}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print(f"视觉伺服启动：ArUco 落在边缘 30% 区域 → 对应方向移 {args.step*1000:.0f}mm；中间 → 前移。q 退出")
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        h, w = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = detector.detectMarkers(gray)

        disp = frame.copy()
        cx = cy = None
        if ids is not None and len(ids) > 0:
            if args.aruco_id is None or args.aruco_id in ids.flatten():
                idx = int(np.flatnonzero(ids.flatten() == args.aruco_id)[0]) if args.aruco_id is not None else 0
                c = corners_list[idx][0]
                cx, cy = float(c[:, 0].mean()), float(c[:, 1].mean())
                cv2.polylines(disp, [c.astype(int)], True, (0, 255, 0), 2)
                cv2.circle(disp, (int(cx), int(cy)), 5, (0, 255, 0), -1)

        # 区域判断 → 末端移动量
        if cx is None:
            cv2.putText(disp, "no aruco", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
            cv2.imshow("servo", disp)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                break
            continue

        z = args.zone
        move = None
        if cx < w * z:
            move = np.array([-args.step, 0, 0])   # 码偏左 → 相机左移
            tag = "LEFT"
        elif cx > w * (1 - z):
            move = np.array([args.step, 0, 0])    # 偏右 → 右移
            tag = "RIGHT"
        elif cy < h * z:
            move = np.array([0, 0, args.step])    # 偏上 → 上移
            tag = "UP"
        elif cy > h * (1 - z):
            move = np.array([0, 0, -args.step])   # 偏下 → 下移
            tag = "DOWN"
        else:
            move = np.array([0, args.step, 0])    # 中间 → 前移靠近
            tag = "FORWARD"

        # 读当前位姿 → 目标 = 当前 + 移动 → IK
        try:
            obs = robot.get_observation()
        except Exception as e:
            print(f"[warn] 读位置失败（跳过本帧）: {type(e).__name__}")
            continue
        q_curr = np.array([float(obs[f"{m}.pos"]) for m in names])
        T_curr = kin.forward_kinematics(q_curr)
        tgt_pos = T_curr[:3, 3] + move
        dist_base = float(np.linalg.norm(tgt_pos))
        if dist_base > 0.45:
            tgt_pos = tgt_pos / dist_base * 0.44
        rotvec = Rotation.from_matrix(T_curr[:3, :3]).as_rotvec()
        T_des = np.eye(4, dtype=float)
        T_des[:3, :3] = Rotation.from_rotvec(rotvec).as_matrix()
        T_des[:3, 3] = tgt_pos
        try:
            q_target = kin.inverse_kinematics(q_curr, T_des, orientation_weight=0.5)
            joints_act = {f"{m}.pos": float(q_target[i]) for i, m in enumerate(names)}
            # 关节限位裁剪
            for i, m in enumerate(names):
                lo, hi = limits.get(m, (-180, 180))
                joints_act[f"{m}.pos"] = min(max(float(q_target[i]), lo), hi)
            dq = np.abs(q_target - q_curr)
            if float(dq.max()) <= 30.0:
                robot.send_action(joints_act)
                print(f"[move] {tag} | aruco=({cx:.0f},{cy:.0f})px | move=({move[0]*1000:+.0f},{move[1]*1000:+.0f},{move[2]*1000:+.0f})mm")
            else:
                print(f"[warn] IK 跳变过大，跳过本步")
        except Exception as e:
            print(f"[warn] 下发失败（跳过）: {type(e).__name__}")

        cv2.line(disp, (int(w * z), 0), (int(w * z), h), (255, 255, 0), 1)
        cv2.line(disp, (int(w * (1 - z)), 0), (int(w * (1 - z)), h), (255, 255, 0), 1)
        cv2.line(disp, (0, int(h * z)), (w, int(h * z)), (255, 255, 0), 1)
        cv2.line(disp, (0, int(h * (1 - z))), (w, int(h * (1 - z))), (255, 255, 0), 1)
        cv2.putText(disp, f"{tag}  aruco=({cx:.0f},{cy:.0f})", (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.imshow("servo", disp)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            break

    cap.release()
    cv2.destroyAllWindows()
    try:
        robot.disconnect()
    except Exception:
        pass
    print("[done] 已断开机械臂")


if __name__ == "__main__":
    main()
