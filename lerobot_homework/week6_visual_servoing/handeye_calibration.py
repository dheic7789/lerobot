#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""手眼标定（课堂练习 2/4）：计算末端→相机变换 E_T_C。

流程：
  1. 固定标定板；程序自动生成 12+ 组关节角姿态（只动关节，基座不动）
  2. 每个姿态：send_action 移动 → 相机拍照 → 棋盘格完整可见则记录
     （FK 求 B_T_E；solvePnP 求 C_T_M）
  3. calibrateHandEye 求解 E_T_C，保存 handeye_result.json
  4. 验证：回代 B_T_M，各姿态与均值偏差（目标 <5cm）

依赖：先完成相机标定（camera_intrinsics.json），棋盘格与相机标定一致。
"""
import argparse
import json
import random
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

DEFAULT_URDF = Path("lerobot_homework/SO101/so101_kinematics.urdf")


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM3")
    p.add_argument("--camera-id", type=int, default=1)
    p.add_argument("--n-poses", type=int, default=14, help="采集姿态数（默认 14，建议 ≥12）")
    p.add_argument("--jitter", type=float, default=12.0, help="姿态随机偏移幅度（度）")
    p.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    p.add_argument("--intrinsics", type=Path, default=Path("lerobot_homework/calib_data/camera_intrinsics.json"))
    p.add_argument("--out-dir", type=Path, default=Path("lerobot_homework/calib_data"))
    p.add_argument("--cols", type=int, default=9)
    p.add_argument("--rows", type=int, default=6)
    p.add_argument("--square-size", type=float, default=0.025)
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
    args.out_dir.mkdir(parents=True, exist_ok=True)
    if not args.intrinsics.exists():
        raise SystemExit(f"缺少相机内参 {args.intrinsics}，请先完成相机标定")
    calib = json.loads(args.intrinsics.read_text(encoding="utf-8"))
    K = np.array(calib["K"], dtype=float)
    dist = np.array(calib["dist"], dtype=float)
    print(f"[load] 相机内参 fx={calib['fx']:.1f} fy={calib['fy']:.1f} 误差={calib['reprojection_error']:.3f}")

    # 棋盘格目标点
    pattern = (args.cols, args.rows)
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_size
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # 机械臂 + 运动学
    cfg = SO101FollowerConfig(port=args.port, id="my_awesome_follower_arm", use_degrees=True)
    robot = SO101Follower(cfg)
    robot.connect()
    names = list(robot.bus.motors.keys())
    limits = load_joint_limits_deg(args.urdf)
    kin = RobotKinematics(urdf_path=str(args.urdf), target_frame_name="gripper_frame_link", joint_names=names)

    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        raise SystemExit(f"无法打开摄像头 #{args.camera_id}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    def read_q():
        obs = robot.get_observation()
        return {f"{m}.pos": float(obs[f"{m}.pos"]) for m in names}

    def solve_board_T():
        """返回 C_T_M（4x4，标定板原点在相机坐标系下的位姿），检测失败返回 None。"""
        ok, frame = cap.read()
        if not ok:
            return None, None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern, None)
        if not ret:
            return None, None
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        ok_pnp, rvec, tvec = cv2.solvePnP(objp, corners, K, dist)
        if not ok_pnp:
            return None, None
        T = np.eye(4, dtype=float)
        T[:3, :3] = cv2.Rodrigues(rvec)[0]
        T[:3, 3] = tvec.flatten()
        return T, frame

    # ---- 1) 采集 N 组姿态 ----
    print(f"开始采集 {args.n_poses} 个姿态：机械臂将自动移动，棋盘格完整可见时记录")
    q_cur = read_q()
    q_vals = np.array([q_cur[f"{m}.pos"] for m in names])
    rng = random.Random(42)
    R_gripper2base, t_gripper2base = [], []
    R_target2cam, t_target2cam = [], []
    imgs_dir = args.out_dir / "handeye_images"
    imgs_dir.mkdir(exist_ok=True)
    collected = 0
    attempts = 0

    while collected < args.n_poses and attempts < args.n_poses * 10:
        attempts += 1
        # 随机选 2~3 个关节加偏移，生成新姿态（限制在关节限位内）
        q_new = q_vals.copy()
        for _ in range(rng.randint(2, 3)):
            idx = rng.randrange(len(names))
            lo, hi = limits.get(names[idx], (-180, 180))
            delta = rng.uniform(-args.jitter, args.jitter)
            q_new[idx] = min(max(q_new[idx] + delta, lo + 2), hi - 2)
        q_vals = q_new
        action = {f"{m}.pos": float(q_vals[i]) for i, m in enumerate(names)}
        try:
            robot.send_action(action)
        except Exception as e:
            print(f"[warn] 下发失败: {type(e).__name__}，重试新姿态")
            continue
        time.sleep(1.5)  # 等机械臂到位

        T_cm, frame = solve_board_T()
        if T_cm is None:
            print(f"[skip] 姿态 {collected + 1} 棋盘格不可见，跳过")
            continue

        # FK：当前关节 → B_T_E
        obs = robot.get_observation()
        q_obs = np.array([float(obs[f"{m}.pos"]) for m in names])
        T_be = kin.forward_kinematics(q_obs)
        R_be, t_be = T_be[:3, :3], T_be[:3, 3]
        R_cm, t_cm = T_cm[:3, :3], T_cm[:3, 3]
        R_gripper2base.append(R_be)
        t_gripper2base.append(t_be)
        R_target2cam.append(R_cm)
        t_target2cam.append(t_cm)
        cv2.imwrite(str(imgs_dir / f"pose_{collected:03d}.png"), frame)
        collected += 1
        print(f"[ok] 姿态 {collected}/{args.n_poses} 已记录 (FK t=({t_be[0]:.2f},{t_be[1]:.2f},{t_be[2]:.2f})m)")

    if collected < 5:
        robot.disconnect()
        raise SystemExit(f"有效姿态不足（{collected}），请调整标定板位置/角度后重试")

    # ---- 2) 手眼标定：AX = XB ----
    R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
        R_gripper2base, t_gripper2base, R_target2cam, t_target2cam, method=cv2.CALIB_HAND_EYE_TSAI
    )
    E_T_C = np.eye(4, dtype=float)
    E_T_C[:3, :3] = R_cam2gripper
    E_T_C[:3, 3] = t_cam2gripper.flatten()

    # ---- 3) 验证：B_T_M = B_T_E · E_T_C · C_T_M 应一致 ----
    B_T_M_list = []
    for i in range(collected):
        T_be = np.eye(4, dtype=float)
        T_be[:3, :3] = R_gripper2base[i]
        T_be[:3, 3] = t_gripper2base[i]
        T_cm = np.eye(4, dtype=float)
        T_cm[:3, :3] = R_target2cam[i]
        T_cm[:3, 3] = t_target2cam[i]
        B_T_M_list.append(T_be @ E_T_C @ T_cm)
    mean_pos = np.mean([T[:3, 3] for T in B_T_M_list], axis=0)
    errs = [np.linalg.norm(T[:3, 3] - mean_pos) for T in B_T_M_list]
    max_err = float(max(errs))
    mean_err = float(np.mean(errs))

    result = {
        "E_T_C": E_T_C.tolist(),
        "cam_in_gripper_t_m": t_cam2gripper.flatten().tolist(),
        "cam_in_gripper_rvec_deg": Rotation.from_matrix(R_cam2gripper).as_euler("xyz", degrees=True).tolist(),
        "poses_used": collected,
        "board_origin_reproj_max_err_m": max_err,
        "board_origin_reproj_mean_err_m": mean_err,
    }
    out_file = args.out_dir / "handeye_result.json"
    out_file.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")

    print("\n========== 手眼标定结果 ==========")
    print(f"E_T_C（末端→相机）:\n{E_T_C}")
    print(f"相机在末端坐标系位置: {t_cam2gripper.flatten()} m")
    print(f"相机在末端坐标系姿态(deg, xyz): {result['cam_in_gripper_rvec_deg']}")
    print(f"标定板原点回代误差: mean={mean_err:.3f} m, max={max_err:.3f} m （目标 <0.05m）")
    print(f"有效姿态: {collected}")
    print(f"已保存: {out_file}")

    cap.release()
    try:
        robot.disconnect()
    except Exception:
        pass


if __name__ == "__main__":
    main()
