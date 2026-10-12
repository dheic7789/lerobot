#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""手眼标定（课堂练习 2/4）：计算末端→相机变换 E_T_C。

两种采集方式：
  1. 手动模式（默认）：舵机关力矩 → 手动掰机械臂 → 每姿态按【空格】记录
     （读取舵机位置 FK 求 B_T_E；solvePnP 求 C_T_M），共 --n-poses 次
  2. 重算模式（--recompute）：从已存的 poses.json + 照片重新求解，无需重新采集

求解：Tsai-Lenz AX=XB（OpenCV 5 移除了 cv2.calibrateHandEye，这里用 numpy 实现）。
验证：回代 B_T_M，各姿态与均值偏差（目标 <5cm）。

依赖：先完成相机标定（camera_intrinsics.json）。
"""
import argparse
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

from lerobot.model.kinematics import RobotKinematics
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

DEFAULT_URDF = Path("lerobot_homework/tools/SO101/so101_kinematics.urdf")


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="COM3")
    p.add_argument("--camera-id", type=int, default=1)
    p.add_argument("--n-poses", type=int, default=14, help="采集姿态数（默认 14，建议 ≥12）")
    p.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    p.add_argument("--intrinsics", type=Path, default=Path("lerobot_homework/calib_data/camera_intrinsics.json"))
    p.add_argument("--out-dir", type=Path, default=Path("lerobot_homework/calib_data"))
    p.add_argument("--square-size", type=float, default=0.025)
    p.add_argument("--recompute", action="store_true",
                   help="从已存 poses.json + 照片重算，跳过采集（无需连机械臂）")
    p.add_argument("--manual", action="store_true",
                   help="手动模式：关力矩手掰 + 空格记录（默认自动旋转）")
    p.add_argument("--jitter", type=float, default=4.0,
                   help="识别到棋盘格后的附近转动幅度（度），默认 4.0")
    p.add_argument("--run-time", type=int, default=800,
                   help="SCS215 每次移动完成时间 ms（默认 800，越大越慢越稳，防过载）")
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


def skew(v):
    return np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]], dtype=float)


def solve_hand_eye_tsai(R_gripper2base, t_gripper2base, R_target2cam, t_target2cam):
    """Tsai-Lenz AX=XB 手眼标定（OpenCV 5 移除了 cv2.calibrateHandEye）。

    A = inv(B_T_E_i)·B_T_E_j（gripper2base 相对变化）
    B = C_T_M_i·inv(C_T_M_j)（target2cam 相对变化）
    X = E_T_C（末端→相机）
    """
    n = len(R_gripper2base)
    # ---- 旋转：R_X·rB = rA（skew(P+Q)·r = Q-P, P=rB, Q=rA）----
    S, q = [], []
    for i in range(n):
        for j in range(i + 1, n):
            R_A = R_gripper2base[i].T @ R_gripper2base[j]
            R_B = R_target2cam[i] @ R_target2cam[j].T
            rA = Rotation.from_matrix(R_A).as_rotvec()
            rB = Rotation.from_matrix(R_B).as_rotvec()
            if np.linalg.norm(rA) < 1e-6 or np.linalg.norm(rB) < 1e-6:
                continue
            Sk = skew(rA + rB)
            if np.linalg.matrix_rank(Sk) < 3:
                continue
            S.append(Sk)
            q.append(rB - rA)
    if len(S) < 2:
        raise RuntimeError("姿态对不足，无法求解旋转（需要 ≥2 组有效姿态对）")
    r = np.linalg.lstsq(np.vstack(S), np.concatenate(q), rcond=None)[0]
    R_X = Rotation.from_rotvec(r).as_matrix()

    # ---- 平移：(R_A - I)·t_X = R_X·t_B - t_A ----
    M, b = [], []
    for i in range(n):
        for j in range(i + 1, n):
            R_A = R_gripper2base[i].T @ R_gripper2base[j]
            t_A = R_gripper2base[i].T @ (t_gripper2base[j] - t_gripper2base[i])
            R_B = R_target2cam[i] @ R_target2cam[j].T
            t_B = t_target2cam[i] - R_B @ t_target2cam[j]
            M.append(R_A - np.eye(3))
            b.append(R_X @ t_B - t_A)
    t_X = np.linalg.lstsq(np.vstack(M), np.concatenate(b), rcond=None)[0]
    return R_X, t_X


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    if not args.intrinsics.exists():
        raise SystemExit(f"缺少相机内参 {args.intrinsics}，请先完成相机标定")
    calib = json.loads(args.intrinsics.read_text(encoding="utf-8"))
    K = np.array(calib["K"], dtype=float)
    dist = np.array(calib["dist"], dtype=float)
    print(f"[load] 相机内参 fx={calib['fx']:.1f} fy={calib['fy']:.1f} 误差={calib['reprojection_error']:.3f}")

    chess = calib["chessboard"]
    detect_patterns = [(int(chess["cols"]), int(chess["rows"]))]
    for p in [(7, 5), (9, 6), (6, 4), (10, 7)]:
        if p not in detect_patterns:
            detect_patterns.append(p)
    square_size = float(chess.get("square_size_m", args.square_size))
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    pattern = None
    objp = None
    imgs_dir = args.out_dir / "handeye_images"
    imgs_dir.mkdir(exist_ok=True)
    poses_file = imgs_dir / "poses.json"

    R_gripper2base, t_gripper2base = [], []
    R_target2cam, t_target2cam = [], []
    collected = 0

    if args.recompute:
        # ---- 从 poses.json + 照片重算（不连机械臂）----
        if not poses_file.exists():
            raise SystemExit(f"缺少 {poses_file}，请先完成一次采集")
        poses = json.loads(poses_file.read_text(encoding="utf-8"))
        print(f"[recompute] 从 {poses_file} 恢复 {len(poses)} 个姿态")
        for i, p in enumerate(poses):
            img_f = imgs_dir / f"pose_{i:03d}.png"
            if not img_f.exists():
                continue
            img = cv2.imread(str(img_f))
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, pattern if pattern else tuple(p["pattern"]),
                                                     cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
            if not ret:
                print(f"[warn] {img_f.name} 棋盘格检测失败，跳过")
                continue
            if pattern is None:
                pattern = tuple(p["pattern"])
                objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
                objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_size
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            ok_pnp, rvec, tvec = cv2.solvePnP(objp, corners, K, dist)
            if not ok_pnp:
                continue
            T_cm = np.eye(4, dtype=float)
            T_cm[:3, :3] = cv2.Rodrigues(rvec)[0]
            T_cm[:3, 3] = tvec.flatten()
            q_obs = np.array(p["q"])
            # FK 结果在采集时已保存，直接使用
            if p.get("B_T_E") is None:
                print(f"[warn] {img_f.name} 缺少 FK 结果，跳过")
                continue
            T_be = np.array(p["B_T_E"])
            R_gripper2base.append(T_be[:3, :3])
            t_gripper2base.append(T_be[:3, 3])
            R_target2cam.append(T_cm[:3, :3])
            t_target2cam.append(T_cm[:3, 3])
            collected += 1
    else:
        # ---- 采集：连机械臂 + 相机 ----
        limits = load_joint_limits_deg(args.urdf)
        cfg = SO101FollowerConfig(port=args.port, id="my_awesome_follower_arm", use_degrees=True)
        robot = SO101Follower(cfg)
        robot.connect()
        # SCS215 限速（地址44，ms/步），慢速移动避免瞬时电流过大触发过载保护
        for m in robot.bus.motors:
            robot.bus.write("Running_Time", m, args.run_time, normalize=False)
        print(f"[setup] 已设每步运行时间 {args.run_time} ms（防过载）")
        names = list(robot.bus.motors.keys())
        kin = RobotKinematics(urdf_path=str(args.urdf), target_frame_name="gripper_frame_link", joint_names=names)

        cap = cv2.VideoCapture(args.camera_id)
        if not cap.isOpened():
            raise SystemExit(f"无法打开摄像头 #{args.camera_id}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        # 启动探测棋盘格规格：实时预览，等待识别（按 q 退出）
        detect_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FILTER_QUADS)
        print("[探测] 请把棋盘格完整放入相机画面...（识别到自动继续，按 q 退出）")
        pattern = None
        objp = None
        while True:
            ok, frame = cap.read()
            if not ok:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            cv2.putText(frame, "Looking for chessboard... (q quit)", (20, 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.imshow("handeye-preview", frame)
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                break
            for p in detect_patterns:
                r, c = cv2.findChessboardCorners(gray, p, detect_flags)
                if r:
                    pattern = p
                    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
                    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_size
                    print(f"[info] 启动探测识别棋盘格规格: {pattern[0]}x{pattern[1]}，边长 {square_size * 1000:.1f}mm")
                    break
            if pattern is not None:
                break
        cv2.destroyWindow("handeye-preview")
        if pattern is None or objp is None:
            raise SystemExit("未识别到棋盘格：请把棋盘格完整放入画面后重新运行")

        def read_q():
            obs = robot.get_observation()
            return {f"{m}.pos": float(obs[f"{m}.pos"]) for m in names}

        dbg_saved = 0

        def solve_board_T():
            nonlocal dbg_saved
            ok, frame = cap.read()
            if not ok:
                return None, None
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ret, corners = cv2.findChessboardCorners(gray, pattern, detect_flags)
            if not ret:
                if dbg_saved < 3:
                    f = imgs_dir / f"debug_noboard_{dbg_saved}.png"
                    cv2.imwrite(str(f), frame)
                    dbg_saved += 1
                    print(f"[debug] 检测失败帧已存: {f}")
                cv2.putText(frame, "NO BOARD", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                cv2.imshow("handeye", frame)
                return None, None
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(frame, pattern, corners, True)
            ok_pnp, rvec, tvec = cv2.solvePnP(objp, corners, K, dist)
            if not ok_pnp:
                return None, None
            T = np.eye(4, dtype=float)
            T[:3, :3] = cv2.Rodrigues(rvec)[0]
            T[:3, 3] = tvec.flatten()
            cv2.putText(frame, "BOARD OK", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imshow("handeye", frame)
            return T, frame

        poses = []
        if args.manual:
            # ---------- 手动模式：关力矩 → 手掰 → 空格记录 ----------
            print(f"开始手动采集 {args.n_poses} 个姿态：")
            print("  1) 舵机力矩已关闭，可手动掰动机械臂")
            print(f"  2) 摆好姿态后按【空格】记录（读取舵机位置 + 拍照检测棋盘格），共 {args.n_poses} 次")
            print("  3) 按 q 提前结束")
            for m in names:
                robot.bus.write("Torque_Enable", m, 0)
            time.sleep(0.5)

            print(f"[提示] 力矩已关闭，开始摆姿态（当前 0/{args.n_poses}）")
            while collected < args.n_poses:
                T_cm, frame = solve_board_T()
                key = cv2.waitKey(30) & 0xFF
                if key in (ord("q"), 27):
                    break
                if key == ord(" ") and T_cm is not None:
                    obs = robot.get_observation()
                    q_obs = np.array([float(obs[f"{m}.pos"]) for m in names])
                    T_be = kin.forward_kinematics(q_obs)
                    R_gripper2base.append(T_be[:3, :3])
                    t_gripper2base.append(T_be[:3, 3])
                    R_target2cam.append(T_cm[:3, :3])
                    t_target2cam.append(T_cm[:3, 3])
                    cv2.imwrite(str(imgs_dir / f"pose_{collected:03d}.png"), frame)
                    poses.append({"q": q_obs.tolist(), "B_T_E": T_be.tolist(),
                                  "pattern": list(pattern)})
                    collected += 1
                    print(f"[ok] 姿态 {collected}/{args.n_poses} 已记录（舵机角 {q_obs.round(1)}）")
                elif key == ord(" ") and T_cm is None:
                    print("[skip] 棋盘格未检测到，请调整姿态/角度后重试")
            for m in names:
                robot.bus.write("Torque_Enable", m, 1)
            time.sleep(0.5)
        else:
            # ---------- 自动模式：自动旋转，识别到棋盘格后只在附近小幅转动 ----------
            import random
            rng = random.Random(42)
            jitter = args.jitter
            send = lambda q: robot.send_action({f"{m}.pos": float(q[i]) for i, m in enumerate(names)})

            def clip_joint(q):
                for i, m in enumerate(names):
                    lo, hi = limits.get(m, (-180, 180))
                    q[i] = min(max(q[i], lo + 2), hi - 2)
                return q

            # 阶段1：自动旋转探索，直到识别到棋盘格
            q_cur = np.array([float(read_q()[f"{m}.pos"]) for m in names])
            anchor = None
            print(f"[探索] 自动旋转寻找棋盘格（最多 60 步）...")
            for step in range(60):
                q_new = clip_joint(q_cur.copy())
                for _ in range(rng.randint(1, 2)):
                    idx = rng.randrange(3)   # 只动关节 1-3
                    q_new[idx] += rng.uniform(-jitter * 1.5, jitter * 1.5)
                q_new = clip_joint(q_new)
                send(q_new)
                time.sleep(1.5)
                q_cur = q_new
                T_cm, frame = solve_board_T()
                if T_cm is not None:
                    anchor = q_cur
                    print(f"[info] 第 {step + 1} 步识别到棋盘格，进入附近采集")
                    break
            if anchor is None:
                raise SystemExit("探索 60 步未识别到棋盘格：请调整棋盘格位置/相机角度后重试")

            # 阶段2：在 anchor 附近小幅转动，采集 n_poses 个姿态
            print(f"[采集] 在识别位置附近小幅转动（±{jitter}°），采集 {args.n_poses} 个姿态")
            while collected < args.n_poses:
                q_new = clip_joint(anchor.copy())
                for _ in range(rng.randint(1, 3)):
                    idx = rng.randrange(3)
                    q_new[idx] += rng.uniform(-jitter, jitter)
                q_new = clip_joint(q_new)
                send(q_new)
                time.sleep(1.5)
                T_cm, frame = solve_board_T()
                if T_cm is None:
                    print("[warn] 附近转动丢失棋盘格，回到锚点重试")
                    send(anchor)
                    time.sleep(1.5)
                    continue
                anchor = q_new
                obs = None
                for _ in range(6):
                    try:
                        obs = robot.get_observation()
                        break
                    except Exception as e:
                        print(f"[warn] 读取位置失败（{type(e).__name__}），等待恢复后重试...")
                        time.sleep(1.0)
                if obs is None:
                    print("[warn] 连续读取失败，跳过本姿态")
                    continue
                q_obs = np.array([float(obs[f"{m}.pos"]) for m in names])
                T_be = kin.forward_kinematics(q_obs)
                R_gripper2base.append(T_be[:3, :3])
                t_gripper2base.append(T_be[:3, 3])
                R_target2cam.append(T_cm[:3, :3])
                t_target2cam.append(T_cm[:3, 3])
                cv2.imwrite(str(imgs_dir / f"pose_{collected:03d}.png"), frame)
                poses.append({"q": q_obs.tolist(), "B_T_E": T_be.tolist(),
                              "pattern": list(pattern)})
                collected += 1
                print(f"[ok] 姿态 {collected}/{args.n_poses} 已记录（舵机角 {q_obs.round(1)}）")

        cap.release()
        cv2.destroyAllWindows()
        robot.disconnect()

        poses_file.write_text(json.dumps(poses, ensure_ascii=False), encoding="utf-8")
        print(f"[save] 姿态数据已存: {poses_file}（可用 --recompute 重算）")

    if collected < 5:
        raise SystemExit(f"有效姿态不足（{collected}），请至少采集 5 个")

    # ---- 求解：Tsai-Lenz AX=XB ----
    R_cam2gripper, t_cam2gripper = solve_hand_eye_tsai(
        R_gripper2base, t_gripper2base, R_target2cam, t_target2cam
    )
    E_T_C = np.eye(4, dtype=float)
    E_T_C[:3, :3] = R_cam2gripper
    E_T_C[:3, 3] = t_cam2gripper

    # ---- 验证：B_T_M = B_T_E · E_T_C · C_T_M 应一致 ----
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
        "cam_in_gripper_t_m": t_cam2gripper.tolist(),
        "cam_in_gripper_rvec_deg": Rotation.from_matrix(R_cam2gripper).as_euler("xyz", degrees=True).tolist(),
        "poses_used": collected,
        "board_origin_reproj_max_err_m": max_err,
        "board_origin_reproj_mean_err_m": mean_err,
    }
    out_file = args.out_dir / "handeye_result.json"
    out_file.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")

    print("\n========== 手眼标定结果 ==========")
    print(f"E_T_C（末端→相机）:\n{E_T_C}")
    print(f"相机在末端坐标系位置: {t_cam2gripper} m")
    print(f"相机在末端坐标系姿态(deg, xyz): {result['cam_in_gripper_rvec_deg']}")
    print(f"标定板原点回代误差: mean={mean_err:.3f} m, max={max_err:.3f} m （目标 <0.05m）")
    print(f"有效姿态: {collected}")
    print(f"已保存: {out_file}")


if __name__ == "__main__":
    main()
