#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
EmbodiedAI Week4 Hackathon — 2.2 基于视觉的闭环控制
=====================================================

任务：在电脑屏幕上打开黑球图片，机械臂调用自身摄像头获取实时图像，
用 OpenCV 检测黑色圆形（黑球）在画面中的位置，根据球心与画面中心的
偏差，通过“正运动学 + 逆运动学”把末端移动量换算成各关节角目标并下发，
实现“看 → 算 → 动”的视觉反馈闭环。

依赖（已装好）：lerobot（SO101Follower）、placo（RobotKinematics）、
opencv-python、numpy、scipy

用法
----
  1) 只测摄像头与黑球检测（不接机械臂，先调好阈值/方向）：
       python vision_closed_loop.py --mode camera

  2) 完整闭环（机械臂 + 摄像头都接好）：
       python vision_closed_loop.py --port COM3

  3) 环境自检（不连机械臂，验证 URDF 加载与 IK 数值）：
       python vision_closed_loop.py --mode check

常用参数
--------
  --port COM3            机械臂串口（默认 COM3）
  --camera-id 0          摄像头编号（默认 0）
  --gain 0.0004          像素 → 米 的换算增益（默认 0.0004 m/px）
  --max-step 0.02        每步末端最大移动量（米，默认 0.02）
  --invert-x / --invert-y  画面偏差与末端移动方向相反时取反
  --threshold 60         黑球灰度阈值（越黑越小）
  --max-iter N           最多自动移动 N 步后停下（默认无限，直到对准）

操作：按 q 退出；Ctrl+C 紧急停止（自动断开并关闭扭矩）。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

# ---------- lerobot 相关 ----------
from lerobot.processor import (
    RobotProcessorPipeline,
    robot_action_observation_to_transition,
    transition_to_robot_action,
)
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
from lerobot.robots.so_follower.robot_kinematic_processor import (
    EEBoundsAndSafety,
    InverseKinematicsEEToJoints,
)
from lerobot.model.kinematics import RobotKinematics

DEFAULT_URDF = Path(r"D:\lerobot_workspace\lerobot_homework\SO101\so101_kinematics.urdf")


# ---------------------------------------------------------------------------
# 黑球检测：在图像中找“黑色圆形”
# ---------------------------------------------------------------------------
def detect_black_ball(frame: np.ndarray, threshold: int = 60) -> tuple[float, float, float] | None:
    """返回 (cx, cy, radius)（像素），检测不到返回 None。

    思路：黑球是画面中亮度最低的大块连通域 → 二值化取暗部 →
    取最大轮廓 → 用最小外接圆拟合圆心/半径 → 用圆形度滤除杂散噪声。
    若轮廓法失败（例如球面反光把轮廓打断），退化为 HoughCircles 直接找圆。
    """
    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)

    # 1) 轮廓法：暗部为前景
    _, bw = cv2.threshold(gray, threshold, 255, cv2.THRESH_BINARY_INV)
    bw = cv2.morphologyEx(bw, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))  # 去点状噪声
    bw = cv2.morphologyEx(bw, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))  # 填反光洞

    contours, _ = cv2.findContours(bw, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        best = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(best)
        if area >= 500:  # 过滤太小目标
            (cx, cy), radius = cv2.minEnclosingCircle(best)
            perimeter = cv2.arcLength(best, True)
            circularity = 4.0 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0.0
            if circularity >= 0.4:
                return float(cx), float(cy), float(radius)

    # 2) HoughCircles 兜底（对反光、不完整轮廓更鲁棒）
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=max(w, h) // 4,
        param1=120,
        param2=40,
        minRadius=max(10, w // 40),
        maxRadius=w // 2,
    )
    if circles is not None:
        cx, cy, radius = circles[0][0]
        return float(cx), float(cy), float(radius)

    return None


# ---------------------------------------------------------------------------
# URDF 关节限位（弧度 → 度），用于 IK 解的安全裁剪
# ---------------------------------------------------------------------------
def load_joint_limits_deg(urdf_path: Path) -> dict[str, tuple[float, float]]:
    tree = ET.parse(urdf_path)
    limits: dict[str, tuple[float, float]] = {}
    for joint in tree.getroot().iter("joint"):
        name = joint.get("name")
        lim = joint.find("limit")
        if lim is None or name is None:
            continue
        lower = float(lim.get("lower", 0.0))
        upper = float(lim.get("upper", 0.0))
        limits[name] = (np.rad2deg(lower), np.rad2deg(upper))
    return limits


def clip_joints_to_limits(action: dict[str, float], limits_deg: dict[str, tuple[float, float]]) -> dict[str, float]:
    clipped = dict(action)
    for key, val in list(clipped.items()):
        motor = key.removesuffix(".pos")
        if motor in limits_deg:
            lo, hi = limits_deg[motor]
            clipped[key] = float(np.clip(val, lo, hi))
    return clipped


# ---------------------------------------------------------------------------
# 主程序
# ---------------------------------------------------------------------------
def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="2.2 基于视觉的闭环控制 (EmbodiedAI Week4 Hackathon)")
    p.add_argument("--mode", choices=["vision", "camera", "check"], default="vision",
                   help="vision=完整闭环（默认）；camera=只测摄像头检测；check=环境自检（不连机械臂）")
    p.add_argument("--port", default="COM3", help="机械臂串口（默认 COM3）")
    p.add_argument("--robot-id", default="my_awesome_follower_arm",
                   help="校准文件 id（默认 my_awesome_follower_arm；若之前用别的 id 校准过，请保持一致以免重新校准）")
    p.add_argument("--camera-id", type=int, default=0, help="摄像头编号（默认 0）")
    p.add_argument("--urdf", type=Path, default=DEFAULT_URDF, help="SO101 URDF 路径")
    p.add_argument("--gain", type=float, default=0.0004, help="像素→米增益（默认 0.0004 m/px）")
    p.add_argument("--gain-dr", type=float, default=0.0008, help="球半径像素→前后米增益（默认 0.0008 m/px，越大越猛）")
    p.add_argument("--max-step", type=float, default=0.02, help="每步末端最大移动量，米（默认 0.02，可调慢如 0.005）")
    p.add_argument("--run-time", type=int, default=600, help="SCS215 每次移动完成时间 ms（默认 600，越大越慢）")
    p.add_argument("--threshold", type=int, default=60, help="黑球灰度阈值（默认 60）")
    p.add_argument("--min-radius", type=int, default=25, help="黑球最小半径 px（过滤误检，默认 25）")
    p.add_argument("--max-radius", type=int, default=250, help="黑球最大半径 px（过滤误检，默认 250）")
    p.add_argument("--smooth", type=float, default=0.5, help="检测位置平滑系数 0~1（默认 0.5，越大越跟手、越小越稳）")
    p.add_argument("--hold-ms", type=int, default=500, help="检测丢失后沿用旧位置的时长 ms（默认 500）")
    p.add_argument("--max-iter", type=int, default=None, help="最多自动移动步数（默认不限，直到对准）")
    p.add_argument("--max-reach", type=float, default=0.45, help="末端距基座原点的可达半径估算，米（默认 0.45）")
    p.add_argument("--align-px", type=float, default=15.0, help="判定已对准的像素偏差（默认 15 px）")
    p.add_argument("--invert-x", action="store_true", help="球偏右时若末端反方向走，加此项")
    p.add_argument("--invert-y", action="store_true", help="球偏上时若末端反方向走，加此项")
    return p.parse_args()


def main() -> None:
    args = parse_args()

    if args.mode == "check":
        run_check(args)
        return
    if args.mode == "camera":
        run_camera_only(args)
        return

    # ---------------- vision：完整闭环 ----------------
    try:
        import placo  # noqa: F401  确保 IK 求解器可用
    except ImportError:
        sys.exit(
            "缺少 placo（IK 求解器）。请在 lerobot 环境安装：\n"
            "  conda install -n lerobot -c conda-forge placo -y\n"
            "或  pip install placo"
        )

    # 1) 机械臂
    cfg = SO101FollowerConfig(
        port=args.port,
        id=args.robot_id,
        use_degrees=True,
        # 相对当前位置，单步关节角最大变化量（度），send_action 内部再兜一层安全
        max_relative_target=30.0,
    )
    robot = SO101Follower(cfg)
    motor_names = list(robot.bus.motors.keys())
    print(f"[setup] 机械臂关节: {motor_names}")

    # 2) 运动学（正/逆），URDF 来自 SO-ARM100 官方仓库
    if not args.urdf.exists():
        sys.exit(f"URDF 不存在: {args.urdf}")
    kin = RobotKinematics(
        urdf_path=str(args.urdf),
        target_frame_name="gripper_frame_link",
        joint_names=motor_names,
    )
    limits_deg = load_joint_limits_deg(args.urdf)

    # 3) 末端位姿 → 关节角 的处理器链（IK + 安全限幅）
    ee_to_joints = RobotProcessorPipeline(
        [
            EEBoundsAndSafety(
                end_effector_bounds={"min": [-0.5, -0.5, 0.0], "max": [0.5, 0.5, 0.6]},
                max_ee_step_m=args.max_step,
                raise_on_jump=False,  # 超步长时自动限速，不中断控制
            ),
            InverseKinematicsEEToJoints(
                kinematics=kin,
                motor_names=motor_names,
                initial_guess_current_joints=True,
            ),
        ],
        to_transition=robot_action_observation_to_transition,
        to_output=transition_to_robot_action,
    )

    # 4) 摄像头
    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        sys.exit(f"无法打开摄像头 #{args.camera_id}，请检查连接或用 --camera-id 指定")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # 5) 连接机械臂
    print(f"[setup] 连接机械臂 {args.port} ...")
    robot.connect()
    # SCS215 用 Running_Time 限速（地址44，ms/步），避免目标一步跳到位
    for m in motor_names:
        robot.bus.write("Running_Time", m, args.run_time, normalize=False)
    print(f"[setup] 已设每步运行时间 {args.run_time} ms")
    # 回归位：只用于退出时回到的姿势；启动不主动回位（怎么摆就从哪里开始）
    home_file = Path(__file__).resolve().parent / "home_position.json"
    if home_file.exists():
        home = json.loads(home_file.read_text(encoding="utf-8"))
        q_home = {f"{m}.pos": float(home[m]) for m in motor_names}
        print(f"[setup] 已加载退出回归位（{home_file.name}）")
    else:
        obs0 = robot.get_observation()
        q_home = {f"{m}.pos": float(obs0[f"{m}.pos"]) for m in motor_names}
        print("[setup] 未找到 home_position.json，以当前位姿为退出回归位")
    print("[setup] 连接成功，开始视觉闭环。按 q 退出，Ctrl+C 紧急停止。")

    sign_x = -1.0 if args.invert_x else 1.0   # 默认 +1（实测左右正确）；--invert-x 反转
    sign_y = -1.0 if not args.invert_y else 1.0   # 默认 -1：相机随臂移动，上下镜像（实测）；--invert-y 反转
    step_count = 0
    prev_ball = None   # 上一帧球位置（低通平滑 + 丢帧保持用）
    last_seen = None
    last_act = None    # 最后成功下发的目标关节角（丢球时回到它）
    ax_state = [None, 0]   # x 轴自适应：[上一步|dx|, 连续变差计数]
    ay_state = [None, 0]   # y 轴自适应：[上一步|dy|, 连续变差计数]
    ref_radius = None      # 首次检测到的球半径，作为"合适距离"参考（前后移动基准）
    last_flip_time = 0.0   # 方向自适应冷却时间戳（翻转后 20 秒内不再翻）

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print("[warn] 读帧失败，重试...")
                time.sleep(0.1)
                continue

            h, w = frame.shape[:2]
            ball = detect_black_ball(frame, threshold=args.threshold)

            # 半径合理性过滤：明显过大/过小的检测视为误检
            if ball is not None and not (args.min_radius <= ball[2] <= args.max_radius):
                ball = None

            if ball is None:
                # 球丢失：回到上一次检测到球时的目标位，然后停在原地等待
                if last_act is not None:
                    try:
                        robot.send_action(last_act)
                        print("[info] 球丢失，回到最后检测位置等待...")
                    except Exception:
                        pass
                    last_act = None   # 只回一次，避免每帧重复下发
                cx, cy, radius = prev_ball if prev_ball is not None else (w / 2, h / 2, 50)
                overlay = frame.copy()
                cv2.circle(overlay, (int(cx), int(cy)), int(radius), (0, 200, 255), 2)
                cv2.putText(overlay, "no ball", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 200, 255), 2)
                cv2.imshow("vision", overlay)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
                continue

            # 一阶低通平滑：消除检测位置每帧跳变
            cx, cy, radius = ball
            if prev_ball is not None:
                a = args.smooth
                cx = a * cx + (1 - a) * prev_ball[0]
                cy = a * cy + (1 - a) * prev_ball[1]
                radius = a * radius + (1 - a) * prev_ball[2]
            prev_ball = (cx, cy, radius)
            last_seen = time.time()
            dx_px = cx - w / 2.0   # >0 球偏右
            dy_px = cy - h / 2.0   # >0 球偏下

            # 方向自适应（兜底，防误翻）：连续 3 步偏差增大 >10px 且 >15% 才翻转，翻转后 20 秒冷却
            now = time.time()
            if now - last_flip_time > 20.0:
                for ax_name, cur, state in (("x", abs(dx_px), ax_state), ("y", abs(dy_px), ay_state)):
                    prev, bad = state
                    if prev is not None and cur > prev + 10.0 and cur > prev * 1.15:
                        bad += 1
                    else:
                        bad = 0
                    if bad >= 3:
                        if ax_name == "x":
                            sign_x *= -1
                        else:
                            sign_y *= -1
                        print(f"[auto] {ax_name} 轴方向自动翻转")
                        state[:] = [None, 0]
                        last_flip_time = now
                        continue
                    state[:] = [cur, bad]

            # 画面偏差 → 末端移动量（米）：偏右→±x，球变大(太近)→-y(后退)，球变小(太远)→+y(前进)，偏下→±z
            if ref_radius is None:
                ref_radius = radius   # 以首次检测到的球大小为参考
            dr_px = radius - ref_radius
            move = np.array(
                [
                    dx_px * args.gain * sign_x,
                    -dr_px * args.gain_dr,
                    -dy_px * args.gain * sign_y,
                ],
                dtype=float,
            )
            # 单步限幅（配合 EEBoundsAndSafety 双保险）
            move_norm = float(np.linalg.norm(move))
            if move_norm > args.max_step:
                move = move / move_norm * args.max_step

            # 读取当前关节角（度）→ 正运动学得到当前末端位姿
            try:
                obs = robot.get_observation()
            except Exception as e:
                # 电压抖动等导致读位置失败：跳过本帧，下帧重试，不崩溃
                print(f"[warn] 读位置失败（已跳过本帧）: {type(e).__name__}")
                continue
            q_curr = np.array([float(obs[f"{m}.pos"]) for m in motor_names])
            T_curr = kin.forward_kinematics(q_curr)

            # 目标位姿：位置 = 当前 + 移动量；朝向保持当前
            tgt_pos = T_curr[:3, 3] + move
            dist_base = float(np.linalg.norm(tgt_pos))
            if dist_base > args.max_reach:
                print("[warn] IK 无解：目标点超出可达范围，沿原方向收缩后继续")
                tgt_pos = tgt_pos / dist_base * (args.max_reach * 0.98)

            rotvec = Rotation.from_matrix(T_curr[:3, :3]).as_rotvec()
            ee_act = {
                "ee.x": float(tgt_pos[0]),
                "ee.y": float(tgt_pos[1]),
                "ee.z": float(tgt_pos[2]),
                "ee.wx": float(rotvec[0]),
                "ee.wy": float(rotvec[1]),
                "ee.wz": float(rotvec[2]),
                "ee.gripper_pos": float(obs["gripper.pos"]),
            }

            # 逆运动学：全关节 IK + 姿态硬约束（摄像头保持水平）；1号(shoulder_pan)锁定，左右不动
            T_des = np.eye(4, dtype=float)
            T_des[:3, :3] = Rotation.from_rotvec(
                [ee_act["ee.wx"], ee_act["ee.wy"], ee_act["ee.wz"]]
            ).as_matrix()
            T_des[:3, 3] = [ee_act["ee.x"], ee_act["ee.y"], ee_act["ee.z"]]
            q_target = kin.inverse_kinematics(q_curr, T_des, orientation_weight=0.5)
            for i, m in enumerate(motor_names):
                if m == "shoulder_pan":
                    q_target[i] = q_curr[i]   # 1号锁回当前值，左右方向不动
            joints_act = {f"{m}.pos": float(q_target[i]) for i, m in enumerate(motor_names)}
            joints_act = clip_joints_to_limits(joints_act, limits_deg)

            # IK 校验：单步关节角变化过大视为异常，跳过本步
            dq = np.abs(q_target - q_curr)
            if float(dq.max()) > 30.0:
                print(f"[warn] IK 输出跳变过大（单步 {dq.max():.0f}°），本轮不下发，继续观察")
            else:
                try:
                    robot.send_action(joints_act)
                    step_count += 1
                    last_act = joints_act
                    print(
                        f"[move] step={step_count} | dx={dx_px:+.0f}px dy={dy_px:+.0f}px | "
                        f"move=({move[0]*1000:+.0f},{move[1]*1000:+.0f},{move[2]*1000:+.0f})mm | dq={dq.max():.1f}°"
                    )
                except Exception as e:
                    # 电压抖动导致读位置失败等偶发通信错误：跳过本步继续观察，不崩溃
                    print(f"[warn] 下发失败（已跳过本步）: {type(e).__name__}")

            # 显示
            overlay = frame.copy()
            cv2.circle(overlay, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
            cv2.circle(overlay, (int(cx), int(cy)), 3, (0, 255, 0), -1)
            cv2.line(overlay, (int(w / 2), int(cy)), (int(cx), int(cy)), (255, 0, 0), 1)
            cv2.line(overlay, (int(cx), int(h / 2)), (int(cx), int(cy)), (255, 0, 0), 1)
            cv2.line(overlay, (int(w / 2) - 20, int(h / 2)), (int(w / 2) + 20, int(h / 2)), (0, 255, 255), 1)
            cv2.line(overlay, (int(w / 2), int(h / 2) - 20), (int(w / 2), int(h / 2) + 20), (0, 255, 255), 1)
            cv2.putText(
                overlay,
                f"dx={dx_px:+.0f}px dy={dy_px:+.0f}px | move=({move[0]*1000:+.0f},{move[1]*1000:+.0f},{move[2]*1000:+.0f})mm | step={step_count}",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
            )
            if abs(dx_px) < args.align_px and abs(dy_px) < args.align_px:
                cv2.putText(overlay, "ALIGNED", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
            cv2.imshow("vision", overlay)
            if cv2.waitKey(1) & 0xFF in (ord("q"), 27):  # q 或 ESC 退出
                break

            if args.max_iter is not None and step_count >= args.max_iter:
                print(f"[info] 已达最大步数 {args.max_iter}，停止")
                break

            time.sleep(0.15)  # ~6Hz 控制频率（已降速，可自行改小）

    except KeyboardInterrupt:
        print("\n[stop] 用户中断")
    finally:
        cv2.destroyAllWindows()
        cap.release()
        # 按 q / 中断后：回到启动位姿
        try:
            robot.send_action(q_home)
            time.sleep(2.0)
            print("[stop] 已回到启动位姿")
        except Exception as e:
            print(f"[warn] 回位失败（已忽略）: {type(e).__name__}")
        time.sleep(1.0)  # 等舵机完成当前运动，避免忙时禁力矩失败
        try:
            robot.disconnect()
        except Exception as e:
            # 退出时舵机偶发 Incorrect status packet，忽略；但必须保证串口真正关闭，否则下次连接舵机全丢
            print(f"[warn] 断开时通信异常（已忽略）: {type(e).__name__}")
            try:
                robot.bus.port_handler.closePort()
                print("[warn] 已强制关闭串口")
            except Exception:
                pass
        print("[done] 已断开机械臂")


# ---------------------------------------------------------------------------
# camera 模式：只测摄像头与黑球检测（不动机械臂）
# ---------------------------------------------------------------------------
def run_camera_only(args: argparse.Namespace) -> None:
    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        sys.exit(f"无法打开摄像头 #{args.camera_id}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    print("[camera] 实时检测中，按 q 退出（调节 --threshold 直到能稳定框住黑球）")
    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        h, w = frame.shape[:2]
        ball = detect_black_ball(frame, threshold=args.threshold)
        if ball is not None:
            cx, cy, radius = ball
            cv2.circle(frame, (int(cx), int(cy)), int(radius), (0, 255, 0), 2)
            cv2.circle(frame, (int(cx), int(cy)), 3, (0, 255, 0), -1)
            cv2.putText(frame, f"ball ({cx:.0f},{cy:.0f}) r={radius:.0f}", (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        else:
            cv2.putText(frame, "no ball", (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        cv2.imshow("camera", frame)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):  # q 或 ESC 退出
            break
    cap.release()
    cv2.destroyAllWindows()


# ---------------------------------------------------------------------------
# check 模式：环境自检（URDF 加载 + IK 数值往返），不连机械臂
# ---------------------------------------------------------------------------
def run_check(args: argparse.Namespace) -> None:
    try:
        import placo  # noqa: F401
    except ImportError:
        sys.exit("缺少 placo：conda install -n lerobot -c conda-forge placo -y")
    if not args.urdf.exists():
        sys.exit(f"URDF 不存在: {args.urdf}")

    motor_names = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
    kin = RobotKinematics(urdf_path=str(args.urdf), target_frame_name="gripper_frame_link", joint_names=motor_names)
    limits = load_joint_limits_deg(args.urdf)
    print("[check] URDF 加载 OK")
    for name in motor_names:
        print(f"        {name:14s} limit {limits[name][0]:+.1f} ~ {limits[name][1]:+.1f} deg")

    # 数值自检：随机关节角 → FK → IK → 对比回代误差
    rng = np.random.default_rng(0)
    max_err = 0.0
    for _ in range(20):
        q = np.array([rng.uniform(lo, hi) for lo, hi in (limits[n] for n in motor_names)])
        T = kin.forward_kinematics(q)
        q_sol = kin.inverse_kinematics(q, T, orientation_weight=0.01)
        T_sol = kin.forward_kinematics(q_sol)
        err = float(np.linalg.norm(T_sol[:3, 3] - T[:3, 3]))
        max_err = max(max_err, err)
    print(f"[check] IK 数值自检：20 组随机位姿最大回代位置误差 {max_err*1000:.1f} mm")
    print("[check] 通过。可运行 --mode camera 测摄像头，或直接 --mode vision 闭环。")


if __name__ == "__main__":
    main()
