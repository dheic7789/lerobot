#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Week7 机械臂工具层：把 SO-ARM101 的感知与控制能力封装成可被 LLM Agent 调用的工具。

提供 6 个工具接口（4.2 要求至少 3 个）：
  - get_robot_state()    返回关节角、追踪状态、错误状态
  - move_to_joints()     按关节角移动（带限位裁剪 + 降速）
  - move_to_pose()       按末端目标位置移动（placo IK，锁腕 4-6）
  - detect_object()      根据文本描述检测物体（当前支持黑色小球）
  - start_tracking()     启动视觉伺服追踪
  - stop_tracking()      停止追踪
  - capture_frame()      拍照保存（任务一用）

支持 --dry-run 仿真模式（不连机械臂，验证 Agent 流程）。
"""
import json
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import cv2
import numpy as np

URDF_PATH = Path(__file__).resolve().parent.parent / "tools" / "SO101" / "so101_kinematics.urdf"

# 关节顺序 = 舵机 ID 1-6 顺序（与 lerobot SO101Follower 校准一致）
JOINT_NAMES = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# 关节中文说明（VLM prompt 用）
JOINT_HUMAN = {
    "shoulder_pan":  "肩部旋转（左右摆动整条手臂），负=向右转，正=向左转",
    "shoulder_lift": "肩部抬升（手臂上下摆动），负=手臂下垂，正=手臂抬起",
    "elbow_flex":    "肘部弯曲（小臂相对大臂折起），负=伸直，正=弯曲",
    "wrist_flex":    "腕部俯仰（末端上下点头），负=向下，正=向上",
    "wrist_roll":    "腕部旋转（末端绕自身轴线转），负=逆时针，正=顺时针",
    "gripper":       "夹爪开合，负/小=闭合，正/大=张开",
}


def load_joint_limits_deg(urdf_path: Path = URDF_PATH) -> dict[str, tuple[float, float]]:
    """从 URDF 读取各关节角度限位（度）。"""
    tree = ET.parse(urdf_path)
    limits = {}
    for joint in tree.getroot().iter("joint"):
        name = joint.get("name")
        lim = joint.find("limit")
        if lim is None or name is None:
            continue
        limits[name] = (float(lim.get("lower", 0)) * 180 / np.pi,
                        float(lim.get("upper", 0)) * 180 / np.pi)
    return limits


def clip_joints(joints: dict) -> dict:
    """把关节角裁剪到 URDF 限位内，返回裁剪后的 dict。"""
    limits = load_joint_limits_deg()
    out = {}
    for name, val in joints.items():
        lo, hi = limits.get(name, (-180, 180))
        out[name] = float(min(max(float(val), lo), hi))
    return out


class ArmController:
    """SO-ARM101 控制/感知工具层。dry_run=True 时仿真，不连机械臂。"""

    def __init__(self, port: str = "COM3", dry_run: bool = False, run_time: int = 800):
        self.port = port
        self.dry_run = dry_run
        self.run_time = run_time
        self.tracking = False
        self.tracking_object = None
        self.robot = None
        self.kin = None
        self.motor_names = None
        if not dry_run:
            self._connect()

    # ---- 连接 --------------------------------------------------------
    def _connect(self):
        from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig
        from lerobot.model.kinematics import RobotKinematics
        cfg = SO101FollowerConfig(port=self.port, id="my_awesome_follower_arm", use_degrees=True)
        robot = SO101Follower(cfg)
        robot.connect()
        # SCS215 降速（地址44，ms/步），避免瞬时电流过大触发过载保护
        for m in robot.bus.motors:
            robot.bus.write("Running_Time", m, self.run_time, normalize=False)
        self.robot = robot
        self.motor_names = list(robot.bus.motors.keys())
        self.kin = RobotKinematics(
            urdf_path=str(URDF_PATH), target_frame_name="gripper_frame_link", joint_names=self.motor_names)
        print(f"[arm] 已连接 {self.port}（降速 {self.run_time}ms/步）")

    def close(self):
        if self.robot is not None:
            try:
                self.robot.disconnect()
            except Exception:
                pass

    # ---- 工具 1：查询状态 ----------------------------------------------
    def get_robot_state(self) -> dict:
        """返回关节角、追踪状态和错误状态。"""
        if self.dry_run:
            return {"joints_deg": {n: 0.0 for n in JOINT_NAMES},
                    "tracking": self.tracking,
                    "tracking_object": self.tracking_object,
                    "error": None}
        try:
            obs = self.robot.get_observation()
            joints = {f"{m}.pos": float(obs[f"{m}.pos"]) for m in self.motor_names}
            return {"joints_deg": joints, "tracking": self.tracking,
                    "tracking_object": self.tracking_object, "error": None}
        except Exception as e:
            return {"joints_deg": None, "tracking": self.tracking,
                    "tracking_object": self.tracking_object, "error": f"{type(e).__name__}: {e}"}

    # ---- 工具 2：按关节角移动 -------------------------------------------
    def move_to_joints(self, joints: dict, run_time_ms: int | None = None) -> dict:
        """按关节角移动。joints: {"shoulder_pan": 0.0, ...}，自动裁剪限位。"""
        clipped = clip_joints(joints)
        if self.dry_run:
            print(f"[dry-run] move_to_joints -> {json.dumps(clipped, ensure_ascii=False)}")
            return {"ok": True, "applied": clipped, "clipped": clipped != joints, "dry_run": True}
        try:
            action = {f"{m}.pos": float(clipped.get(m, 0.0)) for m in self.motor_names}
            self.robot.send_action(action)
            time.sleep(0.5)
            return {"ok": True, "applied": clipped, "clipped": clipped != joints}
        except Exception as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}

    # ---- 工具 3：按末端位置移动（IK，锁腕）---------------------------------
    def move_to_pose(self, x: float, y: float, z: float) -> dict:
        """末端移动到目标位置 (x,y,z) 米（基座坐标系，x=前 y=左 z=上）。
        位置 IK 解算 1-3 关节，4-6（腕部/夹爪）保持当前值。"""
        if self.dry_run:
            print(f"[dry-run] move_to_pose -> ({x}, {y}, {z})")
            return {"ok": True, "target": {"x": x, "y": y, "z": z}, "dry_run": True}
        try:
            obs = self.robot.get_observation()
            q_curr = np.array([float(obs[f"{m}.pos"]) for m in self.motor_names])
            T_curr = self.kin.forward_kinematics(q_curr)
            target = np.eye(4)
            target[:3, :3] = T_curr[:3, :3]
            target[:3, 3] = [x, y, z]
            q_sol = self.kin.inverse_kinematics(q_curr, target, orientation_weight=0.01)
            q_sol[3:] = q_curr[3:]  # 锁腕 4-6
            clipped = {m: float(np.clip(q_sol[i], *self._limit(m))) for i, m in enumerate(self.motor_names)}
            action = {f"{m}.pos": clipped[m] for m in self.motor_names}
            self.robot.send_action(action)
            time.sleep(0.5)
            return {"ok": True, "target": {"x": x, "y": y, "z": z}, "applied_joints": clipped}
        except Exception as e:
            return {"ok": False, "error": f"{type(e).__name__}: {e}"}

    def _limit(self, name):
        return load_joint_limits_deg().get(name, (-180, 180))

    # ---- 工具 4：物体检测（黑色小球）--------------------------------------
    def detect_object(self, description: str) -> dict:
        """根据文本描述检测物体。当前支持：黑色小球/球（ball），返回 found/cx/cy/radius。"""
        desc = description.lower()
        if not any(k in desc for k in ("黑", "球", "ball", "circle", "圆")):
            return {"found": False, "reason": f"暂不支持描述 '{description}'（当前仅支持黑色小球）"}
        if self.dry_run:
            return {"found": True, "object_id": 1, "cx": 640, "cy": 360, "radius_px": 80, "dry_run": True}
        ok, frame = self._read_frame()
        if not ok:
            return {"found": False, "reason": "无法读取相机画面"}
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mask = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best = None
        for c in contours:
            (x, y), r = cv2.minEnclosingCircle(c)
            if 20 <= r <= 250:
                area_ratio = cv2.contourArea(c) / (np.pi * r * r)
                if area_ratio > 0.5 and (best is None or r > best[2]):
                    best = (int(x), int(y), int(r))
        if best is None:
            return {"found": False, "reason": "画面中未检测到黑色小球"}
        cx, cy, r = best
        return {"found": True, "object_id": 1, "cx": cx, "cy": cy, "radius_px": r}

    def _read_frame(self, camera_id: int = 1):
        cap = cv2.VideoCapture(camera_id)
        if not cap.isOpened():
            return False, None
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        ok, frame = cap.read()
        cap.release()
        return ok, frame

    # ---- 工具 5/6：追踪启停（状态机）---------------------------------------
    def start_tracking(self, object_id: int) -> dict:
        """启动视觉伺服追踪（当前为状态机；真机可接入 vision_closed_loop.py 子进程）。"""
        if self.tracking:
            return {"ok": True, "message": f"已在追踪 object_id={self.tracking_object}"}
        self.tracking = True
        self.tracking_object = object_id
        return {"ok": True, "message": f"开始追踪 object_id={object_id}"}

    def stop_tracking(self) -> dict:
        """停止当前追踪任务。"""
        self.tracking = False
        self.tracking_object = None
        return {"ok": True, "message": "已停止追踪"}


def tools_schema() -> list[dict]:
    """OpenAI function calling 工具声明（4.2 展示用）。"""
    return [
        {"type": "function", "function": {
            "name": "get_robot_state",
            "description": "返回机械臂当前关节角（度）、追踪状态和错误状态。执行任何动作前建议先调用。",
            "parameters": {"type": "object", "properties": {}}}} ,
        {"type": "function", "function": {
            "name": "move_to_joints",
            "description": "按关节角（度）移动机械臂。关节名：" + ", ".join(JOINT_NAMES) +
                           "。各关节范围：" + "; ".join(
                               f"{n}={load_joint_limits_deg().get(n, ('', ''))[0]:.0f}~{load_joint_limits_deg().get(n, ('', ''))[1]:.0f}"
                               for n in JOINT_NAMES),
            "parameters": {"type": "object", "properties": {
                "joints": {"type": "object", "description": "关节角字典，如 {\"shoulder_lift\": 30, \"elbow_flex\": 40}"}}},
            "required": ["joints"]}},
        {"type": "function", "function": {
            "name": "move_to_pose",
            "description": "把机械臂末端移动到目标位置 (x, y, z) 米（基座坐标系：x=前、y=左、z=上）。位置 IK 自动解算关节，腕部保持。",
            "parameters": {"type": "object", "properties": {
                "x": {"type": "number"}, "y": {"type": "number"}, "z": {"type": "number"}},
                "required": ["x", "y", "z"]}}},
        {"type": "function", "function": {
            "name": "detect_object",
            "description": "根据文本描述在腕部相机画面中检测物体。当前支持黑色小球（描述含“黑/球/ball/圆”）。返回 found、object_id、像素位置。",
            "parameters": {"type": "object", "properties": {
                "description": {"type": "string", "description": "物体文本描述，如 '黑色小球'"}},
                "required": ["description"]}}},
        {"type": "function", "function": {
            "name": "start_tracking",
            "description": "启动视觉伺服追踪，让机械臂持续跟随指定物体（object_id 来自 detect_object）。",
            "parameters": {"type": "object", "properties": {
                "object_id": {"type": "integer"}}, "required": ["object_id"]}}},
        {"type": "function", "function": {
            "name": "stop_tracking",
            "description": "停止当前追踪任务。",
            "parameters": {"type": "object", "properties": {}}}},
    ]


if __name__ == "__main__":
    # 简单自检（dry-run）
    arm = ArmController(dry_run=True)
    print(json.dumps(arm.get_robot_state(), ensure_ascii=False))
    print(json.dumps(arm.move_to_joints({"shoulder_lift": 30, "elbow_flex": 40, "gripper": 50}), ensure_ascii=False))
    print(json.dumps(arm.move_to_pose(0.2, 0.0, 0.3), ensure_ascii=False))
    print(json.dumps(arm.detect_object("黑色小球"), ensure_ascii=False))
    print(json.dumps(arm.start_tracking(1), ensure_ascii=False))
    print(json.dumps(arm.stop_tracking(), ensure_ascii=False))
