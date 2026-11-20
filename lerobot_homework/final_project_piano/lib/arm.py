# -*- coding: utf-8 -*-
"""SCS215 六轴机械臂底层库（弹钢琴项目独立版本）

包含四部分：

1. SCS215 总线协议：读写位置、使能扭矩、同步下发目标（与第 3、4 周脚本一致）。
2. 舵机步数 <-> 关节角：以每个关节校准范围的中点为关节角 0°。
3. 正运动学 FK：SO-101 的 URDF 连杆链，末端取 ``gripper_frame_link``。
4. 逆运动学 IK：位置版（只解 3 个关节，末端走直线用）和位姿版（解 5 个关节，
   阻尼最小二乘）。

坐标系约定（和 lerobot / URDF 一致）：
    B = 机械臂底座 base_link，E = 末端 gripper_frame_link
    末端位姿 ^B T_E 由 FK 给出，单位米、弧度。
"""

from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np

PORT = "COM5"
BAUDRATE = 1_000_000
RANGES_FILE = Path(__file__).resolve().parent.parent / "config" / "joint_ranges.json"

RESOLUTION = 1024
POSITION_MODULUS = 1 << 10
SCS215_FULL_RANGE_DEG = 300.0
RAD_PER_STEP = math.radians(SCS215_FULL_RANGE_DEG) / (RESOLUTION - 1)

JOINT_BY_ID = {
    1: "shoulder_pan",
    2: "shoulder_lift",
    3: "elbow_flex",
    4: "wrist_flex",
    5: "wrist_roll",
    6: "gripper",
}
JOINT_ORDER = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
POSE_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]
POSITION_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex"]
ID_BY_JOINT = {name: sid for sid, name in JOINT_BY_ID.items()}

# SO101 URDF 连杆链：(子连杆, 父连杆, 安装平移 xyz, 安装姿态 rpy, 关节名)
CHAIN = [
    ("base_link", None, (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), None),
    ("shoulder_link", "base_link",
     (0.0388353, -8.97657e-09, 0.0624), (3.14159, 4.18253e-17, -3.14159), "shoulder_pan"),
    ("upper_arm_link", "shoulder_link",
     (-0.0303992, -0.0182778, -0.0542), (-1.5708, -1.5708, 0.0), "shoulder_lift"),
    ("lower_arm_link", "upper_arm_link",
     (-0.11257, -0.028, 1.73763e-16), (-3.63608e-16, 8.74301e-16, 1.5708), "elbow_flex"),
    ("wrist_link", "lower_arm_link",
     (-0.1349, 0.0052, 3.62355e-17), (4.02456e-15, 8.67362e-16, -1.5708), "wrist_flex"),
    ("gripper_link", "wrist_link",
     (5.55112e-17, -0.0611, 0.0181), (1.5708, 0.0486795, 3.14159), "wrist_roll"),
    ("gripper_frame_link", "gripper_link",
     (-0.0079, -0.000218121, -0.0981274), (0.0, 3.14159, 0.0), None),
]
EE_FRAME = "gripper_frame_link"


# ---------------------------------------------------------------------------
# 旋转与变换小工具
# ---------------------------------------------------------------------------
def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def rpy_to_rot(r: float, p: float, y: float) -> np.ndarray:
    return rot_z(y) @ rot_y(p) @ rot_x(r)


def make_transform(rotation, translation) -> np.ndarray:
    t = np.eye(4)
    t[:3, :3] = np.asarray(rotation, dtype=float)
    t[:3, 3] = np.asarray(translation, dtype=float).reshape(3)
    return t


def rotvec_to_rot(v) -> np.ndarray:
    """罗德里格斯公式：旋转向量（弧度，模长为转角）-> 旋转矩阵。"""
    v = np.asarray(v, dtype=float).reshape(3)
    theta = float(np.linalg.norm(v))
    if theta < 1e-12:
        return np.eye(3)
    k = v / theta
    kx = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + math.sin(theta) * kx + (1 - math.cos(theta)) * (kx @ kx)


def rot_to_rotvec(rotation) -> np.ndarray:
    """旋转矩阵 -> 旋转向量，用于位姿 IK 的姿态误差。"""
    r = np.asarray(rotation, dtype=float)
    cos_theta = max(-1.0, min(1.0, (np.trace(r) - 1.0) / 2.0))
    theta = math.acos(cos_theta)
    if theta < 1e-9:
        return np.array([
            (r[2, 1] - r[1, 2]) / 2.0,
            (r[0, 2] - r[2, 0]) / 2.0,
            (r[1, 0] - r[0, 1]) / 2.0,
        ])
    if abs(math.pi - theta) < 1e-6:
        axis = np.sqrt(np.maximum((np.diag(r) + 1.0) / 2.0, 0.0))
        k = int(np.argmax(np.abs(np.diag(r))))
        sign = np.sign(r[(k + 1) % 3, (k + 2) % 3]) or 1.0
        axis = axis * (1.0 if sign > 0 else -1.0)
        axis = axis / max(np.linalg.norm(axis), 1e-12)
        return axis * theta
    return (theta / (2.0 * math.sin(theta))) * np.array(
        [r[2, 1] - r[1, 2], r[0, 2] - r[2, 0], r[1, 0] - r[0, 1]]
    )


def transform_to_rvec_tvec(t) -> tuple:
    t = np.asarray(t, dtype=float)
    return rot_to_rotvec(t[:3, :3]), t[:3, 3].copy()


# ---------------------------------------------------------------------------
# SCS215 串口协议
# ---------------------------------------------------------------------------
def make_frame(servo_id: int, instruction: int, params) -> bytes:
    body = [servo_id, len(params) + 2, instruction] + list(params)
    return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])


def _send(ser, servo_id, instruction, params, expect=6):
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, instruction, params))
    return ser.read(expect) if expect else b""


def set_torque(ser, servo_id: int, enabled: bool) -> None:
    _send(ser, servo_id, 0x03, [0x28, 1 if enabled else 0])


def read_position(ser, servo_id: int) -> int:
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, 0x02, [0x38, 2]))
    reply = ser.read(8)
    if (len(reply) != 8 or reply[:2] != b"\xff\xff" or reply[2] != servo_id
            or reply[3] != 4 or reply[4] != 0 or sum(reply[2:]) & 0xFF != 0xFF):
        raise RuntimeError(f"ID {servo_id} 读取当前位置失败，请检查供电与接线。")
    return (reply[5] << 8) | reply[6]


def read_torque(ser, servo_id: int):
    """读扭矩使能寄存器（0x28）：1=使能，0=松开。读不到返回 None。"""
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, 0x02, [0x28, 1]))
    reply = ser.read(7)
    if len(reply) != 7 or reply[:2] != b"\xff\xff":
        return None
    return reply[5]


def read_voltage(ser, servo_id: int):
    """读输入电压（0x3E，单位 0.1V）。读不到返回 None。"""
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, 0x02, [0x3E, 1]))
    reply = ser.read(6)
    if len(reply) != 6 or reply[:2] != b"\xff\xff":
        return None
    return reply[5] / 10.0


def sync_move(ser, targets: dict, duration: float, speed: int) -> None:
    """同步下发多个舵机的目标位置（同一条指令，保证同时到达）。"""
    if not math.isfinite(duration) or not 0 < duration <= 65.535:
        raise ValueError("运动时间必须大于 0 且不超过 65.535 秒。")
    if not isinstance(speed, int) or isinstance(speed, bool) or not 1 <= speed <= 0xFFFF:
        raise ValueError("速度寄存器值必须是 1~65535 的整数。")
    duration_ms = max(1, min(0xFFFF, round(duration * 1000)))
    # 第二个字节是“每颗舵机写几个字节”，固定 6（位置2+时间2+速度2）。
    # 之前误写成舵机个数，导致舵机按错误长度解析、位置乱掉、直接冲到底。
    params = [0x2A, 0x06]
    for servo_id in sorted(targets):
        pos = int(targets[servo_id])
        if not 0 <= pos < POSITION_MODULUS:
            raise ValueError(f"ID {servo_id} 的目标位置 {pos} 超出 0~1023。")
        params += [servo_id, pos >> 8, pos & 0xFF,
                   duration_ms >> 8, duration_ms & 0xFF,
                   speed >> 8, speed & 0xFF]
    _send(ser, 0xFE, 0x83, params, expect=0)


def write_goal(ser, servo_id: int, position: int, duration: float, speed: int) -> None:
    """只给一颗舵机下发目标位置（地址 0x2A：位置 2 字节 + 时间 2 字节 + 速度 2 字节）。

    和 sync_move 的区别：sync_move 是"把 targets 里的每颗舵机一起动"，
    这个函数一次只动你指定的这一个关节，不会牵连其他关节。
    """
    if not math.isfinite(duration) or not 0 < duration <= 65.535:
        raise ValueError("运动时间必须大于 0 且不超过 65.535 秒。")
    duration_ms = max(1, min(0xFFFF, round(duration * 1000)))
    params = [0x2A,
              (position >> 8) & 0xFF, position & 0xFF,
              (duration_ms >> 8) & 0xFF, duration_ms & 0xFF,
              (speed >> 8) & 0xFF, speed & 0xFF]
    _send(ser, servo_id, 0x03, params)


# ---------------------------------------------------------------------------
# 标定范围 <-> 关节角
# ---------------------------------------------------------------------------
def load_ranges(path=RANGES_FILE) -> dict:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    ranges = {}
    for sid in "123456":
        low, high = data[sid]["min_step"], data[sid]["max_step"]
        if not 0 <= low < high < POSITION_MODULUS:
            raise ValueError(f"joint_ranges.json 中 ID {sid} 的范围无效。")
        ranges[int(sid)] = (int(low), int(high))
    return ranges


def find_serial_ports() -> list:
    """列出看起来像舵机控制板的串口（CH340/CH343/CP210x/FTDI/USB-Serial）。"""
    try:
        from serial.tools import list_ports
    except Exception:
        return []
    found = []
    for port in list_ports.comports():
        text = f"{port.description or ''} {port.manufacturer or ''}".lower()
        if any(key in text for key in ("ch340", "ch343", "cp210", "ftdi",
                                       "usb-serial", "usb serial", "silicon labs")):
            found.append(port.device)
    return found


def midpoints(ranges: dict) -> dict:
    return {sid: (low + high) / 2.0 for sid, (low, high) in ranges.items()}


def limits_rad(ranges: dict, mids: dict) -> dict:
    return {sid: (step_to_rad(low, mids[sid]), step_to_rad(high, mids[sid]))
            for sid, (low, high) in ranges.items()}


def step_to_rad(step: float, mid: float) -> float:
    return (step - mid) * RAD_PER_STEP


def rad_to_step(rad: float, mid: float) -> int:
    return int(round(rad / RAD_PER_STEP + mid))


def steps_to_joint_rad(steps: dict, mids: dict) -> dict:
    return {JOINT_BY_ID[sid]: step_to_rad(steps[sid], mids[sid]) for sid in range(1, 6)}


def joint_rad_to_steps(joint_rad: dict, mids: dict, ranges: dict) -> dict:
    steps = {}
    for sid in range(1, 6):
        lo, hi = ranges[sid]
        steps[sid] = max(lo, min(hi, rad_to_step(joint_rad[JOINT_BY_ID[sid]], mids[sid])))
    return steps


def clamp_joint(value: float, name: str, limits: dict) -> float:
    sid = ID_BY_JOINT[name]
    if sid in limits:
        lo, hi = limits[sid]
        return max(lo, min(hi, value))
    return value


def home_joint_rad() -> dict:
    return {name: 0.0 for name in JOINT_ORDER}


# 自动扫姿态用的关节偏移（单位：度），相对启动时的姿态。
# 目的：让装在末端的相机从尽量多的方向、距离看同一块固定住的标定板。
SWEEP_OFFSETS_DEG = [
    (0, 0, 0, 0, 0),
    # 小幅（≤10°）但方向多样：板子基本不会出画面，旋转轴又足够分散
    (8, 0, 0, 0, 0), (-8, 0, 0, 0, 0),
    (0, 6, 0, 0, 0), (0, -6, 0, 0, 0),
    (0, 0, 8, 0, 0), (0, 0, -8, 0, 0),
    (0, 0, 0, 10, 0), (0, 0, 0, -10, 0),
    (6, 5, 6, 6, 0), (-6, -5, -6, -6, 0),
    (-6, 5, -6, 6, 0), (6, -5, 6, -6, 0),
    (5, 5, 8, -8, 0), (-5, -5, -8, 8, 0),
    (8, -6, 6, 8, 0), (-8, 6, -6, -8, 0),
]


# ---------------------------------------------------------------------------
# 正运动学
# ---------------------------------------------------------------------------
def fk_frames(joint_rad: dict) -> dict:
    """返回每个连杆在 base 系下的 (旋转矩阵, 位置)。"""
    frames = {"base_link": (np.eye(3), np.zeros(3))}
    for name, parent, xyz, rpy, joint in CHAIN[1:]:
        pr, pp = frames[parent]
        rot = rpy_to_rot(*rpy)
        if joint is not None:
            rot = rot @ rot_z(joint_rad[joint])
        frames[name] = (pr @ rot, pp + pr @ np.asarray(xyz, dtype=float))
    return frames


def fk_pose(joint_rad: dict, frame: str = EE_FRAME):
    """返回末端（默认 gripper_frame_link）在 base 系下的 (R, p)。"""
    rotation, position = fk_frames(joint_rad)[frame]
    return rotation.copy(), position.copy()


def fk_transform(joint_rad: dict, frame: str = EE_FRAME) -> np.ndarray:
    rotation, position = fk_pose(joint_rad, frame)
    return make_transform(rotation, position)


def fk_position(joint_rad: dict):
    return fk_pose(joint_rad)[1]


def numeric_jacobian_pose(joint_rad: dict, joints, frame: str = EE_FRAME) -> np.ndarray:
    """位姿雅可比 6xN：上 3 行位置，下 3 行姿态（旋转向量误差）。"""
    eps = 1e-6
    base_r, base_p = fk_pose(joint_rad, frame)
    jac = np.zeros((6, len(joints)))
    q = dict(joint_rad)
    for j, name in enumerate(joints):
        q[name] = joint_rad[name] + eps
        rot, pos = fk_pose(q, frame)
        q[name] = joint_rad[name]
        jac[:3, j] = (pos - base_p) / eps
        jac[3:, j] = rot_to_rotvec(rot @ base_r.T) / eps
    return jac


# ---------------------------------------------------------------------------
# 逆运动学
# ---------------------------------------------------------------------------
def ik_position(target, joint_rad: dict, limits: dict, joints=POSITION_JOINTS,
                damping=1e-3, max_step=0.15, iters=200, tol=1e-5):
    """位置 IK：只调 ``joints``（默认前三个关节）让末端到达 target。"""
    q = dict(joint_rad)
    jac_joints = list(joints)
    target = np.asarray(target, dtype=float).reshape(3)
    eps = 1e-6
    best_error, stalled = float("inf"), 0
    for _ in range(iters):
        pos = fk_position(q)
        err = target - pos
        if np.linalg.norm(err) < tol:
            break
        error_now = float(np.linalg.norm(err))
        if error_now < best_error - 1e-6:
            best_error, stalled = error_now, 0
        else:
            stalled += 1
            if stalled >= 6:
                break
        jac = np.zeros((3, len(jac_joints)))
        for j, name in enumerate(jac_joints):
            base = q[name]
            q[name] = base + eps
            jac[:, j] = (fk_position(q) - pos) / eps
            q[name] = base
        dq = jac.T @ np.linalg.solve(jac @ jac.T + damping * np.eye(3), err)
        norm = float(np.max(np.abs(dq))) if dq.size else 0.0
        if norm > max_step:
            dq = dq * (max_step / norm)
        for j, name in enumerate(jac_joints):
            q[name] = clamp_joint(q[name] + dq[j], name, limits)
    return q, float(np.linalg.norm(target - fk_position(q)))


def ik_pose(target_rot, target_pos, joint_rad: dict, limits: dict,
            joints=POSE_JOINTS, damping=2e-3, iters=300,
            tol_pos=1e-4, tol_rot=1e-3, max_step=0.2):
    """位姿 IK：阻尼最小二乘同时逼近位置与姿态。

    返回 (关节角字典, 位置残差 m, 姿态残差 rad)。
    """
    q = dict(joint_rad)
    target_rot = np.asarray(target_rot, dtype=float)
    target_pos = np.asarray(target_pos, dtype=float).reshape(3)
    jac_joints = list(joints)
    best_error, stalled = float("inf"), 0
    for _ in range(iters):
        rot, pos = fk_pose(q)
        err = np.concatenate([target_pos - pos, rot_to_rotvec(target_rot @ rot.T)])
        if np.linalg.norm(err[:3]) < tol_pos and np.linalg.norm(err[3:]) < tol_rot:
            break
        error_now = float(np.linalg.norm(err))
        if error_now < best_error - 1e-6:
            best_error, stalled = error_now, 0
        else:
            stalled += 1
            if stalled >= 6:      # 连续几轮没有变好，说明到极限了，别死磕
                break
        jac = numeric_jacobian_pose(q, jac_joints)
        dq = jac.T @ np.linalg.solve(jac @ jac.T + damping * np.eye(6), err)
        norm = float(np.max(np.abs(dq))) if dq.size else 0.0
        if norm > max_step:
            dq = dq * (max_step / norm)
        for j, name in enumerate(jac_joints):
            q[name] = clamp_joint(q[name] + dq[j], name, limits)
    rot, pos = fk_pose(q)
    return (q,
            float(np.linalg.norm(target_pos - pos)),
            float(np.linalg.norm(rot_to_rotvec(target_rot @ rot.T))))


# ---------------------------------------------------------------------------
# 机械臂对象
# ---------------------------------------------------------------------------
@dataclass
class MoveReport:
    steps: dict
    joint_rad: dict
    position: np.ndarray
    residual: float


class Arm:
    """串口机械臂：连接、读位置、下发目标、回中。``dry_run=True`` 时不碰硬件。"""

    def __init__(self, port: str = PORT, baudrate: int = BAUDRATE,
                 ranges_file=RANGES_FILE, dry_run: bool = False,
                 duration: float = 1.5, speed: int = 250, frozen=None):
        self.port = port
        self.baudrate = baudrate
        self.dry_run = dry_run
        self.duration = duration
        self.speed = speed
        self.ranges = load_ranges(ranges_file)
        self.mids = midpoints(self.ranges)
        self.limits = limits_rad(self.ranges, self.mids)
        self.ser = None
        self.steps = {sid: int(round(self.mids[sid])) for sid in range(1, 7)}
        # 锁定的关节：任何"回中"都不会碰它们（默认锁住 5 号腕部旋转和 6 号夹爪）
        self.frozen = {5, 6} if frozen is None else {int(sid) for sid in frozen}

    def connect(self) -> None:
        if self.dry_run:
            return
        import serial  # 延迟导入：--dry-run 时不需要 pyserial

        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.2)
        except Exception as error:
            candidates = [p for p in find_serial_ports() if p != self.port]
            opened = None
            for candidate in candidates:
                try:
                    opened = serial.Serial(candidate, self.baudrate, timeout=0.2)
                    print(f"打不开 {self.port}（{error}），已自动改用 {candidate}")
                    self.port = candidate
                    break
                except Exception:
                    continue
            if opened is None:
                raise
            self.ser = opened
        # 关键：先把“当前位置”写成目标，再使能。
        # 否则舵机的目标寄存器里可能还留着上次程序设的角度，一使能就会自己转过去。
        steps, failed = {}, []
        for sid in range(1, 7):
            for _ in range(3):
                try:
                    steps[sid] = read_position(self.ser, sid)
                    break
                except Exception:
                    time.sleep(0.05)
            else:
                failed.append(sid)
        # 六颗都先“目标 = 当前位置”，再使能扭矩：
        # 这样锁定关节也会变硬（不会软塌、被手转走），但位置一点都不会变。
        if steps:
            sync_move(self.ser, steps, 0.1, self.speed)
        for sid in steps:
            set_torque(self.ser, sid, True)
        time.sleep(0.2)
        off = [sid for sid in steps if read_torque(self.ser, sid) == 0]
        if off:
            print(f"警告：{off} 号扭矩没有打开（供电不足或过载保护），用手还能转动。")
        voltages = [v for v in (read_voltage(self.ser, sid) for sid in range(1, 7)) if v]
        if voltages:
            lowest = min(voltages)
            print(f"舵机供电电压：{lowest:.1f} ~ {max(voltages):.1f} V")
            if lowest < 5.5:
                print("警告：电压低于 5.5V（在用 USB 5V 供电），六颗舵机一吃劲就会掉电复位、"
                      "扭矩自动关闭。建议换 5V/2A 以上电源或升压到 7.4V。")
        for sid in failed:      # 读不到的保持松开，但补一个占位值，避免后面 KeyError
            steps.setdefault(sid, int(round(self.mids[sid])))
        if steps:
            self.steps = steps
        print(f"连接完成：六颗舵机都已按当前位置锁住（{sorted(self.frozen)} 号只保持、不参与运动）。")
        if failed:
            print(f"警告：ID {failed} 读不到位置，这几颗没有使能（保持松开），请检查接线。")

    def close(self) -> None:
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def __enter__(self):
        self.connect()
        return self

    def __exit__(self, *exc):
        self.close()

    def read_steps(self, sync: bool = True, retries: int = 3) -> dict:
        """读六颗舵机的位置。每颗失败会自动重试，偶发的总线错误不会直接把程序带崩。"""
        if self.dry_run:
            return dict(self.steps)
        steps = {}
        for sid in range(1, 7):
            for attempt in range(retries):
                try:
                    steps[sid] = read_position(self.ser, sid)
                    break
                except Exception:
                    if attempt == retries - 1:
                        raise
                    time.sleep(0.02)
        if sync:
            self.steps = steps
        return steps

    def joint_rad(self, steps: dict | None = None) -> dict:
        return steps_to_joint_rad(steps if steps is not None else self.steps, self.mids)

    def send_steps(self, steps: dict) -> None:
        self.steps = {sid: int(steps[sid]) for sid in sorted(steps)}
        if self.ser is not None:
            sync_move(self.ser, self.steps, self.duration, self.speed)

    def settle(self, wait: float | None = None) -> None:
        time.sleep(self.duration + 0.4 if wait is None else wait)

    def ensure_torque(self) -> bool:
        """检查六颗扭矩；有掉的就“先写当前位置为目标”再重新使能。

        供电不足时舵机会在运动中途复位、扭矩自动变 OFF，这一步用来把它救回来。
        返回 True 表示曾经补过使能。
        """
        if self.ser is None:
            return False
        off = [sid for sid in range(1, 7) if read_torque(self.ser, sid) == 0]
        if not off:
            return False
        steps = {}
        for sid in off:
            try:
                steps[sid] = read_position(self.ser, sid)
            except Exception:
                pass
        if steps:
            sync_move(self.ser, steps, 0.2, self.speed)
        for sid in steps:
            set_torque(self.ser, sid, True)
        time.sleep(0.2)
        return True

    def step_joint(self, name: str, delta_rad: float, settle: bool = True):
        """让**某一个**关节在当前位置上再转 delta_rad。

        只读这一颗、只下发这一颗，绝不动其他关节。
        """
        sid = ID_BY_JOINT[name]
        lo, hi = self.ranges[sid]
        if self.ser is None:                      # dry-run
            current = self.steps[sid]
        else:
            current = read_position(self.ser, sid)
        target_rad = clamp_joint(step_to_rad(current, self.mids[sid]) + delta_rad,
                                 name, self.limits)
        target = max(lo, min(hi, rad_to_step(target_rad, self.mids[sid])))
        if self.ser is not None:
            # 用前几周验证过的同步写（0x83），里面只放这一颗舵机 ——
            # 既能只动一个关节，又走的是确定能工作的指令格式。
            sync_move(self.ser, {sid: target}, self.duration, self.speed)
        self.steps[sid] = target
        if settle:
            self.settle()
        moved_deg = math.degrees((target - current) * RAD_PER_STEP)
        return target, moved_deg

    def move_pose(self, rotation, position, joints=None, settle: bool = True,
                  max_step_deg: float = 10.0):
        # 锁定关节（默认 5、6 号）不参与 IK，避免腕部旋转被拿来“凑”目标而突然甩动
        if joints is None:
            joints = [name for sid, name in JOINT_BY_ID.items()
                      if sid in range(1, 6) and sid not in self.frozen]
        elif hasattr(joints, "__iter__"):
            joints = [name for name in joints
                      if ID_BY_JOINT.get(name) not in self.frozen]
        start = self.joint_rad()
        q, res_pos, _ = ik_pose(rotation, position, start, self.limits, joints=joints)
        biggest = max((abs(q[name] - start[name]) for name in joints), default=0.0)
        if math.degrees(biggest) > max_step_deg:
            # 不拒绝，按比例缩到上限：宁可多走几步，也不能让某一颗猛地甩一下
            scale = math.radians(max_step_deg) / biggest
            for name in joints:
                q[name] = start[name] + (q[name] - start[name]) * scale
        steps = joint_rad_to_steps(q, self.mids, self.ranges)
        steps[6] = self.steps[6]
        self.send_steps(steps)
        if settle:
            self.settle()
        return MoveReport(steps, q, fk_position(q), res_pos)

    def move_position(self, position, settle: bool = True, max_step_deg: float = 10.0):
        start = self.joint_rad()
        q, res = ik_position(position, start, self.limits)
        biggest = max((abs(q[name] - start[name]) for name in POSITION_JOINTS), default=0.0)
        if math.degrees(biggest) > max_step_deg:
            # 单步限幅：避免 IK 解突然跳一大截
            ratio = math.radians(max_step_deg) / biggest
            for name in POSITION_JOINTS:
                q[name] = start[name] + (q[name] - start[name]) * ratio
        steps = joint_rad_to_steps(q, self.mids, self.ranges)
        steps[6] = self.steps[6]
        self.send_steps(steps)
        if settle:
            self.settle()
        return MoveReport(steps, q, fk_position(q), res)

    def go_home(self, settle: bool = True, joints=None) -> dict:
        """回中：默认把 1~5 号关节移到各自范围中点；``joints`` 可指定只动哪几颗。

        被 ``self.frozen`` 锁住的关节（默认 5、6 号）**永远不会**被回中带动。
        """
        ids = list(range(1, 6)) if joints is None else list(joints)
        ids = [sid for sid in ids if sid not in self.frozen]
        if not ids:
            return {}
        targets = {sid: int(round(self.mids[sid])) for sid in ids}
        if self.ser is not None:
            sync_move(self.ser, targets, self.duration, self.speed)
        for sid, position in targets.items():
            self.steps[sid] = position
        if settle:
            self.settle()
        return targets


def main() -> None:
    parser = argparse.ArgumentParser(description="SCS215 机械臂底层库自检")
    parser.add_argument("--selftest", action="store_true", help="离线自检 FK/IK（不需要硬件）")
    args = parser.parse_args()

    ranges = load_ranges()
    mids = midpoints(ranges)
    limits = limits_rad(ranges, mids)
    q0 = home_joint_rad()
    print("零位末端位置: x=%+.4f y=%+.4f z=%+.4f m" % tuple(fk_position(q0)))
    if args.selftest:
        rng = np.random.default_rng(0)
        worst_pos, worst_rot = 0.0, 0.0
        for _ in range(50):
            q = {name: float(rng.uniform(*limits[ID_BY_JOINT[name]])) for name in JOINT_ORDER}
            rot, pos = fk_pose(q)
            near = {name: clamp_joint(q[name] + float(rng.uniform(-0.09, 0.09)), name, limits)
                    for name in JOINT_ORDER}
            _, e_pos, e_rot = ik_pose(rot, pos, near, limits)
            worst_pos, worst_rot = max(worst_pos, e_pos), max(worst_rot, e_rot)
        print(f"50 组小幅扰动 FK->IK 往返：位置最大误差 {worst_pos * 1000:.3f} mm，"
              f"姿态最大误差 {math.degrees(worst_rot):.4f}°")
        arm = Arm(dry_run=True)
        report = arm.move_position(fk_position(q0) + np.array([0.0, 0.0, 0.05]), settle=False)
        print(f"位置 IK 上移 5cm 残差: {report.residual * 1000:.4f} mm")


if __name__ == "__main__":
    main()
