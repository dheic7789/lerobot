# -*- coding: utf-8 -*-
"""钢琴项目独立公共工具：串口、配置和安全预检。"""

from __future__ import annotations

import json
import sys
from pathlib import Path

import serial

ROOT = Path(__file__).resolve().parent
LIB_DIR = ROOT / "lib"
CONFIG_DIR = ROOT / "config"
DEFAULT_RANGES = CONFIG_DIR / "joint_ranges.json"

if str(LIB_DIR) not in sys.path:
    sys.path.insert(0, str(LIB_DIR))

import arm as arm_lib  # noqa: E402


def load_json(path: Path, required: bool = True) -> dict:
    path = Path(path)
    if not path.exists():
        if required:
            raise FileNotFoundError(f"找不到配置文件：{path}")
        return {}
    return json.loads(path.read_text(encoding="utf-8"))


def save_json(path: Path, payload: dict) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def _ping_servo(ser, servo_id: int) -> bool:
    ser.reset_input_buffer()
    ser.write(arm_lib.make_frame(servo_id, 0x01, []))
    reply = ser.read(6)
    return (
        len(reply) == 6
        and reply[:2] == b"\xff\xff"
        and reply[2] == servo_id
        and sum(reply[2:]) & 0xFF == 0xFF
    )


def _read_register(ser, servo_id: int, address: int, length: int):
    ser.reset_input_buffer()
    ser.write(arm_lib.make_frame(servo_id, 0x02, [address, length]))
    reply = ser.read(length + 6)
    if (
        len(reply) != length + 6
        or reply[:2] != b"\xff\xff"
        or reply[2] != servo_id
        or reply[3] != length + 2
        or reply[4] != 0
        or sum(reply[2:]) & 0xFF != 0xFF
    ):
        return None
    return reply[5:-1]


def find_robot_port(explicit: str | None = None) -> str | None:
    if explicit:
        return explicit
    try:
        from serial.tools import list_ports
    except Exception:
        return None
    candidates = []
    for port in list_ports.comports():
        text = f"{port.description or ''} {port.manufacturer or ''}".lower()
        if any(key in text for key in (
            "ch340", "ch343", "cp210", "ftdi", "usb-serial", "usb serial",
            "silicon labs",
        )):
            candidates.append(port.device)
    for device in candidates:
        try:
            with serial.Serial(device, arm_lib.BAUDRATE, timeout=0.15) as ser:
                if _ping_servo(ser, 1):
                    return device
        except Exception:
            continue
    return None


def servo_state_preflight(port: str, ranges: dict) -> None:
    """使能前检查六颗舵机均可通信，且当前位置在软件范围内。"""
    problems = []
    with serial.Serial(port, arm_lib.BAUDRATE, timeout=0.20) as ser:
        for sid in range(1, 7):
            if not _ping_servo(ser, sid):
                problems.append(f"ID {sid} 无应答")
                continue
            position = arm_lib.read_position(ser, sid)
            low, high = ranges[sid]
            if not low <= position <= high:
                problems.append(
                    f"ID {sid} 当前 {position} 超出软件范围 {low}~{high}"
                )
    if problems:
        raise RuntimeError(
            "上电前检查失败，拒绝使能扭矩：\n  - "
            + "\n  - ".join(problems)
        )
