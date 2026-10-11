#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SCS215 六自由度机械臂控制程序（课后作业）

功能
----
1. 编号（scan / set-id）：扫描串行总线上所有 SCS215 舵机，读取 ID 与型号编号；
   也可以把某个舵机重新编号（写入 ID 寄存器）。
2. 校准（calibrate）：交互式记录每个关节的安全运动范围 [min, max]，
   写入舵机 EPROM（Min_Position_Limit / Max_Position_Limit），并保存到本地校准文件。
3. 控制（control）：输入六个关节 0~1 的归一化数值（0=校准范围最小端，1=最大端），
   通过同步写（SYNC_WRITE）一次性下发六个舵机的目标位置，让六个舵机一起运动到目标位置。

依赖
----
- pyserial（Python 串口库）
- 硬件：USB 转 TTL 串行总线（如 CH340 / CP2102），六个 SCS215 舵机菊花链连接

用法示例
--------
python scs215_arm.py --port COM3 scan
python scs215_arm.py --port COM3 set-id --current 1 --new 1
python scs215_arm.py --port COM3 calibrate
python scs215_arm.py --port COM3 control 0.5 0.3 0.7 0.2 0.8 0.4
python scs215_arm.py --port COM3 control            # 交互模式，逐次输入 6 个值
python scs215_arm.py --selftest                     # 离线自测帧编解码逻辑（无需硬件）

协议说明（飞特 SCS 协议 1，参考 scservo_sdk）
-----------------------------------------------
- 指令帧：FF FF ID LEN INSTRUCTION [PARAMS...] CHECKSUM
  LEN = 参数个数 + 2；校验和 = ~(ID + LEN + INSTRUCTION + PARAMS) & 0xFF
- 状态帧：FF FF ID LEN ERROR [DATA...] CHECKSUM
- 数据小端序（低字节在前）
- 广播 ID = 0xFE（254），SYNC_WRITE 指令 = 0x83
- SCS215 参数：分辨率 1024（0~1023）、型号编号 1315、协议 1
  目标位置寄存器 Goal_Position 地址 42（2 字节）
  当前位置寄存器 Present_Position 地址 56（2 字节）
  扭矩开关 Torque_Enable 地址 40；位置限位 Min/Max 地址 9/11
"""

import argparse
import json
import sys
import time
from pathlib import Path

try:
    import serial
    from serial.tools import list_ports
except ImportError:  # 未安装 pyserial 时给出友好提示
    serial = None
    list_ports = None

# ---------------------------------------------------------------------------
# SCS 协议 1 常量
# ---------------------------------------------------------------------------
BROADCAST_ID = 0xFE   # 254，同步写广播地址
MAX_ID = 0xFC         # 252，合法 ID 上限

INST_PING = 0x01
INST_READ = 0x02
INST_WRITE = 0x03
INST_SYNC_WRITE = 0x83

# SCS215 控制表寄存器（地址, 字节数）
REG_MODEL_NUMBER = 3        # 只读，SCS215 应返回 1315
REG_ID = 5
REG_BAUD_RATE = 6
REG_MIN_POSITION_LIMIT = 9
REG_MAX_POSITION_LIMIT = 11
REG_TORQUE_ENABLE = 40
REG_GOAL_POSITION = 42
REG_PRESENT_POSITION = 56   # 只读

RESOLUTION = 1024           # SCS215 为 10bit 分辨率，位置范围 0~1023
SCS215_MODEL_NUMBER = 1315

# 六关节定义（名称, 期望 ID），对应 SO-100 跟随臂
JOINTS = [
    ("shoulder_pan", 1),
    ("shoulder_lift", 2),
    ("elbow_flex", 3),
    ("wrist_flex", 4),
    ("wrist_roll", 5),
    ("gripper", 6),
]
DEFAULT_CALIBRATION_FILE = "arm_calibration.json"


# ---------------------------------------------------------------------------
# 帧编解码（纯逻辑，可离线自测）
# ---------------------------------------------------------------------------
def make_packet(motor_id: int, instruction: int, params: list[int]) -> list[int]:
    """构造 SCS 协议 1 指令帧：[FF FF ID LEN INST P1..Pn CHK]"""
    length = len(params) + 2
    packet = [0xFF, 0xFF, motor_id & 0xFF, length, instruction] + [p & 0xFF for p in params]
    checksum = ~(sum(packet[2:])) & 0xFF  # 从 ID 起求和（不含帧头、不含校验位自身）
    packet.append(checksum)
    return packet


def parse_status(packet: list[int]) -> tuple[int, int, list[int]]:
    """解析状态帧 [FF FF ID LEN ERROR D1..Dn CHK]，返回 (id, error, data)"""
    if len(packet) < 6 or packet[0] != 0xFF or packet[1] != 0xFF:
        raise ValueError(f"非法的状态帧: {packet}")
    length = packet[3]
    if len(packet) != length + 4:
        raise ValueError(f"状态帧长度不符: 期望 {length + 4}，实际 {len(packet)}")
    checksum = ~(sum(packet[2 : length + 3])) & 0xFF
    if packet[-1] != checksum:
        raise ValueError(f"状态帧校验失败: {packet}")
    return packet[2], packet[4], packet[5 : length + 3]  # id, error, data


def make_sync_write_packet(addr: int, length: int, ids_values: dict[int, int]) -> list[int]:
    """构造 SYNC_WRITE 广播帧：FF FF FE LEN 0x83 ADDR DLEN [ID LO HI]... CHK"""
    params = [addr & 0xFF, length & 0xFF]
    for mid, value in ids_values.items():
        params += [mid & 0xFF, value & 0xFF, (value >> 8) & 0xFF]
    return make_packet(BROADCAST_ID, INST_SYNC_WRITE, params)


def reg_value(data: list[int]) -> int:
    """小端合并寄存器数据（2 字节）"""
    if len(data) == 1:
        return data[0]
    return data[0] | (data[1] << 8)


# ---------------------------------------------------------------------------
# 串口总线层
# ---------------------------------------------------------------------------
class SCS215Bus:
    """基于 pyserial 的 SCS 协议 1 总线，负责帧收发。"""

    def __init__(self, port: str, baudrate: int = 1_000_000, timeout: float = 0.1):
        if serial is None:
            sys.exit("错误：未安装 pyserial。请先执行 pip install pyserial")
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    # -- 连接管理 ----------------------------------------------------------
    def connect(self) -> None:
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        except Exception as e:
            sys.exit(f"错误：无法打开串口 {self.port}（{e}）。\n"
                     f"请确认 USB 转串口已连接，并用 --port 指定正确端口（如 COM3）。")
        self.ser.reset_input_buffer()

    def disconnect(self) -> None:
        if self.ser is not None and self.ser.is_open:
            self.ser.close()

    def _flush(self) -> None:
        self.ser.reset_input_buffer()

    # -- 帧收发 ------------------------------------------------------------
    def _write_frame(self, packet: list[int]) -> None:
        self._flush()
        self.ser.write(bytes(packet))

    def _read_frame(self, expect_id: int, timeout: float | None = None) -> list[int]:
        """读取一个状态帧并做基础校验，返回原始字节列表"""
        old_timeout = self.ser.timeout
        if timeout is not None:
            self.ser.timeout = timeout
        try:
            # 先读 4 字节头 [FF FF ID LEN]
            head = self.ser.read(4)
            if len(head) < 4:
                raise TimeoutError(f"等待 {expect_id} 号舵机响应超时")
            if head[0] != 0xFF or head[1] != 0xFF:
                raise ValueError(f"响应帧头错误: {head.hex()}")
            length = head[3]
            # 状态帧总长 = length + 4（头 4 字节 + [ERROR..CHK] 共 length 字节）
            body = self.ser.read(length)
            if len(body) < length:
                raise TimeoutError("响应不完整")
            return list(head) + list(body)
        finally:
            self.ser.timeout = old_timeout

    def _txrx(self, packet: list[int], expect_id: int) -> tuple[int, list[int]]:
        """发送指令帧并等待状态帧，返回 (error, data)"""
        self._write_frame(packet)
        resp = self._read_frame(expect_id)
        mid, error, data = parse_status(resp)
        if mid != expect_id:
            raise ValueError(f"响应 ID 不符: 期望 {expect_id}，实际 {mid}")
        if error:
            print(f"  警告：舵机 {mid} 返回错误码 0x{error:02X}（1=电压 2=角度 4=过热 8=过流 32=过载）")
        return error, data

    # -- 基础指令 ----------------------------------------------------------
    def ping(self, motor_id: int) -> int | None:
        """PING 舵机，成功后返回型号编号；无响应返回 None"""
        packet = make_packet(motor_id, INST_PING, [])
        self._write_frame(packet)
        try:
            resp = self._read_frame(motor_id, timeout=0.02)
            parse_status(resp)  # 校验
        except (TimeoutError, ValueError):
            return None
        # 读型号编号（地址 3，2 字节）
        try:
            _, data = self._txrx(
                make_packet(motor_id, INST_READ, [REG_MODEL_NUMBER, 2]), motor_id
            )
            return reg_value(data)
        except (TimeoutError, ValueError):
            return None

    def read_reg(self, motor_id: int, addr: int, length: int = 1) -> int:
        """读取寄存器值（小端）"""
        _, data = self._txrx(make_packet(motor_id, INST_READ, [addr, length]), motor_id)
        return reg_value(data)

    def write_reg(self, motor_id: int, addr: int, length: int, value: int) -> None:
        """写入寄存器（带状态响应，确保写入成功）"""
        if length == 1:
            payload = [addr, value & 0xFF]
        else:
            payload = [addr, value & 0xFF, (value >> 8) & 0xFF]
        self._txrx(make_packet(motor_id, INST_WRITE, payload), motor_id)

    def sync_write(self, addr: int, length: int, ids_values: dict[int, int]) -> None:
        """同步写：一条广播帧写入多个舵机同一寄存器（一起生效）"""
        self._write_frame(make_sync_write_packet(addr, length, ids_values))
        time.sleep(0.02)  # 广播帧无状态响应，稍候让舵机接收处理

    # -- 上层便捷方法 ------------------------------------------------------
    def scan(self, id_range: range | None = None) -> dict[int, int]:
        """扫描总线上所有舵机，返回 {id: model_number}"""
        id_range = id_range or range(MAX_ID + 1)
        found: dict[int, int] = {}
        for mid in id_range:
            model = self.ping(mid)
            if model is not None:
                found[mid] = model
        return found
    def set_torque(self, motor_id: int, enable: bool) -> None:
        self.write_reg(motor_id, REG_TORQUE_ENABLE, 1, 1 if enable else 0)

    def read_position(self, motor_id: int) -> int:
        return self.read_reg(motor_id, REG_PRESENT_POSITION, 2)


# ---------------------------------------------------------------------------
# 校准文件读写
# ---------------------------------------------------------------------------
def load_calibration(path: str | None = None) -> dict:
    path = path or DEFAULT_CALIBRATION_FILE
    if not Path(path).exists():
        sys.exit(f"错误：未找到校准文件 {path}。请先运行 calibrate 命令完成校准。")
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def save_calibration(calib: dict, path: str | None = None) -> str:
    path = path or DEFAULT_CALIBRATION_FILE
    with open(path, "w", encoding="utf-8") as f:
        json.dump(calib, f, ensure_ascii=False, indent=2)
    return path


# ---------------------------------------------------------------------------
# 各功能实现
# ---------------------------------------------------------------------------
def cmd_scan(args) -> None:
    print(f"正在扫描 {args.port}（波特率 {args.baudrate}）上的舵机……")
    bus = SCS215Bus(args.port, args.baudrate)
    bus.connect()
    try:
        id_range = None
        if args.range:
            parts = args.range.replace("-", " ").split()
            if len(parts) == 2:
                id_range = range(int(parts[0]), int(parts[1]) + 1)
            else:
                sys.exit("错误：--range 格式应为 起始-结束，例如 0-10")
        found = bus.scan(id_range)
        if not found:
            print("未发现任何舵机。请检查接线、串口和波特率。")
            return
        print(f"{'ID':>4}  {'型号编号':>8}  {'型号':>8}  {'当前状态':>12}")
        for mid in sorted(found):
            model = found[mid]
            tag = "SCS215" if model == SCS215_MODEL_NUMBER else "其他/未知"
            try:
                pos = bus.read_position(mid)
                status = f"位置 {pos}"
            except Exception:
                status = "-"
            print(f"{mid:>4}  {model:>8}  {tag:>8}  {status:>12}")
    finally:
        bus.disconnect()


def cmd_set_id(args) -> None:
    bus = SCS215Bus(args.port, args.baudrate)
    bus.connect()
    try:
        model = bus.ping(args.current)
        if model is None:
            sys.exit(f"错误：在 {args.current} 号地址未发现舵机，请先用 scan 确认当前 ID。")
        if model != SCS215_MODEL_NUMBER:
            print(f"  提示：该舵机型号编号为 {model}，不是 SCS215（{SCS215_MODEL_NUMBER}）。")
        bus.set_torque(args.current, False)  # 写 EPROM 前需关闭扭矩
        bus.write_reg(args.current, REG_ID, 1, args.new)
        print(f"完成：舵机 {args.current} 已重新编号为 {args.new}（写入 EPROM，断电保留）。")
        bus.set_torque(args.new, False)
    finally:
        bus.disconnect()


def cmd_calibrate(args) -> None:
    """
    校准流程：
    1. 关闭六个舵机扭矩，使其可被手动转动；
    2. 逐个关节：手动扳到安全范围的一端 -> Enter 记录；扳到另一端 -> Enter 记录；
    3. 把 [min, max] 写入舵机 EPROM（位置限位），并保存到本地校准文件。
    """
    bus = SCS215Bus(args.port, args.baudrate)
    bus.connect()
    try:
        # 确认舵机在位
        print("正在检查六个舵机……")
        for name, mid in JOINTS:
            model = bus.ping(mid)
            if model is None:
                print(f"  ! 未发现 {name}（ID={mid}），请检查接线后重试。")
            elif model != SCS215_MODEL_NUMBER:
                print(f"  ! {name}（ID={mid}）型号编号 {model} 不是 SCS215。")
        print()

        print("步骤 1/2：关闭扭矩（舵机将变为可手动转动）……")
        for name, mid in JOINTS:
            bus.set_torque(mid, False)
        print("完成。\n")

        if args.auto:
            # 自动模式：直接读取舵机当前限位作为校准范围
            calib = {}
            for name, mid in JOINTS:
                mn = bus.read_reg(mid, REG_MIN_POSITION_LIMIT, 2)
                mx = bus.read_reg(mid, REG_MAX_POSITION_LIMIT, 2)
                calib[name] = {"id": mid, "min": mn, "max": mx}
            print("已从舵机读取当前位置限位：")
        else:
            print("步骤 2/2：逐个记录每个关节的安全运动范围。")
            print("对每个关节：先手动扳到运动范围的一端，按 Enter 记录；")
            print("再扳到另一端，按 Enter 记录。程序会先记小值、再记大值。\n")
            calib = {}
            for name, mid in JOINTS:
                input(f"  关节 {name}（ID={mid}）：扳到【一端】后按 Enter……")
                p1 = bus.read_position(mid)
                input(f"  关节 {name}（ID={mid}）：扳到【另一端】后按 Enter……")
                p2 = bus.read_position(mid)
                mn, mx = sorted((p1, p2))
                if mn == mx:
                    print(f"    ! 两次位置相同（{mn}），请确认关节确实移动了。仍将记录该值。")
                calib[name] = {"id": mid, "min": mn, "max": mx}
                print(f"    -> {name}: [{mn}, {mx}]\n")

        # 写入舵机 EPROM
        print("正在写入位置限位到舵机 EPROM……")
        for name, c in calib.items():
            bus.write_reg(c["id"], REG_MIN_POSITION_LIMIT, 2, c["min"])
            bus.write_reg(c["id"], REG_MAX_POSITION_LIMIT, 2, c["max"])
        print("写入完成。\n")

        # 保存校准文件
        doc = {
            "model": "scs215",
            "resolution": RESOLUTION,
            "joints": calib,
            "note": "0~1 归一化数值映射: raw = u*(max-min)+min, 0=min, 1=max",
        }
        path = save_calibration(doc, args.output)
        print(f"校准完成，已保存到 {path}")
        print("校准范围：")
        for name, c in calib.items():
            print(f"  {name:<14} ID={c['id']}  [{c['min']}, {c['max']}]")
    finally:
        bus.disconnect()


def raw_from_unit(u: float, mn: int, mx: int) -> int:
    """0~1 归一化 -> 舵机原始位置"""
    u = max(0.0, min(1.0, u))
    return int(round(u * (mx - mn) + mn))


def cmd_control(args) -> None:
    calib = load_calibration(args.calibration)
    joints = calib.get("joints")
    if not joints or len(joints) != 6:
        sys.exit("错误：校准文件缺少六个关节的数据，请重新校准。")

    if args.from_motor:
        print("将从舵机读取位置限位（--from-motor）……")
        bus = SCS215Bus(args.port, args.baudrate)
        bus.connect()
        try:
            for name, c in joints.items():
                c["min"] = bus.read_reg(c["id"], REG_MIN_POSITION_LIMIT, 2)
                c["max"] = bus.read_reg(c["id"], REG_MAX_POSITION_LIMIT, 2)
        finally:
            bus.disconnect()

    bus = SCS215Bus(args.port, args.baudrate)
    bus.connect()
    try:
        print("正在启用扭矩……")
        for name, c in joints.items():
            bus.set_torque(c["id"], True)
        time.sleep(0.1)

        def move(values: list[float]) -> None:
            targets: dict[int, int] = {}
            for (name, c), u in zip(joints.items(), values):
                raw = raw_from_unit(u, c["min"], c["max"])
                targets[c["id"]] = raw
                print(f"  {name:<14} u={u:.3f} -> 目标 {raw}（范围 [{c['min']}, {c['max']}]）")
            bus.sync_write(REG_GOAL_POSITION, 2, targets)
            print("  -> 已同步下发六个舵机目标位置，舵机将一起运动。")
            time.sleep(0.5)
            try:
                actual = {name: bus.read_position(c["id"]) for name, c in joints.items()}
                print("  实际位置：" + ", ".join(f"{name}={actual[name]}" for name in joints))
            except (TimeoutError, ValueError):
                print("  （读取实际位置失败，可能正在运动或通信繁忙）")

        if args.values and len(args.values) == 6:
            move(args.values)
        elif args.values:
            sys.exit("错误：control 需要恰好 6 个 0~1 数值，或省略数值进入交互模式。")
        else:
            print("交互模式：每行输入 6 个 0~1 数值（空格分隔），输入 q 退出。")
            ranges_txt = ", ".join(f"{name}[{c['min']},{c['max']}]" for name, c in joints.items())
            print(f"当前校准：{ranges_txt}")
            while True:
                line = input("\n> ").strip()
                if line.lower() in ("q", "quit", "exit"):
                    break
                parts = line.replace(",", " ").split()
                if len(parts) != 6:
                    print("  需要 6 个数值，例如：0.5 0.3 0.7 0.2 0.8 0.4")
                    continue
                try:
                    values = [float(p) for p in parts]
                except ValueError:
                    print("  数值解析失败，请重新输入。")
                    continue
                if any(not (0.0 <= v <= 1.0) for v in values):
                    print("  数值必须在 0~1 之间。")
                    continue
                move(values)
    finally:
        bus.disconnect()


# ---------------------------------------------------------------------------
# 离线自测：验证帧编解码与归一化逻辑（无需硬件）
# ---------------------------------------------------------------------------
def selftest() -> None:
    print("=== 帧构造自测 ===")
    p = make_packet(1, INST_PING, [])
    assert p == [0xFF, 0xFF, 0x01, 0x02, 0x01, ~(0x01 + 0x02 + 0x01) & 0xFF], p
    print("PING 帧 OK:", " ".join(f"{b:02X}" for b in p))

    p = make_packet(3, INST_READ, [REG_PRESENT_POSITION, 2])
    assert len(p) == 8 and p[3] == 4
    print("READ 帧 OK:", " ".join(f"{b:02X}" for b in p))

    p = make_packet(6, INST_WRITE, [REG_GOAL_POSITION, 0x34, 0x02])
    assert p[3] == 5
    print("WRITE 帧 OK:", " ".join(f"{b:02X}" for b in p))

    p = make_sync_write_packet(REG_GOAL_POSITION, 2, {1: 512, 2: 0, 3: 1023})
    assert p[2] == BROADCAST_ID and p[4] == INST_SYNC_WRITE
    print("SYNC_WRITE 帧 OK:", " ".join(f"{b:02X}" for b in p))

    print("\n=== 状态帧解析自测 ===")
    # 模拟 PING 响应: FF FF 01 02 00 CHK
    resp = [0xFF, 0xFF, 0x01, 0x02, 0x00, ~(0x01 + 0x02 + 0x00) & 0xFF]
    mid, err, data = parse_status(resp)
    assert (mid, err, data) == (1, 0, []), (mid, err, data)
    print("PING 响应 OK:", (mid, err, data))
    # 模拟读位置响应: FF FF 03 04 00 34 02 CHK (位置 = 0x0234 = 564)
    resp = [0xFF, 0xFF, 0x03, 0x04, 0x00, 0x34, 0x02,
            ~(0x03 + 0x04 + 0x00 + 0x34 + 0x02) & 0xFF]
    mid, err, data = parse_status(resp)
    assert reg_value(data) == 564
    print("READ 响应 OK: 位置 =", reg_value(data))

    print("\n=== 归一化映射自测 ===")
    assert raw_from_unit(0.0, 100, 900) == 100
    assert raw_from_unit(1.0, 100, 900) == 900
    assert raw_from_unit(0.5, 100, 900) == 500
    assert raw_from_unit(0.25, 100, 900) == 300
    assert raw_from_unit(-0.5, 100, 900) == 100   # 越界截断
    assert raw_from_unit(2.0, 100, 900) == 900
    print("0~1 -> raw 映射 OK")

    print("\n全部自测通过 ✓")


# ---------------------------------------------------------------------------
# 命令行入口
# ---------------------------------------------------------------------------
def list_ports_text() -> str:
    if list_ports is None:
        return ""
    ports = [p.device for p in list_ports.comports()]
    return "可用串口: " + (", ".join(ports) if ports else "未检测到串口")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="SCS215 六自由度机械臂：编号 / 校准 / 控制（课后作业）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=list_ports_text(),
    )
    parser.add_argument("--port", default=None, help="串口名，如 COM3 / /dev/ttyUSB0")
    parser.add_argument("--baudrate", type=int, default=1_000_000,
                        help="总线波特率，SCS215 默认 1M（%(default)s）")
    parser.add_argument("--selftest", action="store_true", help="离线自测（无需硬件）")
    sub = parser.add_subparsers(dest="command")

    p_scan = sub.add_parser("scan", help="扫描总线上的舵机，读取编号（ID）与型号")
    p_scan.add_argument("--range", default=None,
                        help="扫描 ID 范围，如 0-10（默认 0-252）")

    p_set = sub.add_parser("set-id", help="重新编号某个舵机（写入 ID 寄存器）")
    p_set.add_argument("--current", type=int, required=True, help="舵机当前 ID")
    p_set.add_argument("--new", type=int, required=True, help="要设置的新 ID（0~252）")

    p_cal = sub.add_parser("calibrate", help="校准：记录六关节安全范围并写入舵机")
    p_cal.add_argument("--auto", action="store_true",
                       help="不交互，直接读取舵机当前位置限位作为校准范围")
    p_cal.add_argument("--output", default=DEFAULT_CALIBRATION_FILE,
                       help=f"校准文件路径（默认 {DEFAULT_CALIBRATION_FILE}）")

    p_ctl = sub.add_parser("control", help="控制：输入六关节 0~1 数值，一起运动到目标")
    p_ctl.add_argument("values", nargs="*", type=float,
                       help="六个 0~1 数值；省略则进入交互模式")
    p_ctl.add_argument("--calibration", default=DEFAULT_CALIBRATION_FILE,
                       help=f"校准文件路径（默认 {DEFAULT_CALIBRATION_FILE}）")
    p_ctl.add_argument("--from-motor", action="store_true",
                       help="不从校准文件读取，而是直接从舵机读取位置限位")
    return parser


def main() -> None:
    parser = build_parser()
    args = parser.parse_args()

    if args.selftest:
        selftest()
        return

    if not args.command:
        parser.print_help()
        sys.exit("\n请指定子命令：scan / set-id / calibrate / control")

    if not args.port:
        sys.exit("错误：请用 --port 指定串口（例如 --port COM3）。\n" + list_ports_text())

    if args.command == "scan":
        cmd_scan(args)
    elif args.command == "set-id":
        cmd_set_id(args)
    elif args.command == "calibrate":
        cmd_calibrate(args)
    elif args.command == "control":
        cmd_control(args)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
