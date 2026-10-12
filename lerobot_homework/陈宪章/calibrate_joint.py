# -*- coding: utf-8 -*-
"""逐关节小步点动并记录装配后的安全位置范围。"""

import argparse
import json
import time
from pathlib import Path

import serial


PORT = "COM5"
BAUDRATE = 1_000_000
SCAN_FROM = 1
SCAN_TO = 20
DEFAULT_JOG = 5
MAX_JOG = 10
CONTINUOUS_STEP = 5
CONTINUOUS_INTERVAL = 0.1
RANGES_FILE = Path(__file__).with_name("joint_ranges.json")


def make_frame(servo_id, instruction, params):
    body = [servo_id, len(params) + 2, instruction] + params
    return bytes([0xFF, 0xFF, *body, (~sum(body)) & 0xFF])


def read_register(ser, servo_id, address, length):
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, 0x02, [address, length]))
    reply = ser.read(length + 6)
    if (len(reply) != length + 6 or reply[:2] != b"\xff\xff"
            or reply[2] != servo_id or reply[3] != length + 2
            or sum(reply[2:]) & 0xFF != 0xFF):
        raise RuntimeError(f"ID {servo_id} 读取寄存器 0x{address:02X} 失败。")
    if reply[4] != 0:
        raise RuntimeError(f"ID {servo_id} 读取寄存器时返回错误码 {reply[4]}。")
    return reply[5:-1]


def write_register(ser, servo_id, address, values):
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, 0x03, [address, *values]))
    ser.read(6)


def read_position(ser, servo_id):
    data = read_register(ser, servo_id, 0x38, 2)
    return (data[0] << 8) | data[1]


def read_limits(ser, servo_id):
    data = read_register(ser, servo_id, 0x09, 4)
    low = (data[0] << 8) | data[1]
    high = (data[2] << 8) | data[3]
    if not 0 <= low < high <= 1023:
        raise RuntimeError(f"ID {servo_id} 读到的内部限位无效：{low}~{high}。")
    return low, high


def next_target(previous_target, delta, low_limit, high_limit):
    target = previous_target + delta
    if not low_limit <= target <= high_limit:
        raise ValueError(f"目标 {target} 超出舵机内部限制 {low_limit}~{high_limit} 步。")
    return target


def run_continuous(ser, servo_id, start_position, direction, low_limit, high_limit):
    import msvcrt

    target = start_position
    position = start_position
    print(
        f"连续点动中（每步 {CONTINUOUS_STEP} 步，间隔 {CONTINUOUS_INTERVAL:g} 秒）；"
        "直接按 q 停止（不用回车）。观察机械臂，别等它碰到硬限位。"
    )
    try:
        while True:
            if msvcrt.kbhit() and msvcrt.getwch().lower() == "q":
                write_register(ser, servo_id, 0x2A, [position >> 8, position & 0xFF])
                target = position
                print(f"\n收到 q，已停止继续推进并保持在反馈位置 {position} 步。")
                return position, target

            try:
                target = next_target(
                    target, direction * CONTINUOUS_STEP, low_limit, high_limit
                )
            except ValueError:
                print(f"\n到达舵机内部限位边界附近（{position} 步），自动停止。")
                return position, target

            write_register(ser, servo_id, 0x2A, [target >> 8, target & 0xFF])
            time.sleep(CONTINUOUS_INTERVAL)
            try:
                new_position = read_position(ser, servo_id)
            except RuntimeError as error:
                print(f"\n{error}，停止继续发送目标。")
                return position, target

            if not low_limit <= new_position <= high_limit:
                print(f"\n反馈 {new_position} 超出内部限位，自动停止。")
                return new_position, target
            if abs(new_position - position) > 100:
                print(f"\n反馈位置突变：{position} -> {new_position}，停止以避免继续运动。")
                return new_position, target

            position = new_position
            print(f"反馈={position} 目标={target}")
    except KeyboardInterrupt:
        write_register(ser, servo_id, 0x2A, [position >> 8, position & 0xFF])
        print(f"\n已中断，停止继续推进并保持在反馈位置 {position} 步。")
        return position, position


def save_joint_range(servo_id, endpoint_a, endpoint_b, low_limit, high_limit, path=RANGES_FILE):
    low, high = sorted((endpoint_a, endpoint_b))
    if not low_limit <= low < high <= high_limit:
        raise ValueError(
            f"标定范围 {low}~{high} 不在舵机内部限制 {low_limit}~{high_limit} 内。"
        )

    ranges = {}
    if path.exists():
        ranges = json.loads(path.read_text(encoding="utf-8"))
        if not isinstance(ranges, dict):
            raise ValueError(f"{path.name} 格式错误，必须是 JSON 对象。")

    ranges[str(servo_id)] = {"min_step": low, "max_step": high}
    temporary_path = path.with_suffix(path.suffix + ".tmp")
    temporary_path.write_text(
        json.dumps(ranges, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary_path.replace(path)
    return low, high


def ping(ser, servo_id):
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, 0x01, []))
    reply = ser.read(6)
    return (len(reply) == 6 and reply[:2] == b"\xff\xff"
            and reply[2] == servo_id and sum(reply[2:]) & 0xFF == 0xFF)


def main():
    parser = argparse.ArgumentParser(description="单关节小步点动和安全范围记录")
    parser.add_argument("--id", type=int, required=True, help="要标定的舵机 ID")
    parser.add_argument("--port", default=PORT, help=f"串口号（默认：{PORT}）")
    parser.add_argument(
        "--manual", action="store_true",
        help="关闭该舵机扭矩后手动引导关节，只读取并记录位置",
    )
    parser.add_argument(
        "--confirm", action="store_true",
        help="确认访问舵机；替代启动时输入 YES",
    )
    args = parser.parse_args()
    if not 1 <= args.id <= 0xFD:
        parser.error("舵机 ID 必须在 1~253 之间。")

    print(f"只会访问 ID {args.id}。确认机械臂已支撑、周围无障碍，且手远离夹点。")
    if args.manual:
        print("手动模式会关闭该舵机扭矩；舵机可能变松，必须托住机械臂。不要强扭舵机或推到硬限位。")
    if not args.confirm:
        parser.error("请确认舵机已固定且运动区域安全，并添加 --confirm 后重试。")

    try:
        ser = serial.Serial(args.port, BAUDRATE, timeout=0.2)
    except serial.SerialException as error:
        print(f"无法打开串口 {args.port}：{error}")
        return

    with ser:
        try:
            if not ping(ser, args.id):
                print(f"没有收到 ID {args.id} 的有效应答；未发送运动指令。")
                return
            low_limit, high_limit = read_limits(ser, args.id)
            position = read_position(ser, args.id)
        except RuntimeError as error:
            print(f"{error} 未发送运动指令。")
            return

        print(f"ID {args.id}：当前位置 {position} 步，舵机内部限制 {low_limit}~{high_limit} 步。")
        if not low_limit <= position <= high_limit:
            if not args.manual:
                print("当前位置超出内部限制；为安全起见拒绝点动。请先核对舵机型号、寄存器读数和限位设置。")
                return
            print("当前位置超出内部限制。仅允许手动读取；保存的两个端点仍必须位于内部限制内。")

        target_position = position
        if args.manual:
            write_register(ser, args.id, 0x28, [0])
            print("已关闭该舵机扭矩。用手轻推关节；a/b 记录两个安全端点，p 读位置，q 退出。")
            print("退出后该舵机仍保持扭矩关闭状态，请继续托住机械臂并安全断电。")
        else:
            write_register(ser, args.id, 0x28, [1])
            print("已使能该舵机。输入 + 或 - 点动 5 步；输入 +N / -N 点动 1~10 步。")
            print("也可输入 run+ / run- 连续慢速点动，期间直接按 q 停止。")
            print("a/b 记录两个安全端点，p 读位置，q 退出。看到干涉或接近硬限位时立即停止。")
        endpoint_a = None
        endpoint_b = None

        try:
            while True:
                command = input(
                    f"ID {args.id} 反馈={position} 目标={target_position}> "
                ).strip().lower()
                if command in ("q", "quit", "exit"):
                    break
                if command in ("p", "a", "b"):
                    try:
                        position = read_position(ser, args.id)
                    except RuntimeError as error:
                        print(error)
                        continue
                    if command == "p":
                        print(f"当前位置：{position} 步")
                    else:
                        if command == "a":
                            endpoint_a = position
                        else:
                            endpoint_b = position
                        print(f"已记录端点 {command.upper()}：{position} 步")
                        if endpoint_a is not None and endpoint_b is not None:
                            try:
                                low, high = save_joint_range(
                                    args.id, endpoint_a, endpoint_b,
                                    low_limit, high_limit,
                                )
                            except (OSError, ValueError, json.JSONDecodeError) as error:
                                print(f"保存失败：{error}")
                            else:
                                print(
                                    f"已保存 ID {args.id} 的范围 {low}~{high} 步到 "
                                    f"{RANGES_FILE.name}；六关节控制程序会自动读取。"
                                )
                    continue

                if args.manual:
                    print("手动模式只接受 a、b、p 或 q；请轻推关节，不要强扭。")
                    continue

                if command in ("run+", "run-"):
                    direction = 1 if command == "run+" else -1
                    position, target_position = run_continuous(
                        ser, args.id, position, direction, low_limit, high_limit
                    )
                    continue

                if command in ("+", "-"):
                    delta = DEFAULT_JOG if command == "+" else -DEFAULT_JOG
                elif len(command) > 1 and command[0] in "+-" and command[1:].isdigit():
                    amount = int(command[1:])
                    if not 1 <= amount <= MAX_JOG:
                        print(f"每次只能点动 1~{MAX_JOG} 步。")
                        continue
                    delta = amount if command[0] == "+" else -amount
                else:
                    print("命令无效；使用 +、-、+N、-N、a、b、p 或 q。")
                    continue

                try:
                    target_position = next_target(
                        target_position, delta, low_limit, high_limit
                    )
                except ValueError as error:
                    print(f"{error} 未发送。")
                    continue
                target = target_position
                write_register(ser, args.id, 0x2A, [target >> 8, target & 0xFF])
                time.sleep(0.15)
                try:
                    position = read_position(ser, args.id)
                    print(f"目标 {target} 步；反馈 {position} 步")
                except RuntimeError as error:
                    print(error)
        except (EOFError, KeyboardInterrupt):
            print("\n已停止点动。舵机保持使能，避免机械臂突然卸力。")
        else:
            print("已停止点动。舵机保持使能，避免机械臂突然卸力。")


if __name__ == "__main__":
    main()