import argparse
import time

import serial

PORT = "COM5"
BAUDRATE = 1_000_000
SCAN_FROM = 1
SCAN_TO = 20


def make_frame(servo_id, instruction, params):
    body = [servo_id, len(params) + 2, instruction] + params
    checksum = (~sum(body)) & 0xFF
    return bytes([0xFF, 0xFF, *body, checksum])


def send(ser, servo_id, address, value):
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, 0x03, [address, value]))
    reply = ser.read(6)
    time.sleep(0.1)
    return reply


def ping(ser, servo_id):
    ser.reset_input_buffer()
    ser.write(make_frame(servo_id, 0x01, []))
    reply = ser.read(6)
    return len(reply) == 6 and reply[0:2] == b"\xff\xff" and reply[2] == servo_id


def find_ids(ser):
    return [servo_id for servo_id in range(SCAN_FROM, SCAN_TO + 1) if ping(ser, servo_id)]


def main():
    parser = argparse.ArgumentParser(
        description="改一颗已与其他舵机隔离的舵机 ID；默认扫描 ID 1~20。"
    )
    parser.add_argument("--port", default=PORT, help=f"串口号（默认：{PORT}）")
    parser.add_argument("--new-id", type=int, required=True, help="这颗舵机要改成的 ID")
    args = parser.parse_args()

    if not 1 <= args.new_id <= 0xFD:
        parser.error("新 ID 必须在 1~253 之间。")

    print("注意：多个同 ID 舵机在扫描中可能只显示一个 ID，程序无法判断实际数量。")
    print("安全确认：断电后已将总线隔离为只连接一颗舵机，并已支撑机械臂。")
    if input("确认以上操作？输入 YES 继续：").strip() != "YES":
        print("已取消，未发送指令。")
        return

    try:
        ser = serial.Serial(args.port, BAUDRATE, timeout=0.2)
    except serial.SerialException as error:
        print(f"无法打开串口 {args.port}：{error}")
        return

    with ser:
        ids = find_ids(ser)
        if len(ids) != 1:
            print(f"扫描到的 ID：{ids or '无'}。需要恰好检测到一个 ID 才能继续；未改写。")
            return

        old_id = ids[0]
        print(f"检测到当前 ID={old_id}，目标 ID={args.new_id}。")
        if old_id == args.new_id:
            print("当前 ID 已是目标值，无需修改。")
            return

        if input("确认只修改这颗舵机？输入 YES 继续：").strip() != "YES":
            print("已取消，未发送改号指令。")
            return

        send(ser, old_id, 0x28, 0)          # 关闭扭矩
        send(ser, old_id, 0x30, 0)          # 解锁 EEPROM
        send(ser, old_id, 0x05, args.new_id)  # 写入新 ID
        send(ser, args.new_id, 0x30, 1)     # 使用新 ID 锁定 EEPROM

        if ping(ser, args.new_id):
            print(f"验证成功：舵机 ID 已从 {old_id} 改为 {args.new_id}。")
        else:
            print(f"未能通过 PING 验证新 ID={args.new_id}。请勿重复改号，先检查接线并重新扫描。")


if __name__ == "__main__":
    main()
