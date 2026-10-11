import time
import serial

PORT = "COM3"
BAUD = 1_000_000
DEG2POS = 1000.0 / 240.0   # 1 度 ≈ 4.17 个位置单位


def checksum(body):
    return (~sum(body)) & 0xFF


def write(sid, addr, val):
    body = [sid, 4, 3, addr, val]
    ser.write(bytes([0xFF, 0xFF, *body, checksum(body)]))
    return ser.read(6)


def write_pos(sid, pos):
    h, l = (pos >> 8) & 0xFF, pos & 0xFF
    body = [sid, 5, 3, 0x2A, h, l]
    ser.write(bytes([0xFF, 0xFF, *body, checksum(body)]))


with serial.Serial(PORT, BAUD, timeout=0.2) as ser:
    # 开扭矩，同时检测每个舵机有没有应答
    if not write(1, 0x28, 1):
        print("1号舵机没应答！")
    if not write(2, 0x28, 1):
        print("2号舵机没应答！—— 它还不是2号，或没供电/没接好")

    print("输入两个 0~1 的数字（空格隔开，q 退出）")
    while True:
        line = input(">>> ").strip()
        if line.lower() == "q":
            break 
        a, b = map(float, line.split())
        # 同一拍里发给 1 号和 2 号，两个舵机同时动
        write_pos(1, int(300 + a * 90.0 * DEG2POS))   # 舵机1：90 度
        write_pos(2, int(300 + b * 45.0 * DEG2POS))   # 舵机2：45 度

    try:
        write(1, 0x28, 0)
        write(2, 0x28, 0)
    except Exception:
        pass
