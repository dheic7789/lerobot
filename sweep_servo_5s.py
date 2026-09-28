import time
import serial

PORT = "COM3"
SERVO_ID = 1
DURATION = 5.0              # 持续运动 5 秒
P_MIN, P_MAX = 300, 700     # 扫动范围（位置值，可按需改）
STEP = 20                   # 每次移动的步长
DELAY = 0.02                # 每步间隔（秒）


def write_position(value):
    """向舵机写目标位置（大端格式：高字节在前，低字节在后）"""
    high = (value >> 8) & 0xFF
    low = value & 0xFF
    body = [SERVO_ID, 5, 3, 0x2A, high, low]
    checksum = (~sum(body)) & 0xFF
    ser.write(bytes([0xFF, 0xFF, *body, checksum]))


def set_torque(on):
    """打开/关闭扭矩"""
    body = [SERVO_ID, 4, 3, 0x28, 1 if on else 0]
    checksum = (~sum(body)) & 0xFF
    ser.write(bytes([0xFF, 0xFF, *body, checksum]))
    ser.read(6)


with serial.Serial(PORT, 1_000_000, timeout=0.2) as ser:
    set_torque(True)

    start = time.time()
    pos, direction = P_MIN, 1
    while time.time() - start < DURATION:
        write_position(pos)
        pos += direction * STEP
        # 到边界就折返
        if pos >= P_MAX:
            pos, direction = P_MAX, -1
        elif pos <= P_MIN:
            pos, direction = P_MIN, 1
        time.sleep(DELAY)

    set_torque(False)
    print(f"已持续运动 {DURATION:.0f} 秒，扭矩已关闭")
