"""真机测试：lerobot FeetechMotorsBus 连接 SCS215 六舵机（只读，不动舵机）"""
import sys

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

motors = {
    "shoulder_pan": Motor(1, "scs215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "scs215", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "scs215", MotorNormMode.RANGE_M100_100),
    "wrist_flex": Motor(4, "scs215", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(5, "scs215", MotorNormMode.RANGE_M100_100),
    "gripper": Motor(6, "scs215", MotorNormMode.RANGE_0_100),
}

bus = FeetechMotorsBus(port="COM3", motors=motors, protocol_version=1)
print("connect...")
bus.connect()
print("connect OK")

# 读原始位置（normalize=False 直接读寄存器）
pos = {m: bus.read("Present_Position", m, normalize=False) for m in bus.motors}
print("Present_Position (raw):", pos)

voltage = {m: bus.read("Present_Voltage", m, normalize=False) for m in bus.motors}
print("Present_Voltage (0.1V):", voltage)

# 型号校验（connect 已通过，再显式确认）
from lerobot.motors.feetech.feetech import MODEL_NUMBER_TABLE
print("expected model:", {m: MODEL_NUMBER_TABLE[b.motors[m].model] for m, b in [("x", bus)]})

bus.disconnect()
print("done")
