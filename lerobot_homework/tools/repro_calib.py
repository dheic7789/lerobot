"""复现校准流程错误"""
import traceback

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
bus.connect()
print("connect OK")

try:
    homing = bus.set_half_turn_homings()
    print("homing:", homing)
    pos = {m: bus.read("Present_Position", m, normalize=False, num_retry=5) for m in bus.motors}
    print("positions:", pos)
    bus.disconnect()
    print("done")
except Exception:
    traceback.print_exc()
    try:
        bus.disconnect()
    except Exception:
        pass
