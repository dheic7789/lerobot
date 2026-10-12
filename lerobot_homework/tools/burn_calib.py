"""把校准文件烧写进 6 个舵机 EEPROM（Min/Max_Position_Limit），此后 is_calibrated=True 跳过交互校准"""
import json
from pathlib import Path

from lerobot.motors import Motor, MotorCalibration, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

CAL = Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration" / "robots" / "so_follower" / "my_awesome_follower_arm.json"
with open(CAL, encoding="utf-8") as f:
    raw = json.load(f)

motors = {
    "shoulder_pan": Motor(1, "scs215", MotorNormMode.RANGE_M100_100),
    "shoulder_lift": Motor(2, "scs215", MotorNormMode.RANGE_M100_100),
    "elbow_flex": Motor(3, "scs215", MotorNormMode.RANGE_M100_100),
    "wrist_flex": Motor(4, "scs215", MotorNormMode.RANGE_M100_100),
    "wrist_roll": Motor(5, "scs215", MotorNormMode.RANGE_M100_100),
    "gripper": Motor(6, "scs215", MotorNormMode.RANGE_0_100),
}
calib = {name: MotorCalibration(**raw[name]) for name in raw}

bus = FeetechMotorsBus(port="COM3", motors=motors, calibration=calib, protocol_version=1)
bus.connect()
print("connect OK, writing calibration to EEPROM...")
bus.write_calibration(calib, cache=False)
print("written.")
# 回读确认
cur = bus.read_calibration()
print("read back:", {k: (v.range_min, v.range_max) for k, v in cur.items()})
bus.disconnect()
print("done")
