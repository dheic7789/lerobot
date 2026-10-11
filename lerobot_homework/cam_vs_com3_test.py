"""验证：摄像头打开是否导致 COM3 舵机丢失"""
import time

import cv2

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


def try_connect(tag: str) -> bool:
    try:
        bus = FeetechMotorsBus(port="COM3", motors=motors, protocol_version=1)
        bus.connect()
        bus.disconnect()
        print(f"[{tag}] connect OK")
        return True
    except RuntimeError as e:
        print(f"[{tag}] FAIL: {str(e).splitlines()[1] if 'Missing' in str(e) else e}")
        return False


# 1) 无摄像头
try_connect("no-cam")

# 2) 开摄像头 1280x720
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
ret, frame = cap.read()
print(f"cam1280: ret={ret} shape={frame.shape if ret else None}")
try_connect("cam1280")

# 3) 关掉重开 640x480
cap.release()
cap = cv2.VideoCapture(1)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
ret, frame = cap.read()
print(f"cam640: ret={ret} shape={frame.shape if ret else None}")
try_connect("cam640")
cap.release()
