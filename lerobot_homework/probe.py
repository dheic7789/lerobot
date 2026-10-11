# 快速探测：摄像头帧+黑球检测；COM3 串口打开测试（不校准、不下发）
import sys
import time

import cv2
import serial

sys.path.insert(0, r"C:\Users\juyim\lerobot_homework")
from vision_closed_loop import detect_black_ball

print("== camera ==")
cap = cv2.VideoCapture(0)
print("isOpened:", cap.isOpened())
if cap.isOpened():
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    ok, frame = cap.read()
    print("read frame:", ok, frame.shape if ok else None)
    if ok:
        for t in (60, 80, 100):
            print("detect t=%d:" % t, detect_black_ball(frame, threshold=t))
    cap.release()

print("== COM3 ==")
try:
    s = serial.Serial("COM3", 1000000, timeout=0.3)
    print("opened COM3 OK, is_open:", s.is_open)
    s.close()
except Exception as e:
    print("COM3 open FAILED:", type(e).__name__, e)
