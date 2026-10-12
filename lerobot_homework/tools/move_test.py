"""让能动的舵机动一下：4/5/6 各 +80 步，读回确认，再移回原位"""
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

IDS = [4, 5, 6]
DELTA = 80

motors = {str(i): Motor(i, "scs215", MotorNormMode.RANGE_M100_100) for i in IDS}
bus = FeetechMotorsBus(port="COM3", motors=motors, protocol_version=1)
bus.connect()
print("connect OK, 在线:", IDS)

before = {str(i): bus.read("Present_Position", str(i), normalize=False) for i in IDS}
print("运动前:", before)

for i in IDS:
    bus.write("Goal_Position", str(i), before[str(i)] + DELTA, normalize=False)
time.sleep(2.0)

after = {str(i): bus.read("Present_Position", str(i), normalize=False) for i in IDS}
print("运动后:", after)

for i in IDS:
    bus.write("Goal_Position", str(i), before[str(i)], normalize=False)
time.sleep(2.0)

back = {str(i): bus.read("Present_Position", str(i), normalize=False) for i in IDS}
print("移回后:", back)
bus.disconnect()
print("done")
