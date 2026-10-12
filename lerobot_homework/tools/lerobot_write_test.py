"""真机测试：写路径字节序验证（1 号舵机微动 ±50，然后恢复）"""
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

motors = {
    "shoulder_pan": Motor(1, "scs215", MotorNormMode.RANGE_M100_100),
}
bus = FeetechMotorsBus(port="COM3", motors=motors, protocol_version=1)
bus.connect()
print("connect OK")

cur = bus.read("Present_Position", "shoulder_pan", normalize=False)
print(f"current pos = {cur}")

target = max(0, min(cur + 50, 1023))
print(f"write Goal_Position -> {target}")
bus.write("Goal_Position", "shoulder_pan", target, normalize=False)
time.sleep(3)
after = bus.read("Present_Position", "shoulder_pan", normalize=False)
print(f"pos after 3s = {after}")
ok = abs(after - target) <= 5
print(f"MOVED OK: {ok}")

# 恢复原位
print(f"restore -> {cur}")
bus.write("Goal_Position", "shoulder_pan", cur, normalize=False)
time.sleep(3)
final = bus.read("Present_Position", "shoulder_pan", normalize=False)
print(f"pos restored = {final}")

bus.disconnect()
print("done")
