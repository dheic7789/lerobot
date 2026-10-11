"""完整连接测试：SO101Follower + 校准文件（不碰摄像头，不动舵机）"""
from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

cfg = SO101FollowerConfig(port="COM3", id="my_awesome_follower_arm")
robot = SO101Follower(cfg)

robot.connect()
print("connect OK")
print("is_calibrated:", robot.is_calibrated)
print("calibration loaded:", {k: (v.range_min, v.range_max) for k, v in robot.calibration.items()})

obs = robot.get_observation()
print("observation:", {k: round(v, 1) for k, v in obs.items() if k.endswith(".pos")})

robot.disconnect()
print("done")
