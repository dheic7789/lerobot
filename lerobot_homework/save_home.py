"""读取当前六关节角度，固化为初始位 home_position.json"""
import json
from pathlib import Path

from lerobot.robots.so_follower import SO101Follower, SO101FollowerConfig

cfg = SO101FollowerConfig(port="COM3", id="my_awesome_follower_arm", use_degrees=True)
robot = SO101Follower(cfg)
robot.connect()
obs = robot.get_observation()
names = list(robot.bus.motors.keys())
home = {m: float(obs[f"{m}.pos"]) for m in names}
out = Path(__file__).resolve().parent / "home_position.json"
out.write_text(json.dumps(home, indent=2, ensure_ascii=False), encoding="utf-8")
print(f"已保存初始位: {out}")
print(json.dumps(home, indent=2, ensure_ascii=False))
robot.disconnect()
