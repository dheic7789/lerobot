"""把自研 arm_calibration.json 转成 lerobot 校准文件"""
import json
from pathlib import Path

SRC = Path(r"D:\lerobot_workspace\lerobot_homework\arm_calibration.json")
DST = Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration" / "robots" / "so_follower" / "my_awesome_follower_arm.json"

with open(SRC, encoding="utf-8") as f:
    data = json.load(f)

calib = {}
for name, j in data["joints"].items():
    calib[name] = {
        "id": j["id"],
        "drive_mode": 0,
        "homing_offset": 0,  # SCS215 无 Homing_Offset 寄存器，协议 1 恒为 0
        "range_min": j["min"],
        "range_max": j["max"],
    }

DST.parent.mkdir(parents=True, exist_ok=True)
with open(DST, "w", encoding="utf-8") as f:
    json.dump(calib, f, indent=4)

print("written:", DST)
print(json.dumps(calib, indent=2))
