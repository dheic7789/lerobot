# 生成运动学精简版 URDF：把 <mesh filename=.../> 替换为 <box>，供 placo 加载
import pathlib
import re

src = pathlib.Path(r"C:\Users\juyim\lerobot_homework\SO101\so101_new_calib.urdf")
out = src.with_name("so101_kinematics.urdf")
s = src.read_text(encoding="utf-8")
s2 = re.sub(r'<mesh filename="[^"]*"/>', '<box size="0.01 0.01 0.01"/>', s)
out.write_text(s2, encoding="utf-8")
print("written", out, len(s2), "bytes")
