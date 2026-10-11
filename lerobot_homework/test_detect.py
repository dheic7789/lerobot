import sys
import cv2

sys.path.insert(0, r"C:\Users\juyim\lerobot_homework")
from vision_closed_loop import detect_black_ball

img = cv2.imread(r"C:\Users\juyim\AppData\Local\Temp\black_ball.png")
print("img", img.shape)
for t in (40, 50, 60, 80, 100, 120):
    res = detect_black_ball(img, threshold=t)
    print(f"threshold={t}: {res}")
# 期望：球在图中心偏右，半径约 300+ 像素
