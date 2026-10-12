#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""生成课堂练习打印图：棋盘格标定纸 + ArUco 码。"""
from pathlib import Path

import cv2
import numpy as np

out = Path("lerobot_homework/printables")
out.mkdir(parents=True, exist_ok=True)

# 1) 棋盘格：10x7 格（内角点 9x6），每格 25mm，打印像素密度 10 px/mm（100 dpi 约 3.78px/mm，取 8px/mm）
px_per_mm = 8
square = 25 * px_per_mm  # 200 px/格
cols, rows = 9, 6  # 内角点 → 格数 10x7
img = np.full(((rows + 1) * square, (cols + 1) * square), 255, np.uint8)
for r in range(rows + 1):
    for c in range(cols + 1):
        if (r + c) % 2 == 1:
            img[r * square:(r + 1) * square, c * square:(c + 1) * square] = 0
f1 = out / "chessboard_9x6_25mm.png"
cv2.imwrite(str(f1), img)
print(f"棋盘格已生成: {f1}  (打印时确保每格实际边长 25mm)")

# 2) ArUco 码 DICT_4X4_250（打印 1-3 个备用）
dict4 = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
for aid in (0, 1, 2):
    marker = cv2.aruco.generateImageMarker(dict4, aid, 400)
    marker = cv2.copyMakeBorder(marker, 80, 80, 80, 80, cv2.BORDER_CONSTANT, value=255)
    f = out / f"aruco_id{aid}_4x4_250.png"
    cv2.imwrite(str(f), marker)
    print(f"ArUco 已生成: {f}")
