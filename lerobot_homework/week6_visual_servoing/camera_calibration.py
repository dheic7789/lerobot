#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""相机内参标定（课堂练习 1/4）：OpenCV 棋盘格标定。

用法：
  python camera_calibration.py --camera-id 1

操作：
  - 移动棋盘格纸，使其角点被绿框标出；按 空格 保存当前帧
    （每张必须改变棋盘格相对相机的位姿，建议 ≥15 张）
  - 按 c    用已保存照片标定，打印内参 + 重投影误差，保存 camera_intrinsics.json
  - 按 q    退出

默认棋盘格：内角点 9×6，边长 25mm（--square-size 0.025）。
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--camera-id", type=int, default=1, help="摄像头编号（默认 1=机器人相机）")
    p.add_argument("--cols", type=int, default=9, help="棋盘格内角点列数（默认 9）")
    p.add_argument("--rows", type=int, default=6, help="棋盘格内角点行数（默认 6）")
    p.add_argument("--square-size", type=float, default=0.025, help="棋盘格边长，米（默认 0.025）")
    p.add_argument("--out-dir", type=Path, default=Path("lerobot_homework/calib_data"),
                   help="照片与结果保存目录")
    return p.parse_args()


def main() -> None:
    args = parse_args()
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    imgs_dir = out_dir / "images"
    imgs_dir.mkdir(exist_ok=True)

    pattern = (args.cols, args.rows)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square_size

    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        raise SystemExit(f"无法打开摄像头 #{args.camera_id}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    saved = 0
    print("移动棋盘格，绿框=检测到角点。空格=保存  c=标定  q=退出")
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern, None)
        disp = frame.copy()
        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(disp, pattern, corners2, ret)
        cv2.putText(disp, f"saved={saved}  [SPACE] save  [C] calibrate  [Q] quit",
                    (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("calibration", disp)

        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord(" ") and ret:
            cv2.imwrite(str(imgs_dir / f"img_{saved:03d}.png"), frame)
            saved += 1
            print(f"[save] 第 {saved} 张: {imgs_dir / f'img_{saved - 1:03d}.png'}")
        if key == ord("c"):
            break

    cap.release()
    cv2.destroyAllWindows()

    if saved < 5:
        raise SystemExit(f"照片太少（{saved} 张），至少需要 5 张（建议 ≥15 张）")

    # ---------------- 标定 ----------------
    obj_points, img_points = [], []
    files = sorted(imgs_dir.glob("img_*.png"))
    used = 0
    for f in files:
        img = cv2.imread(str(f))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern, None)
        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_points.append(objp)
            img_points.append(corners2)
            used += 1
        else:
            print(f"[warn] {f.name} 未检测到角点，跳过")
    if used < 5:
        raise SystemExit(f"可用照片不足（{used} 张）")

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

    # 重投影误差
    total_err = 0.0
    for i in range(len(obj_points)):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
        err = cv2.norm(img_points[i], proj, cv2.NORM_L2) / len(proj)
        total_err += err
    mean_err = total_err / len(obj_points)

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    result = {
        "K": K.tolist(),
        "dist": dist.flatten().tolist(),
        "fx": float(fx), "fy": float(fy), "cx": float(cx), "cy": float(cy),
        "reprojection_error": float(mean_err),
        "images_used": used,
        "chessboard": {"cols": args.cols, "rows": args.rows, "square_size_m": args.square_size},
    }
    out_file = out_dir / "camera_intrinsics.json"
    out_file.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")

    print("\n========== 标定结果 ==========")
    print(f"内参矩阵 K:\n{K}")
    print(f"畸变系数 dist: {dist.flatten()}")
    print(f"fx={fx:.1f}  fy={fy:.1f}  cx={cx:.1f}  cy={cy:.1f}")
    print(f"重投影误差: {mean_err:.4f} px  （目标 ~1.0）")
    print(f"使用照片: {used}/{saved}")
    print(f"已保存: {out_file}")


if __name__ == "__main__":
    main()
