#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""相机内参标定（课堂练习 1/4）：OpenCV 棋盘格标定。

用法：
  python camera_calibration.py --camera-id 1          # 拍照 + 标定
  python camera_calibration.py --recalibrate          # 用已存照片重新标定（无需重拍）

操作：
  - 移动棋盘格纸，使其角点被绿框标出；按 空格 保存当前帧
    （每张必须改变棋盘格相对相机的位姿，建议 ≥15 张）
  - 按 c    用已保存照片标定，打印内参 + 重投影误差，保存 camera_intrinsics.json
  - 按 d    保存当前帧到 images/debug_xxx.png（检测失败时用于诊断）
  - 按 q    退出

自动探测常见棋盘格规格：9x6 / 8x6 / 7x5 / 10x7 / 6x4。
"""
import argparse
import json
from pathlib import Path

try:
    import cv2  # type: ignore[import-not-found]
except ModuleNotFoundError as exc:
    raise SystemExit(
        "OpenCV is required for camera calibration. Install it with: pip install opencv-python"
    ) from exc

import numpy as np

PATTERNS = [(9, 6), (8, 6), (7, 5), (10, 7), (6, 4)]


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--camera-id", type=int, default=1, help="摄像头编号（默认 1=机器人相机）")
    p.add_argument("--cols", type=int, default=7, help="棋盘格内角点列数（默认 9）")
    p.add_argument("--rows", type=int, default=5, help="棋盘格内角点行数（默认 6）")
    p.add_argument("--square-size", type=float, default=0.025, help="棋盘格边长，米（默认 0.025）")
    p.add_argument("--out-dir", type=Path, default=Path("lerobot_homework/calib_data"),
                   help="照片与结果保存目录")
    p.add_argument("--recalibrate", action="store_true",
                   help="跳过拍照，用 images/ 已存照片重新标定")
    return p.parse_args()


def try_detect(gray, patterns):
    """依次尝试各规格，返回 (pattern, corners) 或 (None, None)。"""
    for p in patterns:
        ret, corners = cv2.findChessboardCorners(gray, p, None)
        if ret:
            return p, corners
    return None, None


def capture_loop(args, imgs_dir, criteria):
    """拍照采集循环，返回 (saved, pattern)。"""
    cap = cv2.VideoCapture(args.camera_id)
    if not cap.isOpened():
        raise SystemExit(f"无法打开摄像头 #{args.camera_id}")
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    saved = 0
    pattern = None          # 自动探测到的规格
    msg, msg_color = "Move board into view (green box = detected)", (255, 255, 0)
    print("移动棋盘格，绿框=检测到角点。空格=保存  c=标定  d=存诊断帧  q=退出")
    while True:
        ok, frame = cap.read()
        if not ok:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        pat, corners = try_detect(gray, PATTERNS)
        if pat is not None and pattern is None:
            pattern = pat
            print(f"[info] 检测到棋盘格规格: {pat[0]}x{pat[1]}")
        if pattern is None:
            ret = False
        else:
            ret, corners = cv2.findChessboardCorners(gray, pattern, None)
        disp = frame.copy()
        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            cv2.drawChessboardCorners(disp, pattern, corners2, ret)
        pat_txt = f"pattern={pattern[0]}x{pattern[1]}" if pattern else "pattern=?"
        cv2.putText(disp, f"saved={saved}  {pat_txt}  [SPACE] save  [C] calibrate  [Q] quit",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
        cv2.putText(disp, msg, (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, msg_color, 2)
        cv2.imshow("calibration", disp)

        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("d"):
            dbg = imgs_dir / f"debug_{saved:03d}.png"
            cv2.imwrite(str(dbg), frame)
            print(f"[debug] 已保存当前帧: {dbg}")
        if key == ord(" ") and ret:
            cv2.imwrite(str(imgs_dir / f"img_{saved:03d}.png"), frame)
            saved += 1
            msg, msg_color = f"SAVED {saved}/15, keep moving the board", (0, 255, 0)
            print(f"[save] 第 {saved} 张: {imgs_dir / f'img_{saved - 1:03d}.png'}")
        elif key == ord(" ") and not ret:
            msg, msg_color = "NO BOARD: move chessboard fully into view", (0, 0, 255)
            print("[skip] 未检测到完整棋盘格")
        if key == ord("c"):
            if saved < 5:
                msg, msg_color = f"ONLY {saved} saved (need >=5): keep pressing SPACE", (0, 0, 255)
                print(f"[warn] 已保存 {saved} 张，至少 5 张（建议 ≥15），继续拍")
                continue
            break

    cap.release()
    cv2.destroyAllWindows()
    return saved, pattern


def calibrate(imgs_dir, pattern, square_size):
    """用 images/ 下已存照片标定，返回结果 dict。"""
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2) * square_size
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    obj_points, img_points, used_names = [], [], []
    files = sorted(imgs_dir.glob("img_*.png"))
    for f in files:
        img = cv2.imread(str(f))
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ret, corners = cv2.findChessboardCorners(gray, pattern, None)
        if ret:
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            obj_points.append(objp)
            img_points.append(corners2)
            used_names.append(f.name)
        else:
            print(f"[warn] {f.name} 未检测到角点，跳过")
    used = len(obj_points)
    if used < 5:
        raise SystemExit(f"可用照片不足（{used} 张）")

    ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

    # 重投影误差（统一 reshape 避免 OpenCV 5 norm 类型不匹配）
    total_err = 0.0
    for i in range(used):
        proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], K, dist)
        a = img_points[i].reshape(-1, 2).astype(np.float32)
        b = proj.reshape(-1, 2).astype(np.float32)
        err = float(np.linalg.norm(a - b)) / len(b)
        total_err += err
    mean_err = total_err / used

    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    return {
        "K": K.tolist(),
        "dist": dist.flatten().tolist(),
        "fx": float(fx), "fy": float(fy), "cx": float(cx), "cy": float(cy),
        "reprojection_error": float(mean_err),
        "images_used": used,
        "images_total": len(files),
        "chessboard": {"cols": pattern[0], "rows": pattern[1], "square_size_m": square_size},
    }


def main() -> None:
    args = parse_args()
    out_dir = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)
    imgs_dir = out_dir / "images"
    imgs_dir.mkdir(exist_ok=True)

    if args.recalibrate:
        files = sorted(imgs_dir.glob("img_*.png"))
        if not files:
            raise SystemExit("没有已保存的照片，请先正常采集")
        probe = cv2.imread(str(files[0]))
        pg = cv2.cvtColor(probe, cv2.COLOR_BGR2GRAY)
        pattern = None
        for p in PATTERNS:
            r, _ = cv2.findChessboardCorners(pg, p, None)
            if r:
                pattern = p
                break
        if pattern is None:
            raise SystemExit("无法从已存照片识别棋盘格规格，请用 --cols/--rows 指定")
        print(f"[info] 已存照片 {len(files)} 张，规格 {pattern[0]}x{pattern[1]}，直接标定")
    else:
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        saved, pattern = capture_loop(args, imgs_dir, criteria)
        if pattern is None:
            raise SystemExit("未识别到任何棋盘格规格，请按 d 保存一帧发我诊断，或确认打印规格")
        if saved < 5:
            raise SystemExit(f"照片太少（{saved} 张），至少需要 5 张（建议 ≥15 张）")

    result = calibrate(imgs_dir, pattern, args.square_size)
    out_file = out_dir / "camera_intrinsics.json"
    out_file.write_text(json.dumps(result, indent=2, ensure_ascii=False), encoding="utf-8")

    print("\n========== 标定结果 ==========")
    print(f"内参矩阵 K:\n{np.array(result['K'])}")
    print(f"畸变系数 dist: {result['dist']}")
    print(f"fx={result['fx']:.1f}  fy={result['fy']:.1f}  cx={result['cx']:.1f}  cy={result['cy']:.1f}")
    print(f"重投影误差: {result['reprojection_error']:.4f} px  （目标 ~1.0）")
    print(f"使用照片: {result['images_used']}/{result['images_total']}")
    print(f"已保存: {out_file}")


if __name__ == "__main__":
    main()
