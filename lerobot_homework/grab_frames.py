# 从索引 0/1/2 各抓一帧保存，用于确认哪个是机器人摄像头
import cv2

for i in range(3):
    cap = cv2.VideoCapture(i)
    if not cap.isOpened():
        print(f"索引 {i}: 无法打开")
        continue
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    ok = False
    frame = None
    for _ in range(10):  # 等摄像头稳定
        ok, frame = cap.read()
        if ok and frame is not None:
            break
    if ok and frame is not None:
        out = rf"D:\lerobot_workspace\lerobot_homework\cam_{i}.jpg"
        cv2.imwrite(out, frame)
        print(f"索引 {i}: 已保存 {out} ({frame.shape[1]}x{frame.shape[0]})")
    else:
        print(f"索引 {i}: 抓帧失败")
    cap.release()
