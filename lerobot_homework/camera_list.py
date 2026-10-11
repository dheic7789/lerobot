# 枚举本机所有摄像头：逐个尝试打开索引 0..5，打印能否打开与分辨率
import cv2

print("枚举摄像头索引 0..5：")
for i in range(6):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        ok, frame = cap.read()
        print(f"  索引 {i}: 可打开, 分辨率 {w}x{h}, 抓帧 {'成功' if ok else '失败'}")
        cap.release()
    else:
        print(f"  索引 {i}: 无法打开")
        cap.release()
