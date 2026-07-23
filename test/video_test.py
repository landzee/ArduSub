"""图传/摄像头测试"""
import cv2, time, sys

DEVICE = int(sys.argv[1]) if len(sys.argv) > 1 else 0
print(f"打开 /dev/video{DEVICE} ...")

cap = cv2.VideoCapture(DEVICE)
if not cap.isOpened():
    print(f"❌ 无法打开 video{DEVICE}")
    print("用法: python3 video_test.py [设备号]")
    print(f"  0=笔记本摄像头, 2=图传接收器(插上后)")
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
print("✅ 已打开，采集 3 秒...")

deadline = time.time() + 3
count = 0
while time.time() < deadline:
    ret, frame = cap.read()
    if ret:
        count += 1
        h, w = frame.shape[:2]
        print(f"\r  帧数: {count}  分辨率: {w}x{h}", end="")
    time.sleep(0.03)

# 保存截图
_, frame = cap.read()
path = f"/tmp/video{DEVICE}_{time.strftime('%H%M%S')}.jpg"
cv2.imwrite(path, frame)
print(f"\n📸 截图: {path} ({frame.shape[1]}x{frame.shape[0]})")
cap.release()
