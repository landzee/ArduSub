"""摄像头实时显示（支持手动选择设备）"""
import cv2, sys, os


def list_cameras():
    """扫描并列出所有可用摄像头"""
    cams = []
    for d in range(10):
        path = f"/dev/video{d}"
        if os.path.exists(path):
            name = "未知"
            try:
                with open(f"/sys/class/video4linux/video{d}/name") as f:
                    name = f.read().strip()
            except:
                pass
            cams.append((d, name))
    return cams


# 列出可用摄像头
print("可用摄像头:")
cams = list_cameras()
for d, name in cams:
    print(f"  [{d}] {name}")

# 选择设备
if len(sys.argv) > 1:
    DEVICE = int(sys.argv[1])
else:
    sel = input(f"\n选择设备编号 (0-{cams[-1][0] if cams else 0}, 默认=0): ").strip()
    DEVICE = int(sel) if sel else 0

cap = cv2.VideoCapture(DEVICE)
if not cap.isOpened():
    print(f"❌ 无法打开 video{DEVICE}")
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1960)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

print(f"\n✅ 图传已打开 (video{DEVICE})")
print("按 Q 或 ESC 退出")

cv2.namedWindow("ROV 图传", cv2.WINDOW_GUI_NORMAL | cv2.WINDOW_AUTOSIZE)

while True:
    ret, frame = cap.read()
    if not ret:
        continue
    cv2.imshow("ROV 图传", frame)
    key = cv2.waitKey(30)
    if key == ord('q') or key == 27:
        break

cap.release()
cv2.destroyAllWindows()
