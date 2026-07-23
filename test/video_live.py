"""摄像头实时显示（无多余按钮）"""
import cv2, sys

DEVICE = int(sys.argv[1]) if len(sys.argv) > 1 else 0

cap = cv2.VideoCapture(DEVICE)
if not cap.isOpened():
    print(f"❌ 无法打开 video{DEVICE}")
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print(f"✅ 图传已打开 (video{DEVICE})")
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
