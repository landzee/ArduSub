"""深度测试（自动校准水面零点）"""
import time, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pymavlink import mavutil

m = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
m.wait_heartbeat(timeout=5)

# 自动校准水面气压
print("校准水面零点...")
press_sum = 0
count = 0
deadline = time.time() + 3
while time.time() < deadline:
    msg = m.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=1)
    if msg:
        press_sum += msg.press_abs
        count += 1
surface_press = press_sum / count if count > 0 else 1013.0
print(f"  水面气压: {surface_press:.1f} hPa")
print(f"  深度 = 当前气压 - {surface_press:.1f} hPa\n")
print("持续读取，Ctrl+C 退出\n")

try:
    while True:
        msg = m.recv_match(type='SCALED_PRESSURE', blocking=True, timeout=1)
        if msg:
            depth = (msg.press_abs - surface_press) * 0.01
            bar = "█" * max(0, int(depth * 10))
            print(f"\r  深度: {depth:+.2f}m  |  气压: {msg.press_abs:.1f} hPa  |  {bar}", end="")
except KeyboardInterrupt:
    print("\n\n已退出")
