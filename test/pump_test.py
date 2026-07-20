"""水泵测试：通过 MAV_CMD_DO_SET_RELAY 控制"""
import time, sys, os
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from pymavlink import mavutil

m = mavutil.mavlink_connection('/dev/ttyUSB0', baud=57600)
m.wait_heartbeat(timeout=5)
print("✅ 连接成功")

m.set_mode('MANUAL')
time.sleep(0.3)

for _ in range(5):
    m.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,1,0,0,0,0,0,0)
    time.sleep(2)
    msg = m.recv_match(type='HEARTBEAT',blocking=True,timeout=3)
    if msg and (msg.base_mode & 128):
        print("✅ 已解锁")
        break
else:
    print("❌ 解锁失败")
    exit(1)

pump_names = ["泵1(进水)", "泵2(排水)", "泵3(进水)", "泵4(排水)"]

try:
    for i, name in enumerate(pump_names):
        relay_num = i + 1
        print(f"\n=== {name} (RELAY{relay_num} ON 5秒) ===")
        m.mav.command_long_send(1, 1,
            mavutil.mavlink.MAV_CMD_DO_SET_RELAY, 0,
            relay_num, 1, 0, 0, 0, 0, 0)
        time.sleep(5)
        m.mav.command_long_send(1, 1,
            mavutil.mavlink.MAV_CMD_DO_SET_RELAY, 0,
            relay_num, 0, 0, 0, 0, 0, 0)
        print(f"关闭")
        time.sleep(2)
finally:
    for i in range(4):
        m.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_DO_SET_RELAY,0,i+1,0,0,0,0,0,0)
    m.mav.command_long_send(1,1,mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,0,0,0,0,0,0,0,0)
    print("\n✅ 测试完成")
