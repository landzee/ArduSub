import time
import glob
from pymavlink import mavutil

# --- 1. 自动检测端口与连接 ---
BAUD_RATE = 57600

def find_port():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    if not ports:
        print("错误：未检测到飞控设备！")
        exit(1)
    port = ports[0]
    print(f"检测到设备: {port}")
    return port

PORT = find_port()
print(f"正在尝试打开端口 {PORT}...")
master = mavutil.mavlink_connection(PORT, baud=BAUD_RATE)

print("正在等待飞控心跳包 (Heartbeat)...")
master.wait_heartbeat()
print("✅ 连接成功！\n")

# --- 2. 读取当前参数 ---
print("--- 当前电机方向参数 ---")
for k in ['MOT_1_DIRECTION','MOT_2_DIRECTION','MOT_3_DIRECTION','MOT_4_DIRECTION']:
    master.mav.param_request_read_send(master.target_system, master.target_component, k.encode(), -1)
    time.sleep(0.2)
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
    if msg:
        d = "反转" if msg.param_value < 0 else "正转"
        print(f"  {msg.param_id} = {msg.param_value:.0f} ({d})")

# --- 3. 切换 MANUAL 模式 ---
print("\n切换 MANUAL 模式...")
master.set_mode('MANUAL')
time.sleep(0.3)

# --- 4. 解锁 ---
print("发送解锁指令...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0)
time.sleep(1)

# 确认解锁
heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
if not heartbeat or not (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
    print("❌ 解锁失败")
    exit(1)
print("✅ 已解锁\n")

try:
    # --- 5. 前进测试 ---
    print("=== 前进方向测试 (ch5=forward, 1650PWM) ===")
    for i in [5,4,3,2,1]:
        print(f"  {i}...")
        time.sleep(1)

    print(">>> 前进 5 秒")
    deadline = time.time() + 5
    count = 0
    while time.time() < deadline:
        # ch1=pitch ch2=roll ch3=throttle ch4=yaw ch5=forward ch6=lateral
        master.mav.rc_channels_override_send(
            master.target_system, master.target_component,
            1500, 1500, 1500, 1500,  # pitch, roll, throttle, yaw 中立
            1650,  # forward 前进
            1500, 1500, 1500)        # lateral, ch7, ch8 中立
        count += 1
        time.sleep(0.05)
    print(f"  已发送 {count} 条 override 指令")

    # --- 6. 停止 ---
    print(">>> 停止")
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500)

    # --- 7. 加锁 ---
    print("\n加锁电机...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(1)
    print("✅ 测试完成")

except KeyboardInterrupt:
    print("\n中断，紧急加锁...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 0, 0, 0, 0, 0, 0, 0)
    master.mav.rc_channels_override_send(
        master.target_system, master.target_component,
        1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500)
    print("已加锁")
