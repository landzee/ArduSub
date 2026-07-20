import time
import glob
from pymavlink import mavutil

BAUD_RATE = 57600

def find_port():
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    if not ports:
        print("错误：未检测到飞控设备！")
        exit(1)
    return ports[0]

PORT = find_port()
print(f"检测到设备: {PORT}")
m = mavutil.mavlink_connection(PORT, baud=BAUD_RATE)
m.wait_heartbeat()
print("✅ 连接成功\n")

# 切 MANUAL + 解锁
m.set_mode('MANUAL')
time.sleep(0.3)
m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0,0,0,0,0,0)
time.sleep(1)
print("✅ 已解锁\n")

# 逐个测试: DO_SET_SERVO 直接控制伺服输出通道
# Main1-6 对应 SERVO 1-6
for mot in [1,2,3,4]:
    print(f"=== 电机 {mot} ===")

    print(f"  PWM=1650 (正转)")
    m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, mot, 1650, 0,0,0,0,0)
    time.sleep(2)
    m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, mot, 1500, 0,0,0,0,0)
    time.sleep(0.5)

    print(f"  PWM=1350 (反转)")
    m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, mot, 1350, 0,0,0,0,0)
    time.sleep(2)
    m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_DO_SET_SERVO, 0, mot, 1500, 0,0,0,0,0)
    time.sleep(0.5)

# 加锁
m.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0,0,0,0,0,0)
print("\n✅ 测试完成")
print("请报告每个电机的风向:")
for mot in [1,2,3,4]:
    print(f"  电机{mot}: 正转(1650)风往__吹, 反转(1350)风往__吹")
