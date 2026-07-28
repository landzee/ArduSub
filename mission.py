"""ROV 任务：直行 → 右转90° → 保持 (比例控制)"""
import time, sys, os
sys.path.insert(0, '/home/coody/workspace/ArduSub/pymavlink')
from vehicle import ROV
from pid import PID

# 日志文件（在 main 中创建，这里先声明）
logfile = None
t0 = 0.0
from config import RovMode, CONTROL_DT

rov = ROV()

# PD 控制器（默认值，实际 kp/kd 在 turn/hold 前覆盖）
pid_yaw = PID(kp=0.03, kd=0.05, output_min=-1.0, output_max=1.0)


def pwm(pid, measured, setpoint, max_delta, velocity=None):
    """PID 接收实测值 + EKF 角速度做 D 阻尼"""
    pid.set_setpoint(setpoint)
    return int(1500 + pid.update(measured, CONTROL_DT, velocity=velocity) * max_delta)
 

def norm_yaw(y_deg, target):
    """归一化 yaw 到 target±180"""
    while y_deg < target - 180: y_deg += 360
    while y_deg > target + 180: y_deg -= 360
    return y_deg


def stop():
    pid_yaw.reset()
    rov.mav.mav.rc_channels_override_send(1,1,1500,1500,1500,1500,1500,1500,1500,1500)
    time.sleep(0.3)


def forward(sec=3, pwm_val=1580):
    print(f"  直行 {sec}s")
    deadline = time.time() + sec
    while time.time() < deadline:
        rov.mav.mav.rc_channels_override_send(1,1,1500,1500,1500,1500,pwm_val,1500,1500,1500)
        time.sleep(0.05)


def turn_to(target_yaw, timeout=15):
    """P 比例右转"""
    pid_yaw.reset()
    pid_yaw.set_setpoint(target_yaw)
    print(f"  右转到 yaw={target_yaw:.0f}°")
    deadline = time.time() + timeout
    while time.time() < deadline:
        _, _, y_raw = rov.get_attitude()
        y_deg = norm_yaw(y_raw * 57.3, target_yaw)
        err = target_yaw - y_deg
        print(f"\r    当前={y_deg:.0f}° 误差={err:.0f}°  ", end="")
        logfile.write(f"turn,{time.time()-t0:.2f},{y_deg:.1f},{err:.0f}\n")
        if abs(err) < 3:
            break
        yaw_rate = rov.get_sensor('yawspeed')
        ch4 = pwm(pid_yaw, measured=y_deg, setpoint=target_yaw, max_delta=80, velocity=yaw_rate)
        ch4 = max(1400, min(1600, ch4))
        rov.mav.mav.rc_channels_override_send(1,1,1500,1500,1500,ch4,1500,1500,1500,1500)
        time.sleep(CONTROL_DT)
    else:
        print(" 超时")
    stop()
    print(f"   ✓")


def hold_yaw(target_yaw, sec=6):
    """P 比例航向保持"""
    pid_yaw.reset()
    pid_yaw.set_setpoint(target_yaw)
    pid_yaw.kp = 0.01
    pid_yaw.ki = 0.0
    pid_yaw.kd = 0.03
    print(f"  保持 yaw={target_yaw:.0f}° {sec}s")
    deadline = time.time() + sec
    while time.time() < deadline:
        _, _, y_raw = rov.get_attitude()
        y_deg = norm_yaw(y_raw * 57.3, target_yaw)
        err = target_yaw - y_deg
        print(f"\r    保持: yaw={y_deg:.0f}° 误差={err:.0f}°  ", end="")
        logfile.write(f"hold,{time.time()-t0:.2f},{y_deg:.1f},{err:.0f}\n")
        if abs(err) < 3:
            time.sleep(CONTROL_DT)
            continue
        yaw_rate = rov.get_sensor('yawspeed')
        ch4 = pwm(pid_yaw, measured=y_deg, setpoint=target_yaw, max_delta=80, velocity=yaw_rate)
        ch4 = max(1380, min(1620, ch4))
        rov.mav.mav.rc_channels_override_send(1,1,1500,1500,1500,ch4,1500,1500,1500,1500)
        time.sleep(CONTROL_DT)
    stop()
    print(f"   ✓")


def main():
    global logfile, t0
    os.makedirs("log", exist_ok=True)
    logfile = open(f"log/{time.strftime('%m%d_%H%M%S')}.txt", "w")
    logfile.write("phase,time,yaw_deg,error_deg\n")
    t0 = time.time()

    rov.connect()
    rov.set_mode(RovMode.SURFACE)
    if not rov.arm():
        return
    for i in [3,2,1]:
        print(f"  {i}..."); time.sleep(1)

    try:
        print("\n--- 直行 ---")
        forward(sec=0)
        stop()

        _, _, start_yaw = rov.get_attitude()
        target = start_yaw * 57.3 + 90
        if target > 180: target -= 360
        print(f"\n  起始 yaw={start_yaw*57.3:.0f}° → 目标 {target:.0f}°")

        print("\n--- 右转 90° ---")
        pid_yaw.kp = 0.01
        pid_yaw.kd = 0.03
        turn_to(target)

        print(f"\n--- 保持 {target:.0f}° 10s ---")
        hold_yaw(target, sec=20)

        stop()
        rov.disarm()
        rov.close()
    except KeyboardInterrupt:
        print("\n⚠️ 中断")
    finally:
        if logfile: logfile.close()
        print("\n✅ 完成")

if __name__ == '__main__':
    main()
