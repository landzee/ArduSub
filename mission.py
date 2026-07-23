"""ROV 控制任务：直行 → 右转90° → 保持"""
import time, sys
sys.path.insert(0, '/home/coody/workspace/ArduSub/pymavlink')
from vehicle import ROV
from pid import PID
from config import RovMode, CONTROL_DT

rov = ROV()

# PD 控制器（kp/kd 在实际使用时动态设置）
pid_yaw = PID(kp=0.04, ki=0.005, kd=0.12, ff=0.08,
               integral_min=-0.3, integral_max=0.3,
               output_min=-1.0, output_max=1.0)


def yaw_to_pwm(y_deg, max_delta=120):
    """将当前 yaw 喂给 PID，返回 PWM（max_delta 越大刹车越猛）"""
    output = pid_yaw.update(y_deg, CONTROL_DT)
    sign = 1 if output > 0 else -1
    mag = max(1, min(abs(output), 1.0))
    return int(1500 + sign * mag * max_delta)


def stop():
    pid_yaw.reset()
    rov.mav.mav.rc_channels_override_send(1,1,1500,1500,1500,1500,1500,1500,1500,1500)
    time.sleep(0.3)


def forward(sec=3, pwm=1580):
    print(f"  直行 {sec}s")
    deadline = time.time() + sec
    while time.time() < deadline:
        rov.mav.mav.rc_channels_override_send(1,1,1500,1500,1500,1500,pwm,1500,1500,1500)
        time.sleep(0.05)


def norm_yaw(y_deg, target):
    """把 y_deg 归一化到 [target-180, target+180]"""
    while y_deg < target - 180: y_deg += 360
    while y_deg > target + 180: y_deg -= 360
    return y_deg

def turn_to(target_yaw, timeout=15):
    """分级 PWM 右转：全速 → 减速 → 刹车 → 停"""
    pid_yaw.reset()
    pid_yaw.set_setpoint(target_yaw)
    print(f"  右转到 yaw={target_yaw:.0f}°")
    deadline = time.time() + timeout
    last_yaw = None
    while time.time() < deadline:
        _, _, y_raw = rov.get_attitude()
        y_deg = norm_yaw(y_raw * 57.2958, target_yaw)
        err = target_yaw - y_deg
        # 计算角速度 (deg/s)
        rate = 0
        if last_yaw is not None:
            rate = abs(y_deg - last_yaw) / CONTROL_DT
        last_yaw = y_deg
        print(f"\r    当前={y_deg:.0f}° 误差={err:.0f}° 角速度={rate:.0f}°/s  ", end="")
        if abs(err) < 3 and rate < 10:
            break
        # 分级 PWM：误差大→全速，误差中→减速，误差小→刹车或微调
        if abs(err) > 40:
            pwm = 1580             # 全速右转
        elif abs(err) > 15:
            pwm = 1540             # 减速
        elif abs(err) > 5:
            pwm = 1510 if rate < 20 else 1480  # 刹车优先
        else:
            pwm = 1505 if abs(err) < 1 else (1510 if err > 0 else 1490)
        sign = 1 if err > 0 else -1
        ch4 = int(1500 + sign * (pwm - 1500))
        rov.mav.mav.rc_channels_override_send(1,1,1500,1500,1500,ch4,1500,1500,1500,1500)
        time.sleep(CONTROL_DT)
    else:
        print(" 超时")
    stop()
    print(f"   ✓")


def hold_yaw(target_yaw, sec=6):
    """航向保持：分级纠正，避免过冲"""
    last_yaw = None
    print(f"  保持 yaw={target_yaw:.0f}° {sec}s")
    deadline = time.time() + sec
    err_sign = 0
    while time.time() < deadline:
        _, _, y_raw = rov.get_attitude()
        y_deg = norm_yaw(y_raw * 57.2958, target_yaw)
        err = target_yaw - y_deg
        rate = 0
        if last_yaw:
            rate = (y_deg - last_yaw) / CONTROL_DT
        last_yaw = y_deg
        print(f"\r    保持: yaw={y_deg:.0f}° 误差={err:.0f}° 角速度={rate:.0f}°/s ", end="")
        if abs(err) < 3 and abs(rate) < 10:
            time.sleep(CONTROL_DT)
            continue
        # 分级纠正（误差>5°优先刹车，不对称靠岸更用力）
        if abs(err) > 15:
            delta = 60                     # 大偏差，使劲纠
        elif abs(err) > 8:
            delta = 40                     # 中等纠正
        elif abs(err) > 3:
            delta = 25
        else:
            delta = 15
        # 过冲时（朝反方向走）：额外加力刹车
        if (err > 0 and rate < -5) or (err < 0 and rate > 5):
            delta = min(delta + 20, 80)    # 正在远离目标，加强
        sign = 1 if err > 0 else -1
        ch4 = int(1500 + sign * delta)
        ch4 = max(1350, min(1650, ch4))    # 安全限幅
        rov.mav.mav.rc_channels_override_send(1,1,1500,1500,1500,ch4,1500,1500,1500,1500)
        time.sleep(CONTROL_DT)
    stop()
    print(f"   ✓")


def main():
    rov.connect()
    rov.set_mode(RovMode.SURFACE)
    if not rov.arm():
        return

    for i in [3,2,1]:
        print(f"  {i}...")
        time.sleep(1)

    print("\n--- 直行 3s ---")
    forward(sec=3)
    stop()

    _, _, start_yaw = rov.get_attitude()
    target = start_yaw * 57.2958 + 90
    if target > 180: target -= 360
    print(f"\n  起始 yaw={start_yaw*57.3:.0f}° → 目标 {target:.0f}°")

    print("\n--- 右转 90° (PD) ---")
    pid_yaw.kp, pid_yaw.ki, pid_yaw.kd, pid_yaw.ff = 0.04, 0.005, 0.12, 0.08
    turn_to(target)

    print(f"\n--- 保持 {target:.0f}° 3s (PD) ---")
    hold_yaw(target, sec=6)

    stop()
    rov.disarm()
    rov.close()
    print("\n✅ 完成")

if __name__ == '__main__':
    main()
