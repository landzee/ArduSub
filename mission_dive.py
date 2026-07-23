"""ROV 下潜任务：定深 0.4m（含水泵协同）"""
import time, sys
sys.path.insert(0, '/home/coody/workspace/ArduSub/pymavlink')
from vehicle import ROV
from config import RovMode, PUMP_DRAIN_DEPTH, PUMP_INLET_OFF_DEPTH

rov = ROV()
TGT = 0.4
inlets_on = False
drains_on = False

def stop():
    rov.mav.mav.rc_channels_override_send(1,1,1500,1500,1500,1500,1500,1500,1500,1500)
    time.sleep(0.2)

def hold_depth(target_m, duration=30):
    global inlets_on, drains_on
    # 记录起始航向作为保持目标
    _, _, yaw0_rad = rov.get_attitude()
    target_yaw = yaw0_rad * 57.3
    print(f"  定深 {target_m}m + 姿态稳定 (yaw目标={target_yaw:.0f}°) ({duration}s)")
    deadline = time.time() + duration
    while time.time() < deadline:
        depth = rov.get_depth()
        roll_rad, pitch_rad, yaw_rad = rov.get_attitude()
        roll_deg, pitch_deg, yaw_deg = roll_rad * 57.3, pitch_rad * 57.3, yaw_rad * 57.3
        d_err = target_m - depth
        r_err = -roll_deg
        p_err = -pitch_deg
        y_err = target_yaw - yaw_deg
        while y_err > 180: y_err -= 360
        while y_err < -180: y_err += 360

        print(f"\r  深={depth:.2f}m err={d_err:+.2f} | roll={roll_deg:.0f}° pitch={pitch_deg:.0f}° yaw={yaw_deg:.0f}°   ", end="")

        # --- 深度 PWM ---
        if abs(d_err) < 0.03: ch3 = 1500
        else:
            s = 1 if d_err > 0 else -1
            dd = 80 if abs(d_err)>0.15 else (50 if abs(d_err)>0.08 else (30 if abs(d_err)>0.03 else 20))
            ch3 = int(1500 - s * dd)

        # --- 横滚 (ch2) ---
        if abs(r_err) < 3: ch2 = 1500
        else:
            s = 1 if r_err > 0 else -1
            dr = 100 if abs(r_err)>8 else 60
            ch2 = int(1500 + s * dr)

        # --- 俯仰 (ch1) ---
        if abs(p_err) < 3: ch1 = 1500
        else:
            s = 1 if p_err > 0 else -1
            dp = 150 if abs(p_err)>10 else (100 if abs(p_err)>5 else 60)
            ch1 = int(1500 + s * dp)

        # --- 偏航 (ch4) ---
        if abs(y_err) < 3: ch4 = 1500
        else:
            s = 1 if y_err > 0 else -1
            dy = 80 if abs(y_err)>20 else (50 if abs(y_err)>10 else 30)
            ch4 = int(1500 + s * dy)

        # --- 水泵 ---
        if depth > PUMP_INLET_OFF_DEPTH and inlets_on:
            rov.pump_inlets(False); inlets_on = False
        if depth < PUMP_DRAIN_DEPTH and target_m < depth and not drains_on:
            rov.pump_drains(True); drains_on = True
        if depth < 0.02 and drains_on:
            rov.pump_drains(False); drains_on = False

        rov.mav.mav.rc_channels_override_send(1,1,ch1,ch2,ch3,ch4,1500,1500,1500,1500)
        time.sleep(0.05)
    stop()

def main():
    rov.connect()
    print(f"✅ 深度零点: {rov.get_depth():.2f}m")
    rov.set_mode(RovMode.UNDERWATER)
    if not rov.arm(): return
    for i in [3,2,1]:
        print(f"  {i}..."); time.sleep(1)

    try:
        global inlets_on
        rov.pump_inlets(True)
        inlets_on = True

        print(f"\n--- 下潜到 {TGT}m ---")
        hold_depth(TGT)

        print(f"\n--- 上浮 ---")
        hold_depth(0.0, duration=20)

    except KeyboardInterrupt:
        print("\n⚠️ 用户中断")

    finally:
        rov.pumps_all_off()
        stop()
        rov.disarm()
        rov.close()
        print("\n✅ 水泵已关，电机已停")

if __name__ == '__main__':
    main()
