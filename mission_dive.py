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
    print(f"  定深 {target_m}m + 稳俯仰 ({duration}s)")
    deadline = time.time() + duration
    while time.time() < deadline:
        depth = rov.get_depth()
        _, pitch_rad, _ = rov.get_attitude()
        pitch_deg = pitch_rad * 57.3
        d_err = target_m - depth
        p_err = -pitch_deg   # 翘头拍正，埋头拍负

        print(f"\r  深={depth:.2f}m err={d_err:+.2f}  仰={pitch_deg:.0f}° err={p_err:+.0f}°   ", end="")

        # --- 深度 PWM ---
        if abs(d_err) < 0.03:
            ch3 = 1500
        else:
            sign = 1 if d_err > 0 else -1
            if abs(d_err) > 0.15: dd = 80
            elif abs(d_err) > 0.08: dd = 50
            elif abs(d_err) > 0.03: dd = 30
            else: dd = 20
            ch3 = int(1500 - sign * dd)

        # --- 俯仰稳定 PWM (ch1) ---
        if abs(p_err) < 3:
            ch1 = 1500
        else:
            sign = 1 if p_err > 0 else -1
            if abs(p_err) > 10: dp = 150
            elif abs(p_err) > 5: dp = 100
            else: dp = 60
            ch1 = int(1500 + sign * dp)

        # --- 水泵 ---
        if depth > PUMP_INLET_OFF_DEPTH and inlets_on:
            rov.pump_inlets(False); inlets_on = False
        if depth < PUMP_DRAIN_DEPTH and target_m < depth and not drains_on:
            rov.pump_drains(True); drains_on = True
        if depth < 0.02 and drains_on:
            rov.pump_drains(False); drains_on = False

        rov.mav.mav.rc_channels_override_send(1,1,ch1,1500,ch3,1500,1500,1500,1500,1500)
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
