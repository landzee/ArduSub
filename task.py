"""高层任务脚本"""
import time
from vehicle import ROV
from config import RovMode


def task_square(rov, side_sec=5, turn_power=0.3):
    """顺时针正方形"""
    print("\n🔲 正方形轨迹")
    for i in range(4):
        print(f"\n--- 边 {i+1}/4 ---")
        rov.forward(sec=side_sec)
        _, _, yaw = rov.get_attitude()
        rov.turn_to(yaw * 57.3 + 90)


def task_dive(rov, depth_m=0.2, hold_sec=10):
    """下潜→定深→上浮"""
    print(f"\n🌊 下潜到 {depth_m}m")
    rov.dive_to(depth_m)

    print(f"\n⏸️ 定深 {hold_sec}s")
    rov.hold_depth(depth_m, duration=hold_sec)

    print(f"\n🌊 上浮")
    rov.surface()


def task_forward_turn(rov):
    """直行→右转90°→保持（调参用）"""
    _, _, yaw = rov.get_attitude()
    target = yaw * 57.3 + 90
    if target > 180: target -= 360
    print(f"\n  起始 yaw={yaw*57.3:.0f}° → 目标 {target:.0f}°")

    rov.turn_to(target)
    rov.hold_yaw(target, sec=6)


def main():
    rov = ROV()
    rov.connect()
    rov.set_mode(RovMode.SURFACE)
    if not rov.arm(): return

    for i in [3,2,1]:
        print(f"  {i}..."); time.sleep(1)
    rov.start_bilge()

    # ---- 选一个任务执行 ----
    # task_square(rov, side_sec=5)
    rov.forward(sec=3,pwm_val=1580)
    task_dive(rov, depth_m=0.2)
    # task_forward_turn(rov)
    # ----
    
    rov.stop()
    rov.disarm()
    rov.close()
    print("\n✅ 完成")

if __name__ == '__main__':
    main()
