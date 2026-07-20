"""ROV 任务脚本：前进 5 秒"""
import time
import sys
sys.path.insert(0, '/home/coody/workspace/ArduSub/pymavlink')
from vehicle import ROV

def main():
    rov = ROV()
    rov.connect()

    rov.arm()
    if not rov.armed:
        print("❌ 解锁失败，退出")
        return

    for i in [3,2,1]:
        print(f"{i}...")
        time.sleep(1)

    print(">>> 前进 5 秒 (15%)")
    rov.set_raw(forward=0.15)   # 15% 前进 PWM=1560
    time.sleep(5)

    rov.stop()
    rov.disarm()
    rov.close()
    print("✅ 完成")

if __name__ == '__main__':
    main()
