# ROV 水下航行器 — 工作记录

> 最后更新: 2026-07-20

---

## 1. 硬件

### 推进器布局 (Vectored, 全对称)

```
        船头
    [3]     [4]   ← 上方水平 (top)
    [1]     [2]   ← 下方水平 (bottom)
垂直: [5] [6]      ← 升沉
水泵: AUX 1-4      ← 浮力 (2进2排)
```

### 电机方向 (QGC 已验证)
| 电机 | DIRECTION | 物理表现 |
|------|-----------|---------|
| 1 | -1 (反转) | ✅ 往后吹 |
| 2 | -1 (反转) | ✅ 往后吹 |
| 3 | 1 (正转) | ✅ 往后吹 |
| 4 | -1 (反转) | ✅ 往后吹 |

### 通信
- 电台: `/dev/ttyUSB0`, 57600bps
- 烧录: `/dev/ttyACM0` (USB直连)

---

## 2. 项目路径

| 用途 | 路径 |
|------|------|
| **原版固件 (保留不动)** | `/home/coody/workspace/ArduSub/ardupilot/` |
| **改版固件 (工作用)** | `/home/coody/workspace/ArduSub/ardupilot_sub_control/` |
| **Python 脚本** | `/home/coody/workspace/ArduSub/pymavlink/` |
| 工作记录 | `pymavlink/ROV_工作记录.md` |
| 连接/解锁脚本 | `pymavlink/sub_control.py` |
| 电机测试脚本 | `pymavlink/motor_test.py` |

### 编译烧录
```bash
cd /home/coody/workspace/ArduSub/ardupilot_sub_control
./waf configure --board Pixhawk1
./waf sub
./waf --upload sub   # USB直连飞控时
```

---

## 3. 固件改造清单

### #1 MOT_SURFACE_MODE ✅ 已完成
- 新增参数 `MOT_SURFACE_MODE` (index=14, 0=水下, 1=水面)
- =1 时 MOT_3/4 强制 PWM=1500 停转
- 改动了 3 处代码 (h:69, cpp:128, cpp:291)

### #2 MOT_INVERTED_MODE ⏳ 待实现
- 翻转 180° 后自动适配电机的 throttle/lateral 反号

### #3 VECTORED 混控矩阵 ⏳ 需后续验证各轴
- 前进方向已通过 ✅
- 偏航、俯仰、横滚方向待下水后测试

---

## 4. 控制方式验证结果

### ❌ 失败的
| 方法 | 问题 |
|------|------|
| `MAV_CMD_DO_MOTOR_TEST` | 飞控不响应 |
| `MAV_CMD_DO_SET_SERVO` | 被飞控输出冲掉 |
| 改 `MOT_2/4_DIRECTION=1` | 只改参数不编译固件，风向仍反 |
| 改 `forward_factor=-1` 烧录 | 改变了混控逻辑但参数没配对，风向反 |

### 各轴测试结果

| 测试 | 命令 | 观察 | 结论 |
|------|------|------|------|
| 前进 | ch5=1650 | 1、3、4 往后吹，2 不动 | MOT_2 可能有问题 |
| 前进 (QGC参数) | ch5=1650 | 全部往后吹 ✅ | 原始参数正确 |
| 下潜 | ch3=1650 | 5、6 往下吹 | 映射：>1500=上浮, <1500=下潜 |
| 上浮 | ch3=1350 | 5、6 往上吹 | ✅ |
| 右倾 | ch2=1650 | 5、6 都往下吹 | 待确认 roll 正反 |
| 左倾 | ch2=1350 | 5、6 都往下吹 | 待确认 |

### ✅ 成功的
**`RC_CHANNELS_OVERRIDE` + QGC 原始参数 + 原版固件混控矩阵**

```python
# 解锁 → 切 MANUAL → 发 RC override
m.mav.rc_channels_override_send(1, 1,
    1500, 1500, 1500, 1500,  # ch1-4: pitch/roll/throttle/yaw 中立
    1650,                     # ch5: forward (前进)
    1500, 1500, 1500)         # ch6-8: lateral/aux 中立
```

关键教训：**`master.target_system` 可能是 0，必须显式用 `1`**！

### 通道映射 (MANUAL 模式)
| ch | 功能 | 主要用到的电机 |
|----|------|--------------|
| 1 | pitch (俯仰) | 1,2,3,4 上下差速 | ✅ >1500=上抬, <1500=下埋 (RC1_REVERSED=1) |
| 2 | roll (横滚) | **5,6 垂直差速** | ✅ 5↓6↑=右倾, 5↑6↓=左倾 |
| 3 | throttle (升沉) | **5,6 垂直同向** | ✅ >1500=上升(↓风), <1500=下潜(↑风) |
| 4 | yaw (偏航) | 1,2,3,4 左右差速 | ✅ >1500=右转(1,3后吹,2,4前吹) |
| **5** | **forward (前进)** | **1,2,3,4 同向** | ✅ >1500=前进(全往后吹) |
| 6 | lateral (横移) | 船无横移能力 |

---

## 5. 自动化控制方案

**直接用 RC_CHANNELS_OVERRIDE，不需要切 GUIDED 模式，不需要改固件。** 跟 QGC 手柄走完全一样的路径。

```
Python → rc_channels_override → 飞控混控矩阵 → 推进器 PWM
         (跟 QGC 手柄完全相同的数据流)
```

### 脚本框架
```python
# 前进
m.mav.rc_channels_override_send(1, 1,
    1500, 1500, 1500, 1500,   # pitch, roll, throttle, yaw
    forward_pwm,              # 1650=前进, 1350=后退
    1500, 1500, 1500)         # lateral 和 aux

# 转向: 改 ch4(yaw), 结合前进
# 俯仰: 改 ch1(pitch)
# 升沉: 改 ch3(throttle)
```

### vs GUIDED 模式
- RC override: 简单直连，无超时，跟手柄一样
- GUIDED: 需要切模式，有 3 秒超时，需要持续发指令

---

## 6. 后续工作

### 立即
- [ ] 俯仰方向测试 (ch1) — 确认 pitch 电机响应
- [ ] 偏航方向测试 (ch4) — 确认左右转向
- [ ] 升沉方向测试 (ch3) — 确认垂直推进器

### 下水前
- [ ] 读全部参数备份
- [ ] 密封/防水
- [x] 烧录 MOT_SURFACE_MODE 固件

### 下水后
- [x] 通道验证（forward/yaw/roll/throttle/pitch 全部确认）
- [x] 深度零点校准（启动自动记录水面气压）
- [x] 水泵测试（RELAY 0/1/2/3, 0-based, 1,3进水 2,4排水）
- [x] 正方形轨迹（分级PWM控制, 效果见下）
- [ ] STABILIZE 稳姿态  
- [ ] ALT_HOLD 定深
- [ ] 图传接入

### 转向控制关键发现
- **PID 不适合水下惯性系统**: D项在低速离散IMU下输出爆炸, 惯性+延迟导致过冲无法刹车
- **分级PWM 成功**: 误差>40°→1580, >15°→1540, >5°→1510/刹车, <3°+低速→停
- 上次测试: turn_to 误差-1°, hold_yaw ±5°内振荡, 反向纠正生效
- 代码: mission.py 使用 rov.mav.mav.rc_channels_override_send 直接发送(不经过set_raw的锁竞争)
- 水泵: MAV_CMD_DO_SET_RELAY 0-based编号
