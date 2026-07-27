# ROV 控制脚本技术文档

## 架构概览

```
pymavlink/
  config.py      配置常量（通道/PWM/PID/水泵阈值）
  pid.py         PID 控制器
  vehicle.py     ROV 类（MAVLink 连接、传感器、水泵、翻转检测）
  mission.py     水面转弯任务
  mission_dive.py 下潜定深任务
```

---

## 1. 控制原理

```
Python 脚本 → RC_CHANNELS_OVERRIDE 消息 → 飞控混控矩阵 → 推进器 PWM
             ↑ 跟 QGC 手柄完全相同的数据流
```

0 行飞控代码改动。Python 通过 MAVLink 发送 8 通道 RC override，飞控的 VECTORED 混控矩阵自动将通道值分配给 6 个推进器。

## 2. RC 通道映射

| ch | 功能 | >1500 | <1500 | 电机 |
|----|------|-------|-------|------|
| 1 | pitch | 上抬 | 下埋 | 1-4 上下差速 |
| 2 | roll | 右倾 | 左倾 | 5-6 差速 |
| 3 | throttle | 上升 | 下潜 | 5-6 同向 |
| 4 | yaw | 右转 | 左转 | 1-4 左右差速 |
| 5 | forward | 前进 | 后退 | 1-4 同向 |

> 注意: RC1_REVERSED=1，ch1 已反向映射使其符合直觉。

## 3. PID 控制器 (`pid.py`)

```python
pid = PID(kp=0.03, kd=0.015, output_min=-1.0, output_max=1.0)
pid.set_setpoint(target)
output = pid.update(measured, dt, velocity=ekf_velocity)
pwm = 1500 + output * max_delta
```

### 特性
- **D on measurement**: D 项作用在测量值变化上，而非误差。设点阶跃不会引发 D 项爆炸。
- **EKF 角速度直接传入**: `velocity` 参数可选，传入飞控 EKF 滤波后的角速度，跳过数值差分。水下惯性大、IMU 高频噪声环境下效果显著。
- **纯 P/PD 均可**: ki=kd=0 即纯 P；设 ki>0 启用 I 项（有积分限幅防饱和）。

### 增益参考值

| 轴 | kp | kd | 说明 |
|----|-----|-----|------|
| yaw (turn) | 0.03 | 0.015 | 30°→满 PWM |
| yaw (hold) | 0.015 | 0.015 | 小力纠正 |

## 4. 分级控制 vs PID

项目早期尝试过 PID 全闭环，在水下转向场景表现不佳：
- 水下惯性大、延迟长，D 项通过角度差分计算导致数值爆炸
- PID 输出在高频采样下饱和，反向刹车力不足
- 改用分级 PWM 后过冲消失

随后引入 EKF 角速度替代差分，PID 回归并稳定工作。两条路径并存：
- **分级**: 适合开环动作、粗暴刹车
- **比例控制 (P)**: 适合姿态保持、小偏差微调
- **PD (P+D)**: EKF 角速度做阻尼，适合水下转向、防止过冲

## 5. ROV 类 (`vehicle.py`)

```python
rov = ROV()
rov.connect()                    # 连接、读传感器、校准姿态+深度
rov.arm() / rov.disarm()
rov.set_raw(forward=0.4)         # [-1,1] → PWM
rov.get_attitude()               # (roll°, pitch°, yaw°) 校准后
rov.get_depth()                  # (m) 水面校准后
rov.set_mode(RovMode.SURFACE)    # 切换水面/水下 (设 MOT_SURFACE_MODE)
rov.pump(1, True)                # 控制水泵 (RELAY 0-based)
rov.emergency_surface()          # 翻正→全力上升→排水→SURFACE
```

### 后台线程
连接后自动启动传感器读取线程，持续缓存以下 MAVLink 消息:
- ATTITUDE → roll/pitch/yaw + 角速度
- SCALED_PRESSURE → 深度
- HEARTBEAT → arm 状态
- BATTERY_STATUS → 电压

### 校准
- **姿态**: `connect()` 启动后自动记录当前 roll/pitch/yaw 作零点 (`_att_offset`)
- **深度**: 启动后自动记录当前气压作水面零点 (`_surface_pressure`)
- 均为纯软件偏移，不写飞控参数

### 翻转检测
后台线程实时监控 `|roll| > 90°` → 翻转状态。`set_raw()` 自动根据翻转状态反转 throttle/roll/pitch/yaw 符号。

## 6. 水泵控制

4 个水泵对应 RELAY 0-3 (0-based):

| RELAY | 泵 | 类型 | Python |
|-------|----|------|--------|
| 0 | 1 | 进水 | `rov.pump(1, True)` |
| 1 | 2 | 排水 | `rov.pump(2, True)` |
| 2 | 3 | 进水 | `rov.pump(3, True)` |
| 3 | 4 | 排水 | `rov.pump(4, True)` |

水泵逻辑嵌入 `hold_depth()`:
- **下潜** → 入水泵 ON → 深度 > 0.3m → 入水泵 OFF
- **上浮** → 深度 < 0.1m → 排水泵 ON → 到达水面 → OFF

## 7. 固件补丁

`firmware_patches/` 提供两个文件的替换:

| 文件 | 改动 |
|------|------|
| `AP_Motors6DOF.h` | +1 行 `AP_Int8 _surface_mode` |
| `AP_Motors6DOF.cpp` | +5 行注册 `MOT_SURFACE_MODE` +4 行输出逻辑 |

新增参数:
- `0` = 水下 (全电机)
- `1` = 水面正常 (停 3,4)
- `2` = 水面翻转 (停 1,2)

## 8. 任务示例

### 水面转弯 (`mission.py`)
```
直行 3s → PD 右转 90° → P 保持航向 6s
```
- PD 用 EKF 角速度阻尼防过冲
- yaw 归一化处理 ±180 环绕

### 下潜定深 (`mission_dive.py`)
```
入水泵 ON → 下潜 0.4m → 入水泵 OFF → 姿态稳定 →
上浮 → 深度 < 0.1m → 排水泵 ON → SURFACE
```

`hold_depth()` 同时控制 4 轴:
- ch1 俯仰稳定 (>3° 纠正)
- ch2 横滚稳定 (>3° 纠正)
- ch3 深度控制 (分级或 P)
- ch4 航向保持 (记录起始 yaw)
- 自动切换水面/水下模式

## 9. 开发要点

1. **不要通过 `rov.set_raw()` 发指令**（内部有锁竞争）。直接 `rov.mav.mav.rc_channels_override_send(1,1, ...)`
2. **通道位置不能错**: 参数顺序 = ch1,ch2,ch3,ch4,ch5,ch6,ch7,ch8
3. **PWM 死区约 ±30**: 1560 以上才开始有明显推力
4. **深度是正数水下**: 水面=0, 水下>0 (已校准)
5. **EKF 角速度优于差分**: 传入 `velocity=rov.get_sensor('yawspeed')` 给 PID

## 10. 通信

- 电台: `/dev/ttyUSB0` @ 57600bps
- 烧录: `/dev/ttyACM0` (USB 直连飞控)
- `config.py` 中 `SYSID=1, COMPID=1`
