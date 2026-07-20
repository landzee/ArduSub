# ROV 控制脚本 (ArduSub)

基于 `RC_CHANNELS_OVERRIDE` 的水下航行器（ROV）控制脚本，通过 MAVLink 控制 ArduSub 飞控。

## 文件说明

| 文件 | 用途 |
|------|------|
| `vehicle.py` | ROV 控制类 — 连接、传感器、翻转检测、RC 控制 |
| `pid.py` | 通用 PID 控制器 |
| `config.py` | 通道映射、PWM 范围、PID 参数 |
| `mission.py` | 任务脚本示例 |
| `sub_control.py` | 连接+解锁测试（保留） |
| `motor_test.py` | 电机方向测试（保留） |
| `firmware_patches/` | 固件改动文件（覆盖到 ardupilot 源码） |

## 快速开始

```bash
# 1. 创建虚拟环境
python3 -m venv venv
source venv/bin/activate

# 2. 安装依赖
pip install -r requirements.txt

# 3. 连接电台，运行任务
python3 mission.py
```

## 硬件要求

- 飞控: ArduSub (Pixhawk 系列)
- 通信: USB 电台 (/dev/ttyUSB0)
- 推进器: 6x 水平/垂直 (Vectored 布局)
- 固件: 需支持 `MOT_SURFACE_MODE` 参数

### 固件改动

`firmware_patches/` 目录下有两个文件需覆盖到 ardupilot 源码：

| 文件 | 覆盖位置 |
|------|---------|
| `firmware_patches/AP_Motors6DOF.h` | `<ardupilot>/libraries/AP_Motors/AP_Motors6DOF.h` |
| `firmware_patches/AP_Motors6DOF.cpp` | `<ardupilot>/libraries/AP_Motors/AP_Motors6DOF.cpp` |

覆盖后重新编译烧录：

```bash
cd <ardupilot>
./waf configure --board Pixhawk1
./waf sub
./waf --upload sub
```

## 控制方式

Python → RC_CHANNELS_OVERRIDE → 飞控混控矩阵 → 推进器 PWM

## 已验证的通道映射

| 通道 | >1500 | <1500 |
|------|-------|-------|
| CH5 (forward) | 前进 | 后退 |
| CH3 (throttle) | 上升 | 下潜 |
| CH2 (roll) | 右倾 | 左倾 |
| CH1 (pitch) | 上抬 (RC1_REVERSED=1) | 下埋 |
| CH4 (yaw) | 右转 | 左转 |
