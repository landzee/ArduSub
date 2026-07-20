"""ROV 控制配置常量"""

# ---- 通信 ----
PORT = '/dev/ttyUSB0'
BAUD = 57600
SYSID = 1          # 飞控系统 ID
COMPID = 1         # 飞控组件 ID

# ---- RC 通道映射 ----
CH_PITCH    = 1
CH_ROLL     = 2
CH_THROTTLE = 3
CH_YAW      = 4
CH_FORWARD  = 5
CH_LATERAL  = 6

# ---- PWM 范围 ----
PWM_NEUTRAL = 1500
PWM_MIN     = 1100
PWM_MAX     = 1900
PWM_RANGE   = PWM_MAX - PWM_NEUTRAL  # 400

# ---- 方向语义（已验证） ----
# ch5 > 1500 → 前进
# ch3 > 1500 → 上升（5↓6↓）
# ch2 > 1500 → 右倾（5↓6↑）
# ch1 > 1500 → 上抬（需 RC1_REVERSED=1）
# ch4 > 1500 → 右转

# ---- 翻转检测 ----
FLIP_THRESHOLD_DEG = 90    # |roll| > 90° = 翻转
FLIP_HYSTERESIS    = 10    # 滞后，避免边界抖动

# ---- 推进器水面模式分配 ----
# SURFACE + NORMAL:   电机 3,4,5,6 停转，仅 1,2 转
# SURFACE + INVERTED: 电机 1,2,5,6 停转，仅 3,4 转
# UNDERWATER:        全部 1-6 可用

# ---- PID 默认参数（干地初值，下水后调） ----
PID_DEPTH_P   = 0.3
PID_DEPTH_I   = 0.02
PID_DEPTH_D   = 0.1
PID_DEPTH_FF  = 0.0     # 前馈（浮力补偿）

# ---- 水泵 ----
PUMP_INLET1  = 1  # 进水1 → pump(1)
PUMP_INLET2  = 3  # 进水2 → pump(3)
PUMP_DRAIN1  = 2  # 排水1 → pump(2)
PUMP_DRAIN2  = 4  # 排水2 → pump(4)
PUMP_DEPTH_THRESHOLD = 0.3  # 米：排水泵在此深度以下不工作（气孔未露出）

PID_ROLL_P    = 0.5
PID_ROLL_I    = 0.01
PID_ROLL_D    = 0.05

PID_PITCH_P   = 0.5
PID_PITCH_I   = 0.01
PID_PITCH_D   = 0.05

PID_YAW_P     = 0.3
PID_YAW_I     = 0.005
PID_YAW_D     = 0.05

# ---- 深度 ----
PRESSURE_AT_SURFACE = 1013  # 海平面气压 hPa，用于深度估算
DEPTH_PER_HPA       = 0.01 # 1 hPa ≈ 1 cm

# ---- 控制频率 ----
CONTROL_HZ   = 20     # PID 更新频率
CONTROL_DT   = 1.0 / CONTROL_HZ
READ_HZ      = 50     # 传感器读取频率

# ---- 模式 ----
class RovMode:
    SURFACE = 0
    UNDERWATER = 1

# MOT_SURFACE_MODE 参数值（需对应固件定义）
# 0 = 水下模式（全推进器）
# 1 = 水面+正常（停 3,4）
# 2 = 水面+翻转（停 1,2）

class FlipState:
    NORMAL = 0
    INVERTED = 1
