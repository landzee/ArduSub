# 固件改动

基于 ArduSub (Pixhawk1) 固件，改了两个文件：

## 改动说明

### AP_Motors6DOF.h
- 第 69 行：新增 `AP_Int8 _surface_mode` 成员变量

### AP_Motors6DOF.cpp
- 第 128 行：注册 `MOT_SURFACE_MODE` 参数 (index=14, 默认 0)
- 第 291 行：`output_to_motors()` 新增水面模式逻辑

### MOT_SURFACE_MODE 参数

| 值 | 模式 | 效果 |
|----|------|------|
| 0 | 水下 | 全部 6 推进器正常 |
| 1 | 水面正常 | 停 MOT_3, MOT_4 |
| 2 | 水面翻转 | 停 MOT_1, MOT_2 |

## 使用方法

将两个 `.h/.cpp` 文件复制到 ardupilot 源码对应位置：

```bash
# 替换原文件
cp AP_Motors6DOF.h <ardupilot>/libraries/AP_Motors/AP_Motors6DOF.h
cp AP_Motors6DOF.cpp <ardupilot>/libraries/AP_Motors/AP_Motors6DOF.cpp

# 编译烧录
cd <ardupilot>
./waf configure --board Pixhawk1
./waf sub
./waf --upload sub
```
