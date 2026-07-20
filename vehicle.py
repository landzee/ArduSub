"""ROV 控制主类"""

import time
import threading
from pymavlink import mavutil

import config
from pid import PID


class ROV:
    def __init__(self):
        self.mav = None
        self.armed = False
        self.flip_state = config.FlipState.NORMAL
        self.rov_mode = config.RovMode.UNDERWATER
        self.mot_surface_mode_available = False

        # 传感器缓存
        self.sensors = {
            'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0,
            'rollspeed': 0.0, 'pitchspeed': 0.0, 'yawspeed': 0.0,
            'depth_m': 0.0,
            'press_abs': 1013.0, 'press_diff': 0.0,
            'voltage': 0.0,
        }
        self._sensor_lock = threading.Lock()

        # PID 控制器
        self.pid_depth = PID(
            kp=config.PID_DEPTH_P, ki=config.PID_DEPTH_I, kd=config.PID_DEPTH_D, ff=config.PID_DEPTH_FF,
            output_min=-1.0, output_max=1.0)
        self.pid_roll = PID(
            kp=config.PID_ROLL_P, ki=config.PID_ROLL_I, kd=config.PID_ROLL_D,
            output_min=-1.0, output_max=1.0)
        self.pid_pitch = PID(
            kp=config.PID_PITCH_P, ki=config.PID_PITCH_I, kd=config.PID_PITCH_D,
            output_min=-1.0, output_max=1.0)
        self.pid_yaw = PID(
            kp=config.PID_YAW_P, ki=config.PID_YAW_I, kd=config.PID_YAW_D,
            output_min=-1.0, output_max=1.0)

        # 读取线程
        self._reader_running = False
        self._reader_thread = None
        self._heartbeat_armed = False

        # 姿态校准偏移（启动时记下，后续 get_attitude 自动减去）
        self._att_offset = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}

    # ── 连接 ────────────────────────────────────────────────

    def connect(self, port=config.PORT, baud=config.BAUD):
        """连接飞控"""
        self.mav = mavutil.mavlink_connection(port, baud=baud)
        self.mav.wait_heartbeat(timeout=10)
        print(f"✅ 连接成功  sysid={config.SYSID}")

        # 检查 MOT_SURFACE_MODE 参数是否存在
        self.mav.mav.param_request_read_send(
            config.SYSID, config.COMPID, b'MOT_SURFACE_MODE', -1)
        time.sleep(0.5)
        msg = self.mav.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        self.mot_surface_mode_available = (msg is not None)
        if self.mot_surface_mode_available:
            print("✅ MOT_SURFACE_MODE 可用")
        else:
            print("ℹ️ MOT_SURFACE_MODE 不支持（原版固件）")

        # 启动传感器读取线程（先启动，再校准）
        self._reader_running = True
        self._reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader_thread.start()

        # 自动姿态校准：等传感器数据稳定后把当前姿势记作零点
        self.calibrate_attitude()

    def _reader_loop(self):
        """后台持续读取 MAVLink 消息"""
        while self._reader_running:
            if self.mav is None:
                break
            msg = self.mav.recv_match(blocking=True, timeout=0.1)
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'HEARTBEAT':
                self._heartbeat_armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            with self._sensor_lock:
                if t == 'ATTITUDE':
                    self.sensors['roll'] = msg.roll
                    self.sensors['pitch'] = msg.pitch
                    self.sensors['yaw'] = msg.yaw
                    self.sensors['rollspeed'] = msg.rollspeed
                    self.sensors['pitchspeed'] = msg.pitchspeed
                    self.sensors['yawspeed'] = msg.yawspeed
                elif t == 'SCALED_PRESSURE':
                    self.sensors['press_abs'] = msg.press_abs
                    self.sensors['press_diff'] = msg.press_diff
                    # 估算深度（米），水面气压为参考
                    self.sensors['depth_m'] = (msg.press_abs - config.PRESSURE_AT_SURFACE) * config.DEPTH_PER_HPA
                elif t == 'BATTERY_STATUS':
                    if msg.voltages[0] != 65535:
                        self.sensors['voltage'] = msg.voltages[0] / 1000.0

    # ── 传感器读数 ─────────────────────────────────────────

    def get_sensor(self, name):
        """读取最新传感器值（线程安全）"""
        with self._sensor_lock:
            return self.sensors.get(name, 0.0)

    def get_attitude(self):
        """返回校准后的 (roll, pitch, yaw) 弧度"""
        with self._sensor_lock:
            r = self.sensors['roll'] - self._att_offset['roll']
            p = self.sensors['pitch'] - self._att_offset['pitch']
            y = self.sensors['yaw'] - self._att_offset['yaw']
            return (r, p, y)

    def get_depth(self):
        """返回深度（米）"""
        return self.get_sensor('depth_m')

    def calibrate_attitude(self):
        """姿态置零：把当前姿态记作参考零点，纯软件偏移，不影响飞控"""
        # 等传感器有数据（最多等 3 秒）
        for _ in range(30):
            if self.get_sensor('roll') != 0.0:
                break
            time.sleep(0.1)
        self._att_offset['roll'] = self.get_sensor('roll')
        self._att_offset['pitch'] = self.get_sensor('pitch')
        self._att_offset['yaw'] = self.get_sensor('yaw')
        print(f"✅ 姿态校准完成  offset: roll={self._att_offset['roll']*57.3:.1f}° "
              f"pitch={self._att_offset['pitch']*57.3:.1f}° yaw={self._att_offset['yaw']*57.3:.1f}°")
        # 重置 PID
        for pid in [self.pid_roll, self.pid_pitch, self.pid_yaw]:
            pid.reset()
            pid.set_setpoint(0.0)

    # ── 翻转检测 ───────────────────────────────────────────

    def update_flip_state(self):
        """根据 roll 更新翻转状态"""
        roll_deg = self.get_sensor('roll') * 57.2958
        thresh = config.FLIP_THRESHOLD_DEG
        hyst = config.FLIP_HYSTERESIS

        if self.flip_state == config.FlipState.NORMAL:
            if abs(roll_deg) > thresh + hyst:
                self.flip_state = config.FlipState.INVERTED
                print("🔄 检测到翻转 → INVERTED")
        else:
            if abs(roll_deg) < thresh - hyst:
                self.flip_state = config.FlipState.NORMAL
                print("🔄 恢复正常 → NORMAL")

    # ── 模式切换 ───────────────────────────────────────────

    def set_mode(self, mode):
        """切换水面/水下模式"""
        self.rov_mode = mode
        self.update_flip_state()

        if mode == config.RovMode.SURFACE:
            if self.flip_state == config.FlipState.NORMAL:
                # 水面+正常 → 让 3,4,5,6 停转
                if self.mot_surface_mode_available:
                    self._set_param('MOT_SURFACE_MODE', 1.0)
                    print("🌊 水面模式 (NORMAL)")
                else:
                    print("⚠️ MOT_SURFACE_MODE 不可用")
            else:
                # 水面+翻转 → 让 1,2,5,6 停转
                if self.mot_surface_mode_available:
                    self._set_param('MOT_SURFACE_MODE', 2.0)
                    print("🌊 水面模式 (INVERTED)")
                else:
                    print("⚠️ MOT_SURFACE_MODE 不可用")
        else:
            # 水下模式
            if self.mot_surface_mode_available:
                self._set_param('MOT_SURFACE_MODE', 0.0)
            print("🌊 水下模式")

    # ── 底层控制 ──────────────────────────────────────────

    def _set_param(self, name, value):
        """设置飞控参数"""
        if self.mav is None:
            return
        self.mav.mav.param_set_send(
            config.SYSID, config.COMPID,
            name.encode(), float(value),
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    def _pwm(self, value):
        """将 [-1, 1] 转为 PWM 值"""
        return int(config.PWM_NEUTRAL + value * config.PWM_RANGE)

    def set_raw(self, forward=0.0, vertical=0.0, roll=0.0, pitch=0.0, yaw=0.0):
        """直接 RC override，自动处理翻转映射

        forward/vertical/roll/pitch/yaw 范围 [-1, 1]
        """
        self.update_flip_state()

        # 翻转映射修正（180° roll 翻转后体坐标系变化）
        if self.flip_state == config.FlipState.INVERTED:
            vertical = -vertical   # 体 Z 轴反向
            roll = -roll           # 体 Y 轴反向
            pitch = -pitch         # 体 X 轴广义反向
            yaw = -yaw             # 体 Z 轴旋转反向
            # forward 不变（体 X 轴仍指向前方）

        # 水面模式：限制电机输出
        if self.rov_mode == config.RovMode.SURFACE:
            if self.flip_state == config.FlipState.NORMAL:
                # 3,4,5,6 停转 — 只发 forward, yaw(低)
                vertical = 0.0
                roll = 0.0
                pitch = 0.0
            else:
                # 1,2,5,6 停转 — 只发 forward, yaw(低)
                vertical = 0.0
                roll = 0.0
                pitch = 0.0
                # 但 forward/yaw 会转 1,2...
                # 受限，需要 firmware 级 SURFACE_INVERTED 支持

        ch = [config.PWM_NEUTRAL] * 8
        ch[config.CH_PITCH - 1]    = self._pwm(pitch)
        ch[config.CH_ROLL - 1]     = self._pwm(roll)
        ch[config.CH_THROTTLE - 1] = self._pwm(vertical)
        ch[config.CH_YAW - 1]      = self._pwm(yaw)
        ch[config.CH_FORWARD - 1]  = self._pwm(forward)

        self.mav.mav.rc_channels_override_send(
            config.SYSID, config.COMPID,
            ch[0], ch[1], ch[2], ch[3], ch[4], ch[5], ch[6], ch[7])

    def stop(self):
        """全通道回中"""
        self.set_raw(0, 0, 0, 0, 0)

    # ── 水泵控制 ───────────────────────────────────────────

    def pump(self, num, state=True):
        """控制水泵 1-4 (0-based relay)"""
        if self.mav is None:
            return
        self.mav.mav.command_long_send(config.SYSID, config.COMPID,
            mavutil.mavlink.MAV_CMD_DO_SET_RELAY, 0,
            num - 1, 1 if state else 0, 0, 0, 0, 0, 0)

    def pump_inlets(self, state):
        """入水泵 (1,3): 下潜时进水"""
        self.pump(config.PUMP_INLET1, state)
        self.pump(config.PUMP_INLET2, state)
        if state:
            print("💧 入水泵 ON")
        else:
            print("💧 入水泵 OFF")

    def pump_drains(self, state):
        """排水泵 (2,4): 上浮时排水（仅深度<阈值时有效）"""
        depth = self.get_depth()
        if state and depth < -config.PUMP_DEPTH_THRESHOLD:
            print(f"⚠️ 深度 {depth:.1f}m，气孔未露出，不启动排水泵")
            return
        self.pump(config.PUMP_DRAIN1, state)
        self.pump(config.PUMP_DRAIN2, state)
        if state:
            print("💧 排水泵 ON")
        else:
            print("💧 排水泵 OFF")

    def pumps_all_off(self):
        for i in range(1, 5):
            self.pump(i, False)
        print("💧 所有水泵 OFF")

    # ── 深度控制（含泵协同） ────────────────────────────────

    def hold_depth(self, target_m, duration=None):
        """定深，带泵协同（下潜入水时自动启停入水泵）"""
        self.pid_depth.set_setpoint(target_m)

        # 下潜时启动入水泵
        if target_m < self.get_depth():
            self.pump_inlets(True)

        deadline = time.time() + duration if duration else float('inf')
        while time.time() < deadline:
            current = self.get_depth()
            # 深度足够深时关闭入水泵
            if target_m < 0 and current < target_m + 0.5:
                self.pump_inlets(False)
            vertical = self.pid_depth.update(current, config.CONTROL_DT)
            self.set_raw(vertical=vertical)
            time.sleep(config.CONTROL_DT)
        self.pump_inlets(False)

    # ── 紧急上浮 ───────────────────────────────────────────

    def emergency_surface(self):
        """紧急上浮：翻正 → 垂直上升 → 近水面排水 → 到达水面"""
        print("🚨 紧急上浮!")
        self.update_flip_state()
        if self.flip_state == config.FlipState.INVERTED:
            self.roll_to_normal()

        # 全力上升
        self.set_raw(vertical=1.0)
        for _ in range(50):  # 约 5 秒
            depth = self.get_depth()
            # 近水面时开排水泵
            if depth > -config.PUMP_DEPTH_THRESHOLD:
                self.pump_drains(True)
            time.sleep(config.CONTROL_DT)

        self.set_mode(config.RovMode.SURFACE)
        self.stop()
        self.pumps_all_off()
        print("✅ 已上浮")

    # ── 高层命令 ───────────────────────────────────────────

    def arm(self, retries=3):
        """解锁（通过后台读取线程检查状态，不直接读串口）"""
        if self.mav is None:
            return False
        for _ in range(retries):
            self.mav.mav.command_long_send(
                config.SYSID, config.COMPID,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1, 0, 0, 0, 0, 0, 0)
            time.sleep(2)
            if self._heartbeat_armed:
                self.armed = True
                print("✅ 已解锁")
                return True
        print("❌ 解锁失败")
        return False

    def disarm(self):
        """加锁"""
        if self.mav is None:
            return
        self.mav.mav.command_long_send(
            config.SYSID, config.COMPID,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0)
        self.armed = False
        self._heartbeat_armed = False
        print("🔒 已加锁")

    def wait_heartbeat(self, timeout=5):
        """等待心跳"""
        self.mav.wait_heartbeat(timeout=timeout)

    # ── 闭环控制 ───────────────────────────────────────────

    def hold_depth(self, target_m, duration=None):
        """定深 PID 控制"""
        self.pid_depth.set_setpoint(target_m)
        print(f"📏 定深 {target_m:.1f}m")
        deadline = time.time() + duration if duration else float('inf')
        while time.time() < deadline:
            current = self.get_depth()
            vertical = self.pid_depth.update(current, config.CONTROL_DT)
            self.set_raw(vertical=vertical)
            time.sleep(config.CONTROL_DT)

    def hold_attitude(self, target_roll_deg=0, target_pitch_deg=0, target_yaw_deg=None, duration=None):
        """姿态保持 PID"""
        self.pid_roll.set_setpoint(target_roll_deg)
        self.pid_pitch.set_setpoint(target_pitch_deg)
        if target_yaw_deg is not None:
            self.pid_yaw.set_setpoint(target_yaw_deg)

        deadline = time.time() + duration if duration else float('inf')
        while time.time() < deadline:
            r, p, y = self.get_attitude()
            roll_out = self.pid_roll.update(r * 57.2958, config.CONTROL_DT)
            pitch_out = self.pid_pitch.update(p * 57.2958, config.CONTROL_DT)
            yaw_out = self.pid_yaw.update(y * 57.2958, config.CONTROL_DT) if target_yaw_deg is not None else 0.0
            self.set_raw(roll=roll_out, pitch=pitch_out, yaw=yaw_out)
            time.sleep(config.CONTROL_DT)

    def hold_yaw(self, target_deg):
        """航向保持"""
        self.hold_attitude(target_yaw_deg=target_deg)

    def roll_to_normal(self):
        """翻转回正常姿态（5,6 差速）"""
        self.pid_roll.set_setpoint(0.0)
        print("🔄 翻转回正...")
        for _ in range(100):
            r, p, y = self.get_attitude()
            roll_out = self.pid_roll.update(r * 57.2958, config.CONTROL_DT)
            self.set_raw(roll=roll_out)
            time.sleep(config.CONTROL_DT)
            self.update_flip_state()
            if self.flip_state == config.FlipState.NORMAL:
                print("✅ 已回正")
                break

    def emergency_surface(self):
        """紧急上浮"""
        print("🚨 紧急上浮!")
        # 翻转回正
        self.update_flip_state()
        if self.flip_state == config.FlipState.INVERTED:
            self.roll_to_normal()
        # 全力上升
        self.set_raw(vertical=1.0)
        time.sleep(5)
        self.set_mode(config.RovMode.SURFACE)
        self.stop()
        print("✅ 已上浮")

    # ── 清理 ───────────────────────────────────────────────

    def close(self):
        self._reader_running = False
        if self._reader_thread:
            self._reader_thread.join(timeout=2)
        if self.armed:
            self.disarm()
        if self.mav:
            self.stop()
