"""通用 PID 控制器"""

class PID:
    """离散 PID 控制器"""

    def __init__(self, kp=0.0, ki=0.0, kd=0.0, ff=0.0, setpoint=0.0,
                 integral_min=-1.0, integral_max=1.0, output_min=-1.0, output_max=1.0,
                 d_on_meas=True):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.ff = ff          # 前馈系数
        self.setpoint = setpoint

        self.integral = 0.0
        self.last_error = 0.0
        self.last_measured = 0.0
        self.d_on_meas = d_on_meas  # True=D在测量值上(水下推荐)

        self.integral_min = integral_min
        self.integral_max = integral_max
        self.output_min = output_min
        self.output_max = output_max

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_measured = 0.0

    def set_setpoint(self, sp):
        self.setpoint = sp

    def update(self, measured, dt, velocity=None):
        """PID 一步计算，返回控制量 [-1, 1]

        velocity: 可选，EKF 滤波后的角速度/线速度。
                  传入时 D 项直接用此值（跳过差分），适合水下惯性系统。"""
        error = self.setpoint - measured

        p_out = self.kp * error

        self.integral += error * dt
        self.integral = max(self.integral_min, min(self.integral_max, self.integral))
        i_out = self.ki * self.integral

        if velocity is not None:
            # 使用 EKF 滤波速度（干净），跳过差分
            d_out = -self.kd * velocity
        elif dt > 0:
            if self.d_on_meas:
                derivative = -(measured - self.last_measured) / dt
            else:
                derivative = (error - self.last_error) / dt
            d_out = self.kd * derivative
        else:
            d_out = 0.0
        self.last_error = error
        self.last_measured = measured

        ff_out = self.ff * self.setpoint

        output = p_out + i_out + d_out + ff_out
        output = max(self.output_min, min(self.output_max, output))
        return output
