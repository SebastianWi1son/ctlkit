#!/usr/bin/env python3
"""ctlkit 唯一 oracle —— 独立参考实现。

定位与规矩见 oracle/README.md。三条铁律：
  · 本文件**不读、不调** inc/ 下的 C++ 源码（异源）；
  · 只按 docs/spec/*.md 的公式与语义重写（同规范、不同实现）；
  · 测试（tests/）不调用本文件 —— 只读 gen_golden.py 产出的 tests/golden/*.csv。

精度模式：
  · float64（默认）—— 期望值的主源；
  · float32        —— 仅用于【推导容差】：同算法在同精度下的自身偏差上界。

外借的尺子（验证本 oracle 用，见 validate_oracle.py）：
  · scipy.signal.lfilter —— LPF 与 PID 线性部分的教科书式独立实现；
  · fractions.Fraction   —— 精确有理数，包含钳位/分离等非线性路径的算术真值。
"""

import numpy as np

F64 = np.float64
F32 = np.float32
FLOAT_MAX = 3.402823466e+38      # 与 C++ 侧 is_finite 的上界一致


def is_finite(x):
    """非有限判定：NaN 自不相等；上下界取 float 最大值（与 C++ is_finite 同语义）。"""
    return (x == x) and (x <= FLOAT_MAX) and (x >= -FLOAT_MAX)


def clamp(val, limit):
    """对称限幅；**limit <= 0 = 不限幅（直通）**（0 语义统一，roadmap D-1）。"""
    if limit <= 0:
        return val
    if val > limit:
        return limit
    if val < -limit:
        return -limit
    return val


class LPF:
    """一阶低通：alpha = dt/(Tf+dt)；Tf=0 → 直通。无 dt 守卫（调用方保证 dt>0）。"""

    def __init__(self, Tf, dtype=F64):
        self.dtype = dtype
        self.Tf = dtype(Tf)
        self.prev = dtype(0.0)

    def reset(self):
        self.prev = self.dtype(0.0)

    def set_state(self, x):
        self.prev = self.dtype(x)

    def calc(self, raw, dt):
        d = self.dtype
        raw = d(raw)
        dt = d(dt)
        alpha = d(dt / d(self.Tf + dt))
        self.prev = d(d(alpha * raw) + d(d(d(1.0) - alpha) * self.prev))
        return self.prev


class Ramp:
    """斜率限制器：每帧 clamp 到 [prev ± max_rate·dt]；max_rate=0 → 冻结输出。"""

    def __init__(self, max_rate, dtype=F64):
        self.dtype = dtype
        self.max_rate = dtype(max_rate)
        self.prev = dtype(0.0)

    def reset(self):
        self.prev = self.dtype(0.0)

    def set_state(self, x):
        self.prev = self.dtype(x)

    def calc(self, cmd, dt):
        d = self.dtype
        cmd = d(cmd)
        dt = d(dt)
        step = d(self.max_rate * dt)
        out = cmd
        if cmd > d(self.prev + step):
            out = d(self.prev + step)
        elif cmd < d(self.prev - step):
            out = d(self.prev - step)
        self.prev = out
        return out


class PID:
    """工业级 PID：微分先行 + 梯形积分 + 积分分离 + 静态抗饱和 + 输出斜坡 + dt 守卫。"""

    def __init__(self, kp, ki, kd, limit_out, limit_i, thresh_i_sep=0.0,
                 max_rate_out=0.0, d_filter_Tf=0.0, dtype=F64):
        d = dtype
        self.dtype = dtype
        self.kp = d(kp)
        self.ki = d(ki)
        self.kd = d(kd)
        self.limit_out = d(limit_out)
        self.limit_i = d(limit_i)
        self.thresh_i_sep = d(thresh_i_sep)
        self.max_rate_out = d(max_rate_out)
        self.integral = d(0.0)
        self.error_prev = d(0.0)
        self.measure_prev = d(0.0)
        self.last_output = d(0.0)
        self.d_filter = LPF(d_filter_Tf, dtype)
        self.ramp_out = Ramp(d(max_rate_out) if d(max_rate_out) > d(0.0) else d(FLOAT_MAX), dtype)   # 0 = 关闭 → 无上限速率
        # 观测标志（M1）：out/i/dt 为本拍瞬态；input_fault 为粘滞
        self.out_saturated = False
        self.i_saturated = False
        self.dt_rejected = False
        self.input_fault = False

    def reset(self):
        d = self.dtype
        self.integral = d(0.0)
        self.error_prev = d(0.0)
        self.measure_prev = d(0.0)
        self.last_output = d(0.0)
        self.d_filter.reset()
        self.ramp_out.reset()
        self.out_saturated = False
        self.i_saturated = False
        self.dt_rejected = False
        self.input_fault = False

    def calc(self, cmd, measure, dt, measure_dot=None):
        d = self.dtype
        cmd = d(cmd)
        measure = d(measure)
        dt = d(dt)

        # ⓪ NaN/Inf 守卫（M0 · C1）：不更新任何状态，返回上一拍输出；input_fault 粘滞
        if not (is_finite(cmd) and is_finite(measure) and is_finite(dt)):
            self.input_fault = True
            return self.last_output

        # ⓪' 端口（A1）：外部微分也算输入，NaN/Inf 同语义处理
        if measure_dot is not None:
            measure_dot = d(measure_dot)
            if not is_finite(measure_dot):
                self.input_fault = True
                return self.last_output

        # ① dt 守卫（下界含 dt<=0；与 C++ 同步 —— TODO T2）
        self.dt_rejected = bool(dt < d(1e-9) or dt > d(0.5))
        if self.dt_rejected:
            dt = d(0.001)

        # ② 误差与 P 项
        error = d(cmd - measure)
        p_term = d(self.kp * error)

        # ③ I 项：梯形积分 + 预限幅 + 积分分离
        i_temp = d(self.integral + d(d(d(self.ki * dt) * d(0.5)) * d(error + self.error_prev)))
        i_limited = d(clamp(i_temp, self.limit_i))
        i_commit = bool(self.thresh_i_sep <= d(0.0) or d(abs(error)) <= self.thresh_i_sep)
        if i_commit:
            self.integral = i_limited
        # 只报“被采纳的积分值真被削过”（TODO T3）
        self.i_saturated = i_commit and bool(i_limited != i_temp)

        # ④ D 项：微分先行（外部微分优先，否则环内差分）+ LPF
        inv_dt = d(d(1.0) / dt)
        if measure_dot is not None:
            d_term_raw = d(d(-self.kd) * measure_dot)
        else:
            d_term_raw = d(d(-self.kd) * d(inv_dt * d(measure - self.measure_prev)))
        d_term = self.d_filter.calc(d_term_raw, dt)

        # ⑤ 状态更新
        self.error_prev = error
        self.measure_prev = measure

        # ⑥ 合成输出 → 对称限幅 → 输出斜坡
        out = d(clamp(d(d(p_term + d_term) + self.integral), self.limit_out))
        self.out_saturated = bool(out != d(d(p_term + d_term) + self.integral))   # 限幅前取未钳位量（roadmap D-3）
        out = self.ramp_out.calc(out, dt)   # 斜坡恒开启（关闭时速率 = 无上限，直通）
        self.last_output = out
        return out


class SmoothPlanner:
    """二阶轨迹规划：Ramp（梯形限速）+ 两级 LPF（S 曲线圆角）。"""

    def __init__(self, max_rate, Tf, dtype=F64):
        self.dtype = dtype
        self.ramp = Ramp(max_rate, dtype)
        self.f1 = LPF(Tf, dtype)
        self.f2 = LPF(Tf, dtype)

    def reset(self):
        self.ramp.reset()
        self.f1.reset()
        self.f2.reset()

    def set_state(self, x):
        self.ramp.set_state(x)
        self.f1.set_state(x)
        self.f2.set_state(x)

    def calc(self, cmd, dt):
        ramped = self.ramp.calc(cmd, dt)
        return self.f2.calc(self.f1.calc(ramped, dt), dt)


class Deadzone:
    """抛物线软死区：|e| < range → e·(|e|/range)；区外原样；range<=0 → 直通。无状态。"""

    def __init__(self, range=0.0, soft=True, dtype=F64):
        self.dtype = dtype
        self.range = dtype(range)
        self.soft = bool(soft)

    def calc(self, error):
        d = self.dtype
        error = d(error)
        abs_error = d(abs(error))
        if abs_error < self.range:
            if self.soft:
                return d(error * d(abs_error / self.range))
            return d(0.0)
        return error
