#!/usr/bin/env python3
"""validate_oracle.py —— 给 oracle 自己找尺子（本文件不产出黄金向量）。

为什么要有它：oracle 是"唯一可信"，但谁来证明 oracle 可信？
用【外借的独立实现】和【精确算术】交叉验证（各自的语义差异在下面逐条写明）：

  ① scipy.signal.lfilter —— LPF：同一差分方程的教科书实现，应逐样本一致（~1e-15）；
  ② scipy.signal.lfilter —— PID 线性部分（无限幅/无分离/无斜坡时）：
       I 通道（梯形）= ki·dt/2·(1+z⁻¹)/(1-z⁻¹)
       D 通道（对测量微分）= -kd/dt·(1-z⁻¹)
       P 通道 = kp
     三通道叠加，与 oracle 逐样本比对；
  ③ fractions.Fraction —— 精确有理数重算（含钳位、分离、斜坡等全部非线性路径）：
     这是对"算术本身"的真值，不依赖任何浮点实现；
  ④ 解析式 —— LPF 阶跃闭式解 y[k] = 1-(1-α)^k、Ramp 分段线性。

用法：python3 oracle/validate_oracle.py     （退出码 0 = 全部通过）
"""

import sys
from fractions import Fraction as Fr

import numpy as np
from scipy import signal

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from refctl import F64, LPF, PID, Ramp, SmoothPlanner, Deadzone, clamp  # noqa: E402

FAIL = 0


def check(name, cond, detail=""):
    global FAIL
    if cond:
        print(f"  ✅ {name}" + (f" —— {detail}" if detail else ""))
    else:
        FAIL += 1
        print(f"  ❌ {name} —— {detail}")


# ───────────────────────── ① / ④ LPF ─────────────────────────

def validate_lpf():
    print("\n① LPF vs scipy.signal.lfilter")
    dt, Tf = 0.001, 0.01
    x = [float(v) for v in np.sin(np.linspace(0, 20, 400)) * 3.0]
    alpha = dt / (Tf + dt)

    ref = signal.lfilter([alpha], [1.0, -(1.0 - alpha)], x)

    lpf = LPF(Tf)
    got = np.array([lpf.calc(xi, dt) for xi in x])
    err = float(np.max(np.abs(got - ref)))
    check("lfilter 逐样本一致", err < 1e-12, f"max|Δ| = {err:.3e}")

    print("\n④ LPF 阶跃闭式解 y[k] = 1-(1-α)^k")
    lpf = LPF(Tf)
    step_err = 0.0
    for k in range(1, 200):
        got = float(lpf.calc(1.0, dt))
        want = 1.0 - (1.0 - alpha) ** k
        step_err = max(step_err, abs(got - want))
    check("闭式解一致", step_err < 1e-12, f"max|Δ| = {step_err:.3e}")

    lpf = LPF(0.0)
    check("Tf=0 直通", abs(float(lpf.calc(1.234, dt)) - 1.234) == 0.0)


# ───────────────────────── ② PID 线性部分 ─────────────────────────

def validate_pid_linear():
    print("\n② PID 线性部分 vs scipy.signal.lfilter（无限幅/无分离/无斜坡/无 D 滤波）")
    dt = 0.001
    kp, ki, kd = 4.0, 200.0, 0.5
    n = 200
    cmd = np.ones(n)
    measure = 1.0 - np.exp(-np.linspace(0, 30, n))  # 任意确定性测量轨迹
    error = cmd - measure

    # P + I（梯形）合并：y[k]-y[k-1] = kp(e[k]-e[k-1]) + ki·dt/2·(e[k]+e[k-1])
    b = [kp + ki * dt / 2.0, -kp + ki * dt / 2.0]
    a = [1.0, -1.0]
    y_pi = signal.lfilter(b, a, error)

    # D（微分先行 + 无滤波）：y_d[k] = -kd/dt·(m[k]-m[k-1])
    y_d = signal.lfilter([-kd / dt, kd / dt], [1.0], measure)

    ref = y_pi + y_d

    pid = PID(kp, ki, kd, limit_out=1e9, limit_i=1e9)
    got = np.array([pid.calc(c, m, dt) for c, m in zip(cmd, measure)])
    err = float(np.max(np.abs(got - ref)))
    check("三通道叠加逐样本一致", err < 1e-10, f"max|Δ| = {err:.3e}")


# ───────────────────────── ③ 精确有理数 ─────────────────────────

def _exact_step(pid_cfg, rows):
    """精确有理数版 PID（逐字照 spec，与 refctl.PID 独立编写）。"""
    kp, ki, kd, lo, li, sep, mrate, dtf = (Fr(str(v)) for v in pid_cfg)
    integral = Fr(0)
    e_prev, m_prev = Fr(0), Fr(0)
    f_prev = Fr(0)
    r_prev = Fr(0)
    out_row = []
    for cmd, meas, dt in rows:
        cmd, meas, dt = Fr(str(cmd)), Fr(str(meas)), Fr(str(dt))
        if dt <= 0 or dt > Fr(1, 2):
            dt = Fr(1, 1000)
        error = cmd - meas
        p = kp * error
        i_t = integral + ki * dt * Fr(1, 2) * (error + e_prev)
        i_t = min(max(i_t, -li), li)
        if sep <= 0 or abs(error) <= sep:
            integral = i_t
        d_raw = -(kd) * (Fr(1) / dt) * (meas - m_prev)
        alpha = dt / (dtf + dt)
        f_prev = alpha * d_raw + (1 - alpha) * f_prev
        e_prev, m_prev = error, meas
        out = min(max(p + f_prev + integral, -lo), lo)
        if mrate > 0:
            step = mrate * dt
            out = min(max(out, r_prev - step), r_prev + step)
            r_prev = out
        out_row.append(out)
    return out_row


def validate_exact():
    print("\n③ PID（含钳位/分离/斜坡/D 滤波）vs 精确有理数")

    cases = [
        # (配置 kp,ki,kd,limit_out,limit_i,sep,max_rate,dTf, 输入行)
        ((4, 200, 0, 12, 12, 0, 0, 0), [(1, 0, 0.001), (1, 0.3, 0.001), (1, 0.9, 0.001)] * 4),
        ((20, 2000, 0, 1, 1, 0, 0, 0), [(1, 0, 0.001)] * 10 + [(1, 1.05, 0.001)] * 5),
        ((2, 100, 0.5, 50, 20, 0.5, 5, 0.005),
         [(1, 0, 0.001), (1, 0.6, 0.001), (1, 1.4, 0.001), (1, 0.2, 0.001)] * 3),
    ]
    for idx, (cfg, rows) in enumerate(cases):
        exact = _exact_step(cfg, rows)
        pid = PID(*cfg)
        got = [pid.calc(c, m, dt) for c, m, dt in rows]
        scale = max(1.0, max(abs(float(v)) for v in exact))
        err = max(abs(float(a) - float(b)) for a, b in zip(got, exact)) / scale
        check(f"case {idx} 相对误差", err < 1e-12, f"rel = {err:.3e}")

    print("\n③+ Ramp / SmoothPlanner vs 精确有理数（逐段性质）")
    r = Ramp(10.0)
    bound_ok, mono_ok, prev = True, True, 0.0
    cmd_seq = [100.0, 100.0, 100.0, -100.0, -100.0, 0.0, 0.0]
    for c in cmd_seq:
        out = float(r.calc(c, 0.1))
        if abs(out - prev) > 10.0 * 0.1 + 1e-12:
            bound_ok = False
        if prev > 0 and out < prev and c > 0:
            mono_ok = False
        prev = out
    check("Ramp 速率上界", bound_ok, "|Δ| ≤ max_rate·dt 全部成立")
    check("Ramp 单调趋向目标", mono_ok)
    frozen = Ramp(0.0)
    check("Ramp max_rate=0 冻结", float(frozen.calc(5.0, 0.1)) == 0.0)

    exact_rows = [(1.0, 0.001)] * 6
    sp = SmoothPlanner(100.0, 0.001)
    # 精确有理数：Ramp 首步 0.1 → f1: α=0.001/(0.001+0.001)=1/2 → 0.05 → f2: 0.025
    check("SmoothPlanner 首步 = Ramp+两级 LPF 组合",
          abs(float(sp.calc(1.0, 0.001)) - 0.025) < 1e-12, "0.1 → 0.05 → 0.025")


def validate_deadzone():
    print("\n③+ Deadzone vs 精确有理数 + 解析性质")
    dz = Deadzone(0.5, True)
    want = float(Fr(3, 10) * (Fr(3, 10) / Fr(5, 10)))          # 0.3·(0.3/0.5) = 0.18
    got = float(dz.calc(0.3))
    check("软衰减 e=0.3, range=0.5 → 0.18", abs(got - want) < 1e-12, f"{got:.17g}")

    check("边界 |e|=range 连续", float(dz.calc(0.5)) == 0.5)
    check("区外原样", float(dz.calc(1.2)) == 1.2 and float(dz.calc(-1.2)) == -1.2)
    check("同号不过零", all(float(Deadzone(0.5, True).calc(e)) * e >= 0 for e in
                          (-1.2, -0.3, -0.05, 0.0, 0.05, 0.3, 1.2)))

    hard = Deadzone(0.5, False)
    check("硬死区带内归零", float(hard.calc(0.3)) == 0.0 and float(hard.calc(-0.1)) == 0.0)
    check("硬死区带外原样", float(hard.calc(0.5)) == 0.5)

    off = Deadzone(0.0, True)
    check("range=0 直通（无除零）", all(float(off.calc(e)) == e for e in (-1.0, -0.2, 0.0, 0.2, 1.0)))

    # 单调：|e| 增大 → |out| 不减（在带内逐点验证）
    mono = True
    prev = 0.0
    for k in range(1, 60):
        out = abs(float(Deadzone(0.5, True).calc(k * 0.01)))
        if out < prev - 1e-15:
            mono = False
        prev = out
    check("软模式单调不降", mono)


def validate_nan_guard():
    print("\n③++ PID 非法输入守卫（M0 · C1，与 C++ 侧同语义）")
    pid = PID(0.0, 1000.0, 0.0, 1000.0, 1000.0)      # 输出 = 积分项
    base = float(pid.calc(1.0, 0.0, 0.001))           # 0.5
    nan = float("nan")
    inf = float("inf")
    hold = [float(pid.calc(nan, 0.0, 0.001)),
            float(pid.calc(1.0, nan, 0.001)),
            float(pid.calc(1.0, 0.0, nan)),
            float(pid.calc(inf, 0.0, 0.001))]
    resumed = float(pid.calc(1.0, 0.0, 0.001))        # 状态没被污染 → 0.5+1.0
    check("非法输入回退上一拍输出", all(abs(v - base) < 1e-15 for v in hold), f"{hold}")
    check("状态零污染（恢复后继续累积）", abs(resumed - 1.5) < 1e-15, f"{resumed}")


def validate_zero_semantics():
    print("\n③+++ 0 语义统一（roadmap D-1）")
    # 限幅类：0 = 不限幅（不再“钳死到 0”，默认配置输出不再恒 0）
    pid = PID(4.0, 200.0, 0.0, 0.0, 0.0)
    out = 0.0
    for _ in range(20):
        out = float(pid.calc(1.0, 0.0, 0.001))
    check("limit=0 → 不限幅（积分自由累积）", abs(out - 7.9) < 1e-9, f"{out:.6f}")

    # 输出限幅仍生效（> 0 时），而 limit_i=0 时积分不被限
    pid2 = PID(0.0, 500.0, 0.0, 1.0, 0.0)
    out2 = 0.0
    for _ in range(10):
        out2 = float(pid2.calc(1.0, 0.0, 0.001))
    check("limit_out>0 → 仍限幅；limit_i=0 → 不限积分", abs(out2 - 1.0) < 1e-12, f"{out2:.6f}")

    # 斜坡：Ramp 原语 0 = 冻结（不变）；PID 层 0 = 关闭 → 直通（构造期归一化，不串层）
    check("Ramp(0) 仍冻结（原语语义不变）", float(Ramp(0.0).calc(5.0, 0.01)) == 0.0)
    pid3 = PID(1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)      # max_rate_out = 0
    check("PID max_rate_out=0 → 直通（不冻结）", abs(float(pid3.calc(5.0, 0.0, 0.01)) - 5.0) < 1e-15)


def main():
    print("oracle 自验（外借尺子：scipy.signal.lfilter / fractions.Fraction / 解析式）")
    validate_lpf()
    validate_pid_linear()
    validate_exact()
    validate_deadzone()
    validate_nan_guard()
    validate_zero_semantics()
    print()
    if FAIL:
        print(f"❌ {FAIL} 项未通过 —— oracle 不可信，先修 oracle")
        return 1
    print("✅ oracle 全部通过")
    return 0


if __name__ == "__main__":
    sys.exit(main())
