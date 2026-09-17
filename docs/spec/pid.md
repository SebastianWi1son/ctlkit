# PID — 行为契约

> 源码：`include/ctl/pid.hpp` + `src/pid.cpp`
> 血缘：lunokhod `control/wheel` → cyclotron `foc::algo::PID` → ctlkit `ctl::PID`（行为未变）
> 精确定义源自 cyclotron `FOC_MATH_SPEC.md` §3.4

## 配置（`PIDConfig`，构造后不可变）

| 字段 | 默认 | 语义 |
|---|---|---|
| `kp_` / `ki_` / `kd_` | 0 | 增益（`ki_` 的量纲 = 输出/误差/秒；`limit_i_` 直接是输出单位的 I 贡献上限） |
| `limit_out_` | 0 | 输出对称限幅；⚠ **0 = 钳死到 0**（不是不限幅，D-1 待定案） |
| `limit_i_` | 0 | 积分项预限幅；⚠ 同上 0 语义 |
| `thresh_i_sep_` | 0 | 积分分离阈值；`<= 0` = 关闭分离（恒积分） |
| `max_rate_out_` | 0 | 输出斜坡（输出单位/秒）；`> 0` 启用，`0` = 关闭 |
| `d_filter_Tf_` | 0 | D 项一阶 LPF 时间常数；`0` = 直通 |

## 每拍计算（`calc(cmd, measure, dt)`）

```
① dt 守卫：dt <= 0 或 dt > 0.5 → dt := 0.001
② error = cmd - measure;  p_term = kp · error
③ I 项（梯形/Tustin）：
     i_temp  = integral + ki · dt · 0.5 · (error + error_prev)
     i_temp  = clamp(i_temp, ±limit_i)                    # 预限幅（抗饱和）
     if thresh_i_sep <= 0 或 |error| <= thresh_i_sep:
         integral = i_temp                                # 否则冻结（积分分离）
④ D 项（微分先行 + LPF）：
     d_raw = -kd · (measure - measure_prev) / dt          # 只对测量微分，设定值不进 D
     d_term = d_filter.calc(d_raw, dt)                    # Tf=0 → 直通
⑤ 合成输出：
     output = clamp(p_term + d_term + integral, ±limit_out)
     if max_rate_out > 0: output = ramp_out.calc(output, dt)
⑥ 状态更新：error_prev = error;  measure_prev = measure
```

## 状态与复位

- 内部状态：`integral_`、`error_prev_`、`measure_prev_`、`d_filter_`（LPF）、`ramp_out_`（Ramp）；全部私有。
- `reset()`：上述状态全清零（等价于"从未运行过"）；**不触碰** `cfg_`。
- 注意：`reset()` 后 `measure_prev_ = 0`，若首拍 measure 非 0，D 项会出现一拍脉冲
  （微分先行的正常语义；bumpless 场景用候选 A1/A2 解决）。

## 设计决策记录（为什么是这样）

| 决策 | 理由 |
|---|---|
| 微分先行（D 对测量微分） | 设定值阶跃不进 D 项 → 无微分冲击（derivative kick） |
| 梯形（Tustin）积分 | dt 抖动/变化时比矩形积分精度高、漂移小 |
| 积分分离 | 大误差起步阶段冻结积分，防超调；`<=0` 关闭以保持 0=disabled 语义 |
| 积分预限幅（而非只限输出） | 饱和期间限制积分增长（静态抗饱和） |
| 输出斜坡在限幅之后 | 斜坡限"变化率"、限幅限"幅值"，顺序固定；饱和判定必须用在**斜坡前、限幅后**的量 |
| dt 逐次传入（非构造固定 Ts） | 多环共用、变周期现实；守卫防除零/垃圾 dt |

## 已知缺口（候选清单，未实现）

> 完整定义见 `../research/optimization_considerations.md`，排期见 `../roadmap.md`。

| 编号 | 缺口 | 影响 |
|---|---|---|
| C1 | 无 NaN/Inf 防护 | 一次非法输入 → `integral_` 永久 NaN，只能 reset |
| A2 | 无 `set_integral` 注入口 | 与 LPF/Ramp/SmoothPlanner 的 `set_state` 设计语言不一致，bumpless 切换缺手段 |
| B1 | 静态 clamp 抗饱和，无条件积分/back-calculation | 输出饱和期间积分仍可能涨到 `limit_i_`，退出饱和有迟滞 |
| D1/D3 | 无状态 getter / 饱和标志 | 无法观测 P/I/D 贡献与饱和状态，调参/诊断/自整定无从下手 |
| A3 | 仅对称限幅 | 再生等非对称工况表达不了 |
| — | 无前馈、无目标滤波 | 跟踪性能与抗扰能力的提升空间（F1/F2） |

## 变更历史

| 版本 | 变更 |
|---|---|
| v0.0.1 | 库化（`foc::algo` → `ctl`），行为未变 |
