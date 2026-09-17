---
class: fact
generated: false
---
# PID — 行为契约

> 源码：`inc/ctl/pid.hpp` + `src/pid.cpp`
> 血缘：lunokhod `control/wheel` → cyclotron `foc::algo::PID` → ctlkit `ctl::PID`（库化时行为未变，M0 起按本文件演进）
> 精确定义源自 cyclotron `FOC_MATH_SPEC.md` §3.4

## 接口

| 成员 | 说明 |
|---|---|
| `PID(const PIDConfig &)` | 配置一次注入，构造后不可变 |
| `CTL_NODISCARD float calc(cmd, measure, dt, const PIDPorts *ports = nullptr)` | 每拍计算；`ports` = 每拍端口（见下节），缺省 `nullptr` = 旧行为（环内差分）。**签名自模块 5 起冻结**：新特性只往 `PIDPorts` 加字段。返回值不可丢弃（C++17 标准属性，C++11 用 `__attribute__((warn_unused_result))` 兜底） |
| `void reset()` | 清全部运行时状态（含 `last_output_`）；不动 `cfg_` |
| `void set_integral(float x)` | 积分注入（bumpless transfer）；注入值 **clamp 到 `limit_i_`**（`limit_i_ <= 0` 则不限幅） |

## 配置（`PIDConfig`，构造后不可变）

`PIDConfig` 按域分三组（`PIDGains` / `PIDLimits` / `PIDTunings`）；**字段名与旧版一致，无旧路径别名**（分组设计见 `../design/pid_config_and_ports.md`）。

| 字段 | 默认 | 语义 |
|---|---|---|
| `gains_.kp_` / `gains_.ki_` / `gains_.kd_` | 0 | 增益（`ki_` 的量纲 = 输出/误差/秒；`limits_.limit_i_` 直接是输出单位的 I 贡献上限） |
| `limits_.limit_out_` | 0 | 输出对称限幅；**<= 0 = 不限幅**（0 语义统一，roadmap D-1） |
| `limits_.limit_i_` | 0 | 积分项预限幅；**<= 0 = 不限幅** |
| `tunings_.thresh_i_sep_` | 0 | 积分分离阈值；`<= 0` = 关闭分离（恒积分） |
| `tunings_.max_rate_out_` | 0 | 输出斜坡（输出单位/秒）；`> 0` 启用，`0` = 关闭（构造期归一化为“无上限速率”传给 `Ramp`，不依赖 `Ramp` 的 `0=冻结`） |
| `tunings_.d_filter_Tf_` | 0 | D 项一阶 LPF 时间常数；`0` = 直通 |

## 每拍计算（`calc(cmd, measure, dt)`）

```
⓪ NaN/Inf 守卫：cmd / measure / dt 任一非有限 → 本拍**不更新任何状态**（含 D 滤波与输出斜坡的内部状态），
                 直接返回上一拍输出（`last_output_`；代码里写成自实现的 `is_finite`，不引 <cmath>）
① dt 守卫：dt <= 0 或 dt > 0.5 → dt := 0.001
② error = cmd - measure;  p_term = kp · error
③ I 项（梯形/Tustin）：
     i_temp  = integral + ki · dt · 0.5 · (error + error_prev)
     i_temp  = clamp(i_temp, ±limit_i)                    # 预限幅（抗饱和）；limit_i <= 0 → 不限幅
     if thresh_i_sep <= 0 或 |error| <= thresh_i_sep:
         integral = i_temp                                # 否则冻结（积分分离）
④ D 项（微分先行 + LPF）：微分来源二选一
     if ports 提供了 measure_dot:                          # A1 外部微分（观测器）
         d_raw = -kd · measure_dot                         # 直接替代环内差分；符号约定 = d(measure)/dt
     else:
         d_raw = -kd · (measure - measure_prev) / dt       # 环内差分（旧行为）；设定值不进 D
     d_term = d_filter.calc(d_raw, dt)                     # Tf=0 → 直通（两条来源同样过滤波）
⑤ 合成输出：
     output = clamp(p_term + d_term + integral, ±limit_out)   # limit_out <= 0 → 不限幅
     output = ramp_out.calc(output, dt)                       # 斜坡恒开启（关闭 → 无上限速率，直通）
⑥ 状态更新：error_prev = error;  measure_prev = measure
⑦ 记录输出：last_output = output（供 ⓪ 的非法输入路径回退；与 ⑥ 同属“状态写回”）
```

## 状态与复位

- 内部状态：`integral_`、`error_prev_`、`measure_prev_`、`last_output_`、`d_filter_`（LPF）、`ramp_out_`（Ramp）；全部私有。
- `reset()`：上述状态全清零（等价于"从未运行过"）；**不触碰** `cfg_`。
- 注意：`reset()` 后 `measure_prev_ = 0`，若首拍 measure 非 0，D 项会出现一拍脉冲
  （微分先行的正常语义；bumpless 场景用 `set_integral` / 外部微分注入解决）。

## 每拍端口（`PIDPorts`，模块 5 · A1）

```cpp
struct PIDPorts {
    const float *meas_dot_ = nullptr;   // 外部微分（观测器提供）；nullptr → 用环内差分
};
```

| 情况 | 行为 |
|---|---|
| `ports == nullptr` | 旧行为：D 项用环内差分 |
| `ports != nullptr` 但 `meas_dot_ == nullptr` | 同上（等同未提供） |
| `meas_dot_` 指向有限值 | D 项**直接替代**环内差分：`d_raw = -kd·(*meas_dot_)`；**量纲/符号 = d(measure)/dt**（与环内差分同号） |
| `meas_dot_` 指向 NaN/Inf | 按**非法输入**处理（D5-1）：本拍不更新任何状态、返回上一拍输出、置 `input_fault_` |

**为什么要有它（解决的问题）**：D 项本质是对测量的**差分**，差分在高噪声/低采样率下放大噪声；
若系统已有观测器/状态估计器提供高质量导数（如 FOC 的速度估计），环内再差分一次是**重复劳动 + 额外噪声 + 额外相位滞后**。
注入外部导数 = 让 D 项直接用"更干净的同一信息"，同时保留 D 滤波与微分先行的无冲击特性。

## 非法输入（NaN / Inf）行为（M0 · C1）

- `cmd` / `measure` / `dt` 任一非有限 → **本拍不发散**：不更新任何状态（积分、误差、测量、D 滤波、输出斜坡内部状态全部保持），
  返回上一拍输出；下一拍给合法输入即可继续，无需 `reset()`。
- 为什么这样：① 非法值进积分器会**永久锁死**（`constrainf(NaN)` 两个比较均假 → 原样返回 NaN）；
  ② 返回上一拍输出对 FOC 最安全（电压指令不跳变）；③ 不更新状态才能保证"污染零传播"。
- ⚠ 目前是**静默回退**：上层无法区分"正常输出"与"因非法输入回退"——fault 出口待 M1 的 `PIDFlags` 补（roadmap D-5）。
- 覆盖：`tests/smoke_test.cpp` 的 `test_pid_nan_recovery`（含积分状态不被污染、恢复后继续累积）；
  oracle（`oracle/refctl.py`）同步实现了该行为。

## 设计决策记录（为什么是这样）

| 决策 | 理由 |
|---|---|
| 微分先行（D 对测量微分） | 设定值阶跃不进 D 项 → 无微分冲击（derivative kick） |
| 梯形（Tustin）积分 | dt 抖动/变化时比矩形积分精度高、漂移小 |
| 积分分离 | 大误差起步阶段冻结积分，防超调；`<=0` 关闭以保持 0=disabled 语义 |
| 积分预限幅（而非只限输出） | 饱和期间限制积分增长（静态抗饱和） |
| 输出斜坡在限幅之后 | 斜坡限"变化率"、限幅限"幅值"，顺序固定；饱和判定必须用在**斜坡前、限幅后**的量 |
| dt 逐次传入（非构造固定 Ts） | 多环共用、变周期现实；守卫防除零/垃圾 dt |
| NaN/Inf 时返回**上一拍输出**（而非 0） | 对 FOC 安全：输出不跳变，不会把"非法输入"误传成"零电压"；代价是多存一个 `last_output_` |
| `set_integral` 注入值 clamp 到 `limit_i_` | 防"注入即 windup"；与内部预限幅语义一致（AC_PID 同款） |
| `0` 语义统一（D-1） | 限幅类 `<= 0` = 不限幅（原来"钳死到 0"，`PIDConfig{}` 默认配置下输出恒 0，是陷阱）；阈值/速率类 `0` = 关闭；`Tf`/`range` 类 `0` 本就是自然退化值 |
| 斜坡语义不串层 | PID 的 `0 = 关闭` 与 `Ramp` 的 `0 = 冻结` 在**构造期**归一化（换成无上限速率）——`Ramp` 原语语义保持干净，PID 用户看到的仍是 `0 = 关闭` |

## 已知缺口（候选清单，未实现）

> 完整定义见 `../research/optimization_considerations.md`，排期见 `../roadmap.md`。

| 编号 | 缺口 | 影响 |
|---|---|---|
| B1 | 静态 clamp 抗饱和，无条件积分/back-calculation | 输出饱和期间积分仍可能涨到 `limit_i_`，退出饱和有迟滞 |
| D1/D3 | 无状态 getter / 饱和标志 | 无法观测 P/I/D 贡献与饱和状态，调参/诊断/自整定无从下手 |
| D3/M1 | 非法输入无 fault 出口 | 现在是静默回退（返回上一拍输出），上层无法区分"正常"与"回退" |
| A3 | 仅对称限幅 | 再生等非对称工况表达不了 |
| — | 无前馈、无目标滤波 | 跟踪性能与抗扰能力的提升空间（F1/F2） |

## 变更历史

| 版本 | 变更 |
|---|---|
| Unreleased | M2/A1：`PIDPorts` 首次登场（`meas_dot_` 外部微分注入）+ `calc` 签名冻结（尾部默认参数端口） |
| Unreleased | 配置分组：`PIDConfig` → `PIDGains` / `PIDLimits` / `PIDTunings`（字段名不变、无旧路径别名；行为逐字等价） |
| Unreleased | M0：「`0` 语义统一（D-1）——限幅类 `<= 0` = 不限幅；斜坡 0=关闭在构造期归一化；**行为变更点：`limit = 0` 的既有配置** |
| Unreleased | M0 部分：NaN/Inf 守卫（返回上一拍输出、零污染）、`set_integral`（clamp 注入）、`CTL_NODISCARD` |
| v0.0.1 | 库化（`foc::algo` → `ctl`），行为未变 |
