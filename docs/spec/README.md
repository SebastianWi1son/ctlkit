---
class: fact
generated: false
---
# 行为契约（Specs）

> 本目录是 ctlkit 的**唯一事实源**：描述每个组件「现在是什么行为」。
> 与源码不一致时以源码为准，并**立即**修正本目录；任何行为改动必须同步更新本目录 + `tests/`。
> 未来打算怎么做（设计稿）在 `../design/`；为什么这么做（调研）在 `../research/`；排期在 `../roadmap.md`。
>
> **期望值来源**：`tests/` 里的数值期望**只**从 `oracle/` 生成的黄金向量（`tests/golden/*.csv`）读；
> oracle 的定义、外借尺子与容差推导见 `oracle/README.md`（规则见 `AGENTS.md` §2）。

## 全局约定

| 约定 | 规则 |
|---|---|
| 语言/依赖 | C++11；纯 `float`；无 STL / 无堆 / 无异常 |
| 时间单位 | 秒（`dt` 由调用方每拍传入） |
| `0 = disabled` | 已统一：`thresh_i_sep_` / `max_rate_out_` / `d_filter_Tf_`（PID）、`Tf=0` → LPF 直通 |
| ⚠ `0` 语义未统一 | **限幅类**字段（`limit_out_` / `limit_i_`）为 `0` 时是「钳死到 0」而非「不限幅」；定案见 `../roadmap.md` §6 D-1 |
| dt 守卫 | **仅 PID 内部**有：`dt <= 0 或 dt > 0.5` → 强制 `1ms`；LPF / Ramp / SmoothPlanner **无守卫**，调用方保证 `dt > 0` |
| 限幅 | 对称 `clamp(x, ±limit)`；非对称限幅为候选 A3（未实现） |
| 状态注入 | `LPF` / `Ramp` / `SmoothPlanner` 有 `set_state(x)`（bumpless transfer）；`PID` 暂无（候选 A2，未实现）；`Deadzone` 无状态，不需要 |
| 复位语义 | `reset()` 只清运行时状态，不动配置；`PID::reset()` 不清 ramp/LPF 配置（构造时固定） |

## 组件索引

| 组件 | 契约 | 上游血缘 | 备注 |
|---|---|---|---|
| `PID` | [pid.md](pid.md) | lunokhod wheel → cyclotron/foc → ctlkit | 候选清单 A/B/C/D 的主对象 |
| `LPF` | [lpf.md](lpf.md) | 同上 | PID 的 D 项滤波、SmoothPlanner 的两级圆角共用 |
| `Ramp` | [ramp.md](ramp.md) | 同上 | PID 输出斜坡、SmoothPlanner 限速共用 |
| `SmoothPlanner` | [smooth_planner.md](smooth_planner.md) | 同上（`set_state` 由 cyclotron 侧补齐） | 对齐/模式切换需要 bompless |
| `Deadzone` | [deadzone.md](deadzone.md) | cyclotron/foc（源自 lunokhod） | 唯一无状态组件；软/硬两模式；`range<=0` 直通 |
