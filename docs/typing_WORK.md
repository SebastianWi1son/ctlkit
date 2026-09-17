---
class: work
generated: false
accepted: false
---
# typing_WORK — 改造缓冲区（只收"需要改动的文件"）

> **类：D 施工单** —— 全部模块敲完并验收后，本文件移出 `docs/`（或标 `accepted: true` 交给门禁拦）。
>
> **规矩**：
> 1. 缓冲区**只装需要改动的代码**。与上游逐字一致、只做机械库化的文件**不进缓冲区**（`deadzone` 就是例子）。
> 2. 每次只产出**一个模块**；模块 = 一个可独立编译 + 跑测试的改动单元。
> 3. 你手敲 `inc/` 与 `src/` 下的**改动**；`tests/` · `examples/` · `oracle/` · `docs/` 由我同步。
> 4. 行为**变了**的改动必须同批同步 `docs/spec/` + 黄金向量；行为**没变**的改动必须证明"黄金回归仍全绿"。

## 1. 流程

1. 我给**模块 N**：改哪些文件、before → after、逐段理由、验证命令、待你拍板的决策点。
2. 你阅读 + 手敲；任何一行可叫我先讲，或直接提改法。
3. 敲完跑验证（编译 + ctest）。
4. 你给我决策结论 → 我同步 spec / 黄金向量 / 测试（逐处给你过目）。
5. 你说「下一个」→ 模块 N+1。

## 2. 模块清单（按推荐顺序；**顺序本身是决策 0**）

| # | 模块 | 改哪些文件 | 改动性质 | 行为变化 | 决策点 |
|---|---|---|---|---|---|
| 1 | ✅ PID 加固三件套（已敲完） | `inc/ctl/pid.hpp` · `src/pid.cpp` | `CTL_NODISCARD`、NaN/Inf 守卫、`set_integral` | 仅"非法输入"路径 | N1~N5 已定案（见 §4） |
| 2 | ✅ `0` 语义统一（已由 AI 执行） | `inc/ctl/pid.hpp` · `src/pid.cpp` · oracle · docs | 限幅类 `<=0 = 不限幅`；斜坡 0=关闭构造期归一化 | ⚠ 变（`limit = 0` 的配置） | 按提案定案（见 §4） |
| 3 | ✅ 配置分组（已完成） | `inc/ctl/pid.hpp` · `src/pid.cpp` | `PIDConfig` 8 字段平铺 → `gains/limits/tuning` 三组 | 不变（逐字等价） | D3-1 下划线 · D3-2 命名 · D3-3 旧名别名 · D3-4 端口同批？ |
| 4 | 观测出口（M1）⬅ 下一个候选（待你定：接手 / 我做） | `inc/ctl/pid.hpp` · `src/pid.cpp` | `PIDState` / `PIDFlags` / `snapshot()` / `set_gains` | 不变（纯新增） | 出参 vs 返回值 · 缓存放哪 |
| 5 | 外部微分注入（M2，A1）**+ `PIDPorts` 首次登场** | `inc/ctl/pid.hpp` · `src/pid.cpp` | `ports.measure_dot` 替代环内二次差分；`calc` 签名冻结 | 默认不变 | 指针 vs setter（原文档方案 a/b） |
| 6 | 条件积分（M2，F3/B1） | `inc/ctl/pid.hpp` · `src/pid.cpp` + 调用点 | `ports.limit`：受限时积分只缩不涨 | 变（饱和路径） | 入口形态确认 |
| 7 | 前馈通道（M2，F1） | `inc/ctl/pid.hpp` · `src/pid.cpp` | `gains.kff/kdff` + `ports.ff/ff_dot` | 变（默认关闭） | FF 与限幅的顺序（设计稿 Q3） |
| 8 | 目标 LPF（M2，F2） | 调用点（复用 `LPF`，**不进 PID**） | 参考平滑外置组合 | 调用点新增 | 复用 `LPF` vs 新组件 |
| 9 | M3 打磨（A3/B2/C2/F5） | 视决策：`inc/ctl/pid.hpp` · `src/pid.cpp`，或抽公共原语头（⬜ 计划） | 非对称限幅 / 分离迟滞 / dt 可配置 / 缩放钩子 | 各项独立 | 逐项定 |
| 10 | M4 前瞻 | — | 2-DOF / back-calculation / relax / 自整定 | — | 仅预研 |

> **端口结构体为什么不在模块 3**：`PIDPorts` 的每个字段都必须被行为实现消费，否则就是"静默无效"的 API。
> 所以它推迟到模块 5（第一个真正消费端口的特性 = 外部微分注入），在那之前 `calc` 签名不动。

## 3. 模块 3：配置分组（✅ 已完成）

- **你**敲了 `inc/ctl/pid.hpp` 的分组结构：`PIDGains` / `PIDLimits` / `PIDTunings` + `PIDConfig` 成员
  `gains_` / `limits_` / `tunings_`（字段名与旧版一致）；
- **我**完成了 `src/pid.cpp` 的 9 个访问点、`tests/` 与 `examples/` 的同步、文档与 CHANGELOG；
- **行为逐字等价**：黄金 16/16 全绿、smoke 全过、0 warning；**未重生成黄金向量**（oracle 未改）；
- 定案：**字段名不改、不留旧路径别名**（`cfg.kp_` 不再编译）；`PIDPorts` 推迟到第一个真正消费端口的模块。

**下一个候选：模块 4 · 观测出口（M1）** —— `PIDState` / `PIDFlags` / `snapshot()` / `set_gains`。
它的决策点（出参形态、缓存放哪、饱和标志怎么报）比模块 3 更值得你亲自定 —— 你说接手我就把
before/after 按前面流程给你；你说「你来」我就直接做。

## 4. 决策记录（边敲边记 —— 只增不改）

| 日期 | 模块 | 决策 | 影响（spec / 黄金向量 / 下游） |
|---|---|---|---|
| 2026-09-17 | 1 | N1(a) 非法输入返回**上一拍输出**（新增 `last_output_`） | `docs/spec/pid.md` 已同步「非法输入行为」；oracle `refctl.py` 同步实现 |
| 2026-09-17 | 1 | N2(a) 自实现 `is_finite`（不引 `<cmath>`；上界 `3.402823466e+38f`） | 与 deadzone 的 `<cmath>` 依赖差异保留（deadzone 决策 D1-1 待定） |
| 2026-09-17 | 1 | N3(a) `set_integral` 注入值 **clamp 到 `limit_i_`** | smoke 新增 `test_pid_set_integral` |
| 2026-09-17 | 1 | N4(a) 用宏 `CTL_NODISCARD`（定义在 `inc/ctl/pid.hpp`） | 实测生效：抓到测试里一处丢弃返回值的调用 |
| 2026-09-17 | 1 | N5(a) 不并入模块 2 | 模块 2 独立进行 |
| 2026-09-17 | 1 | 手敲验收：首轮漏 `last_output_` 的 **3 处写回**（构造初值 / `calc` 写回 / `reset` 清零），smoke 3 项红；补齐后全绿 | 教训：新状态字段必须"构造给初值 + 每拍写回 + reset 清零"三处齐 |
| 2026-09-17 | 2 | 限幅类 `<= 0` = **不限幅**（未采用 sentinel 巨大值）；斜坡"0 = 关闭"在构造期归一化为无上限速率（`Ramp` 自己 0 = 冻结不变）；`set_integral` 同样不限幅 | `docs/spec/pid.md` + `docs/spec/README.md` 全局约定已同步；oracle `clamp()` 与斜坡归一化同步；新增黄金用例 2 个（共 16）；smoke 新增 `test_pid_zero_means_unlimited`；CHANGELOG / roadmap D-1 已更新 |
| 2026-09-17 | 3 | 用户敲头文件分组（`PIDGains`/`PIDLimits`/`PIDTunings` + `gains_`/`limits_`/`tunings_`，字段名不变）；AI 完成访问点与测试/示例同步 | 行为逐字等价（黄金 16/16）；`docs/spec/pid.md` 配置表、设计稿状态行、CHANGELOG 已同步；不留旧路径别名 |
| | | | |
