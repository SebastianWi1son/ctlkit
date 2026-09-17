---
class: fact
generated: false
---
# 上游基础算法优化概念蓝图（PID 优先）

> 回答一个问题：把 `cyclotron/foc` 的基础算法组件（PID / LPF / Ramp）从"现状 + 候选清单"
> 演进成一个什么样的体系？本蓝图给出 **分层目标架构、主流库对照、里程碑、关键设计决策、验证策略**。
>
> 输入：`docs/research/comparison_report.md`（现状）、`docs/research/optimization_considerations.md`（候选 A~E）
> 调研：主流嵌入式/工业 PID 实现（ArduPilot、SimpleFOC、ODrive、VESC、STM32 MCSDK、MathWorks 等，见 §2）
> 定位：本库（ctlkit）自 v0.0.1 起为 **唯一事实源**——算法源码在 `../inc` + `src/`，
> 行为契约在 `docs/spec/`；`cyclotron/foc` 与 `lunokhod/actuator/wheel` 退化为 **下游消费副本**（同步策略见 §6 D-6）。
> 设计稿：`docs/design/pid_config_and_ports.md`（配置分组 + 端口结构，M0~M2 落地）。

---

## 1. 现状定位（一句话）

库内 `ctl::PID`（v0.0.1 库化自 `foc::algo::PID`，行为逐字未变）已具备 **SimpleFOC 级别以上、ArduPilot 级别以下** 的算法内核：

| 已有（领先多数库） | 缺口（相对业界实践） |
|---|---|
| 微分先行（无设定值冲击，SimpleFOC/ODrive 均为误差微分） | 前馈通道（FF / D-FF，ODrive/AC_PID 标配） |
| 梯形（Tustin）积分（与 SimpleFOC 同款） | 目标值 LPF（AC_PID FLTT / ODrive input_filter） |
| D 项 LPF（SimpleFOC 无） | 动态抗饱和：条件积分 / back-calculation（AC_PID / Simulink 标配） |
| 积分分离（特色，国内嵌入式惯例） | NaN 防护（AC_PID 有） |
| 输出斜坡 + dt 显式传递 + dt 守卫 | 状态注入口（set_integral / bumpless，两边都有缺口，已列 A2） |
| 状态全私有、config 注入、无 STL、纯 float | 可观测性（无任何 getter / log 结构，已列 D1/D3） |

**结论**：不做推倒重来。内核公式（微分先行 + 梯形积分 + 分离 + LPF + 斜坡）保持不变，
蓝图的重点是**补齐"注入口、观测口、抗饱和升级、前馈/目标滤波"四类能力**，并以
"0 语义统一"作为一切扩展的前置工程。

---

## 2. 主流库横向对照（调研摘要）

| 特性 | ctlkit（原 foc::algo） | Liu-Curiousity | SimpleFOC | ODrive | ArduPilot AC_PID | MathWorks/Simulink |
|---|---|---|---|---|---|---|
| 积分 | 梯形 | 矩形/增量 | 梯形 | 欧拉 | 欧拉 | 可选（连续域） |
| D 项 | **微分先行 + LPF** | 误差微分 | 误差微分 | 无 D（P/PI 级联） | 误差微分 + FLTD + Notch | 可选滤波 |
| 抗饱和 | 静态 clamp ×2 + 分离 | 静态 clamp | 静态 clamp | clamp + 带宽法自动定增益 | **条件积分（外部 limit 反馈）** + IMAX | **clamping / back-calculation(Kb)** |
| 前馈 | — | — | 环外加（ff 字段） | **逐级 FF**（vel_ff/current_ff/电压 FF） | **FF + D_FF**（目标及其微分） | 前馈块 |
| 目标滤波 | — | — | — | 2 阶 input filter / 轨迹规划 | **FLTT 目标 LPF** | 参考滤波（2-DOF） |
| 输出整形 | Ramp + 限幅 | 限幅 | Ramp + 限幅 | 各级限幅 + vel_limit | **SMAX slew limiter + PDMX（PD 和限幅）** | 限幅 |
| 状态注入 | — | set_sum_error | — | — | **set_integrator + relax_integrator** | 跟踪模式 |
| NaN 防护 | — | — | — | — | **isfinite → 返回 0** | — |
| 观测 | — | 公开字段 | 公开字段 | 上位机遥测 | **_pid_info 全量 log 结构体** | 示波 |
| 在线改参 | — | 公开字段 | 公开字段 | 运行时配置 | AP_Param 运行时 + save_gains | 运行时 |
| 自整定 | — | — | 调参指南 | **由带宽自动推导电流环增益** | autotune（继电/步进混合） | — |

要点摘录（细节见文末参考）：

- **SimpleFOC `PIDController`**：与本地上位结构几乎同款（Tustin、integral clamp=output limit、
  output ramp、自适应 dt 且 dt 守卫同为 `dt<=0 || dt>0.5 → 1e-3`）。但 D 项是裸误差差分、
  无 D-LPF、无 NaN 防护——本地在其之上。**它证明了本地的 dt 守卫数值与业界惯例一致，无需改。**
- **ODrive**：架构启示大于算法启示——三级级联（P 位置 / PI 速度 / PI 电流）+ **每级前馈** +
  输入端 2 阶滤波或轨迹规划，且**电流环增益由 `current_control_bandwidth` 自动推导**。
  "外环做规划与滤波、内环保持简单快速"是级联系统的正确分工。
- **ArduPilot AC_PID**（功能最全的嵌入式参考）：
  1. `update_all(target, measurement, dt, limit, ...)` 的 **`limit` 布尔入参**——由*调用方*
     （执行器饱和/电压饱和检测）反馈"本拍是否受限"，条件积分实现为
     *"受限时积分只许缩、不许涨"*。**这是比本地"内部猜饱和"更强的抗饱和形式。**
  2. 目标 LPF（FLTT）/ 误差 LPF（FLTE）/ D 项 LPF（FLTD）/ Notch 四级滤波位。
  3. FF（kff·target）与 D_FF（kdff·目标微分），即 **2-DOF 化的前馈实现**。
  4. SMAX slew limiter：快速瞬态时按比例削减 P+D（防高频自激）；PDMX：P+D 和限幅。
  5. `set_integrator(x)`（注入并 clamp）与 `relax_integrator(x, dt, tau)`（**按时间常数
     渐进收敛到期望积分**——比硬注入更平滑的 bumpless 手段）。
  6. NaN/Inf 输入 → 返回 0；`_pid_info` 全量观测结构体与算法同源同拍。
- **MathWorks/Simulink**：工业界把抗饱和收敛为两种——**clamping** 与
  **back-calculation（跟踪增益 Kb）**；2-DOF PID（对 P/D 的设定值加权 b、c）是"目标滤波
  + 微分先行"的统一理论框架。可作为术语与公式基准，不必照搬实现。
- **VESC**：电流环 PI 的抗饱和把"母线电压/调制比限制后的实际可输出电压"回灌积分钳制
  （饱和补偿思路，与 B1 同类）；**STM32 MCSDK** 的 PI 为内置积分限幅（社区讨论确认无动态
  抗饱和）——两者说明静态 clamp 在电机厂商中仍是主流，动态抗饱和是**差异化增强项**而非必修。

---

## 3. 目标架构：四层体系

```
┌────────────────────────────────────────────────────────────────────┐
│ L3 系统层（级联与整定，远期）                                          │
│   级联协议(电流→速度→位置) · bumpless transfer 流程 · 增益调度        │
│   (i_scale/pd_scale) · 继电反馈自整定(Åström–Hägglund, 远期)          │
├────────────────────────────────────────────────────────────────────┤
│ L2 增强层（控制品质，按需启用，默认全 0/关闭）                          │
│   条件积分(外部 limit 反馈) · back-calculation(Kb,可选) · 目标 LPF     │
│   前馈通道(FF / D_FF) · 2-DOF 设定值加权 b(记录在案)                   │
│   非对称 ± 限幅(A3) · 积分分离迟滞(B2)                                 │
├────────────────────────────────────────────────────────────────────┤
│ L1 核心层（公式冻结区——回归测试保护）                                  │
│   PID：微分先行 + 梯形积分 + 积分分离 + D-LPF + 输出斜坡 + dt 守卫      │
│   注入口：set_integral · set_measure_dot(外部微分)                     │
│   观测口：PIDState snapshot · PIDFlags 饱和标志                        │
├────────────────────────────────────────────────────────────────────┤
│ L0 原语层（已有 + 小幅加固）                                           │
│   LPF · Ramp · constrainf(对称) → clamp(v, hi, lo)(非对称)             │
│   NaN 守卫(isnan 技巧) · [[nodiscard]] · (可选)Notch/2阶滤波           │
└────────────────────────────────────────────────────────────────────┘
```

分层规则：

- **L1 公式冻结**：内核离散公式不再改动（除 NaN 守卫与注入口），所有变更配黄金向量回归测试。
- **L2 全部"默认关闭"**：沿用本项目 `0 = disabled` 的设计语言，新特性不启用时行为与旧版逐位一致。
- **L3 不进 PID 类**：级联、调度、自整定以"外部协调器 + PID 的新注入口/缩放钩子"实现，
  保持 PID 单一职责。
- 每一层的每个特性都带**观测出口**（借鉴 AC_PID：算法与 log 同源同拍）。

---

## 4. 现有候选清单 → 架构映射

| 候选（见 research/optimization_considerations.md） | 层 | 里程碑 | 蓝图补充 |
|---|---|---|---|
| C1 NaN 防护 | L0/L1 | **M0** | 采用 AC_PID 语义：非有限输入 → 本拍返回 0（或保持上一拍输出，见 §6 D-5） |
| A2 set_integral 注入 | L1 | **M0** | 注入值 clamp 到 limit_i_；对齐 LPF/Ramp 的 set_state 设计语言 |
| 0 语义定案 | 全局 | **M0**（前置） | 见 §6 D-1，是 D3/A3/L2 一切扩展的开关语义基础 |
| D3 饱和/限幅标志 PIDFlags | L1 | **M1** | 同时把 `out_saturated` 以**入参形式回供**（→ F3 条件积分的接口雏形） |
| D1 PIDState snapshot | L1 | **M1** | calc 内缓存本拍 p/i/d/error；log 结构体形态见 §6 D-4 |
| D2 在线调参 set_gains | L1 | **M1** | 原子性注意：增益成组替换，禁止半拍新半拍旧 |
| A1 外部微分注入 | L1/L2 | **M2** | 方案 a（calc 可选参数）优先；注入直接替代 d_term_raw，保持无冲击 |
| B1 抗饱和升级 | L2 | **M2** | 两步走：先 **条件积分(外部 limit 反馈，AC_PID 式)**，back-calculation(Kb) 仅在实测不满意后追加 |
| A3 非对称 ± 限幅 | L0/L2 | **M3** | 原语层先加 clamp(v,hi,lo)，config 回落规则按原文档 |
| B2 积分分离迟滞 / C2 dt 守卫可配置 | L1/L2 | **M3** | 打磨项，行为变化需回归测试 |
| E 架构性前瞻（增量式/模板化/固定 Ts） | — | 不做 | 维持原文档结论；增量式仅在出现位置/速度外环需求时按 F4 复评 |

## 5. 调研引入的新候选（原清单之外）

| 编号 | 特性 | 来源 | 建议 |
|---|---|---|---|
| **F1** | **前馈通道 FF / D_FF**（目标值与目标微分直接进输出） | AC_PID、ODrive 逐级 FF | **P1、M2**。FOC 电流环的电压前馈（反电动势/交叉耦合解耦项）是最自然入口；建议做成 calc 的可选参数或独立 `set_ff()`，与 I/D 解耦、不进积分 |
| **F2** | **目标 LPF**（参考平滑，防目标阶跃直击环路） | AC_PID FLTT、ODrive input_filter | P1、M2。与现有 Ramp 正交：Ramp 限**输出**变化率，目标 LPF 限**输入**带宽；位置/速度外环优先受益 |
| **F3** | **条件积分 + 外部 limit 反馈**（`calc(..., bool limit)`：受限时积分只缩不涨） | AC_PID | **P1、M2**（与 B1 合并为"抗饱和升级"的两条路径，优先级高于 back-calculation）；执行器饱和信息（如母线电压限制后的可输出电压）由上层提供，比 PID 内部猜更准 |
| **F4** | slew limiter（SMAX，瞬态按比例削 P+D）与 PDMX（P+D 和限幅） | AC_PID | P2、记录在案。电流环带宽高、无明显结构谐振场景收益低；云台/机械臂类负载再评估 |
| **F5** | **增益调度钩子 i_scale / pd_scale** | AC_PID | P2。与 D2 在线调参同批设计（单参数动态缩放 vs 成组替换），是未来自适应/调度系统的预留接口 |
| **F6** | relax_integrator（按时间常数 τ 渐进收敛到期望积分值） | AC_PID | P2。bumpless transfer 的"软"方案，与 A2 的"硬"注入互补；模式切换/恢复场景用 |
| **F7** | 全量观测结构体（target/error/P/I/D/FF/flags 同拍打包） | AC_PID `_pid_info` | P1、M1。D1/D3 的组织形态：一个 POD 结构体一次 memcpy 上传上位机，比逐字段 getter 更适合 DMA/日志通道 |
| **F8** | 继电反馈自整定（bang-bang 激励 → 振荡周期/幅值 → 初值增益） | ESPHome/Åström–Hägglund、ODrive 带宽法 | P2/远期。依赖 D1 snapshot + 在线调参就绪；电流环可走 ODrive 式"由目标带宽解析推导"路线，速度环走继电法 |

---

## 6. 关键设计决策（决策点清单）

- **D-1 `0` 语义统一 —— 已定案并落地（Unreleased）**
  分类：① **`0` = 不限幅**（`limit_out_` / `limit_i_`，原为“钳死到 0”）；② **`0` = 关闭特性**（`thresh_i_sep_` / `max_rate_out_`，PID 层）；
  ③ **`0` = 自然退化值**（`d_filter_Tf_=0` 直通、`Deadzone.range=0` 直通）。原语层不动：`Ramp(0)` 仍是冻结，PID 关闭斜坡时在构造期归一化。
  ⚠ 行为变更点：`limit = 0` 的既有配置（包括 `PIDConfig{}` 全零——现在它是“无限制”，不再是“输出恒 0”）。下游迁移时需检查是否有依赖旧行为的配置。
- **D-2 抗饱和路线**：条件积分（外部 limit 反馈）优先于 back-calculation。理由：
  ① 无新增调参自由度（Kb 需整定）；② 与现有"积分分离/预限幅"正交叠加容易；③ AC_PID 在
  高动态飞行控制上已验证该形式足够。back-calculation 保留为 L2 可选项，仅在条件积分
  实测恢复迟滞明显时启用。
- **D-3 前馈形态**：前馈是"通道"不是"算法"——进 `calc` 的可选参数（与 A1 的
  measure_dot 同一扩展方向），不进积分、不进限幅（或仅受输出限幅约束），disable 时零开销。
- **D-4 观测形态**：`PIDState`（值）+ `PIDFlags`（被削与否）分两个 POD 结构，
  不合并（遵循原文档 D3 的"值 ≠ 是否被削"原则）；getter 与结构体二选一以结构体为主。
- **D-5 NaN 时输出语义**：默认方案 = **本拍跳过状态更新、返回上一拍输出**（对 FOC 更安全：
  电压指令不跳变），置 fault 标志由上层决定是否切断；备选"返回 0"会令输出瞬间跳零，
  需上层故障逻辑配合。与 foc 上层过流/故障语义对齐后定案（原文档 C1 的开放问题）。
- **D-6 下游副本同步**：本库（ctlkit）为 **上游唯一事实源**；`cyclotron/foc`（原 `foc::algo`）与
  `lunokhod/actuator/wheel`（原始血缘）退化为 **下游消费副本**：拷贝 `../inc` + `src/*.cpp`，
  拷贝文件头保留来源戳 `// from ctlkit vX.Y.Z`，批次迁移后跑下游 diff 校验。
  两下游 **不得各自继续演化**（此前的双活副本已造成血缘混乱——本库正是为终结它而建）。
  迁移同时结清 `FOC_MATH_SPEC` §10 的 P5（namespace 统一为 `ctl`）与 P1（0 语义）。
- **D-7 配置/端口结构（M0~M2 的落地形态）**：PID 配置按 `gains/limits/tuning` 分组、
  每拍输入收进 `PIDPorts`（`calc(..., const PIDPorts * = nullptr)`，签名从此冻结）。
  完整草案、备选方案取舍、边界判据（什么进 PID、什么外置）与迁移对照见
  `docs/design/pid_config_and_ports.md`。

## 7. 里程碑

| 里程碑 | 内容 | 出口标准 |
|---|---|---|
| **M0 加固** | 0 语义定案 → C1 NaN 防护 + A2 set_integral + `[[nodiscard]]` | 黄金向量回归全绿；NaN 注入用例证明可恢复 |
| **M1 观测** | D3 PIDFlags + D1 PIDState + F7 观测结构体 + D2 set_gains | 上位机可同拍看到 P/I/D/饱和标志；改参无半拍混合 |
| **M2 品质** | A1 外部微分 + F3 条件积分（含 B1 评估）+ F1 前馈 + F2 目标 LPF | 饱和恢复无迟滞；前馈使阶跃跟踪误差显著下降；各特性默认关闭时与 M1 输出逐位一致 |
| **M3 场景驱动** | A3 非对称限幅 + B2 迟滞 + C2 dt 可配置 + F5 钩子 | 有真实工况触发才开工；每项独立开关 |
| **M4 前瞻** | 2-DOF 加权 b、back-calculation、F6 relax、F8 自整定 | 仅做预研文档与接口占位，不动 L1 |

依赖链：**0 语义 → M1 观测 → M2 品质**（观测是抗饱和与前馈调参的眼睛）；
M4 依赖 M1+M2 的注入口与调参口就绪。

### 7.1 工时估算

> 假设：熟悉代码库；主机端可跑测试（本库 CMake 已就绪）；硬件台架可用；含编码 + 测试 + 文档同步。
> 量级参考，误差约 ±50%，不要当承诺。

| 包 | 内容 | 工时 |
|---|---|---|
| **地基** | 仓库结构 + oracle + 黄金向量回归骨架 | 4~8h（**已完成**：`oracle/` + `tests/golden/` + `tests/golden_test.cpp`） |
| **M0 加固** | 0 语义定案(2~3h) + NaN 防护(2h) + set_integral(1h) + `[[nodiscard]]` | 6~12h |
| **M1 观测** | PIDFlags(2h) + PIDState(3h) + 观测结构体(2h) + set_gains(2h) | 8~15h |
| **M2 品质** | 外部微分(4h) + 条件积分(5h) + 前馈(4h) + 目标 LPF(3h) + 回归/硬件验证(10h) | 20~30h |
| **M3 场景驱动** | 非对称限幅(6h) + 迟滞(2h) + dt 可配置(2h) + 调度钩子(3h) | 10~15h |
| **M4 前瞻** | 2-DOF(4h) + back-calculation(4h) + relax(3h) + **自整定 25~60h** | 视范围（自整定是独立项目量级） |

**三条结论**：
1. 「应付现有项目」的**轻量包 = 地基 + M0 + M1 ≈ 18~30h（2~3 个整天，业余节奏 2~3 周）**
   —— 恰好也是本文依赖链的头两步，不算绕路。
2. 「完全体」M0~M3 ≈ **40~60h**（5~8 个整天，业余 1~2 个月）。
3. 若含 M4 自整定，总量 **≈ 70~120h**。

**关键提醒**：黄金向量回归骨架（地基）必须随轻量包一起做——没有它，后面每一步都在裸奔改公式。

## 8. 验证与回归策略

1. **黄金向量**：固定输入序列（阶跃/斜坡 + dt 扰动）下逐拍比对。
   **已建立**（`oracle/` → `tests/golden/*.csv` → `tests/golden_test.cpp`，11 个用例，容差逐例推导）；
   当前快照的是**输出**；p/i/d/integral 分量快照待 D1（PIDState）落地后加密。
   任何 L1/L2 改动必须证明"默认关闭 = 旧版逐位一致"。
2. **特性测试**：饱和注入（看恢复拍数）、积分分离边界抖动（B2 前后对比）、NaN 注入
   （3 拍内恢复）、bumpless 切换（注入 vs 不注入的输出跳变）。
3. **硬件验收**（cyclotron 实机）：电流环阶跃 + 速度环打滑/堵转场景
   （D3 饱和标志 + 残差监测是判定依据，对应 lunokhod TODO P19）。
4. **文档同步**：行为契约以本库 `docs/spec/` 为唯一事实源（本库改动先改 spec 再加回归）；
   下游（cyclotron 侧的 FOC_MATH_SPEC 等）改为**引用**本库 spec，不再各自维护公式副本。

## 9. 参考来源

- ArduPilot AC_PID 源码：https://github.com/ArduPilot/ardupilot/blob/master/libraries/AC_PID/AC_PID.cpp
- SimpleFOC PID 实现：https://docs.simplefoc.com/pid_implementation
- ODrive 控制框架：https://docs.odriverobotics.com/v/latest/manual/control.html
- MathWorks 抗饱和（clamping / back-calculation）：https://www.mathworks.com/help/simulink/slref/anti-windup-control-using-a-pid-controller.html
- MathWorks 2-DOF PID（设定值加权）：https://www.mathworks.com/help/control/ug/two-degree-of-freedom-2-dof-pid-controllers.html
- Arduino PID 改进系列（Brett Beauregard，微分先行/无扰调参/积分钳制的经典出处）：http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-introduction/
- ESPHome PID autotuner（继电法实例）：https://api-docs.esphome.io/pid__autotuner_8cpp_source
- 本地已有：`docs/research/comparison_report.md`、`docs/research/optimization_considerations.md`（候选 A~E 原始定义）
- 设计稿：`docs/design/pid_config_and_ports.md`（配置分组 + 端口结构）

## 10. 终点线判定（2026-09-17）

> 终点定义（用户定）：**不打算全部做完 —— 能给下游项目用、不影响未来构建，即结束。**

### 10.1 完整概念实现度（蓝图四层 / 候选清单）

| 层 | 内容 | 状态 |
|---|---|---|
| L0 原语 | LPF · Ramp · clamp（0 = 不限幅）· NaN 守卫 | ✅（Notch/二阶滤波未做，可选） |
| L1 核心 | PID 公式冻结 + 注入口（set_integral / set_gains）+ 观测出口（get_state / status / input_fault） | ✅（缺 A1 外部微分注入 → 随模块 5 的 PIDPorts 引入） |
| L2 增强 | 条件积分 · 前馈 · 目标 LPF · 2-DOF · 非对称限幅 · 分离迟滞 | ⬜ 0%（设计就绪；全部按“0 = 默认关闭、只增不改”预留） |
| L3 系统 | 级联协议 · 增益调度 · 自整定 | ⬜（远期） |

里程碑：M0 ✅ · M1 ✅ · M2 ⬜ · M3 ⬜（场景驱动）· M4 ⬜（前瞻）。
候选清单 17 项完成 6 项（A2 · C1 · D1 · D2 · D3 · D-1），约 1/3。

### 10.2 地基终点线对照（真正的验收标准）

| 地基要求 | 状态 |
|---|---|
| 组件可用且被验证 | ✅ 5 组件 · 黄金 18 例 · smoke · 判别力实测（改坏必红） |
| 行为契约（下游读什么） | ✅ `docs/spec/` 五份 + 全局约定 |
| 下游消费方式 | ✅ 拷贝 inc 与 src 加来源戳 + diff 校验（README「下游消费方式」） |
| 未来只拓展不破坏 | ✅ 机制已定：新特性 = config 新字段（0 = 默认关闭）+ `calc` 尾部默认参数端口；破坏性改动（分组改名 / 0 语义）已在 v0.0.x 付清 |
| 版本与兼容承诺 | ⬜ 建议 v0.1.0 + 一句话政策（minor 只增不改；breaking → major） |
| 许可 | ⬜ 待定 |
| 下游接入演练 | ⬜ 把 cyclotron 侧下游切换一次并跑 diff 校验（下游侧工作，非本库） |

**判定**：库本体已到终点线；剩三项收尾（版本 / 许可 / 接入演练）均为小时级且不阻塞下游试用。
M2 的四个品质特性属于“拓展”——按只增不改的设计随时可后补，不构成终点线的一部分。
