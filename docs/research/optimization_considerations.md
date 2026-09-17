---
class: fact
generated: false
---
# PID 优化考量（未来优化候选清单）

> 针对本库 `ctl::PID`（库化前 `foc::algo::PID`）的优化候选，按"借鉴 GitHub 库 → 补全工业级能力 → 健壮性 → 诊断性"组织。
> 每条含：动机 / 参考来源 / 改动建议（签名或伪码）/ 风险与注意 / 优先级（P0=建议近期做, P1=有场景再做, P2=可选）。
> 落地时同步更新本库 `docs/spec/pid.md`（并参照 `docs/design/pid_config_and_ports.md`），保持"源码 + spec = 唯一事实源"。

---

## A. 借鉴 GitHub 库的接口点

### A1. 外部微分注入（dinput / measurement derivative override）

- **动机**：本地 D 项 = `-kd·(measure-measure_prev)/dt`，是对测量的**环内二次差分**。若系统已由观测器/状态估计器提供高质量的导数（如 `angle_tracking::vel_`），重复差分只会放大噪声并引入 LPF 滞后。
- **参考来源**：GitHub `calc(input, optional<float> dinput = {})`。
- **注意差异**：GitHub 的 dinput 语义是"替代测量微分"，但其 D 项仍含设定值差分 `(target-pre_target)/Ts`，阶跃仍有冲击；本地是微分先行，注入点应设计为**直接替代 d_term_raw**，保持无冲击特性：

```cpp
// 方案 a（推荐，签名不变）：注入到 calc 的可选参数
float calc(float cmd, float measure, float dt, const float *measure_dot = nullptr);
// 内部：
//   d_term_raw = measure_dot ? -cfg_.kd_ * (*measure_dot)
//                            : -cfg_.kd_ * (measure - measure_prev_) / dt;

// 方案 b：setter（适合"每拍先设导数再 calc"的写法）
void set_measure_dot(float dot);   // 内部 mark 有效，calc 内消费一次后失效
```

- **场景**：速度环（用 `angle_tracking::vel_`）、或电流测量已由高带宽观测器滤波时；避免 D 项 LPF 相位滞后。
- **风险**：外部导数必须与 measure 同源同量纲；相位/标定不一致会引入错误反馈。方案 a 的指针参数破坏 ABI（本项目内无碍）；方案 b 存在"设了忘传"的陈旧值风险——建议内部记录"本拍是否注入"，一帧消费一次。
- **优先级**：P1（有速度环/观测器需求时做）。

### A2. 积分状态注入（set_integral / bumpless transfer 一致性）

- **动机**：LPF/Ramp 都有 `set_state(x)` 支持无扰切换，唯独 PID 的 `integral_` 不可注入——内部设计不一致。从开环/前馈切入闭环、或控制器输出被外部改写后再交还时，积分从 0 起步会造成输出突跳。
- **参考来源**：GitHub `set_sum_error(float)`（注释即"用于高自由度积分限幅"）。
- **改动建议**：

```cpp
void set_integral(float x) { integral_ = constrainf(x, cfg_.limit_i_); }  // 保持不超限幅
// reset() 语义不变；可再提供 integral() getter 配合（见 D1）
```

- **场景**：电流环使能瞬间（先让输出 = 当前占空比对应的电压估计，再切入闭环）；模式切换、级联外环切内环目标。
- **风险**：注入值必须与环路当前工作点一致，否则造成新的冲击；建议配合 A1/B1 一起做成"完整的工作点注入口"。
- **优先级**：P0（改动 3 行，补全既有 set_state 设计语言）。

### A3. 非对称独立 ± 限幅

- **动机**：本地 `constrainf(x, limit)` 是对称限幅。FOC 中 dq 电压受母线电压/调制比约束本身近似对称，但**再生工况**（母线被泵高需降额）、单象限拓扑、或输出侧被非对称约束时，对称限幅过保守或不够。
- **参考来源**：GitHub `sum_error_limit_p/n`、`output_limit_p/n`（std::optional，独立上下限）。
- **改动建议**：config 扩展 + 兼容默认（`<=0` 仍表"不限/禁用"，保持 0 默认语义）：

```cpp
struct PIDConfig {
    // ...现有字段不变...
    float limit_out_p_ = 0;   // 0 = 不限（注意与现 limit_out_ 的兼容策略）
    float limit_out_n_ = 0;
    float limit_i_p_   = 0;
    float limit_i_n_   = 0;
};
// 兼容：若 limit_out_>0 且 p/n 未设 → 回落对称限幅；否则用非对称
```

- **风险**：改动量最大（config 结构 + constrainf 需换成 clamp_hi/clamp_lo 两个原语），且要定"新旧字段同时存在的回落规则"，易引入歧义——**建议等真正遇到不对称工况再落地**，否则维持对称更简洁。
- **优先级**：P1。

---

## B. 补全工业级抗饱和能力

### B1. 基于输出饱和的 back-calculation / 条件积分（当前最大算法短板）

- **动机**：当前抗饱和 = 静态 clamp（integral 预 clamp ±limit_i_ + 输出 clamp）。若 `limit_i_` 取值接近/大于 `limit_out_`，饱和期间积分仍会涨到 limit_i_，退出饱和时输出迟滞/超调；本质是"不知道输出真的被限住了"。GitHub 增量式更糟（内部累加器无上限）。
- **参考来源**：工业实践（back-calculation / anti-windup with tracking / conditional integration）。
- **改动建议**（二选一或组合）：

```cpp
// 方案 A：条件积分（最简单）——输出饱和且 I 与饱和同向时，本轮不积分
float output_unclamped = p_term + integral_ + d_term;
bool sat_hi = output_unclamped >  cfg_.limit_out_;
bool sat_lo = output_unclamped < -cfg_.limit_out_;
if (!(sat_hi && integral_ > 0) && !(sat_lo && integral_ < 0)) {
    integral_ = i_term_temp;   // 积分分离条件照旧保留
}

// 方案 B：back-calculation —— 饱和时把"被截断量"按比例回灌到积分
//   e_windup = output_unclamped - clamp(output_unclamped)
//   integral_ -= Kb * e_windup * dt;    // Kb 常用 1/Ti 量级，建议做成 config 项或固定经验值
```

- **注意与现有机制叠加**：积分分离（thresh_i_sep_）管"大误差起步"，back-calculation 管"输出饱和"，两者正交，可共存。
- **风险**：方案 B 的 Kb 是新增调参自由度；方案 A 需小心与"输出斜坡"的交互——斜坡限住的是变化率而非幅值，饱和判定应以 **clamp 后、斜坡前**的量为准（当前代码正是该顺序，可直接复用 `output_unclamped` 计算点）。
- **优先级**：P1（若实测出现过调/饱和迟滞再上；P0 不必要）。

### B2. 积分分离阈值迟滞（防抖动）

- **动机**：`thresh_i_sep_` 是单阈值：误差在阈值附近抖动 → 积分"冻/解冻"频繁切换，积分轨迹出现小锯齿。
- **改动建议**：双阈值（进入冻结用 `thresh_i_sep_`，解除用 `0.7×thresh_i_sep_` 之类），需新增一个状态位 `i_frozen_`。
- **风险**：新增状态 → reset 要一并清；行为变化需回归测试。
- **优先级**：P2。

---

## C. 健壮性

### C1. NaN / 非法输入防护

- **动机**：`constrainf(NaN)` 的两个比较均 false → 原样返回 NaN。一旦某拍 measure/cmd 为 NaN（ADC 异常、计算溢出），`integral_ = NaN` **永久锁死**，只能 reset 恢复；输出 NaN 还会向下游（SVPWM/占空比）传播。GitHub 库同样有此问题。
- **改动建议**：

```cpp
// calc 入口：
if (!(error <= 3.4e38f)) { /* isnan 判断（避免引入 <cmath> 可用位运算/自实现）*/ 
    // 策略一（推荐）：本拍跳过积分与 D 更新，仅透传 p_term 或上一拍输出
    // 策略二：直接返回 0 并置 FAULT 标志（配合上层保护）
}
// constrainf 内部同理：val != val → 返回 0 或 limit
```

- **注意**：嵌入式避免 `<cmath>`（当前 `fabs` 是自实现），isnan 可用 `x != x` 或 `fabs(x) <= 3.4e38f` 技巧；需定"NaN 时输出什么"的语义（保持上一拍输出 vs 归零），取决于环路保护逻辑，需与 foc 上层（过流/故障）对齐。
- **优先级**：P0（3~5 行，直接消除"一次异常永久锁死"类故障）。

### C2. dt 守卫策略可配置化

- **动机**：当前 `dt>0.5 → 0.001` 的静默替换：保护了除零，但在**故意低环率**（如 1Hz 周期巡检）场景会引入 1000× 的模型偏差，且无任何提示。
- **改动建议**：config 增 `max_dt_`（默认 0.5 保持行为）；或 dt 超界时返回上一拍输出 + 置位一个可查询的 `clk_err_` 标志，由上层决定策略。
- **优先级**：P2。

---

## D. 可观测性 / 调参辅助

### D1. 状态 getter（诊断 + 未来自整定）

- **动机**：integral_、d_term、error 全私有且无读取口 → 无法观测各分量贡献，示波器/上位机调参、自动整定（如 relay / ZN）都无从下手。
- **改动建议**：

```cpp
struct PIDState { float error, p_term, i_term, d_term, integral, output; };
PIDState snapshot() const;   // 或逐字段 const getter
```

- **注意**：snapshot 需在 calc 内缓存最近一拍的 p/i/d（目前是局部变量，需提升为成员或由 calc 填充一个 out 参数）。
- **优先级**：P1（要上自整定/上位机调试时 P0）。

### D2. 在线调参接口

- **动机**：本地 cfg_ 构造后不可变；跑机调参（KI/KD 在线整定）需要运行时改参口。
- **改动建议**：`void set_gains(kp, ki, kd)`（与 D1 配套），或直接把 cfg_ 暴露 const 引用 + 原子替换（RT 场景注意读改写竞争）。
- **优先级**：P1（与 D1 配套做）。

### D3. 饱和 / 限幅状态上报（可观测性，也是 B1 的公共前置）

- **动机**：`calc()` 里有**两处钳位**——`constrainf(i_term_temp, cfg_.limit_i_)`（积分限幅）与
  `constrainf((p_term + d_term + integral_), cfg_.limit_out_)`（输出限幅）——**都不上报**。
  外部只看得见「被削之后的值」，于是**分不清两种完全不同的处境**：
  「环路恰好工作在这个输出」vs「控制器已经尽力到顶了」。

  这正是两类消费者共同要的第一手事实：
  ① **PID 调参**：输出长期贴顶 = 增益不足 / 负载过重；
  ② **底盘侧残差监测**：`目标 − 实测` + `努力度` + `饱和标志` 三元组才能**区分打滑与堵转**
  （只有速度分不开：堵转与空转打滑在转速上都能是「跟不上」）。

- **与 D1 的区别（别合并）**：**D1 交「值」，D3 交「这个值是不是被削过」**。二者互补，不重复。
- **与 B1 的关系**：B1（条件积分 / back-calculation）**必须**先算出 `output_unclamped` 才能判饱和 ——
  **D3 是 B1 的免费副产品**；反过来 D3 也是 B1 落地前就能独立交付的观测能力。
  建议 D3 与 B1 同期，或 D3 先行（它不改行为，只加出口，风险更低）。

- **实现位置（精确）**：在 `constrainf((p_term + d_term + integral_), cfg_.limit_out_)` **之前**引入未钳位量：

```cpp
const float output_unclamped = p_term + d_term + integral_;
const bool  out_saturated = (output_unclamped >  cfg_.limit_out_) ||
                            (output_unclamped < -cfg_.limit_out_);
float output = constrainf(output_unclamped, cfg_.limit_out_);
```

  **饱和判定必须以「clamp 后、斜坡前」的量为准** —— 与 B1 的注意事项同一条理由：
  斜坡限的是**变化率**，不是幅值。

- **改动建议**：

```cpp
struct PIDFlags {
    bool out_saturated;   // 输出被 limit_out_ 钳位（控制器已尽力到顶）
    bool i_saturated;     // 积分被 limit_i_ 钳位（windup 状态，恢复滞后的根因）
};
PIDFlags flags() const;   // 或逐字段 const getter
```

- **只报「钳位」，**不报**「斜坡」与「积分分离」**：
  `ramp_out_` 削的是变化率、`thresh_i_sep_` 冻结积分 —— **两者都是设计意图**（故意的慢 / 故意的冻结），
  不是饱和。把它们一起报出来只会当噪声，把真饱和淹没。

- **⚠ 前置：0 语义必须先定案**：`PIDConfig::limit_out_` 默认 `0.0f`，而 `constrainf(x, 0)`
  把输出**钳死到 0** → 「输出饱和」标志在默认配置下**恒为 true**。
  而同一结构体里 `thresh_i_sep_ <= 0` / `max_rate_out_ <= 0` / `d_filter_Tf_ = 0` 走的却是
  **0 = disabled** —— **两套 0 语义共存**（lunokhod `TODO.md` P11 就是这一条）。
  **D3 落地前必须先定 0 语义**，否则标志会骗人。

- **风险**：新增状态 → `reset()` 必须一并清（照 WHEEL_LESSONS「状态写回铁律」）；
  `calc()` 每拍多存 2 个 bool（可打包进 1 byte，RT 无碍）；
  `PIDFlags` 默认初始化必须给（`= false`），否则首次 `flags()` 读到垃圾值。

- **落地位置（⚠ 上游库定案后已变更）**：`reference/` 下两份是 **本地保留的只读归档快照**（不随仓库分发），不落地。
  落地目标是 **上游库 ctlkit**（`../../inc` + `src/pid.cpp`）；下游消费副本为
  `cyclotron/foc/inc/foc/algo/pid.hpp` 与 `lunokhod/actuator/wheel/inc/pid.hpp`，由库统一演化后同步
  （策略见库 `docs/roadmap.md` §6 D-6）。落地时同步本库 `docs/spec/pid.md` 与 `docs/design/pid_config_and_ports.md`。

- **优先级**：**P1**（与 B1 / D1 同期）；若先在底盘侧做残差监测，则**升 P0**。

---

## E. 架构性前瞻（暂不建议，记录在案）

- **增量式(delta)模式**：GitHub 双模式之一，适合外环/无扰切换。FOC 电流环不需要（输出需绝对电压、积分需显式可复位）。**若未来新增位置/速度外环**，可参考其 delta 公式，但务必补条件积分（其原版长期饱和会 windup）。
- **target 内置于控制器**（SetTarget 风格）：本地 calc(cmd, measure, dt) 的函数式风格对级联环路更干净，不建议改。
- **header-only / 模板化 / optional 化**：与嵌入式纯 float 目标冲突，不建议。
- **固定 Ts 构造**：与"多环共用、变周期"的现实冲突，不建议。

---

## 优先级汇总

| 编号 | 优化项 | 优先级 | 改动量 | 依赖 |
|---|---|---|---|---|
| A2 | 积分状态注入 set_integral | **P0** | ~3 行 | — |
| C1 | NaN 防护 | **P0** | ~5 行 | 上层故障语义 |
| A1 | 外部微分注入 | P1 | ~10 行 | 观测器可用时 |
| B1 | back-calculation/条件积分 | P1 | ~10 行 | 实测触发时 |
| A3 | 非对称 ± 限幅 | P1 | 中 | 遇到不对称工况时 |
| D1 | 状态 getter/snapshot | P1 | 中 | 调参/自整定需求 |
| D2 | 在线调参 | P1 | 小 | D1 |
| **D3** | **饱和/限幅状态上报** | **P1** | **小** | **0 语义定案；与 B1/D1 同期（D3 是 B1 的副产品）** |
| B2 | 积分分离迟滞 | P2 | 小 | — |
| C2 | dt 守卫可配置 | P2 | 小 | — |

**推荐落地顺序**：C1 + A2（健壮性 & 一致性，低风险）→ A1/D1/D2（按需）→ B1/A3（实测驱动）→ B2/C2（打磨）。

**D3 的位置说明**：它同时是「可观测性」与「抗饱和」的公共前置 ——
若底盘侧先做残差监测（lunokhod `TODO.md` P19），D3 应提前到 D1 之前；
若走调参/自整定路线，则与 D1 同批。
