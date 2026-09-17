---
class: work
generated: false
accepted: false
---
# PID 配置与端口设计（设计稿 v0.1）

> **状态**：**配置分组已落地**（v0.1.0 冻结面）。实际命名以源码为准：`PIDGains` / `PIDLimits` / `PIDTunings`
> + `PIDConfig` 成员 `gains_` / `limits_` / `tunings_`；**字段名与旧版一致、不保留旧路径别名**。
> **支撑类型已拆到 `inc/ctl/pid_types.hpp`**（v0.1.0 冻结面之一；`pid.hpp` 只留类）。
> **端口结构体已落地（模块 5 / A1）**：`PIDPorts` 当前只有 `meas_dot_`（外部微分注入），
> `calc(..., const PIDPorts *ports = nullptr)` **签名自此冻结** —— 后续前馈/条件积分只往 `PIDPorts` 加字段（0 = 缺省 = 旧行为）。
> **性质**：回答「PID 功能不断增加时，配置与接口如何不臃肿」——属于 `docs/design/`（允许过期，
> 实现后把定案沉淀进 `docs/spec/pid.md`）。
> **依据**：`docs/roadmap.md` §2/§5（候选 A~F 与调研）、`docs/research/optimization_considerations.md`。

---

## 1. 问题：三条正在逼近的膨胀线

候选清单（原 A~E + 调研 F1~F8）全部落地后：

1. **`PIDConfig` 会平铺到 20+ 个字段**（现 8 个 + 增益类 2~3 + 非对称限幅 4~6 + 抗饱和 Kb + 目标滤波 + dt 策略 + NaN 策略…），
   平面结构里"哪些属于增益、哪些属于限幅、哪些属于整形"全靠注释区分。
2. **`calc()` 签名会反复变动**：A1 要注入外部微分、F1 要前馈、F3 要外部饱和反馈——
   签名将从 `calc(cmd, measure, dt)` 长成 `calc(cmd, measure, dt, measure_dot, ff, ff_dot, limit, ...)`，
   每个下游调用点每次都要改。
3. **三种不同生命周期的东西混在一起**：
   - 构造期一次定死（增益、限幅、滤波常数）；
   - 每拍变化的输入（测量微分、前馈、饱和反馈）；
   - 内部运行时状态（积分、上一拍误差/测量、滤波器状态）。

> 结论：臃肿的根源不是"字段多"，而是**生命周期与归属没有分开**。解法是把三者拆开，而不是继续平铺。

## 2. 目标与约束

| # | 目标 | 验收方式 |
|---|---|---|
| G1 | `calc()` 签名**从此冻结**——新增每拍特性只改端口结构体，不改函数签名 | 新增一个特性时 diff 不含 `calc` 声明 |
| G2 | 配置**按域分组**，结构与 spec 章节可对应 | 目录式阅读：gains/limits/tuning 三组 |
| G3 | 每拍输入集中在端口结构体，可缺省 | 旧调用 `calc(cmd, m, dt)` 零改动可编译 |
| G4 | 未启用特性时**零行为变化、零额外状态** | 黄金向量逐位一致 |
| G5 | 与在线调参（D2）、观测（D1/D3）、条件积分（F3）天然配套 | M1/M2 验收 |

约束（延续本库既有语言）：C++11；纯 `float`；无 STL / 无堆；ISR 可调用；**不用虚函数与模板多态**（避免间接跳转与代码膨胀）。

## 3. 设计：生命周期三分

```
        调用点                                PID 内部
────────────────────────────────────────────────────────────────
  PIDConfig  ────── 构造期一次 ─────────▶  cfg_（不可变，分组）
  PIDPorts   ────── 每拍可选传入 ────────▶  本拍输入（缺省 = 旧行为）
      calc(cmd, measure, dt, ports?)  ───▶  PIDState / PIDFlags（出参，M1）
                                              State（私有：积分/滤波器/斜坡）
```

### 3.1 结构草案

```cpp
// ---- ① 配置：构造期定死，按域分组（M0 落地）----
struct PIDGains  { float kp, ki, kd, kff, kdff; };              // 增益（含前馈 F1）
struct PIDLimits { float out, out_p, out_n;                     // 输出（A3 非对称）
                   float i, i_p, i_n; };                        // 积分（A3）
struct PIDTuning {
    float thresh_i_sep;      // 积分分离阈值（B2 可加迟滞带）
    float d_filter_Tf;       // D 项 LPF（沿用）
    float max_rate_out;      // 输出斜坡（沿用）
    float target_lpf_Tf;     // 目标滤波？→ 见 §4 边界判据：建议外置，不在此
    float kb;                // back-calculation 跟踪增益（B1，可选）
    float max_dt;            // dt 守卫上界（C2，可选配置化）
};
struct PIDConfig { PIDGains gains; PIDLimits limits; PIDTuning tuning; };
// 兼容性：`PIDConfig cfg = {};` 仍全默认（嵌套聚合零初始化）

// ---- ② 端口：每拍输入，缺省全不启用（M2 逐步启用）----
struct PIDPorts {
    const float *measure_dot;   // A1 外部微分（观测器提供；nullptr = 用环内差分）
    float ff;                   // F1 前馈（直接进输出，不进积分）
    float ff_dot;               // F1 前馈微分（D_FF）
    bool  limit;                // F3 外部饱和反馈（受限时积分只缩不涨）
};

// ---- ③ 出口：观测（M1）----
struct PIDState { float error, p_term, i_term, d_term, integral, output; };
struct PIDFlags { bool out_saturated, i_saturated; };

// ---- 接口 ----
float calc(float cmd, float measure, float dt, const PIDPorts *ports = nullptr);
PIDState snapshot() const;   PIDFlags flags() const;
void set_integral(float x);  void set_gains(const PIDGains &g);   // A2 / D2
```

### 3.2 语义规则

| 规则 | 内容 |
|---|---|
| 缺省即旧行为 | `ports == nullptr` → 与 v0.0.1 逐位一致 |
| 非空即显式 | `ports != nullptr` → 各字段**按字面使用**（`limit=false`、`measure_dot=nullptr` 表示不启用，而不是"未填写"）；调用点用 `PIDPorts p = ctl::pid_ports_off();` 起手填写，避免 0/1 默认值歧义 |
| 只读端口 | `calc` 不修改 `ports` |
| 特性开关 | 延续 `0 = disabled`；限幅类按 D-1 定案改为 `0 = 不限幅`（M0 一次性迁移） |
| 结构体演进 | 加字段 = 兼容（调用点用 `pid_ports_off()` 重新起手）；删/改名 = 破坏，需走 CHANGELOG |
| 不进积分的量 | `ff` / `ff_dot` 只进输出合成；`measure_dot` 只替代 D 项差分 |

### 3.3 构造写法：具名链式设置器（v0.1.1 落地）

字段分成三组后，位置初始化（`PIDConfig{1.0f, 50.0f, 0, 3.0f, 3.0f, 0, 0, 0}`）虽然还能编译（聚合 + 组内顺序 = 老平铺顺序），
但**只能靠数位置读**，且一旦往组中间插字段就**静默错位** —— 与「不许静默吞错」冲突；
C++20 的指定初始化（`.kp_ = 1.0f`）在本库的 C++11 底线不可用。

落地：`PIDConfig` 增**具名链式设置器**，名 = 字段名去掉尾下划线。

```cpp
const PIDConfig cfg = PIDConfig{}.kp(2.0f).ki(50.0f).limit_out(3.0f).limit_i(3.0f);
// 只设需要的字段；其余保持默认（全 0）—— 未设的限幅 = 不限幅（0 语义 D-1）
```

取舍与约束：

- **只加成员函数，不加构造函数**：加构造函数会让 `PIDConfig` 失去聚合性，破坏下游 `foc::Config` 那类嵌套聚合初始化（冻结面）。
  测试里有 `std::is_aggregate<ctl::PIDConfig>` 守卫（C++17 起生效）。
- 返回 `PIDConfig&`（单语句本体，C++11 可用）；本质是逐字段赋值，无运行期开销，可用于 `static`/局部/成员就地构造。
- 链式表达式可直接作聚合初始化器的元素，下游 `Config` 工厂**不必改结构**：
  `foc::Config{ ..., PIDConfig{}.kp(0.5f).limit_out(3.0f), ... }`。
- 表驱动场景（黄金向量按 CSV 参数逐字段填）继续用逐字段赋值；两种写法并存，无优先级。

## 4. 备选方案与取舍

| 方案 | 形态 | 优点 | 缺点 | 结论 |
|---|---|---|---|---|
| **A 平铺扩展** | `PIDConfig` 继续加字段 | 最省事 | 20+ 字段平面；签名反复变；G1/G2 不满足 | ❌ 拒绝（现状路线） |
| **B 端口结构体 + 分组配置** | 本设计 | 签名冻结；配置可读；缺省零成本 | 一次性迁移成本（字段改名/分组） | ✅ **采用** |
| **C policy 模板组合** | `PID<AntiWindup, DerivSource, …>` | 编译期组合、零运行时开销 | 模板膨胀、ISR 调试困难、在线改参别扭、编译时间上升 | ❌ 拒绝（与嵌入式约束冲突） |
| **D 装饰器/组合** | `PIDWithFF(PID&)` 等包装类 | 关注点分离最彻底 | 间接层；状态分散；虚函数或模板传递成本 | ⚠ 仅用于**信号预处理类**（见边界判据） |

### 边界判据：什么进 PID、什么外置

| 特性类型 | 归属 | 理由 |
|---|---|---|
| 改变**反馈/积分语义**（抗饱和、条件积分、外部微分、前馈、非对称限幅） | **进 PID**（配置或端口） | 必须与积分器/输出合成同拍、同源，外置无法正确实现 |
| 只对 cmd/measure 做**信号整形**（目标 LPF、Notch、死区） | **外置独立组件**，调用点串联 | 可自由组合顺序、可单独测试、PID 保持瘦；ODrive 即此模式（input filter 与 controller 分离） |

> 这条判据比"字段数量"更重要：它决定 PID 长期保持小类，而新增能力落在库的其他组件上（`docs/spec/` 增组件而非 PID 增字段）。

## 5. 与候选清单的映射

| 候选 | 落位 | 备注 |
|---|---|---|
| A1 外部微分 | `ports.measure_dot` | 指向观测器输出；`nullptr` 回落环内差分 |
| A2 积分注入 | `set_integral(x)` 方法 | 不进 config；注入值 clamp 到积分限幅 |
| A3 非对称限幅 | `limits.out_p/out_n/i_p/i_n` | 回落规则：`out_p/out_n` 均为 0 → 用 `out` 对称 |
| B1 抗饱和升级 | `tuning.kb` + `ports.limit` | 条件积分优先（外部反馈）；back-calculation 可选 |
| B2 分离迟滞 | `tuning.thresh_i_sep` + 内部状态位 | 迟滞带回差常数建议内置（不新增字段） |
| C1 NaN 防护 | `calc` 入口守卫（无配置项） | 输出策略见开放问题 Q2 |
| C2 dt 上界 | `tuning.max_dt` | 默认 0.5 保持现状 |
| D1/D3 观测 | `PIDState` / `PIDFlags` | 出参或 `snapshot()`；与 `calc` 同拍 |
| D2 在线调参 | `set_gains(gains)` | **成组替换**，避免半拍新半拍旧 |
| F1 前馈 | `gains.kff` + `ports.ff/ff_dot` | 增益在 config，输入在 ports（与 D_FF 同构） |
| F2 目标 LPF | **外置组件**（不进 PID） | 见 §4 边界判据 |
| F3 外部饱和反馈 | `ports.limit` | AC_PID `update_all(..., limit, ...)` 同款 |
| F5 增益调度 | 开放问题 Q5 | `i_scale/pd_scale` 是否入 ports 待定 |
| F6 渐进松弛 | `relax_integrator(x, dt, tau)` 方法 | 与 A2 的硬注入互补 |

## 6. 迁移与兼容

- **推荐路径（M0 一次付清）**：直接落地分组配置（方案 B）。理由：本库处于 `v0.0.x`，尚无行为兼容承诺；
  下游接入本来就发生在 M0（要改 `foc::algo` → `ctl`），一次改净比"双胞胎字段"更干净。
- 迁移对照（示例）：

| v0.0.1 | 目标 |
|---|---|
| `cfg.kp_` | `cfg.gains.kp` |
| `cfg.limit_out_` | `cfg.limits.out` |
| `cfg.d_filter_Tf_` | `cfg.tuning.d_filter_Tf` |
| `cfg.max_rate_out_` | `cfg.tuning.max_rate_out` |
| `pid.calc(cmd, m, dt)` | 不变（`ports = nullptr`） |

- 若需零迁移期：可临时保留旧字段名作为分组字段的**别名**；但匿名 struct/union 别名是非标准扩展
  （GCC/Clang 可，`-pedantic` 警告），不推荐进主干。
- 字段命名去掉尾下划线（POD 不需要成员记号）；此项与分组改名同批完成。

## 7. 验收（各阶段）

| 阶段 | 验收 |
|---|---|
| M0 | 分组配置落地；黄金向量证明"默认值逐位一致"；迁移对照表全绿 |
| M1 | `snapshot()`/`flags()` 与 `calc` 同拍；`set_gains` 成组生效 |
| M2 | 端口逐项启用测试：`measure_dot` 注入后 D 项不再二次差分；`ff` 不污染积分；`limit=true` 时积分只缩不涨 |

## 8. 开放问题

| # | 问题 | 倾向 |
|---|---|---|
| Q1 | 观测出参形态：`snapshot()` 返回值 vs `calc(..., PIDState *out)` | 两者都提供；`snapshot()` 返回内部缓存（零额外计算） |
| Q2 | NaN 时输出什么 | 保持上一拍输出 + 置 fault（需与 foc 上层故障语义对齐，见 roadmap D-5） |
| Q3 | `ff` 与限幅的顺序 | 建议：FF 进"限幅前求和"，让限幅对总输出有效；待 M2 实测定 |
| Q4 | 端口结构体跨版本拷贝风险 | 端口结构体加 `uint16_t version` 字段（或宏），不匹配即编译期/运行期自检 |
| Q5 | `i_scale/pd_scale`（F5）是否入端口 | 倾向入 ports（每拍可变），与 F5 调度器同期定 |
| Q6 | 配置是否需要"目标 LPF 常数" | 倾向不要（外置组件，见 §4） |

## 9. 参考

- ArduPilot AC_PID：`update_all(target, measurement, dt, limit, pd_scale, i_scale)` —— 端口化 + 外部饱和反馈的现实先例
- ODrive：input filter / trajectory 与 controller 分离 —— "外置组合"先例
- 本库：`docs/roadmap.md` §2 对照表、§5 新候选 F1~F8；`docs/research/optimization_considerations.md` 候选 A~D
