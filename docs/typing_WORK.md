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

## 3. 模块 4：观测出口（写前写后对照 · 待你手敲）

> 决策已按推荐定案（M4-1 缓存+零拷贝引用 / M4-2 限幅前判饱和 / M4-3 set_gains 只换增益组 / M4-4 input_fault 粘滞）。
> **改前 = 6c57e3b 的源码**（下面 before 全部来自 `git show HEAD:`，逐字）。
> 你敲完 `inc/` + `src/` 说一声 → 我同步测试/oracle/文档（已备好）→ 全绿验证。

### 3.1 `inc/ctl/pid.hpp` —— 3 处

**① `PIDConfig` 结构体之后、`class PID` 之前，插入两个观测结构体**

before：
```cpp
// --- father struct ---
struct PIDConfig {
    PIDGains gains_;
    PIDLimits limits_;
    PIDTunings tunings_;
};

class PID {
```

after：
```cpp
// --- father struct ---
struct PIDConfig {
    PIDGains gains_;
    PIDLimits limits_;
    PIDTunings tunings_;
};

// --- 观测出口（M1）：与 calc 同拍的值与标志 ---
struct PIDState {
    float error = 0.0f;      // 本拍误差
    float p_term = 0.0f;     // P 项贡献
    float d_term = 0.0f;     // D 项贡献（含 D 滤波）
    float integral = 0.0f;   // 积分累加器 = I 项贡献（本实现里两者是同一个量）
    float output = 0.0f;     // 最终输出（限幅 + 斜坡之后）
};

struct PIDFlags {
    bool out_saturated = false;   // 输出被 limits_.limit_out_ 钳位（控制器已尽力到顶）
    bool i_saturated = false;     // 积分被 limits_.limit_i_ 钳位（windup 状态）
    bool input_fault = false;     // 曾收到 NaN/Inf（粘滞：置位后保持到 reset()）
};

class PID {
```

**② public 区：`set_integral` 声明之后，加 `set_gains` 与两个观测访问器**

before：
```cpp
    /// @brief bumpless transfer: integral start from set point
    /// @param x
    void set_integral(float x);
private:
```

after：
```cpp
    /// @brief bumpless transfer: integral start from set point
    /// @param x
    void set_integral(float x);
    // 在线改增益：成组替换（只动 gains_；tunings_ 里的滤波/斜坡常数构造期已固化，在线改需重建 → 不在本接口范围）
    void set_gains(const PIDGains &g);
    // 观测出口（M1）：state() 零拷贝（引用内部缓存，内容 = 最近一次 calc 的结果）；flags() 按值返回
    const PIDState &state() const { return state_; }
    PIDFlags flags() const { return flags_; }
private:
```

**③ private 成员区：`last_output_` 之后，加两个缓存成员**

before：
```cpp
    float last_output_;
    // --- dsp tools ---
    LPF d_filter_;
    Ramp ramp_out_;
```

after：
```cpp
    float last_output_;
    // --- 观测缓存（M1）---
    PIDState state_;
    PIDFlags flags_;
    // --- dsp tools ---
    LPF d_filter_;
    Ramp ramp_out_;
```

### 3.2 `src/pid.cpp` —— 5 处

**① NaN 守卫：加 fault 标志（一行变四行）**

before：
```cpp
    // ----- NaN Guard -----
    if (!is_finite(cmd) || !is_finite(measure) || !is_finite(dt)) { return last_output_; }
```

after：
```cpp
    // ----- NaN Guard -----
    if (!is_finite(cmd) || !is_finite(measure) || !is_finite(dt)) {
        flags_.input_fault = true;   // 粘滞：reset() 才清
        return last_output_;
    }
```

**② I 项：保留未钳位量用于判定**

before：
```cpp
    float i_term_temp = integral_ + cfg_.gains_.ki_ * dt * 0.5f * (error + error_prev_);
    i_term_temp = constrainf(i_term_temp, cfg_.limits_.limit_i_);                                 // Integral Windup Limit
    if (cfg_.tunings_.thresh_i_sep_ <= 0.0f || fabs(error) <= cfg_.tunings_.thresh_i_sep_) { integral_ = i_term_temp; } // Integral Separation
```

after：
```cpp
    float i_term_temp = integral_ + cfg_.gains_.ki_ * dt * 0.5f * (error + error_prev_);
    float i_term_limited = constrainf(i_term_temp, cfg_.limits_.limit_i_);               // Integral Windup Limit（<= 0 = 不限幅）
    flags_.i_saturated = (i_term_limited != i_term_temp);                                // 只在真被钳位时置位
    if (cfg_.tunings_.thresh_i_sep_ <= 0.0f || fabs(error) <= cfg_.tunings_.thresh_i_sep_) { integral_ = i_term_limited; } // Integral Separation
```

**③ 输出段：未钳位量 + 饱和标志 + 观测缓存**

before：
```cpp
    // ----- Integrate Output-----
    float output = constrainf((p_term + d_term + integral_), cfg_.limits_.limit_out_); // limit output（<= 0 = 不限幅）
    output = ramp_out_.calc(output, dt);   // 斜坡恒开启；关闭时速率已归一化为"无上限"（见构造函数）
    last_output_ = output;
    return output;
```

after：
```cpp
    // ----- Integrate Output-----
    float output_unclamped = p_term + d_term + integral_;
    float output = constrainf(output_unclamped, cfg_.limits_.limit_out_);      // limit output（<= 0 = 不限幅）
    flags_.out_saturated = (output != output_unclamped);                       // 限幅前取未钳位量（roadmap D-3）
    output = ramp_out_.calc(output, dt);   // 斜坡恒开启；关闭时速率已归一化为"无上限"（见构造函数）
    // ----- 观测缓存（M1）-----
    state_.error = error;
    state_.p_term = p_term;
    state_.d_term = d_term;
    state_.integral = integral_;
    state_.output = output;
    last_output_ = output;
    return output;
```

**④ `reset()`：观测缓存一并清零**

before：
```cpp
void PID::reset() {
    integral_ = 0.0f;
    error_prev_ = 0.0f;
    measure_prev_ = 0.0f;
    last_output_ = 0.0f;
    d_filter_.reset();
    ramp_out_.reset();
}
```

after：
```cpp
void PID::reset() {
    integral_ = 0.0f;
    error_prev_ = 0.0f;
    measure_prev_ = 0.0f;
    last_output_ = 0.0f;
    d_filter_.reset();
    ramp_out_.reset();
    state_ = PIDState();      // 观测缓存清空（input_fault 也在此被清）
    flags_ = PIDFlags();
}
```

**⑤ `set_integral` 定义之后，新增 `set_gains` 定义**

before：
```cpp
void PID::set_integral(float x) { integral_ = constrainf(x, cfg_.limits_.limit_i_); }
```

after：
```cpp
void PID::set_integral(float x) { integral_ = constrainf(x, cfg_.limits_.limit_i_); }

void PID::set_gains(const PIDGains &g) { cfg_.gains_ = g; }   // 成组替换：只动增益
```

### 3.3 敲时注意点

1. **② 和 ③ 的"比较放两份"是关键**：先留未钳位量，再钳位，两者 `!=` 才是"真饱和"。
   直接拿钳位后的值和 limit 比较会**恒为 true**（等于自己贴着边界），标志就骗人了。
2. ③ 里 `state_.integral = integral_;` 要放在**积分分离更新之后**（快照的是本拍最终积分）。
3. ④ `state_ = PIDState();` 是值初始化（C++11 下 NSDMI 生效）；别只清一个字段。
4. `state()` 返回 **const 引用**：调用方拿着它跨过下一次 `calc` 会看到新数据——这是设计（零拷贝），文档已写明。
5. 敲完跑 `cmake --build build && ctest --test-dir build --output-on-failure`：
   此时测试还是模块 4 之前的（16 例），应当**全绿**——模块 4 是纯新增，不改旧路径行为。
   然后我同步观测用例（oracle/黄金/smoke）再验证一次。

### 3.4 你敲完之后我做的事（已备好，不占用你）

- `tests/smoke_test.cpp`：加 `test_pid_observation`（state 各分量 / 输出饱和 / 积分饱和 / set_gains / fault 粘滞）
- `tests/golden_test.cpp` + `oracle/`：新增 `pid_flags` 黄金组件 2 例（饱和标志**精确**比对，不进容差）→ 18 例
- `docs/spec/pid.md`：接口表 + 「观测出口」节 + 状态清单 + 变更历史；`CHANGELOG` / roadmap M1 / 设计稿状态行
- 全绿后一并提交

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
| 2026-09-17 | 4 | **AI 越界纠正**：未等用户手敲就自行实现模块 4 源码 → 已全部撤销（WIP 存 `/tmp/module4-wip.patch`），改为写前写后对照交用户手敲；测试/oracle/文档同步物一并撤回，待敲完再上 | 源码回到 6c57e3b（干净基线）；本文件 §3 即敲写依据 |
| 日期 | 模块 | 决策 | 影响（spec / 黄金向量 / 下游） |
|---|---|---|---|
| 2026-09-17 | 4 | **AI 越界纠正**：未等用户手敲就自行实现模块 4 源码 → 已全部撤销（WIP 存 `/tmp/module4-wip.patch`），改为写前写后对照交用户手敲；测试/oracle/文档同步物一并撤回，待敲完再上 | 源码回到 6c57e3b（干净基线）；本文件 §3 即敲写依据 |
