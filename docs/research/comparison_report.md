# PID 对比报告：Liu-Curiousity/pid vs cyclotron foc::algo::PID

## 0. 对比对象

| | GitHub 库 | 本地实现 |
|---|---|---|
| 来源 | https://github.com/Liu-Curiousity/pid（V1.2.2, `d0b1045`, 2026-08-27） | `cyclotron/foc/inc/foc/algo/pid.hpp` + `src/foc/algo/pid.cpp` |
| 形态 | 单文件 header-only 通用库（137 行） | 组件化 .hpp/.cpp 分离，依赖 `lpf.hpp`、`ramp.hpp` |
| 面向场景 | 教学/通用控制（RoboMaster 风格双模式库） | FOC 电流环（dq 轴电流调节），源自 lunokhod wheel |
| 许可 | 无 LICENSE 文件（头文件标注 `(c) 2025 QDrive`） | 项目内部 |

GitHub 库文件清单：`PID.h`、`CMakeLists.txt`（INTERFACE 库，仅 include 目录）。
本地依赖链：`PID` → `LPF`（一阶低通，α = dt/(Tf+dt)，Tf=0 直通）→ `Ramp`（斜率限幅，max_rate=0 冻结）。

---

## 1. 架构与 API 设计对比

### 1.1 GitHub：字段公开、目标内置、模式可选的"功能库"

```cpp
enum PID_type { position_type, delta_type };
PID() = delete;
PID(PID_type, kp, ki, kd, Ts = 1,
    optional<float> sum_error_limit_p/n, optional<float> output_limit_p/n);

void SetTarget(float);          // 控制器持有目标值
void set_sum_error(float);      // 积分状态注入（公开）
void reset();                   // 状态+目标+输出全部清零
[[nodiscard]] float calc(float input, optional<float> dinput = {});

float target, kp, ki, kd;       // 全部公开，可随时改
float Ts;
optional<float> sum_error_limit_p, sum_error_limit_n;  // 非对称
optional<float> output_limit_p, output_limit_n;
```

- 构造函数直接吃标量参数 + 默认值；**默认构造被删除**，强制显式传类型与三增益。
- 无 STL 依赖之外用了 `<optional>`、`<cmath>`。
- 使用风格：`pid.SetTarget(v); y = pid.calc(measure);` —— **目标值归控制器所有**。

### 1.2 本地：config 注入、状态私有、dt 显式的"控制器组件"

```cpp
struct PIDConfig {
    float kp_, ki_, kd_;
    float limit_out_, limit_i_, thresh_i_sep_, max_rate_out_, d_filter_Tf_;  // 全 0 默认
};
explicit PID(const PIDConfig&);
float calc(float cmd, float measure, float dt);
void reset();
// 私有：integral_, error_prev_, measure_prev_, d_filter_, ramp_out_
```

- 构造后配置**不可变**（私有 cfg_），运行时只能 reset。
- 使用风格：`u = pid.calc(ref, measure, dt);` —— 目标值由调用方每拍传入，对象无目标状态。
- **0 = 禁用**语义：`thresh_i_sep_<=0` 关积分分离；`max_rate_out_=0` 关斜坡；`d_filter_Tf_=0` 直通。
- 纯 float、无 STL，嵌入式友好。

### 1.3 结构性差异小结

| 设计点 | GitHub | 本地 | 评价 |
|---|---|---|---|
| 状态封装 | 公开字段 | 全私有 | 本地约束更强、防误用；GitHub 灵活（运行中改 kp） |
| 设定值归属 | 控制器内 | 调用方 | 本地形式对多环级联（每拍换 ref）更干净；GitHub 调用点更短 |
| 运行时改参 | 直接赋值 | 不支持 | 本地若需在线调参需另开接口 |
| reset 语义 | 连 target 清零（需重 SetTarget） | 只清运行时状态 | 语义不同：本地更接近"纯状态复位" |
| 依赖 | `<optional>/<cmath>` | 无 STL | 场景差异（通用 C++ vs 嵌入式 float） |
| 形态 | header-only 易部署 | .hpp/.cpp | — |

---

## 2. 算法/公式逐项对比

### 2.1 误差定义
两边一致：`e = cmd(target) - measure(input)`，D 项均取测量负差分方向，同为标准负反馈极性。无差异。

### 2.2 P 项
两边都是 `kp·e`。无差异。

### 2.3 I 项（本对比最大算法差异之一）

**GitHub（位置式）**——矩形（前向欧拉）积分，无条件限幅：
```cpp
sum_error += error * Ts;                                  // 量纲：误差×秒
if (sum_error_limit_p && sum_error >= sum_error_limit_p) sum_error = *sum_error_limit_p;
if (sum_error_limit_n && sum_error <= sum_error_limit_n) sum_error = *sum_error_limit_n;
```

**本地**——梯形（Tustin）积分 + 抗饱和预限幅 + 积分分离：
```cpp
i_term_temp = integral_ + ki_ * dt * 0.5f * (error + error_prev_);  // 梯形：新旧误差均值
i_term_temp = constrainf(i_term_temp, limit_i_);                    // 限幅后（而非限幅前）再写入
if (thresh_i_sep_ <= 0 || fabs(error) <= thresh_i_sep_)             // 大误差冻结积分（分离）
    integral_ = i_term_temp;
```

差异点：
1. **积分算法**：梯形 vs 矩形。dt 抖动/变化时梯形精度更好、更平滑（矩形对 dt 敏感）。
2. **限幅量纲**：GitHub 限"误差×秒"的累加量（`sum_error_limit`），需要自行换算 ki 贡献；本地 `limit_i_` 直接是**输出单位的 I 贡献上限**，调参直觉更好、跨移植需换算。
3. **积分分离**：本地在大误差（如阶跃起步）时冻结积分，防止起步超调 + 快速收敛；GitHub 位置式无分离（大误差期间积分持续累加 → 更易 overshoot/饱和）。
4. **条件更新顺序**：本地"先 clamp 再决定是否写入"，分离时丢弃本次增量；GitHub"累加后 clamp"。两者都能限制积分上限，但本地多一层分离。

### 2.4 D 项（核心差异）

**GitHub（位置式默认）**——对误差微分，无滤波：
```cpp
kd * (error_ - error) / Ts        // e 变化率 → 设定值跳变时产生微分冲击(kick)
```
**GitHub（位置式 + dinput）**——设定值差分 + 外部测量微分：
```cpp
kd * ((target - pre_target) / Ts - dinput.value())
//    ↑ 设定值后向差分：阶跃时 (target-pre_target)/Ts 仍是一个大脉冲 → 冲击仍在
```
> 即：`dinput` 只替代"对测量的内部差分"（降低在环内重复差分噪声），并未消除设定值阶跃冲击——因为设定值项仍由差分产生。若目标来自轨迹规划器（每拍缓变），该冲击可忽略；若直接给阶跃目标，仍有 kick。

**本地**——微分先行（derivative-on-measurement）：
```cpp
d_term_raw = -kd_ * (measure - measure_prev_) / dt;   // 只对测量微分
d_term = d_filter_.calc(d_term_raw, dt);              // D 项一阶 LPF（Tf 可调）
```
- 设定值（cmd）变化**完全不进 D 项** → 阶跃无冲击。
- 代价：测量噪声被微分放大 → 用 D 项 LPF 压制（Tf 大则 D 高频响应滞后，需权衡）。
- GitHub 无 D 滤波：其 D 项裸差分，噪声大（除非外部 dinput 质量高）。

### 2.5 增量式（delta）模式 —— GitHub 独有

```cpp
output += kp*(e-e1) + ki*e*Ts + kd*(e - 2*e1 + e2)/Ts;   // 输出自累加 = 隐式积分
pre_error = e1; e1 = e;  ...
// 仅末端 output clamp
```
- 适用：手动/自动无扰切换、外环（速度/位置）驱动增量执行机构、输出即增量变化量。
- **已知弱点（其代码未处理）**：长期饱和时内部累加器继续涨（clamp 只截末端返回值），误差反向恢复时存在 windup 迟滞/超调——增量式同样需要条件积分/back-calculation 才完整。
- 本地没有增量模式；对 FOC 电流环（输出为电压绝对量、需显式积分状态可复位）位置式是正确选择，无需借鉴此模式，除非未来做外环。

### 2.6 输出处理

| | GitHub | 本地 |
|---|---|---|
| 输出限幅 | 独立非对称 ±（optional 缺省 = 不限幅） | 对称 ±limit_out_ |
| 斜坡/速率限制 | 无 | `Ramp`（max_rate_out_>0 启用；每帧 clamp `prev ± max_rate·dt`） |
| 顺序 | 计算 → clamp → 返回 | 合成 clamp → 斜坡 → 返回 |

- 本地多了**输出斜坡**：限制控制器输出变化率，防止输出阶跃（如使能瞬间、参考突变）直击执行器；对电流环常用于限制 Iq 指令突变。
- 本地为对称限幅，无法表达不对称（如再生/电动能力不同）；GitHub 的独立 ± 限幅覆盖此场景。

### 2.7 时间处理

| | GitHub | 本地 |
|---|---|---|
| dt 来源 | 构造固定 `Ts`（默认 1s） | `calc(..., dt)` 逐次传入 |
| 守卫 | 无（调用周期必须严格 = Ts） | `dt<=0 || dt>0.5 → 0.001`（防除零/垃圾 dt） |
| 影响 | Ts 不匹配则公式失真（尤其 I 项 Σe·Ts） | 异常 dt 被静默替换为 1ms（低环率 <2Hz 时有偏差，需知晓） |

### 2.8 复位与状态注入

| | GitHub | 本地 |
|---|---|---|
| reset | error/target/pre_target/sum_error/output 全清零（target 也清零！） | integral_/error_prev_/measure_prev_ + d_filter_.reset() + ramp_out_.reset()（**不碰 config，无目标状态**） |
| 状态注入 | `set_sum_error(x)` 公开（注释：用于高自由度积分限幅/手动整定） | 无 PID 级注入；仅 LPF/Ramp 有 `set_state(x)`（bumpless transfer） |

本地设计缺口：PID 的 `integral_` 无法被外部写入 → 环路切入/控制器切换时缺少"从当前工作点连续起步"的手段（LPF/Ramp 都有 `set_state`，唯独 PID 没有，内部设计不一致）。

---

## 3. 健壮性/易用性对比

| 项 | GitHub | 本地 |
|---|---|---|
| 默认构造 | `= delete`（强制显式初始化）✅ | 无默认构造；但 `PIDConfig{}` 全 0 会静默生成"永远输出 0"的 PID ⚠️ |
| calc 返回值 | `[[nodiscard]]` ✅ | 未标注（丢弃返回值编译器不提示） |
| NaN 传染 | clamp 用比较实现：NaN 既不过上界也不低下界 → **返回 NaN**；sum_error 一次 NaN 后永久 NaN（无恢复） | 同样问题：`constrainf(NaN)` 返回 NaN；integral_ 一旦污染需 reset |
| 限幅缺省 | optional 不设 = 不限幅（忘设则无保护） | 全 0 = 禁用/归零（limit_out_=0 → 恒输出 0，需显式配置） |
| 编译依赖 | 需 C++17（optional） | C++11 即可（纯 float） |

---

## 4. 结论

1. **算法层面**：本地整体显著更"工业级"——微分先行（无冲击）、梯形积分、积分分离、D 滤波、输出斜坡、dt 守卫，均为 FOC/伺服场景的正确选择；GitHub 库更适合教学与通用快速验证。
2. **GitHub 值得参考的点（可移植性高、改动小）**：
   - 外部微分注入 `dinput`（配合已有观测器，替代环内二次差分）；
   - 积分状态注入 `set_sum_error`（补全 bumpless transfer 一致性）；
   - 非对称独立 ± 限幅（再生/单象限不对称工况）；
   - `[[nodiscard]]`、删默认构造等接口习惯。
3. **两边共同的短板**（未来优化方向见 `optimization_considerations.md`）：
   - 均为静态 clamp 抗饱和，无基于输出饱和的 back-calculation/条件积分；
   - NaN 无防护（ADC 异常/除零污染会永久锁死积分）；
   - 无可观测性接口（无状态 getter，不利于调参/诊断/自整定）。

> 详细落地建议见同目录 `optimization_considerations.md`。
