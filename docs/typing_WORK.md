---
class: work
generated: false
accepted: false
---
# typing_WORK — 手敲代码缓冲区（一次一个模块）

> **类：D 施工单** —— 全部模块敲完并验收后，本文件移出 `docs/`（或改 `accepted: true` 交给门禁拦下）。
> **规矩**：我只准备内容、不代抄；每次只产出**一个模块**；你在敲的过程中随时追问、随时改决策；
> 任何决策若改了行为，必须**同批**同步 `docs/spec/` 与黄金向量（否则门禁和回归都会说谎）。

## 1. 流程

1. 我给出**模块 N**：文件、真实代码、逐行理由、敲时注意点、验证命令、待你拍板的决策点。
2. 你阅读 + 手敲；任何一行都可以叫我先解释，或提出改法。
3. 敲完跑验证（编译 + 测试）；不通过就回到第 2 步讨论。
4. 你把决策结论告诉我 → 我更新代码 / spec / 黄金向量（改动逐处给你过目）。
5. 你说「下一个」→ 我给模块 N+1。

## 2. 模块清单（缓冲区）

| # | 模块 | 文件 | 行数 | 状态 | 主要学习点 |
|---|---|---|---|---|---|
| 1 | **Deadzone** | `inc/ctl/deadzone.hpp` + `src/deadzone.cpp` | 23 + 13 | ⬅ **本次产出** | 最小完整组件；无状态；软/硬死区；边界与除零；库内唯一 `<cmath>` |
| 2 | LPF | `inc/ctl/lpf.hpp` + `src/lpf.cpp` | 23 + 17 | 待 | 有状态组件：`reset()` / `set_state()`；`Tf=0` 直通；无 dt 守卫的取舍 |
| 3 | Ramp | `inc/ctl/ramp.hpp` + `src/ramp.cpp` | 23 + 20 | 待 | 状态写回铁律；`max_rate=0` 冻结语义（roadmap 决策点 D-1 的一半） |
| 4 | SmoothPlanner | `inc/ctl/smooth_planner.hpp` + `src/smooth_planner.cpp` | 26 + 16 | 待 | 组件组合；三层状态一致注入 |
| 5 | **PID** | `inc/ctl/pid.hpp` + `src/pid.cpp` | 47 + 48 | 待 | 核心：calc 六步、梯形积分、积分分离、静态抗饱和、微分先行、dt 守卫；6 个改进点入口 |
| 6 | 验证链 | `tests/golden_test.cpp` + `oracle/` 走一遍 | — | 待 | 黄金向量从哪来；故意改坏一行 → 变红 |
| 7 | PID 端口骨架 | 落地 `docs/design/pid_config_and_ports.md` | — | 待（取决于模块 5 的决策） | 生命周期三分：Config / Ports / State；calc 签名冻结 |

> 状态只在你确认敲完（或改完）后由我改成 ✅；「待」= 内容还没给你。

## 3. 模块 1：Deadzone

### 3.1 代码（真实代码，与仓库逐字一致）

`inc/ctl/deadzone.hpp`

```cpp
#pragma once

#include <cmath>

// ctlkit —— 嵌入式实时控制原语（上游库）
// 来源：cyclotron/foc 的 foc::algo::Deadzone（其自身复用搬运自 lunokhod）
// 行为契约：docs/spec/deadzone.md
// 抛物线软死区：|e| < range 时按 e·(|e|/range) 衰减（0 处增益 0，|e|=range 处连续）；
// 区外原样；range <= 0 时软分支恒不成立 → 天然直通，无除零。
// 注意：这是库内唯一直接使用 <cmath> 的组件（其余组件自带 float 原语）。

namespace ctl {

class Deadzone {
public:
    Deadzone(float range = 0.0f, bool soft = true);
    float calc(float error) const;
private:
    float range_;
    bool soft_;
};

}  // namespace ctl
```

`src/deadzone.cpp`

```cpp
#include "ctl/deadzone.hpp"

namespace ctl {

Deadzone::Deadzone(float range, bool soft) : range_(range), soft_(soft) {}

float Deadzone::calc(float error) const {
    float abs_error = std::fabs(error);
    if (abs_error < range_) return soft_ ? (error * (abs_error / range_)) : 0.0f;
    return error;
}

}  // namespace ctl
```

### 3.2 逐行理由

| 行 | 理由 |
|---|---|
| `#include <cmath>` | 只为 `std::fabs`。这是**全库唯一**的标准数学头依赖（见决策点 D1-1） |
| `float calc(float error) const` | 无状态纯函数：同一输入永远同一输出；`const` 让编译器可放宽优化。**注意它没有 `reset()/set_state()`** —— Deadzone 没有状态要复位（决策点 D1-5） |
| `Deadzone(float range = 0.0f, bool soft = true)` | 默认值 = 禁用（`range=0` → 直通）。这是全库 5 个组件里唯一带默认构造参数的（决策点 D1-3） |
| `float abs_error = std::fabs(error);` | 先取绝对值，后面所有比较都用它 |
| `if (abs_error < range_)` | **严格小于**：边界 `|e| == range` 走"区外原样"分支 —— 这保证了软模式在边界连续（`e·(range/range) = e`） |
| `soft_ ? (error * (abs_error / range_)) : 0.0f` | 软：抛物线衰减（0 处增益 0、边界增益 1）；硬：带内直接归零（不连续，属设计意图）。**注意 `0.0f` 的 `f` 后缀**，漏了会被提升成 double 再截断 |
| `return error;` | 区外原样（不在带内就不动它） |

### 3.3 敲时注意点（容易敲错的四处）

1. **`abs_error / range_` 不要提到 `if` 外面**：`range_ = 0` 时那会除零 → NaN。
   现在的结构是"先比大小、后相除"，从结构上就避开了这个坑（FOC_MATH_SPEC §7 记着 legacy 就是这么踩的）。
2. **边界用严格小于**：写成 `<=` 会让 `|e| = range` 时也进带内（软模式结果不变，硬模式会多归零一个点，
   黄金向量 `deadzone_hard` 会立刻变红）。
3. **三元运算符的两边类型**：`(error * (abs_error / range_))` 是 float，`0.0f` 也是 float —— 类型一致。
4. **`const` 不能漏**：漏了编译器不报错，但接口退化成可变（并且会误导"这个组件有状态"）。

### 3.4 验证

```bash
cmake --build build && ctest --test-dir build --output-on-failure
```

期望：`2/2` 通过。黄金回归里 deadzone 三个用例：

| 用例 | 覆盖 | 实测偏差 vs 容差（2026-09-17） |
|---|---|---|
| `deadzone_soft` | 软衰减 + 边界 + 区外 | 4.8e-08 vs 1.58e-06 |
| `deadzone_hard` | 硬死区（带内归零/带外原样） | 4.8e-08 vs 1.58e-06 |
| `deadzone_disabled` | `range=0` 直通 | 3.0e-09 vs 1.02e-06 |

另外 `tests/smoke_test.cpp` 的 `test_deadzone()` 有 7 条解析断言（0.18 / 边界 / 同号 / range=0）。

### 3.5 待你拍板的决策点（敲完告诉我结论）

| # | 问题 | 现状 | 选项与取舍 |
|---|---|---|---|
| **D1-1** | 库内唯一 `<cmath>` 依赖要不要去掉？ | `std::fabs` | (a) 保留：嵌入式的 `std::fabs` 通常被内联成 `fabsf` 指令，零成本；(b) 换 `error < 0.0f ? -error : error`：与 `pid.cpp` 的自实现风格一致，去掉唯一标准头（注意 `-0.0f` 与 NaN 语义两边一致，已核对） |
| **D1-2** | `soft_` 运行时分支 vs 编译期两态？ | 运行时 `bool` | (a) 保留：可配置、代码只有一份；(b) 拆成 `SoftDeadzone` / `HardDeadzone` 两个类：零分支但 API 分裂、组合器要写两遍 |
| **D1-3** | 构造函数默认值与其他组件风格不一致（PID/LPF/Ramp 都要显式传参） | 有默认值 | (a) 保留：参数少，默认=禁用很自然；(b) 去默认值，强制显式（调用点更啰嗦）；(c) 跟 PID 一样走 config 结构（对这个两参数组件是过度设计） |
| **D1-4** | `range < 0` 的意图在代码里不可见（靠"软分支恒不成立"隐式直通） | 隐式 | (a) 保持（与 spec 的 `range<=0 → 直通` 一致）；(b) 开头加一句 `if (range_ <= 0.0f) return error;` 把意图摆明（多一次比较） |
| **D1-5** | 无状态组件要不要也提供 `reset()` / `set_state()` 空实现？ | 没有 | (a) 不加（YAGNI：没有任何调用点需要）；(b) 加空实现，让"算法链"可以统一处理所有组件 |
| **D1-6** | 类名与术语 | `Deadzone` | (a) 保持；(b) `Deadband`（英文语境更常用，"死区/死带"）；(c) `SoftDeadzone`（突出默认模式，但硬模式就在同一个类里，名字会骗人） |

## 4. 决策记录（边敲边记 —— 只增不改）

| 日期 | 模块 | 决策 | 影响（spec / 黄金向量 / 下游） |
|---|---|---|---|
| | | | |
