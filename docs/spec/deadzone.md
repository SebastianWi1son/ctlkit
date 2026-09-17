---
class: fact
generated: false
---
# Deadzone — 行为契约

> 源码：`inc/ctl/deadzone.hpp` + `src/deadzone.cpp`
> 血缘：cyclotron `foc::algo::Deadzone` → ctlkit `ctl::Deadzone`（行为未变）
> 血缘：与 cyclotron 侧 `soft_deadzone` 一致（`FOC_MATH_SPEC.md` §7，已退役）；精确定义**以本文为准**

## 接口

```cpp
class Deadzone {
public:
    Deadzone(float range = 0.0f, bool soft = true);   // 构造后不可变
    float calc(float error) const;                    // 无状态纯函数
private:
    float range_;
    bool soft_;
};
```

- 库内**唯一无状态**组件：没有 `reset()` / `set_state()`（没有状态需要注入）。

## 计算

```
a = |error|
if a < range:
    if soft:  return error · (a / range)      # 抛物线软衰减
    else:     return 0.0f                     # 硬死区
return error                                   # 区外原样
```

| 参数 | 语义 |
|---|---|
| `range <= 0` | **直通**（软分支要求 `a < range`，恒不成立）——即 `0 = disabled`；且**天然无除零**（与那种先把 `a/range` 算出来的写法不同） |
| `soft = true` | 抛物线衰减：`e·(|e|/range)`；`e → 0` 时增益 → 0；`|e| = range` 处与区外连续 |
| `soft = false` | 硬死区：带内归零；在 `|e| = range` 处**不连续**（设计意图，不是缺陷） |

## 性质（可直接当测试锚点）

- 不放大：`|out| ≤ |error|`
- 同号：`error · out ≥ 0`（不过零）
- 软模式在边界连续：`|e| = range` 时 `e·(range/range) = e`
- `range <= 0` → 恒等映射
- 软模式单调：`|e|` 增大 → `|out|` 不减

## 已知缺口

- 无 NaN/Inf 防护：`error = NaN` → 比较为假 → 原样返回 NaN（与 PID 同类问题，见候选 C1）
- 软衰减在带内**有意**保留稳态误差（抑制超调/抖动的代价，属设计取舍）
- 库里唯一使用 `<cmath>`（`std::fabs`）的组件；其余组件自带 float 原语（是否统一见打字缓冲区的模块 1 决策点）

## 用法约定

- 典型用法（cyclotron）：对位置误差 `planned - tracker_.angle_abs()` 做软衰减，
  抑制稳态超调与低频震荡；级联链路中 `range = 0` 与 `> 0` 两条路径都要出现在锚点测试里。
- 禁用请用 `range = 0`，不要靠"调用点跳过"来禁用（调用点会忘，参数不会）。

## 变更历史

| 版本 | 变更 |
|---|---|
| Unreleased | 库化（`foc::algo` → `ctl`，接入 CMake、spec、oracle），行为未变 |
