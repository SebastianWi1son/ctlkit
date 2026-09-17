---
class: fact
generated: false
---
# Ramp — 行为契约

> 源码：`../../inc` + `src/ramp.cpp` ｜ 精确定义源自 cyclotron `FOC_MATH_SPEC.md` §3.2

## 接口

```cpp
class Ramp {
public:
    explicit Ramp(float max_rate);   // 速率上限（单位/秒）；构造后不可变
    float calc(float cmd, float dt);
    void reset();                    // prev_ = 0
    void set_state(float x);         // prev_ = x（bumpless transfer）
private:
    float max_rate_, prev_;
};
```

## 计算

```
step = max_rate · dt
out  = cmd
if cmd > prev + step:  out = prev + step
if cmd < prev - step:  out = prev - step
prev = out                       # 关键：每帧都写回（含直通帧）
return out
```

- ⚠ **`max_rate = 0` → `step = 0` → 输出被拉回 `prev_`（初始 0）→ 冻结输出**，**不是直通**。
  这是 wheel 的历史语义；与 LPF 的 0=直通 冲突，属 D-1（`0` 语义统一）定案的输入之一。
- 无 dt 守卫（与 LPF 同）：调用方保证 `dt > 0`。

## 状态与复位

- 唯一状态 `prev_`（初始 0，`reset()` 清零，`set_state(x)` 注入）。
- 端点语义：比较为严格不等号（`cmd == prev ± step` 时走直通分支，`out = cmd`）——浮点下等价，无需关心。

## 用法约定

- PID 输出斜坡（`max_rate_out_ > 0` 时启用；PID 的 0 语义是"关闭"，不是"冻结"——注意层级差异）。
- SmoothPlanner 的限速级（`max_rate = 0` 时同样冻结，与 Ramp 一致）。
- 上层若要"0 = 不限速"的直通语义，应在调用点跳过 Ramp（cyclotron 的 P1 决策即此方案），
  而不是依赖 Ramp 内部行为。

## 变更历史

| 版本 | 变更 |
|---|---|
| v0.0.1 | 库化（`foc::algo` → `ctl`），行为未变 |
