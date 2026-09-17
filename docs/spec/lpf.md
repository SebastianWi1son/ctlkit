---
class: fact
generated: false
---
# LPF — 行为契约

> 源码：`../../inc` + `src/lpf.cpp` ｜ 精确定义源自 cyclotron `FOC_MATH_SPEC.md` §3.1

## 接口

```cpp
class LPF {
public:
    explicit LPF(float Tf);      // Tf: 时间常数（秒）；构造后不可变
    float calc(float raw, float dt);
    void reset();                // prev_ = 0
    void set_state(float x);     // prev_ = x（bumpless transfer）
private:
    float Tf_, prev_;
};
```

## 计算

```
alpha = dt / (Tf + dt)
prev_ = alpha · raw + (1 - alpha) · prev_
return prev_
```

- `Tf = 0` → `alpha = 1` → **直通**（`dt/(0+dt)`，无除零）；这是 0=disabled 语义的原始出处。
- **无 dt 守卫**：`calc` 不检查 `dt`；调用方必须保证 `dt > 0`（`dt = 0` 会得到 `alpha = 0`，输出冻结；
  `dt < 0` 行为未定义）。这是与 PID 的刻意差异——LPF 是最底层原语，不做策略性兜底。

## 状态与复位

- 唯一状态 `prev_`（初始 0，`reset()` 清零，`set_state(x)` 注入）。
- 注入语义：`prev_` 直接赋值，下一拍从 `x` 连续演化（无跳变）。

## 用法约定

- PID 的 D 项滤波（`d_filter_Tf_` 透传给内部 LPF）。
- SmoothPlanner 的两级圆角（同一 `Tf` 先用两级，构成二阶低通）。
- 与 Ramp 的 0 语义**不同**：LPF 的 0 = 直通；Ramp 的 0 = 冻结（见 `ramp.md`，D-1 待定案）。

## 变更历史

| 版本 | 变更 |
|---|---|
| v0.0.1 | 库化（`foc::algo` → `ctl`），行为未变 |
