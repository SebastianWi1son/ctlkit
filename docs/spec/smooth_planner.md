# SmoothPlanner — 行为契约

> 源码：`include/ctl/smooth_planner.hpp` + `src/smooth_planner.cpp`
> 精确定义源自 cyclotron `FOC_MATH_SPEC.md` §3.3（与 legacy `dsp_traj` 完全同构）

## 接口

```cpp
class SmoothPlanner {
public:
    SmoothPlanner(float max_rate, float Tf);
    float calc(float cmd, float dt);
    void reset();               // ramp/f1/f2 全清零
    void set_state(float x);    // ramp/f1/f2 三层状态统一置 x（bumpless transfer）
private:
    Ramp ramp_;
    LPF f1_, f2_;
};
```

## 计算

```
ramped = ramp_.calc(cmd, dt)          # 梯形限速（max_rate）
return f2_.calc(f1_.calc(ramped, dt), dt)   # 两级一阶 LPF（同一 Tf）→ S 曲线圆角
```

即：**先限速、后两级平滑**。`max_rate` 决定梯形斜坡，`Tf` 决定拐角圆滑度（越大越平滑、滞后越大）。

## 状态与复位

- 三层状态：`ramp_.prev_`、`f1_.prev_`、`f2_.prev_`（初始 0）。
- `reset()`：三层全清零。
- `set_state(x)`：三层统一置 `x`——**整体注入**，保证三者一致（只注其一会导致内部不一致的动态）。
  用途：对齐完成 / 模式切换时把规划器同步到当前物理量，避免首帧从 0 渐变造成阶跃。
  （此接口由 cyclotron 侧补齐，等价 legacy `dsp_traj` 的 `ramp_target/filter1/filter2 = settled` 三行赋值。）

## 继承的语义注意

- `max_rate = 0` → Ramp 冻结（见 `ramp.md`），即整个规划器输出冻结在 `prev_`。
- `Tf = 0` → 两级 LPF 均直通，退化为纯 Ramp。
- 无 dt 守卫，调用方保证 `dt > 0`。

## 变更历史

| 版本 | 变更 |
|---|---|
| v0.0.1 | 库化（`foc::algo` → `ctl`），行为未变；收录 cyclotron 的 `set_state` |
