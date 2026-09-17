#pragma once

// ctlkit —— 嵌入式实时控制原语（上游库）
// 来源：cyclotron/foc 的 foc::algo::Ramp（复用搬运自 lunokhod/actuator/wheel）
// 行为契约：docs/spec/ramp.md
// 斜率限制器：每帧 clamp 到 [prev ± max_rate·dt]
// ⚠ max_rate=0 → step=0 → 输出冻结在 prev_（不是直通）；0 语义待定案，见 docs/roadmap.md §6 D-1

namespace ctl {

class Ramp {
public:
    Ramp(float max_rate);
    float calc(float cmd, float dt);
    void reset();
    // 状态注入（bumpless transfer）：prev_ = x，后续 calc 从 x 连续起步
    void set_state(float x);
private:
    float max_rate_;
    float prev_;
};

}  // namespace ctl
