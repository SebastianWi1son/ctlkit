#pragma once

#include "ctl/lpf.hpp"
#include "ctl/ramp.hpp"

// ctlkit —— 嵌入式实时控制原语（上游库）
// 来源：cyclotron/foc 的 foc::algo::PID（其自身复用搬运自 lunokhod/actuator/wheel）
// 行为契约：docs/spec/pid.md ｜ 优化路线：docs/roadmap.md
// 工业级 PID：微分先行（无微分冲击）+ 梯形积分 + 积分分离 + 抗饱和 + 输出斜坡

namespace ctl {

struct PIDConfig {
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;

    float limit_out_ = 0.0f;
    // --- i_term method property ---
    float limit_i_ = 0.0f;
    float thresh_i_sep_ = 0.0f;
    // --- dsp tools property ---
    float max_rate_out_ = 0.0f;
    float d_filter_Tf_ = 0.0f;
};

class PID {
public:
    explicit PID(const PIDConfig &cfg);     // 显式确保PIDConfig作为参数参与构造
    float calc(float cmd, float measure, float dt);
    void reset();
private:
    // ----- Math Tools -----
    static float fabs(float val);
    static float constrainf(float val, float limit);

    // --- property ---
    PIDConfig cfg_;
    float integral_;
    float error_prev_;
    float measure_prev_;
    // --- dsp tools ---
    LPF d_filter_;
    Ramp ramp_out_;
};

}  // namespace ctl
