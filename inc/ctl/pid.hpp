#pragma once

#include "ctl/lpf.hpp"
#include "ctl/ramp.hpp"

#if defined(__cplusplus) && __cplusplus >= 201703L
#  define CTL_NODISCARD [[nodiscard]]
#elif defined(__GNUC__) || defined(__clang__)
#  define CTL_NODISCARD __attribute__((warn_unused_result))
#else
#  define CTL_NODISCARD
#endif

// ctlkit —— 嵌入式实时控制原语（上游库）
// 来源：cyclotron/foc 的 foc::algo::PID（其自身复用搬运自 lunokhod/actuator/wheel）
// 行为契约：docs/spec/pid.md ｜ 优化路线：docs/roadmap.md
// 工业级 PID：微分先行（无微分冲击）+ 梯形积分 + 积分分离 + 抗饱和 + 输出斜坡

namespace ctl {

// --- son struct ---
struct PIDGains {
    float kp_ = 0.0f;
    float ki_ = 0.0f;
    float kd_ = 0.0f;
};

struct PIDLimits {
    float limit_out_ = 0.0f;   // 输出对称限幅；<= 0 = 不限幅（0 语义统一，roadmap D-1）
    float limit_i_ = 0.0f;     // 积分项预限幅；<= 0 = 不限幅
};

struct PIDTunings {
    float thresh_i_sep_ = 0.0f;
    float max_rate_out_ = 0.0f; // 输出斜坡速率；0 = 关闭（构造函数内归一化为“无上限速率”传给 Ramp：PID 层 0=关闭，Ramp 层 0=冻结，语义不串层）
    float d_filter_Tf_ = 0.0f;
};

// --- father struct ---
struct PIDConfig {
    PIDGains gains_;
    PIDLimits limits_;
    PIDTunings tunings_;
};

struct PIDPorts {
    const float *meas_dot_ = nullptr;
};

// --- observe sink ---
struct PIDState {
    float error_ = 0.0f;
    float p_term_ = 0.0f;
    float d_term_ = 0.0f;
    float integral_ = 0.0f;
    float output_ = 0.0f;
};

// 本拍瞬态布尔量（A 方案）：粘滞的 input_fault 不在此，见 input_fault()
struct PIDStatus {
    bool out_saturated_ = false;            // is_out_limited
    bool i_saturated_ = false;              // is_i_out_limited
};

class PID {
public:
    explicit PID(const PIDConfig &cfg);     // 显式确保PIDConfig作为参数参与构造
    CTL_NODISCARD float calc(float cmd, float measure, float dt, const PIDPorts *ports = nullptr);
    void reset();
    void set_integral(float x);
    void set_gains(const PIDGains &g);
    const PIDState &get_state() const;
    PIDStatus status() const { return status_; }
    bool input_fault() const { return input_fault_; }   // 粘滞：置位后保持到 reset()（与 status() 的本拍瞬态不同）

private:
    // ----- Math Tools -----
    static float fabs(float val);
    static float constrainf(float val, float limit);
    static bool is_finite(float x) { return (x == x) && (x <= 3.402823466e+38f) && (x >= -3.402823466e+38f); }

    // --- property ---
    PIDConfig cfg_;
    float integral_;
    float error_prev_;
    float measure_prev_;
    float last_output_;
    // --- observe ---
    PIDState state_;
    PIDStatus status_;
    bool input_fault_ = false;   // 粘滞 fault（A 方案拆出）：只有 reset() 清
    // --- dsp tools ---
    LPF d_filter_;
    Ramp ramp_out_;
};

}  // namespace ctl
