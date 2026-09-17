#pragma once

// ctlkit —— 嵌入式实时控制原语（上游库）
// PID 支撑类型（配置 / 每拍端口 / 观测记录）：与类定义分离，下游读类型不必翻类实现
// 行为契约：docs/spec/pid.md ｜ 设计：docs/design/pid_config_and_ports.md
//
// 命名与冻结：对外数据记录（配置 / 端口 / 观测）字段沿用尾下划线；
// 自 v0.1.0 起字段名与语义**冻结** —— 只增不改，破坏进 major（见 README「兼容政策」）。

namespace ctl {

// --- 配置（构造期一次注入，构造后不可变）---
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

struct PIDConfig {
    PIDGains gains_;
    PIDLimits limits_;
    PIDTunings tunings_;
};

// --- 每拍端口（模块 5 起；calc 签名冻结，新特性只往这里加字段）---
// 全部可缺省：ports == nullptr 或字段 nullptr = 旧行为
struct PIDPorts {
    const float *meas_dot_ = nullptr;   // external derivative source injection（外部微分：观测器提供，量纲/符号 = d(measure)/dt）
};

// --- 观测记录：数值快照 ---
struct PIDState {
    float error_ = 0.0f;
    float p_term_ = 0.0f;
    float d_term_ = 0.0f;
    float integral_ = 0.0f;
    float output_ = 0.0f;
};

// 本拍瞬态布尔量（A 方案）：粘滞的 input_fault 不在此，见 PID::input_fault()
struct PIDStatus {
    bool out_saturated_ = false;            // is_out_limited
    bool i_saturated_ = false;              // is_i_out_limited
};

}  // namespace ctl
