// ctlkit 最小示例：一阶被控对象上的 PI 控制（电流环风格）
//
// 运行：cmake -S . -B build && cmake --build build && ./build/examples/pid_basic

#include <ctl/pid.hpp>

#include <cstdio>

int main() {
    ctl::PIDConfig cfg;
    cfg.gains_.kp_ = 4.0f;
    cfg.gains_.ki_ = 200.0f;
    cfg.limits_.limit_out_ = 12.0f;   // 输出限幅（如电压上限）
    cfg.limits_.limit_i_ = 12.0f;     // 积分限幅（抗饱和预限幅）

    ctl::PID pid(cfg);

    const float dt = 1e-3f;    // 1 kHz 控制周期
    const float tau = 5e-3f;   // 假想被控对象时间常数

    float measure = 0.0f;
    for (int k = 0; k < 20; ++k) {
        const float u = pid.calc(1.0f, measure, dt);   // 目标 1.0，实测 measure
        measure += (u - measure) * (dt / tau);         // 一阶对象：u → measure
        std::printf("k=%2d  measure=%.4f  u=%.4f\n", k, measure, u);
    }
    return 0;
}
