// ctlkit 最小示例：一阶被控对象上的 PI 控制（电流环风格）
//
// 运行：cmake -S . -B build && cmake --build build && ./build/examples/pid_basic

#include <ctl/pid.hpp>

#include <cstdio>

int main() {
    const ctl::PIDConfig cfg = ctl::PIDConfig{}
            .kp(4.0f).ki(200.0f)
            .limit_out(12.0f)         // 输出限幅（如电压上限）
            .limit_i(12.0f);          // 积分限幅（抗饱和预限幅）

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
