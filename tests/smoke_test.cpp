// ctlkit 冒烟测试（主机端）
//
// 定位：验证「库能编、基本行为符合 docs/spec/」的快速回归。
// 正式的黄金向量回归（逐拍快照、dt 扰动、饱和恢复、NaN 注入等）在 M0 建立，
// 见 docs/roadmap.md §8 验证与回归策略。
//
// 注意：本文件中的断言即当前行为契约的可执行版本；行为变更时必须同步改这里。

#include <ctl/deadzone.hpp>
#include <ctl/pid.hpp>
#include <ctl/lpf.hpp>
#include <ctl/ramp.hpp>
#include <ctl/smooth_planner.hpp>

#include <cmath>
#include <cstdio>

static int g_failures = 0;

#define CHECK(cond)                                                    \
    do {                                                               \
        if (!(cond)) {                                                 \
            std::printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                              \
        }                                                              \
    } while (0)

static bool near(float a, float b, float tol = 1e-6f) { return std::fabs(a - b) <= tol; }

static void test_pid_p_term() {
    ctl::PIDConfig cfg;
    cfg.gains_.kp_ = 2.0f;
    cfg.limits_.limit_out_ = 100.0f;
    ctl::PID pid(cfg);

    CHECK(near(pid.calc(1.0f, 0.0f, 1e-3f), 2.0f));   // u = kp·e
    CHECK(near(pid.calc(1.0f, 1.0f, 1e-3f), 0.0f));   // 误差归零 → 输出 0
}

static void test_pid_trapezoid_integral() {
    ctl::PIDConfig cfg;
    cfg.gains_.ki_ = 1000.0f;
    cfg.limits_.limit_out_ = 1000.0f;
    cfg.limits_.limit_i_ = 1000.0f;
    ctl::PID pid(cfg);

    // 梯形积分：i = ki·dt·(e + e_prev)/2 = 1000·0.001·(1+0)/2 = 0.5
    CHECK(near(pid.calc(1.0f, 0.0f, 1e-3f), 0.5f));
    // 第二拍：(1+1)/2 = 1 → 再 +1.0 → 1.5
    CHECK(near(pid.calc(1.0f, 0.0f, 1e-3f), 1.5f));
}

static void test_pid_derivative_on_measurement() {
    ctl::PIDConfig cfg;
    cfg.gains_.kd_ = 0.1f;
    cfg.limits_.limit_out_ = 100.0f;
    cfg.limits_.limit_i_ = 100.0f;
    ctl::PID pid(cfg);

    CHECK(near(pid.calc(0.0f, 0.0f, 1e-3f), 0.0f));   // 建立 measure_prev_ 状态（首拍输出 0）
    // d = -kd·(m - m_prev)/dt = -0.1·(0.1-0)/0.001 = -10
    // 设定值（cmd=0）不参与微分 → 无微分冲击（微分先行）
    CHECK(near(pid.calc(0.0f, 0.1f, 1e-3f), -10.0f, 1e-4f));
}

static void test_pid_dt_guard() {
    ctl::PIDConfig cfg;
    cfg.gains_.ki_ = 1000.0f;
    cfg.limits_.limit_out_ = 1000.0f;
    cfg.limits_.limit_i_ = 1000.0f;
    ctl::PID pid(cfg);

    // 非法 dt（<=0 或 >0.5）被守卫替换为 1ms → 积分贡献仍按 1ms 计（0.5）
    CHECK(near(pid.calc(1.0f, 0.0f, 0.0f), 0.5f));
}

static void test_lpf() {
    ctl::LPF lpf(0.1f);
    // alpha = dt/(Tf+dt) = 0.1/(0.1+0.1) = 0.5
    CHECK(near(lpf.calc(1.0f, 0.1f), 0.5f));

    lpf.set_state(3.0f);                          // 状态注入后从注入值连续起步
    CHECK(near(lpf.calc(3.0f, 0.1f), 3.0f));

    ctl::LPF passthrough(0.0f);                   // Tf=0 → 直通
    CHECK(near(passthrough.calc(1.234f, 1e-3f), 1.234f));
}

static void test_ramp() {
    ctl::Ramp ramp(10.0f);
    // step = max_rate·dt = 1
    CHECK(near(ramp.calc(100.0f, 0.1f), 1.0f));
    CHECK(near(ramp.calc(100.0f, 0.1f), 2.0f));

    ramp.set_state(2.0f);
    CHECK(near(ramp.calc(-100.0f, 0.1f), 1.0f));   // 反向同样限速

    // ⚠ 现状语义：max_rate=0 → 输出冻结在 prev_（不是直通）。
    // 0 语义定案见 docs/roadmap.md §6 D-1；定案后本断言会变。
    ctl::Ramp frozen(0.0f);
    CHECK(near(frozen.calc(5.0f, 0.1f), 0.0f));
}

static void test_smooth_planner() {
    ctl::SmoothPlanner planner(100.0f, 0.01f);
    planner.set_state(1.0f);
    // 状态注入的连续性：目标=当前状态时不产生跳变
    CHECK(near(planner.calc(1.0f, 1e-3f), 1.0f, 1e-4f));

    planner.reset();
    CHECK(near(planner.calc(0.0f, 1e-3f), 0.0f));
}

static void test_deadzone() {
    ctl::Deadzone soft(0.5f, true);
    CHECK(near(soft.calc(0.3f), 0.18f, 1e-6f));    // e·(|e|/range) = 0.3·0.6
    CHECK(near(soft.calc(-0.3f), -0.18f, 1e-6f));  // 同号衰减
    CHECK(near(soft.calc(0.5f), 0.5f));            // 边界 |e|=range 连续
    CHECK(near(soft.calc(1.2f), 1.2f));            // 区外原样

    ctl::Deadzone hard(0.5f, false);
    CHECK(near(hard.calc(0.3f), 0.0f));            // 硬死区带内归零
    CHECK(near(hard.calc(-1.2f), -1.2f));

    ctl::Deadzone off(0.0f, true);                 // range=0 → 直通（无除零）
    CHECK(near(off.calc(0.7f), 0.7f));
}

static void test_pid_nan_recovery() {
    ctl::PIDConfig cfg;
    cfg.gains_.ki_ = 1000.0f;
    cfg.limits_.limit_out_ = 1000.0f;
    cfg.limits_.limit_i_ = 1000.0f;
    ctl::PID pid(cfg);

    // 第一拍：积分 = 1000·0.001·(1+0)/2 = 0.5，输出 0.5（成为“上一拍输出”）
    CHECK(near(pid.calc(1.0f, 0.0f, 1e-3f), 0.5f));
    // 非法输入：返回上一拍输出 0.5，且不更新任何状态（含积分/D 滤波/斜坡）
    CHECK(near(pid.calc(std::nanf(""), 0.0f, 1e-3f), 0.5f));   // cmd = NaN
    CHECK(near(pid.calc(1.0f, std::nanf(""), 1e-3f), 0.5f));   // measure = NaN
    CHECK(near(pid.calc(1.0f, 0.0f, std::nanf("")), 0.5f));    // dt = NaN
    // 恢复：状态没被污染，继续累积 → (1+1)/2 → +1.0 → 1.5
    CHECK(near(pid.calc(1.0f, 0.0f, 1e-3f), 1.5f));
}

static void test_pid_set_integral() {
    ctl::PIDConfig cfg;
    cfg.limits_.limit_out_ = 100.0f;
    cfg.limits_.limit_i_ = 10.0f;   // kp/ki/kd 全 0：输出 = 积分项
    ctl::PID pid(cfg);

    pid.set_integral(3.0f);
    CHECK(near(pid.calc(0.0f, 0.0f, 1e-3f), 3.0f));    // 注入即生效

    pid.set_integral(999.0f);                          // 注入值被 clamp 到 limit_i_
    CHECK(near(pid.calc(0.0f, 0.0f, 1e-3f), 10.0f));
}

static void test_pid_zero_means_unlimited() {
    // 0 语义统一（roadmap D-1）：limit_out_ / limit_i_ = 0 → 不限幅
    ctl::PIDConfig cfg;
    cfg.gains_.kp_ = 4.0f;
    cfg.gains_.ki_ = 200.0f;
    ctl::PID pid(cfg);
    float out = 0.0f;
    for (int k = 0; k < 20; ++k) { out = pid.calc(1.0f, 0.0f, 1e-3f); }
    CHECK(near(out, 7.9f, 1e-4f));   // 积分自由累积；旧语义（0=钳死）下这里会是 0

    // limit_i_ = 0 → 注入不被 clamp
    ctl::PIDConfig cfg2;
    ctl::PID pid2(cfg2);
    pid2.set_integral(999.0f);
    CHECK(near(pid2.calc(0.0f, 0.0f, 1e-3f), 999.0f));

    // limit_out_ > 0 仍限幅，而 limit_i_ = 0 时积分不被限
    ctl::PIDConfig cfg3;
    cfg3.gains_.ki_ = 500.0f;
    cfg3.limits_.limit_out_ = 1.0f;
    ctl::PID pid3(cfg3);
    float out3 = 0.0f;
    for (int k = 0; k < 10; ++k) { out3 = pid3.calc(1.0f, 0.0f, 1e-3f); }
    CHECK(near(out3, 1.0f));
}

int main() {
    test_pid_p_term();
    test_pid_trapezoid_integral();
    test_pid_derivative_on_measurement();
    test_pid_dt_guard();
    test_pid_nan_recovery();
    test_pid_set_integral();
    test_pid_zero_means_unlimited();
    test_lpf();
    test_ramp();
    test_smooth_planner();
    test_deadzone();

    if (g_failures == 0) {
        std::printf("ctlkit smoke test: all checks passed\n");
        return 0;
    }
    std::printf("ctlkit smoke test: %d check(s) failed\n", g_failures);
    return 1;
}
