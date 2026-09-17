// ctlkit 冒烟测试（主机端）
//
// 定位：验证「库能编、基本行为符合 docs/spec/」的快速回归。
// 正式的黄金向量回归（逐拍快照、dt 扰动、饱和恢复、NaN 注入等）在 M0 建立，
// 见 docs/roadmap.md §8 验证与回归策略。
//
// 注意：本文件中的断言即当前行为契约的可执行版本；行为变更时必须同步改这里。

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
    cfg.kp_ = 2.0f;
    cfg.limit_out_ = 100.0f;
    ctl::PID pid(cfg);

    CHECK(near(pid.calc(1.0f, 0.0f, 1e-3f), 2.0f));   // u = kp·e
    CHECK(near(pid.calc(1.0f, 1.0f, 1e-3f), 0.0f));   // 误差归零 → 输出 0
}

static void test_pid_trapezoid_integral() {
    ctl::PIDConfig cfg;
    cfg.ki_ = 1000.0f;
    cfg.limit_out_ = 1000.0f;
    cfg.limit_i_ = 1000.0f;
    ctl::PID pid(cfg);

    // 梯形积分：i = ki·dt·(e + e_prev)/2 = 1000·0.001·(1+0)/2 = 0.5
    CHECK(near(pid.calc(1.0f, 0.0f, 1e-3f), 0.5f));
    // 第二拍：(1+1)/2 = 1 → 再 +1.0 → 1.5
    CHECK(near(pid.calc(1.0f, 0.0f, 1e-3f), 1.5f));
}

static void test_pid_derivative_on_measurement() {
    ctl::PIDConfig cfg;
    cfg.kd_ = 0.1f;
    cfg.limit_out_ = 100.0f;
    cfg.limit_i_ = 100.0f;
    ctl::PID pid(cfg);

    pid.calc(0.0f, 0.0f, 1e-3f);   // 建立 measure_prev_ 状态
    // d = -kd·(m - m_prev)/dt = -0.1·(0.1-0)/0.001 = -10
    // 设定值（cmd=0）不参与微分 → 无微分冲击（微分先行）
    CHECK(near(pid.calc(0.0f, 0.1f, 1e-3f), -10.0f, 1e-4f));
}

static void test_pid_dt_guard() {
    ctl::PIDConfig cfg;
    cfg.ki_ = 1000.0f;
    cfg.limit_out_ = 1000.0f;
    cfg.limit_i_ = 1000.0f;
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

int main() {
    test_pid_p_term();
    test_pid_trapezoid_integral();
    test_pid_derivative_on_measurement();
    test_pid_dt_guard();
    test_lpf();
    test_ramp();
    test_smooth_planner();

    if (g_failures == 0) {
        std::printf("ctlkit smoke test: all checks passed\n");
        return 0;
    }
    std::printf("ctlkit smoke test: %d check(s) failed\n", g_failures);
    return 1;
}
