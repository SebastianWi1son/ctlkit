// tests/golden_test.cpp —— 黄金向量回归（期望值 100% 来自 oracle）
//
// 规矩（见 AGENTS.md §2 oracle 规则）：
//   · 期望值不得由被测对象算出 —— 本文件只读 tests/golden/*.csv（oracle 生成），
//     不含任何手写数值断言；
//   · 容差来自 CSV 头部记录的推导值：
//       tol = 8·(oracle float32 自身偏差) + 1e-6·scale
//     （推导过程见 oracle/gen_golden.py 文件头）；
//   · 判别力：故意改坏 inc/ 下一行实现 → 本测试必须变红（见 oracle/README.md）。

#include <ctl/deadzone.hpp>
#include <ctl/lpf.hpp>
#include <ctl/pid.hpp>
#include <ctl/ramp.hpp>
#include <ctl/smooth_planner.hpp>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string trim(const std::string &s) {
    const std::size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return std::string();
    const std::size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

struct Case {
    std::string name;
    std::string component;
    std::map<std::string, float> params;
    double tol;
    std::vector<std::vector<float> > rows;   // 输入字段 + 末列 = expected

    Case() : tol(0.0) {}
};

bool parse_file(const std::string &path, Case &c) {
    std::ifstream in(path.c_str());
    if (!in) {
        std::printf("FAIL 无法打开黄金文件：%s\n", path.c_str());
        return false;
    }
    std::string line;
    while (std::getline(in, line)) {
        if (trim(line).empty()) continue;
        if (line[0] == '#') {
            const std::size_t colon = line.find(':');
            if (colon == std::string::npos) continue;
            const std::string key = trim(line.substr(1, colon - 1));
            const std::string val = trim(line.substr(colon + 1));
            if (key == "case") {
                c.name = val;
            } else if (key == "component") {
                c.component = val;
            } else if (key == "tol") {
                c.tol = std::strtod(val.c_str(), nullptr);
            } else if (key == "params") {
                std::istringstream ss(val);
                std::string kv;
                while (ss >> kv) {
                    const std::size_t eq = kv.find('=');
                    if (eq == std::string::npos) continue;
                    c.params[kv.substr(0, eq)] = std::strtof(kv.substr(eq + 1).c_str(), nullptr);
                }
            }
            continue;
        }
        std::istringstream ss(line);
        std::vector<float> row;
        float v;
        while (ss >> v) row.push_back(v);
        if (!row.empty()) c.rows.push_back(row);
    }
    return true;
}

float param(const Case &c, const std::string &k, float dflt = 0.0f) {
    const std::map<std::string, float>::const_iterator it = c.params.find(k);
    return it == c.params.end() ? dflt : it->second;
}

struct Outcome {
    int bad;
    double max_dev;

    Outcome() : bad(0), max_dev(0.0) {}
};

void check_row(float got, float expected, double tol, Outcome &o) {
    const double dev = std::fabs(static_cast<double>(got) - static_cast<double>(expected));
    if (dev > o.max_dev) o.max_dev = dev;
    if (!(dev <= tol)) {
        ++o.bad;
        if (o.bad <= 3) {
            std::printf("      · 超差行：got=%.9g expected=%.9g dev=%.3e > tol=%.3e\n",
                        static_cast<double>(got), static_cast<double>(expected), dev, tol);
        }
    }
}

bool run_case(const Case &c, Outcome &o) {
    std::size_t i;
    if (c.component == "pid") {
        ctl::PIDConfig cfg;
        cfg.gains_.kp_ = param(c, "kp");
        cfg.gains_.ki_ = param(c, "ki");
        cfg.gains_.kd_ = param(c, "kd");
        cfg.limits_.limit_out_ = param(c, "limit_out");
        cfg.limits_.limit_i_ = param(c, "limit_i");
        cfg.tunings_.thresh_i_sep_ = param(c, "thresh_i_sep");
        cfg.tunings_.max_rate_out_ = param(c, "max_rate_out");
        cfg.tunings_.d_filter_Tf_ = param(c, "d_filter_Tf");
        ctl::PID inst(cfg);
        for (i = 0; i < c.rows.size(); ++i) {
            if (c.rows[i].size() != 4) return false;
            check_row(inst.calc(c.rows[i][0], c.rows[i][1], c.rows[i][2]), c.rows[i][3], c.tol, o);
        }
    } else if (c.component == "lpf") {
        ctl::LPF inst(param(c, "Tf"));
        for (i = 0; i < c.rows.size(); ++i) {
            if (c.rows[i].size() != 3) return false;
            check_row(inst.calc(c.rows[i][0], c.rows[i][1]), c.rows[i][2], c.tol, o);
        }
    } else if (c.component == "ramp") {
        ctl::Ramp inst(param(c, "max_rate"));
        for (i = 0; i < c.rows.size(); ++i) {
            if (c.rows[i].size() != 3) return false;
            check_row(inst.calc(c.rows[i][0], c.rows[i][1]), c.rows[i][2], c.tol, o);
        }
    } else if (c.component == "smooth_planner") {
        ctl::SmoothPlanner inst(param(c, "max_rate"), param(c, "Tf"));
        for (i = 0; i < c.rows.size(); ++i) {
            if (c.rows[i].size() != 3) return false;
            check_row(inst.calc(c.rows[i][0], c.rows[i][1]), c.rows[i][2], c.tol, o);
        }
    } else if (c.component == "deadzone") {
        ctl::Deadzone inst(param(c, "range"), param(c, "soft", 1.0f) != 0.0f);
        for (i = 0; i < c.rows.size(); ++i) {
            if (c.rows[i].size() != 2) return false;
            check_row(inst.calc(c.rows[i][0]), c.rows[i][1], c.tol, o);
        }
    } else {
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char **argv) {
    if (argc < 2) {
        std::printf("FAIL 没有传入黄金向量（应由 CMake 收集 tests/golden/*.csv 作为参数）\n");
        return 2;
    }

    int total_bad = 0;
    std::printf("%-28s %-15s %5s %12s %12s\n", "case", "component", "rows", "max_dev", "tol");
    std::printf("%-28s %-15s %5s %12s %12s\n", "----", "---------", "----", "-------", "---");

    for (int k = 1; k < argc; ++k) {
        Case c;
        Outcome o;
        if (!parse_file(argv[k], c)) return 2;
        if (!run_case(c, o)) {
            std::printf("FAIL 用例无法执行（component 未知或列数不符）：%s\n", argv[k]);
            return 2;
        }
        const bool ok = (o.bad == 0);
        if (!ok) total_bad += o.bad;
        std::printf("%-28s %-15s %5d %12.3e %12.3e %s\n", c.name.c_str(), c.component.c_str(),
                    static_cast<int>(c.rows.size()), o.max_dev, c.tol, ok ? "OK" : "FAIL");
    }

    std::printf("\n");
    if (total_bad) {
        std::printf("❌ 黄金向量回归失败：%d 行超差\n", total_bad);
        return 1;
    }
    std::printf("✅ 黄金向量回归全部通过（期望值来源 oracle/refctl.py；容差见各 CSV 头部）\n");
    return 0;
}
