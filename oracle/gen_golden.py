#!/usr/bin/env python3
"""gen_golden.py —— 用 oracle 生成 tests/golden/*.csv（提交进仓库，CI 无需 Python）。

容差怎么来（不许手拍，见 AGENTS.md §2）：
    dev   = max|oracle 在 float32 下的输出 − oracle 在 float64 下的输出|
            —— 同一算法、同一输入、同精度下的自身舍入上界（实测，不是估计）
    scale = max(1, max|float64 输出|)
    tol   = 8·dev + 1e-6·scale
            —— 8 = 安全系数（覆盖 C++ 侧算子顺序/常量折叠差异）；1e-6·scale = 绝对下限
文件头把 dev 与 tol 都写出来，C++ 测试读 tol 并打印实测偏差，便于审计。

用法：python3 oracle/gen_golden.py
"""

import hashlib
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, HERE)

from refctl import F32, F64, LPF, PID, Ramp, SmoothPlanner, Deadzone  # noqa: E402

GOLDEN_DIR = os.path.join(ROOT, "tests", "golden")
SAFETY = 8.0
ABS_FLOOR = 1e-6


def const(v, n):
    return [v] * n


def ladder(vals, n):
    return [vals[k % len(vals)] for k in range(n)]


def pid_rows(cmd_seq, measure_seq, dt_seq):
    return [(c, m, d) for c, m, d in zip(cmd_seq, measure_seq, dt_seq)]


def sim_rows(seq, dt_seq):
    return [(v, d) for v, d in zip(seq, dt_seq)]


def build_cases():
    cases = []

    def add(name, component, params, rows):
        cases.append({"name": name, "component": component, "params": params, "rows": rows})

    # ── PID ──────────────────────────────────────────────
    n = 24
    add("pid_p_only", "pid",
        {"kp": 4.0, "ki": 0.0, "kd": 0.0, "limit_out": 100.0, "limit_i": 0.0},
        pid_rows(const(1.0, n), ladder([0.0, 0.2, 0.5, 0.8, 0.95, 1.0, 1.0, 0.9], n), const(0.001, n)))

    n = 32
    add("pid_step_integral", "pid",
        {"kp": 4.0, "ki": 200.0, "kd": 0.0, "limit_out": 1000.0, "limit_i": 1000.0},
        pid_rows(const(1.0, n), ladder([0.0, 0.0, 0.1, 0.25, 0.45, 0.65, 0.8, 0.9, 0.96, 1.0], n),
                 const(0.001, n)))

    n = 30
    add("pid_sat_limit", "pid",
        {"kp": 20.0, "ki": 2000.0, "kd": 0.0, "limit_out": 1.0, "limit_i": 1.0},
        pid_rows(const(1.0, n),
                 [0.0] * 15 + [0.2] * 5 + [0.8] * 5 + [1.2] * 5, const(0.001, n)))

    n = 24
    add("pid_integral_separation", "pid",
        {"kp": 4.0, "ki": 200.0, "kd": 0.0, "limit_out": 100.0, "limit_i": 100.0,
         "thresh_i_sep": 0.5},
        pid_rows(const(1.0, n),
                 [0.0] * 4 + [0.2] * 4 + [0.6] * 4 + [0.75] * 4 + [0.9] * 4 + [1.05] * 4,
                 const(0.001, n)))

    n = 30
    add("pid_d_lpf_ramp", "pid",
        {"kp": 2.0, "ki": 50.0, "kd": 0.5, "limit_out": 50.0, "limit_i": 20.0,
         "max_rate_out": 5.0, "d_filter_Tf": 0.005},
        pid_rows(const(1.0, n),
                 ladder([0.0, 0.0, 0.5, 0.52, 0.51, 0.9, 0.92, 0.4, 0.38, 0.4], n),
                 const(0.001, n)))

    n = 16
    add("pid_dt_guard", "pid",
        {"kp": 2.0, "ki": 500.0, "kd": 0.1, "limit_out": 50.0, "limit_i": 50.0},
        pid_rows(const(1.0, n), const(0.3, n),
                 ladder([0.001, 0.0, -0.5, 2.0, 0.001, 0.6, 0.001, 0.0005], n)))

    # ── LPF ──────────────────────────────────────────────
    n = 24
    add("lpf_step", "lpf", {"Tf": 0.01},
        sim_rows([1.0] * 10 + [0.0] * 7 + [-1.0] * 7, const(0.001, n)))

    n = 16
    add("lpf_passthrough", "lpf", {"Tf": 0.0},
        sim_rows(ladder([0.5, 1.0, -0.25, 2.0, 0.0], n), const(0.002, n)))

    # ── Ramp ─────────────────────────────────────────────
    n = 20
    add("ramp_rate_limit", "ramp", {"max_rate": 10.0},
        sim_rows([100.0] * 6 + [-100.0] * 8 + [0.0] * 6, const(0.05, n)))

    n = 10
    add("ramp_frozen", "ramp", {"max_rate": 0.0},
        sim_rows(ladder([5.0, -3.0, 0.0], n), const(0.01, n)))

    # ── SmoothPlanner ────────────────────────────────────
    n = 24
    add("planner_step", "smooth_planner", {"max_rate": 5.0, "Tf": 0.01},
        sim_rows([1.0] * 12 + [0.0] * 12, const(0.001, n)))

    # ── Deadzone ─────────────────────────────────────────
    dz = ladder([-1.2, -0.5, -0.3, -0.1, 0.0, 0.1, 0.3, 0.5, 1.2], 15)
    add("deadzone_soft", "deadzone", {"range": 0.5, "soft": 1.0}, [(e,) for e in dz])
    add("deadzone_hard", "deadzone", {"range": 0.5, "soft": 0.0}, [(e,) for e in dz])
    add("deadzone_disabled", "deadzone", {"range": 0.0, "soft": 1.0},
        [(e,) for e in ladder([-1.0, -0.2, 0.0, 0.2, 1.0], 10)])

    return cases


def run_case(case, dtype):
    p = case["params"]
    if case["component"] == "pid":
        inst = PID(p.get("kp", 0.0), p.get("ki", 0.0), p.get("kd", 0.0),
                   p.get("limit_out", 0.0), p.get("limit_i", 0.0),
                   p.get("thresh_i_sep", 0.0), p.get("max_rate_out", 0.0),
                   p.get("d_filter_Tf", 0.0), dtype=dtype)
        return [float(inst.calc(c, m, dt)) for c, m, dt in case["rows"]]
    if case["component"] == "lpf":
        inst = LPF(p["Tf"], dtype=dtype)
        return [float(inst.calc(raw, dt)) for raw, dt in case["rows"]]
    if case["component"] == "ramp":
        inst = Ramp(p["max_rate"], dtype=dtype)
        return [float(inst.calc(cmd, dt)) for cmd, dt in case["rows"]]
    if case["component"] == "smooth_planner":
        inst = SmoothPlanner(p["max_rate"], p["Tf"], dtype=dtype)
        return [float(inst.calc(cmd, dt)) for cmd, dt in case["rows"]]
    if case["component"] == "deadzone":
        inst = Deadzone(p.get("range", 0.0), p.get("soft", 1.0) != 0.0, dtype=dtype)
        return [float(inst.calc(row[0])) for row in case["rows"]]
    raise ValueError(f"未知组件：{case['component']}")


def param_str(params):
    return " ".join(f"{k}={v}" for k, v in params.items())


def main():
    oracle_sha = hashlib.sha256(open(os.path.join(HERE, "refctl.py"), "rb").read()).hexdigest()[:12]
    os.makedirs(GOLDEN_DIR, exist_ok=True)

    print(f"oracle sha256[:12] = {oracle_sha}")
    print(f"{'case':28s} {'component':15s} {'rows':>4s} {'dev(f32-f64)':>13s} {'scale':>8s} {'tol':>10s}")

    for case in build_cases():
        exp64 = run_case(case, F64)
        exp32 = run_case(case, F32)
        dev = max(abs(a - b) for a, b in zip(exp32, exp64))
        scale = max(1.0, max(abs(v) for v in exp64))
        tol = SAFETY * dev + ABS_FLOOR * scale

        path = os.path.join(GOLDEN_DIR, case["name"] + ".csv")
        with open(path, "w", encoding="utf-8") as f:
            f.write("# ctlkit-golden v1\n")
            f.write(f"# case: {case['name']}\n")
            f.write(f"# component: {case['component']}\n")
            f.write(f"# oracle: oracle/refctl.py\n")
            f.write(f"# oracle-sha256: {oracle_sha}\n")
            f.write(f"# params: {param_str(case['params'])}\n")
            f.write(f"# tol: {tol!r}\n")
            f.write(f"# dev: {dev!r}\n")
            f.write(f"# scale: {scale!r}\n")
            cols = {"pid": "cmd measure dt expected",
                    "lpf": "raw dt expected",
                    "ramp": "cmd dt expected",
                    "smooth_planner": "cmd dt expected",
                    "deadzone": "error expected"}[case["component"]]
            f.write(f"# columns: {cols}\n")
            for row, exp in zip(case["rows"], exp64):
                f.write(" ".join([repr(float(v)) for v in row] + [repr(exp)]) + "\n")

        print(f"{case['name']:28s} {case['component']:15s} {len(case['rows']):4d} "
              f"{dev:13.3e} {scale:8.3f} {tol:10.3e}")

    print(f"\n✅ 已写入 {len(build_cases())} 个黄金向量 → {os.path.relpath(GOLDEN_DIR, ROOT)}/")
    return 0


if __name__ == "__main__":
    sys.exit(main())
