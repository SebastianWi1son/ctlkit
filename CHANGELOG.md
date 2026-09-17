---
class: log
generated: false
---
# Changelog

遵循语义化版本。**v0.1.0 起 API 冻结**（兼容政策见 README「兼容政策」）：
既有字段名 / 语义 / `calc` 签名只增不改，破坏性变更进 major。
v0.0.x 是库化与加固阶段（M0 之前，无兼容承诺）。路线见 `docs/roadmap.md`。

## [Unreleased]

### Added
- `status().i_frozen_`：分离冻结出口（本该拍因积分分离而未积分；与 `i_saturated_` 互补且互斥）

### Changed
- `status().i_saturated_`：改为「只报**被采纳**的钳位」（分离冻结不再计入）——修正一处误导性上报

## [0.1.0] - 2026-09-17

### Changed
- **归档快照移出仓库**：第三方（`Liu-Curiousity/pid`）与历史（`cyclotron_foc`）源码快照
  不再随仓库分发，本地保留在 `docs/research/reference/`（已 gitignore），历史一并清理

### 冻结（Freeze）
- **API 冻结**：`calc(cmd, measure, dt, const PIDPorts *ports = nullptr)` 签名、
  `PIDConfig` / `PIDPorts` / `PIDState` / `PIDStatus` 的既有字段名与语义、五个组件的公开接口。
  新特性只允许“加字段 / 加组件 / 加重载”，破坏进 major。
- 支撑类型拆到 `inc/ctl/pid_types.hpp`（`pid.hpp` 只留类），此文件布局同为冻结面。


### Added
- **`Deadzone` 库化**：`inc/ctl/deadzone.hpp` + `src/deadzone.cpp`（原 `foc::algo::Deadzone`，行为未变）
  + `docs/spec/deadzone.md` 契约 + oracle/黄金向量覆盖（软/硬/range=0 三路径）
- **PID M0（部分）**：NaN/Inf 守卫（返回上一拍输出、零污染状态）、`set_integral`（clamp 注入）、
  `CTL_NODISCARD`（C++11 兜底宏）—— 有限输入行为不变，黄金回归 14/14 仍绿
- **PID M0：`0` 语义统一（roadmap D-1）**：限幅类 `<= 0` = 不限幅（原“0 = 钳死到 0”）；
  斜坡 “0 = 关闭” 在构造期归一化为无上限速率（不动 `Ramp` 的 `0 = 冻结`）。
  ⚠ 行为变更点：`limit = 0` 的既有配置；新增控覆盖 `pid_limit_unlimited` / `pid_i_unlimited_out_limited`（黄金向量 16 例）
- **PID 配置分组**：`PIDConfig` 扁平 8 字段 → `PIDGains` / `PIDLimits` / `PIDTunings`
  （`PIDConfig` 成员 `gains_` / `limits_` / `tunings_`；字段名不变、无旧路径别名，行为逐字等价）
- **唯一可信 oracle**：`oracle/refctl.py`（按 `docs/spec/` 独立重写）+
  `oracle/validate_oracle.py`（外借尺子：`scipy.signal.lfilter` / `fractions.Fraction` 精确有理数 / 解析式）
  + `oracle/gen_golden.py`（生成器）；说明见 `oracle/README.md`
- `tests/golden/*.csv`：11 个黄金向量用例（容差 `tol = 8·dev + 1e-6·scale` 逐用例推导并记录在头部）
- `tests/golden_test.cpp`：黄金向量回归（PID 饱和/积分分离/D 滤波/斜坡/dt 守卫、LPF、Ramp、SmoothPlanner）
- 文档体系与门禁：`AGENTS.md` · `docs/README.md` · `scripts/check_docs.py` ·
  `scripts/doc_lint_baseline.txt` · `scripts/ci_local.py` · `.github/workflows/ci.yml`（docs-gate + build-test）

### Notes
- 判别力已实测：4 个变异（PID 梯形系数/微分符号、LPF alpha、Ramp 上界）全部变红，
  实测偏差为容差的 2~80 倍；记录见 `oracle/README.md` §6
- 组件清单：PID · LPF · Ramp · SmoothPlanner · Deadzone（共 5 个，全部接入 CMake/spec/oracle）

## [0.0.1] - 2026-09-17

### Added
- 库化骨架：`inc`（公开头）与 `src/`（实现）分离；CMake 主机构建；`examples/`；`tests/`（冒烟测试）
- 组件纳入：`PID`、`LPF`、`Ramp`、`SmoothPlanner`（第 4 个来自 cyclotron，含 `set_state`）
- 文档体系三层：`docs/spec/`（现状契约）、`docs/design/`（设计稿）、`docs/research/`（调研归档，含第三方只读快照）
- `docs/roadmap.md`：M0~M4 路线图 + 工时估算 + 决策点（由调研蓝图升级）

### Changed
- 命名空间 `foc::algo` → `ctl`；包含路径 `algo/*.hpp` → `ctl/*.hpp`（**算法行为不变，与 cyclotron/foc 快照逐字一致**）
- 归档重组：原 `cyclotron_foc/`、`reference/` → `docs/research/reference/`（只读，不参与构建）

### Notes
- 未做任何算法/公式改动：本版本只确定结构与命名（上游库地基）
- 下游（cyclotron/foc、lunokhod/actuator/wheel）的接入迁移不在本版本范围内，见 `docs/roadmap.md` §6 D-6
