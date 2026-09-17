# Changelog

遵循语义化版本。v0.0.x 为库化与加固阶段（M0 之前），尚未承诺跨版本行为兼容；
行为兼容承诺自 M0（黄金向量回归建立）起生效。路线见 `docs/roadmap.md`。

## [0.0.1] - 2026-09-17

### Added
- 库化骨架：`include/ctl/`（公开头）与 `src/`（实现）分离；CMake 主机构建；`examples/`；`tests/`（冒烟测试）
- 组件纳入：`PID`、`LPF`、`Ramp`、`SmoothPlanner`（第 4 个来自 cyclotron，含 `set_state`）
- 文档体系三层：`docs/spec/`（现状契约）、`docs/design/`（设计稿）、`docs/research/`（调研归档，含第三方只读快照）
- `docs/roadmap.md`：M0~M4 路线图 + 工时估算 + 决策点（由调研蓝图升级）

### Changed
- 命名空间 `foc::algo` → `ctl`；包含路径 `algo/*.hpp` → `ctl/*.hpp`（**算法行为不变，与 cyclotron/foc 快照逐字一致**）
- 归档重组：原 `cyclotron_foc/`、`reference/` → `docs/research/reference/`（只读，不参与构建）

### Notes
- 未做任何算法/公式改动：本版本只确定结构与命名（上游库地基）
- 下游（cyclotron/foc、lunokhod/actuator/wheel）的接入迁移不在本版本范围内，见 `docs/roadmap.md` §6 D-6
