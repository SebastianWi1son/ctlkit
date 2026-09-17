---
class: fact
generated: false
---
# 调研归档索引（原「PID 对比归档」）

> 归档目的：对比 [Liu-Curiousity/pid](https://github.com/Liu-Curiousity/pid)（GitHub 公开库）与本库
> `ctlkit`（原 `cyclotron/foc` 自研 PID，源自 lunokhod wheel）的实现差异，沉淀**优化考量**清单。
> 本目录是 `docs/research/`：回答「为什么这么设计」（允许过期）；
> 现状契约在 `../spec/`，设计稿在 `../design/`，排期在 `../roadmap.md`。

**归档日期**：2026（本次分析）
**上游源码版本**：
- GitHub：`V1.2.2`，commit `d0b1045e362b3925cbe9b98968c79e9c171d98ef`（2026-08-27）
- 本地：`cyclotron/foc/inc/foc/algo/pid.hpp` + `src/foc/algo/pid.cpp`（及其依赖 lpf / ramp）

## 目录结构

```
docs/research/
├── README.md                      # 本文件（归档索引）
├── comparison_report.md           # 对比报告（架构 / 公式 / 行为逐项对照 + 结论）
├── optimization_considerations.md # 优化考量（可落地的改进点 + 伪码 + 风险 + 优先级）
└── reference/                     # 第三方/历史源码只读快照（不参与构建）
    ├── github_Liu-Curiousity/     # GitHub 源码原样快照（PID.h / CMakeLists.txt + 溯源说明）
    └── cyclotron_foc/             # cyclotron 源码原样快照（pid / lpf / ramp 的 hpp+cpp + 溯源说明）
```

> 活代码（库化后）在仓库的 `../../inc` + `src/`；本目录两份快照仅供血缘考古， **不以它们为准**。

## 快速结论

| 维度 | GitHub (Liu-Curiousity/pid) | 本库 (ctl::PID，原 foc::algo::PID) |
|---|---|---|
| 定位 | 通用教学型 PID 库（header-only，双模式） | 面向 FOC 电流环的工业级单模式组件 |
| D 项 | 误差微分（有冲击），可选外部 `dinput` | **微分先行**（无冲击）+ D 项 LPF |
| I 项 | 矩形积分 + 无条件限幅 | **梯形积分 + 积分分离 + 预限幅** |
| 抗饱和 | 静态 clamp（增量式长期饱和会 windup） | 静态 clamp + 分离（更强，仍无 back-calculation） |
| dt | 构造固定 `Ts` | 逐次传入 + 守卫 `(0, 0.5]` |
| 状态 | 公开可写（含 `set_sum_error` 注入） | 私有，`reset()` 清零（LPF/Ramp 有 `set_state`，PID 无） |
| 限幅 | 独立非对称 ± 限幅（optional） | 对称 ± 限幅 |
| 特色 API | `[[nodiscard]]`、删默认构造、外部微分注入、积分注入 | config 全 0 禁用语义、无 STL、纯 float |

**值得借鉴的 4 个点**（详见优化考量）：
1. 外部微分注入（dinput）—— 与已有观测器（如 `angle_tracking::vel_`）配合，避免二次差分噪声
2. 积分状态注入（set_sum_error）—— 补全 PID 的 bumpless transfer（对齐 LPF/Ramp 的 `set_state`）
3. 非对称独立 ± 限幅 —— 适配再生/单象限等不对称工况
4. `[[nodiscard]]` / 接口健壮性习惯

**优化路线**：详见 `../roadmap.md` —— 基于主流库调研（ArduPilot / SimpleFOC / ODrive /
MathWorks 等）的分层目标架构（原语 → 核心 → 增强 → 系统）与 M0~M4 里程碑；
核心公式冻结不动，重点是注入口 / 观测口 / 抗饱和升级 / 前馈与目标滤波。

## 注意事项

- GitHub 仓库**未附带 LICENSE 文件**（头文件标注 `(c) 2025 QDrive`），此处仅作内部学习/对比归档，不直接引入商用代码。
- 本地源码为 cyclotron 内原样快照（归档）；自 v0.0.1 起 **上游库 ctlkit 为准**（活代码 `../../inc` + `src/`，
  行为契约 `docs/spec/`），下游（cyclotron / lunokhod）改为从本库同步，不再各自演化。
