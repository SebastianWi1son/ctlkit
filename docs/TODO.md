---
class: fact
generated: false
---
# TODO — 已核实的待办问题（暂不影响当前使用）

> 来源：2026-09-17 交叉审计（另一 agent 出题）+ 本仓逐条复现/实测。
> **收件标准**：① **已核实**（有证据、可复现）② **不影响当前使用**（不阻塞下游接入，也不动 v0.1.0 冻结面）。
> 影响使用的当批修掉（见文末「已修」），这里只留**未修**项。
> 优先级：**P1** 修法廉价且属承诺 / 语义缺口 · **P2** CI 看守缺口 · **P3** 交付链与覆盖缺口。

## 待办

> 2026-09-17：以下 T1~T9 + T2b **全部已补**（见文末「已修」批次的第二批）。本表保留原始证据，便于回查。

| # | 级别 | 问题 | 证据（可复现） | 影响当前使用？ | 建议修法 |
|---|---|---|---|---|---|
| T1 | P1 | **黄金向量证"没变"、不证"对"**：链路同源（源码 → 人工抄 spec → oracle → 黄金），公式层面的错误会一路通过 | 逻辑论证；本次反证实测：Tustin 对 ∫t² 的误差随 dt 减半 ÷4（ratio **4.00 / 3.99**），矩形参考 ÷2（**2.01**）→ 目前公式阶数正确 | 否（公式来自硬件验证过的血缘） | 加**公式级锚点**：① 收敛阶（Tustin ÷4 vs 矩形 ÷2，探针已就绪）② 闭环指标上界（超调 / 上升时间）③ 跨离散化家族对照（`scipy.signal.lfilter` 只做 AR/MA 校验，不构成公式判据） |
| T2 | P1 | **dt 极小 → D 状态被静默污染**：`inv_dt = 1/dt` 溢出为 inf，下一拍 `0·inf = NaN`；`input_fault()` 全程不置位 | 实测 Tf=0：dt=1e-45 → `d_term=-inf`，下一拍 `-nan`，此后永久 NaN 且 fault=0；Tf>0 时 ±inf 携带（不清零）；上界侧 dt=+inf 正常（fault=1） | 否（需 dt < ~1e-38，现实到不了；但违反"零污染"承诺） | 守卫加下界：`dt < 1e-9f` 与 `dt <= 0` 同路（一行）；顺带可选：`dt_rejected` 位（现在 dt 被判非法换成 1ms 完全不可观测） |
| T2b | P3 | T2 的二阶效应：关闭斜坡时速率归一化为 `FLT_MAX`，在极小 dt 下**反而成为真实速率限制**；`-inf` 还会让 `out_saturated_` 误报 | 实测 dt=1e-45：输出被限到 `-4.77e-07`、`out_sat=1` | 否（同 T2 触发条件） | 与 T2 同批（守好 dt 下界即消除） |
| T3 | P1 | **`i_saturated_` 语义偏直觉**：表示"本拍 I 项**候选值**被削"，不是"积分器饱和" | 实测：积分分离冻结（error≫thresh）时候选 1.0 被削到 0.5 → `i_saturated_=1` 而 `integral_=0` | 是（M2/F3 要拿它当判据）→ **本轮已把语义写进 spec**，故不再阻塞 | 若要更贴名：仅在候选值**被采纳**（未被分离丢弃）时置位 —— 属行为变更，进 minor 且需决策 |
| T4 | P2 | **CI 不验证 C++11 承诺**：`cxx_std_11` 表达"至少 11"，GCC 13 默认 gnu++17 → `__attribute__((warn_unused_result))` 兜底分支永不编译 | `flags.make` 里无 `-std`；实测 `-std=c++11` 下整套测试 **全绿 0 warning**（承诺今天成立，只是没看守） | 否 | build-test 加矩阵 `{c++11, c++17} × {gcc, clang}` + `-Werror` |
| T5 | P2 | **CI 不锁 golden ↔ oracle 同步**：改了 `oracle/refctl.py` 忘了 `oracle/gen_golden.py`，CI 不响 | 当前二者一致（重生成无 diff）；`scripts/ci_local.py` 顶部注释里提到的 golden job 实际不存在（模板残留） | 否 | 加可选 job：装 numpy/scipy → 跑 `gen_golden.py` → `git diff --exit-code tests/golden/`；并把 `ci_local.py` 的 stale 注释改掉 |
| T6 | P3 | **来源戳机制空转**：文档承诺拷贝时打 `// from ctlkit vX.Y.Z`，但源码里一行戳都没有；而 `scripts/downstream_diff.py` 已按戳忽略 | `grep 'from ctlkit' inc src` 无结果 | 否（但下游演练（roadmap D-6）要靠它） | 演练前定稿：整目录 vendor 时**戳可省**（推荐，`migration_WORK` §2.1 方式 ②）；或按文件加戳 |
| T7 | P3 | **版本只活在 `CMakeLists.txt` 与 CHANGELOG**：下游拷走文件后无法自证版本 | 无 `install()` / CPack / 版本宏（grep = 0） | 否 | 加版本宏（⬜ 计划新头文件）或 `ctl::version()`；可选 install 规则 |
| T8 | P3 | **未覆盖路径**：`set_state`（仅 smoke 解析断言）、`LPF`/`Ramp` 的 `dt<=0`、`Deadzone` 的 NaN；`Ramp` 收 NaN dt 会**静默**原样输出 | 实测 `Ramp(10).calc(5, NaN) = 5` | 否（"调用方保证 dt>0"已写进 spec） | 补用例或补守卫（与 T2 同批考虑） |
| T9 | P3 | **斜坡约束不可观测**：`out_saturated_` 只反映 `limit_out_` 钳位，斜坡减速 / 冻结不算 | 设计如此（D-3 故意不报斜坡） | 否（但 M2/F3 别拿它当"输出未受限"） | M2 条件积分另设判据（如"斜坡是否在削"） |

## 已修（2026-09-17 同批，均为文档契约类，不动代码行为）

- **spec 漏整块 M1 观测接口** → `docs/spec/pid.md` 补：接口表 4 行（`set_gains` / `get_state` / `status` / `input_fault`）、
  新增「观测出口」节（含 `i_saturated_` 的精确语义与陷阱）、状态清单与 `reset()` 行为、
  非法输入节删掉 stale 的"待 M1 补"、已知缺口表删掉已落地两项、变更历史补 M1 行
- **`docs/spec/README.md` 的 A2 stale 行**（"PID 暂无状态注入"）→ 改为 `set_integral` / `set_gains` 已落地
- **同一可变量多处漂移**（README「4 个组件」、oracle/README「16 个用例」、roadmap「11 个 / 18 例」）→
  散文里的**可数数字一律删掉**，只留 `tests/golden/` 目录作为唯一事实源（脚本派生记 T5/T7 之外，暂靠约定）
- **`pid_types.hpp` 注释与 `set_gains` 矛盾**（"构造后不可变"）→ 注释改为"只有 `gains_` 可在线改"

### 第二批（2026-09-17，「全部你来补」；代码改动的 before/after 见对应提交说明）

- **T1 已补**：`tests/smoke_test.cpp` 加三个**公式级锚点**（期望值全部是外部解析真值，非本库产出）：
  ① 收敛阶 Tustin ÷4.00/3.99 vs 矩形参考 ÷2.01（容差 ±12.5%）② 二阶闭环 T(s)=100/(s+100) 解析无超调 / t_r=21.97 ms
  （实测 21.90 ms，带宽 ±5%）③ 抗饱和：反向误差后 ≤5 拍离开饱和（实测 3 拍；无抗饱和需 ~1000 拍）
- **T2 / T2b 已补**：dt 守卫加下界 `dt < 1e-9`（`src/pid.cpp`）+ 新增 `status().dt_rejected_`（本拍是否被守卫兜底）
- **T3 已补**：`i_saturated_` 改为「只报被采纳的钳位」（分离冻结不算）；并按用户决定**补上分离出口** `status().i_frozen_`（方案 B："顶住了吗"与"这拍积分了吗"各一个位，互斥）——spec / oracle / smoke 同步
- **T4 已补**：CI build-test 改矩阵 `{g++, clang++} × {c++11, c++17}` + `-Werror`
- **T5 已补**：CI 新增 `golden-sync` job（validate_oracle → gen_golden → `git diff --exit-code tests/golden/`）；
  顺手改掉 `scripts/ci_local.py` 里提到"不存在的 golden job"的 stale 注释
- **T6 / T7 已补**：新增 `inc/ctl/version.hpp`（版本唯一事实源：`CTL_VERSION_MAJOR/MINOR/PATCH` + 派生 `CTL_VERSION_STRING`）；
  `CMakeLists.txt` 改从它解析 `project(VERSION)` —— 下游拷贝后可自证版本，来源戳不再是唯一手段
- **T8 已补**：`tests/smoke_test.cpp` 新增 `test_uncovered_paths` 钉住现状（LPF `dt=0` 冻结、LPF/Deadzone 的 NaN 透传、
  Ramp 的 NaN dt 原样透传）；`docs/spec/lpf.md` 与 `docs/spec/ramp.md` 各补一句
- **T9 已定**：不修（D-3 设计如此：斜坡不是饱和）；spec「观测出口」节已写明"只报钳位，不报斜坡与分离"

## 记录

| 日期 | 动作 |
|---|---|
| 2026-09-17 | 交叉审计（P0~P3 共 11 条）逐条复现/实测：真 11 / 假 0 / 部分 0（其中 4 条影响使用 → 当批修；其余入账） |
