---
class: work
generated: false
accepted: false
---
# migration_WORK — 下游接入演练（把下游切到 ctlkit v0.1.0）

> **类：D 施工单** —— 演练验收后移出 `docs/`（或标 `accepted: true` 交给门禁拦）。
> 目标：证明「拷贝 + 来源戳 + diff 校验」这条消费链路真的跑得通，且**行为不变**。
> 前置：ctlkit v0.1.0 已冻结（API + 黄金 19 例）；**下游开工前必须 git 干净或先提交基线**（回滚用）。

## 1. 演练对象与顺序

| 顺序 | 下游 | 理由 | 预估 |
|---|---|---|---|
| ① | cyclotron/foc | 五个组件全在用（PID/LPF/Ramp/SmoothPlanner/Deadzone）；上游 spec 已接管数学权威 | ✅ **2026-09-17 完成**（记录见 §4） |
| ② | lunokhod/actuator/wheel | 血缘源头（自带一份 `pid.hpp`/`smooth_planner.hpp`）；① 验过的流程直接套 | ~1h |

> **计划外消费者**（本机扫描发现，均用平铺具名字段写法，不在本单范围内）：`KND_Trial/app`、`fw_poc`、`lunokhod/chassis_loop`。
> （外部仓库只写目录，不写其中的文件名 —— 门禁 R5 按本仓相对路径校验，写文件名会踩空）三者的 `ki_`/`limit_i_` 组合经核对均不在 0 语义雷区（见 §2.3），待接入时另行开单。

## 2. 步骤（每个下游重复一遍）

### 2.0 冻结基线

```bash
cd <下游> && git status --short     # 必须干净；不干净先提交
git rev-parse HEAD                  # 记到 §4 记录表（回滚用）
```

### 2.1 拷贝（两种方式选一，推荐 ②）

| 方式 | 做法 | 校验 |
|---|---|---|
| ① 散拷贝（roadmap D-6 原方案） | 按下游现有布局散放文件，每个文件头加来源戳 `// from ctlkit vX.Y.Z` | `scripts/downstream_diff.py`（无 vendor 目录时全树按文件名比对） |
| ② 整目录 vendor | 下游建 `third_party/ctlkit/`，上游 `inc/` + `src/` 整个放进去 + `VERSION` 记来源（**记实际来源**：发布版记 `vX.Y.Z (sha)`，未发布就记分支 + sha，别写成版本号骗人） | 同上（vendor 目录逐字比对） |
| ③ **vendor + 转发头（本次采用）** | 在 ② 基础上，把下游旧位置的 `algo/*.hpp` 改成**转发头**：`#include "ctl/x.hpp"` + `using ctl::X;`，文件里带标记行 `// ctlkit-forwarder` | 校验脚本除 vendor 比对，还要求旧位置的同名文件**要么删掉、要么带标记且真转发** |

**③ 相对 ② 多买到两样**：下游调用点（含应用侧）的 `#include "algo/pid.hpp"` 与 `foc::algo::PID` 等名字**都不用改**；
升级上游 = 只重拷 vendor 目录。代价是多一层 5 个文件的转发头（每个 ~10 行，不含逻辑）。

回滚：删 `third_party/ctlkit/` + `git checkout <基线 sha> -- .`。

### 2.2 接构建

- include 指向 `.../ctlkit/inc`；编译 `.../ctlkit/src/*.cpp`。
- **用转发头（③）时**：下游源码的 include 与命名空间都不用改，只需把上游 `inc/` 加进 include 路径。
- 不用转发头时：包含路径从 `foc/algo/pid.hpp` 改成 `ctl/pid.hpp`（其余组件同理）。

### 2.3 改调用点（唯一要动业务代码的地方）

- 命名空间：`foc::algo::PID` → `ctl::PID`（LPF/Ramp/SmoothPlanner/Deadzone 同理）。**用转发头方案时这步免了**：
  旧名保留为别名（`using ctl::PID;`）。
- ⚠ **配置写法要动**（③ 也躲不掉）：字段从平铺变分组（`cfg.limit_out_` → `cfg.limits_.limit_out_`）。
  两种可选写法：① 逐字段（`cfg.limits_.limit_out_ = 3.0f;`，编译器逐条点名，散点最省心）；
  ② 上游 v0.1.1 起的**具名链式**（`PIDConfig{}.kp(2.0f).ki(50.0f).limit_out(3.0f).limit_i(3.0f)`，密集块最清楚）。
  位置初始化（`PIDConfig{a, b, ...}`）**仍能编译**（组内顺序 = 老顺序），但只能靠数位置读 → 新代码别再这么写。
- 已知调用点（cyclotron）：`current_loop.cpp` 的 iq/id 电流环 —— **签名向后兼容，`calc(cmd, measure, dt)` 不用改**；`foc.hpp` 的 `algo::Deadzone deadzone_`。
- ⚠ **0 语义变更点**：`limit = 0` 的配置从"钳死到 0"变成"不限幅"——逐个核对下游配置（已知 FOC 电流环是非零限幅，不受影响）。`max_rate_out = 0` 含义不变（仍是"关闭斜坡"）。

### 2.4 行为校验（验收，三条都要过）

```bash
# ① 库自身：黄金回归 19/19
cd <ctlkit> && ctest --test-dir build --output-on-failure
# ② 下游：构建 + 下游自己的测试锚点
cd <下游> && cmake --build build && ctest --test-dir build
# ③ 副本一致性：逐字比对（自动忽略来源戳行）
python3 <ctlkit>/scripts/downstream_diff.py <下游>          # 自动识别 vendor 目录 / 转发头 / 散拷贝
python3 <ctlkit>/scripts/downstream_diff.py --selftest      # 先自测工具本身（7 例，含改坏必红）
```

### 2.4.1 行为零漂移的最硬证据（可选但推荐）

下游测试通常只断言、不打印，数字看不见 → 用**下游自带的 demo 逐字节比对**补一刀：

```bash
# 迁移前的版本放进临时 worktree，两边跑同一个 demo，diff 输出
git worktree add --detach /tmp/pre <迁移前 sha> && cmake -S /tmp/pre -B /tmp/pre/b && cmake --build /tmp/pre/b -j
/tmp/pre/b/example_foc > /tmp/pre.txt 2>&1
cmake --build build -j && ./build/example_foc > /tmp/post.txt 2>&1
diff /tmp/pre.txt /tmp/post.txt && echo 行为零漂移
```

### 2.5 记录与回滚

- 记录：上游 sha + 下游基线 sha + 版本（`inc/ctl/version.hpp` 为准）+ 三条校验结论 → §4。
- 回滚：散拷贝方式 `git checkout <基线 sha> -- .`；vendor 方式直接删 `third_party/ctlkit/`。

## 3. 风险与对策

| 风险 | 对策 |
|---|---|
| 双活副本再次分叉（历史上发生过） | 迁移后下游**只改调用点**；库文件一律从上游拷，diff 脚本守门；脚本对旧位置的同名文件**强制**「删掉或带 `// ctlkit-forwarder` 标记」——不留未标记副本 |
| 0 语义变更漏检 | §2.3 核对清单 + 下游实测 |
| 命名空间切换漏改 | 编译器会抓；`using` 可减少改动面 |
| 下游测试锚点不足 | 迁移前后各跑一遍，数字逐条对比（FOC_MATH_SPEC 的锚点公式） |

## 4. 演练记录（边做边填）

| 日期 | 下游 | 上游版本/sha | 下游基线 sha | 方式 | ① 库 19/19 | ② 下游测试 | ③ diff | 结论 |
|---|---|---|---|---|---|---|---|---|
| 2026-09-17 | cyclotron/foc | `dev/v0.1.1-core` @ `a1d83c4`（未发布：v0.1.0 + i_frozen_ + 链式设置器） | `13570d0`（文档体系重排） | ③ vendor + 转发头 | —（目标仓非本库） | ✅ 5/5 + **demo 输出 33 行逐字节相同** | ✅ 副本 12 处 identical · 转发头 5 处 | ✅ **通过**：迁移提交 `223dd99`；本仓 0 语义专项核对无雷区 |
