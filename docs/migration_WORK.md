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
| ① | cyclotron/foc | 五个组件全在用（PID/LPF/Ramp/SmoothPlanner/Deadzone），且有 FOC_MATH_SPEC 锚点 | 1~2h |
| ② | lunokhod/actuator/wheel | 血缘源头、用法简单；① 验过的流程直接套 | ~1h |

## 2. 步骤（每个下游重复一遍）

### 2.0 冻结基线

```bash
cd <下游> && git status --short     # 必须干净；不干净先提交
git rev-parse HEAD                  # 记到 §4 记录表（回滚用）
```

### 2.1 拷贝（两种方式选一，推荐 ②）

| 方式 | 做法 | 校验 |
|---|---|---|
| ① 散拷贝（roadmap D-6 原方案） | 按下游现有布局散放文件，每个文件头加来源戳 `// from ctlkit v0.1.0` | `scripts/downstream_diff.py`（按文件名匹配，忽略戳行） |
| ② 整目录 vendor（**推荐**） | 下游建 `third_party/ctlkit/`，把上游 `inc/` + `src/` 整个放进去，另写一个 `VERSION` 文件记 `v0.1.0 (<上游 sha>)` | 同上（目录级逐字比对，更省心） |

推荐 ② 的理由：库的目录布局本身是冻结面；整目录即来源，不用逐文件打戳；回滚 = 删一个目录。

### 2.2 接构建

- include 指向 `.../ctlkit/inc`；编译 `.../ctlkit/src/*.cpp`。
- 包含路径从 `foc/algo/pid.hpp` 改成 `ctl/pid.hpp`（其余组件同理）。

### 2.3 改调用点（唯一要动业务代码的地方）

- 命名空间：`foc::algo::PID` → `ctl::PID`（LPF/Ramp/SmoothPlanner/Deadzone 同理）；可用 `using ctl::PID;` 缩小 diff 面。
- 已知调用点（cyclotron）：`current_loop.cpp` 的 iq/id 电流环 —— **签名向后兼容，`calc(cmd, measure, dt)` 不用改**；`foc.hpp` 的 `algo::Deadzone deadzone_`。
- ⚠ **0 语义变更点**：`limit = 0` 的配置从"钳死到 0"变成"不限幅"——逐个核对下游配置（已知 FOC 电流环是非零限幅，不受影响）。`max_rate_out = 0` 含义不变（仍是"关闭斜坡"）。

### 2.4 行为校验（验收，三条都要过）

```bash
# ① 库自身：黄金回归 19/19
cd <ctlkit> && ctest --test-dir build --output-on-failure
# ② 下游：构建 + 下游自己的测试锚点
cd <下游> && cmake --build build && ctest --test-dir build
# ③ 副本一致性：逐字比对（自动忽略来源戳行）
python3 <ctlkit>/scripts/downstream_diff.py <下游>
```

### 2.5 记录与回滚

- 记录：上游 sha + 下游基线 sha + 版本（v0.1.0）+ 三条校验结论 → §4。
- 回滚：散拷贝方式 `git checkout <基线 sha> -- .`；vendor 方式直接删 `third_party/ctlkit/`。

## 3. 风险与对策

| 风险 | 对策 |
|---|---|
| 双活副本再次分叉（历史上发生过） | 迁移后下游**只改调用点**；库文件一律从上游拷，diff 脚本守门 |
| 0 语义变更漏检 | §2.3 核对清单 + 下游实测 |
| 命名空间切换漏改 | 编译器会抓；`using` 可减少改动面 |
| 下游测试锚点不足 | 迁移前后各跑一遍，数字逐条对比（FOC_MATH_SPEC 的锚点公式） |

## 4. 演练记录（边做边填）

| 日期 | 下游 | 上游版本/sha | 下游基线 sha | 方式 | ① 库 19/19 | ② 下游测试 | ③ diff | 结论 |
|---|---|---|---|---|---|---|---|---|
| | | | | | | | | |
