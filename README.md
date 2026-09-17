---
class: fact
generated: false
---
# ctlkit

> 个人上游基础算法库：**嵌入式实时控制原语**（控制 + 滤波）。
> 诚实定位：一组小而确定的离散时间控制组件（目前 4 个，见下）。**不是**控制框架，也不提供调度、参数管理或通信。

下游项目（`cyclotron/foc`、`lunokhod/actuator/wheel`）的 PID / LPF / Ramp 等组件以本库为**唯一事实源**，
从本库同步代码；此前分散在多个仓库的改动、注释、决策在此统一沉淀。

---

## 为什么叫 ctlkit

- 这类组件的领域术语是「**控制算法 / 控制器原语**」——Arm CMSIS-DSP 把 PID 归在 *Controller Functions*
  分类下，MathWorks 称 *Control System Toolbox*；单说 "algorithm" 属于描述层级不对。
- **不用 `algorithm` 做名字**：范围太泛（排序/加密/寻路都叫 algorithm），且与 C++ 标准头 `<algorithm>`
  撞名——`algorithm/pid.hpp`、`namespace algorithm` 都会造成阅读与检索歧义。
- `ctl` 是 control 的通用缩写（Unix/嵌入式语境成熟），`kit` 表明是**组件集合**而非框架；
  代码命名空间 `ctl`，包含路径 `<ctl/*.hpp>`。
- 排除项：`libctl`（已有同名 Scheme 科学计算库）、`ControlKit`（已有 JS GUI 库）。

## 组件

| 组件 | 头文件 | 说明 | 行为契约 |
|---|---|---|---|
| `PID` | `<ctl/pid.hpp>`（类型：`<ctl/pid_types.hpp>`） | 微分先行 + 梯形积分 + 积分分离 + D 项 LPF + 输出斜坡 + dt 守卫；观测出口 + 每拍端口 | [docs/spec/pid.md](docs/spec/pid.md) |
| `LPF` | `<ctl/lpf.hpp>` | 一阶低通 `α = dt/(Tf+dt)`；Tf=0 直通 | [docs/spec/lpf.md](docs/spec/lpf.md) |
| `Ramp` | `<ctl/ramp.hpp>` | 斜率限幅（每帧 clamp 到 `prev ± max_rate·dt`） | [docs/spec/ramp.md](docs/spec/ramp.md) |
| `SmoothPlanner` | `<ctl/smooth_planner.hpp>` | 二阶轨迹规划 = Ramp + 两级 LPF | [docs/spec/smooth_planner.md](docs/spec/smooth_planner.md) |
| `Deadzone` | `<ctl/deadzone.hpp>` | 抛物线软死区 / 硬死区；`range<=0` 直通 | [docs/spec/deadzone.md](docs/spec/deadzone.md) |

**待纳入候选**（尚未收编，见 [docs/roadmap.md](docs/roadmap.md)）：
Notch / 二阶滤波、观测器与状态估计（M4 预研）。

## 目录结构

```
ctlkit/
├── inc/ctl/              # 公开头文件（API 面）
├── src/                  # 实现
├── examples/             # 最小示例（也可当文档看）
├── tests/                # 主机端单测 + golden/（oracle 生成的黄金向量）
├── oracle/               # 唯一可信 oracle（期望值来源，见 oracle/README.md）
├── scripts/              # 文档门禁与本 CI 脚本
├── CMakeLists.txt        # 主机端构建（静态库 + 测试 + 示例）
└── docs/
    ├── spec/             # 现状契约（唯一事实源，不许过期）
    ├── design/           # 设计稿（前瞻，允许过期）
    ├── research/         # 调研与归档（为什么这么设计；含第三方只读快照）
    └── roadmap.md        # 路线图 M0~M4 + 工时估算 + 决策点
```

## 测试与 oracle

**唯一可信 oracle** = [`oracle/refctl.py`](oracle/README.md)：按 `docs/spec/` 独立重写的参考实现
（不读不调 C++ 源码），已用**外借尺子**交叉验证：`scipy.signal.lfilter`（线性部分）、
`fractions.Fraction` 精确有理数（含钳位等非线性路径）、解析闭式解。规则见 [AGENTS.md](AGENTS.md) §2。

```
python3 oracle/validate_oracle.py    # 先验 oracle（全绿才可信）
python3 oracle/gen_golden.py         # 重新生成 tests/golden/*.csv（改 oracle 后必须重跑）
```

- `tests/` 的期望值**只**来自 `tests/golden/*.csv`（已提交，CI 无需 Python）；
  容差 `tol = 8·dev + 1e-6·scale` 逐用例记录在 CSV 头部，可审计。
- 判别力已实测：改坏实现一行 → 黄金回归变红（记录见 [oracle/README.md](oracle/README.md) §6）。

```bash
cmake -S . -B build && cmake --build build
ctest --test-dir build --output-on-failure
./build/examples/pid_basic
```

## 下游消费方式

- 现阶段： **拷贝** `inc` 与 `src/*.cpp` 进下游工程（嵌入式项目惯例）；
  拷贝文件头保留来源戳与版本，便于溯源与 diff。
- 下游：`cyclotron/foc`、`lunokhod/actuator/wheel`。迁移与双副本同步策略见
  [docs/roadmap.md](docs/roadmap.md) §6 D-6（本库为上游，下游只读消费）。

## 约束与兼容

- C++11；纯 `float`；无 STL / 无堆 / 无异常；可作为 ISR 内调用（每组件 O(1)、无阻塞）。
- `v0.0.x` 阶段算法行为与 `cyclotron/foc` 快照**逐字一致**（只做了命名空间与目录的库化）；
  行为兼容承诺自 M0（黄金向量回归建立）起生效。

## 文档体系（三层，各司其职）

| 目录 | 回答的问题 | 是否允许过期 |
|---|---|---|
| `docs/spec/` | 现在**是什么行为**（契约） | ❌ 不许过期，改行为必须同步改 |
| `docs/design/` | 打算**怎么改**（设计稿） | ✅ 允许，实现后沉淀进 spec |
| `docs/research/` | **为什么**这么设计（调研/对比/归档） | ✅ 允许，作为历史记录 |
| `docs/roadmap.md` | 接下来做什么、多久 | ✅ 滚动更新 |

文档的类与位置、命名、机器门禁的规矩见 [docs/README.md](docs/README.md) 与 [AGENTS.md](AGENTS.md)；
改完任何文档跑 `python3 scripts/check_docs.py`。

## 版本与许可

- 版本见 [CHANGELOG.md](CHANGELOG.md)；当前 **`v0.1.0`（API 冻结版）**。
- 许可：**MIT**（见 [LICENSE](LICENSE)）。`docs/research/reference/` 下第三方快照
  **不在本许可范围内**（`Liu-Curiousity/pid` 无 LICENSE），仅供内部研究对比，
  不参与构建、不引入商用代码。

## 兼容政策（v0.1.0 起）

```
patch（0.1.x）：只修 bug，行为修复也走 patch；不新增/不改 API
minor（0.x）  ：只增不改 —— 加字段（默认值 = 旧行为）、加组件、加重载、加端口字段
major（1.0 起）：才允许破坏 —— 改字段名/语义、改签名、删接口
```

- 已冻结面：`calc` 签名、`PIDConfig` / `PIDPorts` / `PIDState` / `PIDStatus` 的既有字段、
  五个组件（PID · LPF · Ramp · SmoothPlanner · Deadzone）的公开接口、`inc/ctl/` 的文件布局。
- 行为兼容由 `tests/golden/`（19 例黄金向量）看守：任何“默认关闭 = 旧行为”的改动必须证明黄金全绿。
