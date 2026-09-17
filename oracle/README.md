---
class: fact
generated: false
---
# oracle —— 本项目的唯一可信期望值来源

> **类：B 事实** —— 规矩写在 [AGENTS.md](../AGENTS.md) §2；本文件是 oracle 的说明书与验证记录。
> 谁要用期望值（测试、对账、验收），都从这里取；**别处不许再有一个"真相"**。

## 1. 是什么 / 不是什么

| | 内容 |
|---|---|
| **是** | `oracle/refctl.py` —— 按 `docs/spec/` **独立重写**的参考实现（float64 主源 + float32 仅用于推导容差） |
| **是** | `tests/golden/*.csv` —— 由它生成、已提交的黄金向量；CI 不需要 Python 即可回归 |
| **不是** | 不是被测代码的另一种写法：它**不读、不调** `inc/` 下的 C++ 源码，也不把 C++ 输出当输入 |
| **不是** | 不是规范本身：规范在 `docs/spec/`；oracle 是规范的**可执行版本**（两者不一致时以 spec 为准并修 oracle） |

## 2. 三条铁律（对应 AGENTS.md §2）

1. **期望值不得由被测对象算出** —— 测试只读 `tests/golden/*.csv`；
2. **测试输入也不由被测对象生成** —— 输入序列在 `gen_golden.py` 里显式写死（无随机数）；
3. **容差必须推导** —— `tol = 8·dev + 1e-6·scale`，其中 `dev` 是 oracle 自身 float32 偏差实测值，
   与 `tol` 一起写在每个 CSV 头部，可审计。

## 3. 用法

```bash
python3 oracle/validate_oracle.py     # 给 oracle 找尺子：外借实现 + 精确算术，必须全绿
python3 oracle/gen_golden.py          # 重新生成 tests/golden/*.csv（改 oracle 后必须重跑）
cmake -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure
```

改动 oracle 的正确姿势：改 `refctl.py` → 跑 `validate_oracle.py`（全绿）→ 跑 `gen_golden.py`
（CSV 头部的 `oracle-sha256` 会变，diff 里一眼可见）→ 跑 ctest → 确认变化符合预期再提交。

## 4. oracle 自己的尺子（都是"外借"的）

`validate_oracle.py` 用三种独立来源交叉验证 oracle：

| 尺子 | 用途 | 结果（2026-09-17 实测） |
|---|---|---|
| `scipy.signal.lfilter`（BSD-3） | LPF 差分方程的教科书实现；PID **线性部分**（无限幅/无分离/无斜坡时，P + 梯形 I + 微分先行 D 三通道叠加） | LPF `max|Δ| = 0`；PID 三通道 `max|Δ| = 6.3e-14` |
| `fractions.Fraction`（stdlib） | **精确有理数**逐字重算（含钳位、积分分离、斜坡、D 滤波、软死区等全部非线性路径）——不依赖任何浮点实现 | 含钳位/分离用例相对误差 ≤ 1.2e-15；软死区 `0.3·(0.3/0.5)` 精确 |
| 解析式 | LPF 阶跃闭式解 `y[k] = 1-(1-α)^k`；Ramp 速率上界/单调性/冻结语义；SmoothPlanner 首步手算 | 全部 ≤ 2.2e-16 或精确成立 |

**未采纳的候选（记录在案，防考古困惑）**：

- `simple_pid`（MIT，Python 生态最常用的 PID）：本机无 pip、装不上；且它是**矩形积分**，
  与本库的梯形积分不同源，即使装上也只能做受限子集对比。不列为 oracle。
- SimpleFOC `PIDController`（MIT）：语义最近（同为 Tustin + 限幅 + 斜坡），但与本库血缘过近
  （本地实现极可能受其影响）——**同源风险**，只作语义参考（见 `docs/research/`），不作 oracle。
- ArduPilot `AC_PID`：**GPLv3**，代码不可借用；只作语义参考（条件积分/外部 limit 反馈）。
- 精确有理数版虽在 `validate_oracle.py` 内，但它是**验证工具**不是第二 oracle：
  黄金向量只由 `refctl.py` 生成，保持"唯一"。

## 5. 容差怎么推（不许手拍）

```
dev   = max|oracle(f32) − oracle(f64)|        # 同一算法、同输入、同精度的自身舍入上界（实测）
scale = max(1, max|oracle(f64)|)              # 量纲归一
tol   = 8·dev + 1e-6·scale                    # 8 = 安全系数；1e-6·scale = 绝对下限
```

- 8 的来源：覆盖 C++ 侧算子顺序/常量折叠差异（实测 C++ 与 f64 的偏差 ≈ dev 的 0.1~1 倍，
  即安全余量 8~80 倍；见下节实测表）。
- CSV 头部同时记录 `dev`、`scale`、`tol`，测试打印实测 `max_dev` —— 偏差随实现漂移一眼可见。

## 6. 判别力（改坏一行 → 必须变红）

2026-09-17 实测（改 `inc/`/`src/` 一行，重编译，跑 `golden_test`）：

| 变异 | 结果 | 最大实测偏差 vs 该用例 tol |
|---|---|---|
| PID 梯形系数 `0.5f` → `0.6f` | ✅ 变红（9 行超差） | 3.5e-2 vs 3.76e-5 |
| PID 微分符号翻转 | ✅ 变红（4 行超差） | 1.0e-2 vs 1.01e-6 |
| LPF `alpha` 公式改坏 | ✅ 变红（12 行超差） | 2.5e-1 vs 2.0e-6 |
| Ramp 速率上界 ×2 | ✅ 变红（1 行超差） | 5.0e-1 vs 3.0e-6 |

正常实现的实测偏差（16 个用例，2026-09-17）：`max_dev ≤ 9.6e-07`（最大在 `pid_limit_unlimited`），对应 tol 余量 2~80 倍。

## 7. 覆盖范围与局限

- **覆盖**：`PID`（含饱和/积分分离/D 滤波/输出斜坡/dt 守卫/NaN-Inf 守卫/`0 = 不限幅`）、`LPF`（含 Tf=0）、
  `Ramp`（含 max_rate=0）、`SmoothPlanner`、`Deadzone`（软/硬/range=0 三路径）—— 共 16 个黄金用例。
- **NaN/Inf 为什么不在黄金向量里**：CSV 判定用 `|got − expected| ≤ tol`，NaN 参与比较恒为假，无法在 CSV 里表达；
  所以它由 `tests/smoke_test.cpp` 的 `test_pid_nan_recovery` 覆盖，oracle 侧在 `validate_oracle.py` 同步验证。
- **未覆盖**：`set_state` 注入路径（由 `tests/smoke_test.cpp` 的解析断言覆盖）。
- **精度**：oracle 主源是 float64；float32 仅用于推导容差，不是被测实现的镜像。

## 8. 可复现性

- 依赖版本见 `oracle/requirements.txt`（本机：Python 3.12.3 / numpy 1.26.4 / scipy 1.11.4）。
- 每个 CSV 记录生成时的 `oracle-sha256`（前 12 位）+ `params` + `tol`，diff 可审计。
- 黄金向量**提交进仓库**：CI（无 Python 环境假设）只跑 ctest，不重新生成。
