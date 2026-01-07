# Klipper Host（Python -> Go）迁移计划（不改变功能逻辑）

本文档描述将 Klipper 的 Host 端（当前主要为 `klippy/` Python + `klippy/chelper/` C）逐步迁移为 Go 实现的可执行计划。目标是在 **功能逻辑与外部行为不改变** 的前提下，提升性能与可维护性，并确保与现有 MCU 固件（`src/`）以及生态（Moonraker/前端等）兼容。

> 关键提醒：Klipper 的性能关键路径中已有大量 C helper（step 生成/压缩/串口队列等）。因此“改成 Go 就一定更快”并不成立；必须用数据验证收益，并以“行为等价”作为第一优先级。

---

## 1. 目标、边界与成功标准

### 1.1 目标（Goals）

- **Host 全量 Go 化**：最终不依赖 `klippy/` 的 Python 运行时（最终产物为 Go 二进制），MCU 端代码保持不变。
- **行为等价**：相同 `printer.cfg`、相同输入（GCode/API 调用/硬件回包）应产生一致的外部行为（运动、温控、状态、错误处理）。
- **可测与可回滚**：整个迁移过程始终保持可对比（Go vs Python）、可回退（遇到重大不一致可停在上一阶段）。

### 1.2 非目标（Non-goals）

- 不在迁移过程中“顺便重构/改功能/改协议/改配置语义”。
- 不在第一阶段强行优化算法；先对齐行为，再做性能迭代。

### 1.3 成功标准（Definition of Done）

必须同时满足：

1) **协议与时序正确**：Go Host 与 MCU 的握手、字典、命令发送、回包处理、clocksync 行为与 Python Host 一致，且能稳定运行。
2) **Golden 测试对齐**：在离线 batch 模式下，对一组固定配置+gcode，Go 生成的 MCU 命令序列与 Python 生成的序列在定义的对齐规则下等价（详见 §4）。
3) **生态兼容**：关键 API（UDS webhooks）、对象状态与订阅机制对 Moonraker/客户端保持兼容（详见 §3.6）。
4) **硬件验证**：至少 1 台测试机完成一套回归用例（homing、加热、打印、取消、异常恢复等）。

---

## 2. 现状架构速览（迁移对象地图）

Host 端核心职责（高层）：

- **配置与对象图**：读取 cfg、实例化各对象（printer objects）、模块事件（connect/ready/…）
- **GCode 层**：解析、分发、宏/模板执行、错误处理
- **运动规划**：look-ahead、速度规划、trapezoid 分解、运动学变换、step time 生成与压缩
- **MCU 通信**：协议字典、命令编码/发送、回包解析、时钟同步、多 MCU 协调
- **API/Webhooks**：UDS JSON 协议、对象状态查询、订阅、远程方法

关键目录与边界：

- `klippy/`：Python Host（目标：最终替换）
- `klippy/chelper/`：性能关键 C helper（建议：先复用，后评估是否替换）
- `src/`：MCU firmware（本计划中保持不变）
- `docs/Protocol.md` / `docs/MCU_Commands.md`：Host<->MCU 关键协议规范
- `docs/Code_Overview.md`：运动管线与关键调用链说明

---

## 3. 迁移策略选择（推荐路线）

你提出的“按模块把 Python 改 Go 并共存联调”需要非常谨慎：如果 Python 与 Go 同时尝试控制同一 MCU/同一打印机状态，会引入不可控竞争（时钟、队列、状态机、对象生命周期），问题将极难定位。

因此推荐采用 **“双实现并行 + 单点控制”**：

- **生产控制始终只有一个 Host（Python 或 Go）连接并控制 MCU**
- 迁移过程中 Go 先以 **离线/仿真/测试驱动** 方式对齐输出与行为，再切换到真实设备

### 3.1 推荐总体路线（四层推进）

1) **测试与对齐基础设施（先行）**：建立可重复、可量化的对齐方法（batch 输出、API 行为、仿真）
2) **协议与时钟层（底座）**：先让 Go 能正确说 Klipper 协议并维持 clocksync
3) **运动/控制闭环（核心）**：先复用 `chelper`，让 Go 可以生成并发送正确的 `queue_step`
4) **生态与扩展（广度）**：逐步补齐 extras、宏、API，最终完全替换 Python

### 3.2 C helper 的处理建议

- **第一阶段建议保留并复用 `klippy/chelper`**（通过 cgo 或编译为静态库）
  原因：这是最难且最容易引入细微不一致的部分（步进时间求解、压缩、串口队列）。先复用能最大化“行为等价”的成功概率。
- 后续若确有性能/维护需求，再评估将部分 C helper 改写为 Go（这属于第二阶段优化，不属于“先迁移再优化”）。

### 3.3 “完全不改变逻辑”如何定义（对齐规则）

建议把“等价”拆成三类，并在每类给出可执行的验收：

- **硬等价（必须一致）**：协议握手流程、字典解析结果、错误条件与恢复（如超时、重连、shutdown）、配置语义、关键状态字段含义
- **输出等价（允许可控差异）**：离线生成的 MCU 命令文本可能有序列号/时钟对齐差异，但在定义的归一化规则后应等价（见 §4.3）
- **时序等价（以约束判定）**：实时系统允许微小调度差异，但必须满足约束（队列提前量、无 underrun、无 watchdog、温控稳定）

### 3.4 宏/模板（Jinja2）风险提示与策略

宏系统与模板语义是高风险点（极容易“不改变逻辑但行为不同”）。

策略建议：

- Phase 1-3 暂时只保证核心打印路径不依赖复杂宏（用最小宏集/或固定 cfg）
- 进入宏迁移时，先用 **“宏一致性测试集”** 覆盖典型宏（条件分支、变量、循环、过滤器、字符串/数值行为）
- 在最终目标“无 Python”约束下，需要：
  - 实现足够兼容的 Jinja2 子集（优先覆盖 Klipper 常用子集），或
  - 重写宏引擎并提供兼容层（成本更高）

### 3.5 extras 模块体系迁移策略

extras 数量多且演进快，建议按“最小可用集合”推进：

- 核心：运动学/步进/温控/端点/homing/基本传感器
- 常用：bed_mesh、probe、input_shaper、fan、display、filament sensor
- 高级/生态：webhooks 扩展、CAN bus 相关、各类外设驱动

### 3.6 API/Webhooks 兼容策略（Moonraker）

Go Host 必须实现 `docs/API_Server.md` 中的 UDS JSON 协议（对象列表、对象查询、订阅、emergency_stop 等），并保持字段语义一致。

建议：

- 先实现最小兼容面（`info`、`objects/list`、`objects/query`、订阅）
- 再补齐远程方法注册、通知、错误码与异常路径

---

## 4. 验证与测试体系（迁移能否推进的核心）

### 4.1 Golden：离线 MCU 命令序列对齐（最重要）

利用 Klipper 现有的 batch 模式生成“参考输出”（Python Host），再让 Go Host 生成同样的输出进行对比。

参考流程（见 `docs/Debugging.md`）：

- Python（参考实现）：
  - `klippy.py -i test.gcode -o test.serial -d out/klipper.dict`
  - `parsedump.py out/klipper.dict test.serial > test.txt`
- Go（待实现）：
  - 产生等价的 `test.serial` 或直接产生等价的可读 `test.txt`

### 4.2 用例集设计（建议从小到大）

建议建立目录（后续实现时落地）：

- `test/go_migration/cases/<case_name>/`
  - `printer.cfg`
  - `input.gcode`
  - `expected.txt`（Python 生成的归一化结果）
  - `notes.md`（该用例覆盖哪些功能）

用例优先级：

1) 纯运动：直线、加减速、拐角、速度限制、不同 kinematics
2) 挤出同步：E 轴同步、retract、pressure advance（若启用）
3) homing：端点触发、错误路径（未触发、超时）
4) 温控：PID/阈值报警（离线可模拟部分）
5) 常用 extras：bed_mesh、probe（先做最小行为）

### 4.3 输出归一化（避免“无意义差异”）

定义归一化规则，避免因无关字段导致大量 diff：

- 允许忽略：日志行、非确定性的统计字段、序列号等
- 对 `queue_step`：允许在保持等价的情况下对时钟基准进行线性平移（若能证明不影响 MCU 执行），但**步进间隔与数量必须一致**
- 对多 MCU：需要额外规则（主/从时钟映射一致性）

### 4.4 实机验证（硬件闭环）

硬件回归建议至少包含：

- 冷启动、热启动、MCU reset、重连
- homing + 简单移动 + 限位保护
- 加热到目标温度并稳定
- 打印一段短 GCode（5-10 分钟），包含暂停/恢复/取消
- 异常注入：断开串口、温度探头故障、看门狗触发（仅在安全环境）

---

## 5. 分阶段详细计划（里程碑 + 交付物）

### Phase 0：项目准备与基线（1-2 周）

**目标**：建立“能对齐/能衡量/能回退”的基础设施。

任务：

- 建立 Go 项目骨架（建议新增 `go/` 或独立 repo；先不改现有 Python）
- 明确性能指标（CPU、延迟、queue 预留时间、打印质量指标）与采集方式
- 建立 Golden 生成脚本（Python 侧）与对比工具（diff + 归一化）
- 选定 10-20 个最小用例集（见 §4.2）

交付物：

- `docs/Go_Host_Migration_Plan.md`（本文档，持续更新）
- Golden 用例目录与脚本（后续落地）
- 第一版对齐规则文档（后续落地）

验收：

- 能在本地稳定生成 Python 侧 `expected.txt`（可重复）

决策点：

- 若基线 profile 表明瓶颈不在 Python（而在 C helper/IO/MCU），应重新评估“全量 Go 重写”的收益与必要性。

### Phase 1：协议/字典/时钟底座（2-4 周）

**目标**：Go Host 能正确与 MCU 建立连接并完成协议层闭环（不要求完整功能）。

任务：

- 实现/复刻数据字典解析（对应 `msgproto` 概念）
- 实现消息块编码/解码（CRC、VLQ、sequence、重传/ack）
- 实现 MCU 配置流程（`get_config`、`allocate_oids`、`config_*`、`finalize_config` 的最小子集）
- clocksync：实现 print time / MCU clock / system time 转换与漂移处理（至少单 MCU）

交付物：

- Go 端协议库（可独立测试）
- 可连接到 MCU simulator 或真实 MCU 并完成握手（只做只读/最小写入）

验收：

- 能跑通握手并稳定接收 `status`/基础响应
- 连接稳定性与超时/重连行为与 Python 参考一致（或有明确差异说明）

### Phase 2：配置系统 + 对象模型 + 事件（2-4 周）

**目标**：Go Host 能读取 `printer.cfg` 并建立对象图，具备模块化扩展机制。

任务：

- 复刻 config 语义（类型转换、默认值、错误信息、include 行为）
- 实现“printer object registry”（类似 `Printer.add_object()/lookup_object()`）
- 实现事件总线（connect/ready/shutdown）
- 先实现最小模块集合：`mcu`、`gcode`、`toolhead` 的骨架（不必完整）

验收：

- 能加载典型配置并得到与 Python 等价的对象拓扑（至少关键对象存在且配置参数一致）

### Phase 3：运动规划与 step 生成（优先复用 chelper）（4-8 周）

**目标**：Go Host 在离线 batch 模式生成与 Python 等价的 MCU 命令流，并能在实机上安全执行基础运动。

任务：

- GCode 基础解析与分发（先覆盖运动相关：G0/G1/G92/M204/M220/M221 等最小集）
- Toolhead：look-ahead、junction、trapezoid 队列管理（可先实现必要子集）
- 运动学：先覆盖 1-2 种（Cartesian/CoreXY），逐步扩展
- **复用 `klippy/chelper`**：通过 cgo 调用 trapq/itersolve/stepcompress/serialqueue（或将其编译为库）
- 生成 `queue_step` 并按协议发送/离线输出

验收：

- 至少 5 个 Golden 用例输出对齐（归一化后无差异）
- 实机：无 underrun，无 watchdog，基础运动与速度规划符合预期

### Phase 4：温控/端点/常用 extras（4-10 周，按优先级滚动）

**目标**：具备可用的打印闭环（运动 + 温控 + homing + 常用传感器）。

任务（建议顺序）：

1) heaters（热床/热端）、temperature sensors、保护逻辑
2) endstops/homing、probe（最小）
3) fans、idle timeout、安全停机
4) bed_mesh / resonance / input_shaper（按需求）

验收：

- 完成“短打印回归用例”并与 Python 结果一致（允许性能差异但行为一致）

### Phase 5：宏/模板系统（高风险）（4-12 周）

**目标**：实现与现有 gcode_macro/Jinja2 行为兼容的宏系统。

任务：

- 明确兼容范围：变量、条件、循环、过滤器、表达式、数值/字符串行为
- 建立宏用例集（包含社区常见宏模式）
- 实现宏引擎与与 printer 状态绑定（`{printer.xxx}` 等）

验收：

- 宏用例集通过；真实配置（你现有的 `printer.cfg`）宏行为一致

### Phase 6：API/Webhooks 全量兼容（2-6 周）

**目标**：Moonraker/客户端不改或少改即可工作。

任务：

- 按 `API_Server.md` 复刻 UDS JSON 协议、对象订阅、远程方法
- 对齐字段语义、错误码、通知顺序（必要时归一化）

验收：

- Moonraker 连接稳定，关键页面/控制功能可用

### Phase 7：切换与收尾（2-4 周）

**目标**：Go Host 成为默认控制面，Python 仅作为参考或移除。

任务：

- 完整实机回归与异常注入测试
- 文档化部署/升级/回滚流程
- 最终移除 Python 依赖（若项目要求）

验收：

- 在目标机型上持续运行稳定（建议至少 1-2 周实际使用）

---

## 6. 风险清单与缓解措施

- 宏/Jinja2 兼容：优先建立宏用例集；明确兼容范围；逐步扩展
- extras 面过大：按需求优先级推进；先覆盖你自己的配置与使用场景
- 时序/并发差异：严格遵循 clocksync/队列提前量约束；先复用 `chelper`
- “等价性”争议：提前定义对齐规则（硬等价/输出等价/时序等价），并写入 CI
- 项目收益不确定：Phase 0 做 profile；若收益不足则及时止损或调整目标

---

## 7. 建议的仓库组织（实施时落地）

（实施阶段再决定，以下为建议）

- `go/`：Go Host 源码
  - `go/cmd/klipper-go/`：主程序
  - `go/pkg/protocol/`：消息编码/字典/CRC/VLQ
  - `go/pkg/mcu/`：连接、配置、命令队列、clocksync
  - `go/pkg/gcode/`：GCode parser/dispatcher
  - `go/pkg/toolhead/`：look-ahead + motion planning
  - `go/pkg/extras/`：模块体系
  - `go/pkg/api/`：UDS webhooks
- `test/go_migration/`：golden 用例与对比工具

---

## 8. 立即可执行的下一步（建议你先选一个）

1) 先做 Phase 0：我可以帮你把 Golden 用例框架与归一化规则写成脚本/文档，并选定第一批用例。
2) 若你明确想“先能跑起来”：从 Phase 1+3 组合（协议底座 + 复用 chelper 的 batch 输出）开始，先把“离线命令流对齐”打通。
3) 若目标主要是性能：先做 profile 与热点分析，再决定是否需要“全量 Go 重写”还是仅替换某些 Python 调度点/优化 C helper。
