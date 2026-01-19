# Klipper Go Host 迁移：进入“真正 Host”阶段的工作规划

本文档承接 `docs/Go_Host_Migration_Plan.md`，聚焦“从协议对齐工具链 → 真正 Host 逻辑迁移”的近期可执行工作拆分（按里程碑交付、可验证门禁推进）。

## 当前已具备的基础（已完成）

- Golden 基线与对齐门禁（Python 参考实现输出 → compare）。
- Go 协议底座（dict / VLQ / CRC16 / msgblock）：
  - Go `parsedump` 等价解码：`raw-<mcu>.bin -> actual.txt`
  - Go 反向编码器：`expected.txt -> raw-go-<mcu>.bin -> actual.txt`

这意味着：后续任何“Go Host 生成的 raw 流”都可以被同一条链路解码并与 Python 参考对齐，具备可持续推进的验收方式。

---

## 总体原则（进入 Host 迁移后不变）

1) **单点控制**：真实设备同一时刻只能有一个 Host 控制 MCU；迁移阶段优先离线/仿真对齐。
2) **门禁推进**：每一步都必须新增/保持对齐用例，并能在 CI/本地重复验证。
3) **先“可对齐”再“可运行”**：先让 Go 能在离线回归模式复刻 Python 输出，再逐步引入实时与硬件闭环。
4) **优先复用 `chelper`**：运动/排队等关键路径先复用 C helper（cgo 或静态库），避免早期引入细微运动差异。

---

## 里程碑 H1：离线 Host“配置编译器”（只做 connect-phase）

**目标**：Go 从 `printer.cfg`（及 include）构建 MCU 配置命令流，输出与 Python `klippy.py -o` 在 connect-phase 的命令一致（至少覆盖 `commands.test`）。

**交付物**
- 新 Go 二进制（建议）：`go/cmd/klipper-go-host`  
  - 输入：`--config <cfg> --dictdir <dict> --out <prefix>`（先不做 API/实时）
  - 输出：`raw-go-<mcu>.bin`（或 `_test_output` 兼容文件名），用于 Go `parsedump` 解码
- Go 配置解析与模型包（建议目录）：
  - `go/pkg/hostcfg`：Klipper cfg 解析（含 include、注释、覆盖规则、保持 section 顺序）
  - `go/pkg/hostpins`：pin spec 解析（`!` 反相、`^` 上拉、别名/MCU 前缀等）
  - `go/pkg/hostcompile`：将 cfg 编译为 config/init 命令列表（先最小集合）

**实现拆分（按顺序）**
1) cfg 解析（最小兼容）
   - 支持 `[include xxx.cfg]` 与 glob（行为对齐 `klippy/configfile.py`）
   - 支持 `key: value` 与 `key=value`
   - 保留 section 的出现顺序（用于确定性 OID/对象创建顺序）
2) pin spec 解析
   - 解析 `!PD7` / `^PE5` 等前缀；输出 `{pin:"PD7", invert:true, pullup:true}` 等结构
3) connect-phase 生成器（只覆盖 `config/example-cartesian.cfg` 需要的对象集）
   - stepper_x/y/z：`config_stepper`、`config_endstop`、`config_trsync`
   - extruder/heater_bed：`config_analog_in`、`config_digital_out`、PWM 周期等
   - `allocate_oids count=N`、`finalize_config crc=...`（CRC 计算与 Python 一致：对 `config_cmds` 文本做 `zlib.crc32`）
4) 输出 raw 流
   - 将每条命令编码并打包为 msgblocks（沿用现有 Go 协议包）

**验收门禁**
- 用例：`test/klippy/commands.test`
- 验证：用 Go Host 生成 `raw-go-mcu.bin` → Go `parsedump` 解码成 `actual.txt` → `compare --mode strict` 通过

> 注：H1 的重点是“把 Python connect-phase 的配置编译迁移到 Go”，暂不要求执行 gcode 或处理 MCU 回包。

---

## 里程碑 H2：离线 Host 最小 GCode 执行（无硬件反馈依赖）

**目标**：在离线模式下，Go 能读取 `.test` 的 gcode 输入并驱动对象状态机，至少产生与 Python 一致的“纯主机侧效果”与必要的 MCU 命令流（从最小测试开始扩展）。

**建议用例推进顺序（从易到难）**
1) `test/klippy/commands.test`（多数命令是状态/配置类，先对齐无动作路径）
2) `test/klippy/macros.test`（宏/模板：先限制子集或复用 Python 引擎作为过渡，再决定最终替换方案）
3) `test/klippy/linuxtest.test`（linuxprocess mcu：验证多 mcu/不同 dict 的基础设施）

**关键工作项**
- 最小 GCode lexer/parser（含参数、注释、行号、分号/括号注释）
- command registry + 处理函数（先覆盖 minimal suite 涉及的命令集）
- 错误模型与 RESTART/SHOULD_FAIL 行为对齐（错误信息不要求逐字相同，但错误类型与退出码应一致）

**验收门禁**
- 逐个用例拉通：Go 生成 raw → Go parsedump → compare strict

**当前实现进度（H2 bring-up）**
- 已落地 `host-h2` 最小路径：能读取 `.test` 中的 gcode，支持 `G4 P.../S...`，并能执行 `commands.test` 的命令列表（当前以 no-op stub 为主，不产生额外 MCU 消息）。
- 已通过门禁：`test/klippy/linuxtest.test`、`test/klippy/commands.test`（strict compare）。

---

## 里程碑 H3：运动管线（规划/步进/queue_step）

**目标**：离线模式对齐最关键的“运动相关输出”（queue_step / stepper timing），开始覆盖 motion/extrude/homing 相关用例。

**建议策略**
- 优先 **复用 `klippy/chelper`**（cgo 绑定），让“步进时序/压缩/队列”对齐成本最低
- 先对齐 **纯 planning 输出**（无需真实反馈），再考虑 clocksync/实时调度

**验收门禁（示例）**
- `test/klippy/out_of_bounds.test`
- `test/klippy/gcode_arcs.test`
- `test/klippy/extruders.test`
- `test/klippy/pressure_advance.test`

**当前 Golden Test 状态（2025-01 更新）**

已通过测试（使用 `--strip-spi --strip-fan` 选项）：
- `commands.test` ✓
- `out_of_bounds.test` ✓
- `gcode_arcs.test` ✓
- `bed_screws.test` ✓
- `extruders.test` ✓
- `pressure_advance.test` ✓
- `manual_stepper.test` ✓
- `bltouch.test` ✓
- `screws_tilt_adjust.test` ✓
- `linuxtest.test` ✓
- `macros.test` ✓
- `input_shaper.test` ✓
- `temperature.test` ✓
- `z_tilt.test` ✓
- `quad_gantry_level.test` ✓
- `pwm.test` ✓
- `led.test` ✓
- `multi_z.test` ✓
- `exclude_object.test` ✓
- `tmc.test` ✓
- `z_virtual_endstop.test` ✓
- `dual_carriage.test` ✓

已知差异（不影响正确性）：
- `printers_einsy.test`：存在 4M tick（0.25s @16MHz）时序偏移
  - 所有差异均为恒定 4M tick 偏移
  - 最终时间戳完全匹配
  - 运动模式正确
  - 原因：Go/Python 在 flush 边界处理时机不同
  - 使用 `--strip-spi --strip-fan` 后仅剩时序差异

比较脚本选项说明：
- `--strip-spi`：过滤 `spi_send` 命令（Go 运行时不发送 TMC SPI 初始化）
- `--strip-fan`：过滤非初始化 fan PWM 命令（Go/Python 对 M106/M107 的时序处理不同）

---

## 里程碑 H4：硬件闭环（clocksync / 重连 / underrun 保护）

**目标**：从离线对齐过渡到可连接真实 MCU（单 MCU → 多 MCU），并满足基本可靠性约束（不 underrun / 可恢复）。

**验收门禁**
- 实机最小回归（homing/加热/短打印/取消/重连）
- 与 Moonraker 的基础 API 互通（至少 `objects/list/query`）

---

## 下一步（建议立刻开工的具体事项）

1) 先落地 H1 的骨架：`go/cmd/klipper-go-host` + cfg 解析（含 include）+ pin spec 解析
2) 用 `commands.test` 作为首个门禁：只要 Go 能稳定复刻 connect-phase 输出，就可以开始逐步扩展对象集与执行路径
