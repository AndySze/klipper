# Go语言迁移进展分析与计划

**日期**: 2026-01-09
**目标**: 用Go语言取代Python实现Klipper Host，提高运行效率

---

## 一、当前进展概览

### 1.1 整体进度

| 阶段 | 状态 | 完成度 | 说明 |
|------|------|--------|------|
| **Phase 0: Golden基准** | ✅ 完成 | 100% | 建立Python参考输出baseline和对齐工具链 |
| **Protocol层** | ✅ 完成 | 100% | dict/VLQ/CRC/msgblock编解码 |
| **Host H1: 配置编译** | ✅ 完成 | 100% | connect-phase命令生成 |
| **Host H2: G-code执行** | ✅ 完成 | 100% | 基础G-code执行（G4, commands, linuxtest） |
| **Host H3: Manual Stepper** | ✅ 完成 | 100% | manual_stepper支持 |
| **Host H4: Cartesian运动** | 🟡 进行中 | 75% | out_of_bounds通过，gcode_arcs有差异 |
| **Host H5: 硬件闭环** | ⏸ 未开始 | 0% | clocksync/实时MCU通信 |
| **Host H6: 完整功能** | ⏸ 未开始 | 0% | heaters/macros/API服务器 |

### 1.2 代码统计

```
Go代码总量: ~8,857行
├── protocol/     ~1,500行  (协议底层)
├── chelper/      ~500行    (CGO绑定)
├── hosth1/       ~800行    (配置编译)
├── hosth2/       ~600行    (基础G-code)
├── hosth3/       ~1,200行  (manual stepper)
└── hosth4/       ~3,500行  (cartesian运动 - 最大模块)
```

### 1.3 测试覆盖情况

| 测试用例 | 状态 | 说明 |
|----------|------|------|
| `commands.test` | ✅ 通过 | H2阶段，基础命令列表 |
| `linuxtest.test` | ✅ 通过 | H2阶段，多MCU基础设施 |
| `manual_stepper.test` | ✅ 通过 | H3阶段 |
| `out_of_bounds.test` | ✅ 通过 | H4阶段，bounds checking |
| `gcode_arcs.test` | ❌ 失败 | H4阶段，G2/G3弧线运动 |
| `extruders.test` | ⏸ 未测试 | H5+ |
| `pressure_advance.test` | ⏸ 未测试 | H5+ |
| `bed_screws.test` | ⏸ 未测试 | 需要probe支持 |
| `bltouch.test` | ⏸ 未测试 | 需要probe支持 |
| `macros.test` | ⏸ 未测试 | 需要模板引擎 |

---

## 二、当前阻塞问题分析

### 2.1 `gcode_arcs.test` 差异详情

**问题1: queue_step分块边界差异**
```
# 期望输出 (Python)
queue_step oid=8 interval=8000 count=499 add=0
queue_step oid=2 interval=39000 count=103 add=0
queue_step oid=5 interval=39000 count=103 add=0
queue_step oid=8 interval=8000 count=500 add=0
queue_step oid=2 interval=39000 count=102 add=0
queue_step oid=5 interval=39000 count=102 add=0
queue_step oid=8 interval=8000 count=501 add=0

# 实际输出 (Go)
queue_step oid=8 interval=8000 count=500 add=0
queue_step oid=2 interval=39000 count=103 add=0
queue_step oid=5 interval=39000 count=103 add=0
queue_step oid=8 interval=8000 count=500 add=0
queue_step oid=2 interval=39000 count=102 add=0
queue_step oid=5 interval=39000 count=102 add=0
queue_step oid=8 interval=8000 count=500 add=0
```

**根因分析**:
- Go的step generation在`interval=8000`边界处的批次划分与Python不一致
- 可能是`flushHandlerDebug`调用时机或`dripMove`推进时机的问题
- 0.25秒批量边界的时间对齐差异

**问题2: EOF motor-off时钟偏移**
```
# 期望: clock=1577030721
# 实际: clock=1577393335
# 差异: 362614 ticks @ 16MHz ≈ 22.66ms
```

**根因分析**:
- EOF时的`printTime`推进与Python不一致
- `request_restart('exit')`处理流程的时序差异

### 2.2 已尝试的修复

1. ✅ 减少`flush_handler_debug`的eager调用
2. ✅ 使用真实的`conv_time`进行clock-est初始化
3. ✅ 分离trsync/endstop到独立command queue
4. ✅ 在`dripMove`后推进`toolhead.printTime`
5. ✅ 添加trace诊断功能

### 2.3 下一步调试方向

**方案A: 时间序列对比**
- 增强trace输出，记录每次`advanceFlushTime()`的详细时间
- 在Python侧添加相同的trace，对比时间序列
- 定位具体的分歧点

**方案B: 放宽strict模式**
- 对于`gcode_arcs.test`暂时使用`relaxed-clock`模式
- 优先扩展功能覆盖，回头优化精度

**方案C: 深度chelper对齐**
- 检查CGO绑定的参数传递是否与Python一致
- 验证`stepcompress`和`steppersync`的调用时机

---

## 三、短期计划（1-2周）

### 目标: 完成H4核心功能，为H5做准备

#### Week 1: 解决gcode_arcs差异

**Day 1-2: 深度诊断**
```bash
# 1. 增强trace工具
- 记录所有flush事件的时间戳
- 记录step generation的每次调用
- 导出为JSON/CSV对比

# 2. Python侧添加trace
- 修改klippy/chelper/添加相同trace点
- 生成reference trace

# 3. 自动对比脚本
- scripts/trace_diff.py
- 高亮首次分歧点
```

**Day 3-4: 尝试修复**
```bash
# 优先级1: 检查dripMove边界
- 确认drip segment的endTime计算
- 验证printTime推进是否与Python一致

# 优先级2: 检查batch边界
- 验证0.25s批量的对齐
- 检查flushHandlerDebug的触发条件

# 优先级3: EOF处理
- 确认motor-off的时机
- 验证restart handlers
```

**Day 5-7: 验证与文档**
```bash
# 如果strict通过:
✅ 更新Agent_Changes.md
✅ 添加gcode_arcs到CI

# 如果仍失败:
⚠️ 记录已知差距
⚠️ 决定是否使用relaxed模式
⚠️ 继续扩展功能覆盖
```

#### Week 2: 扩展H4测试覆盖

**目标测试用例**:
1. `extruders.test` - 基础挤出
2. `gcode_move.test` - 更多G-code特性
3. `homing_moves.test` - 其他homing场景

**需要添加的功能**:
```go
// gcode.go: 添加G-code命令支持
- G0/G1 (已有，但需增强)
- G28 (已有，但需覆盖更多场景)
- M220 (speed factor override)
- M221 (extrude factor override)
- SET_KINEMATIC_POSITION

// extruder.go: 完善extruder逻辑
- pressure_advance (基础支持已有，需测试)
- extruder skew calibration
- extruder rotation distance
```

---

## 四、中期计划（1-2个月）

### 4.1 Host H5: 温度控制与硬件闭环

**目标**: 支持heaters、temperature sensors、实时MCU通信

**阶段1: 温度控制 (2-3周)**
```go
// 新增包: go/pkg/hosttemp/
type Heater struct {
    sensor      *TemperatureSensor
    pwm         *PWMOutput
    target      float64
    current     float64
    pid         *PID
    maxPower    float64
}

// 需要实现的命令
- SET_HEATER_TEMPERATURE
- SET_HEATER_LIMIT
- TEMPERATURE_WAIT
- M104/M109/M140/M190
```

**测试用例**:
- `heaters.test`
- `temperature_fan.test`
- `pid_calibrate.test`

**阶段2: ClockSync与实时通信 (2-3周)**
```go
// clocksync.go: 实现clock同步
type ClockSync struct {
    mcu          *MCU
    lastClock    uint64
    lastTime     float64
    freq         float64
    estFreq      float64
}

// 需要实现
- get_clock() / get_uptime() 命令处理
- clock estimate更新
- clock drift补偿
- serial FIFO full处理
- underrun检测与恢复
```

**验收标准**:
- 能连接真实MCU
- 能执行基本homing
- 能加热并保持温度
- 能完成简单打印（不会underrun）

### 4.2 Host H6: 核心功能补全

**Macros与模板引擎** (2周)
```go
// options.go: 模板引擎
type TemplateEngine struct {
    context map[string]interface{}
}

// 需要支持
- {% %} 语句块
- {{ }} 变量替换
- Jinja2兼容子集
- 自定义G-code命令
```

**API Server** (1-2周)
```go
// api.go: Moonraker兼容API
type APIServer struct {
    printer *Printer
    mux     *http.ServeMux
}

// 需要实现的端点
- /objects/list
- /objects/query
- /gcode/script
- /printer/objects/subscribe
```

**更多Kinematics** (2-3周)
```go
// kinematics/
├── corexy.go      // 核心XY
├── delta.go       // Delta
├── hybrid_corexy.go // 混合CoreXY
└── idex.go        // 双挤出
```

---

## 五、长期计划（3-6个月）

### 5.1 性能优化

**目标**: 相比Python实现，显著提升性能

**优化方向**:
1. **并发优化**
   - G-code解析并行化
   - 多MCU并发通信
   - Motion planning异步化

2. **内存优化**
   - 减少GC压力（对象池、预分配）
   - 使用sync.Pool减少分配
   - 避免频繁的小对象分配

3. **算法优化**
   - 优化lookahead算法
   - 改进step compression效率
   - 加速kinematics计算

**基准测试**:
```bash
# 性能对比脚本
scripts/benchmark_python_vs_go.sh
├── G-code解析速度
├── Motion planning延迟
├── 内存占用
└── 复杂模型打印时间
```

### 5.2 生产就绪

**稳定性** (1个月)
- 完整的错误处理
- 优雅的shutdown流程
- MCU重连机制
- 数据持久化（file-position）

**可观测性** (2周)
- 结构化日志
- Prometheus metrics
- 性能profiling
- 调试web界面

**兼容性** (2周)
- Moonraker完整支持
- Mainsail/Fluidd前端兼容
- 配置文件向后兼容
- 迁移工具

### 5.3 上游贡献

**准备**:
1. 完成核心功能测试
2. 性能有明显提升
3. 代码质量符合上游标准
4. 文档完善

**策略**:
1. 先作为experimental feature
2. 逐步收集反馈
3. 性能对比数据
4. 最终可能作为可选host实现

---

## 六、风险与缓解

### 6.1 技术风险

| 风险 | 影响 | 概率 | 缓解措施 |
|------|------|------|----------|
| Step timing精度无法匹配Python | 高 | 中 | 持续使用golden test严格验证 |
| CGO性能开销抵消Go优势 | 中 | 低 | 关键路径使用纯Go或优化CGO |
| MCU通信协议差异 | 高 | 低 | 严格遵循protocol.md规范 |
| 内存占用增加 | 低 | 中 | 持续profiling和优化 |

### 6.2 项目风险

| 风险 | 影响 | 概率 | 缓解措施 |
|------|------|------|----------|
| 开发周期过长 | 中 | 中 | 分阶段发布，H1-H4可用即可 |
| 维护成本高 | 中 | 低 | 保持代码简洁，充分测试 |
| 社区接受度低 | 高 | 低 | 性能数据说话，保持兼容性 |

---

## 七、下一步立即行动

### 今日任务清单

- [ ] 评估`gcode_arcs.test`差异严重性
- [ ] 决定: 继续debug vs 使用relaxed模式继续推进
- [ ] 如果继续debug: 实现增强trace工具
- [ ] 如果继续推进: 选择下一个测试用例（extruders.test?）

### 本周目标

1. **解决gcode_arcs或决定绕过**
2. **至少通过1个新的测试用例**
3. **更新Agent_Changes.md记录进展**

### 本月里程碑

- [ ] H4核心功能稳定（至少3个测试用例通过）
- [ ] H5温度控制框架搭建完成
- [ ] 发布第一个可演示的Go Host版本

---

## 八、资源与支持

### 开发环境
```bash
# Go环境要求
Go 1.21+
CGO_ENABLED=1  # 必须启用CGO用于chelper

# 测试环境
Python 3.8+  # 用于生成golden baseline
dict files    # MCU data dictionaries

# 调试工具
scripts/go_migration_golden.py  # Golden测试
go test ./...                    # Go单元测试
```

### 关键文档
- `docs/Code_Overview.md` - Klipper架构
- `docs/Protocol.md` - Host-MCU协议
- `docs/Go_Host_Migration_Workplan.md` - 迁移工作计划
- `docs/Agent_Changes.md` - 变更日志

### 代码参考
- `klippy/toolhead.py` - Python toolhead实现
- `klippy/chelper/` - C helper库（通过CGO复用）
- `go/pkg/hosth4/` - 当前最新的Go实现

---

**更新**: 本文档应随着进展每周更新，记录实际完成情况与计划的偏差。
