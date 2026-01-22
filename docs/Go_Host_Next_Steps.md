# Go Host 迁移：下一步开发计划

## 当前状态（2026-01-22）

### 测试覆盖率
- **全部 61 个 golden tests 通过** ✅
- 包括所有 13 个架构测试 (arch_*)
- 包括所有 5 个打印机集成测试 (printers_*)
- 包括所有 3 个 multi-mcu 测试

### 已完成的里程碑
- **H1**: 配置编译器 ✅
- **H2**: 最小 GCode 执行 ✅
- **H3**: 运动管线（步进/排队/homing）✅
- **H4 基础设施**: 已就位 ✅
  - clocksync, reactor, mcu_connection, mcu_manager
  - realtime_integration 层
  - Multi-MCU 支持

---

## 下一步开发计划

### 阶段 1：实机验证（优先级最高）

#### 1.1 Linux MCU 模拟器测试 ✅ (2026-01-22 完成)
- [x] Docker 镜像构建 (`scripts/linux-mcu/build.sh`)
- [x] TCP 端口模式连接 (localhost:5555)
- [x] 验证实时时钟同步 (offset/slope 稳定)
- [x] 验证命令收发完整性 (IDENTIFY, get_clock)
- [x] MCU 识别成功 (50MHz Linux MCU)
- [x] 状态查询 API 验证 (GetStatus)

**测试工具**: `hardware-test`
```bash
# 构建 Docker 镜像
./scripts/linux-mcu/build.sh

# 启动模拟器
docker run -d --rm -p 5555:5555 -e MCU_PORT=5555 --name klipper-linux-mcu klipper-linux-mcu

# 运行测试
./hardware-test -device localhost:5555 -tcp -test connect
./hardware-test -device localhost:5555 -tcp -test query
./hardware-test -device localhost:5555 -tcp -test clock -trace

# 停止模拟器
docker stop klipper-linux-mcu
```

#### 1.2 真实 MCU 连接测试
- [ ] 连接真实 MCU（推荐 STM32F103 或 RP2040）
- [ ] 验证串口通信
- [ ] 验证固件识别和字典加载
- [ ] 验证基本命令执行

#### 1.3 基本打印功能验证
- [ ] Homing 测试（单轴 → 全轴）
- [ ] 加热器 PID 控制测试
- [ ] 短距离移动测试
- [ ] 简单打印测试（首层方块）

---

### 阶段 2：Moonraker API 集成 ✅ (2026-01-20 完成)

#### 2.1 核心 API 端点
- [x] `server/info` - 服务器信息
- [x] `printer/objects/list` - 对象列表
- [x] `printer/objects/query` - 对象查询
- [x] `printer/objects/subscribe` - 对象订阅
- [x] `printer/gcode/script` - GCode 执行

#### 2.2 状态订阅
- [x] WebSocket 实时推送 (4Hz)
- [x] 温度状态 (extruder, heater_bed)
- [x] 运动状态 (toolhead, motion_report)
- [x] 打印进度 (print_stats, virtual_sdcard)

#### 2.3 文件管理
- [x] 上传 GCode 文件
- [x] 列出文件
- [x] 删除文件
- [x] 移动/复制文件
- [x] GCode 元数据提取

#### 2.4 打印历史
- [x] 作业跟踪 (start, update, finish)
- [x] 历史记录查询
- [x] 统计信息 (total_jobs, print_time, filament_used)

---

### 阶段 3：功能完善

#### 3.1 缺失的 extras 模块
- [x] `virtual_sdcard` - 完整实现 ✅ (2026-01-20)
  - 工作定时器/goroutine 模式
  - 暂停/恢复/取消支持
  - 错误 G-code 模板
  - analyze_shutdown 日志
  - 并发安全锁
- [x] `input_shaper` - 完整实现 ✅ (2026-01-20)
  - chelper C 函数集成 (input_shaper_alloc, input_shaper_set_sk, input_shaper_set_shaper_params)
  - 所有 shaper 算法支持 (ZV, MZV, ZVD, EI, 2HumpEI, 3HumpEI)
  - Stepper kinematics 包装器
  - 动态参数更新 (SET_INPUT_SHAPER)
  - 双挤出机 (IDEX) 支持检测
  - 步进生成扫描窗口管理
- [x] `gcode_move` - GCode 移动核心 ✅ (2026-01-21)
  - 完整坐标系统管理 (absolute/relative, extrude mode)
  - G0/G1 移动, G2/G3 圆弧, G17/G18/G19 平面选择
  - G90/G91 坐标模式, G92 位置设置
  - M82/M83 挤出模式, M114 位置报告
  - M220 速度倍率, M221 挤出倍率
  - SET_GCODE_OFFSET 虚拟偏移
  - SAVE/RESTORE_GCODE_STATE 状态保存恢复
  - homing_position 归零偏移跟踪
  - 17 个单元测试
- [x] `tmc2209` - TMC2209 驱动 ✅ (2026-01-21)
  - TMC2208/TMC2209 驱动实现
  - 电流计算 (vsense, irun, ihold)
  - CHOPCONF, GCONF, PWMCONF 寄存器配置
  - StallGuard (SGTHRS, SG_RESULT) 和 CoolStep (TCOOLTHRS, COOLCONF)
  - SET_TMC_CURRENT, DUMP_TMC, INIT_TMC, SET_TMC_FIELD 命令
  - TMCDriverManager 驱动管理器
  - 10 个单元测试

#### 3.2 高级功能
- [x] Bed mesh 补偿 ✅ (2026-01-21)
  - Profile 管理 (save/load/remove)
  - BED_MESH_OUTPUT, BED_MESH_MAP, BED_MESH_CLEAR, BED_MESH_OFFSET, BED_MESH_PROFILE 命令
  - GetStatus() API 查询 (profile_name, mesh_min/max, probed_matrix, mesh_matrix)
  - zMesh 辅助方法
  - 12 个单元测试
- [x] Pressure advance 微调 ✅ (2026-01-21)
  - 配置加载 (pressure_advance, pressure_advance_smooth_time 从 [extruder] 读取)
  - 连接阶段初始化 (SetPressureAdvance 应用配置值)
  - SET_PRESSURE_ADVANCE 动态调整命令
  - 动态挤出机注册表 (extruderSteppers map)
  - chelper 集成 (extruder_set_pressure_advance)
  - 平滑窗口管理 (smooth_time 变更需 flush)
- [x] Input shaper 自动调谐 ✅ (2026-01-21)
  - FFT/PSD 计算 - Welch 算法实现 (psd_calc.go)
    - Kaiser 窗口函数 (β=6)
    - Bessel I0 函数 (多项式近似)
    - 50% 重叠窗口分割
    - Gonum FFT 集成 (gonum.org/v1/gonum/dsp/fourier)
  - 完整评分逻辑 (shaper_calibrate.go)
    - 振动残留估计 (EstimateResidualVibration)
    - 平滑度计算 (calcSmoothing - 二阶矩法)
    - 最大加速度计算 (MaxAccelFromSmoothing)
    - 评分公式: score = smoothing * (vibrs^1.5 + vibrs*0.2 + 0.01)
    - 多阻尼比鲁棒优化 (0.075, 0.1, 0.15)
  - 共振测试执行 (resonance_tester.go)
    - VibrationPulseTestGenerator 脉冲测试
    - SweepingVibrationsTestGenerator 扫频测试
    - ResonanceTestExecutor 运动执行
    - AccelDataCollector 数据采集
  - G-code 命令
    - TEST_RESONANCES AXIS=<x|y> [FREQ_START=] [FREQ_END=]
    - SHAPER_CALIBRATE AXIS=<x|y> [NAME_PREFIX=] [MAX_SMOOTHING=]
    - MEASURE_AXES_NOISE [MEAS_TIME=]
  - CSV 数据输出 (#time,accel_x,accel_y,accel_z)
  - 单元测试: 30+ 测试用例 (psd_calc_test.go, shaper_calibrate_test.go)
- [x] 多挤出机支持 ✅ (2026-01-21)
  - ACTIVATE_EXTRUDER 工具切换
    - 位置保存/恢复 (lastPosition 同步)
    - toolhead.extraAxes 动态更新
    - 步进生成刷新 (flushStepGeneration)
  - SET_PRESSURE_ADVANCE EXTRUDER 参数路由
    - 默认使用当前活动挤出机
    - 支持 extruder/extruder1/my_extra_stepper
  - SYNC_EXTRUDER_MOTION 增强
    - 支持 extruder1 作为步进器和运动队列
    - 动态注册表查找 + 回退
  - M104/M109 T 参数支持
    - 温度命令路由到正确加热器 (extruder, extruder1, ...)
  - 辅助方法
    - getExtruderByName(name) - 按名称获取挤出机轴
    - getActiveExtruder() - 获取当前活动挤出机

#### 3.3 错误处理和恢复 ✅ (2026-01-21)
- [x] MCU 断连重连
  - ShutdownCoordinator 协调器 (shutdown.go) - 444 行
  - HeartbeatMonitor 心跳监控 (heartbeat.go) - 414 行
  - ReconnectionManager 自动重连 (reconnect.go) - 395 行
  - RestartHelper 重启助手 (restart.go) - 421 行
  - MCUDiagnostics 错误诊断 (mcu_diagnostics.go) - 456 行
  - 完整单元测试覆盖 (shutdown_test.go) - 506 行
- [x] 紧急停止处理
  - M112 命令支持 (HandleM112)
  - emergency_stop MCU 命令 (SendEmergencyStop)
  - 级联关闭所有 MCU (sendEmergencyStopToAll)
- [x] 心跳超时检测
  - 周期性时钟查询 (~0.98s 间隔)
  - 超时阈值检测 (queriesPending > 4)
  - 自动触发 shutdown
- [x] RESTART/FIRMWARE_RESTART 命令
  - 软重启 (host 重启)
  - 硬重启 (MCU 固件重启)
  - 四种重启方法 (arduino/cheetah/command/rpi_usb)
- [x] 打印中断恢复 ✅ (2026-01-21)
  - PrintRecoveryState 完整状态结构 (位置/温度/风扇/G-code模式)
  - JSON 格式状态持久化 (原子写入)
  - 周期性自动保存 (每100行或30秒)
  - GenerateResumeGCode 恢复序列生成
  - SAVE_PRINT_STATE/CLEAR_PRINT_STATE/QUERY_PRINT_STATE 命令
  - RESUME_PRINT 从断点恢复打印
  - 9 个单元测试

---

### 阶段 4：性能优化和生产就绪

#### 4.1 性能优化 ✅ (2026-01-21)
- [x] 减少内存分配
  - 对象池系统 (`go/pkg/pool/`)
    - ArgsMap 池 - G-code 参数 map
    - Float64Slice 池 - 位置/坐标切片 (3/4/5/6/8 元素)
    - ByteBuffer 池 - 编码缓冲区
    - StringSlice 池 - 字符串切片
    - StatusMap 池 - GetStatus() 返回值
  - 17 个单元测试 + 并发测试
- [x] 优化热路径
  - `parseGCodeLineFast()` - 池化 G-code 解析器
    - 2x 速度提升 (446ns → 221ns)
    - 91% 内存减少 (533B → 48B, 7 allocs → 1 alloc)
  - `FastG1Parser` - 零分配 G1 命令解析器
    - 5x 速度提升 (446ns → 90ns)
    - 零内存分配
    - 并行性能: 16ns/op
  - 辅助函数: `floatArgFast`, `intArgFast`, `stringArg`
- [x] 基准测试套件
  - G-code 解析基准测试
  - 对象池性能对比
  - 并行性能测试

#### 4.2 日志和诊断
- [x] 结构化日志系统 ✅ (2026-01-21)
  - 多级别日志 (DEBUG, INFO, WARN, ERROR)
  - 结构化字段支持 (WithField, WithFields, WithError)
  - 双格式输出 (FormatText, FormatJSON)
  - 调用位置追踪 (SetCaller)
  - ANSI 彩色输出 (可配置)
  - 日志文件轮转 (RotatingFileWriter)
    - 按大小轮转 (MaxSize MB)
    - 备份文件限制 (MaxBackups)
    - 可选 gzip 压缩
  - 环境变量配置
    - KLIPPER_LOG_LEVEL (DEBUG/INFO/WARN/ERROR)
    - KLIPPER_LOG_FORMAT (text/json)
    - KLIPPER_LOG_CALLER (启用调用位置)
    - NO_COLOR (禁用颜色)
  - 21 个单元测试
- [x] 性能指标收集 ✅ (2026-01-21)
  - Prometheus 兼容指标系统 (`go/pkg/metrics/`)
  - 核心指标类型 (Counter, Gauge, Histogram)
    - 原子操作线程安全
    - Labels 支持多维度
    - sync.Map 高并发访问
  - 38+ Klipper 专用指标
    - 运动指标 (toolhead_position, steps_executed, lookahead_depth)
    - 温度指标 (sensor_temperature, heater_target, heater_pwm)
    - MCU 指标 (mcu_connected, mcu_latency, mcu_messages)
    - 打印指标 (print_state, print_duration, filament_used)
    - 系统指标 (go_goroutines, go_memory_heap, gc_cycles)
    - 错误指标 (errors_total, warnings_total, shutdown_events)
  - HTTP 服务器端点
    - `/metrics` - Prometheus 抓取端点
    - `/health` - 健康检查
    - `/ready` - 就绪检查
    - 可选 Basic Auth 认证
  - 辅助函数 (LinearBuckets, ExponentialBuckets)
  - 62 个单元测试 (包括并发测试和 HTTP 端点测试)
- [x] 诊断命令 ✅ (2026-01-21)
  - CommandManager 命令管理器 (diagnostics.go)
  - DEBUG_READ - 读取 MCU 内存/寄存器
  - DEBUG_WRITE - 写入 MCU 内存/寄存器
  - DEBUG_STATS - Go 运行时统计 (goroutines, memory, GC)
  - DEBUG_TIMING - 时序统计
  - STATUS - 打印机状态报告
  - HELP - 可用命令列表
  - M115 - 固件信息
  - GET_UPTIME - 主机运行时间
  - 支持十进制/十六进制地址解析
  - 动态命令注册/注销
  - 16 个单元测试

#### 4.3 生产部署
- [ ] 系统服务配置
- [ ] 配置文件兼容性验证
- [ ] 升级/回滚流程

---

## 建议的立即行动

1. **~~设置 Linux MCU 模拟器测试环境~~ ✅ 已完成** (阶段 1.1)
   - Docker 镜像和测试工具已就绪
   - 使用 `./scripts/linux-mcu/build.sh` 构建
   - 使用 `./hardware-test -tcp` 进行测试

2. **~~实现最小 Moonraker API~~ ✅ 已完成**
   - Moonraker API 服务器已实现 (`go/pkg/moonraker/`)
   - 支持 Fluidd/Mainsail 连接

3. **准备 Fluidd/Mainsail 集成测试**
   - 配置 Moonraker 端点指向 Go 服务器
   - 验证前端功能正常

4. **准备真实打印机测试** (下一步重点)
   - 选择一台测试打印机
   - 备份原有配置
   - 准备回滚方案
   - 使用 `hardware-test` 工具验证串口连接

---

## 风险和缓解措施

| 风险 | 影响 | 缓解措施 |
|------|------|----------|
| 实时性能不足 | 打印质量问题 | 使用 chelper C 代码，优化热路径 |
| MCU 通信不稳定 | 打印中断 | 实现重连机制，增加超时处理 |
| 配置不兼容 | 用户无法迁移 | 保持与 Python 版本配置兼容 |
| Moonraker 兼容性 | 前端不可用 | 严格遵循 API 规范 |

---

## 时间线估计

| 阶段 | 预计工作量 | 状态 |
|------|------------|------|
| 阶段 1.1 Linux MCU | 1 周 | ✅ 完成 |
| 阶段 1.2 真实 MCU | 1-2 周 | ⏳ 下一步 |
| 阶段 1.3 打印验证 | 1-2 周 | ⏳ 待开始 |
| 阶段 2 Moonraker | 3-4 周 | ✅ 完成 |
| 阶段 3 功能完善 | 4-6 周 | ✅ 完成 |
| 阶段 4.1-4.2 优化 | 2-3 周 | ✅ 完成 |
| 阶段 4.3 生产部署 | 1-2 周 | ⏳ 待开始 |

**当前进度：代码实现 ~95% 完成，进入硬件验证阶段**

---

## 附录：hardware-test 工具

`hardware-test` 是用于测试 MCU 连接的命令行工具。

### 支持的测试模式

| 测试 | 说明 |
|------|------|
| `serial` | 直接串口测试（无协议） |
| `connect` | MCU 连接和握手测试 |
| `clock` | 时钟同步测试 |
| `query` | MCU 能力查询 |
| `config` | 配置验证 |
| `stepper` | 步进电机测试 |
| `gpio` | GPIO 测试 |
| `endstop` | 限位开关测试 |
| `temp` | 温度传感器测试 |
| `temploop` | 温度循环测试 |
| `homing` | 归零测试 |
| `motion` | 运动测试 |
| `stress` | 压力测试 |
| `all` | 运行所有测试 |

### 连接模式

```bash
# 串口连接
./hardware-test -device /dev/ttyUSB0 -test connect

# Unix 套接字连接
./hardware-test -device /tmp/klipper_mcu -socket -test connect

# TCP 连接（Linux MCU Docker）
./hardware-test -device localhost:5555 -tcp -test connect

# 使用 printer.cfg
./hardware-test -config ~/printer.cfg -test connect

# 启用调试跟踪
./hardware-test -device /dev/ttyUSB0 -test connect -trace
```
