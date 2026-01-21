# Go Host 迁移：下一步开发计划

## 当前状态（2026-01-20）

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

#### 1.1 Linux MCU 模拟器测试
- [ ] 使用 `linuxprocess` MCU 进行端到端测试
- [ ] 验证实时时钟同步
- [ ] 验证命令收发完整性

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
- [ ] `tmc2209` - TMC2209 驱动

#### 3.2 高级功能
- [ ] Bed mesh 补偿
- [ ] Pressure advance 微调
- [ ] Input shaper 自动调谐
- [ ] 多挤出机支持

#### 3.3 错误处理和恢复
- [ ] MCU 断连重连
- [ ] 打印中断恢复
- [ ] 紧急停止处理

---

### 阶段 4：性能优化和生产就绪

#### 4.1 性能优化
- [ ] 减少内存分配
- [ ] 优化热路径
- [ ] 并行处理优化

#### 4.2 日志和诊断
- [ ] 结构化日志
- [ ] 性能指标收集
- [ ] 诊断命令

#### 4.3 生产部署
- [ ] 系统服务配置
- [ ] 配置文件兼容性验证
- [ ] 升级/回滚流程

---

## 建议的立即行动

1. **设置 Linux MCU 模拟器测试环境** (阶段 1.1)
   ```bash
   # 构建 Linux MCU 模拟器
   make flash FLASH_DEVICE=/tmp/klipper_host_mcu
   ```

2. **~~实现最小 Moonraker API~~ ✅ 已完成**
   - Moonraker API 服务器已实现 (`go/pkg/moonraker/`)
   - 支持 Fluidd/Mainsail 连接

3. **准备 Fluidd/Mainsail 集成测试**
   - 配置 Moonraker 端点指向 Go 服务器
   - 验证前端功能正常

4. **准备真实打印机测试**
   - 选择一台测试打印机
   - 备份原有配置
   - 准备回滚方案

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

| 阶段 | 预计工作量 | 前置条件 |
|------|------------|----------|
| 阶段 1 | 2-3 周 | 测试设备就绪 |
| 阶段 2 | 3-4 周 | 阶段 1 完成 |
| 阶段 3 | 4-6 周 | 阶段 2 完成 |
| 阶段 4 | 2-4 周 | 阶段 3 完成 |

**总计：约 3-4 个月达到生产就绪状态**
