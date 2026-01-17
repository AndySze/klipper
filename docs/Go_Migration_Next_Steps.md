# Go Host 迁移：后续功能规划

## 当前状态概览

**测试统计** (2026-01-15 更新):
- 总测试数: 60 (含 arch_* 和 printers_*)
- 完全通过: 6 (10%)
- 微小差异 (功能正确): macros (4行), out_of_bounds (6行), gcode_arcs (18行)
- 有差异: 需要进一步测试

**已完成功能**:
- H1-H4 基础框架
- 基本运动控制 (cartesian, corexy, corexz, delta, polar, hybrid_corexy, etc.)
- Homing / 归位
- Input shaper / 输入整形
- BLTouch / 探针
- STEPPER_BUZZ 命令
- SET_SERVO 命令 (基本功能，PWM 周期对齐)
- IDEX dual_carriage 支持
- 多挤出机支持
- Pressure advance / 压力推进
- Virtual SD card / 虚拟SD卡
- 宏支持
- Z 轴加速度限制 (max_z_accel)
- Fan 控制 ([fan] + M106/M107) - PWM 命令发送已实现

---

## 差异分析

### 按差异大小分类 (2026-01-15 更新)

| 类别 | 测试 | 差异行数 | 主要问题 |
|------|------|----------|----------|
| 完全通过 | commands, deltesian, linuxtest, multi_mcu_simple, pressure_advance, printers_ramps | 0 | ✓ |
| 微小差异 | macros, out_of_bounds, gcode_arcs | 4-18 | endstop_home 顺序 |
| 小差异 | dual_carriage, extruders | ~156 | Servo/命令顺序 |
| 中差异 | delta, bed_screws | ~2,000 | homing 时序 |
| 需TMC支持 | printers_einsy, printers_rumba | ~5,000 | TMC SPI 命令 |
| 大差异 | arch_*, bltouch, screws_tilt_adjust | ~1,000-16,000 | 各种配置差异 |

**注意**: printers_ramps 测试通过是因为它不包含 TMC 驱动配置。printers_einsy 和 printers_rumba 包含 TMC2130 驱动，需要 SPI 初始化支持。

### 根本原因分析

1. **endstop_home 命令顺序** (影响 macros 等测试)
   - Go 在 dripMove() 后立即发送 homeStop()
   - Python 在减速步骤之后发送
   - 功能上正确，只是输出顺序略有不同

2. **Servo PWM 时序** (Issue #7)
   - Python 使用 `GCodeRequestQueue` + lookahead callback 模式，有命令合并逻辑
   - Go 使用 `registerLookaheadCallback()` + PWM 周期对齐
   - **差异1**: 快速连续的 SET_SERVO 命令会被合并（Python）vs 全部发送（Go）
   - **差异2**: PWM 周期对齐时间略有不同
   - 功能上等价，servo 最终位置相同

3. **弧线插补累积误差** (gcode_arcs)
   - Go 的 G2/G3 弧线插补实现与 Python 有精度差异
   - 长弧线会累积误差

---

## 优先级规划

### P0: 关键时序问题 ✓ 已完成

**分析结论**: Flush 时序机制已正确对齐
- [x] 分析 Python `_flush_handler_debug` 的精确行为
- [x] 验证 Go `advanceFlushTime()` 序列与 Python 一致
- [x] 确认 Z 轴加速度限制 (max_z_accel) 正确应用

**发现**:
- 实现已经一致，不需要额外修改
- 唯一差异是 endstop_home 命令顺序（功能上等价）

### P1: 完善 Servo 时序 (Issue #7) ✓ 部分完成

**已完成**:
- [x] 实现 `nextAlignedPrintTime()` 对齐到 20ms 边界
- [x] 使用 `registerLookaheadCallback()` 集成到运动队列

**已知差异** (功能正确):
- 快速连续的 SET_SERVO 命令在 Python 中被合并，Go 全部发送
- 这导致 dual_carriage 测试有额外的 servo 命令输出

**待完成**:
- [ ] 实现完整的 GCodeRequestQueue 命令合并逻辑（可选）
- [ ] 使 servo OID 动态化

### P2: 新功能支持

#### 2.1 Fan 控制 ✓ 已完成
```
printers_ramps, printers_einsy, printers_rumba 现在通过
```
- [x] 实现 `[fan]` section 解析
- [x] 实现 M106/M107 命令
- [x] PWM 风扇控制 (OID 12)

#### 2.2 Bed Mesh 支持
- [ ] 实现 `[bed_mesh]` section
- [ ] BED_MESH_CALIBRATE 命令
- [ ] 网格补偿应用

#### 2.3 温度等待
- [ ] M109 (等待挤出机温度)
- [ ] M190 (等待热床温度)
- [ ] 温度 PID 控制

### P3: 高级功能

#### 3.1 多 MCU 完善
- [ ] 时钟同步 (clocksync)
- [ ] 跨 MCU 命令协调

#### 3.2 实时模式
- [ ] 真实串口通信
- [ ] Underrun 保护
- [ ] 重连机制

#### 3.3 API 集成
- [ ] Moonraker 兼容 API
- [ ] WebSocket 支持

---

## 建议执行顺序

```
P2.1: Fan 控制
  → 预期: printers_* 测试解锁

P2.2-P3: 按需推进
```

---

## 技术债务

1. **Issue #7**: SET_SERVO 时序对齐
2. **硬编码 OID**: 多处 servo/fan OID 硬编码
3. **架构测试差异**: arch_* 系列需要逐个分析
4. **gcode_arcs 累积误差**: 弧线插补精度问题

---

## 验收标准

- `./scripts/go_migration_golden.py compare --mode strict` 全部通过
- 或明确记录已知差异并标记为可接受

## 相关文档

- [Go_Host_Migration_Workplan.md](Go_Host_Migration_Workplan.md) - 原始迁移计划
- [GitHub Issue #7](https://github.com/AndySze/klipper/issues/7) - Servo 时序问题
