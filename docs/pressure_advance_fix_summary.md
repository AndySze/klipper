# Pressure Advance 修复总结

**日期**: 2026-01-09
**状态**: ✅ 成功修复 pressure_advance.test 及相关测试

## 问题分析

### 原始问题
`pressure_advance.test` 中 `endstop_home` 命令的位置不正确：
- **Python (expected)**: `endstop_home oid=4 clock=0` 出现在所有 queue_step 命令之后
- **Go (actual)**: `endstop_home oid=4 clock=0` 提前出现，打断了 queue_step 的生成

具体表现为：
- Python 有 10 个 queue_step 命令（包括 4 个 count=100）
- Go 只有 7 个 queue_step 命令（缺少最后 4 个 count=100）

### 根本原因

问题出在 `processLookahead` 函数中：

1. **Go 的原始实现**：
   ```go
   moves := th.lookahead.flush(lazy)
   if len(moves) == 0 {
       return nil  // ← 问题：提前返回，不调用 noteMovequeueActivity
   }
   ```

2. **Python 的实现**：
   ```python
   def _process_lookahead(self, lazy=False):
       moves = self.lookahead.flush(lazy=lazy)
       if not moves:
           return  # ← 但在调用之前总是调用 note_mcu_movequeue_activity
       # ... 处理 moves ...
       self.motion_queuing.note_mcu_movequeue_activity(next_move_time)
   ```

3. **问题的影响**：
   - 在 homing 期间的 retract 移动中，`addMove` 可能返回 `false`（队列未达到 flush 条件）
   - `lookahead.flush()` 返回空 moves 列表
   - Go 提前返回，不调用 `noteMovequeueActivity`
   - `needStepGenTime` 没有被更新
   - 随后的 `flushStepGeneration` 只能 flush 到旧的 `needStepGenTime`
   - 结果：retract 移动的 steps 没有被完全生成，`endstop_home` 提前出现

## 修复方案

### 修改 1: processLookahead 处理空 moves

**文件**: `go/pkg/hosth4/runtime.go`
**位置**: 行 522-524

```go
moves := th.lookahead.flush(lazy)
if len(moves) == 0 {
    // Match Python: still note activity even if no moves to process
    return th.motion.noteMovequeueActivity(th.printTime, true)
}
```

**理由**: 确保即使没有 moves 待处理，也会调用 `noteMovequeueActivity` 更新 `needStepGenTime`。

### 修改 2: processLookahead 使用正确的 isStepGen 参数

**文件**: `go/pkg/hosth4/runtime.go`
**位置**: 行 564

```go
// 修改前
return th.motion.noteMovequeueActivity(nextMoveTime, false)

// 修改后
// Note the activity with stepgen=true to match Python's behavior
// Python's _process_lookahead calls note_mcu_movequeue_activity with default is_step_gen=True
return th.motion.noteMovequeueActivity(nextMoveTime, true)
```

**理由**: Python 的 `note_mcu_movequeue_activity` 默认参数是 `is_step_gen=True`，Go 应该匹配这一行为。

### 修改 3: flushStepGeneration 添加保护性检查

**文件**: `go/pkg/hosth4/runtime.go`
**位置**: 行 580-589

```go
func (th *toolhead) flushStepGeneration() error {
    if err := th.flushLookahead(false); err != nil {
        return err
    }
    // If processLookahead wasn't called (e.g., addMove returned false),
    // needStepGenTime may not be updated. Ensure it's at least needFlushTime.
    if th.motion.needStepGenTime < th.motion.needFlushTime {
        th.motion.needStepGenTime = th.motion.needFlushTime
    }
    return th.motion.flushAllSteps()
}
```

**理由**: 作为额外的保护，确保 `needStepGenTime` 至少被更新到 `needFlushTime`。

## 测试结果

### 修复前

| 测试用例 | 状态 | 备注 |
|---------|------|------|
| pressure_advance.test | ❌ FAILED | endstop_home 位置错误 |
| bed_screws.test | ❌ FAILED | 同样的问题 |
| bltouch.test | ❌ FAILED | 同样的问题 |
| macros.test | ❌ FAILED | 同样的问题 |

### 修复后

| 测试用例 | 状态 | 备注 |
|---------|------|------|
| commands.test | ✅ PASSED | 基础命令测试 |
| linuxtest.test | ✅ PASSED | 多 MCU 测试 |
| out_of_bounds.test | ✅ PASSED | 边界检查 |
| extruders.test | ✅ PASSED | 挤出机功能 |
| manual_stepper.test | ✅ PASSED | 手动步进 |
| **pressure_advance.test** | ✅ **PASSED** | **新修复！** |
| **bed_screws.test** | ✅ **PASSED** | **新修复！** |
| **bltouch.test** | ✅ **PASSED** | **新修复！** |
| **macros.test** | ✅ **PASSED** | **新修复！** |
| gcode_arcs.test | ⚠️ 98.5% 改进 | EOF 时序从 22.66ms → 0.34ms，剩余 bit-wise 差异 |

## 关键发现

1. **Python 和 Go 的细微差异**：
   - Python 的 `note_mcu_movequeue_activity` 默认参数是 `is_step_gen=True`
   - Go 需要明确匹配这一行为

2. **空 moves 处理的重要性**：
   - 即使没有 moves 待处理，也需要调用 `noteMovequeueActivity`
   - 这确保了 `needStepGenTime` 被正确更新

3. **Lookahead 队列的 flush 条件**：
   - `addMove` 返回 `false` 不意味着没有移动
   - 可能只是队列未达到 flush 条件
   - 但 `flushLookahead` 仍然需要调用 `noteMovequeueActivity`

## 影响范围

这个修复解决了：
- ✅ pressure_advance.test
- ✅ bed_screws.test
- ✅ bltouch.test
- ✅ macros.test

同时保持了：
- ✅ 所有之前通过的测试用例（commands, linuxtest, out_of_bounds, extruders, manual_stepper）

未影响：
- ⚠️ gcode_arcs.test（仍然有之前已知的小差异）

## 下一步建议

1. **继续优化 gcode_arcs.test**（可选）：
   - 解决剩余的 bit-wise 差异
   - 但这可能需要更多工作

2. **实现新功能**（推荐）：
   - 继续扩展功能覆盖
   - 实现更多 Klipper 功能

3. **性能优化**（可选）：
   - 优化批量 flush 策略
   - 改进 step generation 性能

## 文件修改清单

- `go/pkg/hosth4/runtime.go`:
  - `processLookahead` (行 522-524): 处理空 moves 时仍然调用 `noteMovequeueActivity`
  - `processLookahead` (行 564): 使用 `isStepGen=true`
  - `flushStepGeneration` (行 586-588): 添加 `needStepGenTime` 保护性更新

## 参考资料

- Python 实现: `klippy/toolhead.py` - `_process_lookahead`, `note_mcu_movequeue_activity`
- Python 实现: `klippy/extras/motion_queuing.py` - `flush_all_steps`, `_advance_flush_time`
- Go 实现: `go/pkg/hosth4/runtime.go` - `processLookahead`, `flushStepGeneration`
