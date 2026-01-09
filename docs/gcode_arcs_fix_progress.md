# gcode_arcs 修复进展报告

**日期**: 2026-01-09
**修改**: 实现了250ms批量flush间隔
**状态**: 显著改进但未完全解决

## 修改内容

### 1. 添加了批量flush函数

在`go/pkg/hosth4/runtime.go`的`motionQueuing`中添加了`flushPendingBatch()`方法：

```go
// flushPendingBatch advances step generation in ~0.25s batches during normal motion.
// This matches Python's _flush_handler_debug behavior for fileoutput mode.
func (mq *motionQueuing) flushPendingBatch() error {
    fauxTime := mq.needFlushTime - bgflushFauxTimeOffset
    batchTime := bgflushSGHighTimeSec - bgflushSGLowTimeSec  // 0.250
    flushCount := 0
    for mq.lastStepGenTime < fauxTime {
        target := mq.lastStepGenTime + batchTime
        if flushCount > 100 && fauxTime > target {
            skip := math.Floor((fauxTime - target) / batchTime)
            target += skip * batchTime
        }
        if err := mq.advanceFlushTime(0.0, target); err != nil {
            return err
        }
        flushCount++
    }
    return nil
}
```

### 2. 修改了processLookahead

将`noteMovequeueActivity`的`isStepGen`参数改为`false`，不立即触发step generation flush：

```go
// 修改前
return th.motion.noteMovequeueActivity(nextMoveTime, true)

// 修改后
// Just note the activity, don't flush immediately - let batch flushing handle it
return th.motion.noteMovequeueActivity(nextMoveTime, false)
```

### 3. 在exec后调用批量flush

在每个G-code命令执行后调用批量flush：

```go
func (r *runtime) exec(cmd *gcodeCommand) error {
    // ... 执行命令 ...

    // After executing each gcode command, trigger batch flushing
    // This matches Python's _flush_handler_debug behavior
    return r.toolhead.motion.flushPendingBatch()
}
```

## 测试结果

### ✅ out_of_bounds.test - PASSED

修改后仍然通过strict compare，没有破坏现有功能。

### ⚠️ gcode_arcs.test - 改进但未完全解决

**修改前**:
- EOF motor-off时序偏移: 362614 ticks @ 16MHz ≈ 22.66ms
- queue_step分块: 严重的count分配差异

**修改后**:
- EOF motor-off时序偏移: 5386 ticks @ 16MHz ≈ 0.34ms  (**改进了98.5%!**)
- queue_step分块: 仍有微小差异，但已大幅改善

### 差异示例

**Homing期间的时间差异** (修改后的新问题):
```
# Python (expected)
trsync_start oid=1 report_clock=102229333
endstop_home oid=0 clock=102229333

# Go (actual)
trsync_start oid=1 report_clock=102197333
endstop_home oid=0 clock=102197333

# 差异: 32000 ticks = 2ms
```

**正常运动期间的差异** (仍然存在):
```
# Python
queue_step oid=8 interval=8001 count=250 add=0

# Go (修改后)
queue_step oid=8 interval=8000 count=250 add=0
```

## 分析

### 改进的原因

批量flush策略确实有效，让Go与Python的flush时机更加接近。EOF时序偏移从22.66ms减少到0.34ms，证明了方向是正确的。

### 剩余问题

1. **Homing期间的时间对齐**: homing使用drip模式（50ms间隔），与正常运动的批量模式（250ms）之间存在边界对齐问题

2. **Step compression的舍入差异**: 即使flush时机接近，step compression算法在处理边界时仍有微小差异

3. **时钟累积误差**: 每次2ms的微小差异在长时间运行中累积

## 下一步选项

### 选项1: 精细调整homing与正常运动的边界 (推荐)

在homing结束后，对齐flush时间到下一个批量边界：

```go
func (r *runtime) homeAll() error {
    // ... existing homing code ...

    // After homing, align to next batch boundary
    if err := r.toolhead.motion.flushPendingBatch(); err != nil {
        return err
    }
    return r.gm.resetFromToolhead()
}
```

### 选项2: 使用relaxed-clock模式 (快速推进)

接受当前的改进成果，对gcode_arcs使用relaxed-clock模式继续推进：

```bash
python3 scripts/go_migration_golden.py compare --only gcode_arcs --mode relaxed-clock
```

### 选项3: 深度对齐step compression (长期)

继续改进step compression的对齐，但这可能需要更多时间。

## 建议

考虑到：
1. EOF时序偏移已从22.66ms改进到0.34ms（98.5%改进）
2. out_of_bounds等重要测试用例已经通过
3. 剩余差异主要在homing边界和step compression的舍入

**建议**:
- 短期：使用relaxed-clock模式继续推进其他功能（extruders、heaters等）
- 中期：作为技术债记录，后续优化
- 长期：在H5硬件闭环时重新评估（可能需要实时测试）

## 文件修改清单

- `go/pkg/hosth4/runtime.go`:
  - 添加`flushPendingBatch()`方法
  - 修改`processLookahead()`的`isStepGen`参数
  - 修改`exec()`在每条命令后调用批量flush
