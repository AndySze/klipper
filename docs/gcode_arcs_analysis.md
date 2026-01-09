# gcode_arcs.test 问题分析报告

**日期**: 2026-01-09
**状态**: 已定位根本原因

## 问题表现

1. **queue_step分块边界差异**: 在`interval=8000`边界处，count值在499/500/501之间分配不同
2. **EOF motor-off时序偏移**: 约22.66ms (362614 ticks @ 16MHz)

## 根本原因分析

### 发现1: Flush间隔模式

通过trace分析发现：

**Python行为（预期）**:
- Homing期间: 50ms间隔 (DRIP_SEGMENT_TIME) ✅
- 正常运动期间: 250ms批量间隔 (BGFLUSH_SG_HIGH - BGFLUSH_SG_LOW) ❌
- EOF时: 一次性flush所有剩余

**Go行为（实际）**:
- Homing期间: 50ms间隔 ✅
- 正常运动期间: **仍然使用50ms间隔** ❌
- EOF时: 一次性flush所有剩余 ✅

**关键发现**: Go在正常运动期间没有正确使用250ms批量间隔！

### 发现2: Flush触发机制

**Python**:
```python
# _flush_handler_debug 是一个reactor timer，定期被调用
# 每次调用时批量处理多个0.25秒增量
faux_time = self.need_flush_time - 1.5
batch_time = BGFLUSH_SG_HIGH_TIME - BGFLUSH_SG_LOW_TIME  # 0.250
while self.last_step_gen_time < faux_time:
    target = self.last_step_gen_time + batch_time
    self._advance_flush_time(0., target)  # 批量处理
    flush_count += 1
```

**Go**:
```go
// flushHandlerDebugOnce 只在EOF时调用一次
// 在运动期间，通过noteMovequeueActivity -> advanceFlushTime
// 但每次都是单独调用，不是批量处理

// 运动期间: 每个move触发一次advanceFlushTime
noteMovequeueActivity(mqTime) -> advanceFlushTime(...)

// EOF时: 一次性处理所有
for i := 0; i < 10000; i++ {
    didWork := flushHandlerDebugOnce()
    if !didWork { break }
}
```

**关键差异**:
- Python: 持续批量处理（每次0.25秒）
- Go: 逐个处理（每次约50ms），然后在EOF时批量处理

## 为什么造成差异

### Step Compression对齐问题

step compress算法会将连续的step压缩成queue_step命令。当flush间隔不同时：

**Python (250ms批量)**:
- Flush at T=10.0s
- Flush at T=10.25s
- Flush at T=10.5s
- step compression在这个周期内工作，产生特定的count分配

**Go (50ms增量)**:
- Flush at T=10.0s
- Flush at T=10.05s
- Flush at T=10.10s
- ...
- Flush at T=10.25s
- step compression在更细粒度的周期内工作，产生不同的count分配

虽然最终的物理step时间应该相同，但由于step compression算法在处理边界时的舍入和对齐策略不同，导致queue_step命令的count分配产生微小差异。

### EOF时序偏移

由于Go在运动期间的flush时间点与Python不同，导致：
- `last_flush_time` 和 `last_step_gen_time` 的值略有不同
- EOF时的motor-off命令时钟 = last_flush_time + offset
- 产生22.66ms的差异

## 解决方案

### 方案A: 修改Go的运动期间flush策略（推荐）

**目标**: 让Go在正常运动期间也使用250ms批量间隔

**实现**:

1. **修改 `processLookahead` 后的行为**
```go
// 当前: 每次processLookahead后立即flush
func (th *toolhead) flushLookahead(...) {
    th.processLookahead(false)
    // ...
}

// 修改为: 收集flush需求，定期批量处理
type toolhead struct {
    // ...
    pendingFlushTime float64
    needBatchFlush bool
}

func (th *toolhead) flushLookahead(...) {
    th.processLookahead(false)
    th.needBatchFlush = true
    // 不立即flush，等待批量处理
}

func (th *toolhead) maybeFlushBatch() error {
    if !th.needBatchFlush {
        return nil
    }

    // 批量处理0.25秒增量（类似Python的_flush_handler_debug）
    fauxTime := th.motion.needFlushTime - 1.5
    for th.motion.lastStepGenTime < fauxTime {
        target := th.motion.lastStepGenTime + 0.25
        if err := th.motion.advanceFlushTime(0.0, target); err != nil {
            return err
        }
    }
    th.needBatchFlush = false
    return nil
}
```

2. **在关键点触发批量flush**
```go
func (rt *runtime) exec(cmd *gcodeCommand) error {
    // 执行命令...

    // 周期性触发批量flush
    if err := rt.toolhead.maybeFlushBatch(); err != nil {
        return err
    }
}
```

**优点**:
- 从根本上解决flush间隔差异
- 与Python行为对齐
- 可能解决EOF时序偏移

**缺点**:
- 需要重构flush逻辑
- 可能影响其他测试用例

### 方案B: 接受差异，放宽验证（快速方案）

**目标**: 对gcode_arcs使用relaxed-clock模式继续推进

**实现**:
```bash
# 在CI中对gcode_arcs使用relaxed模式
./scripts/go_migration_golden.py compare --only gcode_arcs --mode relaxed-clock
```

**优点**:
- 快速推进，继续扩展功能
- 不阻塞开发进度

**缺点**:
- 技术债累积
- 可能掩盖其他问题

### 方案C: 深度对齐step compression算法（复杂方案）

**目标**: 让Go的step compression算法与Python完全一致

**实现**:
- 详细对比C helper库的调用顺序和参数
- 可能需要修改chelper的CGO绑定
- 或者在Go中重新实现step compression

**优点**:
- 彻底解决精度问题
- 保证与Python的bit-wise一致性

**缺点**:
- 工作量大
- 可能引入新的bug
- chelper已经很复杂，重写风险高

## 推荐行动计划

### 短期（本周）

1. **验证方案A的可行性** (1-2天)
   - 实现maybeFlushBatch函数
   - 修改flushLookahead和exec函数
   - 在gcode_arcs上测试
   - 检查是否影响out_of_bounds

2. **如果方案A成功** (1天)
   - 应用到其他运动测试用例
   - 更新文档
   - 提交changes log

3. **如果方案A失败** (1天)
   - 评估方案B的可行性
   - 决定是否使用relaxed模式
   - 继续推进其他功能

### 中期（本月）

- 完成H4的所有基础测试
- 开始H5温度控制
- 保持代码的可维护性

## 附录: Trace分析数据

**Go trace统计**:
- 总advance events: 4824
- 总note events: 5129
- Flush间隔: 绝大多数为50ms
- 接近0.25s的间隔: 仅1/4823

**预期Python行为**:
- Flush间隔: 应该是250ms批量
- 在相同时间范围内应该有更少的advance events

**结论**: Go在正常运动期间没有使用批量flush策略，这是造成差异的主要原因。
