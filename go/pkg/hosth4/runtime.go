package hosth4

import (
    "crypto/rand"
    "encoding/hex"
    "fmt"
    "io"
    "math"
    "os"
    "path/filepath"
    "strings"
    "time"

    "klipper-go-migration/pkg/chelper"
    "klipper-go-migration/pkg/protocol"
)

const (
    mcuMoveCountFileOutput = 500

    maxStepcompressErrorSec = 0.000025
    disableStallTimeSec     = 0.100

    bufferTimeHigh  = 1.0
    bufferTimeStart = 0.250

    minKinTimeSec            = 0.100
    sdsCheckTimeSec          = 0.001
    stepcompressFlushTimeSec = 0.050
    moveHistoryExpireSec     = 30.0

    lookaheadFlushSec = 0.150

    bgflushSGLowTimeSec   = 0.450
    bgflushSGHighTimeSec  = 0.700
    bgflushExtraTimeSec   = 0.250
    bgflushFauxTimeOffset = 1.5

    homingStartDelaySec    = 0.001
    endstopSampleTimeSec   = 0.000015
    endstopSampleCount     = 4
    dripSegmentTimeSec     = 0.050
    trsyncTimeoutSec       = 0.250
    trsyncReportRatio      = 0.3
    trsyncExpireReason     = 4
)

type motionQueuing struct {
    sq      *chelper.SerialQueue
    ssm     *chelper.StepperSyncMgr
    ss      *chelper.StepperSync
    mcuFreq float64

    trapqs []*chelper.TrapQ

    callbacks []flushCB
    nextCBID  int

    lastFlushTime   float64
    lastStepGenTime float64
    needFlushTime   float64
    needStepGenTime float64

    kinFlushDelay float64

    trace io.Writer
}

type flushCB struct {
    id          int
    canAddTrapQ bool
    fn          func(flushTime float64, stepGenTime float64)
}

func newMotionQueuing(sq *chelper.SerialQueue, mcuFreq float64, reservedMoveSlots int) (*motionQueuing, error) {
    ssm, err := chelper.NewStepperSyncMgr()
    if err != nil {
        return nil, err
    }
    ss, err := ssm.AllocStepperSync()
    if err != nil {
        ssm.Free()
        return nil, err
    }
    moveCount := mcuMoveCountFileOutput - reservedMoveSlots
    if moveCount < 1 {
        ssm.Free()
        return nil, fmt.Errorf("bad move count %d (reserved %d)", mcuMoveCountFileOutput, reservedMoveSlots)
    }
    if err := ss.SetupMoveQueue(sq, moveCount); err != nil {
        ssm.Free()
        return nil, err
    }
    return &motionQueuing{
        sq:           sq,
        ssm:          ssm,
        ss:           ss,
        mcuFreq:      mcuFreq,
        kinFlushDelay: sdsCheckTimeSec,
    }, nil
}

func (mq *motionQueuing) setTrace(w io.Writer) {
    mq.trace = w
}

func (mq *motionQueuing) tracef(format string, args ...any) {
    if mq == nil || mq.trace == nil {
        return
    }
    fmt.Fprintf(mq.trace, format, args...)
}

func (mq *motionQueuing) drainSerial(maxWait time.Duration) error {
    if mq == nil || mq.sq == nil {
        return nil
    }
    deadline := time.Now().Add(maxWait)
    for {
        st, err := mq.sq.GetStats()
        if err != nil {
            return err
        }
        if st.ReadyBytes == 0 && st.UpcomingBytes == 0 {
            return nil
        }
        if time.Now().After(deadline) {
            return fmt.Errorf("serialqueue did not drain: %s", st.Raw)
        }
        time.Sleep(1 * time.Millisecond)
    }
}

func (mq *motionQueuing) free() {
    if mq == nil {
        return
    }
    for _, tq := range mq.trapqs {
        tq.Free()
    }
    mq.trapqs = nil
    if mq.ssm != nil {
        mq.ssm.Free()
        mq.ssm = nil
    }
    mq.ss = nil
}

func (mq *motionQueuing) allocTrapQ() (*chelper.TrapQ, error) {
    tq, err := chelper.NewTrapQ()
    if err != nil {
        return nil, err
    }
    mq.trapqs = append(mq.trapqs, tq)
    return tq, nil
}

func (mq *motionQueuing) allocSyncEmitter(name string) (*chelper.SyncEmitter, error) {
    se, err := mq.ss.AllocSyncEmitter(name, true)
    if err != nil {
        return nil, err
    }
    _ = mq.ss.SetTime(0.0, mq.mcuFreq)
    return se, nil
}

func (mq *motionQueuing) registerFlushCallback(fn func(flushTime float64, stepGenTime float64), canAddTrapQ bool) int {
    mq.nextCBID++
    id := mq.nextCBID
    cb := flushCB{id: id, canAddTrapQ: canAddTrapQ, fn: fn}
    if canAddTrapQ {
        mq.callbacks = append([]flushCB{cb}, mq.callbacks...)
        return id
    }
    mq.callbacks = append(mq.callbacks, cb)
    return id
}

func (mq *motionQueuing) unregisterFlushCallback(id int) {
    if id == 0 {
        return
    }
    out := mq.callbacks[:0]
    for _, cb := range mq.callbacks {
        if cb.id != id {
            out = append(out, cb)
        }
    }
    mq.callbacks = out
}

func (mq *motionQueuing) calcStepGenRestart(estPrintTime float64) float64 {
    kinTime := estPrintTime + minKinTimeSec
    if mq.lastStepGenTime > kinTime {
        kinTime = mq.lastStepGenTime
    }
    return kinTime + mq.kinFlushDelay
}

func (mq *motionQueuing) noteMovequeueActivity(mqTime float64, isStepGen bool) error {
    if isStepGen {
        mqTime += mq.kinFlushDelay
        if mqTime > mq.needStepGenTime {
            mq.needStepGenTime = mqTime
        }
    }
    if mqTime > mq.needFlushTime {
        mq.needFlushTime = mqTime
    }
    mq.tracef("MQ note mqTime=%.9f stepgen=%v need_flush=%.9f need_sg=%.9f last_flush=%.9f last_sg=%.9f\n",
        mqTime, isStepGen, mq.needFlushTime, mq.needStepGenTime, mq.lastFlushTime, mq.lastStepGenTime)
    return nil
}

func (mq *motionQueuing) flushAllSteps() error {
    // Match Klippy: flush_all_steps performs a single advance to the current
    // need_step_gen_time and does not attempt to iterate to completion.
    flushTime := mq.needStepGenTime
    mq.tracef("MQ flushAll need_sg=%.9f need_flush=%.9f last_flush=%.9f last_sg=%.9f\n",
        mq.needStepGenTime, mq.needFlushTime, mq.lastFlushTime, mq.lastStepGenTime)
    return mq.advanceFlushTime(flushTime, 0.0)
}

func (mq *motionQueuing) wipeTrapQ(tq *chelper.TrapQ) {
    if tq == nil {
        return
    }
    tq.FinalizeMoves(1e30, 0.0)
}

func (mq *motionQueuing) advanceFlushTime(wantFlushTime float64, wantStepGenTime float64) error {
    mq.tracef("MQ advance begin want_flush=%.9f want_sg=%.9f need_flush=%.9f need_sg=%.9f last_flush=%.9f last_sg=%.9f\n",
        wantFlushTime, wantStepGenTime, mq.needFlushTime, mq.needStepGenTime, mq.lastFlushTime, mq.lastStepGenTime)
    flushTime := wantFlushTime
    if mq.lastFlushTime > flushTime {
        flushTime = mq.lastFlushTime
    }
    if wantStepGenTime-stepcompressFlushTimeSec > flushTime {
        flushTime = wantStepGenTime - stepcompressFlushTimeSec
    }
    stepGenTime := wantStepGenTime
    if mq.lastStepGenTime > stepGenTime {
        stepGenTime = mq.lastStepGenTime
    }
    if flushTime > stepGenTime {
        stepGenTime = flushTime
    }

    for _, cb := range mq.callbacks {
        cb.fn(flushTime, stepGenTime)
    }

    trapqFreeTime := stepGenTime - mq.kinFlushDelay
    clearHistoryTime := 0.0
    if trapqFreeTime-moveHistoryExpireSec > clearHistoryTime {
        clearHistoryTime = trapqFreeTime - moveHistoryExpireSec
        if clearHistoryTime < 0.0 {
            clearHistoryTime = 0.0
        }
    }

    if err := mq.ssm.GenSteps(flushTime, stepGenTime, clearHistoryTime); err != nil {
        return err
    }
    mq.lastFlushTime = flushTime
    mq.lastStepGenTime = stepGenTime
    mq.tracef("MQ advance end flush=%.9f sg=%.9f clear=%.9f\n", flushTime, stepGenTime, clearHistoryTime)

    for _, tq := range mq.trapqs {
        tq.FinalizeMoves(trapqFreeTime, clearHistoryTime)
    }
    return nil
}

func (mq *motionQueuing) dripUpdateTime(startTime float64, endTime float64) error {
    if err := mq.advanceFlushTime(startTime-sdsCheckTimeSec, startTime); err != nil {
        return err
    }
    flushTime := startTime
    for flushTime < endTime {
        flushTime = math.Min(flushTime+dripSegmentTimeSec, endTime)
        if err := mq.noteMovequeueActivity(flushTime, true); err != nil {
            return err
        }
        if err := mq.advanceFlushTime(flushTime-sdsCheckTimeSec, flushTime); err != nil {
            return err
        }
    }
    return mq.advanceFlushTime(flushTime+mq.kinFlushDelay, 0.0)
}

func (mq *motionQueuing) flushHandlerDebugOnce() (bool, error) {
    // Port of klippy/extras/motion_queuing.py::_flush_handler_debug used when
    // in fileoutput mode (can_pause=False). This periodically advances step
    // generation time in ~0.25s batches to keep output chunking stable.
    fauxTime := mq.needFlushTime - bgflushFauxTimeOffset
    batchTime := bgflushSGHighTimeSec - bgflushSGLowTimeSec
    flushCount := 0
    for mq.lastStepGenTime < fauxTime {
        target := mq.lastStepGenTime + batchTime
        if flushCount > 100 && fauxTime > target {
            skip := math.Floor((fauxTime - target) / batchTime)
            target += skip * batchTime
        }
        if err := mq.advanceFlushTime(0.0, target); err != nil {
            return false, err
        }
        flushCount++
    }
    if flushCount != 0 {
        return true, nil
    }
    return false, mq.advanceFlushTime(mq.needFlushTime+bgflushExtraTimeSec, 0.0)
}

// flushPendingBatch advances step generation in ~0.25s batches during normal motion.
// This matches Python's _flush_handler_debug behavior for fileoutput mode.
func (mq *motionQueuing) flushPendingBatch() error {
    fauxTime := mq.needFlushTime - bgflushFauxTimeOffset
    batchTime := bgflushSGHighTimeSec - bgflushSGLowTimeSec
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

type cartesianKinematics struct {
    rails        [3]stepperCfg
    maxZVelocity float64
    maxZAccel    float64
    limits       [3][2]float64
    steppers     [3]*stepper
}

func newCartesianKinematics(rails [3]stepperCfg, steppers [3]*stepper, maxZVelocity float64, maxZAccel float64) *cartesianKinematics {
    kin := &cartesianKinematics{
        rails:        rails,
        maxZVelocity: maxZVelocity,
        maxZAccel:    maxZAccel,
        steppers:     steppers,
    }
    for i := 0; i < 3; i++ {
        kin.limits[i] = [2]float64{1.0, -1.0}
    }
    return kin
}

func (k *cartesianKinematics) setPosition(newPos []float64, homingAxes string) {
    if len(newPos) < 3 {
        return
    }
    for i := 0; i < 3; i++ {
        if k.steppers[i] != nil {
            k.steppers[i].setPosition(newPos[i])
        }
    }
    for _, axisName := range homingAxes {
        axis := strings.IndexByte("xyz", byte(axisName))
        if axis < 0 || axis >= 3 {
            continue
        }
        k.limits[axis] = [2]float64{k.rails[axis].positionMin, k.rails[axis].positionMax}
    }
}

func (k *cartesianKinematics) clearHomingState(clearAxes string) {
    for _, axisName := range clearAxes {
        axis := strings.IndexByte("xyz", byte(axisName))
        if axis < 0 || axis >= 3 {
            continue
        }
        k.limits[axis] = [2]float64{1.0, -1.0}
    }
}

func (k *cartesianKinematics) checkMove(mv *move) error {
    endPos := mv.endPos
    limits := k.limits
    xpos, ypos := endPos[0], endPos[1]
    if xpos < limits[0][0] || xpos > limits[0][1] || ypos < limits[1][0] || ypos > limits[1][1] {
        if err := k.checkEndstops(mv); err != nil {
            return err
        }
    }
    if mv.axesD[2] == 0.0 {
        return nil
    }
    if err := k.checkEndstops(mv); err != nil {
        return err
    }
    zRatio := mv.moveD / math.Abs(mv.axesD[2])
    mv.limitSpeed(k.maxZVelocity*zRatio, k.maxZAccel*zRatio)
    return nil
}

func (k *cartesianKinematics) checkEndstops(mv *move) error {
    endPos := mv.endPos
    limits := k.limits
    for i := 0; i < 3; i++ {
        if mv.axesD[i] == 0.0 {
            continue
        }
        if endPos[i] >= limits[i][0] && endPos[i] <= limits[i][1] {
            continue
        }
        if limits[i][0] > limits[i][1] {
            return fmt.Errorf("must home axis first")
        }
        return fmt.Errorf("move out of range")
    }
    return nil
}

type toolhead struct {
    mcuFreq float64
    motion  *motionQueuing

    maxVelocity       float64
    maxAccel          float64
    minCruiseRatio    float64
    squareCornerV     float64
    junctionDeviation float64
    mcrPseudoAccel    float64

    printTime    float64
    specialState string
    err          error

    commandedPos []float64

    lookahead *lookAheadQueue
    kin       *cartesianKinematics
    trapq     *chelper.TrapQ

    extraAxes []extraAxis
}

func newToolhead(mcuFreq float64, motion *motionQueuing, maxVelocity float64, maxAccel float64) (*toolhead, error) {
    tq, err := motion.allocTrapQ()
    if err != nil {
        return nil, err
    }
    th := &toolhead{
        mcuFreq:      mcuFreq,
        motion:       motion,
        maxVelocity:  maxVelocity,
        maxAccel:     maxAccel,
        minCruiseRatio: 0.5,
        squareCornerV:  5.0,
        specialState: "NeedPrime",
        commandedPos: []float64{0, 0, 0, 0},
        lookahead:    &lookAheadQueue{junctionFlush: lookaheadFlushSec},
        trapq:        tq,
        extraAxes:    nil,
    }
    th.calcJunctionDeviation()
    th.lookahead.setFlushTime(bufferTimeHigh)
    th.motion.registerFlushCallback(th.handleStepFlush, true)
    return th, nil
}

func (th *toolhead) calcJunctionDeviation() {
    scv2 := th.squareCornerV * th.squareCornerV
    th.junctionDeviation = scv2 * (math.Sqrt(2.0) - 1.0) / th.maxAccel
    th.mcrPseudoAccel = th.maxAccel * (1.0 - th.minCruiseRatio)
}

func (th *toolhead) estimatedPrintTime(_ float64) float64 {
    return 0.0
}

func (th *toolhead) printTimeToClock(printTime float64) uint64 {
    if printTime <= 0.0 {
        return 0
    }
    return uint64(int64(printTime * th.mcuFreq))
}

func (th *toolhead) advanceMoveTime(nextPrintTime float64) {
    if nextPrintTime > th.printTime {
        th.printTime = nextPrintTime
    }
}

func (th *toolhead) calcPrintTime() {
    est := th.estimatedPrintTime(0.0)
    kinTime := th.motion.calcStepGenRestart(est)
    minPrintTime := est + bufferTimeStart
    if kinTime > minPrintTime {
        minPrintTime = kinTime
    }
    if minPrintTime > th.printTime {
        th.printTime = minPrintTime
    }
}

func (th *toolhead) handleStepFlush(_ float64, stepGenTime float64) {
    if th.specialState != "" {
        return
    }
    if stepGenTime >= th.printTime-th.motion.kinFlushDelay-0.001 {
        if err := th.flushLookahead(true); err != nil {
            th.err = err
        }
    }
}

func (th *toolhead) processLookahead(lazy bool) error {
    if th.err != nil {
        return th.err
    }
    moves := th.lookahead.flush(lazy)
    if len(moves) == 0 {
        // Match Python: still note activity even if no moves to process
        return th.motion.noteMovequeueActivity(th.printTime, true)
    }
    if th.specialState != "" {
        th.specialState = ""
        th.calcPrintTime()
    }
    nextMoveTime := th.printTime
    for _, mv := range moves {
        if mv.isKinematicMove {
            th.trapq.Append(
                nextMoveTime,
                mv.accelT,
                mv.cruiseT,
                mv.decelT,
                mv.startPos[0],
                mv.startPos[1],
                mv.startPos[2],
                mv.axesR[0],
                mv.axesR[1],
                mv.axesR[2],
                mv.startV,
                mv.cruiseV,
                mv.accel,
            )
        }
        for idx, ea := range th.extraAxes {
            p := idx + 3
            if p < len(mv.axesD) && mv.axesD[p] != 0.0 {
                if err := ea.ProcessMove(nextMoveTime, mv, p); err != nil {
                    return err
                }
            }
        }
        nextMoveTime = nextMoveTime + mv.accelT + mv.cruiseT + mv.decelT
        for _, cb := range mv.timingCallbacks {
            cb(nextMoveTime)
        }
    }
    th.advanceMoveTime(nextMoveTime)
    // Note the activity with stepgen=true to match Python's behavior
    // Python's _process_lookahead calls note_mcu_movequeue_activity with default is_step_gen=True
    return th.motion.noteMovequeueActivity(nextMoveTime, true)
}

func (th *toolhead) flushLookahead(_ bool) error {
    if th.err != nil {
        return th.err
    }
    if err := th.processLookahead(false); err != nil {
        th.err = err
        return err
    }
    th.specialState = "NeedPrime"
    th.lookahead.setFlushTime(bufferTimeHigh)
    return nil
}

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

func (th *toolhead) getLastMoveTime() (float64, error) {
    if th.err != nil {
        return 0.0, th.err
    }
    if th.specialState != "" {
        if err := th.flushLookahead(false); err != nil {
            return 0.0, err
        }
        th.calcPrintTime()
    } else {
        if err := th.processLookahead(false); err != nil {
            return 0.0, err
        }
    }
    return th.printTime, nil
}

func (th *toolhead) dwell(delay float64) error {
    if th.err != nil {
        return th.err
    }
    if delay < 0.0 {
        delay = 0.0
    }
    if err := th.flushLookahead(false); err != nil {
        return err
    }
    pt, err := th.getLastMoveTime()
    if err != nil {
        return err
    }
    th.advanceMoveTime(pt + delay)
    return nil
}

func (th *toolhead) setPosition(newPos []float64, homingAxes string) error {
    if err := th.flushStepGeneration(); err != nil {
        return err
    }
    if len(newPos) < 3 {
        return fmt.Errorf("setPosition requires xyz")
    }
    th.trapq.SetPosition(th.printTime, newPos[0], newPos[1], newPos[2])
    for i := 0; i < len(th.commandedPos) && i < len(newPos); i++ {
        th.commandedPos[i] = newPos[i]
    }
    if th.kin != nil {
        th.kin.setPosition(newPos, homingAxes)
    }
    return nil
}

func (th *toolhead) move(newPos []float64, speed float64) error {
    if th.err != nil {
        return th.err
    }
    mv := newMove(th, th.commandedPos, newPos, speed)
    if mv.moveD == 0.0 {
        return nil
    }
    if mv.isKinematicMove && th.kin != nil {
        if err := th.kin.checkMove(mv); err != nil {
            return err
        }
    }
    for idx, ea := range th.extraAxes {
        p := idx + 3
        if p < len(mv.axesD) && mv.axesD[p] != 0.0 {
            if err := ea.CheckMove(mv, p); err != nil {
                return err
            }
        }
    }
    th.commandedPos = append([]float64{}, mv.endPos...)
    if th.lookahead.addMove(mv) {
        return th.processLookahead(true)
    }
    return nil
}

func (th *toolhead) dripLoadTrapQ(submitMove *move) (float64, float64, error) {
    if submitMove != nil && submitMove.moveD != 0.0 {
        th.commandedPos = append([]float64{}, submitMove.endPos...)
        _ = th.lookahead.addMove(submitMove)
    }
    moves := th.lookahead.flush(false)
    th.calcPrintTime()
    startTime := th.printTime
    endTime := th.printTime
    for _, mv := range moves {
        th.trapq.Append(
            endTime,
            mv.accelT,
            mv.cruiseT,
            mv.decelT,
            mv.startPos[0],
            mv.startPos[1],
            mv.startPos[2],
            mv.axesR[0],
            mv.axesR[1],
            mv.axesR[2],
            mv.startV,
            mv.cruiseV,
            mv.accel,
        )
        endTime = endTime + mv.accelT + mv.cruiseT + mv.decelT
    }
    th.lookahead.reset()
    return startTime, endTime, nil
}

func (th *toolhead) dripMove(newPos []float64, speed float64) error {
    if th.err != nil {
        return th.err
    }
    mv := newMove(th, th.commandedPos, newPos, speed)
    if mv.moveD != 0.0 && mv.isKinematicMove && th.kin != nil {
        if err := th.kin.checkMove(mv); err != nil {
            return err
        }
    }
    if err := th.dwell(th.motion.kinFlushDelay); err != nil {
        return err
    }
    if err := th.processLookahead(false); err != nil {
        return err
    }
    startTime, endTime, err := th.dripLoadTrapQ(mv)
    if err != nil {
        return err
    }
    if err := th.motion.dripUpdateTime(startTime, endTime); err != nil {
        return err
    }
    th.advanceMoveTime(endTime)
    th.motion.wipeTrapQ(th.trapq)
    return nil
}

type move struct {
    toolhead *toolhead

    startPos []float64
    endPos   []float64

    accel            float64
    junctionDeviation float64
    timingCallbacks  []func(nextPrintTime float64)

    axesD []float64
    axesR []float64
    moveD float64

    minMoveT        float64
    isKinematicMove bool

    maxStartV2     float64
    maxCruiseV2    float64
    deltaV2        float64
    nextJunctionV2 float64

    maxMcrStartV2 float64
    mcrDeltaV2    float64

    startV  float64
    cruiseV float64
    endV    float64

    accelT  float64
    cruiseT float64
    decelT  float64
}

func newMove(th *toolhead, startPos []float64, endPos []float64, speed float64) *move {
    sp := append([]float64{}, startPos...)
    ep := append([]float64{}, endPos...)
    mv := &move{
        toolhead:          th,
        startPos:          sp,
        endPos:            ep,
        accel:             th.maxAccel,
        junctionDeviation: th.junctionDeviation,
        timingCallbacks:   nil,
        nextJunctionV2:    999999999.9,
        isKinematicMove:   true,
    }
    velocity := speed
    if velocity > th.maxVelocity {
        velocity = th.maxVelocity
    }

    mv.axesD = make([]float64, len(ep))
    for i := 0; i < len(ep) && i < len(sp); i++ {
        mv.axesD[i] = ep[i] - sp[i]
    }
    moveD := math.Sqrt(mv.axesD[0]*mv.axesD[0] + mv.axesD[1]*mv.axesD[1] + mv.axesD[2]*mv.axesD[2])
    mv.moveD = moveD
    invMoveD := 0.0
    if moveD < 0.000000001 {
        mv.endPos = append([]float64{sp[0], sp[1], sp[2]}, ep[3:]...)
        mv.axesD[0] = 0.0
        mv.axesD[1] = 0.0
        mv.axesD[2] = 0.0
        maxAbs := 0.0
        for i := 3; i < len(mv.axesD); i++ {
            a := math.Abs(mv.axesD[i])
            if a > maxAbs {
                maxAbs = a
            }
        }
        mv.moveD = maxAbs
        if mv.moveD != 0.0 {
            invMoveD = 1.0 / mv.moveD
        }
        mv.accel = 99999999.9
        velocity = speed
        mv.isKinematicMove = false
    } else {
        invMoveD = 1.0 / mv.moveD
    }
    mv.axesR = make([]float64, len(mv.axesD))
    for i := range mv.axesD {
        mv.axesR[i] = mv.axesD[i] * invMoveD
    }
    mv.minMoveT = 0.0
    if velocity != 0.0 {
        mv.minMoveT = mv.moveD / velocity
    }
    mv.maxStartV2 = 0.0
    mv.maxCruiseV2 = velocity * velocity
    mv.deltaV2 = 2.0 * mv.moveD * mv.accel
    mv.maxMcrStartV2 = 0.0
    mv.mcrDeltaV2 = 2.0 * mv.moveD * th.mcrPseudoAccel
    return mv
}

func (mv *move) limitSpeed(speed float64, accel float64) {
    speed2 := speed * speed
    if speed2 < mv.maxCruiseV2 {
        mv.maxCruiseV2 = speed2
        mv.minMoveT = mv.moveD / speed
    }
    if accel < mv.accel {
        mv.accel = accel
    }
    mv.deltaV2 = 2.0 * mv.moveD * mv.accel
    if mv.deltaV2 < mv.mcrDeltaV2 {
        mv.mcrDeltaV2 = mv.deltaV2
    }
}

func (mv *move) calcJunction(prev *move) {
    if prev == nil {
        return
    }
    if !mv.isKinematicMove || !prev.isKinematicMove {
        return
    }
    eaV2 := make([]float64, 0, len(mv.toolhead.extraAxes))
    for idx, ea := range mv.toolhead.extraAxes {
        eaV2 = append(eaV2, ea.CalcJunction(prev, mv, idx+3))
    }
    maxStartV2 := mv.maxCruiseV2
    if prev.maxCruiseV2 < maxStartV2 {
        maxStartV2 = prev.maxCruiseV2
    }
    if prev.nextJunctionV2 < maxStartV2 {
        maxStartV2 = prev.nextJunctionV2
    }
    if prev.maxStartV2+prev.deltaV2 < maxStartV2 {
        maxStartV2 = prev.maxStartV2 + prev.deltaV2
    }
    for _, v2 := range eaV2 {
        if v2 < maxStartV2 {
            maxStartV2 = v2
        }
    }

    axesR := mv.axesR
    prevAxesR := prev.axesR
    junctionCosTheta := -(axesR[0]*prevAxesR[0] + axesR[1]*prevAxesR[1] + axesR[2]*prevAxesR[2])
    sinThetaD2 := math.Sqrt(math.Max(0.5*(1.0-junctionCosTheta), 0.0))
    cosThetaD2 := math.Sqrt(math.Max(0.5*(1.0+junctionCosTheta), 0.0))
    oneMinusSinThetaD2 := 1.0 - sinThetaD2
    if oneMinusSinThetaD2 > 0.0 && cosThetaD2 > 0.0 {
        rJD := sinThetaD2 / oneMinusSinThetaD2
        moveJDv2 := rJD * mv.junctionDeviation * mv.accel
        pmoveJDv2 := rJD * prev.junctionDeviation * prev.accel
        quarterTanThetaD2 := 0.25 * sinThetaD2 / cosThetaD2
        moveCentripetalV2 := mv.deltaV2 * quarterTanThetaD2
        pmoveCentripetalV2 := prev.deltaV2 * quarterTanThetaD2
        maxStartV2 = math.Min(maxStartV2, moveJDv2)
        maxStartV2 = math.Min(maxStartV2, pmoveJDv2)
        maxStartV2 = math.Min(maxStartV2, moveCentripetalV2)
        maxStartV2 = math.Min(maxStartV2, pmoveCentripetalV2)
    }
    mv.maxStartV2 = maxStartV2
    mv.maxMcrStartV2 = math.Min(maxStartV2, prev.maxMcrStartV2+prev.mcrDeltaV2)
}

func (mv *move) setJunction(startV2 float64, cruiseV2 float64, endV2 float64) {
    halfInvAccel := 0.5 / mv.accel
    accelD := (cruiseV2 - startV2) * halfInvAccel
    decelD := (cruiseV2 - endV2) * halfInvAccel
    cruiseD := mv.moveD - accelD - decelD
    mv.startV = math.Sqrt(startV2)
    mv.cruiseV = math.Sqrt(cruiseV2)
    mv.endV = math.Sqrt(endV2)
    mv.accelT = accelD / ((mv.startV + mv.cruiseV) * 0.5)
    mv.cruiseT = cruiseD / mv.cruiseV
    mv.decelT = decelD / ((mv.endV + mv.cruiseV) * 0.5)
}

type lookAheadQueue struct {
    queue         []*move
    junctionFlush float64
}

func (q *lookAheadQueue) reset() {
    q.queue = q.queue[:0]
    q.junctionFlush = lookaheadFlushSec
}

func (q *lookAheadQueue) setFlushTime(flushTime float64) {
    q.junctionFlush = flushTime
}

func (q *lookAheadQueue) addMove(mv *move) bool {
    q.queue = append(q.queue, mv)
    if len(q.queue) == 1 {
        return false
    }
    mv.calcJunction(q.queue[len(q.queue)-2])
    q.junctionFlush -= mv.minMoveT
    return q.junctionFlush <= 0.0
}

type junctionInfo struct {
    mv          *move
    startV2     float64
    cruiseV2    *float64
    nextStartV2 float64
}

func (q *lookAheadQueue) flush(lazy bool) []*move {
    q.junctionFlush = lookaheadFlushSec
    updateFlushCount := lazy
    queue := q.queue
    flushCount := len(queue)
    if flushCount == 0 {
        return nil
    }
    junction := make([]junctionInfo, flushCount)
    nextStartV2 := 0.0
    nextMcrStartV2 := 0.0
    peakCruiseV2 := 0.0
    pendingCV2Assign := 0
    for i := flushCount - 1; i >= 0; i-- {
        mv := queue[i]
        reachableStartV2 := nextStartV2 + mv.deltaV2
        startV2 := math.Min(mv.maxStartV2, reachableStartV2)
        var cruiseV2 *float64
        pendingCV2Assign++
        reachMcrStartV2 := nextMcrStartV2 + mv.mcrDeltaV2
        mcrStartV2 := math.Min(mv.maxMcrStartV2, reachMcrStartV2)
        if mcrStartV2 < reachMcrStartV2 {
            if (mcrStartV2+mv.mcrDeltaV2 > nextMcrStartV2) || pendingCV2Assign > 1 {
                if updateFlushCount && peakCruiseV2 != 0.0 {
                    flushCount = i + pendingCV2Assign
                    updateFlushCount = false
                }
                peakCruiseV2 = (mcrStartV2 + reachMcrStartV2) * 0.5
            }
            cv2 := math.Min((startV2+reachableStartV2)*0.5, mv.maxCruiseV2)
            cv2 = math.Min(cv2, peakCruiseV2)
            cruiseV2 = &cv2
            pendingCV2Assign = 0
        }
        junction[i] = junctionInfo{mv: mv, startV2: startV2, cruiseV2: cruiseV2, nextStartV2: nextStartV2}
        nextStartV2 = startV2
        nextMcrStartV2 = mcrStartV2
    }
    if updateFlushCount || flushCount == 0 {
        return nil
    }
    prevCruiseV2 := 0.0
    for i := 0; i < flushCount; i++ {
        ji := junction[i]
        mv := ji.mv
        cruiseV2 := ji.cruiseV2
        if cruiseV2 == nil {
            cv2 := math.Min(prevCruiseV2, ji.startV2)
            cruiseV2 = &cv2
        }
        startV2 := math.Min(ji.startV2, *cruiseV2)
        endV2 := math.Min(ji.nextStartV2, *cruiseV2)
        mv.setJunction(startV2, *cruiseV2, endV2)
        prevCruiseV2 = *cruiseV2
    }
    res := append([]*move{}, queue[:flushCount]...)
    q.queue = append([]*move{}, queue[flushCount:]...)
    return res
}

type stepper struct {
    axis      byte
    oid       uint32
    stepDist  float64
    invertDir bool

    motion *motionQueuing
    sq     *chelper.SerialQueue

    se        *chelper.SyncEmitter
    stepqueue *chelper.Stepcompress
    sk        *chelper.StepperKinematics
    trapq     *chelper.TrapQ

    activeCallbacks []func(printTime float64)
    activeCBID      int
}

func newStepper(
    motion *motionQueuing,
    sq *chelper.SerialQueue,
    seName string,
    axis byte,
    oid uint32,
    stepDist float64,
    invertDir bool,
    queueStepTag int32,
    setDirTag int32,
    mcuFreq float64,
) (*stepper, error) {
    se, err := motion.allocSyncEmitter(seName)
    if err != nil {
        return nil, err
    }
    sc := se.GetStepcompress()
    if sc == nil {
        return nil, fmt.Errorf("missing stepcompress for %s", seName)
    }
    maxErr := uint32(int64(maxStepcompressErrorSec * mcuFreq))
    if err := sc.Fill(oid, maxErr, queueStepTag, setDirTag); err != nil {
        return nil, err
    }
    if err := sc.SetInvertSdir(invertDir); err != nil {
        return nil, err
    }
    var sk *chelper.StepperKinematics
    if axis == 'e' {
        sk, err = chelper.NewExtruderStepperKinematics()
    } else {
        sk, err = chelper.NewCartesianStepperKinematics(axis)
    }
    if err != nil {
        return nil, err
    }
    if err := se.SetStepperKinematics(sk); err != nil {
        sk.Free()
        return nil, err
    }
    st := &stepper{
        axis:      axis,
        oid:       oid,
        stepDist:  stepDist,
        invertDir: invertDir,
        motion:    motion,
        sq:        sq,
        se:        se,
        stepqueue: sc,
        sk:        sk,
    }
    return st, nil
}

func (s *stepper) free() {
    if s == nil {
        return
    }
    if s.sk != nil {
        s.sk.Free()
        s.sk = nil
    }
}

func (s *stepper) setTrapQ(tq *chelper.TrapQ) *chelper.TrapQ {
    prev := s.trapq
    s.trapq = tq
    if s.sk != nil {
        s.sk.SetTrapQ(tq, s.stepDist)
    }
    return prev
}

func (s *stepper) setPosition(pos float64) {
    if s.sk != nil {
        switch s.axis {
        case 'x':
            s.sk.SetPosition(pos, 0.0, 0.0)
        case 'e':
            s.sk.SetPosition(pos, 0.0, 0.0)
        case 'y':
            s.sk.SetPosition(0.0, pos, 0.0)
        case 'z':
            s.sk.SetPosition(0.0, 0.0, pos)
        default:
            s.sk.SetPosition(pos, 0.0, 0.0)
        }
    }
}

func (s *stepper) addActiveCallback(cb func(printTime float64)) {
    s.activeCallbacks = append(s.activeCallbacks, cb)
    if len(s.activeCallbacks) == 1 {
        s.activeCBID = s.motion.registerFlushCallback(s.checkActive, false)
    }
}

func (s *stepper) checkActive(_ float64, maxStepGenTime float64) {
    if s.sk == nil {
        return
    }
    pt := s.sk.CheckActive(maxStepGenTime + 1e-9)
    if pt == 0.0 {
        return
    }
    s.motion.unregisterFlushCallback(s.activeCBID)
    s.activeCBID = 0
    cbs := s.activeCallbacks
    s.activeCallbacks = nil
    for _, cb := range cbs {
        cb(pt)
    }
}

type digitalOut struct {
    oid     uint32
    invert  bool
    sq      *chelper.SerialQueue
    cq      *chelper.CommandQueue
    formats map[string]*protocol.MessageFormat
    mcuFreq float64
    lastClk uint64
}

func newDigitalOut(oid uint32, invert bool, sq *chelper.SerialQueue, cq *chelper.CommandQueue, formats map[string]*protocol.MessageFormat, mcuFreq float64) *digitalOut {
    return &digitalOut{oid: oid, invert: invert, sq: sq, cq: cq, formats: formats, mcuFreq: mcuFreq}
}

func (d *digitalOut) setDigital(printTime float64, value bool) error {
    clk := uint64(int64(printTime * d.mcuFreq))
    on := 0
    if value {
        on = 1
    }
    if d.invert {
        on ^= 1
    }
    line := fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", d.oid, clk, on)
    b, err := protocol.EncodeCommand(d.formats, line)
    if err != nil {
        return err
    }
    if err := d.sq.Send(d.cq, b, d.lastClk, clk); err != nil {
        return err
    }
    d.lastClk = clk
    return nil
}

type stepperEnablePin struct {
    out         *digitalOut
    enableCount int
}

func (p *stepperEnablePin) setEnable(printTime float64) error {
    if p.enableCount == 0 {
        if err := p.out.setDigital(printTime, true); err != nil {
            return err
        }
    }
    p.enableCount++
    return nil
}

func (p *stepperEnablePin) setDisable(printTime float64) error {
    p.enableCount--
    if p.enableCount == 0 {
        if err := p.out.setDigital(printTime, false); err != nil {
            return err
        }
    }
    return nil
}

type enableTracking struct {
    stepper   *stepper
    enablePin *stepperEnablePin
    isEnabled bool
}

func newEnableTracking(stepper *stepper, pin *stepperEnablePin) *enableTracking {
    et := &enableTracking{stepper: stepper, enablePin: pin}
    stepper.addActiveCallback(et.motorEnable)
    return et
}

func (et *enableTracking) motorEnable(printTime float64) {
    if et.isEnabled {
        return
    }
    _ = et.enablePin.setEnable(printTime)
    et.isEnabled = true
}

func (et *enableTracking) motorDisable(printTime float64) {
    if !et.isEnabled {
        return
    }
    _ = et.enablePin.setDisable(printTime)
    et.isEnabled = false
    et.stepper.addActiveCallback(et.motorEnable)
}

type stepperEnable struct {
    toolhead *toolhead
    lines    map[string]*enableTracking
}

func newStepperEnable(th *toolhead) *stepperEnable {
    return &stepperEnable{toolhead: th, lines: map[string]*enableTracking{}}
}

func (se *stepperEnable) registerStepper(name string, stepper *stepper, pin *stepperEnablePin) {
    se.lines[name] = newEnableTracking(stepper, pin)
}

func (se *stepperEnable) motorOffOrdered(names []string) error {
    if err := se.toolhead.flushStepGeneration(); err != nil {
        return err
    }
    if err := se.toolhead.dwell(disableStallTimeSec); err != nil {
        return err
    }
    pt, err := se.toolhead.getLastMoveTime()
    if err != nil {
        return err
    }
    for _, name := range names {
        et := se.lines[name]
        if et != nil && et.isEnabled {
            et.motorDisable(pt)
        }
    }
    if err := se.toolhead.dwell(disableStallTimeSec); err != nil {
        return err
    }
    if se.toolhead.kin != nil {
        se.toolhead.kin.clearHomingState("xyz")
    }
    return nil
}

func (se *stepperEnable) motorOff() error {
    names := make([]string, 0, len(se.lines))
    for name := range se.lines {
        names = append(names, name)
    }
    for i := 1; i < len(names); i++ {
        j := i
        for j > 0 && names[j] < names[j-1] {
            names[j], names[j-1] = names[j-1], names[j]
            j--
        }
    }
    return se.motorOffOrdered(names)
}

type trsync struct {
    oid    uint32
    rt     *runtime
    mcuFreq float64
}

func (t *trsync) start(printTime float64) error {
    // Match Klippy: trsync_start is queued with reqclock=clock (start clock),
    // and the report_clock field is derived from that clock.
    clock := uint64(int64(printTime * t.mcuFreq))
    reportClock := clock
    reportTicks := uint64(int64(t.mcuFreq * (trsyncTimeoutSec * trsyncReportRatio)))
    line := fmt.Sprintf("trsync_start oid=%d report_clock=%d report_ticks=%d expire_reason=%d",
        t.oid, reportClock, reportTicks, trsyncExpireReason)
    cq := t.rt.cqTrigger
    if cq == nil {
        cq = t.rt.cqMain
    }
    return t.rt.sendLine(line, cq, 0, clock)
}

func (t *trsync) setTimeout(printTime float64) error {
    // Match Klippy: expire clock is computed from the start clock, but the
    // command is queued with reqclock=start clock.
    clock := uint64(int64(printTime * t.mcuFreq))
    expireTicks := uint64(int64(trsyncTimeoutSec * t.mcuFreq))
    expireClock := clock + expireTicks
    line := fmt.Sprintf("trsync_set_timeout oid=%d clock=%d", t.oid, expireClock)
    cq := t.rt.cqTrigger
    if cq == nil {
        cq = t.rt.cqMain
    }
    return t.rt.sendLine(line, cq, 0, clock)
}

type endstop struct {
    oid       uint32
    stepperOID uint32
    tr        *trsync
    rt        *runtime
    mcuFreq   float64
}

func (e *endstop) homeStart(printTime float64, restTime float64) error {
    clock := uint64(int64(printTime * e.mcuFreq))
    cq := e.rt.cqTrigger
    if cq == nil {
        cq = e.rt.cqMain
    }
    if err := e.tr.start(printTime); err != nil {
        return err
    }
    // In Klippy, this is sent without an explicit reqclock, but it is queued
    // immediately after trsync_start and must remain adjacent in fileoutput.
    if err := e.rt.sendLine(fmt.Sprintf("stepper_stop_on_trigger oid=%d trsync_oid=%d", e.stepperOID, e.tr.oid), cq, 0, clock); err != nil {
        return err
    }
    if err := e.tr.setTimeout(printTime); err != nil {
        return err
    }
    sampleTicks := uint64(int64(endstopSampleTimeSec * e.mcuFreq))
    restClk := uint64(int64((printTime + restTime) * e.mcuFreq))
    restTicks := restClk - clock
    line := fmt.Sprintf("endstop_home oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d pin_value=1 trsync_oid=%d trigger_reason=1",
        e.oid, clock, sampleTicks, endstopSampleCount, restTicks, e.tr.oid)
    return e.rt.sendLine(line, cq, 0, clock)
}

func (e *endstop) homeStop() error {
    line := fmt.Sprintf("endstop_home oid=%d clock=0 sample_ticks=0 sample_count=0 rest_ticks=0 pin_value=0 trsync_oid=0 trigger_reason=0", e.oid)
    cq := e.rt.cqTrigger
    if cq == nil {
        cq = e.rt.cqMain
    }
    return e.rt.sendLine(line, cq, 0, 0)
}

type runtime struct {
    dict        *protocol.Dictionary
    formats     map[string]*protocol.MessageFormat
    mcuFreq     float64
    queueStepID int32
    setDirID    int32

    sq     *chelper.SerialQueue
    cqMain *chelper.CommandQueue
    cqTrigger *chelper.CommandQueue

    motion       *motionQueuing
    toolhead     *toolhead
    stepperEnable *stepperEnable

    rails   [3]stepperCfg
    steppers [3]*stepper
    extruderCfg extruderStepperCfg
    stepperE    *stepper
    extruder    *extruderAxis
    endstops [3]*endstop
    cqEnPins []*chelper.CommandQueue

    gm *gcodeMove
    motorOffOrder []string

    trace io.Writer

    rawPath string
    rawFile *os.File
    closed  bool
}

func (r *runtime) setTrace(w io.Writer) {
    r.trace = w
    if r.motion != nil {
        r.motion.setTrace(w)
    }
}

func (r *runtime) tracef(format string, args ...any) {
    if r == nil || r.trace == nil {
        return
    }
    fmt.Fprintf(r.trace, format, args...)
}

func (r *runtime) traceMotion(prefix string) {
    if r == nil || r.trace == nil || r.motion == nil {
        return
    }
    mq := r.motion
    fmt.Fprintf(r.trace, "%s need_flush=%.9f need_sg=%.9f last_flush=%.9f last_sg=%.9f kin_delay=%.9f\n",
        prefix, mq.needFlushTime, mq.needStepGenTime, mq.lastFlushTime, mq.lastStepGenTime, mq.kinFlushDelay)
}

func newRuntime(cfgPath string, dict *protocol.Dictionary, cfg *config) (*runtime, error) {
    mcuFreq, err := dictConfigFloat(dict, "CLOCK_FREQ")
    if err != nil {
        return nil, err
    }
    formats, err := dict.BuildCommandFormats()
    if err != nil {
        return nil, err
    }
    queueStepID, err := dictCommandTag(dict, "queue_step oid=%c interval=%u count=%hu add=%hi")
    if err != nil {
        return nil, err
    }
    setDirID, err := dictCommandTag(dict, "set_next_step_dir oid=%c dir=%c")
    if err != nil {
        return nil, err
    }

    printerSec, ok := cfg.section("printer")
    if !ok {
        return nil, fmt.Errorf("missing [printer] section")
    }
    maxVelocity, err := parseFloat(printerSec, "max_velocity", nil)
    if err != nil {
        return nil, err
    }
    maxAccel, err := parseFloat(printerSec, "max_accel", nil)
    if err != nil {
        return nil, err
    }
    maxZVelocity, err := parseFloat(printerSec, "max_z_velocity", &maxVelocity)
    if err != nil {
        return nil, err
    }
    maxZAccel, err := parseFloat(printerSec, "max_z_accel", &maxAccel)
    if err != nil {
        return nil, err
    }

    sx, err := readStepper(cfg, 'x')
    if err != nil {
        return nil, err
    }
    sy, err := readStepper(cfg, 'y')
    if err != nil {
        return nil, err
    }
    sz, err := readStepper(cfg, 'z')
    if err != nil {
        return nil, err
    }
    rails := [3]stepperCfg{sx, sy, sz}
    extruderCfg, err := readExtruderStepper(cfg)
    if err != nil {
        return nil, err
    }
    mmPerArcSegment, err := readGcodeArcsResolution(cfg)
    if err != nil {
        return nil, err
    }

    tmpDir := filepath.Join("out", "go_migration_tmp")
    if err := os.MkdirAll(tmpDir, 0o755); err != nil {
        return nil, err
    }
    rawPath := filepath.Join(tmpDir, "serial-"+randSuffix()+".bin")
    f, err := os.OpenFile(rawPath, os.O_CREATE|os.O_TRUNC|os.O_RDWR, 0o644)
    if err != nil {
        return nil, err
    }

    sq, err := chelper.NewSerialQueue(int(f.Fd()), 'f', 0, "serialq mcu")
    if err != nil {
        f.Close()
        return nil, err
    }
    sq.SetClockEst(1e12, chelper.Monotonic(), 0, 0)

    cqMain := chelper.NewCommandQueue()
    if cqMain == nil {
        sq.Free()
        f.Close()
        return nil, fmt.Errorf("alloc main command queue failed")
    }

    reservedMoveSlots := 2
    motion, err := newMotionQueuing(sq, mcuFreq, reservedMoveSlots)
    if err != nil {
        cqMain.Free()
        sq.Free()
        f.Close()
        return nil, err
    }
    th, err := newToolhead(mcuFreq, motion, maxVelocity, maxAccel)
    if err != nil {
        motion.free()
        cqMain.Free()
        sq.Free()
        f.Close()
        return nil, err
    }

    se := newStepperEnable(th)

    stepDistX := rails[0].rotationDistance / float64(rails[0].fullSteps*rails[0].microsteps)
    stepDistY := rails[1].rotationDistance / float64(rails[1].fullSteps*rails[1].microsteps)
    stepDistZ := rails[2].rotationDistance / float64(rails[2].fullSteps*rails[2].microsteps)
    stepDistE := extruderCfg.rotationDistance / float64(extruderCfg.fullSteps*extruderCfg.microsteps)

    stX, err := newStepper(motion, sq, "stepper_x", 'x', 2, stepDistX, rails[0].dirPin.invert, queueStepID, setDirID, mcuFreq)
    if err != nil {
        motion.free()
        cqMain.Free()
        sq.Free()
        f.Close()
        return nil, err
    }
    stY, err := newStepper(motion, sq, "stepper_y", 'y', 5, stepDistY, rails[1].dirPin.invert, queueStepID, setDirID, mcuFreq)
    if err != nil {
        stX.free()
        motion.free()
        cqMain.Free()
        sq.Free()
        f.Close()
        return nil, err
    }
    stZ, err := newStepper(motion, sq, "stepper_z", 'z', 8, stepDistZ, rails[2].dirPin.invert, queueStepID, setDirID, mcuFreq)
    if err != nil {
        stY.free()
        stX.free()
        motion.free()
        cqMain.Free()
        sq.Free()
        f.Close()
        return nil, err
    }
    steppers := [3]*stepper{stX, stY, stZ}
    stX.setTrapQ(th.trapq)
    stY.setTrapQ(th.trapq)
    stZ.setTrapQ(th.trapq)

    kin := newCartesianKinematics(rails, steppers, maxZVelocity, maxZAccel)
    th.kin = kin

    tqExtruder, err := motion.allocTrapQ()
    if err != nil {
        stZ.free()
        stY.free()
        stX.free()
        motion.free()
        cqMain.Free()
        sq.Free()
        f.Close()
        return nil, err
    }
    exAxis := newExtruderAxis("extruder", tqExtruder)
    th.extraAxes = []extraAxis{exAxis}

    stE, err := newStepper(motion, sq, "extruder", 'e', 9, stepDistE, extruderCfg.dirPin.invert, queueStepID, setDirID, mcuFreq)
    if err != nil {
        stZ.free()
        stY.free()
        stX.free()
        motion.free()
        cqMain.Free()
        sq.Free()
        f.Close()
        return nil, err
    }
    stE.setTrapQ(tqExtruder)
    stE.setPosition(0.0)

    cqEnX := chelper.NewCommandQueue()
    cqEnY := chelper.NewCommandQueue()
    cqEnZ := chelper.NewCommandQueue()
    cqEnE := chelper.NewCommandQueue()
    if cqEnX == nil || cqEnY == nil || cqEnZ == nil || cqEnE == nil {
        if cqEnX != nil {
            cqEnX.Free()
        }
        if cqEnY != nil {
            cqEnY.Free()
        }
        if cqEnZ != nil {
            cqEnZ.Free()
        }
        if cqEnE != nil {
            cqEnE.Free()
        }
        stE.free()
        stZ.free()
        stY.free()
        stX.free()
        motion.free()
        cqMain.Free()
        sq.Free()
        f.Close()
        return nil, fmt.Errorf("alloc enable command queue failed")
    }
    enX := &stepperEnablePin{out: newDigitalOut(12, rails[0].enablePin.invert, sq, cqEnX, formats, mcuFreq)}
    enY := &stepperEnablePin{out: newDigitalOut(13, rails[1].enablePin.invert, sq, cqEnY, formats, mcuFreq)}
    enZ := &stepperEnablePin{out: newDigitalOut(14, rails[2].enablePin.invert, sq, cqEnZ, formats, mcuFreq)}
    enE := &stepperEnablePin{out: newDigitalOut(17, extruderCfg.enablePin.invert, sq, cqEnE, formats, mcuFreq)}
    se.registerStepper("stepper_x", stX, enX)
    se.registerStepper("stepper_y", stY, enY)
    se.registerStepper("stepper_z", stZ, enZ)
    se.registerStepper("extruder", stE, enE)

    cqTrigger := chelper.NewCommandQueue()
    if cqTrigger == nil {
        cqEnE.Free()
        cqEnZ.Free()
        cqEnY.Free()
        cqEnX.Free()
        stE.free()
        stZ.free()
        stY.free()
        stX.free()
        motion.free()
        cqMain.Free()
        sq.Free()
        f.Close()
        return nil, fmt.Errorf("alloc trigger command queue failed")
    }

    trX := &trsync{oid: 1, rt: nil, mcuFreq: mcuFreq}
    trY := &trsync{oid: 4, rt: nil, mcuFreq: mcuFreq}
    trZ := &trsync{oid: 7, rt: nil, mcuFreq: mcuFreq}
    rt := &runtime{
        dict:          dict,
        formats:       formats,
        mcuFreq:       mcuFreq,
        queueStepID:   queueStepID,
        setDirID:      setDirID,
        sq:            sq,
        cqMain:        cqMain,
        cqTrigger:     cqTrigger,
        motion:        motion,
        toolhead:      th,
        stepperEnable: se,
        rails:         rails,
        steppers:      steppers,
        extruderCfg:   extruderCfg,
        stepperE:      stE,
        extruder:      exAxis,
        cqEnPins:      []*chelper.CommandQueue{cqEnX, cqEnY, cqEnZ, cqEnE},
        motorOffOrder: []string{"stepper_x", "stepper_y", "stepper_z", "extruder"},
        rawPath:       rawPath,
        rawFile:       f,
    }
    rt.gm = newGCodeMove(th, mmPerArcSegment)
    trX.rt = rt
    trY.rt = rt
    trZ.rt = rt

    rt.endstops[0] = &endstop{oid: 0, stepperOID: 2, tr: trX, rt: rt, mcuFreq: mcuFreq}
    rt.endstops[1] = &endstop{oid: 3, stepperOID: 5, tr: trY, rt: rt, mcuFreq: mcuFreq}
    rt.endstops[2] = &endstop{oid: 6, stepperOID: 8, tr: trZ, rt: rt, mcuFreq: mcuFreq}

    kinFlushDelay := sdsCheckTimeSec
    for i := 0; i < 3; i++ {
        st := steppers[i]
        if st == nil || st.sk == nil {
            continue
        }
        pre := st.sk.GenStepsPreActive()
        post := st.sk.GenStepsPostActive()
        if pre > kinFlushDelay {
            kinFlushDelay = pre
        }
        if post > kinFlushDelay {
            kinFlushDelay = post
        }
    }
    if stE != nil && stE.sk != nil {
        pre := stE.sk.GenStepsPreActive()
        post := stE.sk.GenStepsPostActive()
        if pre > kinFlushDelay {
            kinFlushDelay = pre
        }
        if post > kinFlushDelay {
            kinFlushDelay = post
        }
    }
    rt.motion.kinFlushDelay = kinFlushDelay

    _ = cfgPath
    return rt, nil
}

func (r *runtime) free() {
    if r == nil || r.closed {
        return
    }
    _ = r.cleanup(false)
}

func (r *runtime) cleanup(removeFile bool) error {
    if r.closed {
        return nil
    }
    r.closed = true

    if r.motion != nil {
        _ = r.motion.flushAllSteps()
    }

    for _, cq := range r.cqEnPins {
        if cq != nil {
            cq.Free()
        }
    }
    r.cqEnPins = nil

    if r.cqMain != nil {
        r.cqMain.Free()
        r.cqMain = nil
    }
    if r.cqTrigger != nil {
        r.cqTrigger.Free()
        r.cqTrigger = nil
    }
    if r.motion != nil {
        r.motion.free()
        r.motion = nil
    }
    for i := range r.steppers {
        if r.steppers[i] != nil {
            r.steppers[i].free()
            r.steppers[i] = nil
        }
    }
    if r.stepperE != nil {
        r.stepperE.free()
        r.stepperE = nil
    }
    if r.sq != nil {
        r.sq.Free()
        r.sq = nil
    }
    if r.rawFile != nil {
        _ = r.rawFile.Close()
        r.rawFile = nil
    }
    if removeFile && r.rawPath != "" {
        _ = os.Remove(r.rawPath)
    }
    return nil
}

func (r *runtime) closeAndRead() ([]byte, error) {
    rawPath := r.rawPath
    for i := 0; i < 2000; i++ {
        st, err := r.sq.GetStats()
        if err != nil {
            break
        }
        if st.ReadyBytes == 0 && st.UpcomingBytes == 0 {
            break
        }
        time.Sleep(1 * time.Millisecond)
    }
    _ = r.motion.flushAllSteps()
    for i := 0; i < 2000; i++ {
        st, err := r.sq.GetStats()
        if err != nil {
            break
        }
        if st.ReadyBytes == 0 && st.UpcomingBytes == 0 {
            break
        }
        time.Sleep(1 * time.Millisecond)
    }
    _ = r.cleanup(false)

    b, err := os.ReadFile(rawPath)
    _ = os.Remove(rawPath)
    if err != nil {
        return nil, err
    }
    return b, nil
}

func (r *runtime) sendLine(line string, cq *chelper.CommandQueue, minClock uint64, reqClock uint64) error {
    if r.trace != nil {
        cqName := "unknown"
        if cq == r.cqMain {
            cqName = "main"
        } else if cq == r.cqTrigger {
            cqName = "trigger"
        } else {
            for idx, q := range r.cqEnPins {
                if cq == q {
                    cqName = fmt.Sprintf("en%d", idx)
                    break
                }
            }
        }
        r.tracef("SEND cq=%s min=%d req=%d line=%s\n", cqName, minClock, reqClock, strings.TrimSpace(line))
    }
    b, err := protocol.EncodeCommand(r.formats, line)
    if err != nil {
        return err
    }
    return r.sq.Send(cq, b, minClock, reqClock)
}

func (r *runtime) exec(cmd *gcodeCommand) error {
    if cmd == nil {
        return nil
    }
    switch cmd.Name {
    case "G28":
        if err := r.homeAll(); err != nil {
            return err
        }
        r.gm.resetFromToolhead()
    case "G90":
        r.gm.cmdG90()
    case "G91":
        r.gm.cmdG91()
    case "G17":
        r.gm.cmdG17()
    case "G18":
        r.gm.cmdG18()
    case "G19":
        r.gm.cmdG19()
    case "G2":
        if err := r.gm.cmdG2G3(cmd.Args, true); err != nil {
            return err
        }
    case "G3":
        if err := r.gm.cmdG2G3(cmd.Args, false); err != nil {
            return err
        }
    case "G0", "G1":
        if err := r.gm.cmdG1(cmd.Args); err != nil {
            return err
        }
    default:
        return fmt.Errorf("unsupported gcode %q (H4)", cmd.Name)
    }

    // After executing each gcode command, trigger batch flushing
    // This matches Python's _flush_handler_debug behavior
    return r.toolhead.motion.flushPendingBatch()
}

func (r *runtime) onEOF() error {
    if r.stepperEnable == nil {
        return nil
    }
    r.traceMotion("EOF: before motorOff")
    if len(r.motorOffOrder) == 0 {
        return r.stepperEnable.motorOff()
    }
    return r.stepperEnable.motorOffOrdered(r.motorOffOrder)
}

func (r *runtime) homingMove(axis int, movepos []float64, speed float64) error {
    r.tracef("HOMING axis=%d begin speed=%.6f movepos=%v\n", axis, speed, movepos)
    r.traceMotion("HOMING: pre flushStepGeneration")
    if err := r.toolhead.flushStepGeneration(); err != nil {
        return err
    }
    r.traceMotion("HOMING: post flushStepGeneration")
    pt, err := r.toolhead.getLastMoveTime()
    if err != nil {
        return err
    }
    r.tracef("HOMING axis=%d start_pt=%.9f\n", axis, pt)
    if r.trace != nil {
        st := r.steppers[axis]
        if st != nil && st.stepqueue != nil {
            center := uint64(int64(pt * r.mcuFreq))
            half := uint64(int64(0.5 * r.mcuFreq))
            start := uint64(0)
            if center > half {
                start = center - half
            }
            end := center + half
            hs, err := st.stepqueue.ExtractOld(start, end, 64)
            if err != nil {
                r.tracef("HOMING axis=%d stepcompress_extract_old err=%v\n", axis, err)
            } else {
                r.tracef("HOMING axis=%d stepcompress_history window=[%d,%d] entries=%d\n", axis, start, end, len(hs))
                for i := 0; i < len(hs) && i < 10; i++ {
                    p := hs[i]
                    r.tracef("HOMING axis=%d hist[%d] first=%d last=%d count=%d interval=%d add=%d\n",
                        axis, i, p.FirstClock, p.LastClock, p.StepCount, p.Interval, p.Add)
                }
                if len(hs) > 0 {
                    p := hs[len(hs)-1]
                    r.tracef("HOMING axis=%d hist_last first=%d last=%d count=%d interval=%d add=%d\n",
                        axis, p.FirstClock, p.LastClock, p.StepCount, p.Interval, p.Add)
                }
            }
        }
    }
    if axis < 0 || axis >= 3 {
        return fmt.Errorf("bad axis %d", axis)
    }
    startPos := append([]float64{}, r.toolhead.commandedPos...)
    delta := movepos[axis] - startPos[axis]
    moveD := math.Abs(delta)
    if moveD == 0.0 {
        return nil
    }
    stepDist := r.steppers[axis].stepDist
    maxSteps := moveD / stepDist
    if maxSteps <= 0.0 {
        maxSteps = 1.0
    }
    moveT := moveD / speed
    restTime := moveT / maxSteps
    r.tracef("HOMING axis=%d restTime=%.9f moveT=%.9f maxSteps=%.3f\n", axis, restTime, moveT, maxSteps)
    if err := r.endstops[axis].homeStart(pt, restTime); err != nil {
        return err
    }
    r.traceMotion("HOMING: after homeStart")
    if err := r.toolhead.dwell(homingStartDelaySec); err != nil {
        return err
    }
    if err := r.toolhead.dripMove(movepos, speed); err != nil {
        return err
    }
    // Ensure all steps from the homing move are emitted before we stop
    // endstop sampling, to match Klippy's ordering in debugoutput.
    if err := r.motion.flushAllSteps(); err != nil {
        return err
    }
    r.traceMotion("HOMING: before homeStop")
    if r.trace != nil {
        st := r.steppers[axis]
        if st != nil && st.stepqueue != nil {
            // Re-dump a narrow history window after emitting homing steps to
            // help debug ordering diffs around trsync_start.
            center := uint64(int64(pt * r.mcuFreq))
            span := uint64(int64(0.05 * r.mcuFreq))
            start := uint64(0)
            if center > span {
                start = center - span
            }
            end := center + span
            hs, err := st.stepqueue.ExtractOld(start, end, 64)
            if err != nil {
                r.tracef("HOMING axis=%d post-move stepcompress_extract_old err=%v\n", axis, err)
            } else {
                r.tracef("HOMING axis=%d post-move history window=[%d,%d] entries=%d\n", axis, start, end, len(hs))
                for i := 0; i < len(hs) && i < 10; i++ {
                    p := hs[i]
                    r.tracef("HOMING axis=%d post[%d] first=%d last=%d count=%d interval=%d add=%d\n",
                        axis, i, p.FirstClock, p.LastClock, p.StepCount, p.Interval, p.Add)
                }
            }
        }
    }
    return r.endstops[axis].homeStop()
}

func (r *runtime) homeAxis(axis int) error {
    rail := r.rails[axis]
    cur := append([]float64{}, r.toolhead.commandedPos...)
    homepos := append([]float64{}, cur...)
    homepos[axis] = rail.positionEndstop
    forcepos := append([]float64{}, homepos...)
    if rail.homingPositiveDir {
        forcepos[axis] -= 1.5 * (rail.positionEndstop - rail.positionMin)
    } else {
        forcepos[axis] += 1.5 * (rail.positionMax - rail.positionEndstop)
    }
    homingAxes := string([]byte{byte("xyz"[axis])})
    if err := r.toolhead.setPosition(forcepos, homingAxes); err != nil {
        return err
    }
    if err := r.homingMove(axis, homepos, rail.homingSpeed); err != nil {
        return err
    }
    if rail.homingRetractDist == 0.0 {
        if err := r.toolhead.flushStepGeneration(); err != nil {
            return err
        }
        return nil
    }
    axesD := make([]float64, 3)
    for i := 0; i < 3; i++ {
        axesD[i] = homepos[i] - forcepos[i]
    }
    moveD := math.Sqrt(axesD[0]*axesD[0] + axesD[1]*axesD[1] + axesD[2]*axesD[2])
    retractR := 1.0
    if moveD != 0.0 {
        retractR = math.Min(1.0, rail.homingRetractDist/moveD)
    }
    retractpos := append([]float64{}, homepos...)
    for i := 0; i < 3; i++ {
        retractpos[i] = homepos[i] - axesD[i]*retractR
    }
    if err := r.toolhead.move(retractpos, rail.homingRetractSpeed); err != nil {
        return err
    }
    start2pos := append([]float64{}, retractpos...)
    for i := 0; i < 3; i++ {
        start2pos[i] = retractpos[i] - axesD[i]*retractR
    }
    if err := r.toolhead.setPosition(start2pos, ""); err != nil {
        return err
    }
    if err := r.homingMove(axis, homepos, rail.secondHomingSpeed); err != nil {
        return err
    }
    return r.toolhead.flushStepGeneration()
}

func (r *runtime) homeAll() error {
    for axis := 0; axis < 3; axis++ {
        if err := r.homeAxis(axis); err != nil {
            return err
        }
    }
    return nil
}

func randSuffix() string {
    var b [8]byte
    if _, err := rand.Read(b[:]); err != nil {
        return fmt.Sprintf("%d", time.Now().UnixNano())
    }
    return hex.EncodeToString(b[:])
}

func dictConfigFloat(d *protocol.Dictionary, key string) (float64, error) {
    v, ok := d.Config[key]
    if !ok {
        return 0, fmt.Errorf("missing dict config %s", key)
    }
    f, ok := v.(float64)
    if !ok {
        return 0, fmt.Errorf("dict config %s is %T", key, v)
    }
    return f, nil
}

func dictCommandTag(d *protocol.Dictionary, format string) (int32, error) {
    id, ok := d.Commands[format]
    if !ok {
        return 0, fmt.Errorf("missing dict command %q", format)
    }
    return int32(id), nil
}
