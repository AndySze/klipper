package hosth3

import (
    "crypto/rand"
    "encoding/hex"
    "fmt"
    "hash/crc32"
    "math"
    "os"
    "path/filepath"
    "sort"
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

    bgflushSGLowSec   = 0.450
    bgflushSGHighSec  = 0.700
    bgflushExtraSec   = 0.250
    lookaheadFlushSec = 0.150

    buzzDistanceMM     = 1.0
    buzzVelocityMMPerS = buzzDistanceMM / 0.250
)

type motionQueuing struct {
    ssm *chelper.StepperSyncMgr
    ss  *chelper.StepperSync
    mcuFreq float64

    trapqs []*chelper.TrapQ

    callbacks []flushCB
    nextCBID  int

    lastFlushTime   float64
    lastStepGenTime float64
    needFlushTime   float64
    needStepGenTime float64

    kinFlushDelay float64
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
        ssm:          ssm,
        ss:           ss,
        mcuFreq:      mcuFreq,
        kinFlushDelay: sdsCheckTimeSec,
    }, nil
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
    // In Python, steppersync_set_time is called after all emitters are
    // allocated (during mcu connect). Ensure newly created emitters have their
    // stepcompress time base configured.
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
    return nil
}

func (mq *motionQueuing) flushAllSteps() error {
    flushTime := mq.needStepGenTime
    return mq.advanceFlushTime(flushTime, 0.0)
}

func (mq *motionQueuing) wipeTrapQ(tq *chelper.TrapQ) {
    if tq == nil {
        return
    }
    tq.FinalizeMoves(1e30, 0.0)
}

func (mq *motionQueuing) runFlushTimerDebug() error {
    // In Klipper's fileoutput mode, the flush timer ensures that queued steps
    // are emitted promptly. For the golden harness we execute it explicitly at
    // well-defined boundaries (after each test command).
    return mq.advanceFlushTime(mq.needFlushTime+bgflushExtraSec, mq.needStepGenTime)
}

func (mq *motionQueuing) advanceFlushTime(wantFlushTime float64, wantStepGenTime float64) error {
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

    for _, tq := range mq.trapqs {
        tq.FinalizeMoves(trapqFreeTime, clearHistoryTime)
    }
    return nil
}

type kinematicsNone struct{}

func (k kinematicsNone) CheckMove(_ *move) error { return nil }
func (k kinematicsNone) SetPosition(_ []float64) {}
func (k kinematicsNone) ClearHomingState(_ string) {}

type toolhead struct {
    mcuFreq float64
    motion  *motionQueuing

    maxVelocity        float64
    maxAccel           float64
    minCruiseRatio     float64
    squareCornerV      float64
    junctionDeviation  float64
    mcrPseudoAccel     float64

    printTime          float64
    specialState       string

    commandedPos []float64

    lookahead *lookAheadQueue
    kin       kinematicsNone
    trapq     *chelper.TrapQ

    extraAxes []extraAxis
}

func newToolhead(mcuFreq float64, motion *motionQueuing, maxVelocity float64, maxAccel float64) (*toolhead, error) {
    tq, err := motion.allocTrapQ()
    if err != nil {
        return nil, err
    }
    th := &toolhead{
        mcuFreq:         mcuFreq,
        motion:          motion,
        maxVelocity:     maxVelocity,
        maxAccel:        maxAccel,
        minCruiseRatio:  0.5,
        squareCornerV:   5.0,
        specialState:    "NeedPrime",
        commandedPos:    []float64{0, 0, 0, 0},
        lookahead:       &lookAheadQueue{junctionFlush: lookaheadFlushSec},
        trapq:           tq,
    }
    th.calcJunctionDeviation()
    extruder := dummyExtruder{}
    th.extraAxes = []extraAxis{extruder}
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
    // Fileoutput mode: always 0.0.
    return 0.0
}

func (th *toolhead) printTimeToClock(printTime float64) uint64 {
    if printTime <= 0 {
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

func (th *toolhead) handleStepFlush(flushTime float64, stepGenTime float64) {
    if th.specialState != "" {
        return
    }
    if stepGenTime >= th.printTime-th.motion.kinFlushDelay-0.001 {
        th.flushLookahead(true)
    }
    _ = flushTime
}

func (th *toolhead) processLookahead(lazy bool) error {
    moves := th.lookahead.flush(lazy)
    if len(moves) == 0 {
        return nil
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
    return th.motion.noteMovequeueActivity(nextMoveTime, true)
}

func (th *toolhead) flushLookahead(isRunout bool) {
    _ = isRunout
    _ = th.processLookahead(false)
    th.specialState = "NeedPrime"
    th.lookahead.setFlushTime(bufferTimeHigh)
}

func (th *toolhead) flushStepGeneration() error {
    th.flushLookahead(false)
    return th.motion.flushAllSteps()
}

func (th *toolhead) getLastMoveTime() (float64, error) {
    if th.specialState != "" {
        th.flushLookahead(false)
        th.calcPrintTime()
    } else {
        if err := th.processLookahead(false); err != nil {
            return 0.0, err
        }
    }
    return th.printTime, nil
}

func (th *toolhead) dwell(delay float64) error {
    if delay < 0 {
        delay = 0
    }
    th.flushLookahead(false)
    pt, err := th.getLastMoveTime()
    if err != nil {
        return err
    }
    th.advanceMoveTime(pt + delay)
    return nil
}

func (th *toolhead) setPosition(newPos []float64) error {
    if err := th.flushStepGeneration(); err != nil {
        return err
    }
    if len(newPos) < 3 {
        return fmt.Errorf("setPosition requires xyz")
    }
    th.trapq.SetPosition(th.printTime, newPos[0], newPos[1], newPos[2])
    th.commandedPos[0] = newPos[0]
    th.commandedPos[1] = newPos[1]
    th.commandedPos[2] = newPos[2]
    th.kin.SetPosition(newPos)
    return nil
}

func (th *toolhead) move(newPos []float64, speed float64) error {
    mv := newMove(th, th.commandedPos, newPos, speed)
    if mv.moveD == 0.0 {
        return nil
    }
    if mv.isKinematicMove {
        if err := th.kin.CheckMove(mv); err != nil {
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

func (th *toolhead) addExtraAxis(ea extraAxis, axisPos float64) {
    th.flushLookahead(false)
    th.extraAxes = append(th.extraAxes, ea)
    th.commandedPos = append(th.commandedPos, axisPos)
}

func (th *toolhead) removeExtraAxis(ea extraAxis) {
    th.flushLookahead(false)
    idx := -1
    for i := range th.extraAxes {
        if th.extraAxes[i] == ea {
            idx = i
            break
        }
    }
    if idx < 0 {
        return
    }
    th.extraAxes = append(th.extraAxes[:idx], th.extraAxes[idx+1:]...)
    posIdx := idx + 3
    if posIdx >= 0 && posIdx < len(th.commandedPos) {
        th.commandedPos = append(th.commandedPos[:posIdx], th.commandedPos[posIdx+1:]...)
    }
}

func (th *toolhead) getExtraAxes() []extraAxisOrNil {
    out := []extraAxisOrNil{nil, nil, nil}
    for _, ea := range th.extraAxes {
        out = append(out, ea)
    }
    return out
}

type extraAxisOrNil interface{}

type extraAxis interface {
    GetName() string
    GetAxisGcodeID() string
    ProcessMove(printTime float64, mv *move, axisIndex int) error
    CheckMove(mv *move, axisIndex int) error
    CalcJunction(prev *move, mv *move, axisIndex int) float64
}

type dummyExtruder struct{}

func (d dummyExtruder) GetName() string       { return "extruder" }
func (d dummyExtruder) GetAxisGcodeID() string { return "" }
func (d dummyExtruder) ProcessMove(_ float64, _ *move, _ int) error { return nil }
func (d dummyExtruder) CheckMove(_ *move, _ int) error             { return nil }
func (d dummyExtruder) CalcJunction(_ *move, mv *move, _ int) float64 {
    return mv.maxCruiseV2
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

func (q *lookAheadQueue) getLast() *move {
    if len(q.queue) == 0 {
        return nil
    }
    return q.queue[len(q.queue)-1]
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
    sk, err := chelper.NewCartesianStepperKinematics('x')
    if err != nil {
        return nil, err
    }
    if err := se.SetStepperKinematics(sk); err != nil {
        sk.Free()
        return nil, err
    }
    st := &stepper{
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

func (s *stepper) setPosition(x float64) {
    if s.sk != nil {
        s.sk.SetPosition(x, 0.0, 0.0)
    }
}

func (s *stepper) setStepperKinematics(sk *chelper.StepperKinematics) *chelper.StepperKinematics {
    prev := s.sk
    s.sk = sk
    _ = s.se.SetStepperKinematics(sk)
    if s.trapq != nil {
        s.sk.SetTrapQ(s.trapq, s.stepDist)
    }
    return prev
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
    pt := s.sk.CheckActive(maxStepGenTime)
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
    isDedicated bool
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

func (se *stepperEnable) setMotorsEnable(names []string, enable bool) error {
    if err := se.toolhead.flushStepGeneration(); err != nil {
        return err
    }
    var printTime *float64
    didChange := false
    for _, name := range names {
        et := se.lines[name]
        if et == nil {
            return fmt.Errorf("unknown stepper %q", name)
        }
        if et.isEnabled == enable {
            continue
        }
        if printTime == nil {
            if !enable {
                if err := se.toolhead.dwell(disableStallTimeSec); err != nil {
                    return err
                }
            }
            pt, err := se.toolhead.getLastMoveTime()
            if err != nil {
                return err
            }
            printTime = &pt
        }
        if enable {
            et.motorEnable(*printTime)
        } else {
            et.motorDisable(*printTime)
        }
        didChange = true
    }
    if didChange && !enable {
        return se.toolhead.dwell(disableStallTimeSec)
    }
    return nil
}

func (se *stepperEnable) motorOff() error {
    var names []string
    for name := range se.lines {
        names = append(names, name)
    }
    sort.Strings(names)
    if err := se.setMotorsEnable(names, false); err != nil {
        return err
    }
    se.toolhead.kin.ClearHomingState("xyz")
    return nil
}

type manualStepper struct {
    name string

    stepper *stepper
    toolhead *toolhead
    motion   *motionQueuing

    trapq *chelper.TrapQ

    velocity float64
    accel    float64

    nextCmdTime  float64
    commandedPos float64

    axisGcodeID        string
    instantCornerV     float64
    gaxisLimitVelocity float64
    gaxisLimitAccel    float64
}

func newManualStepper(name string, st *stepper, th *toolhead, motion *motionQueuing, tq *chelper.TrapQ, velocity float64, accel float64) *manualStepper {
    return &manualStepper{
        name:              name,
        stepper:            st,
        toolhead:          th,
        motion:            motion,
        trapq:             tq,
        velocity:          velocity,
        accel:             accel,
        nextCmdTime:       0.0,
        commandedPos:      0.0,
        axisGcodeID:       "",
        instantCornerV:    0.0,
        gaxisLimitVelocity: 0.0,
        gaxisLimitAccel:    0.0,
    }
}

func (ms *manualStepper) GetName() string { return ms.name }
func (ms *manualStepper) GetAxisGcodeID() string { return ms.axisGcodeID }

func (ms *manualStepper) syncPrintTime() error {
    pt, err := ms.toolhead.getLastMoveTime()
    if err != nil {
        return err
    }
    if ms.nextCmdTime > pt {
        return ms.toolhead.dwell(ms.nextCmdTime - pt)
    }
    ms.nextCmdTime = pt
    return nil
}

func calcMoveTime(dist float64, speed float64, accel float64) (axisR float64, accelT float64, cruiseT float64, cruiseV float64) {
    axisR = 1.0
    if dist < 0.0 {
        axisR = -1.0
        dist = -dist
    }
    if accel == 0.0 || dist == 0.0 {
        return axisR, 0.0, dist / speed, speed
    }
    maxCruiseV2 := dist * accel
    if maxCruiseV2 < speed*speed {
        speed = math.Sqrt(maxCruiseV2)
    }
    accelT = speed / accel
    accelDecelD := accelT * speed
    cruiseT = (dist - accelDecelD) / speed
    return axisR, accelT, cruiseT, speed
}

func (ms *manualStepper) submitMove(moveTime float64, movePos float64, speed float64, accel float64) float64 {
    cp := ms.commandedPos
    dist := movePos - cp
    axisR, accelT, cruiseT, cruiseV := calcMoveTime(dist, speed, accel)
    ms.trapq.Append(
        moveTime,
        accelT,
        cruiseT,
        accelT,
        cp,
        0.0,
        0.0,
        axisR,
        0.0,
        0.0,
        0.0,
        cruiseV,
        accel,
    )
    ms.commandedPos = movePos
    return moveTime + accelT + cruiseT + accelT
}

func (ms *manualStepper) doEnable(enable bool, se *stepperEnable) error {
    stepperName := ms.name
    return se.setMotorsEnable([]string{stepperName}, enable)
}

func (ms *manualStepper) doSetPosition(setPos float64) error {
    if err := ms.toolhead.flushStepGeneration(); err != nil {
        return err
    }
    ms.commandedPos = setPos
    ms.stepper.setPosition(setPos)
    return nil
}

func (ms *manualStepper) doMove(movePos float64, speed float64, accel float64, sync bool) error {
    if err := ms.syncPrintTime(); err != nil {
        return err
    }
    ms.nextCmdTime = ms.submitMove(ms.nextCmdTime, movePos, speed, accel)
    if err := ms.motion.noteMovequeueActivity(ms.nextCmdTime, true); err != nil {
        return err
    }
    if sync {
        return ms.syncPrintTime()
    }
    return nil
}

func (ms *manualStepper) commandWithGcodeAxis(axis string, instantCornerV float64, limitVelocity float64, limitAccel float64, th *toolhead) error {
    gaxis := strings.ToUpper(strings.TrimSpace(axis))
    if ms.axisGcodeID != "" {
        if gaxis != "" {
            return fmt.Errorf("must unregister axis first")
        }
        th.removeExtraAxis(ms)
        ms.axisGcodeID = ""
        return nil
    }
    if gaxis == "" {
        return nil
    }
    if len(gaxis) != 1 || gaxis[0] < 'A' || gaxis[0] > 'Z' || strings.Contains("XYZEFN", gaxis) {
        return fmt.Errorf("invalid GCODE_AXIS %q", gaxis)
    }
    ms.axisGcodeID = gaxis
    ms.instantCornerV = instantCornerV
    ms.gaxisLimitVelocity = limitVelocity
    ms.gaxisLimitAccel = limitAccel
    th.addExtraAxis(ms, ms.commandedPos)
    return nil
}

func (ms *manualStepper) ProcessMove(printTime float64, mv *move, axisIndex int) error {
    axisR := mv.axesR[axisIndex]
    startPos := mv.startPos[axisIndex]
    accel := mv.accel * axisR
    startV := mv.startV * axisR
    cruiseV := mv.cruiseV * axisR
    ms.trapq.Append(
        printTime,
        mv.accelT,
        mv.cruiseT,
        mv.decelT,
        startPos,
        0.0,
        0.0,
        1.0,
        0.0,
        0.0,
        startV,
        cruiseV,
        accel,
    )
    ms.commandedPos = mv.endPos[axisIndex]
    return nil
}

func (ms *manualStepper) CheckMove(mv *move, axisIndex int) error {
    axisRatio := mv.moveD / math.Abs(mv.axesD[axisIndex])
    limitVelocity := ms.gaxisLimitVelocity * axisRatio
    limitAccel := ms.gaxisLimitAccel * axisRatio
    if !mv.isKinematicMove && ms.accel != 0.0 {
        limitAccel = math.Min(limitAccel, ms.accel*axisRatio)
    }
    mv.limitSpeed(limitVelocity, limitAccel)
    return nil
}

func (ms *manualStepper) CalcJunction(prev *move, mv *move, axisIndex int) float64 {
    diffR := mv.axesR[axisIndex] - prev.axesR[axisIndex]
    if diffR != 0.0 {
        v := ms.instantCornerV / math.Abs(diffR)
        return v * v
    }
    return mv.maxCruiseV2
}

type forceMove struct {
    toolhead *toolhead
    motion   *motionQueuing

    trapq *chelper.TrapQ
    sk    *chelper.StepperKinematics

    stepperEnable *stepperEnable
    steppers      map[string]*stepper
}

func newForceMove(th *toolhead, motion *motionQueuing, se *stepperEnable) (*forceMove, error) {
    tq, err := motion.allocTrapQ()
    if err != nil {
        return nil, err
    }
    sk, err := chelper.NewCartesianStepperKinematics('x')
    if err != nil {
        return nil, err
    }
    return &forceMove{
        toolhead:       th,
        motion:         motion,
        trapq:          tq,
        sk:             sk,
        stepperEnable:  se,
        steppers:       map[string]*stepper{},
    }, nil
}

func (fm *forceMove) free() {
    if fm == nil {
        return
    }
    if fm.sk != nil {
        fm.sk.Free()
        fm.sk = nil
    }
}

func (fm *forceMove) registerStepper(name string, st *stepper) {
    fm.steppers[name] = st
}

func (fm *forceMove) manualMove(st *stepper, dist float64, speed float64, accel float64) error {
    if err := fm.toolhead.flushStepGeneration(); err != nil {
        return err
    }
    prevSK := st.setStepperKinematics(fm.sk)
    prevTQ := st.setTrapQ(fm.trapq)
    st.setPosition(0.0)
    axisR, accelT, cruiseT, cruiseV := calcMoveTime(dist, speed, accel)
    pt, err := fm.toolhead.getLastMoveTime()
    if err != nil {
        return err
    }
    fm.trapq.Append(
        pt,
        accelT,
        cruiseT,
        accelT,
        0.0,
        0.0,
        0.0,
        axisR,
        0.0,
        0.0,
        0.0,
        cruiseV,
        accel,
    )
    end := pt + accelT + cruiseT + accelT
    if err := fm.motion.noteMovequeueActivity(end, true); err != nil {
        return err
    }
    if err := fm.toolhead.dwell(accelT + cruiseT + accelT); err != nil {
        return err
    }
    if err := fm.toolhead.flushStepGeneration(); err != nil {
        return err
    }
    st.setTrapQ(prevTQ)
    st.setStepperKinematics(prevSK)
    fm.motion.wipeTrapQ(fm.trapq)
    return nil
}

func (fm *forceMove) stepperBuzz(stepperName string) error {
    st := fm.steppers[stepperName]
    if st == nil {
        return fmt.Errorf("unknown stepper %q", stepperName)
    }
    didEnable := false
    if et := fm.stepperEnable.lines[stepperName]; et != nil && !et.isEnabled {
        didEnable = true
    }
    if err := fm.stepperEnable.setMotorsEnable([]string{stepperName}, true); err != nil {
        return err
    }
    for i := 0; i < 10; i++ {
        if err := fm.manualMove(st, buzzDistanceMM, buzzVelocityMMPerS, 0.0); err != nil {
            return err
        }
        if err := fm.toolhead.dwell(0.050); err != nil {
            return err
        }
        if err := fm.manualMove(st, -buzzDistanceMM, buzzVelocityMMPerS, 0.0); err != nil {
            return err
        }
        if err := fm.toolhead.dwell(0.450); err != nil {
            return err
        }
    }
    if didEnable {
        if err := fm.stepperEnable.setMotorsEnable([]string{stepperName}, false); err != nil {
            return err
        }
    }
    return nil
}

type runtime struct {
    dict        *protocol.Dictionary
    formats     map[string]*protocol.MessageFormat
    mcuFreq     float64
    queueStepID int32
    setDirID    int32

    sq       *chelper.SerialQueue
    cqMain   *chelper.CommandQueue
    cqEnPins map[string]*chelper.CommandQueue

    motion  *motionQueuing
    toolhead *toolhead
    stepperEnable *stepperEnable
    forceMove *forceMove

    steppers map[string]*stepper
    manuals  map[string]*manualStepper

    rawPath string
    rawFile *os.File
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
    sq.SetClockEst(1e12, 0.0, 0, 0)

    cqMain := chelper.NewCommandQueue()
    if cqMain == nil {
        sq.Free()
        f.Close()
        return nil, fmt.Errorf("alloc main command queue failed")
    }
    cqEnPins := map[string]*chelper.CommandQueue{}

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
    fm, err := newForceMove(th, motion, se)
    if err != nil {
        motion.free()
        cqMain.Free()
        sq.Free()
        f.Close()
        return nil, err
    }

    r := &runtime{
        dict:        dict,
        formats:     formats,
        mcuFreq:     mcuFreq,
        queueStepID: queueStepID,
        setDirID:    setDirID,
        sq:          sq,
        cqMain:      cqMain,
        cqEnPins:    cqEnPins,
        motion:      motion,
        toolhead:    th,
        stepperEnable: se,
        forceMove:   fm,
        steppers:    map[string]*stepper{},
        manuals:     map[string]*manualStepper{},
        rawPath:     rawPath,
        rawFile:     f,
    }

    _ = cfgPath
    return r, nil
}

func (r *runtime) closeAndRead() ([]byte, error) {
    rawPath := r.rawPath
    defer os.Remove(rawPath)
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
    r.motion.flushAllSteps()
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
    r.forceMove.free()
    for _, cq := range r.cqEnPins {
        cq.Free()
    }
    r.cqMain.Free()
    r.motion.free()
    r.sq.Free()

    if r.rawFile != nil {
        _ = r.rawFile.Close()
        r.rawFile = nil
    }
    b, err := os.ReadFile(rawPath)
    if err != nil {
        return nil, err
    }
    return b, nil
}

func (r *runtime) sendLine(line string, cq *chelper.CommandQueue, minClock uint64, reqClock uint64) error {
    b, err := protocol.EncodeCommand(r.formats, line)
    if err != nil {
        return err
    }
    return r.sq.Send(cq, b, minClock, reqClock)
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

func computeFinalizeCRC(lines []string) uint32 {
    return crc32.ChecksumIEEE([]byte(strings.Join(lines, "\n")))
}
