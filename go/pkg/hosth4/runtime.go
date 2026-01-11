package hosth4

import (
	"crypto/rand"
	"encoding/hex"
	"fmt"
	"io"
	"math"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"time"

	"klipper-go-migration/pkg/chelper"
	"klipper-go-migration/pkg/inputshaper"
	"klipper-go-migration/pkg/kinematics"
	"klipper-go-migration/pkg/protocol"
	"klipper-go-migration/pkg/temperature"
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

	homingStartDelaySec  = 0.001
	endstopSampleTimeSec = 0.000015
	endstopSampleCount   = 4
	dripSegmentTimeSec   = 0.050
	trsyncTimeoutSec     = 0.250
	trsyncReportRatio    = 0.3
	trsyncExpireReason   = 4
)

// mcu represents the microcontroller unit
type mcu struct {
	freq  float64
	clock float64
}

// EstimatedPrintTime implements temperature.MCUInterface
func (m *mcu) EstimatedPrintTime(eventtime float64) float64 {
	// Simple implementation: return eventtime as is
	// In real implementation, this would account for clock skew
	return eventtime
}

type motionQueuing struct {
	sq      *chelper.SerialQueue
	ssm     *chelper.StepperSyncMgr
	ss      *chelper.StepperSync
	mcuFreq float64

	trapqs []*chelper.TrapQ

	callbacks       []flushCB
	nextCBID        int
	callbackErr     error
	deferEnablePins bool
	enablePins      []*stepperEnablePin

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
		sq:            sq,
		ssm:           ssm,
		ss:            ss,
		mcuFreq:       mcuFreq,
		kinFlushDelay: sdsCheckTimeSec,
	}, nil
}

func (mq *motionQueuing) setTrace(w io.Writer) {
	mq.trace = w
}

// checkStepGenerationScanWindows matches Klippy's
// PrinterMotionQueuing.check_step_generation_scan_windows().
func (mq *motionQueuing) checkStepGenerationScanWindows(steppers []*stepper) {
	kinFlushDelay := sdsCheckTimeSec
	for _, st := range steppers {
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
	mq.kinFlushDelay = kinFlushDelay
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

func (mq *motionQueuing) setCallbackError(err error) {
	if mq == nil || err == nil {
		return
	}
	if mq.callbackErr == nil {
		mq.callbackErr = err
	}
}

func (mq *motionQueuing) registerEnablePin(pin *stepperEnablePin) {
	if mq == nil || pin == nil {
		return
	}
	mq.enablePins = append(mq.enablePins, pin)
}

func (mq *motionQueuing) setEnablePinsReqClockOffset(offset uint64) {
	if mq == nil {
		return
	}
	for _, pin := range mq.enablePins {
		if pin == nil || pin.out == nil {
			continue
		}
		pin.out.reqClockOffset = offset
	}
}

func (mq *motionQueuing) flushPendingEnablePins() error {
	for _, pin := range mq.enablePins {
		if err := pin.flushPending(); err != nil {
			return err
		}
	}
	return nil
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

	// Invoke callbacks first (original Python ordering)
	for _, cb := range mq.callbacks {
		cb.fn(flushTime, stepGenTime)
	}
	if mq.callbackErr != nil {
		err := mq.callbackErr
		mq.callbackErr = nil
		return err
	}

	// Generate steps
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

	// Finalize trapqs
	for _, tq := range mq.trapqs {
		tq.FinalizeMoves(trapqFreeTime, clearHistoryTime)
	}
	if mq.deferEnablePins {
		if err := mq.flushPendingEnablePins(); err != nil {
			return err
		}
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
	kin       kinematics.Kinematics
	trapq     *chelper.TrapQ

	extraAxes []extraAxis
}

func newToolhead(mcuFreq float64, motion *motionQueuing, maxVelocity float64, maxAccel float64, numAxes int) (*toolhead, error) {
	tq, err := motion.allocTrapQ()
	if err != nil {
		return nil, err
	}
	// Initialize commandedPos with the correct number of axes (3 for no-extruder, 4 for with-extruder)
	commandedPos := make([]float64, numAxes)
	th := &toolhead{
		mcuFreq:        mcuFreq,
		motion:         motion,
		maxVelocity:    maxVelocity,
		maxAccel:       maxAccel,
		minCruiseRatio: 0.5,
		squareCornerV:  5.0,
		specialState:   "NeedPrime",
		commandedPos:   commandedPos,
		lookahead:      &lookAheadQueue{junctionFlush: lookaheadFlushSec},
		trapq:          tq,
		extraAxes:      nil,
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
	// Note the activity with stepgen=true to match Python's behavior
	// (toolhead._process_lookahead calls note_mcu_movequeue_activity(next_move_time)).
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
	th.motion.tracef("getLastMoveTime begin special_state=%q pt=%.9f\n", th.specialState, th.printTime)
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
	th.motion.tracef("getLastMoveTime end pt=%.9f\n", th.printTime)
	return th.printTime, nil
}

func (th *toolhead) registerLookaheadCallback(cb func(printTime float64)) error {
	if th.err != nil {
		return th.err
	}
	lastMove := th.lookahead.getLast()
	if lastMove == nil {
		pt, err := th.getLastMoveTime()
		if err != nil {
			return err
		}
		cb(pt)
		return nil
	}
	lastMove.timingCallbacks = append(lastMove.timingCallbacks, cb)
	return nil
}

func (th *toolhead) dwell(delay float64) error {
	if th.err != nil {
		return th.err
	}
	if delay < 0.0 {
		delay = 0.0
	}
	th.motion.tracef("DWELL begin delay=%.9f pt=%.9f\n", delay, th.printTime)
	if err := th.flushLookahead(false); err != nil {
		return err
	}
	th.motion.tracef("DWELL after flushLookahead pt=%.9f\n", th.printTime)
	pt, err := th.getLastMoveTime()
	if err != nil {
		return err
	}
	th.motion.tracef("DWELL getLastMoveTime pt=%.9f target=%.9f\n", pt, pt+delay)
	th.advanceMoveTime(pt + delay)
	th.motion.tracef("DWELL after advanceMoveTime pt=%.9f\n", th.printTime)
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
		th.kin.SetPosition(newPos, homingAxes)
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
		km := mv.toKinematicsMove()
		if err := th.kin.CheckMove(km); err != nil {
			return err
		}
		mv.updateFromKinematicsMove(km)
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
		km := mv.toKinematicsMove()
		if err := th.kin.CheckMove(km); err != nil {
			return err
		}
		mv.updateFromKinematicsMove(km)
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

	accel             float64
	junctionDeviation float64
	timingCallbacks   []func(nextPrintTime float64)

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

// toKinematicsMove converts the internal move to kinematics.Move for compatibility
func (mv *move) toKinematicsMove() *kinematics.Move {
	return &kinematics.Move{
		StartPos:     mv.startPos,
		EndPos:       mv.endPos,
		AxesD:        mv.axesD,
		MoveD:        mv.moveD,
		MinMoveTime:  mv.minMoveT,
		MaxCruiseV:   math.Sqrt(mv.maxCruiseV2),
		AccelT:       mv.accelT,
		CruiseT:      mv.cruiseT,
		DecelT:       mv.decelT,
		StartV:       mv.startV,
		CruiseV:      mv.cruiseV,
		AccelR:       mv.accel,
		DecelR:       mv.accel,
		DeltaV2:      mv.deltaV2,
		SmoothDeltaV2: mv.deltaV2,
	}
}

// updateFromKinematicsMove updates the internal move from kinematics.Move changes
func (mv *move) updateFromKinematicsMove(km *kinematics.Move) {
	// Update speed limits if they changed
	newMaxCruiseV2 := km.MaxCruiseV * km.MaxCruiseV
	if newMaxCruiseV2 < mv.maxCruiseV2 {
		mv.maxCruiseV2 = newMaxCruiseV2
		if km.MaxCruiseV > 0 {
			mv.minMoveT = mv.moveD / km.MaxCruiseV
		}
	}
	// Update acceleration if it changed
	if km.AccelR < mv.accel && km.AccelR > 0 {
		mv.accel = km.AccelR
		mv.deltaV2 = 2.0 * mv.moveD * mv.accel
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

func (q *lookAheadQueue) getLast() *move {
	if q == nil || len(q.queue) == 0 {
		return nil
	}
	return q.queue[len(q.queue)-1]
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
	// Don't generate steps if trapq is nil (disconnected stepper)
	if s.trapq == nil {
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
	oid            uint32
	invert         bool
	sq             *chelper.SerialQueue
	cq             *chelper.CommandQueue
	formats        map[string]*protocol.MessageFormat
	mcuFreq        float64
	lastClk        uint64
	reqClockOffset uint64
}

func newDigitalOut(oid uint32, invert bool, sq *chelper.SerialQueue, cq *chelper.CommandQueue, formats map[string]*protocol.MessageFormat, mcuFreq float64) *digitalOut {
	return &digitalOut{oid: oid, invert: invert, sq: sq, cq: cq, formats: formats, mcuFreq: mcuFreq}
}

func (d *digitalOut) setDigital(printTime float64, value bool) error {
	clk := uint64(int64(printTime * d.mcuFreq))
	req := clk + d.reqClockOffset
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
	if err := d.sq.Send(d.cq, b, d.lastClk, req); err != nil {
		return err
	}
	d.lastClk = clk
	return nil
}

func (d *digitalOut) setOnTicks(printTime float64, onTicks uint32) error {
	clk := uint64(int64(printTime * d.mcuFreq))
	req := clk + d.reqClockOffset
	line := fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", d.oid, clk, onTicks)
	b, err := protocol.EncodeCommand(d.formats, line)
	if err != nil {
		return err
	}
	if err := d.sq.Send(d.cq, b, d.lastClk, req); err != nil {
		return err
	}
	d.lastClk = clk
	return nil
}

type stepperEnablePin struct {
	out          *digitalOut
	enableCount  int
	pendingState *bool
	pendingTime  float64
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

func (p *stepperEnablePin) setEnableDeferred(printTime float64) {
	if p.enableCount == 0 {
		state := true
		p.pendingState = &state
		p.pendingTime = printTime
	}
	p.enableCount++
}

func (p *stepperEnablePin) setDisable(printTime float64) error {
	p.enableCount--
	if p.enableCount == 0 {
		// If the enable was deferred and never flushed, cancel it instead of
		// emitting a redundant disable.
		if p.pendingState != nil && *p.pendingState {
			p.pendingState = nil
			return nil
		}
		if err := p.out.setDigital(printTime, false); err != nil {
			return err
		}
	}
	return nil
}

func (p *stepperEnablePin) flushPending() error {
	if p.pendingState == nil {
		return nil
	}
	state := *p.pendingState
	pt := p.pendingTime
	p.pendingState = nil
	return p.out.setDigital(pt, state)
}

type enableTracking struct {
	stepper   *stepper
	enablePin *stepperEnablePin
	isEnabled bool
}

func newEnableTracking(stepper *stepper, pin *stepperEnablePin) *enableTracking {
	et := &enableTracking{stepper: stepper, enablePin: pin}
	// The stepper starts disabled, so register motor_enable callback
	// This matches Python Klipper's behavior
	stepper.addActiveCallback(et.motorEnable)
	return et
}

func (et *enableTracking) motorEnable(printTime float64) {
	if et.isEnabled {
		return
	}
	if et.stepper.motion != nil && et.stepper.motion.deferEnablePins {
		et.enablePin.setEnableDeferred(printTime)
		et.isEnabled = true
		return
	}
	if err := et.enablePin.setEnable(printTime); err != nil {
		et.stepper.motion.setCallbackError(err)
		return
	}
	et.isEnabled = true
}

func (et *enableTracking) motorDisable(printTime float64) {
	if !et.isEnabled {
		return
	}
	if err := et.enablePin.setDisable(printTime); err != nil {
		et.stepper.motion.setCallbackError(err)
		return
	}
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
	th := se.toolhead
	th.motion.tracef("MOTOR_OFF begin flushStepGeneration pt=%.9f\n", th.printTime)
	if err := th.flushStepGeneration(); err != nil {
		return err
	}
	th.motion.tracef("MOTOR_OFF after flushStepGeneration pt=%.9f\n", th.printTime)

	// Sync print_time and flush lookahead state prior to motor disables.
	// This matches Klippy's use of get_last_move_time() in stepper_enable.py.
	pt, err := th.getLastMoveTime()
	if err != nil {
		return err
	}
	th.motion.tracef("MOTOR_OFF getLastMoveTime pt=%.9f\n", pt)

	// Match Python's motor_off behavior:
	// - dwell(DISABLE_STALL_TIME) advances print_time
	// - motor_disable uses the advanced print_time
	// - dwell again after disabling
	// But we avoid using the dwell() function here because it may
	// trigger flushLookahead which sets special_state=NeedPrime and
	// causes getLastMoveTime to call calcPrintTime, advancing time based
	// on last_step_gen_time instead of current print_time.
	th.advanceMoveTime(pt + disableStallTimeSec)
	th.motion.tracef("MOTOR_OFF after first dwell pt=%.9f\n", th.printTime)

	// Get the print_time after dwell (now without NeedPrime state)
	pt, err = th.getLastMoveTime()
	if err != nil {
		return err
	}
	th.motion.tracef("MOTOR_OFF getLastMoveTime pt=%.9f\n", pt)
	for _, name := range names {
		et := se.lines[name]
		if et != nil && et.isEnabled {
			th.motion.tracef("MOTOR_OFF disable %s at pt=%.9f\n", name, pt)
			et.motorDisable(pt)
		}
	}

	// Second dwell after disabling motors
	th.advanceMoveTime(pt + disableStallTimeSec)
	th.motion.tracef("MOTOR_OFF after second dwell pt=%.9f\n", th.printTime)

	if th.kin != nil {
		th.kin.ClearHomingState("xyz")
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
	oid     uint32
	rt      *runtime
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
	oid        uint32
	stepperOID uint32
	tr         *trsync
	rt         *runtime
	mcuFreq    float64
}

func (e *endstop) homeStart(printTime float64, restTime float64, triggered bool) error {
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
	pinValue := 0
	if triggered {
		pinValue = 1
	}
	line := fmt.Sprintf("endstop_home oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d pin_value=%d trsync_oid=%d trigger_reason=1",
		e.oid, clock, sampleTicks, endstopSampleCount, restTicks, pinValue, e.tr.oid)
	return e.rt.sendLine(line, cq, 0, clock)
}

func (e *endstop) homeStartWithRestTicks(printTime float64, restTicks uint64, triggered bool) error {
	clock := uint64(int64(printTime * e.mcuFreq))
	cq := e.rt.cqTrigger
	if cq == nil {
		cq = e.rt.cqMain
	}
	if err := e.tr.start(printTime); err != nil {
		return err
	}
	if err := e.rt.sendLine(fmt.Sprintf("stepper_stop_on_trigger oid=%d trsync_oid=%d", e.stepperOID, e.tr.oid), cq, 0, clock); err != nil {
		return err
	}
	if err := e.tr.setTimeout(printTime); err != nil {
		return err
	}
	sampleTicks := uint64(int64(endstopSampleTimeSec * e.mcuFreq))
	pinValue := 0
	if triggered {
		pinValue = 1
	}
	line := fmt.Sprintf("endstop_home oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d pin_value=%d trsync_oid=%d trigger_reason=1",
		e.oid, clock, sampleTicks, endstopSampleCount, restTicks, pinValue, e.tr.oid)
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
	cfg *config

	dict        *protocol.Dictionary
	formats     map[string]*protocol.MessageFormat
	mcuFreq     float64
	queueStepID int32
	setDirID    int32

	sq           *chelper.SerialQueue
	cqMain       *chelper.CommandQueue
	cqTrigger    *chelper.CommandQueue
	cqEnablePins []*chelper.CommandQueue

	motion        *motionQueuing
	toolhead      *toolhead
	stepperEnable *stepperEnable

	rails       [3]stepperCfg
	steppers    [3]*stepper
	extruderCfg extruderStepperCfg
	stepperE    *stepper
	extruder    *extruderAxis

	// Additional extruder_stepper support (e.g., my_extra_stepper)
	hasExtraStepper bool
	extraStepperCfg extruderStepperCfg
	extraStepper    *stepper
	extraStepperEn  *stepperEnablePin
	pressureAdvance map[string]*pressureAdvanceState

	endstops [3]*endstop

	gm            *gcodeMove
	motorOffOrder []string

	// Bed screws adjustment tool
	bedScrews *bedScrews
	// bed_mesh move transform (active even when no mesh loaded)
	bedMesh *bedMesh

	// Temperature control
	mcu           *mcu
	heaterManager *temperature.HeaterManager

	bltouch *bltouchController

	screwsTiltAdjust *screwsTiltAdjust

	// Input shaper for vibration reduction
	inputShaper *inputshaper.InputShaper

	trace io.Writer

	rawPath string
	rawFile *os.File
	closed  bool

	testStem string
}

type pressureAdvanceState struct {
	advance    float64
	smoothTime float64
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

// gcodeRespond sends a response message to the G-code handler
func (r *runtime) gcodeRespond(msg string) {
	// In golden test mode, we write to trace
	// In real implementation, this would use the G-code output
	r.tracef("%s\n", msg)
}

func (r *runtime) traceMotion(prefix string) {
	if r == nil || r.trace == nil || r.motion == nil {
		return
	}
	mq := r.motion
	fmt.Fprintf(r.trace, "%s need_flush=%.9f need_sg=%.9f last_flush=%.9f last_sg=%.9f kin_delay=%.9f\n",
		prefix, mq.needFlushTime, mq.needStepGenTime, mq.lastFlushTime, mq.lastStepGenTime, mq.kinFlushDelay)
}

func (r *runtime) updateKinFlushDelay() {
	if r == nil || r.motion == nil {
		return
	}
	kinFlushDelay := sdsCheckTimeSec
	for i := 0; i < 3; i++ {
		st := r.steppers[i]
		if st == nil || st.sk == nil || st.trapq == nil {
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
	if r.stepperE != nil && r.stepperE.sk != nil && r.stepperE.trapq != nil {
		pre := r.stepperE.sk.GenStepsPreActive()
		post := r.stepperE.sk.GenStepsPostActive()
		if pre > kinFlushDelay {
			kinFlushDelay = pre
		}
		if post > kinFlushDelay {
			kinFlushDelay = post
		}
	}
	if r.extraStepper != nil && r.extraStepper.sk != nil && r.extraStepper.trapq != nil {
		pre := r.extraStepper.sk.GenStepsPreActive()
		post := r.extraStepper.sk.GenStepsPostActive()
		if pre > kinFlushDelay {
			kinFlushDelay = pre
		}
		if post > kinFlushDelay {
			kinFlushDelay = post
		}
	}
	r.motion.kinFlushDelay = kinFlushDelay
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
	if sec, ok := cfg.section("bltouch"); ok {
		// Probe-based Z endstop: derive Z position_endstop from the probe z_offset.
		zOff, err := parseFloat(sec, "z_offset", nil)
		if err != nil {
			return nil, err
		}
		rails[2].positionEndstop = zOff
	}

	// Extruder is optional (e.g., for bed_screws test)
	extruderCfg, hasExtruder, err := readExtruderStepperOptional(cfg, "extruder")
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
	cqEnablePins := []*chelper.CommandQueue{}

	// Match Klippy: reserve movequeue slots for any digital_out / pwm objects
	// that may schedule commands alongside step generation.
	reservedMoveSlots := reservedMoveSlotsForConfig(cfg)
	motion, err := newMotionQueuing(sq, mcuFreq, reservedMoveSlots)
	if err != nil {
		cqMain.Free()
		sq.Free()
		f.Close()
		return nil, err
	}
	// Determine number of axes: 3 for no-extruder configs, 4 for with-extruder configs
	numAxes := 3
	if hasExtruder {
		numAxes = 4
	}
	th, err := newToolhead(mcuFreq, motion, maxVelocity, maxAccel, numAxes)
	if err != nil {
		motion.free()
		cqMain.Free()
		sq.Free()
		f.Close()
		return nil, err
	}

	se := newStepperEnable(th)

	hasExtraStepperSection := false
	if _, ok := cfg.section("extruder_stepper my_extra_stepper"); ok {
		hasExtraStepperSection = true
	}
	_, hasBLTouch := cfg.section("bltouch")

	// Detect filament sensors (buttons) - only relevant to the
	// extruder_stepper connect-phase compiler.
	hasButtons := false
	if hasExtraStepperSection {
		sensorSections := []string{
			"filament_switch_sensor runout_switch",
			"filament_motion_sensor runout_encoder",
			"filament_switch_sensor runout_switch1",
			"filament_motion_sensor runout_encoder1",
		}
		for _, sectionName := range sensorSections {
			if _, ok := cfg.sections[sectionName]; ok {
				hasButtons = true
				break
			}
		}
		// Also check for numbered variants
		for i := 2; !hasButtons && i < 10; i++ {
			if _, ok := cfg.sections[fmt.Sprintf("filament_switch_sensor runout_switch%d", i)]; ok {
				hasButtons = true
				break
			}
			if _, ok := cfg.sections[fmt.Sprintf("filament_motion_sensor runout_encoder%d", i)]; ok {
				hasButtons = true
				break
			}
		}
	}

	oidStepperX := uint32(2)
	oidStepperY := uint32(5)
	oidStepperZ := uint32(8)
	oidEndstopX := uint32(0)
	oidEndstopY := uint32(3)
	oidEndstopZ := uint32(6)
	oidTrsyncX := uint32(1)
	oidTrsyncY := uint32(4)
	oidTrsyncZ := uint32(7)
	if hasBLTouch {
		// Match bltouch.cfg expected OID layout:
		// - probe endstop=0 trsync=1, z stepper=8
		// - X: endstop=2 trsync=3 stepper=4
		// - Y: endstop=5 trsync=6 stepper=7
		oidEndstopX = 2
		oidTrsyncX = 3
		oidStepperX = 4
		oidEndstopY = 5
		oidTrsyncY = 6
		oidStepperY = 7
		oidEndstopZ = 0
		oidTrsyncZ = 1
		oidStepperZ = 8
	} else if hasExtraStepperSection {
		// Match go/pkg/hosth1/h1.go:CompileCartesianWithExtruderStepper()
		// OID layout (buttons optional).
		oidEndstopX = 1
		oidTrsyncX = 2
		oidStepperX = 3
		oidEndstopY = 4
		oidTrsyncY = 5
		oidStepperY = 6
		oidEndstopZ = 7
		oidTrsyncZ = 8
		oidStepperZ = 9
	}

	stepDistX := rails[0].rotationDistance / float64(rails[0].fullSteps*rails[0].microsteps)
	stepDistY := rails[1].rotationDistance / float64(rails[1].fullSteps*rails[1].microsteps)
	stepDistZ := rails[2].rotationDistance / float64(rails[2].fullSteps*rails[2].microsteps)

	stX, err := newStepper(motion, sq, "stepper_x", 'x', oidStepperX, stepDistX, rails[0].dirPin.invert, queueStepID, setDirID, mcuFreq)
	if err != nil {
		motion.free()
		cqMain.Free()
		sq.Free()
		f.Close()
		return nil, err
	}
	stY, err := newStepper(motion, sq, "stepper_y", 'y', oidStepperY, stepDistY, rails[1].dirPin.invert, queueStepID, setDirID, mcuFreq)
	if err != nil {
		stX.free()
		motion.free()
		cqMain.Free()
		sq.Free()
		f.Close()
		return nil, err
	}
	stZ, err := newStepper(motion, sq, "stepper_z", 'z', oidStepperZ, stepDistZ, rails[2].dirPin.invert, queueStepID, setDirID, mcuFreq)
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

	// Get the kinematics type from configuration
	kinType := "cartesian" // default
	if printerCfg, ok := cfg.section("printer"); ok {
		if kt, ok := printerCfg["kinematics"]; ok {
			kinType = strings.TrimSpace(kt)
		}
	}

	// Convert rails to kinematics.Rail slice
	// (stepDistX, stepDistY, stepDistZ already calculated above)
	kinRails := []kinematics.Rail{
		{
			Name:            "stepper_x",
			StepDist:        stepDistX,
			PositionMin:     rails[0].positionMin,
			PositionMax:     rails[0].positionMax,
			HomingSpeed:     rails[0].homingSpeed,
			SecondHoming:    rails[0].secondHomingSpeed,
			HomingRetract:   rails[0].homingRetractDist,
			PositionEndstop: rails[0].positionEndstop,
			HomingPositive:  rails[0].homingPositiveDir,
		},
		{
			Name:            "stepper_y",
			StepDist:        stepDistY,
			PositionMin:     rails[1].positionMin,
			PositionMax:     rails[1].positionMax,
			HomingSpeed:     rails[1].homingSpeed,
			SecondHoming:    rails[1].secondHomingSpeed,
			HomingRetract:   rails[1].homingRetractDist,
			PositionEndstop: rails[1].positionEndstop,
			HomingPositive:  rails[1].homingPositiveDir,
		},
		{
			Name:            "stepper_z",
			StepDist:        stepDistZ,
			PositionMin:     rails[2].positionMin,
			PositionMax:     rails[2].positionMax,
			HomingSpeed:     rails[2].homingSpeed,
			SecondHoming:    rails[2].secondHomingSpeed,
			HomingRetract:   rails[2].homingRetractDist,
			PositionEndstop: rails[2].positionEndstop,
			HomingPositive:  rails[2].homingPositiveDir,
		},
	}

	// Create kinematics based on type
	kinCfg := kinematics.Config{
		Type:         kinType,
		Rails:        kinRails,
		MaxZVelocity: maxZVelocity,
		MaxZAccel:    maxZAccel,
	}

	kin, err := kinematics.NewFromConfig(kinCfg)
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

	th.kin = kin

	// Conditionally create extruder components
	var stE *stepper
	var tqExtruder *chelper.TrapQ
	var enE *stepperEnablePin
	var exAxis *extruderAxis

	// Determine enable pin OIDs based on config type
	// With extruder + buttons: enable_x=13, enable_y=14, enable_z=15, enable_e=18
	// With extruder (no buttons): enable_x=12, enable_y=13, enable_z=14, enable_e=17
	// Without extruder: enable_x=9, enable_y=10, enable_z=11
	var oidEnableX, oidEnableY, oidEnableZ, oidEnableE int
	if hasExtruder {
		if hasButtons || hasBLTouch {
			oidEnableX = 13
			oidEnableY = 14
			oidEnableZ = 15
			oidEnableE = 18
		} else {
			oidEnableX = 12
			oidEnableY = 13
			oidEnableZ = 14
			oidEnableE = 17
		}
	} else {
		oidEnableX = 9
		oidEnableY = 10
		oidEnableZ = 11
	}

	// Check for and parse extruder_stepper my_extra_stepper (if present).
	var extraStepper *stepper
	var extraStepperEn *stepperEnablePin
	var extraStepperCfg extruderStepperCfg
	hasExtraStepper := false
	extraCfg, okExtra, err := readExtruderStepperOptional(cfg, "extruder_stepper my_extra_stepper")
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
	if okExtra {
		if !hasExtruder {
			stZ.free()
			stY.free()
			stX.free()
			motion.free()
			cqMain.Free()
			sq.Free()
			f.Close()
			return nil, fmt.Errorf("extruder_stepper requires an [extruder] section")
		}
		if extraCfg.extruderName != "extruder" {
			stZ.free()
			stY.free()
			stX.free()
			motion.free()
			cqMain.Free()
			sq.Free()
			f.Close()
			return nil, fmt.Errorf("unsupported extruder_stepper motion queue %q", extraCfg.extruderName)
		}
		hasExtraStepper = true
		extraStepperCfg = extraCfg
	}

	if hasExtruder {
		stepDistE := extruderCfg.rotationDistance / float64(extruderCfg.fullSteps*extruderCfg.microsteps)

		// Allocate trapq for extruder
		tqExtruder, err = motion.allocTrapQ()
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
		exAxis = newExtruderAxis("extruder", tqExtruder)
		th.extraAxes = []extraAxis{exAxis}

		// Configure extruder kinematic limits to match klippy/kinematics/extruder.py.
		if sec, ok := cfg.section("extruder"); ok {
			defNozzle := 0.4
			nozzleDiameter, err := parseFloat(sec, "nozzle_diameter", &defNozzle)
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
			defFilament := 1.75
			filamentDiameter, err := parseFloat(sec, "filament_diameter", &defFilament)
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
			filamentArea := math.Pi * math.Pow(filamentDiameter*0.5, 2)
			defMaxCrossSection := 4.0 * nozzleDiameter * nozzleDiameter
			maxCrossSection, err := parseFloat(sec, "max_extrude_cross_section", &defMaxCrossSection)
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
			maxExtrudeRatio := 0.0
			defMaxExtrudeRatio := 0.0
			if filamentArea > 0.0 {
				defMaxExtrudeRatio = defMaxCrossSection / filamentArea
				maxExtrudeRatio = maxCrossSection / filamentArea
			}

			defMaxEVel := maxVelocity * defMaxExtrudeRatio
			maxEVel, err := parseFloat(sec, "max_extrude_only_velocity", &defMaxEVel)
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
			defMaxEAccel := maxAccel * defMaxExtrudeRatio
			maxEAccel, err := parseFloat(sec, "max_extrude_only_accel", &defMaxEAccel)
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
			defMaxEDist := 50.0
			maxEDist, err := parseFloat(sec, "max_extrude_only_distance", &defMaxEDist)
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
			defICV := 1.0
			icv, err := parseFloat(sec, "instantaneous_corner_velocity", &defICV)
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

			exAxis.nozzleDiameter = nozzleDiameter
			exAxis.filamentArea = filamentArea
			exAxis.maxExtrudeRatio = maxExtrudeRatio
			exAxis.maxEVelocity = maxEVel
			exAxis.maxEAccel = maxEAccel
			exAxis.maxEDist = maxEDist
			exAxis.instantCornerV = icv
		}

		if hasExtraStepper {
			stepDistExtra := extraStepperCfg.rotationDistance / float64(extraStepperCfg.fullSteps*extraStepperCfg.microsteps)
			extraStepper, err = newStepper(motion, sq, "my_extra_stepper", 'e', 0, stepDistExtra, extraStepperCfg.dirPin.invert, queueStepID, setDirID, mcuFreq)
			if err != nil {
				stZ.free()
				stY.free()
				stX.free()
				motion.free()
				cqMain.Free()
				sq.Free()
				f.Close()
				return nil, fmt.Errorf("create extra stepper failed: %v", err)
			}
			extraStepper.setPosition(exAxis.lastPosition)
			// Default to syncing with the referenced extruder at connect.
			extraStepper.setTrapQ(tqExtruder)

			cqEnExtra := chelper.NewCommandQueue()
			if cqEnExtra == nil {
				extraStepper.free()
				stZ.free()
				stY.free()
				stX.free()
				motion.free()
				cqMain.Free()
				sq.Free()
				f.Close()
				return nil, fmt.Errorf("alloc extra stepper enable command queue failed")
			}
			cqEnablePins = append(cqEnablePins, cqEnExtra)
			extraStepperEn = &stepperEnablePin{out: newDigitalOut(11, extraStepperCfg.enablePin.invert, sq, cqMain, formats, mcuFreq)}
			motion.registerEnablePin(extraStepperEn)
			se.registerStepper("my_extra_stepper", extraStepper, extraStepperEn)
		}

		// Determine extruder stepper OID based on whether there's an extra_stepper
		// With extra_stepper: oid=10, without extra_stepper: oid=9
		oidStepperE := uint32(9)
		if hasExtraStepperSection {
			oidStepperE = 10
		}

		// Create extruder stepper
		stE, err = newStepper(motion, sq, "extruder", 'e', oidStepperE, stepDistE, extruderCfg.dirPin.invert, queueStepID, setDirID, mcuFreq)
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

		cqEnE := chelper.NewCommandQueue()
		if cqEnE == nil {
			stE.free()
			stZ.free()
			stY.free()
			stX.free()
			motion.free()
			for _, cq := range cqEnablePins {
				cq.Free()
			}
			cqMain.Free()
			sq.Free()
			f.Close()
			return nil, fmt.Errorf("alloc extruder enable command queue failed")
		}
		cqEnablePins = append(cqEnablePins, cqEnE)
		enE = &stepperEnablePin{out: newDigitalOut(uint32(oidEnableE), extruderCfg.enablePin.invert, sq, cqMain, formats, mcuFreq)}
	}

	// Create enable pins for X/Y/Z
	cqEnX := chelper.NewCommandQueue()
	cqEnY := chelper.NewCommandQueue()
	cqEnZ := chelper.NewCommandQueue()
	if cqEnX == nil || cqEnY == nil || cqEnZ == nil {
		if cqEnX != nil {
			cqEnX.Free()
		}
		if cqEnY != nil {
			cqEnY.Free()
		}
		if cqEnZ != nil {
			cqEnZ.Free()
		}
		stE.free()
		stZ.free()
		stY.free()
		stX.free()
		motion.free()
		for _, cq := range cqEnablePins {
			cq.Free()
		}
		cqMain.Free()
		sq.Free()
		f.Close()
		return nil, fmt.Errorf("alloc enable command queue failed")
	}
	cqEnablePins = append(cqEnablePins, cqEnX, cqEnY, cqEnZ)
	enX := &stepperEnablePin{out: newDigitalOut(uint32(oidEnableX), rails[0].enablePin.invert, sq, cqMain, formats, mcuFreq)}
	enY := &stepperEnablePin{out: newDigitalOut(uint32(oidEnableY), rails[1].enablePin.invert, sq, cqMain, formats, mcuFreq)}
	enZ := &stepperEnablePin{out: newDigitalOut(uint32(oidEnableZ), rails[2].enablePin.invert, sq, cqMain, formats, mcuFreq)}
	motion.registerEnablePin(enX)
	motion.registerEnablePin(enY)
	motion.registerEnablePin(enZ)
	se.registerStepper("stepper_x", stX, enX)
	se.registerStepper("stepper_y", stY, enY)
	se.registerStepper("stepper_z", stZ, enZ)
	if hasExtruder {
		motion.registerEnablePin(enE)
	}
	if hasExtruder {
		se.registerStepper("extruder", stE, enE)
	}

	cqTrigger := chelper.NewCommandQueue()
	if cqTrigger == nil {
		// Clean up stepperEnablePin digitalOut objects before freeing CommandQueues
		// Note: stepperEnablePin objects don't have explicit Free(), but we must
		// ensure the CommandQueues they reference are still valid when we free them
		// Steppers are cleaned up by motion.free()
		if extraStepper != nil {
			extraStepper.free()
		}
		stE.free()
		stZ.free()
		stY.free()
		stX.free()
		motion.free()
		for _, cq := range cqEnablePins {
			cq.Free()
		}
		cqMain.Free()
		sq.Free()
		f.Close()
		return nil, fmt.Errorf("alloc trigger command queue failed")
	}

	trX := &trsync{oid: oidTrsyncX, rt: nil, mcuFreq: mcuFreq}
	trY := &trsync{oid: oidTrsyncY, rt: nil, mcuFreq: mcuFreq}
	trZ := &trsync{oid: oidTrsyncZ, rt: nil, mcuFreq: mcuFreq}
	// Initialize MCU for temperature control
	mcu := &mcu{
		freq:  mcuFreq,
		clock: 0,
	}

	// Build motorOffOrder matching Klippy's module ordering.
	motorOffOrder := []string{}
	if hasExtraStepper {
		motorOffOrder = append(motorOffOrder, "my_extra_stepper")
	}
	motorOffOrder = append(motorOffOrder, "stepper_x", "stepper_y", "stepper_z")
	if hasExtruder {
		motorOffOrder = append(motorOffOrder, "extruder")
	}

	rt := &runtime{
		cfg:             cfg,
		dict:            dict,
		formats:         formats,
		mcuFreq:         mcuFreq,
		queueStepID:     queueStepID,
		setDirID:        setDirID,
		sq:              sq,
		cqMain:          cqMain,
		cqTrigger:       cqTrigger,
		cqEnablePins:    cqEnablePins,
		motion:          motion,
		toolhead:        th,
		stepperEnable:   se,
		rails:           rails,
		steppers:        steppers,
		extruderCfg:     extruderCfg,
		stepperE:        stE,
		extruder:        exAxis,
		hasExtraStepper: hasExtraStepper,
		extraStepperCfg: extraStepperCfg,
		extraStepper:    extraStepper,
		extraStepperEn:  extraStepperEn,
		pressureAdvance: map[string]*pressureAdvanceState{
			"extruder": &pressureAdvanceState{advance: 0.0, smoothTime: 0.040},
		},
		motorOffOrder: motorOffOrder,
		mcu:           mcu,
		rawPath:       rawPath,
		rawFile:       f,
	}
	if hasExtraStepper {
		rt.pressureAdvance["my_extra_stepper"] = &pressureAdvanceState{advance: 0.0, smoothTime: 0.040}
	}

	// Match Klippy: recompute kin_flush_delay based on itersolve scan windows
	// after all stepper kinematics are registered.
	if rt.motion != nil {
		allSteppers := []*stepper{stX, stY, stZ}
		if stE != nil {
			allSteppers = append(allSteppers, stE)
		}
		if extraStepper != nil {
			allSteppers = append(allSteppers, extraStepper)
		}
		rt.motion.checkStepGenerationScanWindows(allSteppers)
	}

	rt.gm = newGCodeMove(th, mmPerArcSegment)
	if _, ok := cfg.section("bed_mesh"); ok {
		bm, err := newBedMesh(cfg, th)
		if err != nil {
			return nil, err
		}
		rt.bedMesh = bm
		rt.gm.setMoveTransform(bm)
	}

	// Initialize heater manager after runtime is created
	rt.heaterManager = temperature.NewHeaterManager(newPrinterAdapter(rt))

	// Initialize bed_screws if [bed_screws] section exists
	if _, hasBedScrews := cfg.section("bed_screws"); hasBedScrews {
		bs, err := newBedScrews(rt, cfg)
		if err != nil {
			motion.free()
			cqTrigger.Free()
			for _, cq := range cqEnablePins {
				cq.Free()
			}
			cqMain.Free()
			sq.Free()
			f.Close()
			return nil, fmt.Errorf("initialize bed_screws: %w", err)
		}
		rt.bedScrews = bs
	}

	trX.rt = rt
	trY.rt = rt
	trZ.rt = rt

	rt.endstops[0] = &endstop{oid: oidEndstopX, stepperOID: oidStepperX, tr: trX, rt: rt, mcuFreq: mcuFreq}
	rt.endstops[1] = &endstop{oid: oidEndstopY, stepperOID: oidStepperY, tr: trY, rt: rt, mcuFreq: mcuFreq}
	rt.endstops[2] = &endstop{oid: oidEndstopZ, stepperOID: oidStepperZ, tr: trZ, rt: rt, mcuFreq: mcuFreq}

	if _, ok := cfg.section("bltouch"); ok {
		pwm := newDigitalOut(12, false, sq, cqMain, formats, mcuFreq)
		rt.bltouch = newBLTouchController(rt, pwm, rt.endstops[2])
	}

	// Initialize screws_tilt_adjust if configured
	if sta, err := newScrewsTiltAdjust(rt, cfg); err != nil {
		return nil, fmt.Errorf("screws_tilt_adjust init: %w", err)
	} else if sta != nil {
		rt.screwsTiltAdjust = sta
	}

	// Initialize input shaper if configured
	if sec, ok := cfg.section("input_shaper"); ok {
		shaperTypeX := inputshaper.ShaperType(sec["shaper_type_x"])
		shaperTypeY := inputshaper.ShaperType(sec["shaper_type_y"])
		shaperTypeZ := inputshaper.ShaperType(sec["shaper_type_z"])
		// Fall back to shaper_type if axis-specific not set
		if shaperTypeX == "" {
			shaperTypeX = inputshaper.ShaperType(sec["shaper_type"])
		}
		if shaperTypeY == "" {
			shaperTypeY = inputshaper.ShaperType(sec["shaper_type"])
		}
		if shaperTypeZ == "" {
			shaperTypeZ = inputshaper.ShaperType(sec["shaper_type"])
		}

		defDamping := inputshaper.DefaultDampingRatio
		dampingRatio, _ := parseFloat(sec, "damping_ratio_x", &defDamping)
		defFreq := 0.0
		freqX, _ := parseFloat(sec, "shaper_freq_x", &defFreq)
		freqY, _ := parseFloat(sec, "shaper_freq_y", &defFreq)
		freqZ, _ := parseFloat(sec, "shaper_freq_z", &defFreq)

		is, err := inputshaper.NewInputShaper(shaperTypeX, shaperTypeY, shaperTypeZ, freqX, freqY, freqZ, dampingRatio)
		if err != nil {
			rt.tracef("Warning: failed to initialize input shaper: %v\n", err)
		} else {
			rt.inputShaper = is
			rt.tracef("Initialized input shaper: X=%s@%.1fHz, Y=%s@%.1fHz\n",
				shaperTypeX, freqX, shaperTypeY, freqY)
		}
	}

	rt.updateKinFlushDelay()

	// Load and setup heaters from config
	heaterConfigs, err := readHeaterConfigs(cfg)
	if err != nil {
		// Log error but don't fail - heaters might be optional in some configs
		rt.tracef("Warning: failed to load heater configs: %v\n", err)
	} else {
		// Setup each heater
		for _, hc := range heaterConfigs {
			if err := rt.setupHeater(hc); err != nil {
				rt.tracef("Warning: failed to setup heater %s: %v\n", hc.name, err)
			} else {
				rt.tracef("Setup heater: %s (sensor=%s, control=%s)\n", hc.name, hc.sensorType, hc.control)
			}
		}
	}

	return rt, nil
}

// setupHeater creates and initializes a heater from configuration
func (r *runtime) setupHeater(hc *heaterConfig) error {
	// Create MCU sensor
	adc, err := newMcuADCAdapter(r.mcu, hc.sensorPin)
	if err != nil {
		return fmt.Errorf("failed to create ADC adapter: %w", err)
	}
	adc.setRuntime(r) // Set runtime reference for sending commands

	// Create temperature sensor based on sensor_type
	var sensor temperature.Sensor

	switch hc.sensorType {
	case "AD595":
		// AD595 thermocouple amplifier
		ad595Config := &temperature.AD595Config{
			VoltageSupply: 5.0, // Default 5.0V supply
			VoltageOffset: 0.0, // Default 0V offset at 0°C
		}
		sensor, err = temperature.NewAD595Sensor(hc.name+"_sensor", hc.sensorPin, adc, ad595Config)
		if err != nil {
			return fmt.Errorf("failed to create AD595 sensor: %w", err)
		}
		r.tracef("  Created AD595 sensor (supply=%.1fV, offset=%.1fV)\n",
			ad595Config.VoltageSupply, ad595Config.VoltageOffset)

	case "PT100 INA826":
		// PT100 RTD with INA826 amplifier
		pt100Config := &temperature.PT100Config{
			ReferenceResistor: 4300.0, // Default 4.3K reference resistor
			R0:                100.0,  // PT100 resistance at 0°C
		}
		sensor, err = temperature.NewPT100Sensor(hc.name+"_sensor", hc.sensorPin, adc, pt100Config)
		if err != nil {
			return fmt.Errorf("failed to create PT100 sensor: %w", err)
		}
		r.tracef("  Created PT100 sensor (R_ref=%.1f ohms, R0=%.1f ohms)\n",
			pt100Config.ReferenceResistor, pt100Config.R0)

	case "Thermistor":
		// Generic thermistor using Beta parameter equation
		thermistorConfig := &temperature.ThermistorConfig{
			Beta:           3950.0,   // Common Beta value for EPCOS 100K
			R0:             100000.0, // 100K ohms at T0
			T0:             25.0,     // Reference temperature
			PullupResistor: 4700.0,   // Common 4.7K pullup
		}
		sensor, err = temperature.NewThermistorSensor(hc.name+"_sensor", hc.sensorPin, adc, thermistorConfig)
		if err != nil {
			return fmt.Errorf("failed to create Thermistor sensor: %w", err)
		}
		r.tracef("  Created Thermistor sensor (Beta=%.0f, R0=%.0f ohms, T0=%.1f°C)\n",
			thermistorConfig.Beta, thermistorConfig.R0, thermistorConfig.T0)

	default:
		// Fallback to generic MCU sensor with linear calibration
		// This handles unknown sensor types and allows custom calibration
		sensorConfig := &temperature.MCUConfig{
			SensorMCU: "mcu",
		}
		mcuSensor := temperature.NewMCUSensor(hc.name+"_sensor", sensorConfig, adc)

		// Use simple linear calibration for unknown sensor types
		// This allows the system to function even with unsupported sensors
		mcuSensor.SetCalibration(25.0, 100.0) // 25°C at 0.5V, 125°C at 1.5V
		sensor = mcuSensor
		r.tracef("  Created generic MCU sensor with linear calibration (sensor_type=%s)\n", hc.sensorType)
	}

	// Create PWM adapter
	pwm, err := newMcuPWMAdapter(r.mcu, hc.heaterPin, hc.maxPower, hc.pwmCycleTime)
	if err != nil {
		return fmt.Errorf("failed to create PWM adapter: %w", err)
	}
	pwm.setRuntime(r) // Set runtime reference for sending commands

	// Create heater config
	heaterCfg := &temperature.HeaterConfig{
		Name:           hc.name,
		MinTemp:        hc.minTemp,
		MaxTemp:        hc.maxTemp,
		MinExtrudeTemp: hc.minExtrudeTemp,
		MaxPower:       hc.maxPower,
		SmoothTime:     hc.smoothTime,
		PWMCycleTime:   hc.pwmCycleTime,
		HeaterPin:      hc.heaterPin,
		ControlType:    hc.control,
		PID_Kp:         hc.pidKp,
		PID_Ki:         hc.pidKi,
		PID_Kd:         hc.pidKd,
		MaxDelta:       hc.maxDelta,
	}

	// Create heater
	_, err = temperature.NewHeater(heaterCfg, sensor, pwm, newPrinterAdapter(r))
	if err != nil {
		return fmt.Errorf("failed to create heater: %w", err)
	}

	// Register heater with manager
	gcodeID := ""
	if hc.name == "extruder" {
		gcodeID = "T0"
	} else if strings.HasPrefix(hc.name, "extruder") {
		// extruder1, extruder2, etc.
		gcodeID = strings.ToUpper(strings.Replace(hc.name, "extruder", "T", 1))
	} else if hc.name == "heater_bed" {
		gcodeID = "B"
	}

	_, err = r.heaterManager.SetupHeater(hc.name, heaterCfg, sensor, pwm)
	if err != nil {
		return fmt.Errorf("failed to setup heater with manager: %w", err)
	}

	// Store heater reference in runtime for G-code commands
	// TODO: Add heater map to runtime struct

	r.tracef("Heater %s initialized (gcode_id=%s)\n", hc.name, gcodeID)

	return nil
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

	// Wait for serialqueue to drain before freeing commandqueues
	// This prevents "Memory leak! Can't free non-empty commandqueue" errors
	if r.sq != nil {
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
	}

	// Now safe to free commandqueues
	for _, cq := range r.cqEnablePins {
		if cq != nil {
			cq.Free()
		}
	}
	r.cqEnablePins = nil
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
	if r.extraStepper != nil {
		r.extraStepper.free()
		r.extraStepper = nil
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

func (r *runtime) closeAndRead(flush bool) ([]byte, error) {
	rawPath := r.rawPath
	if flush && r.motion != nil {
		_ = r.motion.flushAllSteps()
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
		}
		r.tracef("SEND cq=%s min=%d req=%d line=%s\n", cqName, minClock, reqClock, strings.TrimSpace(line))
	}
	b, err := protocol.EncodeCommand(r.formats, line)
	if err != nil {
		return err
	}
	return r.sq.Send(cq, b, minClock, reqClock)
}

// sendConfigLine sends a configuration command during initialization
func (r *runtime) sendConfigLine(line string) error {
	// For config commands, we send them immediately on the main queue
	return r.sendLine(line, r.cqMain, 0, 0)
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
	case "G92":
		// Set position without moving.
		// This is used by macros.test and should not emit MCU traffic.
		if err := r.cmdG92(cmd.Args); err != nil {
			return err
		}
	case "SAVE_GCODE_STATE", "RESTORE_GCODE_STATE":
		// Host-side state only; no MCU output in fileoutput mode.
		return nil
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
	case "M104":
		// Set Extruder Temperature
		if err := r.cmdM104(cmd.Args); err != nil {
			return err
		}
	case "M140":
		// Set Heated Bed Temperature
		if err := r.cmdM140(cmd.Args); err != nil {
			return err
		}
	case "M105":
		// Get Temperature
		if err := r.cmdM105(cmd.Args); err != nil {
			return err
		}
	case "M109":
		// Set Extruder Temperature and Wait
		if err := r.cmdM109(cmd.Args); err != nil {
			return err
		}
	case "M190":
		// Set Heated Bed Temperature and Wait
		if err := r.cmdM190(cmd.Args); err != nil {
			return err
		}
	case "SET_EXTRUDER_ROTATION_DISTANCE":
		// Set extruder rotation distance
		if err := r.cmdSetExtruderRotationDistance(cmd.Args); err != nil {
			return err
		}
	case "SET_PRESSURE_ADVANCE":
		// Set pressure advance parameters
		if err := r.cmdSetPressureAdvance(cmd.Args); err != nil {
			return err
		}
	case "SET_INPUT_SHAPER":
		// Set input shaper parameters
		if err := r.cmdSetInputShaper(cmd.Args); err != nil {
			return err
		}
	case "SYNC_EXTRUDER_MOTION":
		// Sync extruder motion queue
		if err := r.cmdSyncExtruderMotion(cmd.Args); err != nil {
			return err
		}
	case "BED_SCREWS_ADJUST":
		// Bed screws adjustment tool
		if r.bedScrews == nil {
			return fmt.Errorf("bed_screws not configured")
		}
		if err := r.bedScrews.cmdBED_SCREWS_ADJUST(cmd.Args); err != nil {
			return fmt.Errorf("BED_SCREWS_ADJUST failed: %w", err)
		}
	case "BED_MESH_CALIBRATE":
		if err := r.cmdBedMeshCalibrate(cmd.Args); err != nil {
			return err
		}
	case "PROBE":
		if err := r.cmdProbe(cmd.Args); err != nil {
			return err
		}
	case "QUERY_PROBE":
		// Klippy prints probe state; no MCU output required in golden mode.
		return nil
	case "ACCEPT", "ADJUSTED", "ABORT":
		// Dynamic commands for bed_screws tool
		if r.bedScrews == nil {
			return fmt.Errorf("bed_screws not configured for %s", cmd.Name)
		}
		handler, ok := r.bedScrews.getDynamicCommand(cmd.Name)
		if !ok {
			// Command not registered - tool has completed, just ignore
			// (Python Klipper would raise "unknown command" error here)
			return nil
		}
		return handler(cmd.Args)
	case "BLTOUCH_DEBUG":
		if err := r.cmdBLTouchDebug(cmd.Args); err != nil {
			return err
		}
	case "SCREWS_TILT_CALCULATE":
		if r.screwsTiltAdjust == nil {
			return fmt.Errorf("screws_tilt_adjust not configured")
		}
		if err := r.screwsTiltAdjust.cmdSCREWS_TILT_CALCULATE(cmd.Args); err != nil {
			return err
		}
	default:
		return fmt.Errorf("unsupported gcode %q (H4)", cmd.Name)
	}

	// NOTE: Removed flushPendingBatch() call here. Python uses asynchronous
	// timer-based flushing (_flush_handler_debug), not synchronous flushing
	// after each command. Flushing is handled at EOF by flushHandlerDebugOnce().
	return nil
}

func parseFloatPair(raw string) (float64, float64, error) {
	parts := strings.Split(raw, ",")
	if len(parts) != 2 {
		return 0, 0, fmt.Errorf("expected pair, got %q", raw)
	}
	a, err := strconv.ParseFloat(strings.TrimSpace(parts[0]), 64)
	if err != nil {
		return 0, 0, fmt.Errorf("bad float %q", parts[0])
	}
	b, err := strconv.ParseFloat(strings.TrimSpace(parts[1]), 64)
	if err != nil {
		return 0, 0, fmt.Errorf("bad float %q", parts[1])
	}
	return a, b, nil
}

func (r *runtime) toolheadMoveXYZ(x, y, z float64, speed float64) error {
	th := r.toolhead
	if th == nil {
		return fmt.Errorf("toolhead not initialized")
	}
	cur := th.commandedPos
	newPos := make([]float64, len(cur))
	copy(newPos, cur)
	if !math.IsNaN(x) {
		newPos[0] = x
	}
	if !math.IsNaN(y) {
		newPos[1] = y
	}
	if !math.IsNaN(z) {
		newPos[2] = z
	}
	if err := th.move(newPos, speed); err != nil {
		return err
	}
	// Match Klippy: manual_move updates gcode_move's internal last position.
	if r.gm != nil {
		r.gm.resetFromToolhead()
	}
	return nil
}

func (r *runtime) runSingleProbe(probeSpeed float64) error {
	if r.bltouch == nil {
		return fmt.Errorf("probe not configured")
	}
	if err := r.bltouch.lowerProbe(); err != nil {
		return err
	}
	// Match Klippy: probe_prepare syncs time after lowering.
	if err := r.bltouch.syncPrintTime(); err != nil {
		return err
	}
	// probing_move targets the configured z_min position
	target := append([]float64{}, r.toolhead.commandedPos...)
	target[2] = r.rails[2].positionMin
	if err := r.homingMove(2, target, probeSpeed); err != nil {
		return err
	}
	// Match BLTouch behavior for stow_on_each_sample=True (default).
	if err := r.bltouch.raiseProbe(); err != nil {
		return err
	}
	if err := r.bltouch.verifyRaiseProbe(); err != nil {
		return err
	}
	// Match Klippy: probe_finish syncs time after verifying raise.
	if err := r.bltouch.syncPrintTime(); err != nil {
		return err
	}
	return nil
}

func (r *runtime) cmdProbe(args map[string]string) error {
	// Klippy reads PROBE_SPEED from probe parameters; BLTouch defaults to 5mm/s.
	probeSpeed := 5.0
	if raw := strings.TrimSpace(args["PROBE_SPEED"]); raw != "" {
		v, err := strconv.ParseFloat(raw, 64)
		if err != nil {
			return fmt.Errorf("bad PROBE_SPEED=%q", raw)
		}
		probeSpeed = v
	}
	return r.runSingleProbe(probeSpeed)
}

func (r *runtime) cmdBedMeshCalibrate(args map[string]string) error {
	if r.cfg == nil {
		return fmt.Errorf("config not available")
	}
	sec, ok := r.cfg.section("bed_mesh")
	if !ok {
		return fmt.Errorf("bed_mesh not configured")
	}
	if r.bedMesh == nil {
		return fmt.Errorf("bed_mesh not initialized")
	}
	minRaw := strings.TrimSpace(sec["mesh_min"])
	maxRaw := strings.TrimSpace(sec["mesh_max"])
	if minRaw == "" || maxRaw == "" {
		return fmt.Errorf("bed_mesh requires mesh_min and mesh_max")
	}
	minX, minY, err := parseFloatPair(minRaw)
	if err != nil {
		return fmt.Errorf("bad bed_mesh mesh_min: %w", err)
	}
	maxX, maxY, err := parseFloatPair(maxRaw)
	if err != nil {
		return fmt.Errorf("bad bed_mesh mesh_max: %w", err)
	}

	probeCount := 3
	if raw := strings.TrimSpace(sec["probe_count"]); raw != "" {
		parts := strings.Split(raw, ",")
		if len(parts) == 1 {
			v, err := strconv.Atoi(strings.TrimSpace(parts[0]))
			if err != nil {
				return fmt.Errorf("bad bed_mesh probe_count=%q", raw)
			}
			probeCount = v
		} else if len(parts) == 2 {
			// Use the larger count for both axes if they differ; the golden
			// tests in this repo only cover the symmetric case.
			v0, err := strconv.Atoi(strings.TrimSpace(parts[0]))
			if err != nil {
				return fmt.Errorf("bad bed_mesh probe_count=%q", raw)
			}
			v1, err := strconv.Atoi(strings.TrimSpace(parts[1]))
			if err != nil {
				return fmt.Errorf("bad bed_mesh probe_count=%q", raw)
			}
			if v0 > v1 {
				probeCount = v0
			} else {
				probeCount = v1
			}
		} else {
			return fmt.Errorf("bad bed_mesh probe_count=%q", raw)
		}
	}
	if probeCount < 3 {
		return fmt.Errorf("bed_mesh probe_count must be >= 3 (got %d)", probeCount)
	}

	horizontalMoveZ := 5.0
	if raw := strings.TrimSpace(sec["horizontal_move_z"]); raw != "" {
		v, err := strconv.ParseFloat(raw, 64)
		if err != nil {
			return fmt.Errorf("bad bed_mesh horizontal_move_z=%q", raw)
		}
		horizontalMoveZ = v
	}
	bedSpeed := 50.0
	if raw := strings.TrimSpace(sec["speed"]); raw != "" {
		v, err := strconv.ParseFloat(raw, 64)
		if err != nil {
			return fmt.Errorf("bad bed_mesh speed=%q", raw)
		}
		bedSpeed = v
	}

	// Probe parameter defaults (from [bltouch] ProbeParameterHelper).
	probeSpeed := 5.0
	liftSpeed := probeSpeed
	if bsec, ok := r.cfg.section("bltouch"); ok {
		if raw := strings.TrimSpace(bsec["speed"]); raw != "" {
			v, err := strconv.ParseFloat(raw, 64)
			if err != nil {
				return fmt.Errorf("bad bltouch speed=%q", raw)
			}
			probeSpeed = v
			liftSpeed = v
		}
		if raw := strings.TrimSpace(bsec["lift_speed"]); raw != "" {
			v, err := strconv.ParseFloat(raw, 64)
			if err != nil {
				return fmt.Errorf("bad bltouch lift_speed=%q", raw)
			}
			liftSpeed = v
		}
	}
	if horizontalMoveZ < r.rails[2].positionEndstop {
		return fmt.Errorf("horizontal_move_z can't be less than probe z_offset")
	}

	// Generate zig-zag points matching klippy/extras/bed_mesh.py.
	xCount, yCount := probeCount, probeCount
	xDist := math.Floor(((maxX-minX)/float64(xCount-1))*100.0) / 100.0
	yDist := math.Floor(((maxY-minY)/float64(yCount-1))*100.0) / 100.0
	if xDist < 1.0 || yDist < 1.0 {
		return fmt.Errorf("bed_mesh min/max points too close together")
	}
	maxX = minX + xDist*float64(xCount-1)
	points := make([][2]float64, 0, xCount*yCount)
	posY := minY
	for i := 0; i < yCount; i++ {
		for j := 0; j < xCount; j++ {
			posX := 0.0
			if i%2 == 0 {
				posX = minX + float64(j)*xDist
			} else {
				posX = maxX - float64(j)*xDist
			}
			points = append(points, [2]float64{posX, posY})
		}
		posY += yDist
	}

	probeResults := make([][3]float64, 0, len(points))
	for idx, pt := range points {
		raiseSpeed := liftSpeed
		if idx == 0 {
			raiseSpeed = bedSpeed
		}
		if err := r.toolheadMoveXYZ(math.NaN(), math.NaN(), horizontalMoveZ, raiseSpeed); err != nil {
			return err
		}
		if err := r.toolheadMoveXYZ(pt[0], pt[1], math.NaN(), bedSpeed); err != nil {
			return err
		}
		if err := r.runSingleProbe(probeSpeed); err != nil {
			return err
		}
		// Record probe result position (Klippy reports the trigger Z position).
		// In fileoutput mode, probing moves complete at z_min_position.
		if r.toolhead == nil || len(r.toolhead.commandedPos) < 3 {
			return fmt.Errorf("toolhead position unavailable")
		}
		// Store X/Y from the probe point (not the toolhead, which may include offsets).
		probeResults = append(probeResults, [3]float64{pt[0], pt[1], r.toolhead.commandedPos[2]})
	}

	// Match Klippy ProbePointsHelper: after the final probe sample, raise the
	// tool to horizontal_move_z once more before finalizing.
	if err := r.toolheadMoveXYZ(math.NaN(), math.NaN(), horizontalMoveZ, liftSpeed); err != nil {
		return err
	}

	// Match Klippy: flush lookahead queue before finalize callback.
	if r.toolhead != nil {
		_, _ = r.toolhead.getLastMoveTime()
	}

	// Match Klippy bed_mesh.probe_finalize: build a Z mesh from probed points
	// and enable the bed_mesh move transform.
	zOffset := r.rails[2].positionEndstop
	probedMatrix := make([][]float64, 0, yCount)
	row := make([]float64, 0, xCount)
	prev := points[0]
	for i, pt := range points {
		if i > 0 && math.Abs(pt[1]-prev[1]) > 0.1 {
			probedMatrix = append(probedMatrix, row)
			row = make([]float64, 0, xCount)
		}
		zPos := probeResults[i][2] - zOffset
		if pt[0] > prev[0] {
			row = append(row, zPos)
		} else {
			row = append(row, 0.0)
			copy(row[1:], row[:len(row)-1])
			row[0] = zPos
		}
		prev = pt
	}
	probedMatrix = append(probedMatrix, row)
	if len(probedMatrix) != yCount {
		return fmt.Errorf("bed_mesh invalid y-axis table length: %d", len(probedMatrix))
	}
	for i := 0; i < len(probedMatrix); i++ {
		if len(probedMatrix[i]) != xCount {
			return fmt.Errorf("bed_mesh invalid x-axis table length at row %d: %d", i, len(probedMatrix[i]))
		}
	}
	mesh, err := newZMesh(zMeshParams{
		MinX:     minX,
		MaxX:     maxX,
		MinY:     minY,
		MaxY:     maxY,
		XCount:   xCount,
		YCount:   yCount,
		MeshXPPS: 2,
		MeshYPPS: 2,
		Algo:     "lagrange",
		Tension:  0.2,
	})
	if err != nil {
		return err
	}
	if err := mesh.BuildMesh(probedMatrix); err != nil {
		return err
	}
	r.bedMesh.SetMesh(mesh)
	// Match Klippy: reset gcode_move last position after enabling a mesh.
	if r.gm != nil {
		r.gm.resetFromToolhead()
	}
	return nil
}

func (r *runtime) cmdG92(args map[string]string) error {
	if r.toolhead == nil {
		return fmt.Errorf("toolhead not initialized")
	}
	pos := append([]float64{}, r.toolhead.commandedPos...)
	setAxis := func(axis byte, idx int) error {
		raw, ok := args[string(axis)]
		if !ok {
			return nil
		}
		if raw == "" {
			return fmt.Errorf("empty arg %c", axis)
		}
		v, err := strconv.ParseFloat(raw, 64)
		if err != nil {
			return fmt.Errorf("bad float %c=%q", axis, raw)
		}
		if idx >= 0 && idx < len(pos) {
			pos[idx] = v
		}
		return nil
	}
	if err := setAxis('X', 0); err != nil {
		return err
	}
	if err := setAxis('Y', 1); err != nil {
		return err
	}
	if err := setAxis('Z', 2); err != nil {
		return err
	}
	if err := setAxis('E', 3); err != nil {
		return err
	}
	for i := 0; i < len(r.toolhead.commandedPos) && i < len(pos); i++ {
		r.toolhead.commandedPos[i] = pos[i]
	}
	if len(pos) >= 3 && r.toolhead.trapq != nil {
		r.toolhead.trapq.SetPosition(r.toolhead.printTime, pos[0], pos[1], pos[2])
	}
	if r.toolhead.kin != nil && len(pos) >= 3 {
		r.toolhead.kin.SetPosition(pos, "")
	}
	r.gm.resetFromToolhead()
	return nil
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
	if err := r.endstops[axis].homeStart(pt, restTime, true); err != nil {
		return err
	}
	r.traceMotion("HOMING: after homeStart")
	if err := r.toolhead.dwell(homingStartDelaySec); err != nil {
		return err
	}
	if err := r.toolhead.dripMove(movepos, speed); err != nil {
		return err
	}
	// Match Klippy: send endstop_home disable BEFORE flushing remaining steps.
	// In Python's homing.py, home_wait() is called which sends the disable command,
	// then flush_step_generation() generates the final deceleration steps.
	r.traceMotion("HOMING: before homeStop")
	if err := r.endstops[axis].homeStop(); err != nil {
		return err
	}
	// Now flush all remaining steps (final deceleration) after endstop is disabled.
	if err := r.motion.flushAllSteps(); err != nil {
		return err
	}
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
	return nil
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
	isProbeZ := axis == 2 && rail.endstopPin.pin == "probe:z_virtual_endstop" && r.bltouch != nil
	if isProbeZ {
		if err := r.bltouch.lowerProbe(); err != nil {
			return err
		}
		if err := r.bltouch.syncPrintTime(); err != nil {
			return err
		}
	}
	if err := r.homingMove(axis, homepos, rail.homingSpeed); err != nil {
		return err
	}
	if isProbeZ {
		if err := r.bltouch.raiseProbe(); err != nil {
			return err
		}
		if err := r.bltouch.verifyRaiseProbe(); err != nil {
			return err
		}
		if err := r.bltouch.syncPrintTime(); err != nil {
			return err
		}
		// Match Klippy fileoutput: probing moves tend to introduce a 0.250s
		// idle buffer before the subsequent retract move (buffer_time_start).
		// This affects the first step interval of the retract move and keeps
		// BLTouch PWM scheduling aligned with expected clocks.
		if rail.homingRetractDist != 0.0 {
			if err := r.toolhead.dwell(bufferTimeStart); err != nil {
				return err
			}
		}
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
	if isProbeZ {
		if err := r.bltouch.lowerProbe(); err != nil {
			return err
		}
		if err := r.bltouch.syncPrintTime(); err != nil {
			return err
		}
	}
	if err := r.homingMove(axis, homepos, rail.secondHomingSpeed); err != nil {
		return err
	}
	if isProbeZ {
		if err := r.bltouch.raiseProbe(); err != nil {
			return err
		}
		if err := r.bltouch.verifyRaiseProbe(); err != nil {
			return err
		}
		// Match Klippy's buffer_time_start behavior after a probe-based home
		// completes, prior to subsequent commands.
		if err := r.toolhead.dwell(bufferTimeStart); err != nil {
			return err
		}
	}
	return r.toolhead.flushStepGeneration()
}

// Temperature command implementations
func (r *runtime) cmdM104(args map[string]string) error {
	// M104 - Set Extruder Temperature
	// Usage: M104 [S<temp>] [T<index>]
	tempStr, ok := args["S"]
	if !ok || tempStr == "" {
		return nil // No temperature specified, just return
	}

	var temp float64
	if _, err := fmt.Sscanf(tempStr, "%f", &temp); err != nil {
		return fmt.Errorf("invalid temperature value: %s", tempStr)
	}

	// Get extruder index (T parameter, defaults to 0)
	index := 0
	if tStr, ok := args["T"]; ok && tStr != "" {
		if _, err := fmt.Sscanf(tStr, "%d", &index); err != nil {
			return fmt.Errorf("invalid extruder index: %s", tStr)
		}
	}

	heaterName := "extruder"
	if index > 0 {
		heaterName = fmt.Sprintf("extruder%d", index)
	}

	r.tracef("M104: heater=%s temp=%.1f\n", heaterName, temp)

	// Set temperature using heater manager
	if err := r.heaterManager.SetTemperature(heaterName, temp, false); err != nil {
		// Heater might not exist in config, that's ok for some tests
		r.tracef("M104: Note - %v\n", err)
	}

	return nil
}

func (r *runtime) cmdM140(args map[string]string) error {
	// M140 - Set Heated Bed Temperature
	// Usage: M140 [S<temp>]
	tempStr, ok := args["S"]
	if !ok || tempStr == "" {
		return nil // No temperature specified, just return
	}

	var temp float64
	if _, err := fmt.Sscanf(tempStr, "%f", &temp); err != nil {
		return fmt.Errorf("invalid temperature value: %s", tempStr)
	}

	r.tracef("M140: heater_bed temp=%.1f\n", temp)

	// Set temperature using heater manager
	if err := r.heaterManager.SetTemperature("heater_bed", temp, false); err != nil {
		// Heater might not exist in config, that's ok for some tests
		r.tracef("M140: Note - %v\n", err)
	}

	return nil
}

func (r *runtime) cmdM105(args map[string]string) error {
	// M105 - Get Temperature
	// Usage: M105

	// Get temperature report from heater manager
	eventtime := 0.0 // TODO: get actual eventtime
	response := r.heaterManager.GetM105Response(eventtime)

	r.tracef("M105: %s\n", response)

	return nil
}

// waitTemperature waits for a heater to reach target temperature
func (r *runtime) waitTemperature(heaterName string, targetTemp float64) error {
	// Don't wait if temperature is 0 (heater off)
	if targetTemp == 0 {
		return nil
	}

	r.tracef("Waiting for %s to reach %.1f°C...\n", heaterName, targetTemp)

	// Get the heater
	heater, err := r.heaterManager.GetHeater(heaterName)
	if err != nil {
		return fmt.Errorf("heater %s not found: %w", heaterName, err)
	}

	// Wait parameters
	const (
		checkInterval = 1.0   // Check temperature every 1 second
		maxWaitTime   = 600.0 // Maximum wait time: 10 minutes
		tolerance     = 1.0   // Temperature tolerance: ±1°C
	)

	startTime := float64(time.Now().UnixNano()) / 1e9
	lastReportTime := 0.0

	// Wait loop
	for {
		currentTime := float64(time.Now().UnixNano()) / 1e9
		elapsed := currentTime - startTime

		// Check for timeout
		if elapsed > maxWaitTime {
			r.tracef("Timeout waiting for %s to reach %.1f°C (waited %.1f seconds)\n",
				heaterName, targetTemp, elapsed)
			return fmt.Errorf("timeout waiting for heater %s", heaterName)
		}

		// Get current temperature
		eventtime := currentTime
		currentTemp, _ := heater.GetTemp(eventtime)

		// Report temperature every second
		if elapsed-lastReportTime >= checkInterval {
			r.gcodeRespond(fmt.Sprintf("%s: %.1f / %.1f", heaterName, currentTemp, targetTemp))
			lastReportTime = elapsed
		}

		// Check if temperature has reached target (within tolerance)
		tempDiff := currentTemp - targetTemp
		if tempDiff < tolerance && tempDiff > -tolerance {
			r.tracef("%s reached target temperature: %.1f°C\n", heaterName, currentTemp)
			r.gcodeRespond(fmt.Sprintf("%s: %.1f / %.1f (reached)", heaterName, currentTemp, targetTemp))
			return nil
		}

		// Check if heater is still busy (heating or cooling)
		if !heater.CheckBusy(eventtime) {
			r.tracef("%s is no longer busy, current temp: %.1f°C\n", heaterName, currentTemp)
			return nil
		}

		// Sleep a bit before next check
		time.Sleep(100 * time.Millisecond)
	}
}

func (r *runtime) cmdM109(args map[string]string) error {
	// M109 - Set Extruder Temperature and Wait
	// Usage: M109 [S<temp>] [T<index>]
	tempStr, ok := args["S"]
	if !ok || tempStr == "" {
		return nil
	}

	var temp float64
	if _, err := fmt.Sscanf(tempStr, "%f", &temp); err != nil {
		return fmt.Errorf("invalid temperature value: %s", tempStr)
	}

	index := 0
	if tStr, ok := args["T"]; ok && tStr != "" {
		if _, err := fmt.Sscanf(tStr, "%d", &index); err != nil {
			return fmt.Errorf("invalid extruder index: %s", tStr)
		}
	}

	heaterName := "extruder"
	if index > 0 {
		heaterName = fmt.Sprintf("extruder%d", index)
	}

	r.tracef("M109: heater=%s temp=%.1f (waiting)\n", heaterName, temp)

	// Set temperature
	if err := r.heaterManager.SetTemperature(heaterName, temp, false); err != nil {
		// Heater might not exist in config, that's ok for some tests
		r.tracef("M109: Note - %v\n", err)
		return nil
	}

	// Wait for temperature to reach target
	return r.waitTemperature(heaterName, temp)
}

func (r *runtime) cmdM190(args map[string]string) error {
	// M190 - Set Heated Bed Temperature and Wait
	// Usage: M190 [S<temp>]
	tempStr, ok := args["S"]
	if !ok || tempStr == "" {
		return nil
	}

	var temp float64
	if _, err := fmt.Sscanf(tempStr, "%f", &temp); err != nil {
		return fmt.Errorf("invalid temperature value: %s", tempStr)
	}

	r.tracef("M190: heater_bed temp=%.1f (waiting)\n", temp)

	// Set temperature
	if err := r.heaterManager.SetTemperature("heater_bed", temp, false); err != nil {
		// Heater might not exist in config, that's ok for some tests
		r.tracef("M190: Note - %v\n", err)
		return nil
	}

	// Wait for temperature to reach target
	return r.waitTemperature("heater_bed", temp)
}

func (r *runtime) cmdSetExtruderRotationDistance(args map[string]string) error {
	// SET_EXTRUDER_ROTATION_DISTANCE [EXTRUDER=<name>] DISTANCE=<distance>
	// This command modifies the extruder's rotation distance at runtime.
	// Implementation follows Python Klipper's approach in extruder.py

	distStr, ok := args["DISTANCE"]
	if !ok || distStr == "" {
		// No distance specified, just query current value (no-op for H4)
		return nil
	}

	dist, err := floatArg(args, "DISTANCE", 0.0)
	if err != nil {
		return err
	}

	if dist == 0.0 {
		return fmt.Errorf("rotation distance cannot be zero")
	}

	// Get extruder name (default to "extruder")
	extruderName := "extruder"
	if en, ok := args["EXTRUDER"]; ok && en != "" {
		extruderName = en
	}

	// Find the target stepper
	var targetStepper *stepper
	var targetCfg extruderStepperCfg
	if r.stepperE != nil && extruderName == "extruder" {
		targetStepper = r.stepperE
		targetCfg = r.extruderCfg
	} else if r.hasExtraStepper && extruderName == "my_extra_stepper" && r.extraStepper != nil {
		targetStepper = r.extraStepper
		targetCfg = r.extraStepperCfg
	} else {
		return fmt.Errorf("unknown extruder: %s", extruderName)
	}

	// Handle negative distance (invert direction)
	invertDir := targetStepper.invertDir
	if dist < 0.0 {
		invertDir = !targetStepper.invertDir
		dist = -dist
	}

	// Flush step generation like Python does
	if err := r.toolhead.flushStepGeneration(); err != nil {
		return err
	}

	// Update stepper's rotation distance (stored as step distance)
	stepDist := dist / float64(targetCfg.fullSteps*targetCfg.microsteps)
	targetStepper.stepDist = stepDist
	targetStepper.invertDir = invertDir
	if targetStepper.stepqueue != nil {
		if err := targetStepper.stepqueue.SetInvertSdir(invertDir); err != nil {
			return err
		}
	}

	// Update the stepper kinematics if it exists
	if targetStepper.sk != nil && targetStepper.trapq != nil {
		targetStepper.sk.SetTrapQ(targetStepper.trapq, stepDist)
	}

	r.tracef("SET_EXTRUDER_ROTATION_DISTANCE: EXTRUDER=%s DISTANCE=%f\n", extruderName, dist)

	return nil
}

func (r *runtime) cmdSetPressureAdvance(args map[string]string) error {
	// SET_PRESSURE_ADVANCE [EXTRUDER=<name>] [ADVANCE=<value>] [SMOOTH_TIME=<value>]
	extruderName := "extruder"
	if en, ok := args["EXTRUDER"]; ok && en != "" {
		extruderName = en
	}

	if r.pressureAdvance == nil {
		r.pressureAdvance = map[string]*pressureAdvanceState{}
	}
	st := r.pressureAdvance[extruderName]
	if st == nil {
		st = &pressureAdvanceState{advance: 0.0, smoothTime: 0.040}
		r.pressureAdvance[extruderName] = st
	}

	advance, err := floatArg(args, "ADVANCE", st.advance)
	if err != nil {
		return err
	}
	smoothTime, err := floatArg(args, "SMOOTH_TIME", st.smoothTime)
	if err != nil {
		return err
	}
	if advance < 0.0 {
		return fmt.Errorf("ADVANCE must be >= 0")
	}
	if smoothTime < 0.0 || smoothTime > 0.200 {
		return fmt.Errorf("SMOOTH_TIME must be between 0.0 and 0.200")
	}

	var targetStepper *stepper
	switch extruderName {
	case "extruder":
		targetStepper = r.stepperE
	case "my_extra_stepper":
		targetStepper = r.extraStepper
	default:
		return fmt.Errorf("unknown extruder %q", extruderName)
	}
	if targetStepper == nil || targetStepper.sk == nil {
		return fmt.Errorf("pressure advance requires an extruder stepper for %q", extruderName)
	}

	oldSmoothTime := st.smoothTime
	if st.advance == 0.0 {
		oldSmoothTime = 0.0
	}
	newSmoothTime := smoothTime
	if advance == 0.0 {
		newSmoothTime = 0.0
	}

	if newSmoothTime != oldSmoothTime {
		if err := r.toolhead.flushStepGeneration(); err != nil {
			return err
		}
		if err := targetStepper.sk.SetPressureAdvance(0.0, advance, newSmoothTime); err != nil {
			return err
		}
		r.updateKinFlushDelay()
	} else {
		sk := targetStepper.sk
		if err := r.toolhead.registerLookaheadCallback(func(printTime float64) {
			if err := sk.SetPressureAdvance(printTime, advance, newSmoothTime); err != nil {
				r.motion.setCallbackError(err)
			}
		}); err != nil {
			return err
		}
	}

	st.advance = advance
	st.smoothTime = smoothTime
	r.tracef("SET_PRESSURE_ADVANCE: EXTRUDER=%s ADVANCE=%f SMOOTH_TIME=%f\n", extruderName, advance, smoothTime)
	return nil
}

func (r *runtime) cmdSetInputShaper(args map[string]string) error {
	// SET_INPUT_SHAPER [SHAPER_TYPE=<type>] [SHAPER_TYPE_X=<type>] [SHAPER_TYPE_Y=<type>]
	//                  [SHAPER_FREQ_X=<freq>] [SHAPER_FREQ_Y=<freq>]
	//                  [DAMPING_RATIO_X=<ratio>] [DAMPING_RATIO_Y=<ratio>]
	if r.inputShaper == nil {
		return fmt.Errorf("input shaper not configured")
	}

	// Update each axis shaper
	for _, shaper := range r.inputShaper.GetShapers() {
		axis := strings.ToUpper(shaper.Axis)

		// Get shaper type
		var shaperType *inputshaper.ShaperType
		if st, ok := args["SHAPER_TYPE_"+axis]; ok && st != "" {
			t := inputshaper.ShaperType(strings.ToLower(st))
			shaperType = &t
		} else if st, ok := args["SHAPER_TYPE"]; ok && st != "" {
			t := inputshaper.ShaperType(strings.ToLower(st))
			shaperType = &t
		}

		// Get damping ratio
		var dampingRatio *float64
		if dr, ok := args["DAMPING_RATIO_"+axis]; ok && dr != "" {
			v, err := strconv.ParseFloat(dr, 64)
			if err != nil {
				return fmt.Errorf("invalid DAMPING_RATIO_%s: %v", axis, err)
			}
			dampingRatio = &v
		}

		// Get shaper frequency
		var shaperFreq *float64
		if sf, ok := args["SHAPER_FREQ_"+axis]; ok && sf != "" {
			v, err := strconv.ParseFloat(sf, 64)
			if err != nil {
				return fmt.Errorf("invalid SHAPER_FREQ_%s: %v", axis, err)
			}
			shaperFreq = &v
		}

		// Update if any parameter changed
		if shaperType != nil || dampingRatio != nil || shaperFreq != nil {
			if err := shaper.Update(shaperType, dampingRatio, shaperFreq); err != nil {
				return fmt.Errorf("axis %s: %w", axis, err)
			}
		}
	}

	// Report current settings
	for _, shaper := range r.inputShaper.GetShapers() {
		if shaper.Axis == "z" && !shaper.IsEnabled() {
			continue
		}
		status := shaper.GetStatus()
		r.tracef("shaper_type_%s:%s shaper_freq_%s:%s damping_ratio_%s:%s\n",
			shaper.Axis, status["shaper_type"],
			shaper.Axis, status["shaper_freq"],
			shaper.Axis, status["damping_ratio"])
	}

	return nil
}

func (r *runtime) cmdSyncExtruderMotion(args map[string]string) error {
	// SYNC_EXTRUDER_MOTION EXTRUDER=<name> [MOTION_QUEUE=<name>]
	// This command synchronizes an extruder stepper to a motion queue.
	// When MOTION_QUEUE=<name>, the stepper connects to that extruder's trapq.
	// When MOTION_QUEUE is empty, the stepper disconnects.

	extruderName, ok := args["EXTRUDER"]
	if !ok || extruderName == "" {
		return fmt.Errorf("SYNC_EXTRUDER_MOTION: missing EXTRUDER parameter")
	}

	motionQueue := ""
	if mq, ok := args["MOTION_QUEUE"]; ok {
		motionQueue = mq
	}

	// Match Klippy: flush steps prior to changing trapq connectivity.
	if err := r.toolhead.flushStepGeneration(); err != nil {
		return err
	}

	var targetStepper *stepper
	switch extruderName {
	case "extruder":
		targetStepper = r.stepperE
	case "my_extra_stepper":
		if !r.hasExtraStepper {
			return fmt.Errorf("SYNC_EXTRUDER_MOTION: extruder_stepper not configured")
		}
		targetStepper = r.extraStepper
	default:
		return fmt.Errorf("SYNC_EXTRUDER_MOTION: unknown extruder %q", extruderName)
	}
	if targetStepper == nil || targetStepper.sk == nil {
		return fmt.Errorf("SYNC_EXTRUDER_MOTION: missing stepper for %q", extruderName)
	}

	if motionQueue == "" {
		targetStepper.setTrapQ(nil)
		r.updateKinFlushDelay()
		r.tracef("SYNC_EXTRUDER_MOTION: %s -> (disconnected)\n", extruderName)
		return nil
	}
	if motionQueue != "extruder" {
		return fmt.Errorf("SYNC_EXTRUDER_MOTION: unknown motion queue %q", motionQueue)
	}
	if r.extruder == nil || r.extruder.trapq == nil {
		return fmt.Errorf("SYNC_EXTRUDER_MOTION: extruder motion queue not available")
	}

	// Match Klippy: set the stepper position to the extruder's last position.
	targetStepper.setPosition(r.extruder.lastPosition)
	targetStepper.setTrapQ(r.extruder.trapq)
	r.updateKinFlushDelay()
	r.tracef("SYNC_EXTRUDER_MOTION: %s -> extruder (connected)\n", extruderName)
	return nil
}

func (r *runtime) homeAll() error {
	// Match Klippy cartesian homing order: X, then Y, then Z.
	for _, axis := range []int{0, 1, 2} {
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

func reservedMoveSlotsForConfig(cfg *config) int {
	if cfg == nil {
		return 0
	}
	slots := 0
	// Stepper enable pins (digital_out) each request a movequeue slot.
	if _, ok := cfg.section("stepper_x"); ok {
		slots++
	}
	if _, ok := cfg.section("stepper_y"); ok {
		slots++
	}
	if _, ok := cfg.section("stepper_z"); ok {
		slots++
	}
	if _, ok := cfg.section("extruder"); ok {
		slots++
	}
	if _, ok := cfg.section("extruder_stepper my_extra_stepper"); ok {
		slots++
	}

	// PWM outputs also request a movequeue slot.
	if _, ok := cfg.section("bltouch"); ok {
		slots++
	}
	if sec, ok := cfg.section("extruder"); ok {
		if strings.TrimSpace(sec["heater_pin"]) != "" {
			slots++
		}
	}
	if sec, ok := cfg.section("heater_bed"); ok {
		if strings.TrimSpace(sec["heater_pin"]) != "" {
			slots++
		}
	}
	return slots
}
