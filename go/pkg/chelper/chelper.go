package chelper

/*
#cgo CFLAGS: -I${SRCDIR}/../../../klippy/chelper
#cgo LDFLAGS: ${SRCDIR}/../../../klippy/chelper/c_helper.so

#include <stdint.h>
#include <stdlib.h>

#include "msgblock.h"
#include "serialqueue.h"
#include "itersolve.h"
#include "steppersync.h"
#include "stepcompress.h"
#include "trapq.h"
#include "pyhelper.h"

// kin_cartesian.c does not provide a public header; declare the symbol.
struct stepper_kinematics *cartesian_stepper_alloc(char axis);
struct stepper_kinematics *extruder_stepper_alloc(void);
void extruder_stepper_free(struct stepper_kinematics *sk);
void extruder_set_pressure_advance(struct stepper_kinematics *sk, double print_time
    , double pressure_advance, double smooth_time);

// kin_generic.c - generic cartesian kinematics
struct stepper_kinematics *generic_cartesian_stepper_alloc(double a_x, double a_y, double a_z);
void generic_cartesian_stepper_set_coeffs(struct stepper_kinematics *sk
    , double a_x, double a_y, double a_z);

// kin_corexy.c - corexy kinematics
struct stepper_kinematics *corexy_stepper_alloc(char type);

// kin_corexz.c - corexz kinematics
struct stepper_kinematics *corexz_stepper_alloc(char type);

// kin_polar.c - polar kinematics
struct stepper_kinematics *polar_stepper_alloc(char type);

// kin_delta.c - linear delta kinematics
struct stepper_kinematics *delta_stepper_alloc(double arm2, double tower_x, double tower_y);

// kin_rotary_delta.c - rotary delta kinematics
struct stepper_kinematics *rotary_delta_stepper_alloc(double shoulder_radius
    , double shoulder_height, double angle, double upper_arm, double lower_arm);

// kin_deltesian.c - deltesian kinematics
struct stepper_kinematics *deltesian_stepper_alloc(double arm2, double arm_x);

// kin_winch.c - winch (cable robot) kinematics
struct stepper_kinematics *winch_stepper_alloc(double anchor_x, double anchor_y, double anchor_z);

// kin_idex.c - dual carriage (IDEX) kinematics wrapper
struct stepper_kinematics *dual_carriage_alloc(void);
void dual_carriage_set_sk(struct stepper_kinematics *sk, struct stepper_kinematics *orig_sk);
int dual_carriage_set_transform(struct stepper_kinematics *sk, char axis, double scale, double offs);
*/
import "C"

import (
	"fmt"
	"strings"
	"time"
	"unsafe"
)

type SerialQueue struct {
	ptr *C.struct_serialqueue
}

type CommandQueue struct {
	ptr *C.struct_command_queue
}

type StepperSyncMgr struct {
	ptr *C.struct_steppersyncmgr
}

type StepperSync struct {
	ptr *C.struct_steppersync
}

type SyncEmitter struct {
	ptr *C.struct_syncemitter
}

type Stepcompress struct {
	ptr *C.struct_stepcompress
}

type HistorySteps struct {
	FirstClock    uint64
	LastClock     uint64
	StartPosition int64
	StepCount     int
	Interval      int
	Add           int
}

type TrapQ struct {
	ptr *C.struct_trapq
}

type StepperKinematics struct {
	ptr        *C.struct_stepper_kinematics
	isExtruder bool
}

type SerialStats struct {
	BytesWrite    uint64
	BytesRead     uint64
	BytesInvalid  uint64
	SendSeq       uint64
	ReceiveSeq    uint64
	ReadyBytes    uint64
	UpcomingBytes uint64
	Raw           string
}

// PullQueueMessage represents a message received from the MCU.
// This corresponds to C's struct pull_queue_message.
type PullQueueMessage struct {
	Msg         [64]byte // MESSAGE_MAX = 64
	Len         int
	SentTime    float64
	ReceiveTime float64
	NotifyID    uint64
}

// ClockEstimate holds the clock synchronization parameters.
// This corresponds to C's struct clock_estimate.
type ClockEstimate struct {
	LastClock uint64
	ConvClock uint64
	ConvTime  float64
	EstFreq   float64
}

func NewSerialQueue(fd int, fdType byte, clientID int, name string) (*SerialQueue, error) {
	var cname [16]C.char
	nb := []byte(name)
	if len(nb) > 15 {
		nb = nb[:15]
	}
	for i := 0; i < len(nb); i++ {
		cname[i] = C.char(nb[i])
	}
	sq := C.serialqueue_alloc(C.int(fd), C.char(fdType), C.int(clientID), &cname[0])
	if sq == nil {
		return nil, fmt.Errorf("serialqueue_alloc failed")
	}
	return &SerialQueue{ptr: sq}, nil
}

func (sq *SerialQueue) SetClockEst(estFreq float64, convTime float64, convClock uint64, lastClock uint64) {
	C.serialqueue_set_clock_est(sq.ptr, C.double(estFreq), C.double(convTime), C.uint64_t(convClock), C.uint64_t(lastClock))
}

func Monotonic() float64 {
	return float64(C.get_monotonic())
}

func (sq *SerialQueue) GetStats() (SerialStats, error) {
	if sq == nil || sq.ptr == nil {
		return SerialStats{}, fmt.Errorf("serialqueue is nil")
	}
	buf := make([]byte, 4096)
	C.serialqueue_get_stats(sq.ptr, (*C.char)(unsafe.Pointer(&buf[0])), C.int(len(buf)))
	raw := string(buf)
	if i := strings.IndexByte(raw, 0); i >= 0 {
		raw = raw[:i]
	}
	s := SerialStats{Raw: strings.TrimSpace(raw)}
	fields := strings.Fields(s.Raw)
	for _, f := range fields {
		kv := strings.SplitN(f, "=", 2)
		if len(kv) != 2 {
			continue
		}
		key, val := kv[0], kv[1]
		var n uint64
		for i := 0; i < len(val); i++ {
			c := val[i]
			if c < '0' || c > '9' {
				n = 0
				break
			}
			n = n*10 + uint64(c-'0')
		}
		switch key {
		case "bytes_write":
			s.BytesWrite = n
		case "bytes_read":
			s.BytesRead = n
		case "bytes_invalid":
			s.BytesInvalid = n
		case "send_seq":
			s.SendSeq = n
		case "receive_seq":
			s.ReceiveSeq = n
		case "ready_bytes":
			s.ReadyBytes = n
		case "upcoming_bytes":
			s.UpcomingBytes = n
		}
	}
	return s, nil
}

// WaitDrain waits for the serial queue to finish writing all pending messages.
// This is used in fileoutput mode to ensure all step commands are written
// before sending commands that need to appear in a specific order.
func (sq *SerialQueue) WaitDrain() error {
	if sq == nil || sq.ptr == nil {
		return fmt.Errorf("serialqueue is nil")
	}
	for {
		stats, err := sq.GetStats()
		if err != nil {
			return err
		}
		if stats.ReadyBytes == 0 && stats.UpcomingBytes == 0 {
			return nil
		}
		// Small sleep to avoid busy-waiting
		// In fileoutput mode, the background thread runs fast so this is typically immediate
		time.Sleep(100 * time.Microsecond)
	}
}

func (sq *SerialQueue) Free() {
	if sq == nil || sq.ptr == nil {
		return
	}
	C.serialqueue_free(sq.ptr)
	sq.ptr = nil
}

func NewCommandQueue() *CommandQueue {
	cq := C.serialqueue_alloc_commandqueue()
	if cq == nil {
		return nil
	}
	return &CommandQueue{ptr: cq}
}

func (cq *CommandQueue) Free() {
	if cq == nil || cq.ptr == nil {
		return
	}
	C.serialqueue_free_commandqueue(cq.ptr)
	cq.ptr = nil
}

func (sq *SerialQueue) Send(cq *CommandQueue, msg []byte, minClock uint64, reqClock uint64) error {
	if sq == nil || sq.ptr == nil {
		return fmt.Errorf("serialqueue is nil")
	}
	if cq == nil || cq.ptr == nil {
		return fmt.Errorf("command queue is nil")
	}
	if len(msg) == 0 {
		return nil
	}
	C.serialqueue_send(
		sq.ptr,
		cq.ptr,
		(*C.uint8_t)(unsafe.Pointer(&msg[0])),
		C.int(len(msg)),
		C.uint64_t(minClock),
		C.uint64_t(reqClock),
		0,
	)
	return nil
}

// Pull retrieves the next received message from the serial queue.
// This blocks until a message is available.
// Returns the message data, timing information, and any error.
func (sq *SerialQueue) Pull() (PullQueueMessage, error) {
	if sq == nil || sq.ptr == nil {
		return PullQueueMessage{}, fmt.Errorf("serialqueue is nil")
	}
	var pqm C.struct_pull_queue_message
	C.serialqueue_pull(sq.ptr, &pqm)
	var result PullQueueMessage
	result.Len = int(pqm.len)
	if result.Len > 0 && result.Len <= 64 {
		for i := 0; i < result.Len; i++ {
			result.Msg[i] = byte(pqm.msg[i])
		}
	}
	result.SentTime = float64(pqm.sent_time)
	result.ReceiveTime = float64(pqm.receive_time)
	result.NotifyID = uint64(pqm.notify_id)
	return result, nil
}

// Exit signals the serial queue to stop processing and unblocks any Pull calls.
func (sq *SerialQueue) Exit() {
	if sq == nil || sq.ptr == nil {
		return
	}
	C.serialqueue_exit(sq.ptr)
}

// SetReceiveWindow sets the receive window size for flow control.
func (sq *SerialQueue) SetReceiveWindow(receiveWindow int) {
	if sq == nil || sq.ptr == nil {
		return
	}
	C.serialqueue_set_receive_window(sq.ptr, C.int(receiveWindow))
}

// SetWireFrequency sets the baud rate for timing calculations.
func (sq *SerialQueue) SetWireFrequency(frequency float64) {
	if sq == nil || sq.ptr == nil {
		return
	}
	C.serialqueue_set_wire_frequency(sq.ptr, C.double(frequency))
}

// GetClockEst retrieves the current clock estimation parameters.
func (sq *SerialQueue) GetClockEst() (ClockEstimate, error) {
	if sq == nil || sq.ptr == nil {
		return ClockEstimate{}, fmt.Errorf("serialqueue is nil")
	}
	var ce C.struct_clock_estimate
	C.serialqueue_get_clock_est(sq.ptr, &ce)
	return ClockEstimate{
		LastClock: uint64(ce.last_clock),
		ConvClock: uint64(ce.conv_clock),
		ConvTime:  float64(ce.conv_time),
		EstFreq:   float64(ce.est_freq),
	}, nil
}

func (sq *SerialQueue) Ptr() *C.struct_serialqueue { return sq.ptr }

func NewStepperSyncMgr() (*StepperSyncMgr, error) {
	ssm := C.steppersyncmgr_alloc()
	if ssm == nil {
		return nil, fmt.Errorf("steppersyncmgr_alloc failed")
	}
	return &StepperSyncMgr{ptr: ssm}, nil
}

func (ssm *StepperSyncMgr) Free() {
	if ssm == nil || ssm.ptr == nil {
		return
	}
	C.steppersyncmgr_free(ssm.ptr)
	ssm.ptr = nil
}

func (ssm *StepperSyncMgr) AllocStepperSync() (*StepperSync, error) {
	if ssm == nil || ssm.ptr == nil {
		return nil, fmt.Errorf("steppersyncmgr is nil")
	}
	ss := C.steppersyncmgr_alloc_steppersync(ssm.ptr)
	if ss == nil {
		return nil, fmt.Errorf("steppersyncmgr_alloc_steppersync failed")
	}
	return &StepperSync{ptr: ss}, nil
}

func (ss *StepperSync) SetupMoveQueue(sq *SerialQueue, moveNum int) error {
	if ss == nil || ss.ptr == nil {
		return fmt.Errorf("steppersync is nil")
	}
	if sq == nil || sq.ptr == nil {
		return fmt.Errorf("serialqueue is nil")
	}
	C.steppersync_setup_movequeue(ss.ptr, sq.ptr, C.int(moveNum))
	return nil
}

func (ss *StepperSync) SetTime(timeOffset float64, mcuFreq float64) error {
	if ss == nil || ss.ptr == nil {
		return fmt.Errorf("steppersync is nil")
	}
	C.steppersync_set_time(ss.ptr, C.double(timeOffset), C.double(mcuFreq))
	return nil
}

func (ss *StepperSync) AllocSyncEmitter(name string, allocStepcompress bool) (*SyncEmitter, error) {
	if ss == nil || ss.ptr == nil {
		return nil, fmt.Errorf("steppersync is nil")
	}
	var cname [16]C.char
	nb := []byte(name)
	if len(nb) > 15 {
		nb = nb[:15]
	}
	for i := 0; i < len(nb); i++ {
		cname[i] = C.char(nb[i])
	}
	alloc := 0
	if allocStepcompress {
		alloc = 1
	}
	se := C.steppersync_alloc_syncemitter(ss.ptr, &cname[0], C.int(alloc))
	if se == nil {
		return nil, fmt.Errorf("steppersync_alloc_syncemitter failed")
	}
	return &SyncEmitter{ptr: se}, nil
}

func (se *SyncEmitter) GetStepcompress() *Stepcompress {
	if se == nil || se.ptr == nil {
		return nil
	}
	sc := C.syncemitter_get_stepcompress(se.ptr)
	if sc == nil {
		return nil
	}
	return &Stepcompress{ptr: sc}
}

func (se *SyncEmitter) SetStepperKinematics(sk *StepperKinematics) error {
	if se == nil || se.ptr == nil {
		return fmt.Errorf("syncemitter is nil")
	}
	if sk == nil || sk.ptr == nil {
		return fmt.Errorf("stepper kinematics is nil")
	}
	C.syncemitter_set_stepper_kinematics(se.ptr, sk.ptr)
	return nil
}

func (ssm *StepperSyncMgr) GenSteps(flushTime float64, genStepsTime float64, clearHistoryTime float64) error {
	if ssm == nil || ssm.ptr == nil {
		return fmt.Errorf("steppersyncmgr is nil")
	}
	ret := C.steppersyncmgr_gen_steps(ssm.ptr, C.double(flushTime), C.double(genStepsTime), C.double(clearHistoryTime))
	if ret != 0 {
		return fmt.Errorf("steppersyncmgr_gen_steps returned %d", int32(ret))
	}
	return nil
}

func NewTrapQ() (*TrapQ, error) {
	tq := C.trapq_alloc()
	if tq == nil {
		return nil, fmt.Errorf("trapq_alloc failed")
	}
	return &TrapQ{ptr: tq}, nil
}

func (tq *TrapQ) Free() {
	if tq == nil || tq.ptr == nil {
		return
	}
	C.trapq_free(tq.ptr)
	tq.ptr = nil
}

func (tq *TrapQ) Append(
	printTime float64,
	accelT float64,
	cruiseT float64,
	decelT float64,
	x float64,
	y float64,
	z float64,
	axisRx float64,
	axisRy float64,
	axisRz float64,
	startV float64,
	cruiseV float64,
	accel float64,
) {
	C.trapq_append(
		tq.ptr,
		C.double(printTime),
		C.double(accelT),
		C.double(cruiseT),
		C.double(decelT),
		C.double(x),
		C.double(y),
		C.double(z),
		C.double(axisRx),
		C.double(axisRy),
		C.double(axisRz),
		C.double(startV),
		C.double(cruiseV),
		C.double(accel),
	)
}

func (tq *TrapQ) SetPosition(printTime float64, x float64, y float64, z float64) {
	C.trapq_set_position(tq.ptr, C.double(printTime), C.double(x), C.double(y), C.double(z))
}

func (tq *TrapQ) FinalizeMoves(freeTime float64, clearHistoryTime float64) {
	C.trapq_finalize_moves(tq.ptr, C.double(freeTime), C.double(clearHistoryTime))
}

func NewCartesianStepperKinematics(axis byte) (*StepperKinematics, error) {
	sk := C.cartesian_stepper_alloc(C.char(axis))
	if sk == nil {
		return nil, fmt.Errorf("cartesian_stepper_alloc failed")
	}
	return &StepperKinematics{ptr: sk, isExtruder: false}, nil
}

func NewExtruderStepperKinematics() (*StepperKinematics, error) {
	sk := C.extruder_stepper_alloc()
	if sk == nil {
		return nil, fmt.Errorf("extruder_stepper_alloc failed")
	}
	return &StepperKinematics{ptr: sk, isExtruder: true}, nil
}

func (sk *StepperKinematics) Free() {
	if sk == nil || sk.ptr == nil {
		return
	}
	if sk.isExtruder {
		C.extruder_stepper_free(sk.ptr)
	} else {
		C.free(unsafe.Pointer(sk.ptr))
	}
	sk.ptr = nil
}

func (sk *StepperKinematics) SetTrapQ(tq *TrapQ, stepDist float64) {
	var tp *C.struct_trapq
	if tq != nil {
		tp = tq.ptr
	}
	C.itersolve_set_trapq(sk.ptr, tp, C.double(stepDist))
}

func (sk *StepperKinematics) SetPressureAdvance(printTime float64, pressureAdvance float64, smoothTime float64) error {
	if sk == nil || sk.ptr == nil {
		return fmt.Errorf("stepper kinematics is nil")
	}
	if !sk.isExtruder {
		return fmt.Errorf("pressure advance only supported on extruder kinematics")
	}
	C.extruder_set_pressure_advance(sk.ptr, C.double(printTime), C.double(pressureAdvance), C.double(smoothTime))
	return nil
}

func (sk *StepperKinematics) GetTrapQ() *TrapQ {
	tq := C.itersolve_get_trapq(sk.ptr)
	if tq == nil {
		return nil
	}
	return &TrapQ{ptr: tq}
}

func (sk *StepperKinematics) SetPosition(x float64, y float64, z float64) {
	C.itersolve_set_position(sk.ptr, C.double(x), C.double(y), C.double(z))
}

func (sk *StepperKinematics) GetCommandedPos() float64 {
	return float64(C.itersolve_get_commanded_pos(sk.ptr))
}

func (sk *StepperKinematics) CheckActive(flushTime float64) float64 {
	return float64(C.itersolve_check_active(sk.ptr, C.double(flushTime)))
}

func (sk *StepperKinematics) GenStepsPreActive() float64 {
	if sk == nil || sk.ptr == nil {
		return 0.0
	}
	return float64(C.itersolve_get_gen_steps_pre_active(sk.ptr))
}

func (sk *StepperKinematics) GenStepsPostActive() float64 {
	if sk == nil || sk.ptr == nil {
		return 0.0
	}
	return float64(C.itersolve_get_gen_steps_post_active(sk.ptr))
}

func (sc *Stepcompress) Fill(oid uint32, maxError uint32, queueStepTag int32, setDirTag int32) error {
	if sc == nil || sc.ptr == nil {
		return fmt.Errorf("stepcompress is nil")
	}
	C.stepcompress_fill(sc.ptr, C.uint32_t(oid), C.uint32_t(maxError), C.int32_t(queueStepTag), C.int32_t(setDirTag))
	return nil
}

func (sc *Stepcompress) SetInvertSdir(invert bool) error {
	if sc == nil || sc.ptr == nil {
		return fmt.Errorf("stepcompress is nil")
	}
	v := C.uint32_t(0)
	if invert {
		v = 1
	}
	C.stepcompress_set_invert_sdir(sc.ptr, v)
	return nil
}

func (sc *Stepcompress) ExtractOld(startClock uint64, endClock uint64, max int) ([]HistorySteps, error) {
	if sc == nil || sc.ptr == nil {
		return nil, fmt.Errorf("stepcompress is nil")
	}
	if max <= 0 {
		return nil, nil
	}
	buf := make([]C.struct_pull_history_steps, max)
	n := C.stepcompress_extract_old(
		sc.ptr,
		&buf[0],
		C.int(max),
		C.uint64_t(startClock),
		C.uint64_t(endClock),
	)
	if n < 0 {
		return nil, fmt.Errorf("stepcompress_extract_old failed: %d", int(n))
	}
	out := make([]HistorySteps, 0, int(n))
	for i := 0; i < int(n); i++ {
		p := buf[i]
		out = append(out, HistorySteps{
			FirstClock:    uint64(p.first_clock),
			LastClock:     uint64(p.last_clock),
			StartPosition: int64(p.start_position),
			StepCount:     int(p.step_count),
			Interval:      int(p.interval),
			Add:           int(p.add),
		})
	}
	return out, nil
}

func (sc *Stepcompress) Reset(lastStepClock uint64) error {
	if sc == nil || sc.ptr == nil {
		return fmt.Errorf("stepcompress is nil")
	}
	ret := C.stepcompress_reset(sc.ptr, C.uint64_t(lastStepClock))
	if ret != 0 {
		return fmt.Errorf("stepcompress_reset failed: %d", int32(ret))
	}
	return nil
}

// NewGenericCartesianStepperKinematics creates a stepper kinematics for generic
// cartesian motion where position = a_x*X + a_y*Y + a_z*Z.
func NewGenericCartesianStepperKinematics(aX, aY, aZ float64) (*StepperKinematics, error) {
	sk := C.generic_cartesian_stepper_alloc(C.double(aX), C.double(aY), C.double(aZ))
	if sk == nil {
		return nil, fmt.Errorf("generic_cartesian_stepper_alloc failed")
	}
	return &StepperKinematics{ptr: sk, isExtruder: false}, nil
}

// SetGenericCartesianCoeffs updates the kinematic coefficients for a generic
// cartesian stepper.
func (sk *StepperKinematics) SetGenericCartesianCoeffs(aX, aY, aZ float64) {
	C.generic_cartesian_stepper_set_coeffs(sk.ptr, C.double(aX), C.double(aY), C.double(aZ))
}

// NewCoreXYStepperKinematics creates a corexy stepper kinematics.
// stepperType should be '+' for the first stepper (x+y) or '-' for the second stepper (x-y).
func NewCoreXYStepperKinematics(stepperType byte) (*StepperKinematics, error) {
	sk := C.corexy_stepper_alloc(C.char(stepperType))
	if sk == nil {
		return nil, fmt.Errorf("corexy_stepper_alloc failed")
	}
	return &StepperKinematics{ptr: sk, isExtruder: false}, nil
}

// NewCoreXZStepperKinematics creates a corexz stepper kinematics.
// stepperType should be '+' for the first stepper (x+z) or '-' for the second stepper (x-z).
func NewCoreXZStepperKinematics(stepperType byte) (*StepperKinematics, error) {
	sk := C.corexz_stepper_alloc(C.char(stepperType))
	if sk == nil {
		return nil, fmt.Errorf("corexz_stepper_alloc failed")
	}
	return &StepperKinematics{ptr: sk, isExtruder: false}, nil
}

// NewPolarStepperKinematics creates a polar stepper kinematics.
// stepperType should be 'r' for radius (stepper_arm) or 'a' for angle (stepper_bed).
func NewPolarStepperKinematics(stepperType byte) (*StepperKinematics, error) {
	sk := C.polar_stepper_alloc(C.char(stepperType))
	if sk == nil {
		return nil, fmt.Errorf("polar_stepper_alloc failed")
	}
	return &StepperKinematics{ptr: sk, isExtruder: false}, nil
}

// NewDeltaStepperKinematics creates a linear delta stepper kinematics.
// Parameters:
//   - arm2: arm length squared
//   - towerX: X coordinate of the tower
//   - towerY: Y coordinate of the tower
func NewDeltaStepperKinematics(arm2, towerX, towerY float64) (*StepperKinematics, error) {
	sk := C.delta_stepper_alloc(C.double(arm2), C.double(towerX), C.double(towerY))
	if sk == nil {
		return nil, fmt.Errorf("delta_stepper_alloc failed")
	}
	return &StepperKinematics{ptr: sk, isExtruder: false}, nil
}

// NewRotaryDeltaStepperKinematics creates a rotary delta stepper kinematics.
// Parameters:
//   - shoulderRadius: radius from center to shoulder joint
//   - shoulderHeight: height of shoulder joint
//   - angle: angle of this arm (in radians)
//   - upperArm: length of upper arm
//   - lowerArm: length of lower arm
func NewRotaryDeltaStepperKinematics(shoulderRadius, shoulderHeight, angle, upperArm, lowerArm float64) (*StepperKinematics, error) {
	sk := C.rotary_delta_stepper_alloc(C.double(shoulderRadius), C.double(shoulderHeight),
		C.double(angle), C.double(upperArm), C.double(lowerArm))
	if sk == nil {
		return nil, fmt.Errorf("rotary_delta_stepper_alloc failed")
	}
	return &StepperKinematics{ptr: sk, isExtruder: false}, nil
}

// NewDeltesianStepperKinematics creates a deltesian stepper kinematics.
// arm2 is the arm length squared, arm_x is the X position of the arm pivot.
func NewDeltesianStepperKinematics(arm2, armX float64) (*StepperKinematics, error) {
	sk := C.deltesian_stepper_alloc(C.double(arm2), C.double(armX))
	if sk == nil {
		return nil, fmt.Errorf("deltesian_stepper_alloc failed")
	}
	return &StepperKinematics{ptr: sk, isExtruder: false}, nil
}

// NewWinchStepperKinematics creates a winch (cable robot) stepper kinematics.
// Parameters:
//   - anchorX, anchorY, anchorZ: 3D position of the anchor point for this cable
func NewWinchStepperKinematics(anchorX, anchorY, anchorZ float64) (*StepperKinematics, error) {
	sk := C.winch_stepper_alloc(C.double(anchorX), C.double(anchorY), C.double(anchorZ))
	if sk == nil {
		return nil, fmt.Errorf("winch_stepper_alloc failed")
	}
	return &StepperKinematics{ptr: sk, isExtruder: false}, nil
}

// NewDualCarriageStepperKinematics creates a dual carriage wrapper for IDEX printers.
// The wrapper allows transforming X/Y axes with scale and offset.
func NewDualCarriageStepperKinematics() (*StepperKinematics, error) {
	sk := C.dual_carriage_alloc()
	if sk == nil {
		return nil, fmt.Errorf("dual_carriage_alloc failed")
	}
	return &StepperKinematics{ptr: sk, isExtruder: false}, nil
}

// DualCarriageSetSK sets the original stepper kinematics that the dual carriage
// wrapper will delegate to.
func (sk *StepperKinematics) DualCarriageSetSK(origSK *StepperKinematics) {
	C.dual_carriage_set_sk(sk.ptr, origSK.ptr)
}

// DualCarriageSetTransform sets the scale and offset transform for a given axis.
// When scale is 0, the axis is disabled (no steps generated for that axis).
// axis should be 'x' or 'y'.
func (sk *StepperKinematics) DualCarriageSetTransform(axis byte, scale, offset float64) error {
	ret := C.dual_carriage_set_transform(sk.ptr, C.char(axis), C.double(scale), C.double(offset))
	if ret != 0 {
		return fmt.Errorf("dual_carriage_set_transform failed for axis %c", axis)
	}
	return nil
}
