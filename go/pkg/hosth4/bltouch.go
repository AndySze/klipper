package hosth4

import (
	"fmt"
	"math"
	"strings"
)

const (
	bltSignalPeriodSec = 0.020
	bltMinCmdTimeSec   = 5 * bltSignalPeriodSec

	bltPinMoveTimeSec = 0.680

	bltEndstopRestTimeSec = 0.001

	bltTestTimeSec      = 5 * 60.0
	bltRetryResetTimeSec = 1.0

	bltCmdPinDown   = "pin_down"
	bltCmdTouchMode = "touch_mode"
	bltCmdPinUp     = "pin_up"
	bltCmdReset     = "reset"
)

var bltCommandPulseSec = map[string]float64{
	bltCmdPinDown:   0.000650,
	bltCmdTouchMode: 0.001165,
	bltCmdPinUp:     0.001475,
	bltCmdReset:     0.002190,
}

type bltouchController struct {
	rt *runtime

	pwm     *digitalOut
	endstop *endstop

	nextCmdTime   float64
	actionEndTime float64
	nextTestTime  float64

	pinMoveTime float64

	pinUpTouchTriggered bool
	pinUpNotTriggered   bool
	probeTouchMode      bool
}

func newBLTouchController(rt *runtime, pwm *digitalOut, endstop *endstop) *bltouchController {
	return &bltouchController{
		rt:           rt,
		pwm:          pwm,
		endstop:      endstop,
		nextCmdTime:  bltMinCmdTimeSec + 0.200 + bltPinMoveTimeSec, // connect pin_up scheduling
		nextTestTime: 0.0,
		pinMoveTime:  bltPinMoveTimeSec,
		// Klippy defaults
		pinUpTouchTriggered: true,
		pinUpNotTriggered:   true,
		probeTouchMode:      false,
	}
}

func (b *bltouchController) syncPrintTime() error {
	if b == nil || b.rt == nil || b.rt.toolhead == nil {
		return nil
	}
	pt, err := b.rt.toolhead.getLastMoveTime()
	if err != nil {
		return err
	}
	if b.nextCmdTime > pt {
		return b.rt.toolhead.dwell(b.nextCmdTime - pt)
	}
	b.nextCmdTime = pt
	return nil
}

func (b *bltouchController) syncMCUPrintTime() error {
	// In Klippy, this uses an MCU-side estimated_print_time derived from the
	// reactor monotonic clock. In fileoutput golden mode we don't have an
	// accurate real-time estimate, and forcing nextCmdTime forward here causes
	// large clock skews. Keep it as a no-op and rely on syncPrintTime() when a
	// command must be synchronized with motion.
	return nil
}

func (b *bltouchController) sendCmd(cmd string, duration float64) error {
	if b == nil || b.pwm == nil {
		return fmt.Errorf("bltouch not initialized")
	}
	cmd = strings.ToLower(strings.TrimSpace(cmd))
	pulseSec, ok := bltCommandPulseSec[cmd]
	if !ok {
		return fmt.Errorf("unknown bltouch command %q", cmd)
	}

	// Match Klippy: convert nextCmdTime to an integer start clock via
	// print_time_to_clock (truncation). Note that Klippy's action_end_time is
	// based on the unrounded next_cmd_time (not the truncated start clock).
	startPrintTime := b.nextCmdTime
	startClk := uint64(int64(startPrintTime * b.pwm.mcuFreq))
	startTime := float64(startClk) / b.pwm.mcuFreq
	// Quantize the hold duration to the signal period.
	pulseHold := float64(int((duration-bltMinCmdTimeSec)/bltSignalPeriodSec)) * bltSignalPeriodSec
	holdSec := math.Max(bltMinCmdTimeSec, pulseHold)
	// Match Klippy: seconds_to_clock uses int() truncation.
	holdTicks := uint64(int64(holdSec * b.pwm.mcuFreq))
	endClk := startClk + holdTicks
	endTime := float64(endClk) / b.pwm.mcuFreq

	onTicks := uint32(int64(pulseSec*b.pwm.mcuFreq + 0.5))
	if err := b.pwm.setOnTicks(startTime, onTicks); err != nil {
		return err
	}
	if err := b.pwm.setOnTicks(endTime, 0); err != nil {
		return err
	}

	b.actionEndTime = startPrintTime + duration
	b.nextCmdTime = math.Max(b.actionEndTime, endTime+bltMinCmdTimeSec)
	return nil
}

func (b *bltouchController) raiseProbe() error {
	if b == nil {
		return fmt.Errorf("bltouch not initialized")
	}
	if err := b.syncMCUPrintTime(); err != nil {
		return err
	}
	if !b.pinUpNotTriggered {
		if err := b.sendCmd(bltCmdReset, bltRetryResetTimeSec); err != nil {
			return err
		}
	}
	return b.sendCmd(bltCmdPinUp, b.pinMoveTime)
}

func (b *bltouchController) verifyRaiseProbe() error {
	if b == nil {
		return fmt.Errorf("bltouch not initialized")
	}
	// If pin_up can't be verified, Klippy returns early.
	if !b.pinUpNotTriggered {
		return nil
	}
	// Klippy retries up to 3 times; in golden test mode, we assume success.
	return b.verifyState(false)
}

func (b *bltouchController) testSensor() error {
	if b == nil {
		return fmt.Errorf("bltouch not initialized")
	}
	if !b.pinUpTouchTriggered {
		return nil
	}
	// Klippy: if print_time < next_test_time, extend the window and return.
	if b.rt != nil && b.rt.toolhead != nil {
		pt, err := b.rt.toolhead.getLastMoveTime()
		if err != nil {
			return err
		}
		if pt < b.nextTestTime {
			b.nextTestTime = pt + bltTestTimeSec
			return nil
		}
	}
	// Raise the probe and test if probe is raised.
	if err := b.syncPrintTime(); err != nil {
		return err
	}
	// Klippy retries and may send reset pulses; we assume success on first try.
	if err := b.sendCmd(bltCmdPinUp, b.pinMoveTime); err != nil {
		return err
	}
	if err := b.sendCmd(bltCmdTouchMode, bltMinCmdTimeSec); err != nil {
		return err
	}
	if err := b.verifyState(true); err != nil {
		return err
	}
	if err := b.syncPrintTime(); err != nil {
		return err
	}
	if b.rt != nil && b.rt.toolhead != nil {
		pt, err := b.rt.toolhead.getLastMoveTime()
		if err != nil {
			return err
		}
		b.nextTestTime = pt + bltTestTimeSec
	}
	return nil
}

func (b *bltouchController) lowerProbe() error {
	if b == nil {
		return fmt.Errorf("bltouch not initialized")
	}
	if err := b.testSensor(); err != nil {
		return err
	}
	if err := b.syncPrintTime(); err != nil {
		return err
	}
	if err := b.sendCmd(bltCmdPinDown, b.pinMoveTime); err != nil {
		return err
	}
	if b.probeTouchMode {
		return b.sendCmd(bltCmdTouchMode, bltMinCmdTimeSec)
	}
	return nil
}

func (b *bltouchController) verifyState(triggered bool) error {
	if b == nil || b.endstop == nil {
		return fmt.Errorf("bltouch endstop not initialized")
	}
	// Use a precomputed integer rest_ticks to avoid float truncation jitter
	// (which can otherwise cause off-by-one diffs in golden output).
	restTicks := uint64(int64(bltEndstopRestTimeSec*b.endstop.mcuFreq + 0.5))
	if err := b.endstop.homeStartWithRestTicks(b.actionEndTime, restTicks, triggered); err != nil {
		return err
	}
	return b.endstop.homeStop()
}

func (r *runtime) cmdBLTouchDebug(args map[string]string) error {
	if r.bltouch == nil {
		// Match Klippy: if BLTouch isn't configured, command isn't registered.
		return fmt.Errorf("bltouch not configured")
	}
	cmd := strings.TrimSpace(args["COMMAND"])
	if cmd == "" {
		// Klippy prints available commands; no MCU output.
		return nil
	}
	cmd = strings.ToLower(cmd)
	if _, ok := bltCommandPulseSec[cmd]; !ok {
		return nil
	}
	if err := r.bltouch.syncPrintTime(); err != nil {
		return err
	}
	if err := r.bltouch.sendCmd(cmd, r.bltouch.pinMoveTime); err != nil {
		return err
	}
	return r.bltouch.syncPrintTime()
}
