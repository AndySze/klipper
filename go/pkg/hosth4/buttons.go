// Buttons - port of klippy/extras/buttons.py
//
// Support for button detection and callbacks
//
// Copyright (C) 2018-2023 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"math"
	"sync"
	"time"
)

const (
	buttonQueryTime        = 0.002 // 2ms between button queries
	buttonRetransmitCount  = 50    // Number of retransmits before timeout
	adcReportTime          = 0.015 // 15ms between ADC reports
	adcDebounceTime        = 0.025 // 25ms debounce time
	adcSampleTime          = 0.001 // 1ms sample time
	adcSampleCount         = 6     // Number of samples to average
)

// ButtonCallback is called when a button state changes.
type ButtonCallback func(eventtime float64, state int)

// CWCallback is called on clockwise rotary encoder movement.
type CWCallback func(eventtime float64)

// CCWCallback is called on counter-clockwise rotary encoder movement.
type CCWCallback func(eventtime float64)

// PinParams holds pin configuration parameters.
type PinParams struct {
	Pin    string
	Invert bool
	Pullup bool
}

// MCUButtons handles button state tracking for an MCU.
type MCUButtons struct {
	rt *runtime

	pinList   []PinParams
	callbacks []buttonCallbackEntry
	invert    int
	lastButton int
	ackCount   int
	oid        int

	mu sync.Mutex
}

type buttonCallbackEntry struct {
	mask     int
	shift    int
	callback ButtonCallback
}

// newMCUButtons creates a new MCU buttons handler.
func newMCUButtons(rt *runtime) *MCUButtons {
	return &MCUButtons{
		rt: rt,
	}
}

// SetupButtons registers pins for button monitoring.
func (mb *MCUButtons) SetupButtons(pins []PinParams, callback ButtonCallback) {
	mb.mu.Lock()
	defer mb.mu.Unlock()

	mask := 0
	shift := len(mb.pinList)
	for _, pin := range pins {
		if pin.Invert {
			mb.invert |= 1 << len(mb.pinList)
		}
		mask |= 1 << len(mb.pinList)
		mb.pinList = append(mb.pinList, pin)
	}
	mb.callbacks = append(mb.callbacks, buttonCallbackEntry{
		mask:     mask,
		shift:    shift,
		callback: callback,
	})
}

// HandleButtonsState processes button state updates from MCU.
func (mb *MCUButtons) HandleButtonsState(receiveTime float64, ackCount int, state []byte) {
	mb.mu.Lock()
	defer mb.mu.Unlock()

	// Expand ack_count from 8-bit
	ackDiff := (ackCount - mb.ackCount) & 0xff
	ackDiff -= (ackDiff & 0x80) << 1
	msgAckCount := mb.ackCount + ackDiff

	// Determine new buttons
	newCount := msgAckCount + len(state) - mb.ackCount
	if newCount <= 0 {
		return
	}
	newButtons := state[len(state)-newCount:]

	// Update ack count
	mb.ackCount += newCount

	// Process button events
	for _, button := range newButtons {
		btnState := int(button) ^ mb.invert
		changed := btnState ^ mb.lastButton
		mb.lastButton = btnState

		for _, entry := range mb.callbacks {
			if changed&entry.mask != 0 {
				state := (btnState & entry.mask) >> entry.shift
				entry.callback(receiveTime, state)
			}
		}
	}
}

// MCUADCButtons handles ADC-based button detection.
type MCUADCButtons struct {
	rt *runtime

	buttons         []adcButtonEntry
	lastButton      *int
	lastPressed     *int
	lastDebounceTime float64
	pullup          float64
	pin             string
	minValue        float64
	maxValue        float64

	mu sync.Mutex
}

type adcButtonEntry struct {
	minValue float64
	maxValue float64
	callback ButtonCallback
}

// newMCUADCButtons creates a new ADC buttons handler.
func newMCUADCButtons(rt *runtime, pin string, pullup float64) *MCUADCButtons {
	return &MCUADCButtons{
		rt:       rt,
		pin:      pin,
		pullup:   pullup,
		minValue: 999999999999.9,
		maxValue: 0,
	}
}

// SetupButton registers an ADC button.
func (ab *MCUADCButtons) SetupButton(minValue, maxValue float64, callback ButtonCallback) {
	ab.mu.Lock()
	defer ab.mu.Unlock()

	if minValue < ab.minValue {
		ab.minValue = minValue
	}
	if maxValue > ab.maxValue {
		ab.maxValue = maxValue
	}
	ab.buttons = append(ab.buttons, adcButtonEntry{
		minValue: minValue,
		maxValue: maxValue,
		callback: callback,
	})
}

// ADCCallback is called when ADC value is read.
func (ab *MCUADCButtons) ADCCallback(readTime, readValue float64) {
	ab.mu.Lock()
	defer ab.mu.Unlock()

	// Clamp value
	adc := math.Max(0.00001, math.Min(0.99999, readValue))
	value := ab.pullup * adc / (1.0 - adc)

	// Determine button pressed
	var btn *int
	if value >= ab.minValue && value <= ab.maxValue {
		for i, entry := range ab.buttons {
			if value > entry.minValue && value < entry.maxValue {
				idx := i
				btn = &idx
				break
			}
		}
	}

	// If button changed, reset debounce timer
	if (btn == nil) != (ab.lastButton == nil) ||
		(btn != nil && ab.lastButton != nil && *btn != *ab.lastButton) {
		ab.lastDebounceTime = readTime
	}

	// Debounce check
	if (readTime-ab.lastDebounceTime) >= adcDebounceTime &&
		((btn == nil && ab.lastButton == nil) ||
			(btn != nil && ab.lastButton != nil && *btn == *ab.lastButton)) {

		// Check if different from last pressed
		pressedDiff := (ab.lastPressed == nil) != (btn == nil) ||
			(ab.lastPressed != nil && btn != nil && *ab.lastPressed != *btn)

		if pressedDiff {
			// Release last pressed
			if ab.lastPressed != nil {
				ab.callButton(*ab.lastPressed, 0, readTime)
				ab.lastPressed = nil
			}
			// Press new button
			if btn != nil {
				ab.callButton(*btn, 1, readTime)
				idx := *btn
				ab.lastPressed = &idx
			}
		}
	}

	if btn != nil {
		idx := *btn
		ab.lastButton = &idx
	} else {
		ab.lastButton = nil
	}
}

func (ab *MCUADCButtons) callButton(button, state int, eventtime float64) {
	if button >= 0 && button < len(ab.buttons) {
		ab.buttons[button].callback(eventtime, state)
	}
}

// Rotary encoder direction constants
const (
	encoderRStart  = 0x0
	encoderRDirCW  = 0x10
	encoderRDirCCW = 0x20
	encoderRDirMsk = 0x30
)

// BaseRotaryEncoder handles rotary encoder state machine.
type BaseRotaryEncoder struct {
	cwCallback  CWCallback
	ccwCallback CCWCallback
	state       int
	stateTable  [][]int
}

// FullStepRotaryEncoder implements full-step rotary encoder.
type FullStepRotaryEncoder struct {
	BaseRotaryEncoder
}

// Full step state constants
const (
	rCWFinal  = 0x1
	rCWBegin  = 0x2
	rCWNext   = 0x3
	rCCWBegin = 0x4
	rCCWFinal = 0x5
	rCCWNext  = 0x6
)

// NewFullStepRotaryEncoder creates a full step rotary encoder.
func NewFullStepRotaryEncoder(cwCallback CWCallback, ccwCallback CCWCallback) *FullStepRotaryEncoder {
	re := &FullStepRotaryEncoder{}
	re.cwCallback = cwCallback
	re.ccwCallback = ccwCallback
	re.state = encoderRStart
	re.stateTable = [][]int{
		// R_START
		{encoderRStart, rCWBegin, rCCWBegin, encoderRStart},
		// R_CW_FINAL
		{rCWNext, encoderRStart, rCWFinal, encoderRStart | encoderRDirCW},
		// R_CW_BEGIN
		{rCWNext, rCWBegin, encoderRStart, encoderRStart},
		// R_CW_NEXT
		{rCWNext, rCWBegin, rCWFinal, encoderRStart},
		// R_CCW_BEGIN
		{rCCWNext, encoderRStart, rCCWBegin, encoderRStart},
		// R_CCW_FINAL
		{rCCWNext, rCCWFinal, encoderRStart, encoderRStart | encoderRDirCCW},
		// R_CCW_NEXT
		{rCCWNext, rCCWFinal, rCCWBegin, encoderRStart},
	}
	return re
}

// EncoderCallback processes encoder state changes.
func (re *FullStepRotaryEncoder) EncoderCallback(eventtime float64, state int) {
	es := re.stateTable[re.state&0xf][state&0x3]
	re.state = es
	if es&encoderRDirMsk == encoderRDirCW {
		if re.cwCallback != nil {
			re.cwCallback(eventtime)
		}
	} else if es&encoderRDirMsk == encoderRDirCCW {
		if re.ccwCallback != nil {
			re.ccwCallback(eventtime)
		}
	}
}

// HalfStepRotaryEncoder implements half-step rotary encoder.
type HalfStepRotaryEncoder struct {
	BaseRotaryEncoder
}

// Half step state constants
const (
	hsRCCWBegin  = 0x1
	hsRCWBegin   = 0x2
	hsRStartM    = 0x3
	hsRCWBeginM  = 0x4
	hsRCCWBeginM = 0x5
)

// NewHalfStepRotaryEncoder creates a half step rotary encoder.
func NewHalfStepRotaryEncoder(cwCallback CWCallback, ccwCallback CCWCallback) *HalfStepRotaryEncoder {
	re := &HalfStepRotaryEncoder{}
	re.cwCallback = cwCallback
	re.ccwCallback = ccwCallback
	re.state = encoderRStart
	re.stateTable = [][]int{
		// R_START (00)
		{hsRStartM, hsRCWBegin, hsRCCWBegin, encoderRStart},
		// R_CCW_BEGIN
		{hsRStartM | encoderRDirCCW, encoderRStart, hsRCCWBegin, encoderRStart},
		// R_CW_BEGIN
		{hsRStartM | encoderRDirCW, hsRCWBegin, encoderRStart, encoderRStart},
		// R_START_M (11)
		{hsRStartM, hsRCCWBeginM, hsRCWBeginM, encoderRStart},
		// R_CW_BEGIN_M
		{hsRStartM, hsRStartM, hsRCWBeginM, encoderRStart | encoderRDirCW},
		// R_CCW_BEGIN_M
		{hsRStartM, hsRCCWBeginM, hsRStartM, encoderRStart | encoderRDirCCW},
	}
	return re
}

// EncoderCallback processes encoder state changes.
func (re *HalfStepRotaryEncoder) EncoderCallback(eventtime float64, state int) {
	es := re.stateTable[re.state&0xf][state&0x3]
	re.state = es
	if es&encoderRDirMsk == encoderRDirCW {
		if re.cwCallback != nil {
			re.cwCallback(eventtime)
		}
	} else if es&encoderRDirMsk == encoderRDirCCW {
		if re.ccwCallback != nil {
			re.ccwCallback(eventtime)
		}
	}
}

// DebounceButton adds debouncing to button events.
type DebounceButton struct {
	buttonAction    ButtonCallback
	debounceDelay   time.Duration
	logicalState    *int
	physicalState   *int
	latestEventtime float64

	mu sync.Mutex
}

// NewDebounceButton creates a debounced button handler.
func NewDebounceButton(buttonAction ButtonCallback, debounceDelaySec float64) *DebounceButton {
	return &DebounceButton{
		buttonAction:  buttonAction,
		debounceDelay: time.Duration(debounceDelaySec * float64(time.Second)),
	}
}

// ButtonHandler handles raw button events.
func (db *DebounceButton) ButtonHandler(eventtime float64, state int) {
	db.mu.Lock()
	defer db.mu.Unlock()

	db.physicalState = &state
	db.latestEventtime = eventtime

	// If no state transition, ignore
	if db.logicalState != nil && *db.logicalState == state {
		return
	}

	// Schedule debounce check
	go db.debounceEvent(eventtime + float64(db.debounceDelay)/float64(time.Second))
}

func (db *DebounceButton) debounceEvent(triggerTime float64) {
	// Wait for debounce delay
	time.Sleep(db.debounceDelay)

	db.mu.Lock()
	defer db.mu.Unlock()

	// If no state transition, ignore
	if db.logicalState != nil && db.physicalState != nil && *db.logicalState == *db.physicalState {
		return
	}

	// If newer events supersede, ignore
	if (triggerTime - float64(db.debounceDelay)/float64(time.Second)) < db.latestEventtime {
		return
	}

	// Enact state transition
	if db.physicalState != nil {
		state := *db.physicalState
		db.logicalState = &state
		db.buttonAction(db.latestEventtime, state)
	}
}

// PrinterButtons manages all printer button registrations.
type PrinterButtons struct {
	rt *runtime

	mcuButtons map[string]*MCUButtons
	adcButtons map[string]*MCUADCButtons

	mu sync.RWMutex
}

// newPrinterButtons creates a new printer buttons manager.
func newPrinterButtons(rt *runtime) *PrinterButtons {
	return &PrinterButtons{
		rt:         rt,
		mcuButtons: make(map[string]*MCUButtons),
		adcButtons: make(map[string]*MCUADCButtons),
	}
}

// RegisterADCButton registers an ADC-based button.
func (pb *PrinterButtons) RegisterADCButton(pin string, minVal, maxVal, pullup float64, callback ButtonCallback) {
	pb.mu.Lock()
	defer pb.mu.Unlock()

	adcButtons, ok := pb.adcButtons[pin]
	if !ok {
		adcButtons = newMCUADCButtons(pb.rt, pin, pullup)
		pb.adcButtons[pin] = adcButtons
	}
	adcButtons.SetupButton(minVal, maxVal, callback)
}

// RegisterDebounceButton registers a button with debouncing.
func (pb *PrinterButtons) RegisterDebounceButton(pin string, callback ButtonCallback, debounceDelay float64) {
	debounce := NewDebounceButton(callback, debounceDelay)
	pb.RegisterButtons([]PinParams{{Pin: pin}}, debounce.ButtonHandler)
}

// RegisterDebounceADCButton registers an ADC button with debouncing.
func (pb *PrinterButtons) RegisterDebounceADCButton(pin string, minVal, maxVal, pullup float64, callback ButtonCallback, debounceDelay float64) {
	debounce := NewDebounceButton(callback, debounceDelay)
	pb.RegisterADCButton(pin, minVal, maxVal, pullup, debounce.ButtonHandler)
}

// RegisterADCButtonPush registers an ADC button that fires on push only.
func (pb *PrinterButtons) RegisterADCButtonPush(pin string, minVal, maxVal, pullup float64, callback CWCallback) {
	helper := func(eventtime float64, state int) {
		if state != 0 {
			callback(eventtime)
		}
	}
	pb.RegisterADCButton(pin, minVal, maxVal, pullup, helper)
}

// RegisterButtons registers digital buttons.
func (pb *PrinterButtons) RegisterButtons(pins []PinParams, callback ButtonCallback) error {
	pb.mu.Lock()
	defer pb.mu.Unlock()

	if len(pins) == 0 {
		return fmt.Errorf("no pins specified")
	}

	// For now, use a default MCU name
	mcuName := "mcu"

	mcuButtons, ok := pb.mcuButtons[mcuName]
	if !ok || len(mcuButtons.pinList)+len(pins) > 8 {
		mcuButtons = newMCUButtons(pb.rt)
		pb.mcuButtons[mcuName] = mcuButtons
	}
	mcuButtons.SetupButtons(pins, callback)
	return nil
}

// RegisterRotaryEncoder registers a rotary encoder.
func (pb *PrinterButtons) RegisterRotaryEncoder(pin1, pin2 string, cwCallback CWCallback, ccwCallback CCWCallback, stepsPerDetent int) error {
	var encoderCallback ButtonCallback

	if stepsPerDetent == 2 {
		re := NewHalfStepRotaryEncoder(cwCallback, ccwCallback)
		encoderCallback = re.EncoderCallback
	} else if stepsPerDetent == 4 {
		re := NewFullStepRotaryEncoder(cwCallback, ccwCallback)
		encoderCallback = re.EncoderCallback
	} else {
		return fmt.Errorf("%d steps per detent not supported", stepsPerDetent)
	}

	return pb.RegisterButtons([]PinParams{{Pin: pin1}, {Pin: pin2}}, encoderCallback)
}

// RegisterButtonPush registers a button that fires on push only.
func (pb *PrinterButtons) RegisterButtonPush(pin string, callback CWCallback) error {
	helper := func(eventtime float64, state int) {
		if state != 0 {
			callback(eventtime)
		}
	}
	return pb.RegisterButtons([]PinParams{{Pin: pin}}, helper)
}

// GetStatus returns the status of all buttons.
func (pb *PrinterButtons) GetStatus() map[string]any {
	pb.mu.RLock()
	defer pb.mu.RUnlock()

	return map[string]any{
		"mcu_button_count": len(pb.mcuButtons),
		"adc_button_count": len(pb.adcButtons),
	}
}
