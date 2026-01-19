// Motion Report - port of klippy/extras/motion_report.py
//
// Diagnostic tool for reporting stepper and kinematic positions
//
// Copyright (C) 2021 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

const (
	neverTime = 9999999999999999.0
)

// StepData represents a single step queue entry.
type StepData struct {
	FirstClock    uint64
	LastClock     uint64
	StartPosition int64
	Interval      uint32
	StepCount     uint32
	Add           int32
}

// DumpStepper extracts stepper queue_step messages.
type DumpStepper struct {
	rt             *runtime
	stepperName    string
	lastBatchClock uint64
	mu             sync.Mutex
}

// newDumpStepper creates a new stepper dumper.
func newDumpStepper(rt *runtime, stepperName string) *DumpStepper {
	return &DumpStepper{
		rt:          rt,
		stepperName: stepperName,
	}
}

// GetStepQueue retrieves step queue entries in a time range.
func (ds *DumpStepper) GetStepQueue(startClock, endClock uint64) ([]StepData, error) {
	ds.mu.Lock()
	defer ds.mu.Unlock()

	// In a real implementation, this would query the MCU for step data
	// For now, return empty
	return []StepData{}, nil
}

// LogSteps logs step queue entries.
func (ds *DumpStepper) LogSteps(data []StepData) {
	if len(data) == 0 {
		return
	}

	log.Printf("Dumping stepper '%s' %d queue_step:", ds.stepperName, len(data))
	for i, s := range data {
		log.Printf("queue_step %d: t=%d p=%d i=%d c=%d a=%d",
			i, s.FirstClock, s.StartPosition, s.Interval, s.StepCount, s.Add)
	}
}

// ProcessBatch processes a batch of step data.
func (ds *DumpStepper) ProcessBatch() map[string]any {
	data, err := ds.GetStepQueue(ds.lastBatchClock, 1<<63)
	if err != nil || len(data) == 0 {
		return map[string]any{}
	}

	// Find end block
	for i, d := range data {
		if d.StepCount == 0 {
			data = data[:i+1]
			break
		}
	}

	if len(data) == 0 {
		return map[string]any{}
	}

	first := data[0]
	last := data[len(data)-1]
	ds.lastBatchClock = last.LastClock

	tdata := make([][3]int64, len(data))
	for i, s := range data {
		tdata[i] = [3]int64{int64(s.Interval), int64(s.StepCount), int64(s.Add)}
	}

	return map[string]any{
		"data":               tdata,
		"start_mcu_position": first.StartPosition,
		"first_clock":        first.FirstClock,
		"last_clock":         last.LastClock,
	}
}

// TrapQMove represents a trapezoidal motion queue entry.
type TrapQMove struct {
	PrintTime     float64
	Duration      float64
	StartVelocity float64
	Acceleration  float64
	StartPosition [3]float64
	Direction     [3]float64
}

// DumpTrapQ extracts trapezoidal motion queue entries.
type DumpTrapQ struct {
	rt           *runtime
	name         string
	lastBatchMsg struct {
		time     float64
		duration float64
	}
	mu sync.Mutex
}

// newDumpTrapQ creates a new trapq dumper.
func newDumpTrapQ(rt *runtime, name string) *DumpTrapQ {
	return &DumpTrapQ{
		rt:   rt,
		name: name,
	}
}

// ExtractTrapQ extracts trapq entries in a time range.
func (dt *DumpTrapQ) ExtractTrapQ(startTime, endTime float64) ([]TrapQMove, error) {
	dt.mu.Lock()
	defer dt.mu.Unlock()

	// In a real implementation, this would query the trapq
	// For now, return empty
	return []TrapQMove{}, nil
}

// LogMoves logs trapq entries.
func (dt *DumpTrapQ) LogMoves(moves []TrapQMove) {
	if len(moves) == 0 {
		return
	}

	log.Printf("Dumping trapq '%s' %d moves:", dt.name, len(moves))
	for i, m := range moves {
		log.Printf("move %d: t=%.6f d=%.6f sv=%.3f accel=%.3f pos=(%.3f,%.3f,%.3f) dir=(%.3f,%.3f,%.3f)",
			i, m.PrintTime, m.Duration, m.StartVelocity, m.Acceleration,
			m.StartPosition[0], m.StartPosition[1], m.StartPosition[2],
			m.Direction[0], m.Direction[1], m.Direction[2])
	}
}

// ProcessBatch processes a batch of trapq data.
func (dt *DumpTrapQ) ProcessBatch() map[string]any {
	dt.mu.Lock()
	lastMsg := dt.lastBatchMsg
	dt.mu.Unlock()

	moves, err := dt.ExtractTrapQ(lastMsg.time+lastMsg.duration, neverTime)
	if err != nil || len(moves) == 0 {
		return map[string]any{}
	}

	first := moves[0]
	last := moves[len(moves)-1]

	dt.mu.Lock()
	dt.lastBatchMsg.time = last.PrintTime
	dt.lastBatchMsg.duration = last.Duration
	dt.mu.Unlock()

	data := make([][6]float64, len(moves))
	for i, m := range moves {
		data[i] = [6]float64{
			m.PrintTime, m.Duration, m.StartVelocity, m.Acceleration,
			m.StartPosition[0], m.Direction[0],
		}
	}

	return map[string]any{
		"data":       data,
		"first_time": first.PrintTime,
		"last_time":  last.PrintTime + last.Duration,
	}
}

// MotionReport provides motion diagnostic reporting.
type MotionReport struct {
	rt           *runtime
	dumpSteppers map[string]*DumpStepper
	dumpTrapqs   map[string]*DumpTrapQ
	mu           sync.RWMutex
}

// newMotionReport creates a new motion report handler.
func newMotionReport(rt *runtime) *MotionReport {
	mr := &MotionReport{
		rt:           rt,
		dumpSteppers: make(map[string]*DumpStepper),
		dumpTrapqs:   make(map[string]*DumpTrapQ),
	}

	log.Printf("motion_report: initialized")
	return mr
}

// AddStepper adds a stepper for dumping.
func (mr *MotionReport) AddStepper(name string) {
	mr.mu.Lock()
	defer mr.mu.Unlock()

	if _, exists := mr.dumpSteppers[name]; !exists {
		mr.dumpSteppers[name] = newDumpStepper(mr.rt, name)
	}
}

// AddTrapQ adds a trapq for dumping.
func (mr *MotionReport) AddTrapQ(name string) {
	mr.mu.Lock()
	defer mr.mu.Unlock()

	if _, exists := mr.dumpTrapqs[name]; !exists {
		mr.dumpTrapqs[name] = newDumpTrapQ(mr.rt, name)
	}
}

// GetDumpStepper returns a stepper dumper by name.
func (mr *MotionReport) GetDumpStepper(name string) *DumpStepper {
	mr.mu.RLock()
	defer mr.mu.RUnlock()
	return mr.dumpSteppers[name]
}

// GetDumpTrapQ returns a trapq dumper by name.
func (mr *MotionReport) GetDumpTrapQ(name string) *DumpTrapQ {
	mr.mu.RLock()
	defer mr.mu.RUnlock()
	return mr.dumpTrapqs[name]
}

// GetStatus returns the motion report status.
func (mr *MotionReport) GetStatus() map[string]any {
	mr.mu.RLock()
	defer mr.mu.RUnlock()

	stepperNames := make([]string, 0, len(mr.dumpSteppers))
	for name := range mr.dumpSteppers {
		stepperNames = append(stepperNames, name)
	}

	trapqNames := make([]string, 0, len(mr.dumpTrapqs))
	for name := range mr.dumpTrapqs {
		trapqNames = append(trapqNames, name)
	}

	return map[string]any{
		"steppers": stepperNames,
		"trapqs":   trapqNames,
	}
}

// DumpStepperStatus returns stepper status for a specific stepper.
func (mr *MotionReport) DumpStepperStatus(name string) (map[string]any, error) {
	ds := mr.GetDumpStepper(name)
	if ds == nil {
		return nil, fmt.Errorf("unknown stepper: %s", name)
	}
	return ds.ProcessBatch(), nil
}

// DumpTrapQStatus returns trapq status for a specific trapq.
func (mr *MotionReport) DumpTrapQStatus(name string) (map[string]any, error) {
	dt := mr.GetDumpTrapQ(name)
	if dt == nil {
		return nil, fmt.Errorf("unknown trapq: %s", name)
	}
	return dt.ProcessBatch(), nil
}
