// Package endstop provides endstop reading and event handling.
package endstop

import (
	"errors"
	"sync"
	"time"
)

// Common errors
var (
	ErrEndstopTimeout  = errors.New("endstop: timeout waiting for trigger")
	ErrEndstopTriggered = errors.New("endstop: endstop triggered unexpectedly")
	ErrNotHoming       = errors.New("endstop: not in homing state")
)

// EndstopState represents the current state of an endstop.
type EndstopState int

const (
	StateOpen EndstopState = iota
	StateTriggered
	StateUnknown
)

func (s EndstopState) String() string {
	switch s {
	case StateOpen:
		return "open"
	case StateTriggered:
		return "triggered"
	default:
		return "unknown"
	}
}

// Endstop represents a single endstop switch.
type Endstop struct {
	mu sync.RWMutex

	// Configuration
	name     string
	pin      string
	pullUp   bool
	inverted bool

	// State
	state         EndstopState
	lastTrigger   time.Time
	triggerClock  uint64
	debounceTime  time.Duration
	lastDebounce  time.Time

	// Homing state
	homing        bool
	homingDir     int // 1 or -1
	triggerChan   chan uint64

	// Callbacks
	onTrigger     func(clock uint64)
	queryState    func() (bool, error)
}

// EndstopConfig holds configuration for an endstop.
type EndstopConfig struct {
	Name         string
	Pin          string
	PullUp       bool
	Inverted     bool
	DebounceTime time.Duration
}

// DefaultEndstopConfig returns a default endstop configuration.
func DefaultEndstopConfig() EndstopConfig {
	return EndstopConfig{
		Name:         "endstop",
		PullUp:       true,
		Inverted:     false,
		DebounceTime: 1 * time.Millisecond,
	}
}

// New creates a new endstop.
func New(cfg EndstopConfig) *Endstop {
	return &Endstop{
		name:         cfg.Name,
		pin:          cfg.Pin,
		pullUp:       cfg.PullUp,
		inverted:     cfg.Inverted,
		state:        StateUnknown,
		debounceTime: cfg.DebounceTime,
		triggerChan:  make(chan uint64, 1),
	}
}

// SetQueryCallback sets the callback for querying endstop state.
func (e *Endstop) SetQueryCallback(fn func() (bool, error)) {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.queryState = fn
}

// SetTriggerCallback sets the callback for when the endstop triggers.
func (e *Endstop) SetTriggerCallback(fn func(clock uint64)) {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.onTrigger = fn
}

// HandleTrigger is called when the endstop triggers.
// This is typically called from an MCU event handler.
func (e *Endstop) HandleTrigger(clock uint64) {
	e.mu.Lock()

	// Check debounce
	now := time.Now()
	if now.Sub(e.lastDebounce) < e.debounceTime {
		e.mu.Unlock()
		return
	}
	e.lastDebounce = now

	e.state = StateTriggered
	e.lastTrigger = now
	e.triggerClock = clock

	homing := e.homing
	callback := e.onTrigger
	e.mu.Unlock()

	// Notify homing if active
	if homing {
		select {
		case e.triggerChan <- clock:
		default:
			// Channel full, trigger already pending
		}
	}

	// Call callback
	if callback != nil {
		callback(clock)
	}
}

// Query queries the current endstop state.
func (e *Endstop) Query() (EndstopState, error) {
	e.mu.RLock()
	query := e.queryState
	inverted := e.inverted
	e.mu.RUnlock()

	if query == nil {
		return StateUnknown, errors.New("endstop: no query callback set")
	}

	triggered, err := query()
	if err != nil {
		return StateUnknown, err
	}

	if inverted {
		triggered = !triggered
	}

	e.mu.Lock()
	if triggered {
		e.state = StateTriggered
	} else {
		e.state = StateOpen
	}
	state := e.state
	e.mu.Unlock()

	return state, nil
}

// GetState returns the last known state.
func (e *Endstop) GetState() EndstopState {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.state
}

// GetName returns the endstop name.
func (e *Endstop) GetName() string {
	return e.name
}

// GetPin returns the pin name.
func (e *Endstop) GetPin() string {
	return e.pin
}

// IsTriggered returns true if the endstop is currently triggered.
func (e *Endstop) IsTriggered() bool {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.state == StateTriggered
}

// StartHoming starts homing mode for this endstop.
func (e *Endstop) StartHoming(direction int) error {
	e.mu.Lock()
	defer e.mu.Unlock()

	e.homing = true
	e.homingDir = direction

	// Clear trigger channel
	select {
	case <-e.triggerChan:
	default:
	}

	return nil
}

// WaitForTrigger waits for the endstop to trigger during homing.
func (e *Endstop) WaitForTrigger(timeout time.Duration) (uint64, error) {
	e.mu.RLock()
	if !e.homing {
		e.mu.RUnlock()
		return 0, ErrNotHoming
	}
	triggerChan := e.triggerChan
	e.mu.RUnlock()

	select {
	case clock := <-triggerChan:
		return clock, nil
	case <-time.After(timeout):
		return 0, ErrEndstopTimeout
	}
}

// StopHoming stops homing mode.
func (e *Endstop) StopHoming() {
	e.mu.Lock()
	defer e.mu.Unlock()
	e.homing = false
}

// IsHoming returns true if homing is active.
func (e *Endstop) IsHoming() bool {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.homing
}

// GetLastTrigger returns the time and clock of the last trigger.
func (e *Endstop) GetLastTrigger() (time.Time, uint64) {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.lastTrigger, e.triggerClock
}

// Status holds endstop status information.
type Status struct {
	Name        string
	Pin         string
	State       string
	IsTriggered bool
	IsHoming    bool
	LastTrigger time.Time
}

// GetStatus returns the current endstop status.
func (e *Endstop) GetStatus() Status {
	e.mu.RLock()
	defer e.mu.RUnlock()

	return Status{
		Name:        e.name,
		Pin:         e.pin,
		State:       e.state.String(),
		IsTriggered: e.state == StateTriggered,
		IsHoming:    e.homing,
		LastTrigger: e.lastTrigger,
	}
}

// EndstopGroup manages multiple endstops for an axis.
type EndstopGroup struct {
	mu       sync.RWMutex
	name     string
	endstops []*Endstop
}

// NewEndstopGroup creates a new endstop group.
func NewEndstopGroup(name string) *EndstopGroup {
	return &EndstopGroup{
		name:     name,
		endstops: make([]*Endstop, 0),
	}
}

// Add adds an endstop to the group.
func (g *EndstopGroup) Add(e *Endstop) {
	g.mu.Lock()
	defer g.mu.Unlock()
	g.endstops = append(g.endstops, e)
}

// AnyTriggered returns true if any endstop in the group is triggered.
func (g *EndstopGroup) AnyTriggered() bool {
	g.mu.RLock()
	defer g.mu.RUnlock()

	for _, e := range g.endstops {
		if e.IsTriggered() {
			return true
		}
	}
	return false
}

// QueryAll queries all endstops and returns any that are triggered.
func (g *EndstopGroup) QueryAll() ([]*Endstop, error) {
	g.mu.RLock()
	endstops := make([]*Endstop, len(g.endstops))
	copy(endstops, g.endstops)
	g.mu.RUnlock()

	var triggered []*Endstop
	for _, e := range endstops {
		state, err := e.Query()
		if err != nil {
			return nil, err
		}
		if state == StateTriggered {
			triggered = append(triggered, e)
		}
	}
	return triggered, nil
}

// StartHomingAll starts homing on all endstops.
func (g *EndstopGroup) StartHomingAll(direction int) error {
	g.mu.RLock()
	defer g.mu.RUnlock()

	for _, e := range g.endstops {
		if err := e.StartHoming(direction); err != nil {
			return err
		}
	}
	return nil
}

// StopHomingAll stops homing on all endstops.
func (g *EndstopGroup) StopHomingAll() {
	g.mu.RLock()
	defer g.mu.RUnlock()

	for _, e := range g.endstops {
		e.StopHoming()
	}
}

// WaitForAnyTrigger waits for any endstop in the group to trigger.
func (g *EndstopGroup) WaitForAnyTrigger(timeout time.Duration) (*Endstop, uint64, error) {
	g.mu.RLock()
	endstops := make([]*Endstop, len(g.endstops))
	copy(endstops, g.endstops)
	g.mu.RUnlock()

	// Create a merged channel
	resultChan := make(chan struct {
		endstop *Endstop
		clock   uint64
	}, len(endstops))

	for _, e := range endstops {
		e := e
		go func() {
			clock, err := e.WaitForTrigger(timeout)
			if err == nil {
				select {
				case resultChan <- struct {
					endstop *Endstop
					clock   uint64
				}{e, clock}:
				default:
				}
			}
		}()
	}

	select {
	case result := <-resultChan:
		return result.endstop, result.clock, nil
	case <-time.After(timeout):
		return nil, 0, ErrEndstopTimeout
	}
}
