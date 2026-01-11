// Package reactor provides an event-driven async execution framework.
// This is the Go equivalent of klippy/reactor.py, using goroutines and channels
// instead of greenlets for cooperative multitasking.
package reactor

import (
	"context"
	"errors"
	"sync"
	"sync/atomic"
	"time"
)

// Constants
const (
	NOW   = 0.0
	NEVER = 9999999999999999.0
)

// Common errors
var (
	ErrReactorClosed = errors.New("reactor: reactor closed")
	ErrTimeout       = errors.New("reactor: operation timed out")
)

// TimerCallback is called when a timer fires.
// The callback receives the event time and returns the next wake time.
// Return NEVER to unregister the timer.
type TimerCallback func(eventtime float64) float64

// Timer represents a registered timer.
type Timer struct {
	id        uint64
	callback  TimerCallback
	waketime  float64
	isRunning bool
	mu        sync.Mutex
}

// Waketime returns the timer's current wake time.
func (t *Timer) Waketime() float64 {
	t.mu.Lock()
	defer t.mu.Unlock()
	return t.waketime
}

// Completion represents an async operation that will complete with a result.
type Completion struct {
	reactor *Reactor
	result  interface{}
	done    chan struct{}
	once    sync.Once
}

// Test returns true if the completion has a result.
func (c *Completion) Test() bool {
	select {
	case <-c.done:
		return true
	default:
		return false
	}
}

// Complete sets the completion result and wakes any waiters.
func (c *Completion) Complete(result interface{}) {
	c.once.Do(func() {
		c.result = result
		close(c.done)
	})
}

// Wait blocks until the completion is done or the timeout expires.
// Returns the result or timeoutResult if the timeout expires.
func (c *Completion) Wait(timeout time.Duration, timeoutResult interface{}) interface{} {
	select {
	case <-c.done:
		return c.result
	case <-time.After(timeout):
		return timeoutResult
	case <-c.reactor.ctx.Done():
		return timeoutResult
	}
}

// WaitUntil blocks until the completion is done or the waketime is reached.
func (c *Completion) WaitUntil(waketime float64, waketimeResult interface{}) interface{} {
	if waketime >= NEVER {
		select {
		case <-c.done:
			return c.result
		case <-c.reactor.ctx.Done():
			return waketimeResult
		}
	}

	now := c.reactor.Monotonic()
	if waketime <= now {
		select {
		case <-c.done:
			return c.result
		default:
			return waketimeResult
		}
	}

	timeout := time.Duration((waketime - now) * float64(time.Second))
	return c.Wait(timeout, waketimeResult)
}

// Callback wraps a function to be executed asynchronously.
type Callback struct {
	timer      *Timer
	callback   func(eventtime float64) interface{}
	completion *Completion
}

// Mutex provides a reactor-aware mutex for coordinating goroutines.
type Mutex struct {
	mu       sync.Mutex
	isLocked bool
	waiters  []chan struct{}
}

// Lock acquires the mutex.
func (m *Mutex) Lock() {
	m.mu.Lock()
	if !m.isLocked {
		m.isLocked = true
		m.mu.Unlock()
		return
	}

	// Wait for unlock
	ch := make(chan struct{})
	m.waiters = append(m.waiters, ch)
	m.mu.Unlock()

	<-ch
}

// Unlock releases the mutex.
func (m *Mutex) Unlock() {
	m.mu.Lock()
	defer m.mu.Unlock()

	if len(m.waiters) > 0 {
		// Wake the first waiter
		ch := m.waiters[0]
		m.waiters = m.waiters[1:]
		close(ch)
	} else {
		m.isLocked = false
	}
}

// Test returns true if the mutex is currently locked.
func (m *Mutex) Test() bool {
	m.mu.Lock()
	defer m.mu.Unlock()
	return m.isLocked
}

// Reactor manages timers, callbacks, and event dispatch.
type Reactor struct {
	mu          sync.RWMutex
	timers      []*Timer
	nextTimerID uint64
	nextWake    float64

	// Async callback queue
	asyncQueue chan func()

	// Context for shutdown
	ctx    context.Context
	cancel context.CancelFunc

	// Running state
	running atomic.Bool
	wg      sync.WaitGroup

	// Start time for monotonic clock
	startTime time.Time
}

// New creates a new Reactor.
func New() *Reactor {
	ctx, cancel := context.WithCancel(context.Background())
	return &Reactor{
		timers:     make([]*Timer, 0),
		nextWake:   NEVER,
		asyncQueue: make(chan func(), 1000),
		ctx:        ctx,
		cancel:     cancel,
		startTime:  time.Now(),
	}
}

// Monotonic returns the current monotonic time in seconds.
func (r *Reactor) Monotonic() float64 {
	return time.Since(r.startTime).Seconds()
}

// RegisterTimer registers a new timer with the given callback and wake time.
func (r *Reactor) RegisterTimer(callback TimerCallback, waketime float64) *Timer {
	r.mu.Lock()
	defer r.mu.Unlock()

	timer := &Timer{
		id:       atomic.AddUint64(&r.nextTimerID, 1),
		callback: callback,
		waketime: waketime,
	}

	r.timers = append(r.timers, timer)
	if waketime < r.nextWake {
		r.nextWake = waketime
	}

	return timer
}

// UnregisterTimer removes a timer.
func (r *Reactor) UnregisterTimer(timer *Timer) {
	r.mu.Lock()
	defer r.mu.Unlock()

	timer.mu.Lock()
	timer.waketime = NEVER
	timer.mu.Unlock()

	// Remove from list
	for i, t := range r.timers {
		if t.id == timer.id {
			r.timers = append(r.timers[:i], r.timers[i+1:]...)
			break
		}
	}
}

// UpdateTimer updates a timer's wake time.
func (r *Reactor) UpdateTimer(timer *Timer, waketime float64) {
	timer.mu.Lock()
	if timer.isRunning {
		timer.mu.Unlock()
		return
	}
	timer.waketime = waketime
	timer.mu.Unlock()

	r.mu.Lock()
	if waketime < r.nextWake {
		r.nextWake = waketime
	}
	r.mu.Unlock()
}

// Completion creates a new Completion object.
func (r *Reactor) Completion() *Completion {
	return &Completion{
		reactor: r,
		done:    make(chan struct{}),
	}
}

// RegisterCallback schedules a callback to run at the given time.
// Returns a Completion that will contain the callback's result.
func (r *Reactor) RegisterCallback(callback func(eventtime float64) interface{}, waketime float64) *Completion {
	completion := r.Completion()

	cb := &Callback{
		callback:   callback,
		completion: completion,
	}

	timerCallback := func(eventtime float64) float64 {
		result := cb.callback(eventtime)
		cb.completion.Complete(result)
		return NEVER
	}

	cb.timer = r.RegisterTimer(timerCallback, waketime)
	return completion
}

// RegisterAsyncCallback schedules a callback from another goroutine.
func (r *Reactor) RegisterAsyncCallback(callback func(eventtime float64) interface{}, waketime float64) *Completion {
	completion := r.Completion()

	select {
	case r.asyncQueue <- func() {
		r.RegisterCallback(func(eventtime float64) interface{} {
			result := callback(eventtime)
			completion.Complete(result)
			return result
		}, waketime)
	}:
	default:
		// Queue full, complete with nil
		completion.Complete(nil)
	}

	return completion
}

// AsyncComplete completes a Completion from another goroutine.
func (r *Reactor) AsyncComplete(completion *Completion, result interface{}) {
	select {
	case r.asyncQueue <- func() {
		completion.Complete(result)
	}:
	default:
		// Queue full, try direct complete
		completion.Complete(result)
	}
}

// Mutex creates a new reactor-aware mutex.
func (r *Reactor) NewMutex(isLocked bool) *Mutex {
	return &Mutex{
		isLocked: isLocked,
	}
}

// Pause sleeps until the given wake time.
func (r *Reactor) Pause(waketime float64) float64 {
	now := r.Monotonic()
	if waketime <= now {
		return now
	}

	if waketime >= NEVER {
		<-r.ctx.Done()
		return r.Monotonic()
	}

	delay := time.Duration((waketime - now) * float64(time.Second))
	select {
	case <-time.After(delay):
	case <-r.ctx.Done():
	}
	return r.Monotonic()
}

// Run starts the reactor's main dispatch loop.
func (r *Reactor) Run() {
	if r.running.Swap(true) {
		return // Already running
	}

	r.wg.Add(1)
	go r.dispatchLoop()
}

// End signals the reactor to stop.
func (r *Reactor) End() {
	r.running.Store(false)
	r.cancel()
}

// Wait waits for the reactor to stop.
func (r *Reactor) Wait() {
	r.wg.Wait()
}

// dispatchLoop is the main event dispatch loop.
func (r *Reactor) dispatchLoop() {
	defer r.wg.Done()

	for r.running.Load() {
		eventtime := r.Monotonic()

		// Process async callbacks
		r.processAsyncCallbacks()

		// Check and fire timers
		timeout := r.checkTimers(eventtime)

		// Sleep until next timer or async callback
		if timeout > 0 {
			delay := time.Duration(timeout * float64(time.Second))
			if delay > time.Second {
				delay = time.Second
			}

			select {
			case <-time.After(delay):
			case <-r.asyncQueue:
				// New async callback, continue loop
			case <-r.ctx.Done():
				return
			}
		}
	}
}

// processAsyncCallbacks processes pending async callbacks.
func (r *Reactor) processAsyncCallbacks() {
	for {
		select {
		case fn := <-r.asyncQueue:
			fn()
		default:
			return
		}
	}
}

// checkTimers checks and fires due timers.
// Returns the time until the next timer fires.
func (r *Reactor) checkTimers(eventtime float64) float64 {
	r.mu.Lock()
	if eventtime < r.nextWake {
		delay := r.nextWake - eventtime
		r.mu.Unlock()
		return delay
	}

	// Make a copy of timers to iterate
	timers := make([]*Timer, len(r.timers))
	copy(timers, r.timers)
	r.mu.Unlock()

	r.nextWake = NEVER

	for _, timer := range timers {
		timer.mu.Lock()
		waketime := timer.waketime
		if eventtime >= waketime {
			timer.waketime = NEVER
			timer.isRunning = true
			timer.mu.Unlock()

			// Call the callback
			newWaketime := timer.callback(eventtime)

			timer.mu.Lock()
			timer.isRunning = false
			if newWaketime < timer.waketime {
				timer.waketime = newWaketime
			}
		}
		waketime = timer.waketime
		timer.mu.Unlock()

		r.mu.Lock()
		if waketime < r.nextWake {
			r.nextWake = waketime
		}
		r.mu.Unlock()
	}

	r.mu.RLock()
	delay := r.nextWake - eventtime
	r.mu.RUnlock()

	if delay < 0 {
		delay = 0
	}
	return delay
}
