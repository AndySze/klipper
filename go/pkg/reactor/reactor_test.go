package reactor

import (
	"sync/atomic"
	"testing"
	"time"
)

func TestNew(t *testing.T) {
	r := New()
	if r == nil {
		t.Fatal("New() returned nil")
	}
	defer r.End()
}

func TestMonotonic(t *testing.T) {
	r := New()
	defer r.End()

	t1 := r.Monotonic()
	time.Sleep(10 * time.Millisecond)
	t2 := r.Monotonic()

	if t2 <= t1 {
		t.Errorf("Monotonic time not increasing: %f <= %f", t2, t1)
	}

	elapsed := t2 - t1
	if elapsed < 0.009 || elapsed > 0.050 {
		t.Errorf("Unexpected elapsed time: %f (expected ~0.01)", elapsed)
	}
}

func TestTimer(t *testing.T) {
	r := New()

	var called atomic.Int32
	callback := func(eventtime float64) float64 {
		called.Add(1)
		return NEVER
	}

	// Register timer to fire immediately
	timer := r.RegisterTimer(callback, NOW)
	if timer == nil {
		t.Fatal("RegisterTimer returned nil")
	}

	// Run reactor briefly
	r.Run()
	time.Sleep(50 * time.Millisecond)
	r.End()
	r.Wait()

	if called.Load() != 1 {
		t.Errorf("Timer callback called %d times, expected 1", called.Load())
	}
}

func TestTimerRepeat(t *testing.T) {
	r := New()

	var called atomic.Int32
	callback := func(eventtime float64) float64 {
		count := called.Add(1)
		if count < 3 {
			return eventtime + 0.01 // Repeat in 10ms
		}
		return NEVER
	}

	r.RegisterTimer(callback, NOW)
	r.Run()
	time.Sleep(100 * time.Millisecond)
	r.End()
	r.Wait()

	if called.Load() < 3 {
		t.Errorf("Timer callback called %d times, expected at least 3", called.Load())
	}
}

func TestUnregisterTimer(t *testing.T) {
	r := New()

	var called atomic.Int32
	callback := func(eventtime float64) float64 {
		called.Add(1)
		return NEVER
	}

	// Register then immediately unregister
	timer := r.RegisterTimer(callback, r.Monotonic()+0.1)
	r.UnregisterTimer(timer)

	r.Run()
	time.Sleep(150 * time.Millisecond)
	r.End()
	r.Wait()

	if called.Load() != 0 {
		t.Errorf("Timer callback called %d times after unregister, expected 0", called.Load())
	}
}

func TestCompletion(t *testing.T) {
	r := New()
	defer r.End()

	comp := r.Completion()

	if comp.Test() {
		t.Error("Completion should not be done yet")
	}

	comp.Complete("result")

	if !comp.Test() {
		t.Error("Completion should be done")
	}

	result := comp.Wait(time.Second, nil)
	if result != "result" {
		t.Errorf("Expected 'result', got %v", result)
	}
}

func TestCompletionWaitTimeout(t *testing.T) {
	r := New()
	defer r.End()

	comp := r.Completion()

	start := time.Now()
	result := comp.Wait(50*time.Millisecond, "timeout")
	elapsed := time.Since(start)

	if result != "timeout" {
		t.Errorf("Expected 'timeout', got %v", result)
	}

	if elapsed < 40*time.Millisecond || elapsed > 100*time.Millisecond {
		t.Errorf("Unexpected wait time: %v", elapsed)
	}
}

func TestRegisterCallback(t *testing.T) {
	r := New()

	var called atomic.Bool
	completion := r.RegisterCallback(func(eventtime float64) interface{} {
		called.Store(true)
		return "callback result"
	}, NOW)

	r.Run()
	time.Sleep(50 * time.Millisecond)
	r.End()
	r.Wait()

	if !called.Load() {
		t.Error("Callback was not called")
	}

	if !completion.Test() {
		t.Error("Completion should be done")
	}

	if completion.result != "callback result" {
		t.Errorf("Expected 'callback result', got %v", completion.result)
	}
}

func TestPause(t *testing.T) {
	r := New()
	defer r.End()

	start := r.Monotonic()
	waketime := start + 0.05 // 50ms

	result := r.Pause(waketime)

	if result < waketime-0.01 {
		t.Errorf("Pause returned too early: %f < %f", result, waketime)
	}
}

func TestPauseImmediate(t *testing.T) {
	r := New()
	defer r.End()

	now := r.Monotonic()
	result := r.Pause(now - 1) // Wake time in the past

	if result < now {
		t.Errorf("Pause should return current time, got %f < %f", result, now)
	}
}

func TestMutex(t *testing.T) {
	r := New()
	defer r.End()

	m := r.NewMutex(false)

	if m.Test() {
		t.Error("Mutex should not be locked initially")
	}

	m.Lock()
	if !m.Test() {
		t.Error("Mutex should be locked after Lock()")
	}

	m.Unlock()
	if m.Test() {
		t.Error("Mutex should not be locked after Unlock()")
	}
}

func TestMutexContention(t *testing.T) {
	r := New()
	defer r.End()

	m := r.NewMutex(false)
	var counter atomic.Int32
	done := make(chan struct{})

	// Start goroutine that will wait for lock
	go func() {
		m.Lock()
		counter.Add(1)
		m.Unlock()
		close(done)
	}()

	// Lock first
	m.Lock()
	time.Sleep(20 * time.Millisecond)

	// Counter should still be 0
	if counter.Load() != 0 {
		t.Error("Goroutine should be waiting")
	}

	// Unlock
	m.Unlock()

	// Wait for goroutine
	<-done

	if counter.Load() != 1 {
		t.Error("Goroutine should have incremented counter")
	}
}

func TestConstants(t *testing.T) {
	if NOW != 0.0 {
		t.Errorf("NOW should be 0.0, got %f", NOW)
	}

	if NEVER < 1e15 {
		t.Errorf("NEVER should be very large, got %f", NEVER)
	}
}
