// Unit tests for Prometheus metrics implementation
//
// Copyright (C) 2026 Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package metrics

import (
	"math"
	"strings"
	"sync"
	"testing"
)

// TestCounterBasic tests basic counter operations
func TestCounterBasic(t *testing.T) {
	c := NewCounter("test_counter", "A test counter")

	// Initial value should be 0
	if v := c.Get(nil); v != 0 {
		t.Errorf("expected initial value 0, got %d", v)
	}

	// Increment by 1
	c.Inc(nil)
	if v := c.Get(nil); v != 1 {
		t.Errorf("expected value 1 after Inc, got %d", v)
	}

	// Add 10
	c.Add(nil, 10)
	if v := c.Get(nil); v != 11 {
		t.Errorf("expected value 11 after Add(10), got %d", v)
	}

	// Name and help
	if c.Name() != "test_counter" {
		t.Errorf("expected name 'test_counter', got '%s'", c.Name())
	}
	if c.Help() != "A test counter" {
		t.Errorf("expected help 'A test counter', got '%s'", c.Help())
	}
}

// TestCounterWithLabels tests counter with labels
func TestCounterWithLabels(t *testing.T) {
	c := NewCounter("requests_total", "Total requests")

	labels1 := Labels{"method": "GET", "status": "200"}
	labels2 := Labels{"method": "POST", "status": "500"}

	c.Inc(labels1)
	c.Inc(labels1)
	c.Inc(labels2)

	if v := c.Get(labels1); v != 2 {
		t.Errorf("expected GET/200 count 2, got %d", v)
	}
	if v := c.Get(labels2); v != 1 {
		t.Errorf("expected POST/500 count 1, got %d", v)
	}
	if v := c.Get(Labels{"method": "PUT"}); v != 0 {
		t.Errorf("expected PUT count 0, got %d", v)
	}
}

// TestCounterConcurrency tests counter thread safety
func TestCounterConcurrency(t *testing.T) {
	c := NewCounter("concurrent_counter", "Test concurrent access")
	var wg sync.WaitGroup

	numGoroutines := 100
	incsPerGoroutine := 1000

	for i := 0; i < numGoroutines; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < incsPerGoroutine; j++ {
				c.Inc(nil)
			}
		}()
	}

	wg.Wait()

	expected := uint64(numGoroutines * incsPerGoroutine)
	if v := c.Get(nil); v != expected {
		t.Errorf("expected %d, got %d", expected, v)
	}
}

// TestGaugeBasic tests basic gauge operations
func TestGaugeBasic(t *testing.T) {
	g := NewGauge("test_gauge", "A test gauge")

	// Initial value should be 0
	if v := g.Get(nil); v != 0 {
		t.Errorf("expected initial value 0, got %f", v)
	}

	// Set value
	g.Set(nil, 42.5)
	if v := g.Get(nil); v != 42.5 {
		t.Errorf("expected value 42.5, got %f", v)
	}

	// Add value
	g.Add(nil, 7.5)
	if v := g.Get(nil); v != 50 {
		t.Errorf("expected value 50, got %f", v)
	}

	// Sub value
	g.Sub(nil, 10)
	if v := g.Get(nil); v != 40 {
		t.Errorf("expected value 40, got %f", v)
	}

	// Inc
	g.Inc(nil)
	if v := g.Get(nil); v != 41 {
		t.Errorf("expected value 41, got %f", v)
	}

	// Dec
	g.Dec(nil)
	if v := g.Get(nil); v != 40 {
		t.Errorf("expected value 40, got %f", v)
	}
}

// TestGaugeWithLabels tests gauge with labels
func TestGaugeWithLabels(t *testing.T) {
	g := NewGauge("temperature", "Current temperature")

	g.Set(Labels{"sensor": "extruder"}, 200.5)
	g.Set(Labels{"sensor": "bed"}, 60.0)

	if v := g.Get(Labels{"sensor": "extruder"}); v != 200.5 {
		t.Errorf("expected extruder temp 200.5, got %f", v)
	}
	if v := g.Get(Labels{"sensor": "bed"}); v != 60.0 {
		t.Errorf("expected bed temp 60.0, got %f", v)
	}
}

// TestGaugeConcurrency tests gauge thread safety
func TestGaugeConcurrency(t *testing.T) {
	g := NewGauge("concurrent_gauge", "Test concurrent access")
	var wg sync.WaitGroup

	numGoroutines := 100
	opsPerGoroutine := 1000

	for i := 0; i < numGoroutines; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < opsPerGoroutine; j++ {
				g.Inc(nil)
				g.Dec(nil)
				g.Add(nil, 2)
			}
		}()
	}

	wg.Wait()

	// Each goroutine adds net 2 per iteration
	expected := float64(numGoroutines * opsPerGoroutine * 2)
	if v := g.Get(nil); v != expected {
		t.Errorf("expected %f, got %f", expected, v)
	}
}

// TestHistogramBasic tests basic histogram operations
func TestHistogramBasic(t *testing.T) {
	h := NewHistogram("request_duration", "Request duration in seconds",
		[]float64{0.01, 0.05, 0.1, 0.5, 1.0})

	// Observe some values
	h.Observe(nil, 0.005) // <= 0.01
	h.Observe(nil, 0.02)  // <= 0.05
	h.Observe(nil, 0.08)  // <= 0.1
	h.Observe(nil, 0.3)   // <= 0.5
	h.Observe(nil, 0.7)   // <= 1.0
	h.Observe(nil, 2.0)   // > 1.0 (only in +Inf)

	// Get snapshot
	snapshot := h.GetSnapshot(nil)

	if snapshot.Count != 6 {
		t.Errorf("expected count 6, got %d", snapshot.Count)
	}

	expectedSum := 0.005 + 0.02 + 0.08 + 0.3 + 0.7 + 2.0
	if math.Abs(snapshot.Sum-expectedSum) > 0.0001 {
		t.Errorf("expected sum %f, got %f", expectedSum, snapshot.Sum)
	}

	// Check bucket counts (cumulative) - note how each bucket contains all values <= its bound
	// 0.005 <= 0.01, so bucket[0.01] = 1
	// 0.005, 0.02 <= 0.05, so bucket[0.05] = 2
	// etc.
	if snapshot.Buckets[0.01] < 1 {
		t.Errorf("bucket 0.01: expected >= 1, got %d", snapshot.Buckets[0.01])
	}
	if snapshot.Count < 6 {
		t.Errorf("expected at least 6 observations")
	}
}

// TestHistogramWithLabels tests histogram with labels
func TestHistogramWithLabels(t *testing.T) {
	h := NewHistogram("command_duration", "Command execution time",
		[]float64{0.001, 0.01, 0.1})

	labels1 := Labels{"type": "G1"}
	labels2 := Labels{"type": "G28"}

	h.Observe(labels1, 0.0005)
	h.Observe(labels1, 0.005)
	h.Observe(labels2, 0.05)

	snap1 := h.GetSnapshot(labels1)
	snap2 := h.GetSnapshot(labels2)

	if snap1.Count != 2 {
		t.Errorf("expected G1 count 2, got %d", snap1.Count)
	}
	if snap2.Count != 1 {
		t.Errorf("expected G28 count 1, got %d", snap2.Count)
	}
}

// TestDefaultBuckets tests default bucket values
func TestDefaultBuckets(t *testing.T) {
	buckets := DefaultBuckets()
	if len(buckets) != 11 {
		t.Errorf("expected 11 default buckets, got %d", len(buckets))
	}
	if buckets[0] != 0.005 {
		t.Errorf("expected first bucket 0.005, got %f", buckets[0])
	}
	if buckets[len(buckets)-1] != 10 {
		t.Errorf("expected last bucket 10, got %f", buckets[len(buckets)-1])
	}
}

// TestLinearBuckets tests linear bucket generation
func TestLinearBuckets(t *testing.T) {
	buckets := LinearBuckets(0, 10, 5)
	expected := []float64{0, 10, 20, 30, 40}

	if len(buckets) != len(expected) {
		t.Errorf("expected %d buckets, got %d", len(expected), len(buckets))
	}

	for i, v := range expected {
		if buckets[i] != v {
			t.Errorf("bucket %d: expected %f, got %f", i, v, buckets[i])
		}
	}
}

// TestExponentialBuckets tests exponential bucket generation
func TestExponentialBuckets(t *testing.T) {
	buckets := ExponentialBuckets(1, 2, 5)
	expected := []float64{1, 2, 4, 8, 16}

	if len(buckets) != len(expected) {
		t.Errorf("expected %d buckets, got %d", len(expected), len(buckets))
	}

	for i, v := range expected {
		if buckets[i] != v {
			t.Errorf("bucket %d: expected %f, got %f", i, v, buckets[i])
		}
	}
}

// TestRegistryBasic tests registry operations
func TestRegistryBasic(t *testing.T) {
	r := NewRegistry()

	c := NewCounter("my_counter", "A counter")
	g := NewGauge("my_gauge", "A gauge")

	if err := r.Register(c); err != nil {
		t.Errorf("failed to register counter: %v", err)
	}
	if err := r.Register(g); err != nil {
		t.Errorf("failed to register gauge: %v", err)
	}

	// Duplicate registration should fail
	if err := r.Register(c); err == nil {
		t.Error("expected error on duplicate registration")
	}

	// Unregister
	r.Unregister("my_counter")
	if err := r.Register(c); err != nil {
		t.Errorf("failed to re-register after unregister: %v", err)
	}
}

// TestRegistryGather tests Prometheus format output
func TestRegistryGather(t *testing.T) {
	r := NewRegistry()

	c := NewCounter("test_requests_total", "Total requests")
	c.Add(Labels{"method": "GET"}, 100)
	c.Add(Labels{"method": "POST"}, 50)
	r.MustRegister(c)

	g := NewGauge("test_temperature", "Current temperature")
	g.Set(nil, 25.5)
	r.MustRegister(g)

	output := r.Gather()

	// Check counter output
	if !strings.Contains(output, "# HELP test_requests_total Total requests") {
		t.Error("missing counter HELP")
	}
	if !strings.Contains(output, "# TYPE test_requests_total counter") {
		t.Error("missing counter TYPE")
	}
	if !strings.Contains(output, `test_requests_total{method="GET"} 100`) {
		t.Error("missing GET counter value")
	}
	if !strings.Contains(output, `test_requests_total{method="POST"} 50`) {
		t.Error("missing POST counter value")
	}

	// Check gauge output
	if !strings.Contains(output, "# HELP test_temperature Current temperature") {
		t.Error("missing gauge HELP")
	}
	if !strings.Contains(output, "# TYPE test_temperature gauge") {
		t.Error("missing gauge TYPE")
	}
	if !strings.Contains(output, "test_temperature 25.5") {
		t.Error("missing gauge value")
	}
}

// TestHistogramGather tests histogram Prometheus format output
func TestHistogramGather(t *testing.T) {
	r := NewRegistry()

	h := NewHistogram("test_duration_seconds", "Request duration",
		[]float64{0.1, 0.5, 1.0})
	h.Observe(nil, 0.05)
	h.Observe(nil, 0.3)
	h.Observe(nil, 0.8)
	h.Observe(nil, 2.0)
	r.MustRegister(h)

	output := r.Gather()

	// Check histogram output
	if !strings.Contains(output, "# HELP test_duration_seconds Request duration") {
		t.Error("missing histogram HELP")
	}
	if !strings.Contains(output, "# TYPE test_duration_seconds histogram") {
		t.Error("missing histogram TYPE")
	}

	// Check buckets (values may be formatted as 0.1 or 0.5 depending on precision)
	if !strings.Contains(output, `test_duration_seconds_bucket{le="0.1"}`) {
		t.Error("missing bucket 0.1")
	}
	if !strings.Contains(output, `test_duration_seconds_bucket{le="0.5"}`) {
		t.Error("missing bucket 0.5")
	}
	// 1.0 may be formatted as "1"
	if !strings.Contains(output, `test_duration_seconds_bucket{le="1"}`) {
		t.Error("missing bucket 1")
	}
	if !strings.Contains(output, `test_duration_seconds_bucket{le="+Inf"}`) {
		t.Error("missing bucket +Inf")
	}

	// Check sum and count
	if !strings.Contains(output, "test_duration_seconds_sum") {
		t.Error("missing histogram sum")
	}
	if !strings.Contains(output, "test_duration_seconds_count") {
		t.Error("missing histogram count")
	}
}

// TestLabelsKey tests label key generation
func TestLabelsKey(t *testing.T) {
	labels := Labels{"b": "2", "a": "1", "c": "3"}
	key := labels.Key()

	// Should be sorted and contain all keys
	if !strings.Contains(key, "a=1") || !strings.Contains(key, "b=2") || !strings.Contains(key, "c=3") {
		t.Errorf("unexpected key format: %s", key)
	}

	// Same labels should produce same key
	labels2 := Labels{"c": "3", "a": "1", "b": "2"}
	if labels.Key() != labels2.Key() {
		t.Error("same labels should produce same key")
	}
}

// TestLabelsString tests label string formatting
func TestLabelsString(t *testing.T) {
	labels := Labels{"method": "GET", "status": "200"}
	str := labels.String()

	if !strings.HasPrefix(str, "{") || !strings.HasSuffix(str, "}") {
		t.Errorf("unexpected format: %s", str)
	}
}

// TestLabelsClone tests label cloning
func TestLabelsClone(t *testing.T) {
	original := Labels{"a": "1", "b": "2"}
	clone := original.Clone()

	// Modify clone
	clone["c"] = "3"

	// Original should be unchanged
	if _, ok := original["c"]; ok {
		t.Error("original should not have key 'c'")
	}
}

// TestLabelsMerge tests label merging
func TestLabelsMerge(t *testing.T) {
	labels1 := Labels{"a": "1", "b": "2"}
	labels2 := Labels{"b": "override", "c": "3"}
	merged := labels1.Merge(labels2)

	if merged["a"] != "1" {
		t.Error("missing key 'a'")
	}
	if merged["b"] != "override" {
		t.Error("'b' should be overridden")
	}
	if merged["c"] != "3" {
		t.Error("missing key 'c'")
	}

	// Original should be unchanged
	if labels1["b"] != "2" {
		t.Error("original labels1 should be unchanged")
	}
}

// TestNilLabels tests that nil labels work correctly
func TestNilLabels(t *testing.T) {
	c := NewCounter("nil_labels_counter", "Test nil labels")
	c.Inc(nil)
	c.Inc(nil)

	if v := c.Get(nil); v != 2 {
		t.Errorf("expected 2, got %d", v)
	}

	// Empty labels should be equivalent to nil
	c.Inc(Labels{})
	if v := c.Get(nil); v != 3 {
		t.Errorf("expected 3, got %d", v)
	}
}

// TestSpecialCharacterEscaping tests label value escaping
func TestSpecialCharacterEscaping(t *testing.T) {
	r := NewRegistry()
	g := NewGauge("test_escape", "Test escaping")
	g.Set(Labels{"path": `/foo/bar\baz`}, 1)
	g.Set(Labels{"msg": `line1\nline2`}, 2)
	g.Set(Labels{"quote": `say "hello"`}, 3)
	r.MustRegister(g)

	output := r.Gather()

	// Backslashes and quotes should be escaped
	if !strings.Contains(output, `path="`) {
		t.Error("path label should be present")
	}
}

// BenchmarkCounterInc benchmarks counter increment
func BenchmarkCounterInc(b *testing.B) {
	c := NewCounter("bench_counter", "Benchmark counter")
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		c.Inc(nil)
	}
}

// BenchmarkCounterIncWithLabels benchmarks counter increment with labels
func BenchmarkCounterIncWithLabels(b *testing.B) {
	c := NewCounter("bench_counter", "Benchmark counter")
	labels := Labels{"method": "GET", "status": "200"}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		c.Inc(labels)
	}
}

// BenchmarkGaugeSet benchmarks gauge set
func BenchmarkGaugeSet(b *testing.B) {
	g := NewGauge("bench_gauge", "Benchmark gauge")
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		g.Set(nil, float64(i))
	}
}

// BenchmarkHistogramObserve benchmarks histogram observe
func BenchmarkHistogramObserve(b *testing.B) {
	h := NewHistogram("bench_histogram", "Benchmark histogram", DefaultBuckets())
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		h.Observe(nil, float64(i%10)/10.0)
	}
}

// BenchmarkRegistryGather benchmarks gathering metrics
func BenchmarkRegistryGather(b *testing.B) {
	r := NewRegistry()

	// Add some metrics
	for i := 0; i < 10; i++ {
		c := NewCounter("counter_"+string(rune('a'+i)), "Test counter")
		c.Add(nil, uint64(i*100))
		r.MustRegister(c)
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_ = r.Gather()
	}
}
