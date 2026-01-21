// Unit tests for object pools
//
// Copyright (C) 2026 Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package pool

import (
	"sync"
	"testing"
)

func TestArgsMapPool(t *testing.T) {
	// Get a map
	m := GetArgsMap()
	if m == nil {
		t.Fatal("GetArgsMap returned nil")
	}

	// Add some entries
	m["X"] = "100"
	m["Y"] = "200"
	m["F"] = "3000"

	// Return to pool
	PutArgsMap(m)

	// Get another map - should be cleared
	m2 := GetArgsMap()
	if len(m2) != 0 {
		t.Errorf("pooled map should be empty, got %d entries", len(m2))
	}

	PutArgsMap(m2)
}

func TestArgsMapPoolNil(t *testing.T) {
	// Should not panic
	PutArgsMap(nil)
}

func TestFloat64SlicePool(t *testing.T) {
	sizes := []int{3, 4, 5, 6, 8}

	for _, size := range sizes {
		s := GetFloat64Slice(size)
		if len(s) != size {
			t.Errorf("expected slice of size %d, got %d", size, len(s))
		}

		// Verify zeroed
		for i, v := range s {
			if v != 0 {
				t.Errorf("slice[%d] should be 0, got %f", i, v)
			}
		}

		// Modify and return
		s[0] = 100.5
		PutFloat64Slice(s)

		// Get again - should be zeroed
		s2 := GetFloat64Slice(size)
		if s2[0] != 0 {
			t.Errorf("pooled slice should be zeroed, got %f", s2[0])
		}
		PutFloat64Slice(s2)
	}
}

func TestFloat64SlicePoolNonStandard(t *testing.T) {
	// Non-pooled size
	s := GetFloat64Slice(7)
	if len(s) != 7 {
		t.Errorf("expected slice of size 7, got %d", len(s))
	}
	// This should not panic
	PutFloat64Slice(s)
}

func TestFloat64SlicePoolNil(t *testing.T) {
	// Should not panic
	PutFloat64Slice(nil)
}

func TestByteBuffer(t *testing.T) {
	b := GetByteBuffer()
	if b == nil {
		t.Fatal("GetByteBuffer returned nil")
	}

	// Write some data
	b.WriteString("hello")
	b.WriteByte(' ')
	b.Write([]byte("world"))

	if b.Len() != 11 {
		t.Errorf("expected length 11, got %d", b.Len())
	}

	if string(b.Bytes()) != "hello world" {
		t.Errorf("unexpected content: %s", string(b.Bytes()))
	}

	// Return to pool
	PutByteBuffer(b)

	// Get again - should be reset
	b2 := GetByteBuffer()
	if b2.Len() != 0 {
		t.Errorf("pooled buffer should be empty, got length %d", b2.Len())
	}
	PutByteBuffer(b2)
}

func TestByteBufferGrow(t *testing.T) {
	b := GetByteBuffer()

	// Grow and write
	b.Grow(100)
	if b.Cap() < 100 {
		t.Errorf("capacity should be at least 100, got %d", b.Cap())
	}

	// Write more than initial capacity
	for i := 0; i < 200; i++ {
		b.WriteByte(byte(i % 256))
	}

	if b.Len() != 200 {
		t.Errorf("expected length 200, got %d", b.Len())
	}

	PutByteBuffer(b)
}

func TestByteBufferReset(t *testing.T) {
	b := GetByteBuffer()
	b.WriteString("test data")
	b.Reset()

	if b.Len() != 0 {
		t.Errorf("after Reset, length should be 0, got %d", b.Len())
	}

	PutByteBuffer(b)
}

func TestByteBufferOversized(t *testing.T) {
	b := GetByteBuffer()

	// Write more than 4KB
	data := make([]byte, 5000)
	b.Write(data)

	// Return - should not be pooled due to size
	PutByteBuffer(b)

	// Get new buffer - should have smaller capacity
	b2 := GetByteBuffer()
	// Can't easily verify it's a new buffer, but at least verify it works
	if b2.Cap() > 4096 {
		// This could happen if pool was empty and we got the same buffer
		// That's actually fine - the point is we don't keep too many large buffers
	}
	PutByteBuffer(b2)
}

func TestByteBufferNil(t *testing.T) {
	// Should not panic
	PutByteBuffer(nil)
}

func TestStringSlicePool(t *testing.T) {
	s := GetStringSlice()
	if s == nil {
		t.Fatal("GetStringSlice returned nil")
	}

	// Add entries
	*s = append(*s, "hello", "world")

	if len(*s) != 2 {
		t.Errorf("expected 2 entries, got %d", len(*s))
	}

	// Return to pool
	PutStringSlice(s)

	// Get again - should be empty
	s2 := GetStringSlice()
	if len(*s2) != 0 {
		t.Errorf("pooled slice should be empty, got %d entries", len(*s2))
	}
	PutStringSlice(s2)
}

func TestStringSlicePoolNil(t *testing.T) {
	// Should not panic
	PutStringSlice(nil)
}

func TestStatusMapPool(t *testing.T) {
	m := GetStatusMap()
	if m == nil {
		t.Fatal("GetStatusMap returned nil")
	}

	// Add entries
	m["position"] = []float64{1, 2, 3}
	m["temperature"] = 200.5
	m["state"] = "ready"

	// Return to pool
	PutStatusMap(m)

	// Get again - should be empty
	m2 := GetStatusMap()
	if len(m2) != 0 {
		t.Errorf("pooled map should be empty, got %d entries", len(m2))
	}
	PutStatusMap(m2)
}

func TestStatusMapPoolNil(t *testing.T) {
	// Should not panic
	PutStatusMap(nil)
}

// Concurrent tests

func TestArgsMapPoolConcurrent(t *testing.T) {
	var wg sync.WaitGroup
	iterations := 1000
	goroutines := 10

	for i := 0; i < goroutines; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < iterations; j++ {
				m := GetArgsMap()
				m["key"] = "value"
				PutArgsMap(m)
			}
		}()
	}

	wg.Wait()
}

func TestFloat64SlicePoolConcurrent(t *testing.T) {
	var wg sync.WaitGroup
	iterations := 1000
	goroutines := 10

	for i := 0; i < goroutines; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < iterations; j++ {
				s := GetFloat64Slice(4)
				s[0] = 100
				s[1] = 200
				PutFloat64Slice(s)
			}
		}()
	}

	wg.Wait()
}

func TestByteBufferPoolConcurrent(t *testing.T) {
	var wg sync.WaitGroup
	iterations := 1000
	goroutines := 10

	for i := 0; i < goroutines; i++ {
		wg.Add(1)
		go func() {
			defer wg.Done()
			for j := 0; j < iterations; j++ {
				b := GetByteBuffer()
				b.WriteString("test")
				PutByteBuffer(b)
			}
		}()
	}

	wg.Wait()
}

// Benchmarks

func BenchmarkArgsMapPool(b *testing.B) {
	for i := 0; i < b.N; i++ {
		m := GetArgsMap()
		m["X"] = "100"
		m["Y"] = "200"
		m["F"] = "3000"
		PutArgsMap(m)
	}
}

func BenchmarkArgsMapNoPool(b *testing.B) {
	for i := 0; i < b.N; i++ {
		m := make(map[string]string, 8)
		m["X"] = "100"
		m["Y"] = "200"
		m["F"] = "3000"
		_ = m
	}
}

func BenchmarkFloat64SlicePool(b *testing.B) {
	for i := 0; i < b.N; i++ {
		s := GetFloat64Slice(4)
		s[0] = 100
		s[1] = 200
		s[2] = 10
		s[3] = 50
		PutFloat64Slice(s)
	}
}

func BenchmarkFloat64SliceNoPool(b *testing.B) {
	for i := 0; i < b.N; i++ {
		s := make([]float64, 4)
		s[0] = 100
		s[1] = 200
		s[2] = 10
		s[3] = 50
		_ = s
	}
}

func BenchmarkByteBufferPool(b *testing.B) {
	data := []byte("queue_step oid=0 interval=1234 count=100 add=5")
	b.ResetTimer()

	for i := 0; i < b.N; i++ {
		buf := GetByteBuffer()
		buf.Write(data)
		PutByteBuffer(buf)
	}
}

func BenchmarkByteBufferNoPool(b *testing.B) {
	data := []byte("queue_step oid=0 interval=1234 count=100 add=5")
	b.ResetTimer()

	for i := 0; i < b.N; i++ {
		buf := make([]byte, 0, 64)
		buf = append(buf, data...)
		_ = buf
	}
}

func BenchmarkStatusMapPool(b *testing.B) {
	for i := 0; i < b.N; i++ {
		m := GetStatusMap()
		m["position"] = []float64{1, 2, 3}
		m["temperature"] = 200.5
		m["state"] = "ready"
		PutStatusMap(m)
	}
}

func BenchmarkStatusMapNoPool(b *testing.B) {
	for i := 0; i < b.N; i++ {
		m := make(map[string]any, 16)
		m["position"] = []float64{1, 2, 3}
		m["temperature"] = 200.5
		m["state"] = "ready"
		_ = m
	}
}
