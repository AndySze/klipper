// Object pools for reducing GC pressure in hot paths
//
// Provides reusable object pools for commonly allocated types:
// - String maps (for G-code arguments)
// - Float slices (for positions and coordinates)
// - Byte buffers (for encoding/decoding)
//
// Usage:
//
//	args := pool.GetArgsMap()
//	defer pool.PutArgsMap(args)
//	// use args...
//
// Copyright (C) 2026 Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package pool

import (
	"sync"
)

// ArgsMap pool - for G-code argument maps
var argsMapPool = sync.Pool{
	New: func() any {
		return make(map[string]string, 8) // Pre-allocate common size
	},
}

// GetArgsMap gets a string map from the pool
func GetArgsMap() map[string]string {
	return argsMapPool.Get().(map[string]string)
}

// PutArgsMap returns a string map to the pool after clearing it
func PutArgsMap(m map[string]string) {
	if m == nil {
		return
	}
	// Clear the map
	clear(m)
	argsMapPool.Put(m)
}

// Float64Slice pool - for position arrays
type float64SlicePool struct {
	pools [5]sync.Pool // pools for sizes 3, 4, 5, 6, 8
}

var floatSlicePool = &float64SlicePool{}

func init() {
	// Pre-initialize pools for common sizes
	sizes := []int{3, 4, 5, 6, 8}
	for i, size := range sizes {
		s := size // capture for closure
		floatSlicePool.pools[i].New = func() any {
			return make([]float64, s)
		}
	}
}

// poolIndex returns the pool index for a given size, or -1 if no pool
func poolIndex(size int) int {
	switch size {
	case 3:
		return 0
	case 4:
		return 1
	case 5:
		return 2
	case 6:
		return 3
	case 8:
		return 4
	default:
		return -1
	}
}

// GetFloat64Slice gets a float64 slice from the pool
// If the requested size doesn't match a pool, allocates a new slice
func GetFloat64Slice(size int) []float64 {
	idx := poolIndex(size)
	if idx >= 0 {
		s := floatSlicePool.pools[idx].Get().([]float64)
		// Zero the slice
		for i := range s {
			s[i] = 0
		}
		return s
	}
	return make([]float64, size)
}

// PutFloat64Slice returns a float64 slice to the pool
func PutFloat64Slice(s []float64) {
	if s == nil {
		return
	}
	idx := poolIndex(len(s))
	if idx >= 0 {
		floatSlicePool.pools[idx].Put(s)
	}
	// Non-pooled sizes are just discarded
}

// ByteBuffer pool - for encoding buffers
type ByteBuffer struct {
	buf []byte
}

var byteBufferPool = sync.Pool{
	New: func() any {
		return &ByteBuffer{
			buf: make([]byte, 0, 64), // Common message size
		}
	},
}

// GetByteBuffer gets a byte buffer from the pool
func GetByteBuffer() *ByteBuffer {
	b := byteBufferPool.Get().(*ByteBuffer)
	b.buf = b.buf[:0] // Reset length but keep capacity
	return b
}

// PutByteBuffer returns a byte buffer to the pool
func PutByteBuffer(b *ByteBuffer) {
	if b == nil {
		return
	}
	// Don't pool oversized buffers (> 4KB)
	if cap(b.buf) > 4096 {
		return
	}
	byteBufferPool.Put(b)
}

// Bytes returns the buffer's byte slice
func (b *ByteBuffer) Bytes() []byte {
	return b.buf
}

// Write appends bytes to the buffer
func (b *ByteBuffer) Write(p []byte) (int, error) {
	b.buf = append(b.buf, p...)
	return len(p), nil
}

// WriteByte appends a single byte
func (b *ByteBuffer) WriteByte(c byte) error {
	b.buf = append(b.buf, c)
	return nil
}

// WriteString appends a string
func (b *ByteBuffer) WriteString(s string) (int, error) {
	b.buf = append(b.buf, s...)
	return len(s), nil
}

// Len returns the buffer length
func (b *ByteBuffer) Len() int {
	return len(b.buf)
}

// Cap returns the buffer capacity
func (b *ByteBuffer) Cap() int {
	return cap(b.buf)
}

// Reset clears the buffer
func (b *ByteBuffer) Reset() {
	b.buf = b.buf[:0]
}

// Grow ensures the buffer has capacity for n more bytes
func (b *ByteBuffer) Grow(n int) {
	if cap(b.buf)-len(b.buf) < n {
		newCap := cap(b.buf)*2 + n
		newBuf := make([]byte, len(b.buf), newCap)
		copy(newBuf, b.buf)
		b.buf = newBuf
	}
}

// StringSlice pool - for string slices (e.g., from strings.Fields)
var stringSlicePool = sync.Pool{
	New: func() any {
		s := make([]string, 0, 16)
		return &s
	},
}

// GetStringSlice gets a string slice from the pool
func GetStringSlice() *[]string {
	s := stringSlicePool.Get().(*[]string)
	*s = (*s)[:0]
	return s
}

// PutStringSlice returns a string slice to the pool
func PutStringSlice(s *[]string) {
	if s == nil || cap(*s) > 256 {
		return
	}
	// Clear to allow GC of string contents
	for i := range *s {
		(*s)[i] = ""
	}
	*s = (*s)[:0]
	stringSlicePool.Put(s)
}

// StatusMap pool - for GetStatus() return values
var statusMapPool = sync.Pool{
	New: func() any {
		return make(map[string]any, 16)
	},
}

// GetStatusMap gets a status map from the pool
func GetStatusMap() map[string]any {
	return statusMapPool.Get().(map[string]any)
}

// PutStatusMap returns a status map to the pool
func PutStatusMap(m map[string]any) {
	if m == nil {
		return
	}
	clear(m)
	statusMapPool.Put(m)
}

// PoolStats holds statistics about pool usage
type PoolStats struct {
	ArgsMapHits      uint64
	Float64SliceHits uint64
	ByteBufferHits   uint64
	StringSliceHits  uint64
	StatusMapHits    uint64
}

// Note: In production, you might want to add atomic counters to track
// pool hit rates. This is left as a future enhancement.
