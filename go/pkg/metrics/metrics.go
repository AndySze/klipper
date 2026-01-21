// Metrics collection for Klipper Go migration
//
// Provides Prometheus-compatible metrics collection with support for:
// - Counter: Monotonically increasing values
// - Gauge: Values that can go up and down
// - Histogram: Distribution of observations in buckets
//
// Outputs in Prometheus text format for easy scraping.
//
// Copyright (C) 2026 Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package metrics

import (
	"fmt"
	"sort"
	"strings"
	"sync"
	"sync/atomic"
	"time"
)

// MetricType represents the type of metric
type MetricType int

const (
	TypeCounter MetricType = iota
	TypeGauge
	TypeHistogram
)

func (t MetricType) String() string {
	switch t {
	case TypeCounter:
		return "counter"
	case TypeGauge:
		return "gauge"
	case TypeHistogram:
		return "histogram"
	default:
		return "unknown"
	}
}

// Labels represents metric labels as key-value pairs
type Labels map[string]string

// Key generates a unique key for a label set
func (l Labels) Key() string {
	return labelKey(l)
}

// String returns labels in Prometheus format
func (l Labels) String() string {
	return formatLabels(l)
}

// Clone creates a copy of the labels
func (l Labels) Clone() Labels {
	return copyLabels(l)
}

// Merge merges another label set into this one, returning a new Labels
// Values from other override values in this
func (l Labels) Merge(other Labels) Labels {
	result := l.Clone()
	for k, v := range other {
		result[k] = v
	}
	return result
}

// labelKey generates a unique key for a label set
func labelKey(labels Labels) string {
	if len(labels) == 0 {
		return ""
	}
	keys := make([]string, 0, len(labels))
	for k := range labels {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	var sb strings.Builder
	for i, k := range keys {
		if i > 0 {
			sb.WriteByte(',')
		}
		sb.WriteString(k)
		sb.WriteByte('=')
		sb.WriteString(labels[k])
	}
	return sb.String()
}

// formatLabels formats labels for Prometheus output
func formatLabels(labels Labels) string {
	if len(labels) == 0 {
		return ""
	}
	keys := make([]string, 0, len(labels))
	for k := range labels {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	var sb strings.Builder
	sb.WriteByte('{')
	for i, k := range keys {
		if i > 0 {
			sb.WriteByte(',')
		}
		sb.WriteString(k)
		sb.WriteString("=\"")
		sb.WriteString(escapeLabel(labels[k]))
		sb.WriteByte('"')
	}
	sb.WriteByte('}')
	return sb.String()
}

// escapeLabel escapes special characters in label values
func escapeLabel(s string) string {
	s = strings.ReplaceAll(s, "\\", "\\\\")
	s = strings.ReplaceAll(s, "\"", "\\\"")
	s = strings.ReplaceAll(s, "\n", "\\n")
	return s
}

// Metric is the interface for all metric types
type Metric interface {
	Name() string
	Help() string
	Type() MetricType
	Write(sb *strings.Builder)
}

// Counter is a monotonically increasing metric
type Counter struct {
	name   string
	help   string
	values sync.Map // labelKey -> *uint64
}

// NewCounter creates a new counter metric
func NewCounter(name, help string) *Counter {
	return &Counter{name: name, help: help}
}

func (c *Counter) Name() string     { return c.name }
func (c *Counter) Help() string     { return c.help }
func (c *Counter) Type() MetricType { return TypeCounter }

// Inc increments the counter by 1
func (c *Counter) Inc(labels Labels) {
	c.Add(labels, 1)
}

// Add increments the counter by the given value
func (c *Counter) Add(labels Labels, delta uint64) {
	key := labelKey(labels)
	val, _ := c.values.LoadOrStore(key, &counterValue{labels: labels})
	cv := val.(*counterValue)
	atomic.AddUint64(&cv.value, delta)
}

// Get returns the current counter value for labels
func (c *Counter) Get(labels Labels) uint64 {
	key := labelKey(labels)
	val, ok := c.values.Load(key)
	if !ok {
		return 0
	}
	return atomic.LoadUint64(&val.(*counterValue).value)
}

func (c *Counter) Write(sb *strings.Builder) {
	sb.WriteString("# HELP ")
	sb.WriteString(c.name)
	sb.WriteByte(' ')
	sb.WriteString(c.help)
	sb.WriteByte('\n')
	sb.WriteString("# TYPE ")
	sb.WriteString(c.name)
	sb.WriteString(" counter\n")

	c.values.Range(func(_, value interface{}) bool {
		cv := value.(*counterValue)
		sb.WriteString(c.name)
		sb.WriteString(formatLabels(cv.labels))
		sb.WriteByte(' ')
		sb.WriteString(fmt.Sprintf("%d", atomic.LoadUint64(&cv.value)))
		sb.WriteByte('\n')
		return true
	})
}

type counterValue struct {
	labels Labels
	value  uint64
}

// Gauge is a metric that can go up and down
type Gauge struct {
	name   string
	help   string
	values sync.Map // labelKey -> *gaugeValue
}

// NewGauge creates a new gauge metric
func NewGauge(name, help string) *Gauge {
	return &Gauge{name: name, help: help}
}

func (g *Gauge) Name() string     { return g.name }
func (g *Gauge) Help() string     { return g.help }
func (g *Gauge) Type() MetricType { return TypeGauge }

// Set sets the gauge to the given value
func (g *Gauge) Set(labels Labels, value float64) {
	key := labelKey(labels)
	val, _ := g.values.LoadOrStore(key, &gaugeValue{labels: labels})
	gv := val.(*gaugeValue)
	gv.mu.Lock()
	gv.value = value
	gv.mu.Unlock()
}

// Inc increments the gauge by 1
func (g *Gauge) Inc(labels Labels) {
	g.Add(labels, 1)
}

// Dec decrements the gauge by 1
func (g *Gauge) Dec(labels Labels) {
	g.Add(labels, -1)
}

// Add adds the given value to the gauge
func (g *Gauge) Add(labels Labels, delta float64) {
	key := labelKey(labels)
	val, _ := g.values.LoadOrStore(key, &gaugeValue{labels: labels})
	gv := val.(*gaugeValue)
	gv.mu.Lock()
	gv.value += delta
	gv.mu.Unlock()
}

// Sub subtracts the given value from the gauge
func (g *Gauge) Sub(labels Labels, delta float64) {
	g.Add(labels, -delta)
}

// Get returns the current gauge value for labels
func (g *Gauge) Get(labels Labels) float64 {
	key := labelKey(labels)
	val, ok := g.values.Load(key)
	if !ok {
		return 0
	}
	gv := val.(*gaugeValue)
	gv.mu.Lock()
	defer gv.mu.Unlock()
	return gv.value
}

func (g *Gauge) Write(sb *strings.Builder) {
	sb.WriteString("# HELP ")
	sb.WriteString(g.name)
	sb.WriteByte(' ')
	sb.WriteString(g.help)
	sb.WriteByte('\n')
	sb.WriteString("# TYPE ")
	sb.WriteString(g.name)
	sb.WriteString(" gauge\n")

	g.values.Range(func(_, value interface{}) bool {
		gv := value.(*gaugeValue)
		gv.mu.Lock()
		v := gv.value
		gv.mu.Unlock()
		sb.WriteString(g.name)
		sb.WriteString(formatLabels(gv.labels))
		sb.WriteByte(' ')
		sb.WriteString(formatFloat(v))
		sb.WriteByte('\n')
		return true
	})
}

type gaugeValue struct {
	labels Labels
	value  float64
	mu     sync.Mutex
}

// Histogram tracks the distribution of observations
type Histogram struct {
	name    string
	help    string
	buckets []float64
	values  sync.Map // labelKey -> *histogramValue
}

// NewHistogram creates a new histogram metric with the given buckets
func NewHistogram(name, help string, buckets []float64) *Histogram {
	// Sort and deduplicate buckets
	sorted := make([]float64, len(buckets))
	copy(sorted, buckets)
	sort.Float64s(sorted)
	return &Histogram{name: name, help: help, buckets: sorted}
}

// DefaultBuckets returns default histogram buckets for latency metrics
func DefaultBuckets() []float64 {
	return []float64{.005, .01, .025, .05, .1, .25, .5, 1, 2.5, 5, 10}
}

// LinearBuckets creates count buckets starting at start with width intervals
func LinearBuckets(start, width float64, count int) []float64 {
	buckets := make([]float64, count)
	for i := 0; i < count; i++ {
		buckets[i] = start + float64(i)*width
	}
	return buckets
}

// ExponentialBuckets creates count buckets starting at start with factor multiplier
func ExponentialBuckets(start, factor float64, count int) []float64 {
	buckets := make([]float64, count)
	for i := 0; i < count; i++ {
		buckets[i] = start
		start *= factor
	}
	return buckets
}

func (h *Histogram) Name() string     { return h.name }
func (h *Histogram) Help() string     { return h.help }
func (h *Histogram) Type() MetricType { return TypeHistogram }

// Observe records a value in the histogram
func (h *Histogram) Observe(labels Labels, value float64) {
	key := labelKey(labels)
	val, _ := h.values.LoadOrStore(key, &histogramValue{
		labels:  labels,
		buckets: make([]uint64, len(h.buckets)),
	})
	hv := val.(*histogramValue)
	hv.mu.Lock()
	hv.count++
	hv.sum += value
	for i, bound := range h.buckets {
		if value <= bound {
			hv.buckets[i]++
		}
	}
	hv.mu.Unlock()
}

// Timer returns a function that records the elapsed time when called
func (h *Histogram) Timer(labels Labels) func() {
	start := time.Now()
	return func() {
		h.Observe(labels, time.Since(start).Seconds())
	}
}

func (h *Histogram) Write(sb *strings.Builder) {
	sb.WriteString("# HELP ")
	sb.WriteString(h.name)
	sb.WriteByte(' ')
	sb.WriteString(h.help)
	sb.WriteByte('\n')
	sb.WriteString("# TYPE ")
	sb.WriteString(h.name)
	sb.WriteString(" histogram\n")

	h.values.Range(func(_, value interface{}) bool {
		hv := value.(*histogramValue)
		hv.mu.Lock()
		count := hv.count
		sum := hv.sum
		bucketCounts := make([]uint64, len(hv.buckets))
		copy(bucketCounts, hv.buckets)
		hv.mu.Unlock()

		// Write bucket values
		cumulative := uint64(0)
		for i, bound := range h.buckets {
			cumulative += bucketCounts[i]
			bucketLabels := copyLabels(hv.labels)
			bucketLabels["le"] = formatFloat(bound)
			sb.WriteString(h.name)
			sb.WriteString("_bucket")
			sb.WriteString(formatLabels(bucketLabels))
			sb.WriteByte(' ')
			sb.WriteString(fmt.Sprintf("%d", cumulative))
			sb.WriteByte('\n')
		}
		// +Inf bucket
		infLabels := copyLabels(hv.labels)
		infLabels["le"] = "+Inf"
		sb.WriteString(h.name)
		sb.WriteString("_bucket")
		sb.WriteString(formatLabels(infLabels))
		sb.WriteByte(' ')
		sb.WriteString(fmt.Sprintf("%d", count))
		sb.WriteByte('\n')

		// Sum and count
		sb.WriteString(h.name)
		sb.WriteString("_sum")
		sb.WriteString(formatLabels(hv.labels))
		sb.WriteByte(' ')
		sb.WriteString(formatFloat(sum))
		sb.WriteByte('\n')

		sb.WriteString(h.name)
		sb.WriteString("_count")
		sb.WriteString(formatLabels(hv.labels))
		sb.WriteByte(' ')
		sb.WriteString(fmt.Sprintf("%d", count))
		sb.WriteByte('\n')

		return true
	})
}

type histogramValue struct {
	labels  Labels
	count   uint64
	sum     float64
	buckets []uint64
	mu      sync.Mutex
}

// HistogramSnapshot contains a point-in-time snapshot of histogram values
type HistogramSnapshot struct {
	Count   uint64
	Sum     float64
	Buckets map[float64]uint64
}

// GetSnapshot returns a snapshot of histogram values for the given labels
func (h *Histogram) GetSnapshot(labels Labels) HistogramSnapshot {
	key := labelKey(labels)
	val, ok := h.values.Load(key)
	if !ok {
		return HistogramSnapshot{Buckets: make(map[float64]uint64)}
	}
	hv := val.(*histogramValue)
	hv.mu.Lock()
	defer hv.mu.Unlock()

	buckets := make(map[float64]uint64, len(h.buckets))
	cumulative := uint64(0)
	for i, bound := range h.buckets {
		cumulative += hv.buckets[i]
		buckets[bound] = cumulative
	}

	return HistogramSnapshot{
		Count:   hv.count,
		Sum:     hv.sum,
		Buckets: buckets,
	}
}

// copyLabels creates a copy of the labels map
func copyLabels(labels Labels) Labels {
	if labels == nil {
		return Labels{}
	}
	result := make(Labels, len(labels))
	for k, v := range labels {
		result[k] = v
	}
	return result
}

// formatFloat formats a float64 for Prometheus output
func formatFloat(v float64) string {
	return fmt.Sprintf("%g", v)
}

// Registry holds all registered metrics
type Registry struct {
	mu      sync.RWMutex
	metrics map[string]Metric
	order   []string // Preserve registration order
}

// NewRegistry creates a new metrics registry
func NewRegistry() *Registry {
	return &Registry{
		metrics: make(map[string]Metric),
	}
}

// Register adds a metric to the registry
func (r *Registry) Register(metric Metric) error {
	r.mu.Lock()
	defer r.mu.Unlock()

	name := metric.Name()
	if _, exists := r.metrics[name]; exists {
		return fmt.Errorf("metric %q already registered", name)
	}
	r.metrics[name] = metric
	r.order = append(r.order, name)
	return nil
}

// MustRegister adds a metric and panics on error
func (r *Registry) MustRegister(metric Metric) {
	if err := r.Register(metric); err != nil {
		panic(err)
	}
}

// Unregister removes a metric from the registry
func (r *Registry) Unregister(name string) {
	r.mu.Lock()
	defer r.mu.Unlock()

	delete(r.metrics, name)
	for i, n := range r.order {
		if n == name {
			r.order = append(r.order[:i], r.order[i+1:]...)
			break
		}
	}
}

// Get returns a metric by name
func (r *Registry) Get(name string) Metric {
	r.mu.RLock()
	defer r.mu.RUnlock()
	return r.metrics[name]
}

// Gather collects all metrics in Prometheus text format
func (r *Registry) Gather() string {
	r.mu.RLock()
	defer r.mu.RUnlock()

	var sb strings.Builder
	for _, name := range r.order {
		if metric, ok := r.metrics[name]; ok {
			metric.Write(&sb)
		}
	}
	return sb.String()
}

// Default registry
var defaultRegistry = NewRegistry()

// DefaultRegistry returns the default metrics registry
func DefaultRegistry() *Registry {
	return defaultRegistry
}

// Register adds a metric to the default registry
func Register(metric Metric) error {
	return defaultRegistry.Register(metric)
}

// MustRegister adds a metric to the default registry and panics on error
func MustRegister(metric Metric) {
	defaultRegistry.MustRegister(metric)
}

// Gather collects all metrics from the default registry
func Gather() string {
	return defaultRegistry.Gather()
}
