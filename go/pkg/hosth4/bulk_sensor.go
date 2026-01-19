// Bulk Sensor - port of klippy/extras/bulk_sensor.py
//
// Support for bulk sensor data collection
//
// Copyright (C) 2020-2023 Kevin O'Connor <kevin@koconnor.net>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// BatchBulkHelper assists with collecting bulk sensor data.
type BatchBulkHelper struct {
	rt              *runtime
	batchCallback   func(eventtime float64) map[string]any
	startCallback   func() error
	finishCallback  func() error
	batchInterval   float64
	batchActive     bool
	lastBatchTime   float64
	mu              sync.Mutex
}

// BatchBulkHelperConfig holds configuration for BatchBulkHelper.
type BatchBulkHelperConfig struct {
	BatchCallback   func(eventtime float64) map[string]any
	StartCallback   func() error
	FinishCallback  func() error
	BatchInterval   float64
}

// newBatchBulkHelper creates a new batch bulk helper.
func newBatchBulkHelper(rt *runtime, cfg BatchBulkHelperConfig) *BatchBulkHelper {
	if cfg.BatchInterval <= 0 {
		cfg.BatchInterval = 0.1 // 100ms default
	}

	return &BatchBulkHelper{
		rt:             rt,
		batchCallback:  cfg.BatchCallback,
		startCallback:  cfg.StartCallback,
		finishCallback: cfg.FinishCallback,
		batchInterval:  cfg.BatchInterval,
	}
}

// Start starts batch collection.
func (bbh *BatchBulkHelper) Start() error {
	bbh.mu.Lock()
	defer bbh.mu.Unlock()

	if bbh.batchActive {
		return fmt.Errorf("batch already active")
	}

	if bbh.startCallback != nil {
		if err := bbh.startCallback(); err != nil {
			return err
		}
	}

	bbh.batchActive = true
	log.Printf("bulk_sensor: batch started")
	return nil
}

// Stop stops batch collection.
func (bbh *BatchBulkHelper) Stop() error {
	bbh.mu.Lock()
	defer bbh.mu.Unlock()

	if !bbh.batchActive {
		return nil
	}

	if bbh.finishCallback != nil {
		if err := bbh.finishCallback(); err != nil {
			return err
		}
	}

	bbh.batchActive = false
	log.Printf("bulk_sensor: batch stopped")
	return nil
}

// IsActive returns true if batch collection is active.
func (bbh *BatchBulkHelper) IsActive() bool {
	bbh.mu.Lock()
	defer bbh.mu.Unlock()
	return bbh.batchActive
}

// ProcessBatch processes a batch of data.
func (bbh *BatchBulkHelper) ProcessBatch(eventtime float64) map[string]any {
	bbh.mu.Lock()
	defer bbh.mu.Unlock()

	if !bbh.batchActive || bbh.batchCallback == nil {
		return nil
	}

	bbh.lastBatchTime = eventtime
	return bbh.batchCallback(eventtime)
}

// FixedFreqReader reads data at a fixed frequency.
type FixedFreqReader struct {
	rt              *runtime
	sampleFreq      float64
	samplesPerBlock int
	sampleFormat    string
	lastSequence    int
	data            [][]byte
	mu              sync.Mutex
}

// FixedFreqReaderConfig holds configuration for FixedFreqReader.
type FixedFreqReaderConfig struct {
	SampleFreq      float64
	SamplesPerBlock int
	SampleFormat    string // e.g., "<hhh" for 3 int16 values
}

// newFixedFreqReader creates a new fixed frequency reader.
func newFixedFreqReader(rt *runtime, cfg FixedFreqReaderConfig) *FixedFreqReader {
	return &FixedFreqReader{
		rt:              rt,
		sampleFreq:      cfg.SampleFreq,
		samplesPerBlock: cfg.SamplesPerBlock,
		sampleFormat:    cfg.SampleFormat,
		data:            make([][]byte, 0),
	}
}

// AddData adds a data block.
func (ffr *FixedFreqReader) AddData(data []byte, sequence int) {
	ffr.mu.Lock()
	defer ffr.mu.Unlock()

	ffr.data = append(ffr.data, data)
	ffr.lastSequence = sequence
}

// GetData returns all collected data and clears the buffer.
func (ffr *FixedFreqReader) GetData() [][]byte {
	ffr.mu.Lock()
	defer ffr.mu.Unlock()

	result := ffr.data
	ffr.data = make([][]byte, 0)
	return result
}

// GetSampleFreq returns the sample frequency.
func (ffr *FixedFreqReader) GetSampleFreq() float64 {
	return ffr.sampleFreq
}

// ClockSyncRegression handles clock synchronization between MCU and host.
type ClockSyncRegression struct {
	rt              *runtime
	samples         []bulkClockSample
	minSamples      int
	maxSamples      int
	timeOffset      float64
	timeDrift       float64
	mu              sync.Mutex
}

type bulkClockSample struct {
	mcuClock float64
	time     float64
}

// newClockSyncRegression creates a new clock sync regression.
func newClockSyncRegression(rt *runtime, minSamples, maxSamples int) *ClockSyncRegression {
	if minSamples <= 0 {
		minSamples = 3
	}
	if maxSamples <= 0 {
		maxSamples = 25
	}

	return &ClockSyncRegression{
		rt:         rt,
		samples:    make([]bulkClockSample, 0),
		minSamples: minSamples,
		maxSamples: maxSamples,
	}
}

// AddSample adds a clock synchronization sample.
func (csr *ClockSyncRegression) AddSample(mcuClock, time float64) {
	csr.mu.Lock()
	defer csr.mu.Unlock()

	csr.samples = append(csr.samples, bulkClockSample{mcuClock, time})

	// Keep only maxSamples
	if len(csr.samples) > csr.maxSamples {
		csr.samples = csr.samples[len(csr.samples)-csr.maxSamples:]
	}

	// Update regression if we have enough samples
	if len(csr.samples) >= csr.minSamples {
		csr.updateRegression()
	}
}

// updateRegression updates the linear regression.
func (csr *ClockSyncRegression) updateRegression() {
	n := float64(len(csr.samples))
	var sumX, sumY, sumXY, sumX2 float64

	for _, s := range csr.samples {
		sumX += s.mcuClock
		sumY += s.time
		sumXY += s.mcuClock * s.time
		sumX2 += s.mcuClock * s.mcuClock
	}

	// Linear regression: time = drift * mcu_clock + offset
	denom := n*sumX2 - sumX*sumX
	if denom != 0 {
		csr.timeDrift = (n*sumXY - sumX*sumY) / denom
		csr.timeOffset = (sumY - csr.timeDrift*sumX) / n
	}
}

// GetTime converts MCU clock to host time.
func (csr *ClockSyncRegression) GetTime(mcuClock float64) float64 {
	csr.mu.Lock()
	defer csr.mu.Unlock()

	return csr.timeOffset + csr.timeDrift*mcuClock
}

// GetClock converts host time to MCU clock.
func (csr *ClockSyncRegression) GetClock(time float64) float64 {
	csr.mu.Lock()
	defer csr.mu.Unlock()

	if csr.timeDrift == 0 {
		return 0
	}
	return (time - csr.timeOffset) / csr.timeDrift
}
