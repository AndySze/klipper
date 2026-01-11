// Package clocksync provides micro-controller clock synchronization.
// This is the Go equivalent of klippy/clocksync.py.
package clocksync

import (
	"math"
	"sync"
	"time"
)

// Constants for clock synchronization
const (
	// RTT_AGE is the aging factor for round-trip-time
	RTT_AGE = 0.000010 / (60.0 * 60.0)

	// DECAY is the exponential decay factor for linear regression
	DECAY = 1.0 / 30.0

	// TRANSMIT_EXTRA is extra time added to compensate for transmission delay
	TRANSMIT_EXTRA = 0.001

	// Query interval in seconds (slightly off to avoid resonance)
	QUERY_INTERVAL = 0.9839
)

// ClockEstimate holds the current clock synchronization estimate.
type ClockEstimate struct {
	SampleTime float64 // Host time of the estimate
	Clock      int64   // MCU clock at sample time
	Freq       float64 // Estimated MCU frequency
}

// ClockSync synchronizes the host clock with an MCU clock.
type ClockSync struct {
	mu sync.RWMutex

	// MCU frequency from dictionary
	MCUFreq float64

	// Last known MCU clock (64-bit extended)
	lastClock int64

	// Current clock estimate
	clockEst ClockEstimate

	// Minimum round-trip-time tracking
	minHalfRTT  float64
	minRTTTime  float64

	// Linear regression of mcu clock and system sent_time
	timeAvg          float64
	timeVariance     float64
	clockAvg         float64
	clockCovariance  float64
	predictionVar    float64
	lastPredTime     float64

	// Query tracking
	queriesPending int
}

// New creates a new ClockSync with the given MCU frequency.
func New(mcuFreq float64) *ClockSync {
	return &ClockSync{
		MCUFreq:     mcuFreq,
		minHalfRTT:  999999999.9,
		clockEst:    ClockEstimate{Freq: mcuFreq},
	}
}

// Initialize sets up the initial clock state from an uptime response.
func (cs *ClockSync) Initialize(clock int64, sentTime float64) {
	cs.mu.Lock()
	defer cs.mu.Unlock()

	cs.lastClock = clock
	cs.clockAvg = float64(clock)
	cs.timeAvg = sentTime
	cs.clockEst = ClockEstimate{
		SampleTime: sentTime,
		Clock:      clock,
		Freq:       cs.MCUFreq,
	}
	cs.predictionVar = (0.001 * cs.MCUFreq) * (0.001 * cs.MCUFreq)
}

// HandleClock processes a clock response from the MCU.
// Returns the updated clock estimate.
func (cs *ClockSync) HandleClock(clock32 uint32, sentTime, receiveTime float64) ClockEstimate {
	cs.mu.Lock()
	defer cs.mu.Unlock()

	cs.queriesPending = 0

	// Extend 32-bit clock to 64-bit
	lastClock := cs.lastClock
	clockDelta := int64(int32(clock32 - uint32(lastClock&0xffffffff)))
	clock := lastClock + clockDelta
	cs.lastClock = clock

	// Check if this is the best round-trip-time seen so far
	if sentTime == 0 {
		return cs.clockEst
	}

	halfRTT := 0.5 * (receiveTime - sentTime)
	agedRTT := (sentTime - cs.minRTTTime) * RTT_AGE
	if halfRTT < cs.minHalfRTT+agedRTT {
		cs.minHalfRTT = halfRTT
		cs.minRTTTime = sentTime
	}

	// Filter out samples that are extreme outliers
	expClock := (sentTime-cs.timeAvg)*cs.clockEst.Freq + cs.clockAvg
	clockDiff2 := (float64(clock) - expClock) * (float64(clock) - expClock)
	threshold := 0.000500 * cs.MCUFreq
	if clockDiff2 > 25.0*cs.predictionVar && clockDiff2 > threshold*threshold {
		if float64(clock) > expClock && sentTime < cs.lastPredTime+10.0 {
			// Ignore this outlier sample
			return cs.clockEst
		}
		// Reset prediction variance
		cs.predictionVar = (0.001 * cs.MCUFreq) * (0.001 * cs.MCUFreq)
	} else {
		cs.lastPredTime = sentTime
		cs.predictionVar = (1.0-DECAY)*(cs.predictionVar+clockDiff2*DECAY)
	}

	// Add clock and sent_time to linear regression
	diffSentTime := sentTime - cs.timeAvg
	cs.timeAvg += DECAY * diffSentTime
	cs.timeVariance = (1.0 - DECAY) * (cs.timeVariance + diffSentTime*diffSentTime*DECAY)
	diffClock := float64(clock) - cs.clockAvg
	cs.clockAvg += DECAY * diffClock
	cs.clockCovariance = (1.0 - DECAY) * (cs.clockCovariance + diffSentTime*diffClock*DECAY)

	// Update prediction from linear regression
	var newFreq float64
	if cs.timeVariance > 0 {
		newFreq = cs.clockCovariance / cs.timeVariance
	} else {
		newFreq = cs.MCUFreq
	}

	cs.clockEst = ClockEstimate{
		SampleTime: cs.timeAvg + cs.minHalfRTT,
		Clock:      int64(cs.clockAvg),
		Freq:       newFreq,
	}

	return cs.clockEst
}

// GetClock returns the estimated MCU clock at the given event time.
func (cs *ClockSync) GetClock(eventtime float64) int64 {
	cs.mu.RLock()
	defer cs.mu.RUnlock()

	est := cs.clockEst
	return int64(float64(est.Clock) + (eventtime-est.SampleTime)*est.Freq)
}

// EstimateClockSystime returns the estimated system time when the MCU
// will reach the given clock value.
func (cs *ClockSync) EstimateClockSystime(reqClock int64) float64 {
	cs.mu.RLock()
	defer cs.mu.RUnlock()

	est := cs.clockEst
	return float64(reqClock-est.Clock)/est.Freq + est.SampleTime
}

// PrintTimeToClock converts a print time to an MCU clock value.
func (cs *ClockSync) PrintTimeToClock(printTime float64) int64 {
	cs.mu.RLock()
	defer cs.mu.RUnlock()

	return int64(printTime * cs.MCUFreq)
}

// ClockToPrintTime converts an MCU clock value to a print time.
func (cs *ClockSync) ClockToPrintTime(clock int64) float64 {
	cs.mu.RLock()
	defer cs.mu.RUnlock()

	return float64(clock) / cs.MCUFreq
}

// EstimatedPrintTime returns the estimated print time at the given event time.
func (cs *ClockSync) EstimatedPrintTime(eventtime float64) float64 {
	return cs.ClockToPrintTime(cs.GetClock(eventtime))
}

// Clock32ToClock64 extends a 32-bit clock to 64-bit.
func (cs *ClockSync) Clock32ToClock64(clock32 uint32) int64 {
	cs.mu.RLock()
	defer cs.mu.RUnlock()

	lastClock := cs.lastClock
	clockDiff := int32(clock32 - uint32(lastClock&0xffffffff))
	return lastClock + int64(clockDiff)
}

// IsActive returns true if the clock sync is receiving responses.
func (cs *ClockSync) IsActive() bool {
	cs.mu.RLock()
	defer cs.mu.RUnlock()

	return cs.queriesPending <= 4
}

// GetEstimate returns the current clock estimate.
func (cs *ClockSync) GetEstimate() ClockEstimate {
	cs.mu.RLock()
	defer cs.mu.RUnlock()

	return cs.clockEst
}

// GetLastClock returns the last known 64-bit MCU clock.
func (cs *ClockSync) GetLastClock() int64 {
	cs.mu.RLock()
	defer cs.mu.RUnlock()

	return cs.lastClock
}

// GetPredictionStddev returns the standard deviation of clock prediction.
func (cs *ClockSync) GetPredictionStddev() float64 {
	cs.mu.RLock()
	defer cs.mu.RUnlock()

	return math.Sqrt(cs.predictionVar)
}

// IncrementQueriesPending increments the pending query count.
func (cs *ClockSync) IncrementQueriesPending() {
	cs.mu.Lock()
	defer cs.mu.Unlock()

	cs.queriesPending++
}

// Stats returns statistics about the clock sync.
type Stats struct {
	MCUFreq       float64
	LastClock     int64
	SampleTime    float64
	Clock         int64
	Freq          float64
	MinHalfRTT    float64
	MinRTTTime    float64
	TimeAvg       float64
	TimeVariance  float64
	ClockAvg      float64
	ClockCovar    float64
	PredictionVar float64
}

// GetStats returns current statistics.
func (cs *ClockSync) GetStats() Stats {
	cs.mu.RLock()
	defer cs.mu.RUnlock()

	return Stats{
		MCUFreq:       cs.MCUFreq,
		LastClock:     cs.lastClock,
		SampleTime:    cs.clockEst.SampleTime,
		Clock:         cs.clockEst.Clock,
		Freq:          cs.clockEst.Freq,
		MinHalfRTT:    cs.minHalfRTT,
		MinRTTTime:    cs.minRTTTime,
		TimeAvg:       cs.timeAvg,
		TimeVariance:  cs.timeVariance,
		ClockAvg:      cs.clockAvg,
		ClockCovar:    cs.clockCovariance,
		PredictionVar: cs.predictionVar,
	}
}

// SecondarySync synchronizes a secondary MCU to a primary MCU.
type SecondarySync struct {
	*ClockSync
	mainSync     *ClockSync
	adjustedOff  float64
	adjustedFreq float64
	lastSyncTime float64
	mu2          sync.RWMutex
}

// NewSecondarySync creates a new secondary clock sync.
func NewSecondarySync(mainSync *ClockSync, mcuFreq float64) *SecondarySync {
	return &SecondarySync{
		ClockSync:    New(mcuFreq),
		mainSync:     mainSync,
		adjustedFreq: mcuFreq,
	}
}

// InitializeSecondary sets up the secondary sync after initial clock sync.
func (ss *SecondarySync) InitializeSecondary(curtime float64) {
	ss.mu2.Lock()
	defer ss.mu2.Unlock()

	ss.adjustedFreq = ss.MCUFreq

	mainPrintTime := ss.mainSync.EstimatedPrintTime(curtime)
	localPrintTime := ss.ClockSync.EstimatedPrintTime(curtime)
	ss.adjustedOff = mainPrintTime - localPrintTime
}

// PrintTimeToClock converts a print time to an MCU clock value for secondary.
func (ss *SecondarySync) PrintTimeToClock(printTime float64) int64 {
	ss.mu2.RLock()
	defer ss.mu2.RUnlock()

	return int64((printTime - ss.adjustedOff) * ss.adjustedFreq)
}

// ClockToPrintTime converts an MCU clock value to a print time for secondary.
func (ss *SecondarySync) ClockToPrintTime(clock int64) float64 {
	ss.mu2.RLock()
	defer ss.mu2.RUnlock()

	return float64(clock)/ss.adjustedFreq + ss.adjustedOff
}

// CalibrateClockResult contains the calibration result.
type CalibrateClockResult struct {
	AdjustedOffset float64
	AdjustedFreq   float64
}

// CalibrateClock adjusts the secondary clock to match the primary.
func (ss *SecondarySync) CalibrateClock(printTime, eventtime float64) CalibrateClockResult {
	ss.mu2.Lock()
	defer ss.mu2.Unlock()

	// Calculate: est_print_time = main_sync.estimated_print_time()
	mainEst := ss.mainSync.GetEstimate()
	mainMCUFreq := ss.mainSync.MCUFreq
	estMainClock := (eventtime-mainEst.SampleTime)*mainEst.Freq + float64(mainEst.Clock)
	estPrintTime := estMainClock / mainMCUFreq

	// Determine sync1_print_time and sync2_print_time
	sync1PrintTime := math.Max(printTime, estPrintTime)
	sync2PrintTime := math.Max(
		math.Max(sync1PrintTime+4.0, ss.lastSyncTime),
		printTime+2.5*(printTime-estPrintTime),
	)

	// Calc sync2_sys_time (inverse of main_sync.estimated_print_time)
	sync2MainClock := sync2PrintTime * mainMCUFreq
	sync2SysTime := mainEst.SampleTime + (sync2MainClock-float64(mainEst.Clock))/mainEst.Freq

	// Adjust freq so estimated print_time will match at sync2_print_time
	sync1Clock := int64((sync1PrintTime - ss.adjustedOff) * ss.adjustedFreq)
	sync2Clock := ss.ClockSync.GetClock(sync2SysTime)

	if sync2PrintTime > sync1PrintTime {
		ss.adjustedFreq = float64(sync2Clock-sync1Clock) / (sync2PrintTime - sync1PrintTime)
	}
	if ss.adjustedFreq != 0 {
		ss.adjustedOff = sync1PrintTime - float64(sync1Clock)/ss.adjustedFreq
	}

	ss.lastSyncTime = sync2PrintTime

	return CalibrateClockResult{
		AdjustedOffset: ss.adjustedOff,
		AdjustedFreq:   ss.adjustedFreq,
	}
}

// MonotonicTime provides a high-resolution monotonic time source.
type MonotonicTime struct {
	startTime time.Time
}

// NewMonotonicTime creates a new monotonic time source.
func NewMonotonicTime() *MonotonicTime {
	return &MonotonicTime{
		startTime: time.Now(),
	}
}

// Now returns the current monotonic time in seconds.
func (mt *MonotonicTime) Now() float64 {
	return time.Since(mt.startTime).Seconds()
}
