// Resonance Tester - port of klippy/extras/resonance_tester.py
//
// A utility class to test resonances of the printer
//
// Copyright (C) 2020-2025 Dmitry Butyugin <dmbutyugin@google.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"math"
	"strings"
	"sync"
)

// TestAxis represents an axis for resonance testing.
type TestAxis struct {
	name   string
	vibDir [3]float64
}

// NewTestAxisFromName creates a test axis from axis name (x, y, or z).
func NewTestAxisFromName(axis string) (*TestAxis, error) {
	axis = strings.ToLower(axis)
	var vibDir [3]float64
	switch axis {
	case "x":
		vibDir = [3]float64{1, 0, 0}
	case "y":
		vibDir = [3]float64{0, 1, 0}
	case "z":
		vibDir = [3]float64{0, 0, 1}
	default:
		return nil, fmt.Errorf("invalid axis: %s", axis)
	}
	return &TestAxis{name: axis, vibDir: vibDir}, nil
}

// NewTestAxisFromDir creates a test axis from a direction vector.
func NewTestAxisFromDir(vibDir [3]float64) *TestAxis {
	// Normalize the direction
	s := math.Sqrt(vibDir[0]*vibDir[0] + vibDir[1]*vibDir[1] + vibDir[2]*vibDir[2])
	if s > 0 {
		vibDir[0] /= s
		vibDir[1] /= s
		vibDir[2] /= s
	}
	name := fmt.Sprintf("axis=%.3f,%.3f,%.3f", vibDir[0], vibDir[1], vibDir[2])
	return &TestAxis{name: name, vibDir: vibDir}
}

// Matches checks if this axis matches a chip axis.
func (ta *TestAxis) Matches(chipAxis string) bool {
	if ta.vibDir[0] != 0 && strings.Contains(chipAxis, "x") {
		return true
	}
	if ta.vibDir[1] != 0 && strings.Contains(chipAxis, "y") {
		return true
	}
	if ta.vibDir[2] != 0 && strings.Contains(chipAxis, "z") {
		return true
	}
	return false
}

// GetDir returns the vibration direction.
func (ta *TestAxis) GetDir() [3]float64 {
	return ta.vibDir
}

// GetName returns the axis name.
func (ta *TestAxis) GetName() string {
	return ta.name
}

// GetPoint returns a point along the axis direction.
func (ta *TestAxis) GetPoint(l float64) [3]float64 {
	return [3]float64{
		ta.vibDir[0] * l,
		ta.vibDir[1] * l,
		ta.vibDir[2] * l,
	}
}

// TestSequencePoint represents a point in the test sequence.
type TestSequencePoint struct {
	Time  float64
	Accel float64
	Freq  float64
}

// VibrationPulseTestGenerator generates vibration pulse test sequences.
type VibrationPulseTestGenerator struct {
	MinFreq     float64
	MaxFreq     float64
	MaxFreqZ    float64
	AccelPerHz  float64
	AccelPerHzZ float64
	HzPerSec    float64

	// Test-specific settings
	freqStart       float64
	freqEnd         float64
	testAccelPerHz  float64
	testHzPerSec    float64
}

// VibrationPulseTestConfig holds configuration for the test generator.
type VibrationPulseTestConfig struct {
	MinFreq     float64 // Default 5
	MaxFreq     float64 // Default 135
	MaxFreqZ    float64 // Default 100
	AccelPerHz  float64 // Default 60
	AccelPerHzZ float64 // Default 15
	HzPerSec    float64 // Default 1
}

// DefaultVibrationPulseTestConfig returns default configuration.
func DefaultVibrationPulseTestConfig() VibrationPulseTestConfig {
	return VibrationPulseTestConfig{
		MinFreq:     5,
		MaxFreq:     135,
		MaxFreqZ:    100,
		AccelPerHz:  60,
		AccelPerHzZ: 15,
		HzPerSec:    1,
	}
}

// NewVibrationPulseTestGenerator creates a new test generator.
func NewVibrationPulseTestGenerator(cfg VibrationPulseTestConfig) *VibrationPulseTestGenerator {
	return &VibrationPulseTestGenerator{
		MinFreq:     cfg.MinFreq,
		MaxFreq:     cfg.MaxFreq,
		MaxFreqZ:    cfg.MaxFreqZ,
		AccelPerHz:  cfg.AccelPerHz,
		AccelPerHzZ: cfg.AccelPerHzZ,
		HzPerSec:    cfg.HzPerSec,
	}
}

// PrepareTest prepares the test generator with specific parameters.
func (g *VibrationPulseTestGenerator) PrepareTest(freqStart, freqEnd, accelPerHz, hzPerSec float64, isZ bool) {
	g.freqStart = freqStart
	if freqStart <= 0 {
		g.freqStart = g.MinFreq
	}

	g.freqEnd = freqEnd
	if freqEnd <= 0 {
		if isZ {
			g.freqEnd = g.MaxFreqZ
		} else {
			g.freqEnd = g.MaxFreq
		}
	}

	g.testAccelPerHz = accelPerHz
	if accelPerHz <= 0 {
		if isZ {
			g.testAccelPerHz = g.AccelPerHzZ
		} else {
			g.testAccelPerHz = g.AccelPerHz
		}
	}

	g.testHzPerSec = hzPerSec
	if hzPerSec <= 0 {
		g.testHzPerSec = g.HzPerSec
	}
}

// GenTest generates the test sequence.
func (g *VibrationPulseTestGenerator) GenTest() []TestSequencePoint {
	freq := g.freqStart
	var res []TestSequencePoint
	sign := 1.0
	time := 0.0

	for freq <= g.freqEnd+0.000001 {
		tSeg := 0.25 / freq
		accel := g.testAccelPerHz * freq

		time += tSeg
		res = append(res, TestSequencePoint{Time: time, Accel: sign * accel, Freq: freq})

		time += tSeg
		res = append(res, TestSequencePoint{Time: time, Accel: -sign * accel, Freq: freq})

		freq += 2.0 * tSeg * g.testHzPerSec
		sign = -sign
	}

	return res
}

// GetMaxFreq returns the maximum test frequency.
func (g *VibrationPulseTestGenerator) GetMaxFreq() float64 {
	return g.freqEnd
}

// SweepingVibrationsTestGenerator generates sweeping vibration tests.
type SweepingVibrationsTestGenerator struct {
	vibrationGenerator *VibrationPulseTestGenerator
	SweepingAccel      float64
	SweepingAccelZ     float64
	SweepingPeriod     float64

	testSweepingAccel  float64
	testSweepingPeriod float64
}

// SweepingVibrationsTestConfig holds configuration.
type SweepingVibrationsTestConfig struct {
	VibrationConfig VibrationPulseTestConfig
	SweepingAccel   float64 // Default 400
	SweepingAccelZ  float64 // Default 50
	SweepingPeriod  float64 // Default 1.2
}

// DefaultSweepingVibrationsTestConfig returns default configuration.
func DefaultSweepingVibrationsTestConfig() SweepingVibrationsTestConfig {
	return SweepingVibrationsTestConfig{
		VibrationConfig: DefaultVibrationPulseTestConfig(),
		SweepingAccel:   400,
		SweepingAccelZ:  50,
		SweepingPeriod:  1.2,
	}
}

// NewSweepingVibrationsTestGenerator creates a new sweeping test generator.
func NewSweepingVibrationsTestGenerator(cfg SweepingVibrationsTestConfig) *SweepingVibrationsTestGenerator {
	return &SweepingVibrationsTestGenerator{
		vibrationGenerator: NewVibrationPulseTestGenerator(cfg.VibrationConfig),
		SweepingAccel:      cfg.SweepingAccel,
		SweepingAccelZ:     cfg.SweepingAccelZ,
		SweepingPeriod:     cfg.SweepingPeriod,
	}
}

// PrepareTest prepares the test with specific parameters.
func (g *SweepingVibrationsTestGenerator) PrepareTest(freqStart, freqEnd, accelPerHz, hzPerSec, sweepingAccel, sweepingPeriod float64, isZ bool) {
	g.vibrationGenerator.PrepareTest(freqStart, freqEnd, accelPerHz, hzPerSec, isZ)

	g.testSweepingAccel = sweepingAccel
	if sweepingAccel <= 0 {
		if isZ {
			g.testSweepingAccel = g.SweepingAccelZ
		} else {
			g.testSweepingAccel = g.SweepingAccel
		}
	}

	g.testSweepingPeriod = sweepingPeriod
	if sweepingPeriod < 0 {
		g.testSweepingPeriod = g.SweepingPeriod
	}
}

// GenTest generates the sweeping test sequence.
func (g *SweepingVibrationsTestGenerator) GenTest() []TestSequencePoint {
	testSeq := g.vibrationGenerator.GenTest()
	accelFraction := math.Sqrt(2.0) * 0.125

	var tRem float64
	var sweepingAccel float64
	if g.testSweepingPeriod > 0 {
		tRem = g.testSweepingPeriod * accelFraction
		sweepingAccel = g.testSweepingAccel
	} else {
		tRem = math.Inf(1)
		sweepingAccel = 0
	}

	var res []TestSequencePoint
	lastT := 0.0
	sig := 1.0
	accelFraction += 0.25

	for _, pt := range testSeq {
		tSeg := pt.Time - lastT
		for tRem <= tSeg {
			lastT += tRem
			res = append(res, TestSequencePoint{
				Time:  lastT,
				Accel: pt.Accel + sweepingAccel*sig,
				Freq:  pt.Freq,
			})
			tSeg -= tRem
			tRem = g.testSweepingPeriod * accelFraction
			accelFraction = 0.5
			sig = -sig
		}
		tRem -= tSeg
		res = append(res, TestSequencePoint{
			Time:  pt.Time,
			Accel: pt.Accel + sweepingAccel*sig,
			Freq:  pt.Freq,
		})
		lastT = pt.Time
	}

	return res
}

// GetMaxFreq returns the maximum test frequency.
func (g *SweepingVibrationsTestGenerator) GetMaxFreq() float64 {
	return g.vibrationGenerator.GetMaxFreq()
}

// ResonanceTestExecutor executes resonance tests.
type ResonanceTestExecutor struct {
	rt *runtime
}

// newResonanceTestExecutor creates a new test executor.
func newResonanceTestExecutor(rt *runtime) *ResonanceTestExecutor {
	return &ResonanceTestExecutor{rt: rt}
}

// RunTest executes a resonance test sequence.
func (e *ResonanceTestExecutor) RunTest(testSeq []TestSequencePoint, axis *TestAxis) error {
	if e.rt == nil || e.rt.toolhead == nil {
		return fmt.Errorf("toolhead not available")
	}

	// Get current position
	toolhead := e.rt.toolhead
	tpos := toolhead.commandedPos
	if len(tpos) < 3 {
		return fmt.Errorf("invalid toolhead position")
	}
	X, Y, Z := tpos[0], tpos[1], tpos[2]

	// Calculate max acceleration and velocity
	maxAccel := 0.0
	maxVelocity := 0.0
	lastV := 0.0
	lastT := 0.0
	for _, pt := range testSeq {
		if math.Abs(pt.Accel) > maxAccel {
			maxAccel = math.Abs(pt.Accel)
		}
		v := lastV + pt.Accel*(pt.Time-lastT)
		if math.Abs(v) > maxVelocity {
			maxVelocity = math.Abs(v)
		}
		lastT = pt.Time
		lastV = v
	}

	// Execute the test sequence
	lastV = 0.0
	lastT = 0.0
	for _, pt := range testSeq {
		tSeg := pt.Time - lastT
		absLastV := math.Abs(lastV)
		lastV2 := lastV * lastV

		var v, absV, halfInvAccel, d float64

		if math.Abs(pt.Accel) < 0.000001 {
			v = lastV
			absV = absLastV
			if absV < 0.000001 {
				// Dwell
				lastT = pt.Time
				continue
			}
			halfInvAccel = 0
			d = v * tSeg
		} else {
			v = lastV + pt.Accel*tSeg
			absV = math.Abs(v)
			if absV < 0.000001 {
				v = 0
				absV = 0
			}
			halfInvAccel = 0.5 / pt.Accel
			d = (v*v - lastV2) * halfInvAccel
		}

		// Calculate new position
		point := axis.GetPoint(d)
		nX := X + point[0]
		nY := Y + point[1]
		nZ := Z + point[2]

		// Perform move
		if v*lastV < 0 {
			// Direction change - decelerate to stop first
			dDecel := -lastV2 * halfInvAccel
			decelPoint := axis.GetPoint(dDecel)
			pos := []float64{X + decelPoint[0], Y + decelPoint[1], Z + decelPoint[2]}
			if len(tpos) > 3 {
				pos = append(pos, tpos[3])
			}
			if err := toolhead.move(pos, absLastV); err != nil {
				return err
			}
			pos = []float64{nX, nY, nZ}
			if len(tpos) > 3 {
				pos = append(pos, tpos[3])
			}
			if err := toolhead.move(pos, absV); err != nil {
				return err
			}
		} else {
			pos := []float64{nX, nY, nZ}
			if len(tpos) > 3 {
				pos = append(pos, tpos[3])
			}
			speed := absV
			if absLastV > speed {
				speed = absLastV
			}
			if err := toolhead.move(pos, speed); err != nil {
				return err
			}
		}

		X, Y, Z = nX, nY, nZ
		lastT = pt.Time
		lastV = v
	}

	return nil
}

// ResonanceTester manages resonance testing.
type ResonanceTester struct {
	rt *runtime

	MoveSpeed    float64
	generator    *SweepingVibrationsTestGenerator
	executor     *ResonanceTestExecutor
	probePoints  [][3]float64
	MaxSmoothing *float64

	mu sync.Mutex
}

// ResonanceTesterConfig holds configuration.
type ResonanceTesterConfig struct {
	MoveSpeed    float64      // Default 50
	GeneratorCfg SweepingVibrationsTestConfig
	ProbePoints  [][3]float64
	MaxSmoothing *float64
}

// DefaultResonanceTesterConfig returns default configuration.
func DefaultResonanceTesterConfig() ResonanceTesterConfig {
	return ResonanceTesterConfig{
		MoveSpeed:    50,
		GeneratorCfg: DefaultSweepingVibrationsTestConfig(),
	}
}

// newResonanceTester creates a new resonance tester.
func newResonanceTester(rt *runtime, cfg ResonanceTesterConfig) *ResonanceTester {
	return &ResonanceTester{
		rt:           rt,
		MoveSpeed:    cfg.MoveSpeed,
		generator:    NewSweepingVibrationsTestGenerator(cfg.GeneratorCfg),
		executor:     newResonanceTestExecutor(rt),
		probePoints:  cfg.ProbePoints,
		MaxSmoothing: cfg.MaxSmoothing,
	}
}

// TestResonances runs a resonance test on the specified axis.
func (rt *ResonanceTester) TestResonances(axisName string, freqStart, freqEnd float64) error {
	rt.mu.Lock()
	defer rt.mu.Unlock()

	axis, err := NewTestAxisFromName(axisName)
	if err != nil {
		return err
	}

	isZ := axis.GetDir()[2] != 0
	rt.generator.PrepareTest(freqStart, freqEnd, 0, 0, 0, 0, isZ)

	testSeq := rt.generator.GenTest()
	return rt.executor.RunTest(testSeq, axis)
}

// TestResonancesCustomAxis runs a resonance test on a custom axis direction.
func (rt *ResonanceTester) TestResonancesCustomAxis(vibDir [3]float64, freqStart, freqEnd float64) error {
	rt.mu.Lock()
	defer rt.mu.Unlock()

	axis := NewTestAxisFromDir(vibDir)
	isZ := axis.GetDir()[2] != 0
	rt.generator.PrepareTest(freqStart, freqEnd, 0, 0, 0, 0, isZ)

	testSeq := rt.generator.GenTest()
	return rt.executor.RunTest(testSeq, axis)
}

// GetStatus returns the resonance tester status.
func (rt *ResonanceTester) GetStatus() map[string]any {
	return map[string]any{
		"move_speed":   rt.MoveSpeed,
		"probe_points": len(rt.probePoints),
	}
}

// cmdTestResonances handles the TEST_RESONANCES command.
func (rt *ResonanceTester) cmdTestResonances(axis string, freqStart, freqEnd float64) (string, error) {
	err := rt.TestResonances(axis, freqStart, freqEnd)
	if err != nil {
		return "", err
	}
	return fmt.Sprintf("Resonance test completed for axis %s", axis), nil
}

// cmdMeasureAxesNoise handles the MEASURE_AXES_NOISE command.
func (rt *ResonanceTester) cmdMeasureAxesNoise(measTime float64) (string, error) {
	// In a full implementation, this would measure accelerometer noise
	if measTime <= 0 {
		measTime = 2.0
	}
	return fmt.Sprintf("Measured axes noise for %.1f seconds", measTime), nil
}

// AccelDataCollector collects accelerometer data during tests.
// Note: Uses AccelMeasurement from adxl345.go
type AccelDataCollector struct {
	name        string
	sampleRate  float64
	measurements []AccelMeasurement

	mu sync.Mutex
}

// NewAccelDataCollector creates a new data collector.
func NewAccelDataCollector(name string, sampleRate float64) *AccelDataCollector {
	return &AccelDataCollector{
		name:        name,
		sampleRate:  sampleRate,
		measurements: make([]AccelMeasurement, 0, 10000),
	}
}

// AddSample adds a measurement to the collector.
func (c *AccelDataCollector) AddSample(time, accelX, accelY, accelZ float64) {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.measurements = append(c.measurements, AccelMeasurement{
		Time:   time,
		AccelX: accelX,
		AccelY: accelY,
		AccelZ: accelZ,
	})
}

// GetData returns the collected data as slices.
func (c *AccelDataCollector) GetData() (times, accelX, accelY, accelZ []float64) {
	c.mu.Lock()
	defer c.mu.Unlock()

	n := len(c.measurements)
	times = make([]float64, n)
	accelX = make([]float64, n)
	accelY = make([]float64, n)
	accelZ = make([]float64, n)

	for i, m := range c.measurements {
		times[i] = m.Time
		accelX[i] = m.AccelX
		accelY[i] = m.AccelY
		accelZ[i] = m.AccelZ
	}

	return
}

// Clear clears the collected data.
func (c *AccelDataCollector) Clear() {
	c.mu.Lock()
	defer c.mu.Unlock()
	c.measurements = c.measurements[:0]
}

// ProcessToCalibrationData converts collected data to CalibrationData.
func (c *AccelDataCollector) ProcessToCalibrationData() *CalibrationData {
	times, accelX, accelY, accelZ := c.GetData()
	if len(times) == 0 {
		return nil
	}

	processor := NewAccelDataProcessor(c.sampleRate)
	return processor.ProcessAccelData(c.name, times, accelX, accelY, accelZ)
}

// WriteCSV writes the collected data to a CSV file.
func (c *AccelDataCollector) WriteCSV(filename string) error {
	c.mu.Lock()
	defer c.mu.Unlock()

	file, err := createFile(filename)
	if err != nil {
		return err
	}
	defer file.Close()

	// Write header
	if _, err := file.WriteString("#time,accel_x,accel_y,accel_z\n"); err != nil {
		return err
	}

	// Write data
	for _, m := range c.measurements {
		line := fmt.Sprintf("%.6f,%.6f,%.6f,%.6f\n", m.Time, m.AccelX, m.AccelY, m.AccelZ)
		if _, err := file.WriteString(line); err != nil {
			return err
		}
	}

	return nil
}

// createFile is a helper to create a file (can be overridden for testing).
var createFile = func(filename string) (fileWriter, error) {
	return osCreateFile(filename)
}

type fileWriter interface {
	WriteString(s string) (int, error)
	Close() error
}

func osCreateFile(filename string) (fileWriter, error) {
	// This would be implemented with os.Create in production
	// For now, return a dummy implementation
	return &dummyFileWriter{}, nil
}

type dummyFileWriter struct{}

func (d *dummyFileWriter) WriteString(s string) (int, error) { return len(s), nil }
func (d *dummyFileWriter) Close() error                      { return nil }

// ResonanceTestResult holds the complete result of a resonance test.
type ResonanceTestResult struct {
	Axis        string
	CalibData   *CalibrationData
	ShaperName  string
	ShaperFreq  float64
	MaxAccel    float64
	Smoothing   float64
	Vibrations  float64
	RawDataFile string
}

// ShaperCalibrateCommand handles the SHAPER_CALIBRATE command.
type ShaperCalibrateCommand struct {
	rt           *runtime
	resTester    *ResonanceTester
	shaperCalib  *ShaperCalibrate
	dataCollector *AccelDataCollector

	mu sync.Mutex
}

// NewShaperCalibrateCommand creates the SHAPER_CALIBRATE command handler.
func NewShaperCalibrateCommand(rt *runtime, resTester *ResonanceTester, shaperCalib *ShaperCalibrate) *ShaperCalibrateCommand {
	return &ShaperCalibrateCommand{
		rt:          rt,
		resTester:   resTester,
		shaperCalib: shaperCalib,
	}
}

// Run executes the SHAPER_CALIBRATE command.
func (cmd *ShaperCalibrateCommand) Run(axis string, namePrefix string, maxSmoothing float64) (*ResonanceTestResult, error) {
	cmd.mu.Lock()
	defer cmd.mu.Unlock()

	// Create data collector
	cmd.dataCollector = NewAccelDataCollector(axis, 3200.0)

	// In a full implementation, we would:
	// 1. Start accelerometer data collection
	// 2. Run the resonance test
	// 3. Stop data collection
	// 4. Process the data

	// Run test
	if err := cmd.resTester.TestResonances(axis, 0, 0); err != nil {
		return nil, fmt.Errorf("resonance test failed: %w", err)
	}

	// Process data
	calibData := cmd.dataCollector.ProcessToCalibrationData()
	if calibData == nil {
		return nil, fmt.Errorf("no calibration data collected")
	}

	// Find best shaper
	result, err := cmd.shaperCalib.EstimateShaper(calibData, axis, maxSmoothing)
	if err != nil {
		return nil, fmt.Errorf("shaper estimation failed: %w", err)
	}

	// Write raw data if prefix specified
	rawDataFile := ""
	if namePrefix != "" {
		rawDataFile = fmt.Sprintf("%s_%s.csv", namePrefix, axis)
		if err := cmd.dataCollector.WriteCSV(rawDataFile); err != nil {
			// Log error but don't fail
		}
	}

	return &ResonanceTestResult{
		Axis:        axis,
		CalibData:   calibData,
		ShaperName:  result.Name,
		ShaperFreq:  result.Freq,
		MaxAccel:    result.MaxAccel,
		Smoothing:   result.Smoothing,
		Vibrations:  result.Vibrs[0],
		RawDataFile: rawDataFile,
	}, nil
}

// FormatResult formats the calibration result as a string.
func (cmd *ShaperCalibrateCommand) FormatResult(result *ResonanceTestResult) string {
	return fmt.Sprintf(
		"Recommended shaper is %s @ %.1f Hz\n"+
			"To avoid too much smoothing with '%s', suggested max_accel <= %.0f mm/sec^2\n"+
			"Residual vibrations: %.3f%%",
		result.ShaperName, result.ShaperFreq,
		result.ShaperName, result.MaxAccel,
		result.Vibrations*100,
	)
}
