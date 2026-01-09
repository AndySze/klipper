package hosth4

import (
	"fmt"
	"sync"
	"time"

	"klipper-go-migration/pkg/temperature"
)

// mcuPWMAdapter implements temperature.PWMInterface using hosth4's MCU
type mcuPWMAdapter struct {
	mcu        *mcu
	pin        string
	pwmCycle   float64
	maxPower   float64
	lastPWM    float64
	lastTime   float64
	mu         sync.Mutex
	cmdID      int32
}

// newMcuPWMAdapter creates a new PWM adapter
func newMcuPWMAdapter(mcu *mcu, pin string, maxPower float64, pwmCycleTime float64) (*mcuPWMAdapter, error) {
	if mcu == nil {
		return nil, fmt.Errorf("mcu is nil")
	}
	return &mcuPWMAdapter{
		mcu:      mcu,
		pin:      pin,
		maxPower: maxPower,
		pwmCycle: pwmCycleTime,
	}, nil
}

func (a *mcuPWMAdapter) SetPWM(pwmTime, value float64) error {
	a.mu.Lock()
	defer a.mu.Unlock()

	// Apply max power limit
	if value > a.maxPower {
		value = a.maxPower
	}

	a.lastPWM = value
	a.lastTime = pwmTime

	// In golden test mode, we don't actually send MCU commands
	// This will be implemented when we have full MCU support
	return nil
}

func (a *mcuPWMAdapter) SetupCycleTime(cycleTime float64) error {
	a.mu.Lock()
	defer a.mu.Unlock()
	a.pwmCycle = cycleTime
	return nil
}

func (a *mcuPWMAdapter) SetupMaxDuration(maxDuration float64) error {
	// MCU-specific implementation
	return nil
}

func (a *mcuPWMAdapter) GetMCU() temperature.MCUInterface {
	return a.mcu
}

// mcuADCAdapter implements temperature.ADCInterface using hosth4's MCU
type mcuADCAdapter struct {
	mcu          *mcu
	pin          string
	reportTime   float64
	sampleTime   float64
	sampleCount  int
	callback     temperature.TemperatureCallback
	calibration  struct {
		baseTemp float64
		slope    float64
	}
	mu              sync.Mutex
	lastADCValue    float64
	lastTemp        float64
	started         bool
}

// newMcuADCAdapter creates a new ADC adapter
func newMcuADCAdapter(mcu *mcu, pin string) (*mcuADCAdapter, error) {
	if mcu == nil {
		return nil, fmt.Errorf("mcu is nil")
	}

	return &mcuADCAdapter{
		mcu:         mcu,
		pin:         pin,
		reportTime:  temperature.ReportTime,
		sampleTime:  temperature.SampleTime,
		sampleCount: temperature.SampleCount,
	}, nil
}

func (a *mcuADCAdapter) SetupADCCallback(reportTime float64, callback func(float64, float64)) {
	a.mu.Lock()
	defer a.mu.Unlock()

	a.reportTime = reportTime
	a.callback = callback
	a.started = true

	// Start background ADC reader
	go a.adcReaderLoop()
}

func (a *mcuADCAdapter) SetupADCSample(sampleTime float64, sampleCount int, options ...temperature.ADCSampleOption) error {
	a.mu.Lock()
	defer a.mu.Unlock()

	a.sampleTime = sampleTime
	a.sampleCount = sampleCount
	return nil
}

func (a *mcuADCAdapter) GetMCU() temperature.MCUInterface {
	return a.mcu
}

// adcReaderLoop simulates ADC readings in golden test mode
func (a *mcuADCAdapter) adcReaderLoop() {
	ticker := time.NewTicker(time.Duration(a.reportTime * float64(time.Second)))
	defer ticker.Stop()

	for range ticker.C {
		a.mu.Lock()

		// In golden test mode, return a simulated value
		// Real implementation would read from MCU
		if a.callback != nil {
			// Simulate a temperature reading around 25°C (ambient)
			adcValue := 0.5 // Mock ADC value (0.0 - 1.0)
			readTime := float64(time.Now().UnixNano()) / 1e9

			a.callback(readTime, adcValue)
		}

		a.mu.Unlock()
	}
}

// printerAdapter implements temperature.PrinterInterface for hosth4
type printerAdapter struct {
	runtime *runtime
}

func newPrinterAdapter(rt *runtime) *printerAdapter {
	return &printerAdapter{runtime: rt}
}

func (p *printerAdapter) IsShutdown() bool {
	// In golden test mode, we're never shut down
	return false
}

func (p *printerAdapter) CommandError(format string, args ...interface{}) error {
	return fmt.Errorf(format, args...)
}

// gcodeAdapter implements temperature.GCodeInterface for hosth4
type gcodeAdapter struct {
	runtime *runtime
}

func newGcodeAdapter(rt *runtime) *gcodeAdapter {
	return &gcodeAdapter{runtime: rt}
}

func (g *gcodeAdapter) Respond(msg string) {
	g.runtime.tracef("GCODE RESP: %s\n", msg)
}

func (g *gcodeAdapter) RespondRaw(msg string) {
	g.runtime.tracef("GCODE RESP RAW: %s\n", msg)
}

func (g *gcodeAdapter) GetFloat(args map[string]string, param string, defaultValue float64) float64 {
	raw, ok := args[param]
	if !ok || raw == "" {
		return defaultValue
	}

	var f float64
	if _, err := fmt.Sscanf(raw, "%f", &f); err != nil {
		return defaultValue
	}
	return f
}

// heaterConfig holds parsed heater configuration from config file
type heaterConfig struct {
	name           string
	heaterPin      string
	sensorType     string
	sensorPin      string
	control        string
	minTemp        float64
	maxTemp        float64
	pidKp          float64
	pidKi          float64
	pidKd          float64
	maxDelta       float64
	maxPower       float64
	smoothTime     float64
	pwmCycleTime   float64
	minExtrudeTemp float64
}

// parseHeaterConfig extracts heater configuration from config section
func parseHeaterConfig(name string, section map[string]string) (*heaterConfig, error) {
	cfg := &heaterConfig{
		name:           name,
		control:        "pid",       // default
		minTemp:        0.0,         // default
		maxTemp:        250.0,       // default
		maxPower:       1.0,         // default
		smoothTime:     1.0,         // default
		pwmCycleTime:   0.1,         // default (100ms)
		minExtrudeTemp: 170.0,       // default
		maxDelta:       2.0,         // default for watermark
	}

	// Parse required fields
	if heaterPin, ok := section["heater_pin"]; ok {
		cfg.heaterPin = heaterPin
	} else {
		return nil, fmt.Errorf("missing heater_pin")
	}

	if sensorType, ok := section["sensor_type"]; ok {
		cfg.sensorType = sensorType
	} else {
		return nil, fmt.Errorf("missing sensor_type")
	}

	if sensorPin, ok := section["sensor_pin"]; ok {
		cfg.sensorPin = sensorPin
	} else {
		return nil, fmt.Errorf("missing sensor_pin")
	}

	// Parse optional fields
	if control, ok := section["control"]; ok {
		cfg.control = control
	}

	if minTemp, ok := section["min_temp"]; ok {
		if _, err := fmt.Sscanf(minTemp, "%f", &cfg.minTemp); err != nil {
			return nil, fmt.Errorf("invalid min_temp: %s", minTemp)
		}
	}

	if maxTemp, ok := section["max_temp"]; ok {
		if _, err := fmt.Sscanf(maxTemp, "%f", &cfg.maxTemp); err != nil {
			return nil, fmt.Errorf("invalid max_temp: %s", maxTemp)
		}
	}

	// PID parameters
	if cfg.control == "pid" {
		if pidKp, ok := section["pid_kp"]; ok {
			if _, err := fmt.Sscanf(pidKp, "%f", &cfg.pidKp); err != nil {
				return nil, fmt.Errorf("invalid pid_kp: %s", pidKp)
			}
		} else {
			return nil, fmt.Errorf("pid control requires pid_kp")
		}

		if pidKi, ok := section["pid_ki"]; ok {
			if _, err := fmt.Sscanf(pidKi, "%f", &cfg.pidKi); err != nil {
				return nil, fmt.Errorf("invalid pid_ki: %s", pidKi)
			}
		} else {
			return nil, fmt.Errorf("pid control requires pid_ki")
		}

		if pidKd, ok := section["pid_kd"]; ok {
			if _, err := fmt.Sscanf(pidKd, "%f", &cfg.pidKd); err != nil {
				return nil, fmt.Errorf("invalid pid_kd: %s", pidKd)
			}
		} else {
			return nil, fmt.Errorf("pid control requires pid_kd")
		}
	}

	// Watermark max_delta
	if cfg.control == "watermark" {
		if maxDelta, ok := section["max_delta"]; ok {
			if _, err := fmt.Sscanf(maxDelta, "%f", &cfg.maxDelta); err != nil {
				return nil, fmt.Errorf("invalid max_delta: %s", maxDelta)
			}
		}
	}

	return cfg, nil
}
