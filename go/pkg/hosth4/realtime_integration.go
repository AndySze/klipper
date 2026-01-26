// realtime_integration provides integration between runtime and real MCU hardware.
// This enables temperature sensor ADC reading, heater PWM control, and endstop
// monitoring through the MCU communication layer.
package hosth4

import (
	"fmt"
	"io"
	"sync"
	"time"

	"klipper-go-migration/pkg/temperature"
)

// RealtimeIntegration bridges the runtime with real MCU hardware.
// It manages MCU connections and routes hardware events to runtime components.
type RealtimeIntegration struct {
	mu sync.RWMutex

	// MCU management
	mcuManager *MCUManager

	// Runtime reference (optional, for callbacks)
	runtime *runtime

	// Temperature sensor state
	tempSensors map[string]*realtimeTempSensor
	tempMu      sync.RWMutex

	// Heater PWM state
	heaterPWMs map[string]*realtimeHeaterPWM
	heaterMu   sync.RWMutex

	// Endstop state
	endstops map[int]*realtimeEndstop // keyed by OID
	endMu    sync.RWMutex

	// Fan PWM state
	fans map[string]*realtimeFanPWM
	fanMu sync.RWMutex

	// Output pin state
	outputPins map[string]*realtimeOutputPin
	pinMu      sync.RWMutex

	// Configuration
	trace io.Writer

	// Callbacks
	onShutdown []func(reason string)

	// Moonraker API server
	moonraker     *MoonrakerIntegration
	moonrakerAddr string
}

// realtimeTempSensor holds state for an MCU-connected temperature sensor.
type realtimeTempSensor struct {
	oid         int
	mcuName     string
	reportTime  float64
	sampleTime  float64
	sampleCount int

	// Thermistor for temperature conversion
	thermistor *Thermistor

	// Temperature callback
	callback temperature.TemperatureCallback

	// Last reading
	lastTemp     float64
	lastTempTime float64
	lastADC      float64

	mu sync.RWMutex
}

// realtimeHeaterPWM holds state for an MCU-connected heater PWM output.
type realtimeHeaterPWM struct {
	oid       int
	mcuName   string
	cycleTime float64
	maxPower  float64
	pin       string

	// Last PWM value
	lastValue    float64
	lastPWMTime  float64

	mu sync.RWMutex
}

// realtimeEndstop holds state for an MCU-connected endstop.
type realtimeEndstop struct {
	oid         int
	mcuName     string
	pin         string

	// Trigger state
	triggered   bool
	triggerTime float64

	// Callback
	callback func(triggered bool, triggerTime float64)

	mu sync.RWMutex
}

// realtimeFanPWM holds state for an MCU-connected fan PWM output.
type realtimeFanPWM struct {
	oid            int
	mcuName        string
	pin            string
	cycleTime      float64
	hardwarePWM    bool
	kickStartPower float64
	kickStartTime  float64
	offBelow       float64
	maxPower       float64

	// Last value
	lastValue float64
	lastTime  float64

	mu sync.RWMutex
}

// realtimeOutputPin holds state for a generic output pin.
type realtimeOutputPin struct {
	oid          int
	mcuName      string
	pin          string
	isPWM        bool
	cycleTime    float64
	maxPower     float64
	staticValue  float64

	mu sync.RWMutex
}

// NewRealtimeIntegration creates a new realtime integration layer.
func NewRealtimeIntegration() *RealtimeIntegration {
	return &RealtimeIntegration{
		mcuManager:  NewMCUManager(),
		tempSensors: make(map[string]*realtimeTempSensor),
		heaterPWMs:  make(map[string]*realtimeHeaterPWM),
		endstops:    make(map[int]*realtimeEndstop),
		fans:        make(map[string]*realtimeFanPWM),
		outputPins:  make(map[string]*realtimeOutputPin),
	}
}

// SetTrace enables debug tracing.
func (ri *RealtimeIntegration) SetTrace(w io.Writer) {
	ri.mu.Lock()
	defer ri.mu.Unlock()
	ri.trace = w
	ri.mcuManager.SetTrace(w)
}

// SetRuntime associates a runtime with this integration layer.
func (ri *RealtimeIntegration) SetRuntime(rt *runtime) {
	ri.mu.Lock()
	defer ri.mu.Unlock()
	ri.runtime = rt
}

// SetMoonrakerAddr sets the address for the Moonraker API server.
// Format: ":7125" or "0.0.0.0:7125". Empty string disables Moonraker.
func (ri *RealtimeIntegration) SetMoonrakerAddr(addr string) {
	ri.mu.Lock()
	defer ri.mu.Unlock()
	ri.moonrakerAddr = addr
}

// MCUManager returns the underlying MCU manager.
func (ri *RealtimeIntegration) MCUManager() *MCUManager {
	return ri.mcuManager
}

// AddMCU adds an MCU to the integration layer.
func (ri *RealtimeIntegration) AddMCU(name string, cfg MCUConnectionConfig) error {
	return ri.mcuManager.AddMCU(name, cfg)
}

// Connect establishes connections to all configured MCUs.
func (ri *RealtimeIntegration) Connect() error {
	if err := ri.mcuManager.Connect(); err != nil {
		return err
	}

	// Register message handlers for all MCU types
	ri.registerMessageHandlers()

	// Start Moonraker API server if configured
	ri.mu.RLock()
	moonrakerAddr := ri.moonrakerAddr
	runtime := ri.runtime
	ri.mu.RUnlock()

	if moonrakerAddr != "" {
		ri.mu.Lock()
		// Moonraker can work without runtime (with limited functionality)
		ri.moonraker = NewMoonrakerIntegration(runtime, ri.mcuManager, moonrakerAddr)
		ri.mu.Unlock()

		if err := ri.moonraker.Start(); err != nil {
			return fmt.Errorf("failed to start Moonraker server: %w", err)
		}
	}

	return nil
}

// Disconnect closes all MCU connections.
func (ri *RealtimeIntegration) Disconnect() error {
	// Stop Moonraker server if running
	ri.mu.Lock()
	if ri.moonraker != nil {
		ri.moonraker.Stop()
		ri.moonraker = nil
	}
	ri.mu.Unlock()

	return ri.mcuManager.Disconnect()
}

// Moonraker returns the MoonrakerIntegration for setting callbacks.
func (ri *RealtimeIntegration) Moonraker() *MoonrakerIntegration {
	ri.mu.RLock()
	defer ri.mu.RUnlock()
	return ri.moonraker
}

// registerMessageHandlers sets up handlers for common MCU messages.
func (ri *RealtimeIntegration) registerMessageHandlers() {
	// For each MCU, register handlers
	for _, mcuInfo := range ri.mcuManager.ListMCUs() {
		mcu := ri.mcuManager.GetMCU(mcuInfo.Name)
		if mcu == nil {
			continue
		}

		// Temperature sensor response handler
		mcu.RegisterHandler("analog_in_state", ri.handleAnalogInState)

		// Endstop state handler
		mcu.RegisterHandler("endstop_state", ri.handleEndstopState)

		// Trsync state handler (for homing)
		mcu.RegisterHandler("trsync_state", ri.handleTrsyncState)

		// Shutdown handler
		mcu.RegisterHandler("shutdown", ri.handleShutdown)

		// Clock handler (for timing)
		mcu.RegisterHandler("clock", ri.handleClock)
	}
}

// ============================================================================
// Temperature Sensor Integration
// ============================================================================

// TempSensorConfig holds configuration for a temperature sensor.
type TempSensorConfig struct {
	Name        string
	MCUName     string // MCU that hosts this sensor (default: "mcu")
	Pin         string
	Pullup      float64 // Pullup resistor value (ohms)
	Inline      float64 // Inline resistor value (ohms)
	SampleTime  float64 // Time between samples (seconds)
	SampleCount int     // Number of samples to average
	ReportTime  float64 // Time between reports (seconds)

	// Thermistor parameters (choose one)
	ThermistorType string  // Named profile (e.g., "Generic 3950")
	Beta           float64 // Beta value for simple calculation
	T1, R1         float64 // First calibration point
	T2, R2         float64 // Second calibration point
	T3, R3         float64 // Third calibration point
}

// AddTempSensor configures a temperature sensor on the MCU.
func (ri *RealtimeIntegration) AddTempSensor(cfg TempSensorConfig) error {
	ri.tempMu.Lock()
	defer ri.tempMu.Unlock()

	if cfg.MCUName == "" {
		cfg.MCUName = "mcu"
	}

	// Get or create thermistor
	var thermistor *Thermistor
	var err error

	if cfg.ThermistorType != "" {
		thermistor, err = NewThermistorFromProfile(cfg.ThermistorType, cfg.Pullup, cfg.Inline)
		if err != nil {
			return fmt.Errorf("create thermistor from profile: %w", err)
		}
	} else if cfg.Beta > 0 {
		thermistor = NewThermistor(cfg.Pullup, cfg.Inline)
		if err := thermistor.SetupCoefficientsBeta(cfg.T1, cfg.R1, cfg.Beta); err != nil {
			return fmt.Errorf("setup thermistor beta: %w", err)
		}
	} else if cfg.T1 != 0 && cfg.T2 != 0 && cfg.T3 != 0 {
		thermistor = NewThermistor(cfg.Pullup, cfg.Inline)
		if err := thermistor.SetupCoefficients(cfg.T1, cfg.R1, cfg.T2, cfg.R2, cfg.T3, cfg.R3); err != nil {
			return fmt.Errorf("setup thermistor coefficients: %w", err)
		}
	} else {
		return fmt.Errorf("no thermistor parameters provided for %s", cfg.Name)
	}

	// Default values
	if cfg.SampleTime <= 0 {
		cfg.SampleTime = 0.001 // 1ms
	}
	if cfg.SampleCount <= 0 {
		cfg.SampleCount = 8
	}
	if cfg.ReportTime <= 0 {
		cfg.ReportTime = 0.3 // 300ms
	}

	sensor := &realtimeTempSensor{
		mcuName:     cfg.MCUName,
		reportTime:  cfg.ReportTime,
		sampleTime:  cfg.SampleTime,
		sampleCount: cfg.SampleCount,
		thermistor:  thermistor,
	}

	ri.tempSensors[cfg.Name] = sensor

	ri.tracef("Added temperature sensor %s on MCU %s\n", cfg.Name, cfg.MCUName)

	return nil
}

// ConfigureTempSensorMCU sends configuration commands to the MCU for a temperature sensor.
// This should be called after Connect() when the MCU dictionary is available.
func (ri *RealtimeIntegration) ConfigureTempSensorMCU(name string, oid int) error {
	ri.tempMu.Lock()
	sensor, ok := ri.tempSensors[name]
	ri.tempMu.Unlock()

	if !ok {
		return fmt.Errorf("unknown temperature sensor: %s", name)
	}

	sensor.mu.Lock()
	sensor.oid = oid
	sensor.mu.Unlock()

	mcu := ri.mcuManager.GetMCU(sensor.mcuName)
	if mcu == nil {
		return fmt.Errorf("MCU %s not found", sensor.mcuName)
	}

	// Register OID-specific handler
	mcu.RegisterOIDHandler("analog_in_state", oid, func(params map[string]interface{}, receiveTime time.Time) {
		ri.handleTempSensorReading(name, params)
	})

	return nil
}

// SetTempSensorCallback sets the callback for temperature readings.
func (ri *RealtimeIntegration) SetTempSensorCallback(name string, callback temperature.TemperatureCallback) error {
	ri.tempMu.RLock()
	sensor, ok := ri.tempSensors[name]
	ri.tempMu.RUnlock()

	if !ok {
		return fmt.Errorf("unknown temperature sensor: %s", name)
	}

	sensor.mu.Lock()
	sensor.callback = callback
	sensor.mu.Unlock()

	return nil
}

// handleAnalogInState handles analog_in_state messages (temperature ADC readings).
func (ri *RealtimeIntegration) handleAnalogInState(params map[string]interface{}, receiveTime time.Time) {
	// This is the generic handler; OID-specific handlers will be used for individual sensors
	ri.tracef("analog_in_state: %v\n", params)
}

// handleTempSensorReading processes a temperature reading for a specific sensor.
func (ri *RealtimeIntegration) handleTempSensorReading(name string, params map[string]interface{}) {
	ri.tempMu.RLock()
	sensor, ok := ri.tempSensors[name]
	ri.tempMu.RUnlock()

	if !ok {
		return
	}

	// Extract ADC value and time from params
	nextClock, _ := params["next_clock"].(int64)
	value, _ := params["value"].(int64)

	// Get MCU to convert clock to time
	mcu := ri.mcuManager.GetMCU(sensor.mcuName)
	if mcu == nil {
		return
	}

	// Convert ADC value (typically 0-4095 for 12-bit ADC) to 0.0-1.0
	adcValue := float64(value) / 4095.0

	sensor.mu.Lock()
	sensor.lastADC = adcValue

	// Calculate temperature using thermistor
	if sensor.thermistor != nil {
		sensor.lastTemp = sensor.thermistor.CalcTemp(adcValue)
	}

	// Convert MCU clock to host time (approximate)
	sensor.lastTempTime = float64(nextClock) / mcu.MCUFreq()

	callback := sensor.callback
	temp := sensor.lastTemp
	tempTime := sensor.lastTempTime
	sensor.mu.Unlock()

	// Call the callback if registered
	if callback != nil {
		callback(tempTime, temp)
	}
}

// GetTempSensorReading returns the last temperature reading for a sensor.
func (ri *RealtimeIntegration) GetTempSensorReading(name string) (temp float64, readTime float64, err error) {
	ri.tempMu.RLock()
	sensor, ok := ri.tempSensors[name]
	ri.tempMu.RUnlock()

	if !ok {
		return 0, 0, fmt.Errorf("unknown temperature sensor: %s", name)
	}

	sensor.mu.RLock()
	temp = sensor.lastTemp
	readTime = sensor.lastTempTime
	sensor.mu.RUnlock()

	return temp, readTime, nil
}

// GetTemperature returns the current temperature for a named sensor.
// Returns the temperature and true if the sensor exists, or 0.0 and false if not.
func (ri *RealtimeIntegration) GetTemperature(name string) (float64, bool) {
	temp, _, err := ri.GetTempSensorReading(name)
	if err != nil {
		return 0.0, false
	}
	return temp, true
}

// ============================================================================
// Heater PWM Integration
// ============================================================================

// HeaterPWMConfig holds configuration for a heater PWM output.
type HeaterPWMConfig struct {
	Name      string
	MCUName   string
	Pin       string
	CycleTime float64
	MaxPower  float64
}

// AddHeaterPWM configures a heater PWM output.
func (ri *RealtimeIntegration) AddHeaterPWM(cfg HeaterPWMConfig) error {
	ri.heaterMu.Lock()
	defer ri.heaterMu.Unlock()

	if cfg.MCUName == "" {
		cfg.MCUName = "mcu"
	}
	if cfg.CycleTime <= 0 {
		cfg.CycleTime = 0.100 // 100ms default
	}
	if cfg.MaxPower <= 0 {
		cfg.MaxPower = 1.0
	}

	pwm := &realtimeHeaterPWM{
		mcuName:   cfg.MCUName,
		cycleTime: cfg.CycleTime,
		maxPower:  cfg.MaxPower,
		pin:       cfg.Pin,
	}

	ri.heaterPWMs[cfg.Name] = pwm

	ri.tracef("Added heater PWM %s on MCU %s pin %s\n", cfg.Name, cfg.MCUName, cfg.Pin)

	return nil
}

// ConfigureHeaterPWMMCU sends configuration commands to the MCU for a heater PWM.
func (ri *RealtimeIntegration) ConfigureHeaterPWMMCU(name string, oid int) error {
	ri.heaterMu.Lock()
	pwm, ok := ri.heaterPWMs[name]
	ri.heaterMu.Unlock()

	if !ok {
		return fmt.Errorf("unknown heater PWM: %s", name)
	}

	pwm.mu.Lock()
	pwm.oid = oid
	pwm.mu.Unlock()

	return nil
}

// SetHeaterPWM sets the PWM value for a heater.
func (ri *RealtimeIntegration) SetHeaterPWM(name string, pwmTime, value float64) error {
	ri.heaterMu.RLock()
	pwm, ok := ri.heaterPWMs[name]
	ri.heaterMu.RUnlock()

	if !ok {
		return fmt.Errorf("unknown heater PWM: %s", name)
	}

	pwm.mu.Lock()
	oid := pwm.oid
	mcuName := pwm.mcuName
	maxPower := pwm.maxPower
	pwm.lastValue = value
	pwm.lastPWMTime = pwmTime
	pwm.mu.Unlock()

	// Clamp value to max power
	if value > maxPower {
		value = maxPower
	}

	// Get MCU and convert time to clock
	mcu := ri.mcuManager.GetMCU(mcuName)
	if mcu == nil {
		return fmt.Errorf("MCU %s not found", mcuName)
	}

	clock := mcu.PrintTime(pwmTime)

	// Send PWM command
	// The exact command depends on the MCU firmware
	// For soft PWM: schedule_digital_out or pwm_out commands
	return ri.mcuManager.SendCommandToMCU(mcuName, "schedule_pwm_out", map[string]interface{}{
		"oid":   oid,
		"clock": clock,
		"value": int(value * 255), // Convert to 8-bit PWM value
	})
}

// ============================================================================
// Endstop Integration
// ============================================================================

// EndstopConfig holds configuration for an endstop.
type EndstopConfig struct {
	Name       string
	MCUName    string
	Pin        string
	Pullup     bool
	InvertLogic bool
}

// AddEndstop configures an endstop on the MCU.
func (ri *RealtimeIntegration) AddEndstop(cfg EndstopConfig) error {
	ri.endMu.Lock()
	defer ri.endMu.Unlock()

	if cfg.MCUName == "" {
		cfg.MCUName = "mcu"
	}

	// OID will be assigned during ConfigureEndstopMCU
	es := &realtimeEndstop{
		mcuName: cfg.MCUName,
		pin:     cfg.Pin,
	}

	// Use name as temporary key until OID is assigned
	// This is a simplification - in production, we'd use a proper name->OID mapping
	ri.tracef("Added endstop %s on MCU %s pin %s\n", cfg.Name, cfg.MCUName, cfg.Pin)
	_ = es // Suppress unused variable warning for now

	return nil
}

// ConfigureEndstopMCU sends configuration commands to the MCU for an endstop.
func (ri *RealtimeIntegration) ConfigureEndstopMCU(name string, oid int) error {
	ri.endMu.Lock()
	defer ri.endMu.Unlock()

	es := &realtimeEndstop{
		oid: oid,
	}
	ri.endstops[oid] = es

	// Register OID-specific handler
	mcu := ri.mcuManager.PrimaryMCU()
	if mcu != nil {
		mcu.RegisterOIDHandler("endstop_state", oid, func(params map[string]interface{}, receiveTime time.Time) {
			ri.handleEndstopStateOID(oid, params)
		})
	}

	return nil
}

// SetEndstopCallback sets the callback for endstop triggers.
func (ri *RealtimeIntegration) SetEndstopCallback(oid int, callback func(triggered bool, triggerTime float64)) error {
	ri.endMu.RLock()
	es, ok := ri.endstops[oid]
	ri.endMu.RUnlock()

	if !ok {
		return fmt.Errorf("unknown endstop OID: %d", oid)
	}

	es.mu.Lock()
	es.callback = callback
	es.mu.Unlock()

	return nil
}

// handleEndstopState handles generic endstop_state messages.
func (ri *RealtimeIntegration) handleEndstopState(params map[string]interface{}, receiveTime time.Time) {
	ri.tracef("endstop_state: %v\n", params)
}

// handleEndstopStateOID handles endstop_state messages for a specific OID.
func (ri *RealtimeIntegration) handleEndstopStateOID(oid int, params map[string]interface{}) {
	ri.endMu.RLock()
	es, ok := ri.endstops[oid]
	ri.endMu.RUnlock()

	if !ok {
		return
	}

	// Extract trigger state
	triggered := false
	if pin, ok := params["pin_value"].(int64); ok {
		triggered = pin != 0
	}

	es.mu.Lock()
	es.triggered = triggered
	callback := es.callback
	triggerTime := es.triggerTime
	es.mu.Unlock()

	if callback != nil {
		callback(triggered, triggerTime)
	}
}

// handleTrsyncState handles trsync_state messages for homing.
func (ri *RealtimeIntegration) handleTrsyncState(params map[string]interface{}, receiveTime time.Time) {
	ri.tracef("trsync_state: %v\n", params)

	// Extract trigger reason
	reason, _ := params["trigger_reason"].(int64)
	if reason == trsyncExpireReason {
		ri.tracef("trsync expired\n")
	}
}

// handleShutdown handles MCU shutdown messages.
func (ri *RealtimeIntegration) handleShutdown(params map[string]interface{}, receiveTime time.Time) {
	reason := "unknown"
	if r, ok := params["reason"].(string); ok {
		reason = r
	}

	ri.tracef("MCU shutdown: %s\n", reason)

	// Call shutdown callbacks
	ri.mu.RLock()
	callbacks := ri.onShutdown
	ri.mu.RUnlock()

	for _, cb := range callbacks {
		cb(reason)
	}
}

// handleClock handles clock messages for timing synchronization.
func (ri *RealtimeIntegration) handleClock(params map[string]interface{}, receiveTime time.Time) {
	// Clock synchronization is handled by MCUConnection internally
}

// ============================================================================
// Fan PWM Integration
// ============================================================================

// FanPWMConfig holds configuration for a fan PWM output.
type FanPWMConfig struct {
	Name           string
	MCUName        string
	Pin            string
	CycleTime      float64
	HardwarePWM    bool
	KickStartPower float64
	KickStartTime  float64
	OffBelow       float64
	MaxPower       float64
}

// AddFanPWM configures a fan PWM output.
func (ri *RealtimeIntegration) AddFanPWM(cfg FanPWMConfig) error {
	ri.fanMu.Lock()
	defer ri.fanMu.Unlock()

	if cfg.MCUName == "" {
		cfg.MCUName = "mcu"
	}
	if cfg.CycleTime <= 0 {
		cfg.CycleTime = 0.010 // 10ms default (100Hz)
	}
	if cfg.MaxPower <= 0 {
		cfg.MaxPower = 1.0
	}

	fan := &realtimeFanPWM{
		mcuName:        cfg.MCUName,
		pin:            cfg.Pin,
		cycleTime:      cfg.CycleTime,
		hardwarePWM:    cfg.HardwarePWM,
		kickStartPower: cfg.KickStartPower,
		kickStartTime:  cfg.KickStartTime,
		offBelow:       cfg.OffBelow,
		maxPower:       cfg.MaxPower,
	}

	ri.fans[cfg.Name] = fan

	ri.tracef("Added fan PWM %s on MCU %s pin %s\n", cfg.Name, cfg.MCUName, cfg.Pin)

	return nil
}

// SetFanPWM sets the PWM value for a fan.
func (ri *RealtimeIntegration) SetFanPWM(name string, pwmTime, value float64) error {
	ri.fanMu.RLock()
	fan, ok := ri.fans[name]
	ri.fanMu.RUnlock()

	if !ok {
		return fmt.Errorf("unknown fan: %s", name)
	}

	fan.mu.Lock()
	oid := fan.oid
	mcuName := fan.mcuName
	maxPower := fan.maxPower
	offBelow := fan.offBelow
	fan.lastValue = value
	fan.lastTime = pwmTime
	fan.mu.Unlock()

	// Apply off_below threshold
	if value > 0 && value < offBelow {
		value = 0
	}

	// Clamp value to max power
	if value > maxPower {
		value = maxPower
	}

	// Get MCU and convert time to clock
	mcu := ri.mcuManager.GetMCU(mcuName)
	if mcu == nil {
		return fmt.Errorf("MCU %s not found", mcuName)
	}

	clock := mcu.PrintTime(pwmTime)

	// Send PWM command
	return ri.mcuManager.SendCommandToMCU(mcuName, "schedule_pwm_out", map[string]interface{}{
		"oid":   oid,
		"clock": clock,
		"value": int(value * 255),
	})
}

// ============================================================================
// Output Pin Integration
// ============================================================================

// OutputPinConfig holds configuration for a generic output pin.
type OutputPinConfig struct {
	Name        string
	MCUName     string
	Pin         string
	IsPWM       bool
	CycleTime   float64
	MaxPower    float64
	StaticValue float64
}

// AddOutputPin configures a generic output pin.
func (ri *RealtimeIntegration) AddOutputPin(cfg OutputPinConfig) error {
	ri.pinMu.Lock()
	defer ri.pinMu.Unlock()

	if cfg.MCUName == "" {
		cfg.MCUName = "mcu"
	}
	if cfg.MaxPower <= 0 {
		cfg.MaxPower = 1.0
	}

	pin := &realtimeOutputPin{
		mcuName:     cfg.MCUName,
		pin:         cfg.Pin,
		isPWM:       cfg.IsPWM,
		cycleTime:   cfg.CycleTime,
		maxPower:    cfg.MaxPower,
		staticValue: cfg.StaticValue,
	}

	ri.outputPins[cfg.Name] = pin

	ri.tracef("Added output pin %s on MCU %s pin %s (PWM=%v)\n", cfg.Name, cfg.MCUName, cfg.Pin, cfg.IsPWM)

	return nil
}

// SetOutputPin sets the value for an output pin.
func (ri *RealtimeIntegration) SetOutputPin(name string, printTime, value float64) error {
	ri.pinMu.RLock()
	pin, ok := ri.outputPins[name]
	ri.pinMu.RUnlock()

	if !ok {
		return fmt.Errorf("unknown output pin: %s", name)
	}

	pin.mu.Lock()
	oid := pin.oid
	mcuName := pin.mcuName
	isPWM := pin.isPWM
	maxPower := pin.maxPower
	pin.mu.Unlock()

	// Clamp value
	if value > maxPower {
		value = maxPower
	}

	// Get MCU
	mcu := ri.mcuManager.GetMCU(mcuName)
	if mcu == nil {
		return fmt.Errorf("MCU %s not found", mcuName)
	}

	clock := mcu.PrintTime(printTime)

	if isPWM {
		return ri.mcuManager.SendCommandToMCU(mcuName, "schedule_pwm_out", map[string]interface{}{
			"oid":   oid,
			"clock": clock,
			"value": int(value * 255),
		})
	}

	// Digital output
	digitalValue := 0
	if value >= 0.5 {
		digitalValue = 1
	}
	return ri.mcuManager.SendCommandToMCU(mcuName, "schedule_digital_out", map[string]interface{}{
		"oid":   oid,
		"clock": clock,
		"value": digitalValue,
	})
}

// ============================================================================
// Utility Methods
// ============================================================================

// OnShutdown registers a callback for MCU shutdown events.
func (ri *RealtimeIntegration) OnShutdown(callback func(reason string)) {
	ri.mu.Lock()
	defer ri.mu.Unlock()
	ri.onShutdown = append(ri.onShutdown, callback)
}

// Run starts the MCU manager's reactor event loop.
func (ri *RealtimeIntegration) Run() {
	ri.mcuManager.Run()
}

// End stops the reactor event loop.
func (ri *RealtimeIntegration) End() {
	ri.mcuManager.End()
}

// WaitReady waits until all MCUs are ready.
func (ri *RealtimeIntegration) WaitReady(timeout time.Duration) error {
	return ri.mcuManager.WaitReady(timeout)
}

// GetStatus returns status information about the realtime integration.
func (ri *RealtimeIntegration) GetStatus() map[string]interface{} {
	ri.mu.RLock()
	defer ri.mu.RUnlock()

	ri.tempMu.RLock()
	sensors := make(map[string]interface{})
	for name, sensor := range ri.tempSensors {
		sensor.mu.RLock()
		sensors[name] = map[string]interface{}{
			"temperature": sensor.lastTemp,
			"read_time":   sensor.lastTempTime,
			"adc":         sensor.lastADC,
		}
		sensor.mu.RUnlock()
	}
	ri.tempMu.RUnlock()

	ri.heaterMu.RLock()
	heaters := make(map[string]interface{})
	for name, pwm := range ri.heaterPWMs {
		pwm.mu.RLock()
		heaters[name] = map[string]interface{}{
			"pwm_value": pwm.lastValue,
			"pwm_time":  pwm.lastPWMTime,
		}
		pwm.mu.RUnlock()
	}
	ri.heaterMu.RUnlock()

	return map[string]interface{}{
		"mcu":      ri.mcuManager.GetStatus(),
		"sensors":  sensors,
		"heaters":  heaters,
	}
}

// tracef writes trace output if tracing is enabled.
func (ri *RealtimeIntegration) tracef(format string, args ...interface{}) {
	ri.mu.RLock()
	trace := ri.trace
	ri.mu.RUnlock()

	if trace != nil {
		fmt.Fprintf(trace, format, args...)
	}
}
