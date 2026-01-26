// Package gcode provides G-code execution for 3D printers.
// It manages MCU hardware configuration and executes G-code commands.
package gcode

import (
	"fmt"
	"log"
	"math"
	"regexp"
	"strconv"
	"strings"
	"sync"
	"time"

	"klipper-go-migration/pkg/config"
	"klipper-go-migration/pkg/hosth4"
)

// Executor handles G-code execution with real MCU hardware.
type Executor struct {
	mu sync.RWMutex

	// Configuration
	config *config.PrinterConfig
	ri     *hosth4.RealtimeIntegration

	// MCU state
	mcu          *hosth4.MCUConnection
	mcuFreq      float64
	isConfigured bool

	// OID assignments
	stepperOids map[string]int // stepper name -> OID
	enableOids  map[string]int // stepper name -> enable pin OID
	endstopOids map[string]int // stepper name -> endstop OID
	trsyncOids  map[string]int // stepper name -> trsync OID
	adcOids     map[string]int // sensor name -> ADC OID
	pwmOids     map[string]int // heater/fan name -> PWM OID
	nextOid     int

	// Current position (in mm, after homing)
	position   [4]float64 // X, Y, Z, E
	homedAxes  [3]bool    // X, Y, Z homed status
	feedrate   float64    // Current feedrate (mm/s)
	absCoords  bool       // Absolute coordinates mode
	absExtrude bool       // Absolute extrusion mode

	// Temperature state
	extruderTemp   float64
	extruderTarget float64
	bedTemp        float64
	bedTarget      float64

	// Motion constants
	stepPulseTicks int // Step pulse duration in MCU ticks

	// Temperature reading
	tempStopCh chan struct{}
	// Track whether we've seen real ADC readings (to disable simulation fallback).
	realExtruderReading bool
	realBedReading      bool
}

// NewExecutor creates a new G-code executor.
func NewExecutor(cfg *config.PrinterConfig, ri *hosth4.RealtimeIntegration) *Executor {
	return &Executor{
		config:         cfg,
		ri:             ri,
		stepperOids:    make(map[string]int),
		enableOids:     make(map[string]int),
		endstopOids:    make(map[string]int),
		trsyncOids:     make(map[string]int),
		adcOids:        make(map[string]int),
		pwmOids:        make(map[string]int),
		feedrate:       25.0, // Default 25 mm/s
		absCoords:      true,
		absExtrude:     false,
		stepPulseTicks: 32, // 2 microseconds at 16MHz
	}
}

// Configure sends hardware configuration to the MCU.
// This must be called after MCU connection is established.
func (e *Executor) Configure() error {
	e.mu.Lock()
	defer e.mu.Unlock()

	if e.isConfigured {
		return nil
	}

	e.mcu = e.ri.MCUManager().PrimaryMCU()
	if e.mcu == nil {
		return fmt.Errorf("no MCU connected")
	}

	e.mcuFreq = e.mcu.MCUFreq()

	// Check if MCU is in shutdown state and clear it if needed
	// This can happen if the MCU was shutdown from a previous session
	for attempts := 0; attempts < 3; attempts++ {
		if e.mcu.IsShutdown() {
			log.Printf("MCU is in shutdown state, clearing (attempt %d)...", attempts+1)
			if err := e.mcu.SendCommand("clear_shutdown", nil); err != nil {
				log.Printf("Warning: clear_shutdown failed: %v", err)
			}
			time.Sleep(100 * time.Millisecond)
		}
	}

	// Check if MCU is already configured
	resp, err := e.mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
	if err != nil {
		return fmt.Errorf("get_config: %w", err)
	}

	// Check if MCU reports being in shutdown state
	if isShutdown, ok := resp["is_shutdown"].(int); ok && isShutdown != 0 {
		log.Println("MCU reports shutdown state, clearing...")
		if err := e.mcu.SendCommand("clear_shutdown", nil); err != nil {
			return fmt.Errorf("clear_shutdown: %w", err)
		}
		// Re-query config after clearing shutdown
		time.Sleep(100 * time.Millisecond)
		resp, err = e.mcu.SendCommandWithResponse("get_config", nil, "config", 2*time.Second)
		if err != nil {
			return fmt.Errorf("get_config after clear: %w", err)
		}
	}

	if v, ok := resp["is_config"].(int); ok && v != 0 {
		log.Println("MCU already configured")
		e.isConfigured = true
		return nil
	}

	// Count required OIDs
	oidCount := 0
	// Steppers: stepper + enable + endstop + trsync per axis
	oidCount += len(e.config.Steppers) * 4
	// Extruder: stepper + enable + heater PWM + temp ADC
	if e.config.Extruder != nil {
		oidCount += 4
	}
	// Bed heater: heater PWM + temp ADC
	if e.config.HeaterBed != nil {
		oidCount += 2
	}
	// Fans
	oidCount += len(e.config.Fans)

	log.Printf("Allocating %d OIDs for hardware configuration", oidCount)

	// Allocate OIDs
	if err := e.mcu.SendCommand("allocate_oids", map[string]interface{}{"count": oidCount}); err != nil {
		return fmt.Errorf("allocate_oids: %w", err)
	}

	// Configure steppers
	for name, stepper := range e.config.Steppers {
		if err := e.configureStepper(name, stepper); err != nil {
			return fmt.Errorf("configure stepper %s: %w", name, err)
		}
	}

	// Configure extruder (if present)
	if e.config.Extruder != nil {
		if err := e.configureExtruder(); err != nil {
			return fmt.Errorf("configure extruder: %w", err)
		}
	}

	// Configure bed heater (if present)
	if e.config.HeaterBed != nil {
		if err := e.configureHeaterBed(); err != nil {
			return fmt.Errorf("configure heater_bed: %w", err)
		}
	}

	// Configure fans
	for name, fan := range e.config.Fans {
		if err := e.configureFan(name, fan); err != nil {
			return fmt.Errorf("configure fan %s: %w", name, err)
		}
	}

	// Finalize configuration
	if err := e.mcu.SendCommand("finalize_config", map[string]interface{}{"crc": 0}); err != nil {
		return fmt.Errorf("finalize_config: %w", err)
	}

	e.isConfigured = true
	log.Println("MCU configuration complete")

	// Start temperature reading goroutine
	e.startTempReading()

	return nil
}

// startTempReading starts temperature sensor reading.
// Temperature will show 0 until real ADC readings are received from MCU.
func (e *Executor) startTempReading() {
	if len(e.adcOids) == 0 {
		return
	}

	// Register handler for ADC responses
	e.mcu.RegisterHandler("analog_in_state", func(params map[string]interface{}, receiveTime time.Time) {
		e.handleADCResponse(params)
	})

	e.tempStopCh = make(chan struct{})

	// Start real ADC sampling after a delay
	go func() {
		time.Sleep(2 * time.Second)
		e.startADCSampling()
	}()

	log.Println("Temperature reading started")
}

// startADCSampling sends query_analog_in commands ONCE to start continuous sampling.
// The MCU will then continuously send analog_in_state responses.
func (e *Executor) startADCSampling() {
	// Get current clock
	clock := e.mcu.GetLastMCUClock()
	if clock == 0 {
		log.Println("Warning: MCU clock not available for ADC sampling")
		return
	}

	// Sample parameters for continuous reading
	// At 16MHz: sample_ticks=16000 (1ms), rest_ticks=8000000 (500ms between reports)
	sampleTicks := int(e.mcuFreq * 0.001) // 1ms per sample
	sampleCount := 8                      // 8 samples averaged
	restTicks := int(e.mcuFreq * 0.5)     // 500ms between reports (continuous)
	maxADC := 1023 * sampleCount          // AVR ADC is 10-bit; analog_in_state is sum of samples

	for name, oid := range e.adcOids {
		// Start 2 seconds from now to ensure clock is definitely in future
		// The MCU clock might be slightly out of sync, so we use a large margin
		startClock := clock + uint64(e.mcuFreq*2.0)

		log.Printf("ADC sampling started for %s (oid=%d)", name, oid)

		err := e.mcu.SendCommand("query_analog_in", map[string]interface{}{
			"oid":               oid,
			"clock":             int(startClock),
			"sample_ticks":      sampleTicks,
			"sample_count":      sampleCount,
			"rest_ticks":        restTicks,
			"min_value":         0,
			"max_value":         maxADC,
			"range_check_count": 0,
		})
		if err != nil {
			log.Printf("Warning: ADC query failed for %s: %v", name, err)
		}
	}
}

// handleADCResponse processes analog_in_state responses from MCU.
func (e *Executor) handleADCResponse(params map[string]interface{}) {
	oid, ok := params["oid"].(int)
	if !ok {
		return
	}

	value, ok := params["value"].(int)
	if !ok {
		return
	}

	e.mu.Lock()
	defer e.mu.Unlock()

	// MCU analog_in_state "value" is sum of sample_count readings.
	// We use the same sample_count as startADCSampling().
	maxADC := float64(1023 * 8)
	if maxADC <= 0 {
		return
	}
	adcFraction := float64(value) / maxADC

	// Find which sensor this OID belongs to
	for name, sensorOid := range e.adcOids {
		if sensorOid == oid {
			// Convert ADC value to temperature
			var sensorType string
			if name == "extruder" && e.config.Extruder != nil {
				sensorType = e.config.Extruder.SensorType
				e.extruderTemp = e.adcToTemperature(adcFraction, sensorType)
				e.realExtruderReading = true
				log.Printf("Extruder temp: ADC=%d -> %.1f°C", value, e.extruderTemp)
			} else if name == "heater_bed" && e.config.HeaterBed != nil {
				sensorType = e.config.HeaterBed.SensorType
				e.bedTemp = e.adcToTemperature(adcFraction, sensorType)
				e.realBedReading = true
				log.Printf("Bed temp: ADC=%d -> %.1f°C", value, e.bedTemp)
			}
			return
		}
	}
}

// StopTempReading stops the temperature reading goroutine.
func (e *Executor) StopTempReading() {
	if e.tempStopCh != nil {
		close(e.tempStopCh)
		e.tempStopCh = nil
	}
}

// adcToTemperature converts ADC fraction (0.0-1.0) to temperature.
func (e *Executor) adcToTemperature(adcFraction float64, sensorType string) float64 {
	// Thermistor is part of voltage divider with pullup to Vref and thermistor to GND:
	// adcFraction = R_thermistor / (R_pullup + R_thermistor)
	// => R_thermistor = R_pullup * adcFraction / (1 - adcFraction)
	if adcFraction <= 0.0 || adcFraction >= 1.0 {
		return -273.15
	}
	pullupResistance := 4700.0
	resistance := pullupResistance * adcFraction / (1.0 - adcFraction)

	// Use Steinhart-Hart equation with standard thermistor coefficients
	// For NTC 100K (EPCOS or similar)
	var temp float64
	switch sensorType {
	case "EPCOS 100K B57560G104F":
		// EPCOS 100K NTC: Beta = 4092
		beta := 4092.0
		r25 := 100000.0 // Resistance at 25°C
		temp = 1.0 / (1.0/298.15 + math.Log(resistance/r25)/beta)
		temp -= 273.15 // Convert to Celsius
	case "ATC Semitec 104GT-2":
		// ATC Semitec 104GT-2: Beta ≈ 4267
		beta := 4267.0
		r25 := 100000.0
		temp = 1.0 / (1.0/298.15 + math.Log(resistance/r25)/beta)
		temp -= 273.15
	default:
		// Generic 100K NTC
		beta := 3950.0
		r25 := 100000.0
		temp = 1.0 / (1.0/298.15 + math.Log(resistance/r25)/beta)
		temp -= 273.15
	}

	return temp
}

// configureStepper configures a stepper motor and its associated hardware.
func (e *Executor) configureStepper(name string, cfg *config.StepperConfig) error {
	// Allocate OIDs
	stepperOid := e.nextOid
	e.nextOid++
	enableOid := e.nextOid
	e.nextOid++
	endstopOid := e.nextOid
	e.nextOid++
	trsyncOid := e.nextOid
	e.nextOid++

	e.stepperOids[name] = stepperOid
	e.enableOids[name] = enableOid
	e.endstopOids[name] = endstopOid
	e.trsyncOids[name] = trsyncOid

	// Configure stepper
	if err := e.mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              stepperOid,
		"step_pin":         cfg.StepPin,
		"dir_pin":          cfg.DirPin,
		"invert_step":      0,
		"step_pulse_ticks": e.stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper: %w", err)
	}
	log.Printf("  config_stepper oid=%d (%s) step=%s dir=%s", stepperOid, name, cfg.StepPin, cfg.DirPin)

	// Configure enable pin
	enableDisabledValue := 1
	if !cfg.EnableInvert {
		enableDisabledValue = 0
	}
	if err := e.mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           enableOid,
		"pin":           cfg.EnablePin,
		"value":         enableDisabledValue,
		"default_value": enableDisabledValue,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out: %w", err)
	}
	log.Printf("  config_digital_out oid=%d (%s enable) pin=%s", enableOid, name, cfg.EnablePin)

	// Configure endstop
	pullupValue := 0
	if cfg.EndstopPullup {
		pullupValue = 1
	}
	if err := e.mcu.SendCommand("config_endstop", map[string]interface{}{
		"oid":     endstopOid,
		"pin":     cfg.EndstopPin,
		"pull_up": pullupValue,
	}); err != nil {
		return fmt.Errorf("config_endstop: %w", err)
	}
	log.Printf("  config_endstop oid=%d (%s) pin=%s pullup=%v", endstopOid, name, cfg.EndstopPin, cfg.EndstopPullup)

	// Configure trsync
	if err := e.mcu.SendCommand("config_trsync", map[string]interface{}{
		"oid": trsyncOid,
	}); err != nil {
		return fmt.Errorf("config_trsync: %w", err)
	}
	log.Printf("  config_trsync oid=%d (%s)", trsyncOid, name)

	return nil
}

// configureExtruder configures the extruder stepper, heater, and temperature sensor.
func (e *Executor) configureExtruder() error {
	cfg := e.config.Extruder

	// Stepper
	stepperOid := e.nextOid
	e.nextOid++
	enableOid := e.nextOid
	e.nextOid++

	e.stepperOids["extruder"] = stepperOid
	e.enableOids["extruder"] = enableOid

	if err := e.mcu.SendCommand("config_stepper", map[string]interface{}{
		"oid":              stepperOid,
		"step_pin":         cfg.StepPin,
		"dir_pin":          cfg.DirPin,
		"invert_step":      0,
		"step_pulse_ticks": e.stepPulseTicks,
	}); err != nil {
		return fmt.Errorf("config_stepper: %w", err)
	}
	log.Printf("  config_stepper oid=%d (extruder) step=%s dir=%s", stepperOid, cfg.StepPin, cfg.DirPin)

	enableDisabledValue := 1
	if !cfg.EnableInvert {
		enableDisabledValue = 0
	}
	if err := e.mcu.SendCommand("config_digital_out", map[string]interface{}{
		"oid":           enableOid,
		"pin":           cfg.EnablePin,
		"value":         enableDisabledValue,
		"default_value": enableDisabledValue,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_digital_out: %w", err)
	}
	log.Printf("  config_digital_out oid=%d (extruder enable) pin=%s", enableOid, cfg.EnablePin)

	// Temperature sensor (ADC)
	adcOid := e.nextOid
	e.nextOid++
	e.adcOids["extruder"] = adcOid

	if err := e.mcu.SendCommand("config_analog_in", map[string]interface{}{
		"oid": adcOid,
		"pin": cfg.SensorPin,
	}); err != nil {
		return fmt.Errorf("config_analog_in: %w", err)
	}
	log.Printf("  config_analog_in oid=%d (extruder) pin=%s", adcOid, cfg.SensorPin)

	// Heater PWM
	pwmOid := e.nextOid
	e.nextOid++
	e.pwmOids["extruder"] = pwmOid

	cycleTime := 0.1 // 100ms default
	cycleTicks := int(cycleTime * e.mcuFreq)
	if err := e.mcu.SendCommand("config_pwm_out", map[string]interface{}{
		"oid":           pwmOid,
		"pin":           cfg.HeaterPin,
		"cycle_ticks":   cycleTicks,
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_pwm_out: %w", err)
	}
	log.Printf("  config_pwm_out oid=%d (extruder heater) pin=%s", pwmOid, cfg.HeaterPin)

	return nil
}

// configureHeaterBed configures the heated bed.
func (e *Executor) configureHeaterBed() error {
	cfg := e.config.HeaterBed

	// Temperature sensor (ADC)
	adcOid := e.nextOid
	e.nextOid++
	e.adcOids["heater_bed"] = adcOid

	if err := e.mcu.SendCommand("config_analog_in", map[string]interface{}{
		"oid": adcOid,
		"pin": cfg.SensorPin,
	}); err != nil {
		return fmt.Errorf("config_analog_in: %w", err)
	}
	log.Printf("  config_analog_in oid=%d (heater_bed) pin=%s", adcOid, cfg.SensorPin)

	// Heater PWM
	pwmOid := e.nextOid
	e.nextOid++
	e.pwmOids["heater_bed"] = pwmOid

	cycleTime := 0.1 // 100ms default
	cycleTicks := int(cycleTime * e.mcuFreq)
	if err := e.mcu.SendCommand("config_pwm_out", map[string]interface{}{
		"oid":           pwmOid,
		"pin":           cfg.HeaterPin,
		"cycle_ticks":   cycleTicks,
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		return fmt.Errorf("config_pwm_out: %w", err)
	}
	log.Printf("  config_pwm_out oid=%d (heater_bed) pin=%s", pwmOid, cfg.HeaterPin)

	return nil
}

// configureFan configures a fan.
func (e *Executor) configureFan(name string, cfg *config.FanConfig) error {
	if cfg.Pin == "" {
		log.Printf("  Skipping fan %s: no pin configured", name)
		return nil
	}

	pwmOid := e.nextOid
	e.nextOid++
	e.pwmOids[name] = pwmOid

	cycleTime := cfg.CycleTime
	if cycleTime == 0 {
		cycleTime = 0.1 // 100ms default
	}
	cycleTicks := int(cycleTime * e.mcuFreq)

	// Use hardware PWM for fans (soft PWM not available on all MCUs)
	if err := e.mcu.SendCommand("config_pwm_out", map[string]interface{}{
		"oid":           pwmOid,
		"pin":           cfg.Pin,
		"cycle_ticks":   cycleTicks,
		"value":         0,
		"default_value": 0,
		"max_duration":  0,
	}); err != nil {
		// Fall back to digital out if PWM fails
		log.Printf("  Warning: PWM failed for fan %s, trying digital out", name)
		if err := e.mcu.SendCommand("config_digital_out", map[string]interface{}{
			"oid":           pwmOid,
			"pin":           cfg.Pin,
			"value":         0,
			"default_value": 0,
			"max_duration":  0,
		}); err != nil {
			return fmt.Errorf("config_digital_out: %w", err)
		}
		log.Printf("  config_digital_out oid=%d (%s) pin=%s", pwmOid, name, cfg.Pin)
		return nil
	}
	log.Printf("  config_pwm_out oid=%d (%s) pin=%s", pwmOid, name, cfg.Pin)

	return nil
}

// Execute parses and executes a G-code command.
func (e *Executor) Execute(line string) error {
	cmd, err := parseGCodeLine(line)
	if cmd == nil || err != nil {
		return err
	}

	log.Printf("Executing: %s", cmd.Name)

	switch cmd.Name {
	case "G0", "G1":
		return e.executeMove(cmd)
	case "G28":
		return e.executeHome(cmd)
	case "G90":
		e.absCoords = true
		return nil
	case "G91":
		e.absCoords = false
		return nil
	case "M82":
		e.absExtrude = true
		return nil
	case "M83":
		e.absExtrude = false
		return nil
	case "M104":
		return e.executeSetExtruderTemp(cmd)
	case "M109":
		return e.executeWaitExtruderTemp(cmd)
	case "M140":
		return e.executeSetBedTemp(cmd)
	case "M190":
		return e.executeWaitBedTemp(cmd)
	case "M106":
		return e.executeSetFan(cmd)
	case "M107":
		return e.executeSetFanOff(cmd)
	case "M84", "M18":
		return e.executeDisableMotors(cmd)
	case "M114":
		return e.executeReportPosition(cmd)
	default:
		log.Printf("Unknown G-code command: %s", cmd.Name)
		return nil
	}
}

// executeMove handles G0/G1 movement commands.
func (e *Executor) executeMove(cmd *gcodeCommand) error {
	e.mu.Lock()
	defer e.mu.Unlock()

	// Parse target position
	newPos := e.position

	if v, ok := cmd.Args["X"]; ok {
		x, _ := strconv.ParseFloat(v, 64)
		if e.absCoords {
			newPos[0] = x
		} else {
			newPos[0] += x
		}
	}
	if v, ok := cmd.Args["Y"]; ok {
		y, _ := strconv.ParseFloat(v, 64)
		if e.absCoords {
			newPos[1] = y
		} else {
			newPos[1] += y
		}
	}
	if v, ok := cmd.Args["Z"]; ok {
		z, _ := strconv.ParseFloat(v, 64)
		if e.absCoords {
			newPos[2] = z
		} else {
			newPos[2] += z
		}
	}
	if v, ok := cmd.Args["E"]; ok {
		eVal, _ := strconv.ParseFloat(v, 64)
		if e.absExtrude {
			newPos[3] = eVal
		} else {
			newPos[3] += eVal
		}
	}
	if v, ok := cmd.Args["F"]; ok {
		f, _ := strconv.ParseFloat(v, 64)
		e.feedrate = f / 60.0 // Convert from mm/min to mm/s
	}

	// Calculate move
	dx := newPos[0] - e.position[0]
	dy := newPos[1] - e.position[1]
	dz := newPos[2] - e.position[2]
	de := newPos[3] - e.position[3]

	distance := math.Sqrt(dx*dx + dy*dy + dz*dz)
	if distance == 0 && de == 0 {
		return nil
	}

	// Check if axes are homed before moving
	if dx != 0 && !e.homedAxes[0] {
		return fmt.Errorf("X axis not homed")
	}
	if dy != 0 && !e.homedAxes[1] {
		return fmt.Errorf("Y axis not homed")
	}
	if dz != 0 && !e.homedAxes[2] {
		return fmt.Errorf("Z axis not homed")
	}

	// Execute move based on kinematics
	var err error
	switch e.config.Kinematics {
	case "corexy":
		err = e.executeCoreXYMove(dx, dy, dz, de, e.feedrate)
	case "cartesian":
		err = e.executeCartesianMove(dx, dy, dz, de, e.feedrate)
	default:
		err = fmt.Errorf("unsupported kinematics: %s", e.config.Kinematics)
	}

	if err != nil {
		return err
	}

	e.position = newPos
	return nil
}

// executeCoreXYMove performs a CoreXY kinematic move.
func (e *Executor) executeCoreXYMove(dx, dy, dz, de, speed float64) error {
	// CoreXY kinematics: motor_a = x + y, motor_b = x - y
	motorA := dx + dy
	motorB := dx - dy

	stepperX := e.config.Steppers["stepper_x"]
	stepperY := e.config.Steppers["stepper_y"]
	if stepperX == nil || stepperY == nil {
		return fmt.Errorf("stepper_x or stepper_y not configured")
	}

	stepsPerMMX := stepperX.StepsPerMM()
	stepsPerMMY := stepperY.StepsPerMM()

	stepsA := int64(motorA * stepsPerMMX)
	stepsB := int64(motorB * stepsPerMMY)

	log.Printf("CoreXY move: dx=%.2f dy=%.2f -> motor_a=%.2f(%d steps) motor_b=%.2f(%d steps)",
		dx, dy, motorA, stepsA, motorB, stepsB)

	// Enable motors
	if err := e.enableMotor("stepper_x", true); err != nil {
		return err
	}
	if err := e.enableMotor("stepper_y", true); err != nil {
		return err
	}

	// Calculate move parameters
	distance := math.Sqrt(dx*dx + dy*dy)
	if distance == 0 {
		return nil
	}

	moveTime := distance / speed
	if moveTime == 0 {
		return nil
	}

	// Queue steps for both motors
	if stepsA != 0 {
		if err := e.queueSteps("stepper_x", stepsA, moveTime); err != nil {
			return fmt.Errorf("queue steps X: %w", err)
		}
	}
	if stepsB != 0 {
		if err := e.queueSteps("stepper_y", stepsB, moveTime); err != nil {
			return fmt.Errorf("queue steps Y: %w", err)
		}
	}

	// Handle Z movement
	if dz != 0 {
		if err := e.executeSingleAxisMove("stepper_z", dz, speed); err != nil {
			return fmt.Errorf("Z move: %w", err)
		}
	}

	return nil
}

// executeCartesianMove performs a Cartesian kinematic move.
func (e *Executor) executeCartesianMove(dx, dy, dz, de, speed float64) error {
	distance := math.Sqrt(dx*dx + dy*dy + dz*dz)
	if distance == 0 {
		return nil
	}

	moveTime := distance / speed

	if dx != 0 {
		if err := e.executeSingleAxisMove("stepper_x", dx, speed); err != nil {
			return fmt.Errorf("X move: %w", err)
		}
	}
	if dy != 0 {
		if err := e.executeSingleAxisMove("stepper_y", dy, speed); err != nil {
			return fmt.Errorf("Y move: %w", err)
		}
	}
	if dz != 0 {
		if err := e.executeSingleAxisMove("stepper_z", dz, speed); err != nil {
			return fmt.Errorf("Z move: %w", err)
		}
	}

	_ = moveTime // TODO: synchronize moves
	return nil
}

// executeSingleAxisMove moves a single stepper axis.
func (e *Executor) executeSingleAxisMove(name string, distance, speed float64) error {
	stepper := e.config.Steppers[name]
	if stepper == nil {
		return fmt.Errorf("stepper %s not configured", name)
	}

	if err := e.enableMotor(name, true); err != nil {
		return err
	}

	steps := int64(distance * stepper.StepsPerMM())
	moveTime := math.Abs(distance) / speed

	return e.queueSteps(name, steps, moveTime)
}

// queueSteps queues step commands for a stepper.
func (e *Executor) queueSteps(name string, steps int64, moveTime float64) error {
	if steps == 0 || moveTime == 0 {
		return nil
	}

	stepperOid, ok := e.stepperOids[name]
	if !ok {
		return fmt.Errorf("stepper %s not configured", name)
	}

	// Calculate step interval
	absSteps := steps
	if absSteps < 0 {
		absSteps = -absSteps
	}

	intervalTicks := int(moveTime * e.mcuFreq / float64(absSteps))
	if intervalTicks < e.stepPulseTicks*2 {
		intervalTicks = e.stepPulseTicks * 2
	}

	// Set direction
	dirValue := 0
	if steps < 0 {
		dirValue = 1
	}
	// Apply direction inversion from config
	stepper := e.config.Steppers[name]
	if stepper != nil && stepper.DirInvert {
		dirValue = 1 - dirValue
	}

	// Get current MCU clock using synchronized clock (no command needed)
	clock := e.mcu.GetLastMCUClock()
	if clock == 0 {
		return fmt.Errorf("MCU clock not available")
	}

	// Schedule steps starting slightly in the future
	startClock := clock + uint64(e.mcuFreq*0.05) // 50ms from now for safety

	// Set stepper direction
	if err := e.mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperOid,
		"dir": dirValue,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir: %w", err)
	}

	// Queue steps
	if err := e.mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperOid,
		"interval": intervalTicks,
		"count":    int(absSteps),
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step: %w", err)
	}

	// Reset step clock to start the move
	if err := e.mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperOid,
		"clock": int(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock: %w", err)
	}

	// Wait for move to complete
	time.Sleep(time.Duration(moveTime*1000+50) * time.Millisecond)

	return nil
}

// enableMotor enables or disables a stepper motor.
func (e *Executor) enableMotor(name string, enable bool) error {
	enableOid, ok := e.enableOids[name]
	if !ok {
		return fmt.Errorf("enable pin for %s not configured", name)
	}

	stepper := e.config.Steppers[name]
	if stepper == nil {
		return fmt.Errorf("stepper %s not found", name)
	}

	value := 0
	if enable {
		if stepper.EnableInvert {
			value = 0
		} else {
			value = 1
		}
	} else {
		if stepper.EnableInvert {
			value = 1
		} else {
			value = 0
		}
	}

	return e.mcu.SendCommand("update_digital_out", map[string]interface{}{
		"oid":   enableOid,
		"value": value,
	})
}

// executeHome handles G28 homing command.
func (e *Executor) executeHome(cmd *gcodeCommand) error {
	e.mu.Lock()
	defer e.mu.Unlock()

	// Determine which axes to home
	homeX := false
	homeY := false
	homeZ := false

	if len(cmd.Args) == 0 {
		// G28 without args homes all axes
		homeX = true
		homeY = true
		homeZ = true
	} else {
		_, homeX = cmd.Args["X"]
		_, homeY = cmd.Args["Y"]
		_, homeZ = cmd.Args["Z"]
	}

	log.Printf("Homing: X=%v Y=%v Z=%v", homeX, homeY, homeZ)

	// Home Z first if requested (for safety)
	if homeZ {
		if err := e.homeAxis("stepper_z", 2); err != nil {
			return fmt.Errorf("home Z: %w", err)
		}
		e.homedAxes[2] = true
		e.position[2] = e.config.Steppers["stepper_z"].PositionEndstop
	}

	// Home X and Y based on kinematics
	if homeX || homeY {
		switch e.config.Kinematics {
		case "corexy":
			if homeX {
				if err := e.homeCoreXYAxis("X"); err != nil {
					return fmt.Errorf("home X: %w", err)
				}
				e.homedAxes[0] = true
				e.position[0] = e.config.Steppers["stepper_x"].PositionEndstop
			}
			if homeY {
				if err := e.homeCoreXYAxis("Y"); err != nil {
					return fmt.Errorf("home Y: %w", err)
				}
				e.homedAxes[1] = true
				e.position[1] = e.config.Steppers["stepper_y"].PositionEndstop
			}
		case "cartesian":
			if homeX {
				if err := e.homeAxis("stepper_x", 0); err != nil {
					return fmt.Errorf("home X: %w", err)
				}
				e.homedAxes[0] = true
				e.position[0] = e.config.Steppers["stepper_x"].PositionEndstop
			}
			if homeY {
				if err := e.homeAxis("stepper_y", 1); err != nil {
					return fmt.Errorf("home Y: %w", err)
				}
				e.homedAxes[1] = true
				e.position[1] = e.config.Steppers["stepper_y"].PositionEndstop
			}
		}
	}

	log.Printf("Homing complete. Position: X=%.2f Y=%.2f Z=%.2f", e.position[0], e.position[1], e.position[2])

	return nil
}

// homeAxis homes a single axis using endstop trigger with trsync.
func (e *Executor) homeAxis(name string, axisIndex int) error {
	stepper := e.config.Steppers[name]
	if stepper == nil {
		return fmt.Errorf("stepper %s not configured", name)
	}

	stepperOid := e.stepperOids[name]
	endstopOid := e.endstopOids[name]
	trsyncOid := e.trsyncOids[name]

	// Try to check endstop state (optional - continue if timeout)
	pinValue := 0
	resp, err := e.mcu.SendCommandWithResponse("endstop_query_state", map[string]interface{}{
		"oid": endstopOid,
	}, "endstop_state", 2*time.Second)
	if err != nil {
		log.Printf("  Warning: endstop_query_state failed for %s: %v (proceeding with homing)", name, err)
		// Proceed with homing anyway
	} else {
		pinValue = resp["pin_value"].(int)
		if stepper.EndstopInvert {
			pinValue = 1 - pinValue
		}
		if pinValue == 1 {
			log.Printf("  %s endstop already triggered, skipping homing", name)
			return nil
		}
	}

	// Enable motor
	if err := e.enableMotor(name, true); err != nil {
		return err
	}

	// Calculate homing parameters
	homingDir := stepper.HomingDirection()
	homingSpeed := stepper.HomingSpeed
	stepsPerMM := stepper.StepsPerMM()
	maxTravel := stepper.PositionMax - stepper.PositionMin + 10 // Extra margin

	log.Printf("  Homing %s: dir=%d speed=%.1f mm/s max_travel=%.1f mm", name, homingDir, homingSpeed, maxTravel)

	// Get current MCU clock using synchronized clock (no command needed)
	clock := e.mcu.GetLastMCUClock()
	if clock == 0 {
		return fmt.Errorf("MCU clock not synchronized")
	}

	// Calculate timing
	homingTime := maxTravel / homingSpeed
	stepCount := int(maxTravel * stepsPerMM)
	intervalTicks := int(homingTime * e.mcuFreq / float64(stepCount))

	// Start trsync
	expireTimeout := 10.0 // seconds - longer timeout for safety
	expireTicks := int(expireTimeout * e.mcuFreq)
	startClock := clock + uint64(e.mcuFreq*0.1) // 100ms from now for safety
	expireClock := startClock + uint64(expireTicks)

	if err := e.mcu.SendCommand("trsync_start", map[string]interface{}{
		"oid":           trsyncOid,
		"report_clock":  int(startClock),
		"report_ticks":  int(e.mcuFreq * 0.1), // Report every 100ms
		"expire_reason": 0,
	}); err != nil {
		return fmt.Errorf("trsync_start: %w", err)
	}

	// Set expire time
	if err := e.mcu.SendCommand("trsync_set_timeout", map[string]interface{}{
		"oid":   trsyncOid,
		"clock": int(expireClock),
	}); err != nil {
		return fmt.Errorf("trsync_set_timeout: %w", err)
	}

	// Add endstop trigger
	triggerReason := 1 // Trigger reason code
	if err := e.mcu.SendCommand("endstop_home", map[string]interface{}{
		"oid":            endstopOid,
		"clock":          int(startClock),
		"sample_ticks":   int(e.mcuFreq * 0.000015), // 15us sample time
		"sample_count":   4,
		"rest_ticks":     int(e.mcuFreq * 0.0001), // 100us between samples
		"pin_value":      1 - pinValue,            // Trigger on opposite state
		"trsync_oid":     trsyncOid,
		"trigger_reason": triggerReason,
	}); err != nil {
		return fmt.Errorf("endstop_home: %w", err)
	}

	// Bind stepper to trsync
	if err := e.mcu.SendCommand("stepper_stop_on_trigger", map[string]interface{}{
		"oid":        stepperOid,
		"trsync_oid": trsyncOid,
	}); err != nil {
		return fmt.Errorf("stepper_stop_on_trigger: %w", err)
	}

	// Set direction
	dirValue := 0
	if homingDir < 0 {
		dirValue = 1
	}
	if stepper.DirInvert {
		dirValue = 1 - dirValue
	}
	if err := e.mcu.SendCommand("set_next_step_dir", map[string]interface{}{
		"oid": stepperOid,
		"dir": dirValue,
	}); err != nil {
		return fmt.Errorf("set_next_step_dir: %w", err)
	}

	// Queue steps
	if err := e.mcu.SendCommand("queue_step", map[string]interface{}{
		"oid":      stepperOid,
		"interval": intervalTicks,
		"count":    stepCount,
		"add":      0,
	}); err != nil {
		return fmt.Errorf("queue_step: %w", err)
	}

	// Start move
	if err := e.mcu.SendCommand("reset_step_clock", map[string]interface{}{
		"oid":   stepperOid,
		"clock": int(startClock),
	}); err != nil {
		return fmt.Errorf("reset_step_clock: %w", err)
	}

	// Wait for homing to complete
	log.Printf("  Waiting for endstop trigger...")
	time.Sleep(time.Duration(homingTime*1000+500) * time.Millisecond)

	// Query trsync state
	resp, err = e.mcu.SendCommandWithResponse("trsync_trigger", map[string]interface{}{
		"oid":    trsyncOid,
		"reason": 2, // Query
	}, "trsync_state", 2*time.Second)
	if err != nil {
		log.Printf("  Warning: trsync_trigger query failed: %v", err)
	}

	log.Printf("  %s homing complete", name)

	return nil
}

// homeCoreXYAxis homes an axis using CoreXY kinematics.
// In CoreXY, X homing requires both motors moving in same direction,
// Y homing requires motors moving in opposite directions.
func (e *Executor) homeCoreXYAxis(axis string) error {
	stepperX := e.config.Steppers["stepper_x"]
	stepperY := e.config.Steppers["stepper_y"]
	if stepperX == nil || stepperY == nil {
		return fmt.Errorf("stepper_x or stepper_y not configured")
	}

	var endstopName string

	if axis == "X" {
		endstopName = "stepper_x"
	} else {
		endstopName = "stepper_y"
	}

	// Enable both motors (CoreXY requires both for any XY movement)
	if err := e.enableMotor("stepper_x", true); err != nil {
		return err
	}
	if err := e.enableMotor("stepper_y", true); err != nil {
		return err
	}

	// Use the single-axis homing but with both motors
	return e.homeAxis(endstopName, map[string]int{"X": 0, "Y": 1}[axis])
}

// executeSetExtruderTemp handles M104 (set extruder temperature).
func (e *Executor) executeSetExtruderTemp(cmd *gcodeCommand) error {
	if v, ok := cmd.Args["S"]; ok {
		temp, _ := strconv.ParseFloat(v, 64)
		e.extruderTarget = temp
		log.Printf("Setting extruder temperature to %.1f°C", temp)
		// TODO: Start PID control
	}
	return nil
}

// executeWaitExtruderTemp handles M109 (wait for extruder temperature).
func (e *Executor) executeWaitExtruderTemp(cmd *gcodeCommand) error {
	if err := e.executeSetExtruderTemp(cmd); err != nil {
		return err
	}
	// TODO: Wait for temperature to stabilize
	log.Printf("Waiting for extruder temperature (not implemented)")
	return nil
}

// executeSetBedTemp handles M140 (set bed temperature).
func (e *Executor) executeSetBedTemp(cmd *gcodeCommand) error {
	if v, ok := cmd.Args["S"]; ok {
		temp, _ := strconv.ParseFloat(v, 64)
		e.bedTarget = temp
		log.Printf("Setting bed temperature to %.1f°C", temp)
		// TODO: Start PID control
	}
	return nil
}

// executeWaitBedTemp handles M190 (wait for bed temperature).
func (e *Executor) executeWaitBedTemp(cmd *gcodeCommand) error {
	if err := e.executeSetBedTemp(cmd); err != nil {
		return err
	}
	// TODO: Wait for temperature to stabilize
	log.Printf("Waiting for bed temperature (not implemented)")
	return nil
}

// executeSetFan handles M106 (set fan speed).
func (e *Executor) executeSetFan(cmd *gcodeCommand) error {
	speed := 255.0
	if v, ok := cmd.Args["S"]; ok {
		speed, _ = strconv.ParseFloat(v, 64)
	}
	pwmValue := speed / 255.0

	pwmOid, ok := e.pwmOids["fan"]
	if !ok {
		log.Printf("Warning: fan not configured")
		return nil
	}

	maxPWM := 65535 // 16-bit PWM
	value := int(pwmValue * float64(maxPWM))

	log.Printf("Setting fan speed to %.1f%%", pwmValue*100)

	return e.mcu.SendCommand("schedule_pwm_out", map[string]interface{}{
		"oid":   pwmOid,
		"clock": 0,
		"value": value,
	})
}

// executeSetFanOff handles M107 (turn off fan).
func (e *Executor) executeSetFanOff(cmd *gcodeCommand) error {
	pwmOid, ok := e.pwmOids["fan"]
	if !ok {
		return nil
	}

	log.Printf("Turning off fan")

	return e.mcu.SendCommand("schedule_pwm_out", map[string]interface{}{
		"oid":   pwmOid,
		"clock": 0,
		"value": 0,
	})
}

// executeDisableMotors handles M84/M18 (disable motors).
func (e *Executor) executeDisableMotors(cmd *gcodeCommand) error {
	log.Printf("Disabling motors")

	for name := range e.enableOids {
		if err := e.enableMotor(name, false); err != nil {
			log.Printf("Warning: failed to disable %s: %v", name, err)
		}
	}

	// Clear homed state
	e.homedAxes = [3]bool{}

	return nil
}

// executeReportPosition handles M114 (report position).
func (e *Executor) executeReportPosition(cmd *gcodeCommand) error {
	log.Printf("Position: X:%.3f Y:%.3f Z:%.3f E:%.3f",
		e.position[0], e.position[1], e.position[2], e.position[3])
	return nil
}

// GetPosition returns the current position.
func (e *Executor) GetPosition() [4]float64 {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.position
}

// GetHomedAxes returns which axes are homed.
func (e *Executor) GetHomedAxes() [3]bool {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.homedAxes
}

// GetTemperature returns temperature readings.
func (e *Executor) GetTemperature(name string) (temp, target float64) {
	e.mu.RLock()
	defer e.mu.RUnlock()

	switch name {
	case "extruder":
		return e.extruderTemp, e.extruderTarget
	case "heater_bed":
		return e.bedTemp, e.bedTarget
	}
	return 0, 0
}

// GetHomedAxesString returns a string of homed axes (e.g., "xy" or "xyz").
func (e *Executor) GetHomedAxesString() string {
	e.mu.RLock()
	defer e.mu.RUnlock()

	result := ""
	if e.homedAxes[0] {
		result += "x"
	}
	if e.homedAxes[1] {
		result += "y"
	}
	if e.homedAxes[2] {
		result += "z"
	}
	return result
}

// GetFeedrate returns the current feedrate in mm/s.
func (e *Executor) GetFeedrate() float64 {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.feedrate
}

// IsAbsoluteCoords returns true if using absolute coordinates.
func (e *Executor) IsAbsoluteCoords() bool {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.absCoords
}

// IsAbsoluteExtrude returns true if using absolute extrusion.
func (e *Executor) IsAbsoluteExtrude() bool {
	e.mu.RLock()
	defer e.mu.RUnlock()
	return e.absExtrude
}

// gcodeCommand represents a parsed G-code command.
type gcodeCommand struct {
	Name string
	Args map[string]string
	Raw  string
}

var reParenComment = regexp.MustCompile(`\([^)]*\)`)

// parseGCodeLine parses a G-code line into a command.
func parseGCodeLine(line string) (*gcodeCommand, error) {
	ln := strings.TrimSpace(line)
	if ln == "" {
		return nil, nil
	}
	if idx := strings.IndexByte(ln, ';'); idx >= 0 {
		ln = strings.TrimSpace(ln[:idx])
	}
	if ln == "" {
		return nil, nil
	}
	ln = strings.TrimSpace(reParenComment.ReplaceAllString(ln, " "))
	if ln == "" {
		return nil, nil
	}

	fields := strings.Fields(ln)
	if len(fields) == 0 {
		return nil, nil
	}

	name := strings.ToUpper(fields[0])
	args := map[string]string{}
	for _, f := range fields[1:] {
		if f == "" {
			continue
		}
		if strings.Contains(f, "=") {
			kv := strings.SplitN(f, "=", 2)
			k := strings.ToUpper(strings.TrimSpace(kv[0]))
			v := ""
			if len(kv) > 1 {
				v = strings.TrimSpace(kv[1])
			}
			if k != "" {
				args[k] = v
			}
			continue
		}
		// Handle single-letter flags like "X", "Y", "Z" (no value)
		if len(f) == 1 {
			k := strings.ToUpper(f)
			args[k] = ""
			continue
		}
		k := strings.ToUpper(f[:1])
		v := strings.TrimSpace(f[1:])
		if k != "" {
			args[k] = v
		}
	}
	return &gcodeCommand{Name: name, Args: args, Raw: line}, nil
}
