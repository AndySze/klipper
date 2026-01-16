// Support for accelerometer sensors (ADXL345, MPU9250) in connect-phase compilation.
//
// Copyright (C) 2026  Klipper Go Migration
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth1

import (
	"fmt"
	"hash/crc32"
	"strings"

	"klipper-go-migration/pkg/protocol"
)

// adxl345Config holds parsed [adxl345] section configuration.
type adxl345Config struct {
	csPin        pin
	spiBus       string
	spiMode      int
	spiSpeed     int
	csActiveHigh bool
}

// mpu9250Config holds parsed [mpu9250] section configuration.
type mpu9250Config struct {
	name    string // section name suffix (e.g., "my_mpu" from "[mpu9250 my_mpu]")
	i2cBus  string
	i2cAddr int
	speed   int
}

// readADXL345 parses an [adxl345] section from config.
func readADXL345(cfg *config, sectionName string) (*adxl345Config, error) {
	sec, ok := cfg.sections[sectionName]
	if !ok {
		return nil, nil
	}

	csPin, err := parsePin(sec, "cs_pin", true, false)
	if err != nil {
		return nil, fmt.Errorf("adxl345 cs_pin: %w", err)
	}

	// Default SPI settings for ADXL345
	spiBus := sec["spi_bus"]
	if spiBus == "" {
		spiBus = "spi" // default bus name
	}

	spiSpeed := 5000000 // 5MHz default
	if s := sec["spi_speed"]; s != "" {
		fmt.Sscanf(s, "%d", &spiSpeed)
	}

	return &adxl345Config{
		csPin:        csPin,
		spiBus:       spiBus,
		spiMode:      3, // ADXL345 uses SPI mode 3
		spiSpeed:     spiSpeed,
		csActiveHigh: false,
	}, nil
}

// readMPU9250 parses an [mpu9250] or [mpu9250 name] section from config.
func readMPU9250(cfg *config, sectionName string) (*mpu9250Config, error) {
	sec, ok := cfg.sections[sectionName]
	if !ok {
		return nil, nil
	}

	// Extract name suffix if present
	name := ""
	if strings.HasPrefix(sectionName, "mpu9250 ") {
		name = strings.TrimPrefix(sectionName, "mpu9250 ")
	}

	// Default I2C settings for MPU9250
	i2cBus := sec["i2c_bus"]
	if i2cBus == "" {
		i2cBus = "twi" // default bus name (atmega uses twi)
	}

	i2cAddr := 0x68 // MPU9250 default address (104 decimal)
	if s := sec["i2c_address"]; s != "" {
		fmt.Sscanf(s, "%d", &i2cAddr)
	}

	speed := 400000 // 400kHz default
	if s := sec["i2c_speed"]; s != "" {
		fmt.Sscanf(s, "%d", &speed)
	}

	return &mpu9250Config{
		name:    name,
		i2cBus:  i2cBus,
		i2cAddr: i2cAddr,
		speed:   speed,
	}, nil
}

// CompileInputShaperConnectPhase compiles the connect-phase MCU command stream
// for configs with [input_shaper], [adxl345], and [mpu9250] sections.
//
// This handles the OID allocation order: accelerometers first, then heaters/steppers.
func CompileInputShaperConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
	cfg, err := loadConfig(cfgPath)
	if err != nil {
		return nil, err
	}
	clockFreq, err := dictConfigFloat(dict, "CLOCK_FREQ")
	if err != nil {
		return nil, err
	}
	adcMax, err := dictConfigFloat(dict, "ADC_MAX")
	if err != nil {
		return nil, err
	}
	mcuFreq := clockFreq

	// Parse accelerometer sections
	adxl, err := readADXL345(cfg, "adxl345")
	if err != nil {
		return nil, err
	}

	// Find MPU9250 section (may be [mpu9250] or [mpu9250 name])
	var mpu *mpu9250Config
	for secName := range cfg.sections {
		if secName == "mpu9250" || strings.HasPrefix(secName, "mpu9250 ") {
			mpu, err = readMPU9250(cfg, secName)
			if err != nil {
				return nil, err
			}
			break
		}
	}

	// Extract required pins for standard components.
	stepperX, err := readStepper(cfg, "stepper_x")
	if err != nil {
		return nil, err
	}
	stepperY, err := readStepper(cfg, "stepper_y")
	if err != nil {
		return nil, err
	}
	stepperZ, err := readStepper(cfg, "stepper_z")
	if err != nil {
		return nil, err
	}
	extruder, err := readExtruder(cfg, "extruder")
	if err != nil {
		return nil, err
	}
	bed, err := readHeater(cfg, "heater_bed")
	if err != nil {
		return nil, err
	}

	// Allocate OIDs: accelerometers first, then standard components
	// OID layout for input_shaper.cfg:
	// 0: SPI bus (for ADXL345)
	// 1: ADXL345
	// 2: I2C bus
	// 3: MPU9250
	// 4-13: endstops, trsyncs, steppers
	// 14-21: analog_in, digital_out for heaters

	nextOID := 0

	// Accelerometer OIDs
	spiOID := -1
	adxlOID := -1
	i2cOID := -1
	mpuOID := -1

	if adxl != nil {
		spiOID = nextOID
		nextOID++
		adxlOID = nextOID
		nextOID++
	}

	if mpu != nil {
		i2cOID = nextOID
		nextOID++
		mpuOID = nextOID
		nextOID++
	}

	// Standard component OIDs (shifted to account for accelerometers)
	endstopX := nextOID
	nextOID++
	trsyncX := nextOID
	nextOID++
	stepperXOID := nextOID
	nextOID++

	endstopY := nextOID
	nextOID++
	trsyncY := nextOID
	nextOID++
	stepperYOID := nextOID
	nextOID++

	endstopZ := nextOID
	nextOID++
	trsyncZ := nextOID
	nextOID++
	stepperZOID := nextOID
	nextOID++

	stepperEOID := nextOID
	nextOID++

	adcBed := nextOID
	nextOID++
	pwmBed := nextOID
	nextOID++

	enableX := nextOID
	nextOID++
	enableY := nextOID
	nextOID++
	enableZ := nextOID
	nextOID++

	adcE := nextOID
	nextOID++
	pwmE := nextOID
	nextOID++
	enableE := nextOID
	nextOID++

	totalOIDs := nextOID

	// Derived constants (match klippy defaults).
	const (
		pwmCycleTime     = 0.100
		heaterMaxHeatSec = 3.0
		stepPulseSec     = 0.000002
		adcSampleTime    = 0.001
		adcSampleCount   = 8
		adcReportTime    = 0.300
		adcRangeChecks   = 4
		pwmInitDelaySec  = 0.200
	)
	cycleTicks := secondsToClock(mcuFreq, pwmCycleTime)
	mdurTicks := secondsToClock(mcuFreq, heaterMaxHeatSec)
	stepPulseTicks := secondsToClock(mcuFreq, stepPulseSec)
	sampleTicks := secondsToClock(mcuFreq, adcSampleTime)
	reportTicks := secondsToClock(mcuFreq, adcReportTime)
	pwmInitClock := secondsToClock(mcuFreq, pwmInitDelaySec)

	// ADC min/max for thermistors.
	bedMin, bedMax, err := thermistorMinMaxTicks(bed, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}
	extMin, extMax, err := thermistorMinMaxTicks(extruder.Heater, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}

	// Build config commands in the same order as the Python reference.
	var configCmds []string

	// Accelerometer config commands come first
	if adxl != nil {
		configCmds = append(configCmds,
			fmt.Sprintf("config_spi oid=%d pin=%s cs_active_high=%d",
				spiOID, adxl.csPin.pin, boolToInt(adxl.csActiveHigh)),
			fmt.Sprintf("config_adxl345 oid=%d spi_oid=%d", adxlOID, spiOID),
		)
	}

	if mpu != nil {
		configCmds = append(configCmds,
			fmt.Sprintf("config_i2c oid=%d", i2cOID),
		)
	}

	// Standard heater config
	configCmds = append(configCmds,
		fmt.Sprintf("config_analog_in oid=%d pin=%s", adcBed, bed.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			pwmBed, bed.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", pwmBed, cycleTicks),
	)

	// SPI/I2C bus setup commands (come after initial config, before steppers)
	if adxl != nil {
		configCmds = append(configCmds,
			fmt.Sprintf("spi_set_bus oid=%d spi_bus=%s mode=%d rate=%d",
				spiOID, adxl.spiBus, adxl.spiMode, adxl.spiSpeed),
		)
	}

	if mpu != nil {
		configCmds = append(configCmds,
			fmt.Sprintf("i2c_set_bus oid=%d i2c_bus=%s rate=%d address=%d",
				i2cOID, mpu.i2cBus, mpu.speed, mpu.i2cAddr),
			fmt.Sprintf("config_mpu9250 oid=%d i2c_oid=%d", mpuOID, i2cOID),
		)
	}

	// Stepper config commands
	configCmds = append(configCmds,
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", endstopX, stepperX.Endstop.pin, stepperX.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", trsyncX),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			stepperXOID, stepperX.Step.pin, stepperX.Dir.pin, boolToInt(stepperX.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			enableX, stepperX.Enable.pin, boolToInt(stepperX.Enable.invert), boolToInt(stepperX.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", endstopY, stepperY.Endstop.pin, stepperY.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", trsyncY),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			stepperYOID, stepperY.Step.pin, stepperY.Dir.pin, boolToInt(stepperY.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			enableY, stepperY.Enable.pin, boolToInt(stepperY.Enable.invert), boolToInt(stepperY.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", endstopZ, stepperZ.Endstop.pin, stepperZ.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", trsyncZ),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			stepperZOID, stepperZ.Step.pin, stepperZ.Dir.pin, boolToInt(stepperZ.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			enableZ, stepperZ.Enable.pin, boolToInt(stepperZ.Enable.invert), boolToInt(stepperZ.Enable.invert), 0),

		fmt.Sprintf("config_analog_in oid=%d pin=%s", adcE, extruder.Heater.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			pwmE, extruder.Heater.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", pwmE, cycleTicks),

		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			stepperEOID, extruder.Step.pin, extruder.Dir.pin, boolToInt(extruder.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			enableE, extruder.Enable.pin, boolToInt(extruder.Enable.invert), boolToInt(extruder.Enable.invert), 0),
	)

	// Prepend allocate_oids and compute finalize_config CRC over config_cmds.
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", totalOIDs)}, configCmds...)
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	// Build init commands (same order as the Python reference).
	// Use adjusted OIDs for ADC query clock calculation
	initCmds := []string{
		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			adcBed, querySlotClock(mcuFreq, adcBed), sampleTicks, adcSampleCount, reportTicks, bedMin, bedMax, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", pwmBed, pwmInitClock, 0),

		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			adcE, querySlotClock(mcuFreq, adcE), sampleTicks, adcSampleCount, reportTicks, extMin, extMax, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", pwmE, pwmInitClock, 0),
	}

	out := make([]string, 0, 1+len(configCmds)+1+len(initCmds))
	out = append(out, withAllocate...)
	out = append(out, fmt.Sprintf("finalize_config crc=%d", crc))
	out = append(out, initCmds...)
	return out, nil
}
