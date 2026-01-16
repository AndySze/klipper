// Support for probe-based Z endstop in connect-phase compilation.
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

// probeConfig holds parsed [probe] section configuration.
type probeConfig struct {
	pin      pin
	zOffset  float64
}

// readProbe parses a [probe] section from config.
func readProbe(cfg *config, sectionName string) (*probeConfig, error) {
	sec, ok := cfg.sections[sectionName]
	if !ok {
		return nil, nil
	}

	probePin, err := parsePin(sec, "pin", false, true) // optional pullup
	if err != nil {
		return nil, fmt.Errorf("probe pin: %w", err)
	}

	zOffset := 0.0
	if s := sec["z_offset"]; s != "" {
		fmt.Sscanf(s, "%f", &zOffset)
	}

	return &probeConfig{
		pin:     probePin,
		zOffset: zOffset,
	}, nil
}

// CompileProbeConnectPhase compiles the connect-phase MCU command stream for
// configs with [probe] section and probe:z_virtual_endstop on stepper_z.
//
// This handles standard cartesian printers with a probe instead of Z endstop.
func CompileProbeConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
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

	// Parse probe section
	probe, err := readProbe(cfg, "probe")
	if err != nil {
		return nil, err
	}
	if probe == nil {
		return nil, fmt.Errorf("missing [probe] section")
	}

	// Extract stepper configs
	stepperX, err := readStepper(cfg, "stepper_x")
	if err != nil {
		return nil, err
	}
	stepperY, err := readStepper(cfg, "stepper_y")
	if err != nil {
		return nil, err
	}

	// stepper_z uses probe:z_virtual_endstop, so we read it manually without endstop
	stepperZSec, ok := cfg.sections["stepper_z"]
	if !ok {
		return nil, fmt.Errorf("missing [stepper_z] section")
	}
	stepperZStep, err := parsePin(stepperZSec, "step_pin", true, false)
	if err != nil {
		return nil, err
	}
	stepperZDir, err := parsePin(stepperZSec, "dir_pin", true, false)
	if err != nil {
		return nil, err
	}
	stepperZEn, err := parsePin(stepperZSec, "enable_pin", true, false)
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

	// OID layout for probe-based Z homing:
	// 0: endstop (probe pin)
	// 1: trsync (probe)
	// 2: endstop X
	// 3: trsync X
	// 4: stepper X
	// 5: endstop Y
	// 6: trsync Y
	// 7: stepper Y
	// 8: stepper Z (no endstop/trsync - uses probe)
	// 9: stepper E
	// 10: adc bed
	// 11: pwm bed
	// 12: enable X
	// 13: enable Y
	// 14: enable Z
	// 15: adc E
	// 16: pwm E
	// 17: enable E
	type oids struct {
		endstopProbe, trsyncProbe                           int
		endstopX, trsyncX, stepperX                         int
		endstopY, trsyncY, stepperY                         int
		stepperZ                                            int
		stepperE                                            int
		adcBed, pwmBed                                      int
		enableX, enableY, enableZ                           int
		adcE, pwmE, enableE                                 int
		count                                               int
	}
	o := oids{
		endstopProbe: 0, trsyncProbe: 1,
		endstopX: 2, trsyncX: 3, stepperX: 4,
		endstopY: 5, trsyncY: 6, stepperY: 7,
		stepperZ: 8,
		stepperE: 9,
		adcBed: 10, pwmBed: 11,
		enableX: 12, enableY: 13, enableZ: 14,
		adcE: 15, pwmE: 16, enableE: 17,
		count: 18,
	}

	// Derived constants
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

	// ADC min/max for thermistors
	bedMin, bedMax, err := thermistorMinMaxTicks(bed, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}
	extMin, extMax, err := thermistorMinMaxTicks(extruder.Heater, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}

	// Build config commands in the same order as Python reference
	configCmds := []string{
		// Heater bed first
		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcBed, bed.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmBed, bed.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmBed, cycleTicks),

		// Probe endstop (used for Z homing)
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopProbe, probe.pin.pin, probe.pin.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncProbe),

		// X axis
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopX, stepperX.Endstop.pin, stepperX.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncX),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperX, stepperX.Step.pin, stepperX.Dir.pin, boolToInt(stepperX.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableX, stepperX.Enable.pin, boolToInt(stepperX.Enable.invert), boolToInt(stepperX.Enable.invert), 0),

		// Y axis
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopY, stepperY.Endstop.pin, stepperY.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncY),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperY, stepperY.Step.pin, stepperY.Dir.pin, boolToInt(stepperY.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableY, stepperY.Enable.pin, boolToInt(stepperY.Enable.invert), boolToInt(stepperY.Enable.invert), 0),

		// Z axis (no endstop - uses probe)
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperZ, stepperZStep.pin, stepperZDir.pin, boolToInt(stepperZStep.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableZ, stepperZEn.pin, boolToInt(stepperZEn.invert), boolToInt(stepperZEn.invert), 0),

		// Extruder
		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcE, extruder.Heater.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmE, extruder.Heater.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmE, cycleTicks),

		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperE, extruder.Step.pin, extruder.Dir.pin, boolToInt(extruder.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableE, extruder.Enable.pin, boolToInt(extruder.Enable.invert), boolToInt(extruder.Enable.invert), 0),
	}

	// Prepend allocate_oids and compute CRC
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", o.count)}, configCmds...)
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	// Build init commands
	initCmds := []string{
		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			o.adcBed, querySlotClock(mcuFreq, o.adcBed), sampleTicks, adcSampleCount, reportTicks, bedMin, bedMax, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", o.pwmBed, pwmInitClock, 0),

		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			o.adcE, querySlotClock(mcuFreq, o.adcE), sampleTicks, adcSampleCount, reportTicks, extMin, extMax, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", o.pwmE, pwmInitClock, 0),
	}

	out := make([]string, 0, 1+len(configCmds)+1+len(initCmds))
	out = append(out, withAllocate...)
	out = append(out, fmt.Sprintf("finalize_config crc=%d", crc))
	out = append(out, initCmds...)
	return out, nil
}
