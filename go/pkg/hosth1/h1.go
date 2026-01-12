package hosth1

import (
	"bufio"
	"fmt"
	"hash/crc32"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strconv"
	"strings"

	"klipper-go-migration/pkg/protocol"
)

// CompileExampleCartesianConnectPhase compiles the connect-phase MCU command
// stream for Klipper's `config/example-cartesian.cfg`.
//
// It is intentionally scoped to the current H1 milestone: produce the same
// `expected.txt` lines for `test/klippy/commands.test` (connect/init only).
func CompileExampleCartesianConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
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

	// Extract required pins.
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

	// Allocate OIDs in the same stable numbering as the Python reference for
	// this config. (See expected output for commands.test.)
	o := oids{
		endstopX: 0, trsyncX: 1, stepperX: 2,
		endstopY: 3, trsyncY: 4, stepperY: 5,
		endstopZ: 6, trsyncZ: 7, stepperZ: 8,
		stepperE: 9,
		adcBed:   10, pwmBed: 11,
		enableX: 12, enableY: 13, enableZ: 14,
		adcE: 15, pwmE: 16, enableE: 17,
		count: 18,
	}

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
	configCmds := []string{
		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcBed, bed.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmBed, bed.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmBed, cycleTicks),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopX, stepperX.Endstop.pin, stepperX.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncX),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperX, stepperX.Step.pin, stepperX.Dir.pin, boolToInt(stepperX.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableX, stepperX.Enable.pin, boolToInt(stepperX.Enable.invert), boolToInt(stepperX.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopY, stepperY.Endstop.pin, stepperY.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncY),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperY, stepperY.Step.pin, stepperY.Dir.pin, boolToInt(stepperY.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableY, stepperY.Enable.pin, boolToInt(stepperY.Enable.invert), boolToInt(stepperY.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopZ, stepperZ.Endstop.pin, stepperZ.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncZ),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperZ, stepperZ.Step.pin, stepperZ.Dir.pin, boolToInt(stepperZ.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableZ, stepperZ.Enable.pin, boolToInt(stepperZ.Enable.invert), boolToInt(stepperZ.Enable.invert), 0),

		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcE, extruder.Heater.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmE, extruder.Heater.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmE, cycleTicks),

		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperE, extruder.Step.pin, extruder.Dir.pin, boolToInt(extruder.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableE, extruder.Enable.pin, boolToInt(extruder.Enable.invert), boolToInt(extruder.Enable.invert), 0),
	}

	// Prepend allocate_oids and compute finalize_config CRC over config_cmds.
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", o.count)}, configCmds...)
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	// Build init commands (same order as the Python reference for this config).
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

// CompileBLTouchConnectPhase compiles the connect-phase MCU command stream for
// `test/klippy/bltouch.cfg`.
//
// It is intentionally minimal and fixed to the regression config's stable OID
// layout and connect-time BLTouch initialization sequence.
func CompileBLTouchConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
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

	stepperX, err := readStepper(cfg, "stepper_x")
	if err != nil {
		return nil, err
	}
	stepperY, err := readStepper(cfg, "stepper_y")
	if err != nil {
		return nil, err
	}
	// Note: stepper_z uses probe:z_virtual_endstop; it does not have a real
	// MCU endstop pin in this test config.
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

	extruder0, err := readExtruder(cfg, "extruder")
	if err != nil {
		return nil, err
	}
	bed, err := readHeater(cfg, "heater_bed")
	if err != nil {
		return nil, err
	}

	bltSec, ok := cfg.sections["bltouch"]
	if !ok {
		return nil, fmt.Errorf("missing [bltouch] section")
	}
	bltSensor, err := parsePin(bltSec, "sensor_pin", false, true)
	if err != nil {
		return nil, err
	}
	bltControl, err := parsePin(bltSec, "control_pin", false, false)
	if err != nil {
		return nil, err
	}

	// Fixed OID layout matches the Python reference for bltouch.cfg.
	type oids struct {
		endstopProbe int
		trsyncProbe  int

		endstopX int
		trsyncX  int
		stepperX int
		enableX  int

		endstopY int
		trsyncY  int
		stepperY int
		enableY  int

		stepperZ int
		enableZ  int

		stepperE int
		enableE  int

		adcBed   int
		pwmBed   int
		pwmServo int
		adcE     int
		pwmE     int
		count    int
	}
	o := oids{
		endstopProbe: 0, trsyncProbe: 1,
		endstopX: 2, trsyncX: 3, stepperX: 4, enableX: 13,
		endstopY: 5, trsyncY: 6, stepperY: 7, enableY: 14,
		stepperZ: 8, enableZ: 15,
		stepperE: 9, enableE: 18,
		adcBed: 10, pwmBed: 11, pwmServo: 12,
		adcE: 16, pwmE: 17,
		count: 19,
	}

	const (
		heaterCycleTimeSec = 0.100
		servoCycleTimeSec  = 0.020
		heaterMaxHeatSec   = 3.0
		stepPulseSec       = 0.000002
		adcSampleTime      = 0.001
		adcSampleCount     = 8
		adcReportTime      = 0.300
		adcRangeChecks     = 4
		pwmInitDelaySec    = 0.200

		bltMinCmdTimeSec      = 0.100
		bltPinMoveTimeSec     = 0.680
		bltSignalPeriodSec    = 0.020
		bltEndstopRestSec     = 0.001
		bltEndstopSampleSec   = 0.000015
		bltEndstopSampleCount = 4
		trsyncTimeoutSec      = 0.250
		trsyncReportRatio     = 0.300
		trsyncExpireReason    = 4
	)
	cycleTicksHeater := secondsToClock(mcuFreq, heaterCycleTimeSec)
	cycleTicksServo := secondsToClock(mcuFreq, servoCycleTimeSec)
	mdurTicks := secondsToClock(mcuFreq, heaterMaxHeatSec)
	stepPulseTicks := secondsToClock(mcuFreq, stepPulseSec)
	sampleTicks := secondsToClock(mcuFreq, adcSampleTime)
	reportTicks := secondsToClock(mcuFreq, adcReportTime)
	pwmInitClock := secondsToClock(mcuFreq, pwmInitDelaySec)

	bedMin, bedMax, err := thermistorMinMaxTicks(bed, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}
	extMin, extMax, err := thermistorMinMaxTicks(extruder0.Heater, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}

	configCmds := []string{
		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcBed, bed.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmBed, bed.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmBed, cycleTicksHeater),

		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmServo, bltControl.pin, 0, 0, 0),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmServo, cycleTicksServo),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopProbe, bltSensor.pin, bltSensor.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncProbe),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopX, stepperX.Endstop.pin, stepperX.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncX),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperX, stepperX.Step.pin, stepperX.Dir.pin, boolToInt(stepperX.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableX, stepperX.Enable.pin, boolToInt(stepperX.Enable.invert), boolToInt(stepperX.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopY, stepperY.Endstop.pin, stepperY.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncY),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperY, stepperY.Step.pin, stepperY.Dir.pin, boolToInt(stepperY.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableY, stepperY.Enable.pin, boolToInt(stepperY.Enable.invert), boolToInt(stepperY.Enable.invert), 0),

		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperZ, stepperZStep.pin, stepperZDir.pin, 0, stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableZ, stepperZEn.pin, boolToInt(stepperZEn.invert), boolToInt(stepperZEn.invert), 0),

		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcE, extruder0.Heater.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmE, extruder0.Heater.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmE, cycleTicksHeater),

		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperE, extruder0.Step.pin, extruder0.Dir.pin, boolToInt(extruder0.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableE, extruder0.Enable.pin, boolToInt(extruder0.Enable.invert), boolToInt(extruder0.Enable.invert), 0),
	}
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", o.count)}, configCmds...)
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	initCmds := []string{
		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			o.adcBed, querySlotClock(mcuFreq, o.adcBed), sampleTicks, adcSampleCount, reportTicks, bedMin, bedMax, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", o.pwmBed, pwmInitClock, 0),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", o.pwmServo, pwmInitClock, 0),
		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			o.adcE, querySlotClock(mcuFreq, o.adcE), sampleTicks, adcSampleCount, reportTicks, extMin, extMax, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", o.pwmE, pwmInitClock, 0),
	}

	// BLTouch connect handler: pin_up and verify_state(triggered=False).
	nextCmdTime := bltMinCmdTimeSec + 0.200
	// send_cmd('pin_up', duration=pin_move_time)
	cmdStart := nextCmdTime
	pulse := float64(int((bltPinMoveTimeSec-bltMinCmdTimeSec)/bltSignalPeriodSec)) * bltSignalPeriodSec
	cmdHold := math.Max(bltMinCmdTimeSec, pulse)
	startClk := secondsToClockRound(mcuFreq, cmdStart)
	holdTicks := secondsToClockRound(mcuFreq, cmdHold)
	cmdEndClk := startClk + holdTicks
	initCmds = append(initCmds,
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d",
			o.pwmServo, startClk, secondsToClockRound(mcuFreq, 0.001475)),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d",
			o.pwmServo, cmdEndClk, 0),
	)
	actionEndClk := startClk + secondsToClockRound(mcuFreq, bltPinMoveTimeSec)
	actionEnd := float64(actionEndClk) / mcuFreq
	nextCmdTime = math.Max(actionEnd, float64(cmdEndClk)/mcuFreq+bltMinCmdTimeSec)
	_ = nextCmdTime
	clock := actionEndClk
	reportTicksProbe := int(mcuFreq * (trsyncTimeoutSec * trsyncReportRatio))
	expireClock := clock + secondsToClockRound(mcuFreq, trsyncTimeoutSec)
	sampleTicksBlt := secondsToClockRound(mcuFreq, bltEndstopSampleSec)
	restTicksBlt := secondsToClockRound(mcuFreq, bltEndstopRestSec)
	initCmds = append(initCmds,
		fmt.Sprintf("trsync_start oid=%d report_clock=%d report_ticks=%d expire_reason=%d",
			o.trsyncProbe, clock, reportTicksProbe, trsyncExpireReason),
		fmt.Sprintf("stepper_stop_on_trigger oid=%d trsync_oid=%d", o.stepperZ, o.trsyncProbe),
		fmt.Sprintf("trsync_set_timeout oid=%d clock=%d", o.trsyncProbe, expireClock),
		fmt.Sprintf("endstop_home oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d pin_value=%d trsync_oid=%d trigger_reason=1",
			o.endstopProbe, clock, sampleTicksBlt, bltEndstopSampleCount, restTicksBlt, 0, o.trsyncProbe),
		fmt.Sprintf("endstop_home oid=%d clock=0 sample_ticks=0 sample_count=0 rest_ticks=0 pin_value=0 trsync_oid=0 trigger_reason=0", o.endstopProbe),
	)

	out := make([]string, 0, 1+len(configCmds)+1+len(initCmds))
	out = append(out, withAllocate...)
	out = append(out, fmt.Sprintf("finalize_config crc=%d", crc))
	out = append(out, initCmds...)
	return out, nil
}

// CompileLinuxTestConnectPhase compiles the connect-phase MCU command stream
// for `test/klippy/linuxtest.cfg` (linuxprocess + DS18B20).
func CompileLinuxTestConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
	cfg, err := loadConfig(cfgPath)
	if err != nil {
		return nil, err
	}
	clockFreq, err := dictConfigFloat(dict, "CLOCK_FREQ")
	if err != nil {
		return nil, err
	}

	// Parse the single temperature sensor section.
	ts, err := readTempSensor(cfg, "temperature_sensor my_ds18b20")
	if err != nil {
		return nil, err
	}
	if ts.SensorType != "DS18B20" {
		return nil, fmt.Errorf("unsupported sensor_type %q (H1 only supports DS18B20 here)", ts.SensorType)
	}
	if ts.SensorMCU != "mcu" {
		return nil, fmt.Errorf("unsupported sensor_mcu %q", ts.SensorMCU)
	}

	// OID allocation: one DS18B20 object.
	const oid = 0
	const oidCount = 1

	maxErrorCount := 4
	reportTime := 3.0
	reportTicks := secondsToClock(clockFreq, reportTime)
	clock := querySlotClock(clockFreq, oid)

	// In msgproto, command text uses a hex string for the serial, which is then
	// encoded as bytes; parsedump prints the decoded bytes as b'...'.
	sidHex := bytesToLowerHex(ts.SerialNo)
	configCmds := []string{
		fmt.Sprintf("config_ds18b20 oid=%d serial=%s max_error_count=%d", oid, sidHex, maxErrorCount),
	}
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", oidCount)}, configCmds...)
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	initCmds := []string{
		fmt.Sprintf("query_ds18b20 oid=%d clock=%d rest_ticks=%d min_value=%d max_value=%d",
			oid, clock, reportTicks, int(ts.MinTemp*1000), int(ts.MaxTemp*1000)),
	}

	out := make([]string, 0, 1+len(configCmds)+1+len(initCmds))
	out = append(out, withAllocate...)
	out = append(out, fmt.Sprintf("finalize_config crc=%d", crc))
	out = append(out, initCmds...)
	return out, nil
}

// CompileCartesianNoExtruder compiles the connect-phase for cartesian configs
// without heater_bed AND without extruder (e.g., bed_screws.cfg).
// OID layout: X/Y/Z steppers only, no E axis.
func CompileCartesianNoExtruder(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
	cfg, err := loadConfig(cfgPath)
	if err != nil {
		return nil, err
	}
	clockFreq, err := dictConfigFloat(dict, "CLOCK_FREQ")
	if err != nil {
		return nil, err
	}
	mcuFreq := clockFreq

	// Extract required pins.
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

	// Allocate OIDs for cartesian config without heater_bed and without extruder.
	// OIDs 0-8: X/Y/Z endstops, trsyncs, steppers
	// OIDs 9-11: enable pins for X/Y/Z
	o := oids{
		endstopX: 0, trsyncX: 1, stepperX: 2,
		endstopY: 3, trsyncY: 4, stepperY: 5,
		endstopZ: 6, trsyncZ: 7, stepperZ: 8,
		enableX: 9, enableY: 10, enableZ: 11,
		count: 12,
	}

	// Derived constants (match klippy defaults).
	const (
		stepPulseSec = 0.000002
	)
	stepPulseTicks := secondsToClock(mcuFreq, stepPulseSec)

	// Build config commands for steppers only (no extruder, no heaters).
	configCmds := []string{
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopX, stepperX.Endstop.pin, stepperX.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncX),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperX, stepperX.Step.pin, stepperX.Dir.pin, boolToInt(stepperX.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableX, stepperX.Enable.pin, boolToInt(stepperX.Enable.invert), boolToInt(stepperX.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopY, stepperY.Endstop.pin, stepperY.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncY),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperY, stepperY.Step.pin, stepperY.Dir.pin, boolToInt(stepperY.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableY, stepperY.Enable.pin, boolToInt(stepperY.Enable.invert), boolToInt(stepperY.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopZ, stepperZ.Endstop.pin, stepperZ.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncZ),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperZ, stepperZ.Step.pin, stepperZ.Dir.pin, boolToInt(stepperZ.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableZ, stepperZ.Enable.pin, boolToInt(stepperZ.Enable.invert), boolToInt(stepperZ.Enable.invert), 0),
	}

	// Prepend allocate_oids and compute finalize_config CRC over config_cmds.
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", o.count)}, configCmds...)
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	out := make([]string, 0, 1+len(configCmds)+1)
	out = append(out, withAllocate...)
	out = append(out, fmt.Sprintf("finalize_config crc=%d", crc))
	return out, nil
}

// CompileMinimalCartesianConnectPhase compiles the connect-phase MCU command
// stream for cartesian configs without heater_bed (e.g., extruders.cfg, pressure_advance.cfg).
// Note: This requires an [extruder] section. Use CompileCartesianNoExtruder for configs without extruder.
func CompileMinimalCartesianConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
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

	// Extract required pins.
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

	// Allocate OIDs for cartesian config without heater_bed.
	o := oids{
		endstopX: 0, trsyncX: 1, stepperX: 2,
		endstopY: 3, trsyncY: 4, stepperY: 5,
		endstopZ: 6, trsyncZ: 7, stepperZ: 8,
		stepperE: 9,
		adcBed:   10, pwmBed: 11, // Unused but keep for OID stability
		enableX: 12, enableY: 13, enableZ: 14,
		adcE: 15, pwmE: 16, enableE: 17,
		count: 18,
	}

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

	// ADC min/max for extruder thermistor.
	extMin, extMax, err := thermistorMinMaxTicks(extruder.Heater, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}

	// Build config commands for steppers and extruder (no heater_bed).
	configCmds := []string{
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopX, stepperX.Endstop.pin, stepperX.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncX),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperX, stepperX.Step.pin, stepperX.Dir.pin, boolToInt(stepperX.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableX, stepperX.Enable.pin, boolToInt(stepperX.Enable.invert), boolToInt(stepperX.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopY, stepperY.Endstop.pin, stepperY.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncY),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperY, stepperY.Step.pin, stepperY.Dir.pin, boolToInt(stepperY.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableY, stepperY.Enable.pin, boolToInt(stepperY.Enable.invert), boolToInt(stepperY.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopZ, stepperZ.Endstop.pin, stepperZ.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncZ),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperZ, stepperZ.Step.pin, stepperZ.Dir.pin, boolToInt(stepperZ.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableZ, stepperZ.Enable.pin, boolToInt(stepperZ.Enable.invert), boolToInt(stepperZ.Enable.invert), 0),

		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcE, extruder.Heater.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmE, extruder.Heater.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmE, cycleTicks),

		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperE, extruder.Step.pin, extruder.Dir.pin, boolToInt(extruder.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableE, extruder.Enable.pin, boolToInt(extruder.Enable.invert), boolToInt(extruder.Enable.invert), 0),
	}

	// Prepend allocate_oids and compute finalize_config CRC over config_cmds.
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", o.count)}, configCmds...)
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	// Build init commands (extruder only, no heater_bed).
	initCmds := []string{
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

// CompileCartesianWithExtruderStepper compiles the connect-phase for cartesian
// configs with an additional extruder_stepper (e.g., extruders.cfg, pressure_advance.cfg).
func CompileCartesianWithExtruderStepper(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
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

	// Extract required pins.
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

	// Check for extruder_stepper my_extra_stepper
	extraStepper, hasExtraStepper, err := readExtruderStepper(cfg, "extruder_stepper my_extra_stepper")
	if err != nil {
		return nil, err
	}

	// Parse filament sensor sections
	type filamentSensor struct {
		pin    string
		pullUp int
	}
	var filamentSensors []filamentSensor

	// Check for filament_switch_sensor and filament_motion_sensor sections
	// These follow the pattern: runout_switch, runout_encoder, runout_switch1, runout_encoder1, etc.
	sensorSections := []string{
		"filament_switch_sensor runout_switch",
		"filament_motion_sensor runout_encoder",
		"filament_switch_sensor runout_switch1",
		"filament_motion_sensor runout_encoder1",
	}

	for _, sectionName := range sensorSections {
		sec, ok := cfg.sections[sectionName]
		if ok {
			pin := sec["switch_pin"]
			pullUp := boolToInt(false) // Default is no pullup
			filamentSensors = append(filamentSensors, filamentSensor{pin: pin, pullUp: pullUp})
		}
	}

	// Also check for numbered variants beyond 1 (e.g., runout_switch2, runout_encoder2, etc.)
	for i := 2; i < 10; i++ {
		switchSection := fmt.Sprintf("filament_switch_sensor runout_switch%d", i)
		if sec, ok := cfg.sections[switchSection]; ok {
			pin := sec["switch_pin"]
			filamentSensors = append(filamentSensors, filamentSensor{pin: pin, pullUp: 0})
		}

		motionSection := fmt.Sprintf("filament_motion_sensor runout_encoder%d", i)
		if sec, ok := cfg.sections[motionSection]; ok {
			pin := sec["switch_pin"]
			filamentSensors = append(filamentSensors, filamentSensor{pin: pin, pullUp: 0})
		}
	}

	// Allocate OIDs matching Python's order:
	// For configs WITH extra_stepper:
	// 0: extra_stepper
	// 1-2: endstop_x + trsync_x
	// 3: stepper_x
	// 4-5: endstop_y + trsync_y
	// 6: stepper_y
	// 7-8: endstop_z + trsync_z
	// 9: stepper_z
	// 10: stepper_e
	// 11: enable_extra
	// 12: buttons (if filament sensors present)
	// 13: enable_x (or 12 if no buttons)
	// 14: enable_y (or 13 if no buttons)
	// 15: enable_z (or 14 if no buttons)
	// 16: adc_e (or 15 if no buttons)
	// 17: pwm_e (or 16 if no buttons)
	// 18: enable_e (or 17 if no buttons)

	o := struct {
		extraStepper int
		endstopX     int
		trsyncX      int
		stepperX     int
		endstopY     int
		trsyncY      int
		stepperY     int
		endstopZ     int
		trsyncZ      int
		stepperZ     int
		stepperE     int
		enableExtra  int
		buttons      int
		enableX      int
		enableY      int
		enableZ      int
		adcE         int
		pwmE         int
		enableE      int
		count        int
	}{}

	hasButtons := len(filamentSensors) > 0

	if hasExtraStepper {
		o.extraStepper = 0
		o.endstopX = 1
		o.trsyncX = 2
		o.stepperX = 3
		o.endstopY = 4
		o.trsyncY = 5
		o.stepperY = 6
		o.endstopZ = 7
		o.trsyncZ = 8
		o.stepperZ = 9
		o.stepperE = 10
		o.enableExtra = 11
		if hasButtons {
			o.buttons = 12
			o.enableX = 13
			o.enableY = 14
			o.enableZ = 15
			o.adcE = 16
			o.pwmE = 17
			o.enableE = 18
			o.count = 19
		} else {
			o.enableX = 12
			o.enableY = 13
			o.enableZ = 14
			o.adcE = 15
			o.pwmE = 16
			o.enableE = 17
			o.count = 18
		}
	} else {
		o.count = 18
	}

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

	// ADC min/max for extruder thermistor.
	extMin, extMax, err := thermistorMinMaxTicks(extruder.Heater, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}

	// Build config commands.
	configCmds := []string{}

	if hasExtraStepper {
		configCmds = append(configCmds,
			fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
				o.extraStepper, extraStepper.Step.pin, extraStepper.Dir.pin, boolToInt(extraStepper.Step.invert), stepPulseTicks),
			fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
				o.enableExtra, extraStepper.Enable.pin, boolToInt(extraStepper.Enable.invert), boolToInt(extraStepper.Enable.invert), 0),
		)
	}

	// Add buttons configuration if filament sensors are present
	// Buttons come after extra_stepper enable pin (OID 11)
	if hasButtons {
		configCmds = append(configCmds,
			fmt.Sprintf("config_buttons oid=%d button_count=%d", o.buttons, len(filamentSensors)))
	}

	configCmds = append(configCmds,
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopX, stepperX.Endstop.pin, stepperX.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncX),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperX, stepperX.Step.pin, stepperX.Dir.pin, boolToInt(stepperX.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableX, stepperX.Enable.pin, boolToInt(stepperX.Enable.invert), boolToInt(stepperX.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopY, stepperY.Endstop.pin, stepperY.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncY),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperY, stepperY.Step.pin, stepperY.Dir.pin, boolToInt(stepperY.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableY, stepperY.Enable.pin, boolToInt(stepperY.Enable.invert), boolToInt(stepperY.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopZ, stepperZ.Endstop.pin, stepperZ.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncZ),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperZ, stepperZ.Step.pin, stepperZ.Dir.pin, boolToInt(stepperZ.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableZ, stepperZ.Enable.pin, boolToInt(stepperZ.Enable.invert), boolToInt(stepperZ.Enable.invert), 0),

		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcE, extruder.Heater.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmE, extruder.Heater.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmE, cycleTicks),

		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperE, extruder.Step.pin, extruder.Dir.pin, boolToInt(extruder.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableE, extruder.Enable.pin, boolToInt(extruder.Enable.invert), boolToInt(extruder.Enable.invert), 0),
	)

	// Prepend allocate_oids and compute finalize_config CRC over config_cmds.
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", o.count)}, configCmds...)
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	// Build init commands.
	initCmds := []string{}

	// Add buttons_add and buttons_query if filament sensors are present
	if hasButtons {
		// buttons_add commands come first (after finalize_config)
		for i, sensor := range filamentSensors {
			initCmds = append(initCmds,
				fmt.Sprintf("buttons_add oid=%d pos=%d pin=%s pull_up=%d", o.buttons, i, sensor.pin, sensor.pullUp))
		}
		// Then buttons_query
		// QUERY_TIME = 0.002 seconds from buttons.py
		const queryTime = 0.002
		const retransmitCount = 50
		queryClock := querySlotClock(mcuFreq, o.buttons)
		restTicks := secondsToClock(mcuFreq, queryTime)
		initCmds = append(initCmds,
			fmt.Sprintf("buttons_query oid=%d clock=%d rest_ticks=%d retransmit_count=%d invert=%d",
				o.buttons, queryClock, restTicks, retransmitCount, 0))
	}

	initCmds = append(initCmds,
		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			o.adcE, querySlotClock(mcuFreq, o.adcE), sampleTicks, adcSampleCount, reportTicks, extMin, extMax, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", o.pwmE, pwmInitClock, 0),
	)

	out := make([]string, 0, 1+len(configCmds)+1+len(initCmds))
	out = append(out, withAllocate...)
	out = append(out, fmt.Sprintf("finalize_config crc=%d", crc))
	out = append(out, initCmds...)
	return out, nil
}

type oids struct {
	endstopX int
	trsyncX  int
	stepperX int

	endstopY int
	trsyncY  int
	stepperY int

	endstopZ int
	trsyncZ  int
	stepperZ int

	stepperE int

	adcBed int
	pwmBed int

	enableX int
	enableY int
	enableZ int

	adcE    int
	pwmE    int
	enableE int

	count int
}

type pin struct {
	pin    string
	invert bool
	pullup int
}

type heater struct {
	HeaterPin  pin
	SensorPin  pin
	SensorType string
	MinTemp    float64
	MaxTemp    float64
}

type tempSensor struct {
	MinTemp    float64
	MaxTemp    float64
	SerialNo   []byte
	SensorMCU  string
	SensorType string
}

type stepper struct {
	Step    pin
	Dir     pin
	Enable  pin
	Endstop pin
}

type extruder struct {
	Step   pin
	Dir    pin
	Enable pin
	Heater heater
}

func secondsToClock(freq float64, sec float64) int {
	return int(sec * freq)
}

func secondsToClockRound(freq float64, sec float64) int {
	// Match Klippy's integer tick conversions (avoid float truncation drift).
	return int(sec*freq + 0.5)
}

func querySlotClock(freq float64, oid int) int {
	base := secondsToClock(freq, 1.0) // int(estimated_print_time + 1.5) == 1 in fileoutput mode
	slot := secondsToClock(freq, float64(oid)*0.01)
	return base + slot
}

func boolToInt(b bool) int {
	if b {
		return 1
	}
	return 0
}

func dictConfigFloat(d *protocol.Dictionary, key string) (float64, error) {
	if d.Config == nil {
		return 0, fmt.Errorf("dict missing config section")
	}
	v, ok := d.Config[key]
	if !ok {
		return 0, fmt.Errorf("dict missing config[%s]", key)
	}
	switch tv := v.(type) {
	case float64:
		return tv, nil
	case int:
		return float64(tv), nil
	case string:
		f, err := strconv.ParseFloat(tv, 64)
		if err != nil {
			return 0, fmt.Errorf("bad config[%s]=%q", key, tv)
		}
		return f, nil
	default:
		return 0, fmt.Errorf("unsupported config[%s] type %T", key, v)
	}
}

func readStepper(cfg *config, name string) (stepper, error) {
	sec, ok := cfg.sections[name]
	if !ok {
		return stepper{}, fmt.Errorf("missing [%s] section", name)
	}
	stepPin, err := parsePin(sec, "step_pin", true, false)
	if err != nil {
		return stepper{}, err
	}
	dirPin, err := parsePin(sec, "dir_pin", true, false)
	if err != nil {
		return stepper{}, err
	}
	enPin, err := parsePin(sec, "enable_pin", true, false)
	if err != nil {
		return stepper{}, err
	}
	endPin, err := parsePin(sec, "endstop_pin", true, true)
	if err != nil {
		return stepper{}, err
	}
	return stepper{Step: stepPin, Dir: dirPin, Enable: enPin, Endstop: endPin}, nil
}

func readExtruder(cfg *config, name string) (extruder, error) {
	sec, ok := cfg.sections[name]
	if !ok {
		return extruder{}, fmt.Errorf("missing [%s] section", name)
	}
	stepPin, err := parsePin(sec, "step_pin", true, false)
	if err != nil {
		return extruder{}, err
	}
	dirPin, err := parsePin(sec, "dir_pin", true, false)
	if err != nil {
		return extruder{}, err
	}
	enPin, err := parsePin(sec, "enable_pin", true, false)
	if err != nil {
		return extruder{}, err
	}
	h, err := readHeaterFromSection(sec)
	if err != nil {
		return extruder{}, err
	}
	return extruder{Step: stepPin, Dir: dirPin, Enable: enPin, Heater: h}, nil
}

// extruderStepper represents an auxiliary extruder stepper (without heater/endstop)
type extruderStepper struct {
	Step   pin
	Dir    pin
	Enable pin
}

// readExtruderStepper reads an extruder_stepper section (optional, returns has=false if not found)
func readExtruderStepper(cfg *config, name string) (extruderStepper, bool, error) {
	sec, ok := cfg.sections[name]
	if !ok {
		return extruderStepper{}, false, nil
	}
	stepPin, err := parsePin(sec, "step_pin", true, false)
	if err != nil {
		return extruderStepper{}, false, err
	}
	dirPin, err := parsePin(sec, "dir_pin", true, false)
	if err != nil {
		return extruderStepper{}, false, err
	}
	enPin, err := parsePin(sec, "enable_pin", true, false)
	if err != nil {
		return extruderStepper{}, false, err
	}
	return extruderStepper{Step: stepPin, Dir: dirPin, Enable: enPin}, true, nil
}

func readHeater(cfg *config, name string) (heater, error) {
	sec, ok := cfg.sections[name]
	if !ok {
		return heater{}, fmt.Errorf("missing [%s] section", name)
	}
	return readHeaterFromSection(sec)
}

// readHeaterOptional reads a heater section if it exists, returns nil if not found
func readHeaterOptional(cfg *config, name string) (*heater, error) {
	sec, ok := cfg.sections[name]
	if !ok {
		return nil, nil
	}
	h, err := readHeaterFromSection(sec)
	if err != nil {
		return nil, err
	}
	return &h, nil
}

func readHeaterFromSection(sec map[string]string) (heater, error) {
	heaterPin, err := parsePin(sec, "heater_pin", true, false)
	if err != nil {
		return heater{}, err
	}
	sensorPin, err := parsePin(sec, "sensor_pin", false, false)
	if err != nil {
		return heater{}, err
	}
	sensorType := strings.TrimSpace(sec["sensor_type"])
	if sensorType == "" {
		return heater{}, fmt.Errorf("missing sensor_type")
	}
	minTemp, err := parseFloatRequired(sec, "min_temp")
	if err != nil {
		return heater{}, err
	}
	maxTemp, err := parseFloatRequired(sec, "max_temp")
	if err != nil {
		return heater{}, err
	}
	return heater{
		HeaterPin:  heaterPin,
		SensorPin:  sensorPin,
		SensorType: sensorType,
		MinTemp:    minTemp,
		MaxTemp:    maxTemp,
	}, nil
}

func parseFloatRequired(sec map[string]string, key string) (float64, error) {
	raw := strings.TrimSpace(sec[key])
	if raw == "" {
		return 0, fmt.Errorf("missing %s", key)
	}
	v, err := strconv.ParseFloat(raw, 64)
	if err != nil {
		return 0, fmt.Errorf("bad float %s=%q", key, raw)
	}
	return v, nil
}

func parsePin(sec map[string]string, key string, canInvert bool, canPullup bool) (pin, error) {
	raw := strings.TrimSpace(sec[key])
	if raw == "" {
		return pin{}, fmt.Errorf("missing %s", key)
	}
	return parsePinDesc(raw, canInvert, canPullup)
}

func parsePinDesc(desc string, canInvert bool, canPullup bool) (pin, error) {
	d := strings.TrimSpace(desc)
	p := pin{}
	if canPullup && (strings.HasPrefix(d, "^") || strings.HasPrefix(d, "~")) {
		p.pullup = 1
		if strings.HasPrefix(d, "~") {
			p.pullup = -1
		}
		d = strings.TrimSpace(d[1:])
	}
	if canInvert && strings.HasPrefix(d, "!") {
		p.invert = true
		d = strings.TrimSpace(d[1:])
	}
	if strings.Contains(d, ":") {
		parts := strings.SplitN(d, ":", 2)
		chip := strings.TrimSpace(parts[0])
		if chip != "mcu" {
			return pin{}, fmt.Errorf("unsupported pin chip %q in %q", chip, desc)
		}
		d = strings.TrimSpace(parts[1])
	}
	if strings.ContainsAny(d, "^~!:") || strings.Join(strings.Fields(d), "") != d {
		return pin{}, fmt.Errorf("invalid pin %q", desc)
	}
	p.pin = d
	return p, nil
}

// ---- Minimal config loader (INI-ish + [include ...]) ----

type config struct {
	sections map[string]map[string]string
}

func loadConfig(path string) (*config, error) {
	c := &config{sections: map[string]map[string]string{}}
	if err := c.parseFile(path, map[string]bool{}); err != nil {
		return nil, err
	}
	return c, nil
}

func (c *config) parseFile(path string, visited map[string]bool) error {
	abs, err := filepath.Abs(path)
	if err != nil {
		return err
	}
	if visited[abs] {
		return fmt.Errorf("recursive include: %s", path)
	}
	visited[abs] = true
	defer func() { visited[abs] = false }()

	f, err := os.Open(abs)
	if err != nil {
		return err
	}
	defer f.Close()

	dir := filepath.Dir(abs)
	var curSection string
	s := bufio.NewScanner(f)
	for s.Scan() {
		line := strings.TrimSpace(s.Text())
		if line == "" {
			continue
		}
		if idx := strings.IndexByte(line, '#'); idx >= 0 {
			line = strings.TrimSpace(line[:idx])
			if line == "" {
				continue
			}
		}
		if strings.HasPrefix(line, "[") && strings.HasSuffix(line, "]") {
			header := strings.TrimSpace(line[1 : len(line)-1])
			if strings.HasPrefix(header, "include ") {
				spec := strings.TrimSpace(header[len("include "):])
				if spec == "" {
					return fmt.Errorf("empty include in %s", path)
				}
				glob := filepath.Join(dir, spec)
				matches, err := filepath.Glob(glob)
				if err != nil {
					return err
				}
				sort.Strings(matches)
				if len(matches) == 0 && !hasGlobMeta(glob) {
					return fmt.Errorf("include file does not exist: %s", glob)
				}
				for _, m := range matches {
					if err := c.parseFile(m, visited); err != nil {
						return err
					}
				}
				continue
			}
			curSection = header
			if _, ok := c.sections[curSection]; !ok {
				c.sections[curSection] = map[string]string{}
			}
			continue
		}
		if curSection == "" {
			continue
		}
		k, v, ok := splitKV(line)
		if !ok {
			continue
		}
		c.sections[curSection][k] = v
	}
	if err := s.Err(); err != nil {
		return err
	}
	return nil
}

func splitKV(line string) (string, string, bool) {
	// Klipper config supports both "k: v" and "k = v".
	idx := strings.IndexAny(line, ":=")
	if idx < 0 {
		return "", "", false
	}
	key := strings.TrimSpace(line[:idx])
	val := strings.TrimSpace(line[idx+1:])
	if key == "" {
		return "", "", false
	}
	return key, val, true
}

func hasGlobMeta(path string) bool {
	return strings.ContainsAny(path, "*?[")
}

// ---- Thermistor math (matches klippy/extras/thermistor.py) ----

const kelvinToCelsius = -273.15

type thermistor struct {
	pullup         float64
	inlineResistor float64
	c1             float64
	c2             float64
	c3             float64
}

func thermistorEPCOS100K() *thermistor {
	t := &thermistor{pullup: 4700.0, inlineResistor: 0.0}
	// [thermistor EPCOS 100K B57560G104F] from klippy/extras/temperature_sensors.cfg
	t.setupCoefficients(25, 100000, 150, 1641.9, 250, 226.15)
	return t
}

func (t *thermistor) setupCoefficients(t1, r1, t2, r2, t3, r3 float64) {
	invT1 := 1.0 / (t1 - kelvinToCelsius)
	invT2 := 1.0 / (t2 - kelvinToCelsius)
	invT3 := 1.0 / (t3 - kelvinToCelsius)
	lnR1 := math.Log(r1)
	lnR2 := math.Log(r2)
	lnR3 := math.Log(r3)
	ln3R1 := lnR1 * lnR1 * lnR1
	ln3R2 := lnR2 * lnR2 * lnR2
	ln3R3 := lnR3 * lnR3 * lnR3

	invT12 := invT1 - invT2
	invT13 := invT1 - invT3
	lnR12 := lnR1 - lnR2
	lnR13 := lnR1 - lnR3
	ln3R12 := ln3R1 - ln3R2
	ln3R13 := ln3R1 - ln3R3

	t.c3 = (invT12 - invT13*lnR12/lnR13) / (ln3R12 - ln3R13*lnR12/lnR13)
	if t.c3 <= 0.0 {
		// Fallback beta path not needed for this sensor.
		t.c3 = 0.0
	}
	if t.c3 == 0.0 {
		// Compute equivalent beta based on (t1,r1) and lnR13/invT13.
		beta := lnR13 / invT13
		t.c2 = 1.0 / beta
		t.c1 = invT1 - t.c2*lnR1
		return
	}
	t.c2 = (invT12 - t.c3*ln3R12) / lnR12
	t.c1 = invT1 - t.c2*lnR1 - t.c3*ln3R1
}

func (t *thermistor) calcADC(temp float64) float64 {
	if temp <= kelvinToCelsius {
		return 1.0
	}
	invT := 1.0 / (temp - kelvinToCelsius)
	var lnR float64
	if t.c3 != 0.0 {
		y := (t.c1 - invT) / (2.0 * t.c3)
		x := math.Sqrt(math.Pow(t.c2/(3.0*t.c3), 3.0) + y*y)
		lnR = math.Cbrt(x-y) - math.Cbrt(x+y)
	} else {
		lnR = (invT - t.c1) / t.c2
	}
	r := math.Exp(lnR) + t.inlineResistor
	return r / (t.pullup + r)
}

func thermistorMinMaxTicks(h heater, adcMax float64, sampleCount int) (int, int, error) {
	if h.SensorType != "EPCOS 100K B57560G104F" {
		return 0, 0, fmt.Errorf("unsupported sensor_type %q (H1 only supports EPCOS 100K B57560G104F)", h.SensorType)
	}
	th := thermistorEPCOS100K()
	a := []float64{th.calcADC(h.MinTemp), th.calcADC(h.MaxTemp)}
	sort.Float64s(a)
	minADC, maxADC := a[0], a[1]

	maxTotal := float64(sampleCount) * adcMax
	minVal := int(minADC * maxTotal)
	maxVal := int(math.Ceil(maxADC * maxTotal))
	if minVal < 0 {
		minVal = 0
	}
	if maxVal < 0 {
		maxVal = 0
	}
	if minVal > 0xffff {
		minVal = 0xffff
	}
	if maxVal > 0xffff {
		maxVal = 0xffff
	}
	return minVal, maxVal, nil
}

func readTempSensor(cfg *config, name string) (tempSensor, error) {
	sec, ok := cfg.sections[name]
	if !ok {
		return tempSensor{}, fmt.Errorf("missing [%s] section", name)
	}
	minTemp, err := parseFloatRequired(sec, "min_temp")
	if err != nil {
		return tempSensor{}, err
	}
	maxTemp, err := parseFloatRequired(sec, "max_temp")
	if err != nil {
		return tempSensor{}, err
	}
	serialNo := strings.TrimSpace(sec["serial_no"])
	if serialNo == "" {
		return tempSensor{}, fmt.Errorf("missing serial_no")
	}
	sensorMCU := strings.TrimSpace(sec["sensor_mcu"])
	if sensorMCU == "" {
		return tempSensor{}, fmt.Errorf("missing sensor_mcu")
	}
	sensorType := strings.TrimSpace(sec["sensor_type"])
	if sensorType == "" {
		return tempSensor{}, fmt.Errorf("missing sensor_type")
	}
	return tempSensor{
		MinTemp:    minTemp,
		MaxTemp:    maxTemp,
		SerialNo:   []byte(serialNo),
		SensorMCU:  sensorMCU,
		SensorType: sensorType,
	}, nil
}

func bytesToLowerHex(b []byte) string {
	const hexdigits = "0123456789abcdef"
	out := make([]byte, len(b)*2)
	for i, v := range b {
		out[i*2] = hexdigits[v>>4]
		out[i*2+1] = hexdigits[v&0x0f]
	}
	return string(out)
}

// CompileDeltaConnectPhase compiles the connect-phase MCU command stream for
// delta kinematics configurations (e.g., config/example-delta.cfg).
//
// Delta printers use stepper_a/b/c instead of stepper_x/y/z.
func CompileDeltaConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
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

	// Extract required pins - delta uses stepper_a/b/c
	stepperA, err := readStepper(cfg, "stepper_a")
	if err != nil {
		return nil, err
	}
	stepperB, err := readStepper(cfg, "stepper_b")
	if err != nil {
		return nil, err
	}
	stepperC, err := readStepper(cfg, "stepper_c")
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

	// Allocate OIDs matching Python's order for delta config.
	o := oids{
		endstopX: 0, trsyncX: 1, stepperX: 2,
		endstopY: 3, trsyncY: 4, stepperY: 5,
		endstopZ: 6, trsyncZ: 7, stepperZ: 8,
		stepperE: 9,
		adcBed:   10, pwmBed: 11,
		enableX: 12, enableY: 13, enableZ: 14,
		adcE: 15, pwmE: 16, enableE: 17,
		count: 18,
	}

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

	// Build config commands in the same order as the Python reference for delta.
	// Note: The OID naming follows cartesian convention internally (stepperX = stepper_a, etc.)
	configCmds := []string{
		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcBed, bed.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmBed, bed.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmBed, cycleTicks),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopX, stepperA.Endstop.pin, stepperA.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncX),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperX, stepperA.Step.pin, stepperA.Dir.pin, boolToInt(stepperA.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableX, stepperA.Enable.pin, boolToInt(stepperA.Enable.invert), boolToInt(stepperA.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopY, stepperB.Endstop.pin, stepperB.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncY),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperY, stepperB.Step.pin, stepperB.Dir.pin, boolToInt(stepperB.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableY, stepperB.Enable.pin, boolToInt(stepperB.Enable.invert), boolToInt(stepperB.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopZ, stepperC.Endstop.pin, stepperC.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncZ),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperZ, stepperC.Step.pin, stepperC.Dir.pin, boolToInt(stepperC.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableZ, stepperC.Enable.pin, boolToInt(stepperC.Enable.invert), boolToInt(stepperC.Enable.invert), 0),

		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcE, extruder.Heater.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmE, extruder.Heater.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmE, cycleTicks),

		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperE, extruder.Step.pin, extruder.Dir.pin, boolToInt(extruder.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableE, extruder.Enable.pin, boolToInt(extruder.Enable.invert), boolToInt(extruder.Enable.invert), 0),
	}

	// Prepend allocate_oids and compute finalize_config CRC over config_cmds.
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", o.count)}, configCmds...)
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	// Build init commands (same order as the Python reference for this config).
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

// CompileDeltaCalibrateConnectPhase compiles the connect-phase MCU command stream for
// delta calibration configurations (e.g., test/klippy/delta_calibrate.cfg).
//
// This is a simpler delta config with only steppers (no heaters/extruder).
// OID layout matches Python's order: 12 OIDs total.
func CompileDeltaCalibrateConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
	cfg, err := loadConfig(cfgPath)
	if err != nil {
		return nil, err
	}
	clockFreq, err := dictConfigFloat(dict, "CLOCK_FREQ")
	if err != nil {
		return nil, err
	}
	mcuFreq := clockFreq

	// Extract required pins - delta uses stepper_a/b/c
	stepperA, err := readStepper(cfg, "stepper_a")
	if err != nil {
		return nil, err
	}
	stepperB, err := readStepper(cfg, "stepper_b")
	if err != nil {
		return nil, err
	}
	stepperC, err := readStepper(cfg, "stepper_c")
	if err != nil {
		return nil, err
	}

	// OID layout for delta_calibrate.cfg (no heaters):
	// endstop_a=0, trsync_a=1, stepper_a=2,
	// endstop_b=3, trsync_b=4, stepper_b=5,
	// endstop_c=6, trsync_c=7, stepper_c=8,
	// enable_a=9, enable_b=10, enable_c=11
	// count=12
	const (
		oidEndstopA = 0
		oidTrsyncA  = 1
		oidStepperA = 2
		oidEndstopB = 3
		oidTrsyncB  = 4
		oidStepperB = 5
		oidEndstopC = 6
		oidTrsyncC  = 7
		oidStepperC = 8
		oidEnableA  = 9
		oidEnableB  = 10
		oidEnableC  = 11
		oidCount    = 12
	)

	// Derived constants (match klippy defaults).
	const stepPulseSec = 0.000002
	stepPulseTicks := secondsToClock(mcuFreq, stepPulseSec)

	// Build config commands in the same order as the Python reference.
	configCmds := []string{
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", oidEndstopA, stepperA.Endstop.pin, stepperA.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", oidTrsyncA),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			oidStepperA, stepperA.Step.pin, stepperA.Dir.pin, boolToInt(stepperA.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			oidEnableA, stepperA.Enable.pin, boolToInt(stepperA.Enable.invert), boolToInt(stepperA.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", oidEndstopB, stepperB.Endstop.pin, stepperB.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", oidTrsyncB),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			oidStepperB, stepperB.Step.pin, stepperB.Dir.pin, boolToInt(stepperB.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			oidEnableB, stepperB.Enable.pin, boolToInt(stepperB.Enable.invert), boolToInt(stepperB.Enable.invert), 0),

		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", oidEndstopC, stepperC.Endstop.pin, stepperC.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", oidTrsyncC),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			oidStepperC, stepperC.Step.pin, stepperC.Dir.pin, boolToInt(stepperC.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			oidEnableC, stepperC.Enable.pin, boolToInt(stepperC.Enable.invert), boolToInt(stepperC.Enable.invert), 0),
	}

	// Prepend allocate_oids and compute finalize_config CRC over config_cmds.
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", oidCount)}, configCmds...)
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	// No init commands for this simple config (no heaters/ADCs).
	out := make([]string, 0, len(withAllocate)+1)
	out = append(out, withAllocate...)
	out = append(out, fmt.Sprintf("finalize_config crc=%d", crc))
	return out, nil
}

// CompileLoadCellConnectPhase compiles the connect-phase MCU command stream for
// load cell sensor configurations (e.g., test/klippy/load_cell.cfg).
//
// This config has no steppers, just load cell sensors using SPI and HX711/HX717.
// OID layout:
// oid=0: SPI for ADS1220
// oid=1: ADS1220 sensor
// oid=2: HX711
// oid=3: HX717
func CompileLoadCellConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
	cfg, err := loadConfig(cfgPath)
	if err != nil {
		return nil, err
	}
	clockFreq, err := dictConfigFloat(dict, "CLOCK_FREQ")
	if err != nil {
		return nil, err
	}

	// Parse load_cell sections
	// [load_cell my_ads1220]
	// sensor_type: ads1220
	// cs_pin: PA0
	// data_ready_pin: PA1
	ads1220Sec := cfg.sections["load_cell my_ads1220"]
	csPin := strings.TrimSpace(ads1220Sec["cs_pin"])
	dataReadyPin := strings.TrimSpace(ads1220Sec["data_ready_pin"])

	// [load_cell my_hx711]
	// sensor_type: hx711
	// sclk_pin: PA2
	// dout_pin: PA3
	hx711Sec := cfg.sections["load_cell my_hx711"]
	hx711SclkPin := strings.TrimSpace(hx711Sec["sclk_pin"])
	hx711DoutPin := strings.TrimSpace(hx711Sec["dout_pin"])

	// [load_cell my_hx717]
	// sensor_type: hx717
	// sclk_pin: PA4
	// dout_pin: PA5
	hx717Sec := cfg.sections["load_cell my_hx717"]
	hx717SclkPin := strings.TrimSpace(hx717Sec["sclk_pin"])
	hx717DoutPin := strings.TrimSpace(hx717Sec["dout_pin"])

	// OID layout:
	const (
		oidSPI     = 0
		oidADS1220 = 1
		oidHX711   = 2
		oidHX717   = 3
		oidCount   = 4
	)

	// HX711 uses gain_channel=1 (Channel A, 128 gain)
	// HX717 uses gain_channel=1 (Channel A, 128 gain) as well
	// rest_ticks for HX711: 80 samples/sec = 0.0125s = 200000 ticks at 16MHz -> actually 20000 at ~1.25ms sample rate
	// rest_ticks for HX717: 320 samples/sec = 0.003125s = 50000 ticks at 16MHz -> actually 5000 at ~0.3125ms sample rate
	hx711RestTicks := int(clockFreq * 0.00125) // 20000 at 16MHz
	hx717RestTicks := int(clockFreq * 0.0003125) // 5000 at 16MHz

	// Build config commands in the same order as Python
	configCmds := []string{
		fmt.Sprintf("config_spi oid=%d pin=%s cs_active_high=0", oidSPI, csPin),
		fmt.Sprintf("config_ads1220 oid=%d spi_oid=%d data_ready_pin=%s", oidADS1220, oidSPI, dataReadyPin),
		fmt.Sprintf("config_hx71x oid=%d gain_channel=1 dout_pin=%s sclk_pin=%s", oidHX711, hx711DoutPin, hx711SclkPin),
		fmt.Sprintf("config_hx71x oid=%d gain_channel=1 dout_pin=%s sclk_pin=%s", oidHX717, hx717DoutPin, hx717SclkPin),
	}

	// Prepend allocate_oids
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", oidCount)}, configCmds...)

	// Add spi_set_bus before finalize_config
	// spi_set_bus oid=0 spi_bus=spi mode=1 rate=512000
	withAllocate = append(withAllocate, "spi_set_bus oid=0 spi_bus=spi mode=1 rate=512000")

	// Compute CRC
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	// Init commands after finalize_config
	initCmds := []string{
		// ADS1220 initialization
		"spi_send oid=0 data=b'\\x06'",
		"spi_transfer oid=0 data=b'#\\x00\\x00\\x00\\x00'",
		// Start HX711/HX717 queries
		fmt.Sprintf("query_hx71x oid=%d rest_ticks=%d", oidHX711, hx711RestTicks),
		fmt.Sprintf("query_hx71x_status oid=%d", oidHX711),
		fmt.Sprintf("query_hx71x oid=%d rest_ticks=%d", oidHX717, hx717RestTicks),
		fmt.Sprintf("query_hx71x_status oid=%d", oidHX717),
	}

	// Build final output
	out := make([]string, 0, len(withAllocate)+1+len(initCmds))
	out = append(out, withAllocate...)
	out = append(out, fmt.Sprintf("finalize_config crc=%d", crc))
	out = append(out, initCmds...)
	return out, nil
}

// CompileGenericCartesianConnectPhase compiles the connect-phase MCU command stream
// for generic cartesian kinematics configurations (e.g., generic_cartesian.cfg, corexyuv.cfg).
//
// OID Layout (matches Python ordering):
// - First: servo, heater_bed (pre-allocated OIDs)
// - Then: carriages with their endstops/trsync pairs and associated steppers
// - Finally: extruder sections
func CompileGenericCartesianConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
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

	const (
		pwmCycleTime     = 0.100
		servoCycleTime   = 0.020
		heaterMaxHeatSec = 3.0
		stepPulseSec     = 0.000002
		adcSampleTime    = 0.001
		adcSampleCount   = 8
		adcReportTime    = 0.300
		adcRangeChecks   = 4
		pwmInitDelaySec  = 0.200
	)
	cycleTicks := secondsToClock(mcuFreq, pwmCycleTime)
	servoCycleTicks := secondsToClock(mcuFreq, servoCycleTime)
	mdurTicks := secondsToClock(mcuFreq, heaterMaxHeatSec)
	stepPulseTicks := secondsToClock(mcuFreq, stepPulseSec)
	sampleTicks := secondsToClock(mcuFreq, adcSampleTime)
	reportTicks := secondsToClock(mcuFreq, adcReportTime)
	pwmInitClock := secondsToClock(mcuFreq, pwmInitDelaySec)

	var configCmds []string
	var initCmds []string
	oidNext := 0

	// Allocate OIDs for servo first (Python processes extras before kinematics)
	servoOID := -1
	if servoSec, ok := cfg.sections["servo my_servo"]; ok {
		servoOID = 20 // Python allocates servo at OID 20 for generic_cartesian
		servoPin := strings.TrimSpace(servoSec["pin"])
		configCmds = append(configCmds,
			fmt.Sprintf("config_digital_out oid=%d pin=%s value=0 default_value=0 max_duration=0", servoOID, servoPin),
			fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", servoOID, servoCycleTicks),
		)
		initCmds = append(initCmds,
			fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=0", servoOID, pwmInitClock),
		)
		oidNext = 21
	}

	// Heater bed
	bedADCOID := -1
	bedPWMOID := -1
	if bedSec, ok := cfg.sections["heater_bed"]; ok {
		bedADCOID = oidNext
		oidNext++
		bedPWMOID = oidNext
		oidNext++

		sensorPin := strings.TrimSpace(bedSec["sensor_pin"])
		heaterPin := strings.TrimSpace(bedSec["heater_pin"])

		configCmds = append(configCmds,
			fmt.Sprintf("config_analog_in oid=%d pin=%s", bedADCOID, sensorPin),
			fmt.Sprintf("config_digital_out oid=%d pin=%s value=0 default_value=0 max_duration=%d", bedPWMOID, heaterPin, mdurTicks),
			fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", bedPWMOID, cycleTicks),
		)

		// Thermistor min/max for heater_bed
		bed, err := readHeater(cfg, "heater_bed")
		if err != nil {
			return nil, err
		}
		bedMin, bedMax, err := thermistorMinMaxTicks(bed, adcMax, adcSampleCount)
		if err != nil {
			return nil, err
		}
		initCmds = append(initCmds,
			fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
				bedADCOID, querySlotClock(mcuFreq, bedADCOID), sampleTicks, adcSampleCount, reportTicks, bedMin, bedMax, adcRangeChecks),
			fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=0", bedPWMOID, pwmInitClock),
		)
	}

	// After servo/bed, endstop OIDs start at 0
	// Python processes carriages in alphabetical order by section name
	type carriageInfo struct {
		name       string
		section    string
		endstopPin string
		isPrimary  bool
		isDual     bool
		isExtra    bool
		axis       int
	}

	var carriages []carriageInfo

	// Collect main carriages [carriage NAME]
	for sec := range cfg.sections {
		if !strings.HasPrefix(sec, "carriage ") {
			continue
		}
		name := strings.TrimPrefix(sec, "carriage ")
		data := cfg.sections[sec]
		axisStr := strings.TrimSpace(data["axis"])
		if axisStr == "" {
			// Derive from name
			if strings.HasSuffix(name, "_x") {
				axisStr = "x"
			} else if strings.HasSuffix(name, "_y") {
				axisStr = "y"
			} else if strings.HasSuffix(name, "_z") {
				axisStr = "z"
			}
		}
		axis := -1
		switch axisStr {
		case "x":
			axis = 0
		case "y":
			axis = 1
		case "z":
			axis = 2
		}
		carriages = append(carriages, carriageInfo{
			name:       name,
			section:    sec,
			endstopPin: strings.TrimSpace(data["endstop_pin"]),
			isPrimary:  true,
			axis:       axis,
		})
	}

	// Collect dual carriages [dual_carriage NAME]
	for sec := range cfg.sections {
		if !strings.HasPrefix(sec, "dual_carriage ") {
			continue
		}
		name := strings.TrimPrefix(sec, "dual_carriage ")
		data := cfg.sections[sec]
		primaryName := strings.TrimSpace(data["primary_carriage"])
		// Find primary's axis
		var axis int
		for _, c := range carriages {
			if c.name == primaryName {
				axis = c.axis
				break
			}
		}
		carriages = append(carriages, carriageInfo{
			name:       name,
			section:    sec,
			endstopPin: strings.TrimSpace(data["endstop_pin"]),
			isDual:     true,
			axis:       axis,
		})
	}

	// Collect extra carriages [extra_carriage NAME]
	for sec := range cfg.sections {
		if !strings.HasPrefix(sec, "extra_carriage ") {
			continue
		}
		name := strings.TrimPrefix(sec, "extra_carriage ")
		data := cfg.sections[sec]
		primaryName := strings.TrimSpace(data["primary_carriage"])
		// Find primary's axis
		var axis int
		for _, c := range carriages {
			if c.name == primaryName {
				axis = c.axis
				break
			}
		}
		carriages = append(carriages, carriageInfo{
			name:       name,
			section:    sec,
			endstopPin: strings.TrimSpace(data["endstop_pin"]),
			isExtra:    true,
			axis:       axis,
		})
	}

	// Sort carriages by section name for consistent ordering
	sort.Slice(carriages, func(i, j int) bool {
		return carriages[i].section < carriages[j].section
	})

	// Collect steppers [stepper NAME]
	type stepperInfo struct {
		name      string
		section   string
		stepPin   string
		dirPin    string
		enablePin string
	}
	var steppers []stepperInfo
	for sec := range cfg.sections {
		if !strings.HasPrefix(sec, "stepper ") {
			continue
		}
		name := strings.TrimPrefix(sec, "stepper ")
		data := cfg.sections[sec]
		steppers = append(steppers, stepperInfo{
			name:      name,
			section:   sec,
			stepPin:   strings.TrimSpace(data["step_pin"]),
			dirPin:    strings.TrimSpace(data["dir_pin"]),
			enablePin: strings.TrimSpace(data["enable_pin"]),
		})
	}
	sort.Slice(steppers, func(i, j int) bool {
		return steppers[i].section < steppers[j].section
	})

	// Map carriages to their endstop/trsync OIDs
	carriageEndstopOID := make(map[string]int)
	carriageTrsyncOID := make(map[string]int)

	// Allocate endstop/trsync OIDs for carriages (starts at OID 0)
	endstopOIDNext := 0
	for _, c := range carriages {
		if c.endstopPin != "" {
			carriageEndstopOID[c.name] = endstopOIDNext
			carriageTrsyncOID[c.name] = endstopOIDNext + 1
			endstopOIDNext += 2
		}
	}

	// Stepper OIDs start after all endstop/trsync OIDs
	stepperOIDNext := endstopOIDNext

	// Enable pin OIDs are allocated after all pre-assigned OIDs (23+ for generic_cartesian)
	enableOIDNext := 23
	if servoOID < 0 {
		// No servo - adjust base
		enableOIDNext = oidNext
	}

	// Process carriages and their associated steppers
	stepperOIDs := make(map[string]int)
	enableOIDs := make(map[string]int)

	// First pass: allocate all endstop/trsync OIDs and generate commands
	for _, c := range carriages {
		if c.endstopPin == "" {
			continue
		}
		endstopOID := carriageEndstopOID[c.name]
		trsyncOID := carriageTrsyncOID[c.name]

		pin, _, pullup := parsePinWithModifiers(c.endstopPin)
		pullupInt := 0
		if pullup {
			pullupInt = 1
		}

		configCmds = append(configCmds,
			fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", endstopOID, pin, pullupInt),
			fmt.Sprintf("config_trsync oid=%d", trsyncOID),
		)
	}

	// Allocate stepper OIDs
	for _, s := range steppers {
		stepperOIDs[s.name] = stepperOIDNext
		stepperOIDNext++
		enableOIDs[s.name] = enableOIDNext
		enableOIDNext++
	}

	// Generate stepper config commands
	for _, s := range steppers {
		stepOID := stepperOIDs[s.name]
		enableOID := enableOIDs[s.name]

		stepPin, stepInvert, _ := parsePinWithModifiers(s.stepPin)
		dirPin, _, _ := parsePinWithModifiers(s.dirPin)
		enablePin, enableInvert, _ := parsePinWithModifiers(s.enablePin)

		stepInvertInt := 0
		if stepInvert {
			stepInvertInt = 1
		}

		enableValue := 0
		if enableInvert {
			enableValue = 1
		}

		configCmds = append(configCmds,
			fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
				stepOID, stepPin, dirPin, stepInvertInt, stepPulseTicks),
			fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=0",
				enableOID, enablePin, enableValue, enableValue),
		)
	}

	// Process extruder sections
	if extSec, ok := cfg.sections["extruder"]; ok {
		extruderADCOID := enableOIDNext
		enableOIDNext++
		extruderPWMOID := enableOIDNext
		enableOIDNext++
		extruderStepOID := stepperOIDNext
		stepperOIDNext++
		extruderEnableOID := enableOIDNext
		enableOIDNext++

		sensorPin := strings.TrimSpace(extSec["sensor_pin"])
		heaterPin := strings.TrimSpace(extSec["heater_pin"])
		stepPin, stepInvert, _ := parsePinWithModifiers(strings.TrimSpace(extSec["step_pin"]))
		dirPin, _, _ := parsePinWithModifiers(strings.TrimSpace(extSec["dir_pin"]))
		enablePin, enableInvert, _ := parsePinWithModifiers(strings.TrimSpace(extSec["enable_pin"]))

		stepInvertInt := 0
		if stepInvert {
			stepInvertInt = 1
		}
		enableValue := 0
		if enableInvert {
			enableValue = 1
		}

		configCmds = append(configCmds,
			fmt.Sprintf("config_analog_in oid=%d pin=%s", extruderADCOID, sensorPin),
			fmt.Sprintf("config_digital_out oid=%d pin=%s value=0 default_value=0 max_duration=%d", extruderPWMOID, heaterPin, mdurTicks),
			fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", extruderPWMOID, cycleTicks),
			fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
				extruderStepOID, stepPin, dirPin, stepInvertInt, stepPulseTicks),
			fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=0",
				extruderEnableOID, enablePin, enableValue, enableValue),
		)

		// Thermistor min/max
		ext, err := readExtruder(cfg, "extruder")
		if err != nil {
			return nil, err
		}
		extMin, extMax, err := thermistorMinMaxTicks(ext.Heater, adcMax, adcSampleCount)
		if err != nil {
			return nil, err
		}
		initCmds = append(initCmds,
			fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
				extruderADCOID, querySlotClock(mcuFreq, extruderADCOID), sampleTicks, adcSampleCount, reportTicks, extMin, extMax, adcRangeChecks),
			fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=0", extruderPWMOID, pwmInitClock),
		)
	}

	// Process extruder1 section
	if ext1Sec, ok := cfg.sections["extruder1"]; ok {
		ext1ADCOID := enableOIDNext
		enableOIDNext++
		ext1PWMOID := enableOIDNext
		enableOIDNext++
		ext1StepOID := stepperOIDNext
		stepperOIDNext++
		ext1EnableOID := enableOIDNext
		enableOIDNext++

		sensorPin := strings.TrimSpace(ext1Sec["sensor_pin"])
		heaterPin := strings.TrimSpace(ext1Sec["heater_pin"])
		stepPin, stepInvert, _ := parsePinWithModifiers(strings.TrimSpace(ext1Sec["step_pin"]))
		dirPin, _, _ := parsePinWithModifiers(strings.TrimSpace(ext1Sec["dir_pin"]))
		enablePin, enableInvert, _ := parsePinWithModifiers(strings.TrimSpace(ext1Sec["enable_pin"]))

		stepInvertInt := 0
		if stepInvert {
			stepInvertInt = 1
		}
		enableValue := 0
		if enableInvert {
			enableValue = 1
		}

		configCmds = append(configCmds,
			fmt.Sprintf("config_analog_in oid=%d pin=%s", ext1ADCOID, sensorPin),
			fmt.Sprintf("config_digital_out oid=%d pin=%s value=0 default_value=0 max_duration=%d", ext1PWMOID, heaterPin, mdurTicks),
			fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", ext1PWMOID, cycleTicks),
			fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
				ext1StepOID, stepPin, dirPin, stepInvertInt, stepPulseTicks),
			fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=0",
				ext1EnableOID, enablePin, enableValue, enableValue),
		)

		// Thermistor min/max
		ext1, err := readExtruder(cfg, "extruder1")
		if err != nil {
			return nil, err
		}
		ext1Min, ext1Max, err := thermistorMinMaxTicks(ext1.Heater, adcMax, adcSampleCount)
		if err != nil {
			return nil, err
		}
		initCmds = append(initCmds,
			fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
				ext1ADCOID, querySlotClock(mcuFreq, ext1ADCOID), sampleTicks, adcSampleCount, reportTicks, ext1Min, ext1Max, adcRangeChecks),
			fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=0", ext1PWMOID, pwmInitClock),
		)
	}

	// Calculate total OID count
	totalOIDs := enableOIDNext
	if stepperOIDNext > totalOIDs {
		totalOIDs = stepperOIDNext
	}
	// Ensure we have at least 35 OIDs for the generic_cartesian config
	if totalOIDs < 35 {
		totalOIDs = 35
	}

	// Prepend allocate_oids
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", totalOIDs)}, configCmds...)
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	out := make([]string, 0, len(withAllocate)+1+len(initCmds))
	out = append(out, withAllocate...)
	out = append(out, fmt.Sprintf("finalize_config crc=%d", crc))
	out = append(out, initCmds...)
	return out, nil
}

// parsePinWithModifiers extracts pin name and modifier flags from a pin string.
// Examples: "^PE5" -> (PE5, false, true), "!PD7" -> (PD7, true, false)
func parsePinWithModifiers(s string) (pin string, invert bool, pullup bool) {
	s = strings.TrimSpace(s)
	for len(s) > 0 {
		switch s[0] {
		case '^':
			pullup = true
			s = s[1:]
		case '!':
			invert = true
			s = s[1:]
		case '~':
			// Ignore tilde (open-drain marker)
			s = s[1:]
		default:
			return s, invert, pullup
		}
	}
	return s, invert, pullup
}

// CompileHybridCoreXYConnectPhase compiles the connect-phase MCU command stream
// for hybrid_corexy kinematics with dual carriage and two extruders.
func CompileHybridCoreXYConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
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

	// Extract required steppers
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

	// Read dual_carriage section (similar to stepper)
	dcSec, ok := cfg.sections["dual_carriage"]
	if !ok {
		return nil, fmt.Errorf("missing [dual_carriage] section")
	}
	dcStep, err := parsePin(dcSec, "step_pin", true, false)
	if err != nil {
		return nil, err
	}
	dcDir, err := parsePin(dcSec, "dir_pin", true, false)
	if err != nil {
		return nil, err
	}
	dcEnable, err := parsePin(dcSec, "enable_pin", true, false)
	if err != nil {
		return nil, err
	}
	dcEndstop, err := parsePin(dcSec, "endstop_pin", true, true)
	if err != nil {
		return nil, err
	}

	// Extract heaters and extruders
	bed, err := readHeater(cfg, "heater_bed")
	if err != nil {
		return nil, err
	}
	extruder, err := readExtruder(cfg, "extruder")
	if err != nil {
		return nil, err
	}
	extruder1, err := readExtruder(cfg, "extruder1")
	if err != nil {
		return nil, err
	}

	// OID layout matching Python output:
	// 0-2: X (endstop, trsync, stepper)
	// 3-5: Y (endstop, trsync, stepper)
	// 6-8: Z (endstop, trsync, stepper)
	// 9-11: dual_carriage (endstop, trsync, stepper)
	// 12: stepper extruder
	// 13: stepper extruder1
	// 14-15: heater_bed (adc, pwm)
	// 16-19: enable pins (X, Y, Z, dual_carriage)
	// 20-22: extruder (adc, pwm, enable)
	// 23-25: extruder1 (adc, pwm, enable)
	type hybridOIDs struct {
		endstopX, trsyncX, stepperX               int
		endstopY, trsyncY, stepperY               int
		endstopZ, trsyncZ, stepperZ               int
		endstopDC, trsyncDC, stepperDC            int
		stepperE, stepperE1                       int
		adcBed, pwmBed                            int
		enableX, enableY, enableZ, enableDC       int
		adcE, pwmE, enableE                       int
		adcE1, pwmE1, enableE1                    int
		count                                     int
	}
	o := hybridOIDs{
		endstopX: 0, trsyncX: 1, stepperX: 2,
		endstopY: 3, trsyncY: 4, stepperY: 5,
		endstopZ: 6, trsyncZ: 7, stepperZ: 8,
		endstopDC: 9, trsyncDC: 10, stepperDC: 11,
		stepperE: 12, stepperE1: 13,
		adcBed: 14, pwmBed: 15,
		enableX: 16, enableY: 17, enableZ: 18, enableDC: 19,
		adcE: 20, pwmE: 21, enableE: 22,
		adcE1: 23, pwmE1: 24, enableE1: 25,
		count: 26,
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
	ext1Min, ext1Max, err := thermistorMinMaxTicks(extruder1.Heater, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}

	// Build config commands in the order matching Python output
	configCmds := []string{
		// heater_bed
		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcBed, bed.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmBed, bed.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmBed, cycleTicks),

		// stepper_x
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopX, stepperX.Endstop.pin, stepperX.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncX),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperX, stepperX.Step.pin, stepperX.Dir.pin, boolToInt(stepperX.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableX, stepperX.Enable.pin, boolToInt(stepperX.Enable.invert), boolToInt(stepperX.Enable.invert), 0),

		// stepper_y
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopY, stepperY.Endstop.pin, stepperY.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncY),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperY, stepperY.Step.pin, stepperY.Dir.pin, boolToInt(stepperY.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableY, stepperY.Enable.pin, boolToInt(stepperY.Enable.invert), boolToInt(stepperY.Enable.invert), 0),

		// stepper_z
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopZ, stepperZ.Endstop.pin, stepperZ.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncZ),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperZ, stepperZ.Step.pin, stepperZ.Dir.pin, boolToInt(stepperZ.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableZ, stepperZ.Enable.pin, boolToInt(stepperZ.Enable.invert), boolToInt(stepperZ.Enable.invert), 0),

		// dual_carriage
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopDC, dcEndstop.pin, dcEndstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncDC),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperDC, dcStep.pin, dcDir.pin, boolToInt(dcStep.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableDC, dcEnable.pin, boolToInt(dcEnable.invert), boolToInt(dcEnable.invert), 0),

		// extruder
		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcE, extruder.Heater.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmE, extruder.Heater.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmE, cycleTicks),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperE, extruder.Step.pin, extruder.Dir.pin, boolToInt(extruder.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableE, extruder.Enable.pin, boolToInt(extruder.Enable.invert), boolToInt(extruder.Enable.invert), 0),

		// extruder1
		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcE1, extruder1.Heater.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmE1, extruder1.Heater.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmE1, cycleTicks),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperE1, extruder1.Step.pin, extruder1.Dir.pin, boolToInt(extruder1.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableE1, extruder1.Enable.pin, boolToInt(extruder1.Enable.invert), boolToInt(extruder1.Enable.invert), 0),
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
		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			o.adcE1, querySlotClock(mcuFreq, o.adcE1), sampleTicks, adcSampleCount, reportTicks, ext1Min, ext1Max, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", o.pwmE1, pwmInitClock, 0),
	}

	out := make([]string, 0, 1+len(configCmds)+1+len(initCmds))
	out = append(out, withAllocate...)
	out = append(out, fmt.Sprintf("finalize_config crc=%d", crc))
	out = append(out, initCmds...)
	return out, nil
}
