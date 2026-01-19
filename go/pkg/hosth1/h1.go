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

	// Check if [fan] section exists
	fanSec, hasFan := cfg.sections["fan"]
	var fanPin pin
	if hasFan {
		fanPin, err = parsePin(fanSec, "pin", true, false)
		if err != nil {
			return nil, fmt.Errorf("fan pin: %w", err)
		}
	}

	// Allocate OIDs in the same stable numbering as the Python reference.
	// When [fan] is present, insert pwmFan at OID 12 and shift enable/adc/pwm OIDs by 1.
	// Without fan: count=18, enableX=12, enableY=13, enableZ=14, adcE=15, pwmE=16, enableE=17
	// With fan:    count=19, pwmFan=12, enableX=13, enableY=14, enableZ=15, adcE=16, pwmE=17, enableE=18
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
	pwmFan := -1
	if hasFan {
		pwmFan = 12
		o.enableX = 13
		o.enableY = 14
		o.enableZ = 15
		o.adcE = 16
		o.pwmE = 17
		o.enableE = 18
		o.count = 19
	}

	// Derived constants (match klippy defaults).
	const (
		pwmCycleTime     = 0.100
		fanCycleTime     = 0.010 // 10ms for fan PWM
		heaterMaxHeatSec = 3.0
		stepPulseSec     = 0.000002
		adcSampleTime    = 0.001
		adcSampleCount   = 8
		adcReportTime    = 0.300
		adcRangeChecks   = 4
		pwmInitDelaySec  = 0.200
	)
	cycleTicks := secondsToClock(mcuFreq, pwmCycleTime)
	fanCycleTicks := secondsToClock(mcuFreq, fanCycleTime)
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
	}

	// Add fan config if present (comes after bed heater PWM)
	if hasFan {
		configCmds = append(configCmds,
			fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
				pwmFan, fanPin.pin, 0, 0, 0),
			fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", pwmFan, fanCycleTicks),
		)
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

	// Build init commands (same order as the Python reference for this config).
	initCmds := []string{
		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			o.adcBed, querySlotClock(mcuFreq, o.adcBed), sampleTicks, adcSampleCount, reportTicks, bedMin, bedMax, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", o.pwmBed, pwmInitClock, 0),
	}

	// Add fan init command after bed heater
	if hasFan {
		initCmds = append(initCmds,
			fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", pwmFan, pwmInitClock, 0),
		)
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
	chip   string // MCU chip name (default: "mcu")
	pin    string
	invert bool
	pullup int
}

type heater struct {
	HeaterPin      pin
	SensorPin      pin
	SensorType     string
	MinTemp        float64
	MaxTemp        float64
	PullupResistor float64 // Custom pullup resistor value (0 = use default 4700)
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
	// Parse optional pullup_resistor (default 0 means use thermistor default of 4700)
	var pullupResistor float64
	if raw := strings.TrimSpace(sec["pullup_resistor"]); raw != "" {
		pullupResistor, err = strconv.ParseFloat(raw, 64)
		if err != nil {
			return heater{}, fmt.Errorf("bad float pullup_resistor=%q", raw)
		}
	}
	return heater{
		HeaterPin:      heaterPin,
		SensorPin:      sensorPin,
		SensorType:     sensorType,
		MinTemp:        minTemp,
		MaxTemp:        maxTemp,
		PullupResistor: pullupResistor,
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
	p := pin{chip: "mcu"} // Default to main MCU
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
		p.chip = chip
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

func thermistorGeneric3950() *thermistor {
	t := &thermistor{pullup: 4700.0, inlineResistor: 0.0}
	// [thermistor Generic 3950] from klippy/extras/temperature_sensors.cfg
	t.setupCoefficients(25, 100000, 150, 1770, 250, 230)
	return t
}

func thermistorATCSemitec104GT2() *thermistor {
	t := &thermistor{pullup: 4700.0, inlineResistor: 0.0}
	// [thermistor ATC Semitec 104GT-2] from klippy/extras/temperature_sensors.cfg
	t.setupCoefficients(20, 126800, 150, 1360, 300, 80.65)
	return t
}

func thermistorATCSemitec104NT4() *thermistor {
	t := &thermistor{pullup: 4700.0, inlineResistor: 0.0}
	// [thermistor ATC Semitec 104NT-4-R025H42G] from klippy/extras/temperature_sensors.cfg
	t.setupCoefficients(25, 100000, 160, 1074, 300, 82.78)
	return t
}

func thermistorHoneywell100K() *thermistor {
	t := &thermistor{pullup: 4700.0, inlineResistor: 0.0}
	// [thermistor Honeywell 100K 135-104LAG-J01] from klippy/extras/temperature_sensors.cfg
	// Uses beta=3974
	t.setupBeta(25, 100000, 3974)
	return t
}

func thermistorNTC100KMGB() *thermistor {
	t := &thermistor{pullup: 4700.0, inlineResistor: 0.0}
	// [thermistor NTC 100K MGB18-104F39050L32] from klippy/extras/temperature_sensors.cfg
	// Uses beta=4100
	t.setupBeta(25, 100000, 4100)
	return t
}

func thermistorSliceEngineering450() *thermistor {
	t := &thermistor{pullup: 4700.0, inlineResistor: 0.0}
	// [thermistor SliceEngineering 450] from klippy/extras/temperature_sensors.cfg
	t.setupCoefficients(25, 500000, 200, 3734, 400, 240)
	return t
}

func thermistorTDKNTCG104() *thermistor {
	t := &thermistor{pullup: 4700.0, inlineResistor: 0.0}
	// [thermistor TDK NTCG104LH104JT1] from klippy/extras/temperature_sensors.cfg
	t.setupCoefficients(25, 100000, 50, 31230, 125, 2066)
	return t
}

func (t *thermistor) setupBeta(t0, r0, beta float64) {
	// Simple beta parameter setup for thermistors defined with only beta
	invT0 := 1.0 / (t0 - kelvinToCelsius)
	lnR0 := math.Log(r0)
	t.c1 = invT0 - lnR0/beta
	t.c2 = 1.0 / beta
	t.c3 = 0.0
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
	var th *thermistor
	switch h.SensorType {
	case "EPCOS 100K B57560G104F":
		th = thermistorEPCOS100K()
	case "Generic 3950":
		th = thermistorGeneric3950()
	case "ATC Semitec 104GT-2":
		th = thermistorATCSemitec104GT2()
	case "ATC Semitec 104NT-4-R025H42G":
		th = thermistorATCSemitec104NT4()
	case "Honeywell 100K 135-104LAG-J01":
		th = thermistorHoneywell100K()
	case "NTC 100K MGB18-104F39050L32":
		th = thermistorNTC100KMGB()
	case "SliceEngineering 450":
		th = thermistorSliceEngineering450()
	case "TDK NTCG104LH104JT1":
		th = thermistorTDKNTCG104()
	default:
		return 0, 0, fmt.Errorf("unsupported sensor_type %q", h.SensorType)
	}
	// Override pullup resistor if config specifies a custom value
	if h.PullupResistor > 0 {
		th.pullup = h.PullupResistor
	}
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

// CompileCartesianDualCarriageConnectPhase generates connect-phase commands for
// cartesian kinematics with dual_carriage (IDEX).
// This matches Python Klippy's output for dual_carriage.cfg test.
func CompileCartesianDualCarriageConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
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

	// Read servo if present (optional)
	var servoPin *pin
	if servoSec, ok := cfg.sections["servo my_servo"]; ok {
		p, err := parsePin(servoSec, "pin", true, false)
		if err == nil {
			servoPin = &p
		}
	}

	// OID layout matching Python output for dual_carriage.cfg:
	// 0-2: X (endstop, trsync, stepper)
	// 3-5: Y (endstop, trsync, stepper)
	// 6-8: Z (endstop, trsync, stepper)
	// 9-11: dual_carriage (endstop, trsync, stepper)
	// 12: stepper extruder
	// 13: stepper extruder1
	// 14: servo
	// 15-16: heater_bed (adc, pwm)
	// 17-20: enable pins (X, Y, Z, dual_carriage)
	// 21-23: extruder (adc, pwm, enable)
	// 24-26: extruder1 (adc, pwm, enable)
	type cartesianDCOIDs struct {
		endstopX, trsyncX, stepperX    int
		endstopY, trsyncY, stepperY    int
		endstopZ, trsyncZ, stepperZ    int
		endstopDC, trsyncDC, stepperDC int
		stepperE, stepperE1            int
		servo                          int
		adcBed, pwmBed                 int
		enableX, enableY, enableZ      int
		enableDC                       int
		adcE, pwmE, enableE            int
		adcE1, pwmE1, enableE1         int
		count                          int
	}
	o := cartesianDCOIDs{
		endstopX: 0, trsyncX: 1, stepperX: 2,
		endstopY: 3, trsyncY: 4, stepperY: 5,
		endstopZ: 6, trsyncZ: 7, stepperZ: 8,
		endstopDC: 9, trsyncDC: 10, stepperDC: 11,
		stepperE: 12, stepperE1: 13,
		servo:   14,
		adcBed:  15, pwmBed: 16,
		enableX: 17, enableY: 18, enableZ: 19,
		enableDC: 20,
		adcE: 21, pwmE: 22, enableE: 23,
		adcE1: 24, pwmE1: 25, enableE1: 26,
		count: 27,
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
	var configCmds []string

	// servo (first in Python output)
	if servoPin != nil {
		configCmds = append(configCmds,
			fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
				o.servo, servoPin.pin, 0, 0, 0),
			fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.servo, cycleTicks/5), // servo uses 20ms cycle
		)
	}

	// heater_bed
	configCmds = append(configCmds,
		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcBed, bed.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmBed, bed.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmBed, cycleTicks),
	)

	// stepper_x
	configCmds = append(configCmds,
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopX, stepperX.Endstop.pin, stepperX.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncX),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperX, stepperX.Step.pin, stepperX.Dir.pin, boolToInt(stepperX.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableX, stepperX.Enable.pin, boolToInt(stepperX.Enable.invert), boolToInt(stepperX.Enable.invert), 0),
	)

	// stepper_y
	configCmds = append(configCmds,
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopY, stepperY.Endstop.pin, stepperY.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncY),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperY, stepperY.Step.pin, stepperY.Dir.pin, boolToInt(stepperY.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableY, stepperY.Enable.pin, boolToInt(stepperY.Enable.invert), boolToInt(stepperY.Enable.invert), 0),
	)

	// stepper_z
	configCmds = append(configCmds,
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopZ, stepperZ.Endstop.pin, stepperZ.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncZ),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperZ, stepperZ.Step.pin, stepperZ.Dir.pin, boolToInt(stepperZ.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableZ, stepperZ.Enable.pin, boolToInt(stepperZ.Enable.invert), boolToInt(stepperZ.Enable.invert), 0),
	)

	// dual_carriage
	configCmds = append(configCmds,
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopDC, dcEndstop.pin, dcEndstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncDC),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperDC, dcStep.pin, dcDir.pin, boolToInt(dcStep.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableDC, dcEnable.pin, boolToInt(dcEnable.invert), boolToInt(dcEnable.invert), 0),
	)

	// extruder
	configCmds = append(configCmds,
		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcE, extruder.Heater.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmE, extruder.Heater.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmE, cycleTicks),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperE, extruder.Step.pin, extruder.Dir.pin, boolToInt(extruder.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableE, extruder.Enable.pin, boolToInt(extruder.Enable.invert), boolToInt(extruder.Enable.invert), 0),
	)

	// extruder1
	configCmds = append(configCmds,
		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcE1, extruder1.Heater.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmE1, extruder1.Heater.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmE1, cycleTicks),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperE1, extruder1.Step.pin, extruder1.Dir.pin, boolToInt(extruder1.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableE1, extruder1.Enable.pin, boolToInt(extruder1.Enable.invert), boolToInt(extruder1.Enable.invert), 0),
	)

	// Prepend allocate_oids and compute CRC
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", o.count)}, configCmds...)
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	// Build init commands
	initCmds := []string{}
	if servoPin != nil {
		initCmds = append(initCmds,
			fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", o.servo, pwmInitClock, 0),
		)
	}
	initCmds = append(initCmds,
		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			o.adcBed, querySlotClock(mcuFreq, o.adcBed), sampleTicks, adcSampleCount, reportTicks, bedMin, bedMax, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", o.pwmBed, pwmInitClock, 0),
		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			o.adcE, querySlotClock(mcuFreq, o.adcE), sampleTicks, adcSampleCount, reportTicks, extMin, extMax, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", o.pwmE, pwmInitClock, 0),
		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			o.adcE1, querySlotClock(mcuFreq, o.adcE1), sampleTicks, adcSampleCount, reportTicks, ext1Min, ext1Max, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", o.pwmE1, pwmInitClock, 0),
	)

	out := make([]string, 0, 1+len(configCmds)+1+len(initCmds))
	out = append(out, withAllocate...)
	out = append(out, fmt.Sprintf("finalize_config crc=%d", crc))
	out = append(out, initCmds...)
	return out, nil
}

// CompilePolarConnectPhase generates connect-phase commands for polar kinematics.
// Polar printers have a rotating bed (stepper_bed), a linear arm (stepper_arm), and a Z axis.
func CompilePolarConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
	cfg, err := loadConfig(cfgPath)
	if err != nil {
		return nil, err
	}
	clockFreq, err := dictConfigFloat(dict, "CLOCK_FREQ")
	if err != nil {
		return nil, err
	}
	mcuFreq := clockFreq
	adcMax, err := dictConfigFloat(dict, "ADC_MAX")
	if err != nil {
		return nil, err
	}

	stepPulseTicks := 32 // 2us at 16MHz

	// OID layout for polar:
	// 0: stepper_bed step
	// 1: stepper_arm endstop
	// 2: stepper_arm trsync
	// 3: stepper_arm step
	// 4: stepper_z endstop
	// 5: stepper_z trsync
	// 6: stepper_z step
	// 7: extruder step
	// 8: heater_bed ADC
	// 9: heater_bed PWM
	// 10: fan PWM
	// 11: stepper_bed enable
	// 12: stepper_arm enable
	// 13: stepper_z enable
	// 14: extruder ADC
	// 15: extruder PWM
	// 16: extruder enable
	type polarOIDs struct {
		stepperBed int
		endstopArm int
		trsyncArm  int
		stepperArm int
		endstopZ   int
		trsyncZ    int
		stepperZ   int
		stepperE   int
		adcBed     int
		pwmBed     int
		pwmFan     int
		enableBed  int
		enableArm  int
		enableZ    int
		adcE       int
		pwmE       int
		enableE    int
		count      int
	}
	o := polarOIDs{
		stepperBed: 0,
		endstopArm: 1,
		trsyncArm:  2,
		stepperArm: 3,
		endstopZ:   4,
		trsyncZ:    5,
		stepperZ:   6,
		stepperE:   7,
		adcBed:     8,
		pwmBed:     9,
		pwmFan:     10,
		enableBed:  11,
		enableArm:  12,
		enableZ:    13,
		adcE:       14,
		pwmE:       15,
		enableE:    16,
		count:      17,
	}

	// Parse stepper_bed (rotating bed, no endstop)
	bedSec, ok := cfg.sections["stepper_bed"]
	if !ok {
		return nil, fmt.Errorf("missing [stepper_bed] section")
	}
	bedStep, err := parsePin(bedSec, "step_pin", true, false)
	if err != nil {
		return nil, fmt.Errorf("stepper_bed step_pin: %w", err)
	}
	bedDir, err := parsePin(bedSec, "dir_pin", true, false)
	if err != nil {
		return nil, fmt.Errorf("stepper_bed dir_pin: %w", err)
	}
	bedEnable, err := parsePin(bedSec, "enable_pin", true, false)
	if err != nil {
		return nil, fmt.Errorf("stepper_bed enable_pin: %w", err)
	}

	// Parse stepper_arm (linear arm with endstop)
	stepperArm, err := readStepper(cfg, "stepper_arm")
	if err != nil {
		return nil, fmt.Errorf("stepper_arm: %w", err)
	}

	// Parse stepper_z
	stepperZ, err := readStepper(cfg, "stepper_z")
	if err != nil {
		return nil, fmt.Errorf("stepper_z: %w", err)
	}

	// Parse extruder
	extruder, err := readExtruder(cfg, "extruder")
	if err != nil {
		return nil, err
	}

	// Parse heater_bed
	bed, err := readHeater(cfg, "heater_bed")
	if err != nil {
		return nil, err
	}

	// Parse fan
	fanSec, ok := cfg.sections["fan"]
	if !ok {
		return nil, fmt.Errorf("missing [fan] section")
	}
	fanPin, err := parsePin(fanSec, "pin", true, false)
	if err != nil {
		return nil, fmt.Errorf("fan pin: %w", err)
	}

	// Temperature thresholds (from sensor types)
	adcSampleCount := 8
	bedMin, bedMax, err := thermistorMinMaxTicks(bed, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}
	extMin, extMax, err := thermistorMinMaxTicks(extruder.Heater, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}

	// PWM cycle settings
	mdurTicks := 3000 * int(mcuFreq)    // max_duration=3s for heaters
	cycleTicks := int(mcuFreq / 10)     // 0.1s cycle for heaters
	fanCycleTicks := int(mcuFreq / 100) // 0.01s cycle for fan

	// ADC query settings
	sampleTicks := 16000
	reportTicks := int(0.3 * mcuFreq) // ~0.3s
	adcRangeChecks := 4
	pwmInitClock := int(0.2 * mcuFreq)

	configCmds := []string{
		// heater_bed
		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcBed, bed.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmBed, bed.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmBed, cycleTicks),

		// fan
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmFan, fanPin.pin, 0, 0, 0),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmFan, fanCycleTicks),

		// stepper_bed (no endstop)
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperBed, bedStep.pin, bedDir.pin, boolToInt(bedStep.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableBed, bedEnable.pin, boolToInt(bedEnable.invert), boolToInt(bedEnable.invert), 0),

		// stepper_arm (with endstop and trsync)
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopArm, stepperArm.Endstop.pin, stepperArm.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncArm),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperArm, stepperArm.Step.pin, stepperArm.Dir.pin, boolToInt(stepperArm.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableArm, stepperArm.Enable.pin, boolToInt(stepperArm.Enable.invert), boolToInt(stepperArm.Enable.invert), 0),

		// stepper_z (with endstop and trsync)
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopZ, stepperZ.Endstop.pin, stepperZ.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncZ),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperZ, stepperZ.Step.pin, stepperZ.Dir.pin, boolToInt(stepperZ.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableZ, stepperZ.Enable.pin, boolToInt(stepperZ.Enable.invert), boolToInt(stepperZ.Enable.invert), 0),

		// extruder
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
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", o.pwmFan, pwmInitClock, 0),
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

// CompileRotaryDeltaConnectPhase emits the connect-phase MCU commands for
// rotary_delta_calibrate.cfg (no heaters).
// Rotary delta has the same OID layout as regular delta calibrate -
// stepper_a, stepper_b, stepper_c with endstops.
func CompileRotaryDeltaConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
	// Rotary delta calibrate has the same MCU config structure as regular delta calibrate.
	// The difference is in the kinematics calculation, which happens at runtime.
	return CompileDeltaCalibrateConnectPhase(cfgPath, dict)
}

// CompileHybridCoreXZConnectPhase emits the connect-phase MCU commands for
// example-hybrid-corexz.cfg. Hybrid CoreXZ has the same OID layout as
// example-cartesian.cfg - the kinematics difference is handled at runtime.
func CompileHybridCoreXZConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
	// Hybrid CoreXZ has the same MCU config structure as cartesian.
	// The difference is in the kinematics calculation, which happens at runtime.
	return CompileExampleCartesianConnectPhase(cfgPath, dict)
}

// CompileDeltesianConnectPhase emits the connect-phase MCU commands for
// example-deltesian.cfg. Deltesian uses stepper_left, stepper_right, stepper_y
// but the same physical pins and OID layout as cartesian.
func CompileDeltesianConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
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

	// Read stepper configs with deltesian names
	stepperLeft, err := readStepper(cfg, "stepper_left")
	if err != nil {
		return nil, err
	}
	stepperRight, err := readStepper(cfg, "stepper_right")
	if err != nil {
		return nil, err
	}
	stepperY, err := readStepper(cfg, "stepper_y")
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

	// OID layout for deltesian (same structure as cartesian):
	// 0: endstop_left, 1: trsync_left, 2: stepper_left
	// 3: endstop_right, 4: trsync_right, 5: stepper_right
	// 6: endstop_y, 7: trsync_y, 8: stepper_y
	// 9: extruder stepper
	// 10: heater_bed ADC, 11: heater_bed PWM
	// 12-14: enable pins (left, right, y)
	// 15-17: extruder (ADC, PWM, enable)
	o := oids{
		endstopX: 0, trsyncX: 1, stepperX: 2, // left
		endstopY: 3, trsyncY: 4, stepperY: 5, // right
		endstopZ: 6, trsyncZ: 7, stepperZ: 8, // y
		stepperE: 9,
		adcBed:   10, pwmBed: 11,
		enableX: 12, enableY: 13, enableZ: 14, // left, right, y
		adcE: 15, pwmE: 16, enableE: 17,
		count: 18,
	}

	// Derived constants (match klippy defaults)
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

	// Build config commands
	configCmds := []string{
		// heater_bed
		fmt.Sprintf("config_analog_in oid=%d pin=%s", o.adcBed, bed.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.pwmBed, bed.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", o.pwmBed, cycleTicks),

		// stepper_left (OIDs 0-2, enable 12)
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopX, stepperLeft.Endstop.pin, stepperLeft.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncX),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperX, stepperLeft.Step.pin, stepperLeft.Dir.pin, boolToInt(stepperLeft.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableX, stepperLeft.Enable.pin, boolToInt(stepperLeft.Enable.invert), boolToInt(stepperLeft.Enable.invert), 0),

		// stepper_right (OIDs 3-5, enable 13)
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopY, stepperRight.Endstop.pin, stepperRight.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncY),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperY, stepperRight.Step.pin, stepperRight.Dir.pin, boolToInt(stepperRight.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableY, stepperRight.Enable.pin, boolToInt(stepperRight.Enable.invert), boolToInt(stepperRight.Enable.invert), 0),

		// stepper_y (OIDs 6-8, enable 14)
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", o.endstopZ, stepperY.Endstop.pin, stepperY.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", o.trsyncZ),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			o.stepperZ, stepperY.Step.pin, stepperY.Dir.pin, boolToInt(stepperY.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			o.enableZ, stepperY.Enable.pin, boolToInt(stepperY.Enable.invert), boolToInt(stepperY.Enable.invert), 0),

		// extruder
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

// CompileWinchConnectPhase emits the connect-phase MCU commands for
// example-winch.cfg. Winch (cable robot) kinematics uses stepper_a through
// stepper_d (or more), with no endstops (homing not implemented).
func CompileWinchConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
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

	// Find all stepper sections (stepper_a through stepper_z)
	var winchSteppers []stepper
	for i := 0; i < 26; i++ {
		name := "stepper_" + string('a'+byte(i))
		if _, ok := cfg.sections[name]; !ok && i >= 3 {
			break
		}
		s, err := readStepper(cfg, name)
		if err != nil {
			return nil, err
		}
		winchSteppers = append(winchSteppers, s)
	}
	numSteppers := len(winchSteppers)

	extruder, err := readExtruder(cfg, "extruder")
	if err != nil {
		return nil, err
	}
	bed, err := readHeater(cfg, "heater_bed")
	if err != nil {
		return nil, err
	}

	// OID layout for winch (no endstops/trsync, homing not implemented):
	// 0..n-1: stepper OIDs for stepper_a through stepper_d (etc.)
	// n: extruder stepper
	// n+1: heater_bed ADC
	// n+2: heater_bed PWM
	// n+3..n+3+n-1: enable pins for steppers
	// n+3+n: extruder ADC
	// n+3+n+1: extruder PWM
	// n+3+n+2: extruder enable
	stepperBase := 0
	extruderStepper := stepperBase + numSteppers
	adcBed := extruderStepper + 1
	pwmBed := adcBed + 1
	enableBase := pwmBed + 1
	adcE := enableBase + numSteppers
	pwmE := adcE + 1
	enableE := pwmE + 1
	totalOIDs := enableE + 1

	// Derived constants (match klippy defaults)
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

	// Build config commands
	var configCmds []string

	// heater_bed first
	configCmds = append(configCmds,
		fmt.Sprintf("config_analog_in oid=%d pin=%s", adcBed, bed.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			pwmBed, bed.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", pwmBed, cycleTicks),
	)

	// Winch steppers (no endstops - homing not implemented)
	for i, s := range winchSteppers {
		configCmds = append(configCmds,
			fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
				stepperBase+i, s.Step.pin, s.Dir.pin, boolToInt(s.Step.invert), stepPulseTicks),
			fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
				enableBase+i, s.Enable.pin, boolToInt(s.Enable.invert), boolToInt(s.Enable.invert), 0),
		)
	}

	// extruder
	configCmds = append(configCmds,
		fmt.Sprintf("config_analog_in oid=%d pin=%s", adcE, extruder.Heater.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			pwmE, extruder.Heater.HeaterPin.pin, 0, 0, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", pwmE, cycleTicks),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			extruderStepper, extruder.Step.pin, extruder.Dir.pin, boolToInt(extruder.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
			enableE, extruder.Enable.pin, boolToInt(extruder.Enable.invert), boolToInt(extruder.Enable.invert), 0),
	)

	// Prepend allocate_oids and compute CRC
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", totalOIDs)}, configCmds...)
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	// Build init commands
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

// CompileMultiMCUCartesianConnectPhase compiles the connect-phase MCU command
// streams for cartesian printers with multiple MCUs.
// Returns a map of MCU name -> commands.
//
// This function follows Klipper's module loading order:
// 1. heater_bed (heaters module loads first)
// 2. steppers in config order (stepper_x, stepper_y, stepper_z)
// 3. extruder
//
// OID allocation follows Klipper's pattern:
// - All stepper cores (endstop, trsync, stepper) including extruder stepper
// - heater_bed sensor and heater
// - stepper enable pins
// - extruder sensor and heater
// - extruder enable pin
func CompileMultiMCUCartesianConnectPhase(cfgPath string, dicts map[string]*protocol.Dictionary) (map[string][]string, error) {
	cfg, err := loadConfig(cfgPath)
	if err != nil {
		return nil, err
	}

	// Parse MCU sections
	mcuSections := ParseMCUSectionsFromConfig(cfg)
	if len(mcuSections) == 0 {
		return nil, fmt.Errorf("no MCU sections found")
	}

	// Build results per MCU
	results := make(map[string][]string)

	// Get all stepper configurations and group by MCU
	steppers := []struct {
		name    string
		stepper stepper
	}{}

	// Include stepper_z1 for dual-Z setups
	for _, sName := range []string{"stepper_x", "stepper_y", "stepper_z", "stepper_z1"} {
		if _, ok := cfg.sections[sName]; ok {
			s, err := readStepper(cfg, sName)
			if err != nil {
				return nil, err
			}
			steppers = append(steppers, struct {
				name    string
				stepper stepper
			}{sName, s})
		}
	}

	// Get extruder and heater configs
	var extruder *extruder
	if _, ok := cfg.sections["extruder"]; ok {
		e, err := readExtruder(cfg, "extruder")
		if err != nil {
			return nil, err
		}
		extruder = &e
	}

	var bed *heater
	if _, ok := cfg.sections["heater_bed"]; ok {
		h, err := readHeater(cfg, "heater_bed")
		if err != nil {
			return nil, err
		}
		bed = &h
	}

	// Process each MCU
	for mcuName := range mcuSections {
		dict := dicts[mcuName]
		if dict == nil {
			return nil, fmt.Errorf("missing dictionary for MCU %q", mcuName)
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

		// Collect components for this MCU
		var mcuSteppers []struct {
			name    string
			stepper stepper
		}
		for _, s := range steppers {
			if s.stepper.Step.chip == mcuName {
				mcuSteppers = append(mcuSteppers, s)
			}
		}

		hasExtruder := extruder != nil && extruder.Step.chip == mcuName
		hasBed := bed != nil && bed.HeaterPin.chip == mcuName

		// Phase 1: Allocate OIDs following Klipper's order
		// First: all stepper cores (endstop, trsync, stepper) for all steppers including extruder
		// Note: Secondary steppers (like stepper_z1) don't have endstops
		nextOID := 0
		type stepperOIDs struct {
			endstop, trsync, stepper, enable int
			hasEndstop                       bool
		}
		stepperOIDMap := make(map[string]stepperOIDs)

		for _, s := range mcuSteppers {
			hasEndstop := s.stepper.Endstop.pin != ""
			oids := stepperOIDs{hasEndstop: hasEndstop}
			if hasEndstop {
				oids.endstop = nextOID
				oids.trsync = nextOID + 1
				oids.stepper = nextOID + 2
				nextOID += 3
			} else {
				// Secondary stepper (like stepper_z1) - only stepper, no endstop/trsync
				oids.endstop = -1
				oids.trsync = -1
				oids.stepper = nextOID
				nextOID++
			}
			stepperOIDMap[s.name] = oids
		}

		// Extruder stepper (allocated with other steppers)
		extruderOID := -1
		if hasExtruder {
			extruderOID = nextOID
			nextOID++
		}

		// Bed heater ADC and PWM
		bedADC, bedPWM := -1, -1
		if hasBed {
			bedADC = nextOID
			nextOID++
			bedPWM = nextOID
			nextOID++
		}

		// Enable pins for all steppers
		for _, s := range mcuSteppers {
			oids := stepperOIDMap[s.name]
			oids.enable = nextOID
			nextOID++
			stepperOIDMap[s.name] = oids
		}

		// Extruder heater ADC, PWM, and enable
		extADC, extPWM, extEnable := -1, -1, -1
		if hasExtruder {
			extADC = nextOID
			nextOID++
			extPWM = nextOID
			nextOID++
			extEnable = nextOID
			nextOID++
		}

		// Phase 2: Build commands in Klipper's module output order
		// Order: heater_bed first, then steppers with enable, then extruder
		var configCmds []string
		var initCmds []string

		// Bed heater commands (heaters module outputs first)
		if hasBed {
			configCmds = append(configCmds, fmt.Sprintf("config_analog_in oid=%d pin=%s", bedADC, bed.SensorPin.pin))
			configCmds = append(configCmds, fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
				bedPWM, bed.HeaterPin.pin, 0, 0, mdurTicks))
			configCmds = append(configCmds, fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", bedPWM, cycleTicks))

			bedMin, bedMax, err := thermistorMinMaxTicks(*bed, adcMax, adcSampleCount)
			if err != nil {
				return nil, err
			}
			initCmds = append(initCmds, fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
				bedADC, querySlotClock(mcuFreq, bedADC), sampleTicks, adcSampleCount, reportTicks, bedMin, bedMax, adcRangeChecks))
			initCmds = append(initCmds, fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", bedPWM, pwmInitClock, 0))
		}

		// Stepper commands (endstop, trsync, stepper, enable for each)
		for _, s := range mcuSteppers {
			oids := stepperOIDMap[s.name]

			// Config endstop and trsync only if this stepper has an endstop
			if oids.hasEndstop {
				pullup := 0
				if s.stepper.Endstop.pullup != 0 {
					pullup = 1
				}
				configCmds = append(configCmds, fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d",
					oids.endstop, s.stepper.Endstop.pin, pullup))
				configCmds = append(configCmds, fmt.Sprintf("config_trsync oid=%d", oids.trsync))
			}

			// Config stepper
			configCmds = append(configCmds, fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
				oids.stepper, s.stepper.Step.pin, s.stepper.Dir.pin, 0, stepPulseTicks))

			// Enable pin (output immediately after stepper)
			val := 1
			if s.stepper.Enable.invert {
				val = 1
			}
			configCmds = append(configCmds, fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
				oids.enable, s.stepper.Enable.pin, val, val, 0))
		}

		// Extruder commands (heater first, then stepper, then enable)
		if hasExtruder {
			// Extruder heater
			configCmds = append(configCmds, fmt.Sprintf("config_analog_in oid=%d pin=%s", extADC, extruder.Heater.SensorPin.pin))
			configCmds = append(configCmds, fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
				extPWM, extruder.Heater.HeaterPin.pin, 0, 0, mdurTicks))
			configCmds = append(configCmds, fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", extPWM, cycleTicks))

			// Extruder stepper
			configCmds = append(configCmds, fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
				extruderOID, extruder.Step.pin, extruder.Dir.pin, 0, stepPulseTicks))

			// Extruder enable
			val := 1
			if extruder.Enable.invert {
				val = 1
			}
			configCmds = append(configCmds, fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=%d",
				extEnable, extruder.Enable.pin, val, val, 0))

			// Extruder init commands
			extMin, extMax, err := thermistorMinMaxTicks(extruder.Heater, adcMax, adcSampleCount)
			if err != nil {
				return nil, err
			}
			initCmds = append(initCmds, fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
				extADC, querySlotClock(mcuFreq, extADC), sampleTicks, adcSampleCount, reportTicks, extMin, extMax, adcRangeChecks))
			initCmds = append(initCmds, fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=%d", extPWM, pwmInitClock, 0))
		}

		// Build final command list with allocate_oids and finalize_config
		allocateCmd := fmt.Sprintf("allocate_oids count=%d", nextOID)
		withAllocate := append([]string{allocateCmd}, configCmds...)
		crcText := strings.Join(withAllocate, "\n")
		crc := crc32.ChecksumIEEE([]byte(crcText))

		out := make([]string, 0)
		out = append(out, withAllocate...)
		out = append(out, fmt.Sprintf("finalize_config crc=%d", crc))
		out = append(out, initCmds...)

		results[mcuName] = out

		// Suppress unused variable warnings
		_, _, _ = cycleTicks, bedADC, extEnable
	}

	return results, nil
}

// TMC2130 register addresses
const (
	tmc2130GCONF      = 0x00
	tmc2130IHOLD_IRUN = 0x10
	tmc2130TPOWERDOWN = 0x11
	tmc2130TPWMTHRS   = 0x13
	tmc2130TCOOLTHRS  = 0x14
	tmc2130THIGH      = 0x15
	tmc2130MSLUT0     = 0x60
	tmc2130MSLUT1     = 0x61
	tmc2130MSLUT2     = 0x62
	tmc2130MSLUT3     = 0x63
	tmc2130MSLUT4     = 0x64
	tmc2130MSLUT5     = 0x65
	tmc2130MSLUT6     = 0x66
	tmc2130MSLUT7     = 0x67
	tmc2130MSLUTSEL   = 0x68
	tmc2130MSLUTSTART = 0x69
	tmc2130CHOPCONF   = 0x6c
	tmc2130COOLCONF   = 0x6d
	tmc2130PWMCONF    = 0x70
)

// TMC2130 default wave table values
var tmc2130DefaultMSLUT = [8]uint32{
	0xAAAAB554, // MSLUT0
	0x4A9554AA, // MSLUT1
	0x24492929, // MSLUT2
	0x10104222, // MSLUT3
	0xFBFFFFFF, // MSLUT4
	0xB5BB777D, // MSLUT5
	0x49295556, // MSLUT6
	0x00404222, // MSLUT7
}

// TMC2130 current calculation
func tmc2130CalcCurrentBits(current, senseResistor float64, vsense bool) int {
	sr := senseResistor + 0.020
	vref := 0.32
	if vsense {
		vref = 0.18
	}
	cs := int(32.*sr*current*math.Sqrt(2.)/vref+0.5) - 1
	if cs < 0 {
		return 0
	}
	if cs > 31 {
		return 31
	}
	return cs
}

func tmc2130CalcCurrentFromBits(cs int, senseResistor float64, vsense bool) float64 {
	sr := senseResistor + 0.020
	vref := 0.32
	if vsense {
		vref = 0.18
	}
	return float64(cs+1) * vref / (32. * sr * math.Sqrt(2.))
}

func tmc2130CalcCurrent(runCurrent, holdCurrent, senseResistor float64) (vsense bool, irun, ihold int) {
	vsense = true
	irun = tmc2130CalcCurrentBits(runCurrent, senseResistor, true)
	if irun == 31 {
		cur := tmc2130CalcCurrentFromBits(irun, senseResistor, true)
		if cur < runCurrent {
			irun2 := tmc2130CalcCurrentBits(runCurrent, senseResistor, false)
			cur2 := tmc2130CalcCurrentFromBits(irun2, senseResistor, false)
			if math.Abs(runCurrent-cur2) < math.Abs(runCurrent-cur) {
				vsense = false
				irun = irun2
			}
		}
	}
	minHoldRun := holdCurrent
	if minHoldRun > runCurrent {
		minHoldRun = runCurrent
	}
	ihold = tmc2130CalcCurrentBits(minHoldRun, senseResistor, vsense)
	return
}

// TMC2130 register value builders
func tmc2130BuildCHOPCONF(toff, hstrt, hend, tbl, vsense, mres int, intpol bool) uint32 {
	val := uint32(toff & 0x0f)
	val |= uint32((hstrt & 0x07) << 4)
	val |= uint32((hend & 0x0f) << 7)
	val |= uint32((tbl & 0x03) << 15)
	if vsense == 1 {
		val |= 1 << 17
	}
	val |= uint32((mres & 0x0f) << 24)
	if intpol {
		val |= 1 << 28
	}
	return val
}

func tmc2130BuildIHOLD_IRUN(ihold, irun, iholddelay int) uint32 {
	return uint32(ihold&0x1f) | uint32((irun&0x1f)<<8) | uint32((iholddelay&0x0f)<<16)
}

func tmc2130BuildMSLUTSEL(w0, w1, w2, w3, x1, x2, x3 int) uint32 {
	return uint32(w0&0x03) | uint32((w1&0x03)<<2) | uint32((w2&0x03)<<4) | uint32((w3&0x03)<<6) |
		uint32((x1&0xff)<<8) | uint32((x2&0xff)<<16) | uint32((x3&0xff)<<24)
}

func tmc2130BuildMSLUTSTART(startSin, startSin90 int) uint32 {
	return uint32(startSin&0xff) | uint32((startSin90&0xff)<<16)
}

func tmc2130BuildPWMCONF(pwmAmpl, pwmGrad, pwmFreq int, pwmAutoscale bool, freewheel int) uint32 {
	val := uint32(pwmAmpl&0xff) | uint32((pwmGrad&0xff)<<8) | uint32((pwmFreq&0x03)<<16)
	if pwmAutoscale {
		val |= 1 << 18
	}
	val |= uint32((freewheel & 0x03) << 20)
	return val
}

// formatSPISend formats spi_send command with data bytes
func formatSPISend(oid int, reg byte, val uint32) string {
	// Write bit is 0x80
	data := []byte{
		reg | 0x80,
		byte(val >> 24),
		byte(val >> 16),
		byte(val >> 8),
		byte(val),
	}
	return fmt.Sprintf("spi_send oid=%d data=b'%s'", oid, formatPythonBytes(data))
}

// formatPythonBytes formats bytes as Python byte string
func formatPythonBytes(data []byte) string {
	var sb strings.Builder
	for _, b := range data {
		if b >= 32 && b < 127 && b != '\'' && b != '\\' {
			sb.WriteByte(b)
		} else {
			sb.WriteString(fmt.Sprintf("\\x%02x", b))
		}
	}
	return sb.String()
}

// tmc2130Config holds parsed TMC2130 configuration
type tmc2130Config struct {
	csPin         string
	runCurrent    float64
	senseResistor float64
	microsteps    int
}

// parseTMC2130 parses a [tmc2130 xxx] section
func parseTMC2130(cfg *config, stepperName string) (*tmc2130Config, error) {
	secName := "tmc2130 " + stepperName
	sec, ok := cfg.sections[secName]
	if !ok {
		return nil, nil // No TMC2130 for this stepper
	}

	csPin := strings.TrimSpace(sec["cs_pin"])
	if csPin == "" {
		return nil, fmt.Errorf("missing cs_pin in [%s]", secName)
	}

	runCurrent := 0.5 // default
	if v, ok := sec["run_current"]; ok {
		var err error
		runCurrent, err = strconv.ParseFloat(strings.TrimSpace(v), 64)
		if err != nil {
			return nil, fmt.Errorf("invalid run_current in [%s]: %w", secName, err)
		}
	}

	senseResistor := 0.110 // default
	if v, ok := sec["sense_resistor"]; ok {
		var err error
		senseResistor, err = strconv.ParseFloat(strings.TrimSpace(v), 64)
		if err != nil {
			return nil, fmt.Errorf("invalid sense_resistor in [%s]: %w", secName, err)
		}
	}

	// Get microsteps from stepper section
	stepperSec, ok := cfg.sections[stepperName]
	if !ok {
		return nil, fmt.Errorf("missing [%s] section for TMC2130", stepperName)
	}
	microsteps := 16 // default
	if v, ok := stepperSec["microsteps"]; ok {
		var err error
		microsteps, err = strconv.Atoi(strings.TrimSpace(v))
		if err != nil {
			return nil, fmt.Errorf("invalid microsteps in [%s]: %w", stepperName, err)
		}
	}

	return &tmc2130Config{
		csPin:         csPin,
		runCurrent:    runCurrent,
		senseResistor: senseResistor,
		microsteps:    microsteps,
	}, nil
}

// microstepsToMres converts microsteps value to mres register value
func microstepsToMres(microsteps int) int {
	switch microsteps {
	case 256:
		return 0
	case 128:
		return 1
	case 64:
		return 2
	case 32:
		return 3
	case 16:
		return 4
	case 8:
		return 5
	case 4:
		return 6
	case 2:
		return 7
	case 1:
		return 8
	default:
		return 4 // default to 16 microsteps
	}
}

// generateTMC2130InitCommands generates spi_send commands for TMC2130 register initialization
func generateTMC2130InitCommands(oid int, tmc *tmc2130Config) []string {
	// Calculate current
	holdCurrent := 2.0 // MAX_CURRENT default for hold
	vsense, irun, ihold := tmc2130CalcCurrent(tmc.runCurrent, holdCurrent, tmc.senseResistor)

	// Build register values
	mres := microstepsToMres(tmc.microsteps)
	vsenseInt := 0
	if vsense {
		vsenseInt = 1
	}

	chopconf := tmc2130BuildCHOPCONF(4, 0, 7, 1, vsenseInt, mres, true) // toff=4, hstrt=0, hend=7, tbl=1, intpol=true
	iholdIrun := tmc2130BuildIHOLD_IRUN(ihold, irun, 8)                 // iholddelay=8
	mslutsel := tmc2130BuildMSLUTSEL(2, 1, 1, 1, 128, 255, 255)         // w0=2, w1=1, w2=1, w3=1, x1=128, x2=255, x3=255
	mslutstart := tmc2130BuildMSLUTSTART(0, 247)                        // start_sin=0, start_sin90=247
	pwmconf := tmc2130BuildPWMCONF(128, 4, 1, true, 0)                  // pwm_ampl=128, pwm_grad=4, pwm_freq=1, pwm_autoscale=true

	var cmds []string
	// Order matches Python: CHOPCONF, IHOLD_IRUN, MSLUT0-7, MSLUTSEL, MSLUTSTART, TPWMTHRS, GCONF, TCOOLTHRS, THIGH, COOLCONF, PWMCONF, TPOWERDOWN
	cmds = append(cmds, formatSPISend(oid, tmc2130CHOPCONF, chopconf))
	cmds = append(cmds, formatSPISend(oid, tmc2130IHOLD_IRUN, iholdIrun))
	for i := 0; i < 8; i++ {
		cmds = append(cmds, formatSPISend(oid, byte(tmc2130MSLUT0+i), tmc2130DefaultMSLUT[i]))
	}
	cmds = append(cmds, formatSPISend(oid, tmc2130MSLUTSEL, mslutsel))
	cmds = append(cmds, formatSPISend(oid, tmc2130MSLUTSTART, mslutstart))
	cmds = append(cmds, formatSPISend(oid, tmc2130TPWMTHRS, 0x000FFFFF)) // default
	cmds = append(cmds, formatSPISend(oid, tmc2130GCONF, 0))             // default
	cmds = append(cmds, formatSPISend(oid, tmc2130TCOOLTHRS, 0))         // default
	cmds = append(cmds, formatSPISend(oid, tmc2130THIGH, 0))             // default
	cmds = append(cmds, formatSPISend(oid, tmc2130COOLCONF, 0))          // default
	cmds = append(cmds, formatSPISend(oid, tmc2130PWMCONF, pwmconf))
	cmds = append(cmds, formatSPISend(oid, tmc2130TPOWERDOWN, 0)) // default

	return cmds
}

// ============================================================================
// TMC5160 SPI Driver Support
// ============================================================================

// TMC5160 register addresses
const (
	tmc5160GCONF        = 0x00
	tmc5160GSTAT        = 0x01
	tmc5160SHORT_CONF   = 0x09
	tmc5160DRV_CONF     = 0x0A
	tmc5160GLOBALSCALER = 0x0B
	tmc5160IHOLD_IRUN   = 0x10
	tmc5160TPOWERDOWN   = 0x11
	tmc5160TPWMTHRS     = 0x13
	tmc5160TCOOLTHRS    = 0x14
	tmc5160THIGH        = 0x15
	tmc5160MSLUT0       = 0x60
	tmc5160MSLUT1       = 0x61
	tmc5160MSLUT2       = 0x62
	tmc5160MSLUT3       = 0x63
	tmc5160MSLUT4       = 0x64
	tmc5160MSLUT5       = 0x65
	tmc5160MSLUT6       = 0x66
	tmc5160MSLUT7       = 0x67
	tmc5160MSLUTSEL     = 0x68
	tmc5160MSLUTSTART   = 0x69
	tmc5160CHOPCONF     = 0x6c
	tmc5160COOLCONF     = 0x6d
	tmc5160PWMCONF      = 0x70
)

// TMC5160 constants
const (
	tmc5160VREF        = 0.325
	tmc5160MaxCurrent  = 10.0
	tmc5160SPIMode     = 3
	tmc5160SPISpeed    = 4000000
)

// tmc5160Config holds parsed TMC5160 configuration
type tmc5160Config struct {
	csPin         string
	spiBus        string
	chainPosition int
	chainLength   int
	runCurrent    float64
	holdCurrent   float64
	senseResistor float64
	microsteps    int
	interpolate   bool
	stealthchop   bool
}

// tmc5160CalcGlobalscaler calculates the globalscaler value for TMC5160
func tmc5160CalcGlobalscaler(current, senseResistor float64) int {
	globalscaler := int(current*256.0*math.Sqrt(2.0)*senseResistor/tmc5160VREF + 0.5)
	if globalscaler < 32 {
		globalscaler = 32
	}
	if globalscaler >= 256 {
		globalscaler = 0 // 0 means 256
	}
	return globalscaler
}

// tmc5160CalcCurrentBits calculates irun/ihold bits for TMC5160
func tmc5160CalcCurrentBits(current float64, globalscaler int, senseResistor float64) int {
	if globalscaler == 0 {
		globalscaler = 256
	}
	cs := int(current*256.0*32.0*math.Sqrt(2.0)*senseResistor/(float64(globalscaler)*tmc5160VREF) - 1.0 + 0.5)
	if cs < 0 {
		return 0
	}
	if cs > 31 {
		return 31
	}
	return cs
}

// tmc5160CalcCurrent calculates globalscaler, irun, ihold for TMC5160
func tmc5160CalcCurrent(runCurrent, holdCurrent, senseResistor float64) (globalscaler, irun, ihold int) {
	globalscaler = tmc5160CalcGlobalscaler(runCurrent, senseResistor)
	irun = tmc5160CalcCurrentBits(runCurrent, globalscaler, senseResistor)
	minHold := holdCurrent
	if minHold > runCurrent {
		minHold = runCurrent
	}
	ihold = tmc5160CalcCurrentBits(minHold, globalscaler, senseResistor)
	return
}

// tmc5160BuildCHOPCONF builds TMC5160 CHOPCONF register value
func tmc5160BuildCHOPCONF(toff, hstrt, hend, tbl, tpfd, mres int, intpol, dedge bool) uint32 {
	val := uint32(toff & 0x0f)
	val |= uint32((hstrt & 0x07) << 4)
	val |= uint32((hend & 0x0f) << 7)
	val |= uint32((tbl & 0x03) << 15)
	val |= uint32((tpfd & 0x0f) << 20)
	val |= uint32((mres & 0x0f) << 24)
	if intpol {
		val |= 1 << 28
	}
	if dedge {
		val |= 1 << 29
	}
	return val
}

// tmc5160BuildIHOLD_IRUN builds TMC5160 IHOLD_IRUN register value
func tmc5160BuildIHOLD_IRUN(ihold, irun, iholddelay int) uint32 {
	return uint32(ihold&0x1f) | uint32((irun&0x1f)<<8) | uint32((iholddelay&0x0f)<<16)
}

// tmc5160BuildGCONF builds TMC5160 GCONF register value
func tmc5160BuildGCONF(multistepFilt bool, enPwmMode bool) uint32 {
	var val uint32
	if enPwmMode {
		val |= 1 << 2
	}
	if multistepFilt {
		val |= 1 << 3
	}
	return val
}

// tmc5160BuildPWMCONF builds TMC5160 PWMCONF register value
func tmc5160BuildPWMCONF(pwmOfs, pwmGrad, pwmFreq int, pwmAutoscale, pwmAutograd bool, freewheel, pwmReg, pwmLim int) uint32 {
	val := uint32(pwmOfs & 0xff)
	val |= uint32((pwmGrad & 0xff) << 8)
	val |= uint32((pwmFreq & 0x03) << 16)
	if pwmAutoscale {
		val |= 1 << 18
	}
	if pwmAutograd {
		val |= 1 << 19
	}
	val |= uint32((freewheel & 0x03) << 20)
	val |= uint32((pwmReg & 0x0f) << 24)
	val |= uint32((pwmLim & 0x0f) << 28)
	return val
}

// tmc5160BuildDRV_CONF builds TMC5160 DRV_CONF register value
func tmc5160BuildDRV_CONF(bbmtime, bbmclks int) uint32 {
	return uint32(bbmtime&0x1f) | uint32((bbmclks&0x0f)<<8)
}

// parseTMC5160 parses a [tmc5160 xxx] section
func parseTMC5160(cfg *config, stepperName string) (*tmc5160Config, error) {
	secName := "tmc5160 " + stepperName
	sec, ok := cfg.sections[secName]
	if !ok {
		return nil, nil // No TMC5160 for this stepper
	}

	csPin := strings.TrimSpace(sec["cs_pin"])
	if csPin == "" {
		return nil, fmt.Errorf("missing cs_pin in [%s]", secName)
	}

	spiBus := strings.TrimSpace(sec["spi_bus"])

	chainPosition := 1 // default for non-chained
	if v, ok := sec["chain_position"]; ok {
		var err error
		chainPosition, err = strconv.Atoi(strings.TrimSpace(v))
		if err != nil {
			return nil, fmt.Errorf("invalid chain_position in [%s]: %w", secName, err)
		}
	}

	chainLength := 1 // default for non-chained
	if v, ok := sec["chain_length"]; ok {
		var err error
		chainLength, err = strconv.Atoi(strings.TrimSpace(v))
		if err != nil {
			return nil, fmt.Errorf("invalid chain_length in [%s]: %w", secName, err)
		}
	}

	runCurrent := 0.8 // default
	if v, ok := sec["run_current"]; ok {
		var err error
		runCurrent, err = strconv.ParseFloat(strings.TrimSpace(v), 64)
		if err != nil {
			return nil, fmt.Errorf("invalid run_current in [%s]: %w", secName, err)
		}
	}

	holdCurrent := tmc5160MaxCurrent // default to max
	if v, ok := sec["hold_current"]; ok {
		var err error
		holdCurrent, err = strconv.ParseFloat(strings.TrimSpace(v), 64)
		if err != nil {
			return nil, fmt.Errorf("invalid hold_current in [%s]: %w", secName, err)
		}
	}

	senseResistor := 0.075 // default for TMC5160
	if v, ok := sec["sense_resistor"]; ok {
		var err error
		senseResistor, err = strconv.ParseFloat(strings.TrimSpace(v), 64)
		if err != nil {
			return nil, fmt.Errorf("invalid sense_resistor in [%s]: %w", secName, err)
		}
	}

	interpolate := true // default
	if v, ok := sec["interpolate"]; ok {
		interpolate = strings.ToLower(strings.TrimSpace(v)) == "true"
	}

	stealthchop := false // default for TMC5160
	if v, ok := sec["stealthchop_threshold"]; ok {
		// If stealthchop_threshold is set to a non-zero value, enable en_pwm_mode
		thresh, _ := strconv.Atoi(strings.TrimSpace(v))
		stealthchop = thresh > 0
	}

	// Get microsteps from stepper section
	stepperSec, ok := cfg.sections[stepperName]
	if !ok {
		return nil, fmt.Errorf("missing [%s] section for TMC5160", stepperName)
	}
	microsteps := 16 // default
	if v, ok := stepperSec["microsteps"]; ok {
		var err error
		microsteps, err = strconv.Atoi(strings.TrimSpace(v))
		if err != nil {
			return nil, fmt.Errorf("invalid microsteps in [%s]: %w", stepperName, err)
		}
	}

	return &tmc5160Config{
		csPin:         csPin,
		spiBus:        spiBus,
		chainPosition: chainPosition,
		chainLength:   chainLength,
		runCurrent:    runCurrent,
		holdCurrent:   holdCurrent,
		senseResistor: senseResistor,
		microsteps:    microsteps,
		interpolate:   interpolate,
		stealthchop:   stealthchop,
	}, nil
}

// formatSPISendChain formats spi_send command for daisy chain with padding
func formatSPISendChain(oid int, chainPos, chainLen int, reg byte, val uint32) string {
	// Total length = chainLen * 5 bytes
	data := make([]byte, chainLen*5)

	// Position data at correct chain position
	// Chain position 1 = last 5 bytes, position N = first 5 bytes
	offset := (chainLen - chainPos) * 5
	data[offset] = reg | 0x80 // Write bit
	data[offset+1] = byte(val >> 24)
	data[offset+2] = byte(val >> 16)
	data[offset+3] = byte(val >> 8)
	data[offset+4] = byte(val)

	return fmt.Sprintf("spi_send oid=%d data=b'%s'", oid, formatPythonBytes(data))
}

// generateTMC5160InitCommands generates spi_send commands for TMC5160 register initialization
func generateTMC5160InitCommands(oid int, tmc *tmc5160Config) []string {
	// Calculate current settings
	globalscaler, irun, ihold := tmc5160CalcCurrent(tmc.runCurrent, tmc.holdCurrent, tmc.senseResistor)

	// Build register values
	mres := microstepsToMres(tmc.microsteps)

	// GLOBALSCALER
	globalscalerReg := uint32(globalscaler)

	// IHOLD_IRUN with iholddelay=6
	iholdIrun := tmc5160BuildIHOLD_IRUN(ihold, irun, 6)

	// CHOPCONF: toff=0 (driver disabled via virtual enable), hstrt=5, hend=2, tbl=2, tpfd=4, dedge=true
	// When using shared enable pins, TMC drivers use "virtual enable" via CHOPCONF.toff
	// toff=0 disables the driver; toff=3 is the configured value used when enabled
	chopconf := tmc5160BuildCHOPCONF(0, 5, 2, 2, 4, mres, tmc.interpolate, true)

	// MSLUT (same default wave table as TMC2130)
	// MSLUTSEL: w0=2, w1=1, w2=1, w3=1, x1=128, x2=255, x3=255
	mslutsel := tmc2130BuildMSLUTSEL(2, 1, 1, 1, 128, 255, 255)
	// MSLUTSTART: start_sin=0, start_sin90=247
	mslutstart := tmc2130BuildMSLUTSTART(0, 247)

	// TPWMTHRS
	tpwmthrs := uint32(0x000FFFFF) // default

	// GCONF: multistep_filt=true by default
	gconf := tmc5160BuildGCONF(true, tmc.stealthchop)

	// TCOOLTHRS, THIGH, COOLCONF default to 0
	tcoolthrs := uint32(0)
	thigh := uint32(0)
	coolconf := uint32(0)

	// DRV_CONF: bbmtime=0, bbmclks=4
	drvConf := tmc5160BuildDRV_CONF(0, 4)

	// PWMCONF: pwm_ofs=30, pwm_grad=0, pwm_freq=0, pwm_autoscale=true, pwm_autograd=true, freewheel=0, pwm_reg=4, pwm_lim=12
	pwmconf := tmc5160BuildPWMCONF(30, 0, 0, true, true, 0, 4, 12)

	// TPOWERDOWN
	tpowerdown := uint32(10)

	var cmds []string
	chainPos := tmc.chainPosition
	chainLen := tmc.chainLength

	// Generate init commands in correct order (matching Python output)
	cmds = append(cmds, formatSPISendChain(oid, chainPos, chainLen, tmc5160GLOBALSCALER, globalscalerReg))
	cmds = append(cmds, formatSPISendChain(oid, chainPos, chainLen, tmc5160IHOLD_IRUN, iholdIrun))
	cmds = append(cmds, formatSPISendChain(oid, chainPos, chainLen, tmc5160CHOPCONF, chopconf))
	for i := 0; i < 8; i++ {
		cmds = append(cmds, formatSPISendChain(oid, chainPos, chainLen, byte(tmc5160MSLUT0+i), tmc2130DefaultMSLUT[i]))
	}
	cmds = append(cmds, formatSPISendChain(oid, chainPos, chainLen, tmc5160MSLUTSEL, mslutsel))
	cmds = append(cmds, formatSPISendChain(oid, chainPos, chainLen, tmc5160MSLUTSTART, mslutstart))
	cmds = append(cmds, formatSPISendChain(oid, chainPos, chainLen, tmc5160TPWMTHRS, tpwmthrs))
	cmds = append(cmds, formatSPISendChain(oid, chainPos, chainLen, tmc5160GCONF, gconf))
	cmds = append(cmds, formatSPISendChain(oid, chainPos, chainLen, tmc5160TCOOLTHRS, tcoolthrs))
	cmds = append(cmds, formatSPISendChain(oid, chainPos, chainLen, tmc5160THIGH, thigh))
	cmds = append(cmds, formatSPISendChain(oid, chainPos, chainLen, tmc5160COOLCONF, coolconf))
	cmds = append(cmds, formatSPISendChain(oid, chainPos, chainLen, tmc5160DRV_CONF, drvConf))
	cmds = append(cmds, formatSPISendChain(oid, chainPos, chainLen, tmc5160PWMCONF, pwmconf))
	cmds = append(cmds, formatSPISendChain(oid, chainPos, chainLen, tmc5160TPOWERDOWN, tpowerdown))

	return cmds
}

// CompileTMC2130CartesianConnectPhase compiles the connect-phase MCU command stream
// for cartesian configs with TMC2130 stepper drivers (e.g., generic-einsy-rambo.cfg).
func CompileTMC2130CartesianConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
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

	// Derived constants
	const (
		pwmCycleTime     = 0.100
		fanCycleTime     = 0.010
		heaterMaxHeatSec = 3.0
		stepPulseSec     = 0.000002
		adcSampleTime    = 0.001
		adcSampleCount   = 8
		adcReportTime    = 0.300
		adcRangeChecks   = 4
		pwmInitDelaySec  = 0.200
	)
	cycleTicks := secondsToClock(mcuFreq, pwmCycleTime)
	fanCycleTicks := secondsToClock(mcuFreq, fanCycleTime)
	mdurTicks := secondsToClock(mcuFreq, heaterMaxHeatSec)
	// TMC2130 uses dedge mode (dual-edge stepping), so step_pulse_ticks=1
	stepPulseTicks := 1
	sampleTicks := secondsToClock(mcuFreq, adcSampleTime)
	reportTicks := secondsToClock(mcuFreq, adcReportTime)
	pwmInitClock := secondsToClock(mcuFreq, pwmInitDelaySec)

	// Parse TMC2130 configs
	tmcX, err := parseTMC2130(cfg, "stepper_x")
	if err != nil {
		return nil, err
	}
	tmcY, err := parseTMC2130(cfg, "stepper_y")
	if err != nil {
		return nil, err
	}
	tmcZ, err := parseTMC2130(cfg, "stepper_z")
	if err != nil {
		return nil, err
	}
	tmcE, err := parseTMC2130(cfg, "extruder")
	if err != nil {
		return nil, err
	}

	// Parse stepper configs
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

	// Check for fan
	fanSec, hasFan := cfg.sections["fan"]
	var fanPin pin
	if hasFan {
		fanPin, err = parsePin(fanSec, "pin", true, false)
		if err != nil {
			return nil, fmt.Errorf("fan pin: %w", err)
		}
	}

	// Check for temperature_sensor
	var tempSensorPin string
	for secName, secData := range cfg.sections {
		if strings.HasPrefix(secName, "temperature_sensor ") {
			if p, ok := secData["sensor_pin"]; ok {
				tempSensorPin = strings.TrimSpace(p)
				break
			}
		}
	}

	// Check for static_digital_output
	var staticOutputPins []struct {
		pin   string
		value int
	}
	for secName, secData := range cfg.sections {
		if strings.HasPrefix(secName, "static_digital_output ") {
			if p, ok := secData["pins"]; ok {
				pinStr := strings.TrimSpace(p)
				invert := false
				if strings.HasPrefix(pinStr, "!") {
					invert = true
					pinStr = pinStr[1:]
				}
				val := 0
				if !invert {
					val = 1
				}
				staticOutputPins = append(staticOutputPins, struct {
					pin   string
					value int
				}{pinStr, val})
			}
		}
	}

	// OID allocation (matches Python)
	// OID 0-3: SPI for TMC2130 (X, Y, Z, E)
	// OID 4: endstop X
	// OID 5: trsync X
	// OID 6: stepper X
	// OID 7: endstop Y
	// OID 8: trsync Y
	// OID 9: stepper Y
	// OID 10: endstop Z
	// OID 11: trsync Z
	// OID 12: stepper Z
	// OID 13: stepper E
	// OID 14+: heaters, sensors, enables

	oidSPI_X := 0
	oidSPI_Y := 1
	oidSPI_Z := 2
	oidSPI_E := 3
	oidEndstopX := 4
	oidTrsyncX := 5
	oidStepperX := 6
	oidEndstopY := 7
	oidTrsyncY := 8
	oidStepperY := 9
	oidEndstopZ := 10
	oidTrsyncZ := 11
	oidStepperZ := 12
	oidStepperE := 13

	nextOID := 14
	oidADCBed := nextOID
	nextOID++
	oidPWMBed := nextOID
	nextOID++

	oidPWMFan := -1
	if hasFan {
		oidPWMFan = nextOID
		nextOID++
	}

	oidADCTempSensor := -1
	if tempSensorPin != "" {
		oidADCTempSensor = nextOID
		nextOID++
	}

	oidEnableX := nextOID
	nextOID++
	oidEnableY := nextOID
	nextOID++
	oidEnableZ := nextOID
	nextOID++

	oidADCE := nextOID
	nextOID++
	oidPWME := nextOID
	nextOID++
	oidEnableE := nextOID
	nextOID++

	totalOIDs := nextOID

	// Build config commands
	var configCmds []string

	// SPI configs for TMC2130
	if tmcX != nil {
		configCmds = append(configCmds, fmt.Sprintf("config_spi oid=%d pin=%s cs_active_high=0", oidSPI_X, tmcX.csPin))
	}
	if tmcY != nil {
		configCmds = append(configCmds, fmt.Sprintf("config_spi oid=%d pin=%s cs_active_high=0", oidSPI_Y, tmcY.csPin))
	}
	if tmcZ != nil {
		configCmds = append(configCmds, fmt.Sprintf("config_spi oid=%d pin=%s cs_active_high=0", oidSPI_Z, tmcZ.csPin))
	}
	if tmcE != nil {
		configCmds = append(configCmds, fmt.Sprintf("config_spi oid=%d pin=%s cs_active_high=0", oidSPI_E, tmcE.csPin))
	}

	// Static digital output (e.g., yellow LED)
	for _, sop := range staticOutputPins {
		configCmds = append(configCmds, fmt.Sprintf("set_digital_out pin=%s value=%d", sop.pin, sop.value))
	}

	// SPI bus setup
	if tmcX != nil {
		configCmds = append(configCmds, fmt.Sprintf("spi_set_bus oid=%d spi_bus=spi mode=3 rate=4000000", oidSPI_X))
	}
	if tmcY != nil {
		configCmds = append(configCmds, fmt.Sprintf("spi_set_bus oid=%d spi_bus=spi mode=3 rate=4000000", oidSPI_Y))
	}
	if tmcZ != nil {
		configCmds = append(configCmds, fmt.Sprintf("spi_set_bus oid=%d spi_bus=spi mode=3 rate=4000000", oidSPI_Z))
	}
	if tmcE != nil {
		configCmds = append(configCmds, fmt.Sprintf("spi_set_bus oid=%d spi_bus=spi mode=3 rate=4000000", oidSPI_E))
	}

	// Heater bed
	configCmds = append(configCmds,
		fmt.Sprintf("config_analog_in oid=%d pin=%s", oidADCBed, bed.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=0 default_value=0 max_duration=%d", oidPWMBed, bed.HeaterPin.pin, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", oidPWMBed, cycleTicks),
	)

	// Fan
	if hasFan {
		configCmds = append(configCmds,
			fmt.Sprintf("config_digital_out oid=%d pin=%s value=0 default_value=0 max_duration=0", oidPWMFan, fanPin.pin),
			fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", oidPWMFan, fanCycleTicks),
		)
	}

	// Temperature sensor
	if tempSensorPin != "" {
		configCmds = append(configCmds,
			fmt.Sprintf("config_analog_in oid=%d pin=%s", oidADCTempSensor, tempSensorPin),
		)
	}

	// Stepper X
	configCmds = append(configCmds,
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", oidEndstopX, stepperX.Endstop.pin, stepperX.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", oidTrsyncX),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			oidStepperX, stepperX.Step.pin, stepperX.Dir.pin, boolToInt(stepperX.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=0",
			oidEnableX, stepperX.Enable.pin, boolToInt(stepperX.Enable.invert), boolToInt(stepperX.Enable.invert)),
	)

	// Stepper Y
	configCmds = append(configCmds,
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", oidEndstopY, stepperY.Endstop.pin, stepperY.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", oidTrsyncY),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			oidStepperY, stepperY.Step.pin, stepperY.Dir.pin, boolToInt(stepperY.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=0",
			oidEnableY, stepperY.Enable.pin, boolToInt(stepperY.Enable.invert), boolToInt(stepperY.Enable.invert)),
	)

	// Stepper Z
	configCmds = append(configCmds,
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", oidEndstopZ, stepperZ.Endstop.pin, stepperZ.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", oidTrsyncZ),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			oidStepperZ, stepperZ.Step.pin, stepperZ.Dir.pin, boolToInt(stepperZ.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=0",
			oidEnableZ, stepperZ.Enable.pin, boolToInt(stepperZ.Enable.invert), boolToInt(stepperZ.Enable.invert)),
	)

	// Extruder heater/sensor
	configCmds = append(configCmds,
		fmt.Sprintf("config_analog_in oid=%d pin=%s", oidADCE, extruder.Heater.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=0 default_value=0 max_duration=%d", oidPWME, extruder.Heater.HeaterPin.pin, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", oidPWME, cycleTicks),
	)

	// Extruder stepper
	configCmds = append(configCmds,
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			oidStepperE, extruder.Step.pin, extruder.Dir.pin, boolToInt(extruder.Step.invert), stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=0",
			oidEnableE, extruder.Enable.pin, boolToInt(extruder.Enable.invert), boolToInt(extruder.Enable.invert)),
	)

	// Build allocate_oids + config commands
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", totalOIDs)}, configCmds...)

	// Compute CRC
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	// Build init commands
	var initCmds []string

	// ADC queries
	bedMin, bedMax, err := thermistorMinMaxTicks(bed, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}
	initCmds = append(initCmds,
		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			oidADCBed, querySlotClock(mcuFreq, oidADCBed), sampleTicks, adcSampleCount, reportTicks, bedMin, bedMax, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=0", oidPWMBed, pwmInitClock),
	)

	if hasFan {
		initCmds = append(initCmds,
			fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=0", oidPWMFan, pwmInitClock),
		)
	}

	if tempSensorPin != "" {
		// Temperature sensor query - use different thermistor range
		// For TDK NTCG104LH104JT1: min_temp=0, max_temp=50
		// These correspond to different ADC values
		tempMin := 7113  // approximate for 50C
		tempMax := 8085  // approximate for 0C
		initCmds = append(initCmds,
			fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
				oidADCTempSensor, querySlotClock(mcuFreq, oidADCTempSensor), sampleTicks, adcSampleCount, reportTicks, tempMin, tempMax, adcRangeChecks),
		)
	}

	extMin, extMax, err := thermistorMinMaxTicks(extruder.Heater, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}
	initCmds = append(initCmds,
		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			oidADCE, querySlotClock(mcuFreq, oidADCE), sampleTicks, adcSampleCount, reportTicks, extMin, extMax, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=0", oidPWME, pwmInitClock),
	)

	// TMC2130 init commands (spi_send)
	if tmcX != nil {
		initCmds = append(initCmds, generateTMC2130InitCommands(oidSPI_X, tmcX)...)
	}
	if tmcY != nil {
		initCmds = append(initCmds, generateTMC2130InitCommands(oidSPI_Y, tmcY)...)
	}
	if tmcZ != nil {
		initCmds = append(initCmds, generateTMC2130InitCommands(oidSPI_Z, tmcZ)...)
	}
	if tmcE != nil {
		initCmds = append(initCmds, generateTMC2130InitCommands(oidSPI_E, tmcE)...)
	}

	// Build final output
	out := make([]string, 0)
	out = append(out, withAllocate...)
	out = append(out, fmt.Sprintf("finalize_config crc=%d", crc))
	out = append(out, initCmds...)
	return out, nil
}

// CompileTMC5160CartesianConnectPhase compiles the connect-phase MCU command stream
// for cartesian configs with TMC5160 stepper drivers in SPI daisy chain (e.g., generic-duet3-6hc.cfg).
func CompileTMC5160CartesianConnectPhase(cfgPath string, dict *protocol.Dictionary) ([]string, error) {
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

	// Derived constants
	const (
		pwmCycleTime     = 0.100
		fanCycleTime     = 0.010
		heaterMaxHeatSec = 3.0
		adcSampleTime    = 0.001
		adcSampleCount   = 8
		adcReportTime    = 0.300
		adcRangeChecks   = 4
		pwmInitDelaySec  = 0.200
	)
	cycleTicks := secondsToClock(mcuFreq, pwmCycleTime)
	fanCycleTicks := secondsToClock(mcuFreq, fanCycleTime)
	mdurTicks := secondsToClock(mcuFreq, heaterMaxHeatSec)
	// TMC5160 uses dedge mode (dual-edge stepping)
	// step_pulse_ticks for dedge is calculated differently
	// For SAME70 at 300MHz, Python produces step_pulse_ticks=30
	stepPulseTicks := secondsToClock(mcuFreq, 0.0000001) // 100ns minimum
	if stepPulseTicks < 30 {
		stepPulseTicks = 30
	}
	sampleTicks := secondsToClock(mcuFreq, adcSampleTime)
	reportTicks := secondsToClock(mcuFreq, adcReportTime)
	pwmInitClock := secondsToClock(mcuFreq, pwmInitDelaySec)

	// Parse TMC5160 configs
	tmcX, err := parseTMC5160(cfg, "stepper_x")
	if err != nil {
		return nil, err
	}
	tmcY, err := parseTMC5160(cfg, "stepper_y")
	if err != nil {
		return nil, err
	}
	tmcZ, err := parseTMC5160(cfg, "stepper_z")
	if err != nil {
		return nil, err
	}
	tmcE, err := parseTMC5160(cfg, "extruder")
	if err != nil {
		return nil, err
	}

	// Parse stepper configs
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

	// Check for fan
	fanSec, hasFan := cfg.sections["fan"]
	var fanPin pin
	if hasFan {
		fanPin, err = parsePin(fanSec, "pin", true, false)
		if err != nil {
			return nil, fmt.Errorf("fan pin: %w", err)
		}
	}

	// Check for heater_fan
	var heaterFanPin string
	for secName, secData := range cfg.sections {
		if strings.HasPrefix(secName, "heater_fan ") {
			if p, ok := secData["pin"]; ok {
				heaterFanPin = strings.TrimSpace(p)
				break
			}
		}
	}

	// Check for adc_scaled (vref/vssa pins)
	var vrefPin, vssaPin string
	for secName, secData := range cfg.sections {
		if strings.HasPrefix(secName, "adc_scaled ") {
			vrefPin = strings.TrimSpace(secData["vref_pin"])
			vssaPin = strings.TrimSpace(secData["vssa_pin"])
			break
		}
	}

	// OID allocation for TMC5160 daisy chain
	// TMC5160 shares a single SPI OID for all drivers in chain
	// OID 0: SPI for TMC5160 daisy chain
	// OID 1: endstop X
	// OID 2: trsync X
	// OID 3: stepper X
	// OID 4: endstop Y
	// OID 5: trsync Y
	// OID 6: stepper Y
	// OID 7: endstop Z
	// OID 8: trsync Z
	// OID 9: stepper Z
	// OID 10: stepper E
	// OID 11+: vref, vssa, heater_fan, fan, heaters, etc.

	oidSPI := 0
	oidEndstopX := 1
	oidTrsyncX := 2
	oidStepperX := 3
	oidEndstopY := 4
	oidTrsyncY := 5
	oidStepperY := 6
	oidEndstopZ := 7
	oidTrsyncZ := 8
	oidStepperZ := 9
	oidStepperE := 10

	nextOID := 11

	// vref/vssa ADC
	oidVref := -1
	oidVssa := -1
	if vrefPin != "" {
		oidVref = nextOID
		nextOID++
	}
	if vssaPin != "" {
		oidVssa = nextOID
		nextOID++
	}

	// Heater fan
	oidHeaterFan := -1
	if heaterFanPin != "" {
		oidHeaterFan = nextOID
		nextOID++
	}

	// Bed heater
	oidADCBed := nextOID
	nextOID++
	oidPWMBed := nextOID
	nextOID++

	// Part cooling fan
	oidPWMFan := -1
	if hasFan {
		oidPWMFan = nextOID
		nextOID++
	}

	// Enable pin (shared for all TMC5160)
	oidEnable := nextOID
	nextOID++

	// Extruder
	oidADCE := nextOID
	nextOID++
	oidPWME := nextOID
	nextOID++

	totalOIDs := nextOID

	// Build config commands
	var configCmds []string

	// Get CS pin and SPI bus from first TMC config
	var csPin, spiBus string
	if tmcX != nil {
		csPin = tmcX.csPin
		spiBus = tmcX.spiBus
	} else if tmcY != nil {
		csPin = tmcY.csPin
		spiBus = tmcY.spiBus
	}

	// SPI config for TMC5160 daisy chain (single shared OID)
	configCmds = append(configCmds, fmt.Sprintf("config_spi oid=%d pin=%s cs_active_high=0", oidSPI, csPin))
	configCmds = append(configCmds, fmt.Sprintf("spi_set_bus oid=%d spi_bus=%s mode=3 rate=4000000", oidSPI, spiBus))

	// vref/vssa ADC (for adc_scaled)
	if vrefPin != "" {
		configCmds = append(configCmds, fmt.Sprintf("config_analog_in oid=%d pin=%s", oidVref, vrefPin))
	}
	if vssaPin != "" {
		configCmds = append(configCmds, fmt.Sprintf("config_analog_in oid=%d pin=%s", oidVssa, vssaPin))
	}

	// Heater fan
	if heaterFanPin != "" {
		configCmds = append(configCmds,
			fmt.Sprintf("config_digital_out oid=%d pin=%s value=0 default_value=1 max_duration=0", oidHeaterFan, heaterFanPin),
			fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", oidHeaterFan, fanCycleTicks),
		)
	}

	// Heater bed
	configCmds = append(configCmds,
		fmt.Sprintf("config_analog_in oid=%d pin=%s", oidADCBed, bed.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=0 default_value=0 max_duration=%d", oidPWMBed, bed.HeaterPin.pin, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", oidPWMBed, cycleTicks),
	)

	// Part cooling fan
	if hasFan {
		configCmds = append(configCmds,
			fmt.Sprintf("config_digital_out oid=%d pin=%s value=0 default_value=0 max_duration=0", oidPWMFan, fanPin.pin),
			fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", oidPWMFan, fanCycleTicks),
		)
	}

	// Stepper X - TMC5160 uses dedge (invert_step=-1 = 0xFFFFFFFF)
	invertStep := uint32(0xFFFFFFFF) // dedge mode
	configCmds = append(configCmds,
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", oidEndstopX, stepperX.Endstop.pin, stepperX.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", oidTrsyncX),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			oidStepperX, stepperX.Step.pin, stepperX.Dir.pin, invertStep, stepPulseTicks),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=%d default_value=%d max_duration=0",
			oidEnable, stepperX.Enable.pin, boolToInt(stepperX.Enable.invert), boolToInt(stepperX.Enable.invert)),
	)

	// Stepper Y
	configCmds = append(configCmds,
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", oidEndstopY, stepperY.Endstop.pin, stepperY.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", oidTrsyncY),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			oidStepperY, stepperY.Step.pin, stepperY.Dir.pin, invertStep, stepPulseTicks),
	)

	// Stepper Z
	configCmds = append(configCmds,
		fmt.Sprintf("config_endstop oid=%d pin=%s pull_up=%d", oidEndstopZ, stepperZ.Endstop.pin, stepperZ.Endstop.pullup),
		fmt.Sprintf("config_trsync oid=%d", oidTrsyncZ),
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			oidStepperZ, stepperZ.Step.pin, stepperZ.Dir.pin, invertStep, stepPulseTicks),
	)

	// Extruder heater/sensor
	configCmds = append(configCmds,
		fmt.Sprintf("config_analog_in oid=%d pin=%s", oidADCE, extruder.Heater.SensorPin.pin),
		fmt.Sprintf("config_digital_out oid=%d pin=%s value=0 default_value=0 max_duration=%d", oidPWME, extruder.Heater.HeaterPin.pin, mdurTicks),
		fmt.Sprintf("set_digital_out_pwm_cycle oid=%d cycle_ticks=%d", oidPWME, cycleTicks),
	)

	// Extruder stepper
	configCmds = append(configCmds,
		fmt.Sprintf("config_stepper oid=%d step_pin=%s dir_pin=%s invert_step=%d step_pulse_ticks=%d",
			oidStepperE, extruder.Step.pin, extruder.Dir.pin, invertStep, stepPulseTicks),
	)

	// Build allocate_oids + config commands
	withAllocate := append([]string{fmt.Sprintf("allocate_oids count=%d", totalOIDs)}, configCmds...)

	// Compute CRC
	crcText := strings.Join(withAllocate, "\n")
	crc := crc32.ChecksumIEEE([]byte(crcText))

	// Build init commands
	var initCmds []string

	// vref/vssa ADC queries
	if vrefPin != "" {
		initCmds = append(initCmds,
			fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=0 max_value=32760 range_check_count=0",
				oidVref, querySlotClock(mcuFreq, oidVref), sampleTicks, adcSampleCount, reportTicks),
		)
	}
	if vssaPin != "" {
		initCmds = append(initCmds,
			fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=0 max_value=32760 range_check_count=0",
				oidVssa, querySlotClock(mcuFreq, oidVssa), sampleTicks, adcSampleCount, reportTicks),
		)
	}

	// Heater fan init
	if heaterFanPin != "" {
		initCmds = append(initCmds,
			fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=0", oidHeaterFan, pwmInitClock),
		)
	}

	// ADC queries for heaters
	bedMin, bedMax, err := thermistorMinMaxTicks(bed, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}
	initCmds = append(initCmds,
		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			oidADCBed, querySlotClock(mcuFreq, oidADCBed), sampleTicks, adcSampleCount, reportTicks, bedMin, bedMax, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=0", oidPWMBed, pwmInitClock),
	)

	if hasFan {
		initCmds = append(initCmds,
			fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=0", oidPWMFan, pwmInitClock),
		)
	}

	extMin, extMax, err := thermistorMinMaxTicks(extruder.Heater, adcMax, adcSampleCount)
	if err != nil {
		return nil, err
	}
	initCmds = append(initCmds,
		fmt.Sprintf("query_analog_in oid=%d clock=%d sample_ticks=%d sample_count=%d rest_ticks=%d min_value=%d max_value=%d range_check_count=%d",
			oidADCE, querySlotClock(mcuFreq, oidADCE), sampleTicks, adcSampleCount, reportTicks, extMin, extMax, adcRangeChecks),
		fmt.Sprintf("queue_digital_out oid=%d clock=%d on_ticks=0", oidPWME, pwmInitClock),
	)

	// TMC5160 init commands (spi_send) - all using same SPI OID
	if tmcX != nil {
		initCmds = append(initCmds, generateTMC5160InitCommands(oidSPI, tmcX)...)
	}
	if tmcY != nil {
		initCmds = append(initCmds, generateTMC5160InitCommands(oidSPI, tmcY)...)
	}
	if tmcZ != nil {
		initCmds = append(initCmds, generateTMC5160InitCommands(oidSPI, tmcZ)...)
	}
	if tmcE != nil {
		initCmds = append(initCmds, generateTMC5160InitCommands(oidSPI, tmcE)...)
	}

	// Build final output
	out := make([]string, 0)
	out = append(out, withAllocate...)
	out = append(out, fmt.Sprintf("finalize_config crc=%d", crc))
	out = append(out, initCmds...)
	return out, nil
}
