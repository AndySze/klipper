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
        adcBed: 10, pwmBed: 11,
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
        count:    12,
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
        adcBed: 10, pwmBed: 11, // Unused but keep for OID stability
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
        extraStepper    int
        endstopX        int
        trsyncX         int
        stepperX        int
        endstopY        int
        trsyncY         int
        stepperY        int
        endstopZ        int
        trsyncZ         int
        stepperZ        int
        stepperE        int
        enableExtra     int
        buttons         int
        enableX         int
        enableY         int
        enableZ         int
        adcE            int
        pwmE            int
        enableE         int
        count           int
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

    adcE int
    pwmE int
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
