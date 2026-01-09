# Temperature Package

Temperature control system for Klipper Go migration.

## Overview

This package provides a complete temperature control system for 3D printer firmware, including:

- Temperature sensor interfaces and implementations
- Heater control with PWM output
- Multiple control algorithms (PID and Bang-Bang)
- G-code command handlers (M104, M140, M105, M109, M190)
- Multi-heater management

## Architecture

```
temperature/
├── sensor.go      # Temperature sensor interfaces and MCU sensor
├── heater.go      # Heater control logic
├── control.go     # PID and bang-bang control algorithms
├── manager.go     # Multi-heater management
├── gcode.go       # G-code command handlers
└── example_test.go # Usage examples
```

## Components

### Sensor Interface

```go
type Sensor interface {
    SetupCallback(callback TemperatureCallback)
    GetReportTimeDelta() float64
    SetupMinMax(minTemp, maxTemp float64) error
    GetTemp(eventtime float64) (current, target float64)
    GetName() string
}
```

The sensor interface is implemented by various temperature sensors:
- `MCUSensor` - MCU internal temperature sensor via ADC
- Future: Thermistor, AD595, PT100, MAX6675, etc.

### Heater Control

The `Heater` struct manages a heating element with temperature feedback:

```go
heater, err := temperature.NewHeater(config, sensor, pwm, printer)
heater.SetTemp(210.0)  // Set target temperature to 210°C
current, target := heater.GetTemp(eventtime)
busy := heater.CheckBusy(eventtime)
```

Features:
- Temperature tracking with smoothing
- Min/max temperature validation
- PWM output control
- Min extrude temperature checking
- Thread-safe operations

### Control Algorithms

Two control algorithms are implemented:

#### Bang-Bang Control (Watermark)

Simple on/off control with hysteresis:

```go
config := &temperature.HeaterConfig{
    ControlType: "watermark",
    MaxDelta:    2.0,  // Turn on at target-2, off at target+2
}
```

#### PID Control

Proportional-Integral-Derivative control:

```go
config := &temperature.HeaterConfig{
    ControlType: "pid",
    PID_Kp:      22.2,
    PID_Ki:      1.08,
    PID_Kd:      114.0,
}
```

### Heater Manager

Manages multiple heaters and sensors:

```go
manager := temperature.NewHeaterManager(printer)

// Setup heaters
manager.SetupHeater("extruder", extruderConfig, sensor, pwm)
manager.SetupHeater("heater_bed", bedConfig, bedSensor, bedPWM)

// Control heaters
manager.SetTemperature("extruder", 210.0, false)
manager.SetTemperature("heater_bed", 60.0, false)

// Get M105 response
response := manager.GetM105Response(eventtime)
// Output: "T0:210.0 /210.0 B:60.0 /60.0"
```

### G-Code Commands

The package implements standard temperature G-code commands:

| Command | Description |
|---------|-------------|
| M104 S<temp> | Set extruder temperature |
| M140 S<temp> | Set heated bed temperature |
| M105 | Get temperature status |
| M109 S<temp> | Set extruder temperature and wait |
| M190 S<temp> | Set heated bed temperature and wait |
| SET_HEATER_TEMPERATURE HEATER=<name> TARGET=<temp> | Set any heater temperature |
| TURN_OFF_HEATERS | Turn off all heaters |
| TEMPERATURE_WAIT SENSOR=<name> MINIMUM=<temp> MAXIMUM=<temp> | Wait for temperature |

## Usage Example

```go
import "github.com/klipper-go/temperature"

// Create manager
manager := temperature.NewHeaterManager(printer)

// Create sensor
sensorConfig := &temperature.MCUConfig{
    SensorMCU: "mcu",
}
sensor := temperature.NewMCUSensor("temperature_sensor", sensorConfig, adc)
sensor.SetCalibration(25.0, 100.0)

// Create heater config
heaterConfig := &temperature.HeaterConfig{
    Name:         "extruder",
    MinTemp:      0.0,
    MaxTemp:      250.0,
    MaxPower:     1.0,
    SmoothTime:   1.0,
    PWMCycleTime: 0.1,
    ControlType:  "pid",
    PID_Kp:       22.2,
    PID_Ki:       1.08,
    PID_Kd:       114.0,
}

// Setup heater
heater, err := temperature.NewHeater(heaterConfig, sensor, pwm, printer)

// Set temperature
heater.SetTemp(210.0)

// Get status
status := heater.GetStatus(eventtime)
```

## Constants

Important constants defined in the package:

```go
const (
    KelvinToCelsius    = -273.15  // Kelvin to Celsius conversion
    MaxHeatTime        = 3.0      // Maximum heater on time (seconds)
    AmbientTemp        = 25.0     // Default ambient temperature (°C)
    PIDParamBase       = 255.0    // PID parameter normalization base
    ReportTime         = 0.300    // Temperature report interval (seconds)
    QuellStaleTime     = 7.0      // Time before readings are stale (seconds)
)
```

## Thread Safety

The `Heater` struct uses mutex locks to ensure thread-safe operations:
- All public methods that access state are mutex-protected
- Temperature callbacks are synchronized
- Safe for concurrent access from multiple goroutines

## Temperature Smoothing

The heater implements exponential smoothing on temperature readings:

```
smoothed_temp += (temp - smoothed_temp) * min(time_diff / smooth_time, 1.0)
```

This helps filter out noise from sensor readings.

## PWM Output

PWM output is controlled with the following features:
- Cycle time configuration (default 0.1 seconds)
- Maximum power limiting (0.0 to 1.0)
- PWM change suppression (ignores changes < 5%)
- Automatic shutoff on target=0 or main thread timeout

## PID Algorithm Details

The PID controller implements standard PID control:

```
error = target - current
integral = integral + error * dt
derivative = (current - prev) / dt
output = Kp*error + Ki*integral - Kd*derivative
```

Features:
- Integral windup protection (clamped to valid range)
- Derivative smoothing over `min_deriv_time`
- Output bounded to [0, max_power]

## Integration with Klipper

To integrate with the Klipper host code:

1. Implement the required interfaces:
   - `PrinterInterface` - Printer status and error reporting
   - `PWMInterface` - PWM output to MCU
   - `ADCInterface` - ADC input from MCU
   - `MCUInterface` - MCU timing

2. Register G-code command handlers with the gcode dispatcher

3. Call `manager.HandleReady()` when the printer is ready

4. Call `manager.VerifyMainThreadTime()` periodically

## Future Work

- [ ] Additional sensor types (Thermistor, PT100, MAX6675, etc.)
- [ ] Temperature fan control
- [ ] Config file parser integration
- [ ] MCU protocol integration
- [ ] Unit tests
- [ ] Integration tests with golden test framework

## License

GNU GPLv3
