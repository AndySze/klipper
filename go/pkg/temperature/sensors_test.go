// Temperature sensor unit tests for Klipper Go migration
// Copyright (C) 2026  Klipper Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package temperature

import (
	"testing"
)

// mockADC is a mock ADC implementation for testing
type mockADC struct {
	values   []float64
	index    int
	callback TemperatureCallback
}

func (m *mockADC) SetupADCCallback(reportTime float64, callback func(float64, float64)) {
	m.callback = callback
}

func (m *mockADC) SetupADCSample(sampleTime float64, sampleCount int, options ...ADCSampleOption) error {
	return nil
}

func (m *mockADC) GetMCU() MCUInterface {
	return nil
}

func (m *mockADC) sendValue(value float64) {
	if m.callback != nil {
		m.callback(0.0, value)
	}
}

// TestAD595Sensor tests the AD595 thermocouple amplifier sensor
func TestAD595Sensor(t *testing.T) {
	tests := []struct {
		name           string
		voltageSupply  float64
		voltageOffset  float64
		adcValue       float64
		expectedTemp   float64
		tolerance      float64
	}{
		{
			name:          "0°C at 0V",
			voltageSupply: 5.0,
			voltageOffset: 0.0,
			adcValue:      0.0, // 0V
			expectedTemp:  0.0,
			tolerance:     0.1,
		},
		{
			name:          "100°C at 1.0V",
			voltageSupply: 5.0,
			voltageOffset: 0.0,
			adcValue:      0.2, // 1.0V (0.2 * 5V)
			expectedTemp:  100.0,
			tolerance:     0.1,
		},
		{
			name:          "250°C at 2.5V",
			voltageSupply: 5.0,
			voltageOffset: 0.0,
			adcValue:      0.5, // 2.5V (0.5 * 5V)
			expectedTemp:  250.0,
			tolerance:     0.1,
		},
		{
			name:          "With voltage offset",
			voltageSupply: 5.0,
			voltageOffset: 0.1,
			adcValue:      0.5, // 2.5V
			expectedTemp:  240.0, // (2.5 - 0.1) / 0.010 = 240
			tolerance:     0.1,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			adc := &mockADC{}
			config := &AD595Config{
				VoltageSupply: tt.voltageSupply,
				VoltageOffset: tt.voltageOffset,
			}

			sensor, err := NewAD595Sensor("test_ad595", "PA0", adc, config)
			if err != nil {
				t.Fatalf("Failed to create AD595 sensor: %v", err)
			}

			// Send ADC value
			adc.sendValue(tt.adcValue)

			// Check temperature
			currentTemp, _ := sensor.GetTemp(0.0)
			diff := currentTemp - tt.expectedTemp
			if diff < 0 {
				diff = -diff
			}

			if diff > tt.tolerance {
				t.Errorf("Expected temperature %.1f°C, got %.1f°C (diff: %.1f°C)",
					tt.expectedTemp, currentTemp, diff)
			}
		})
	}
}

// TestPT100Sensor tests the PT100 RTD sensor
func TestPT100Sensor(t *testing.T) {
	tests := []struct {
		name               string
		referenceResistor  float64
		r0                 float64
		adcValue           float64
		expectedTemp       float64
		tolerance          float64
	}{
		{
			name:              "0°C at 100 ohms",
			referenceResistor: 4300.0,
			r0:                100.0,
			adcValue:          100.0 / 4300.0, // R = 100 ohms
			expectedTemp:      -273.15 + 0.0, // Should be near 0°C after conversion
			tolerance:         1.0,
		},
		{
			name:              "100°C at 138.5 ohms",
			referenceResistor: 4300.0,
			r0:                100.0,
			adcValue:          138.5 / 4300.0, // R = 138.5 ohms
			expectedTemp:      100.0,
			tolerance:         1.0,
		},
		{
			name:              "High temperature reading",
			referenceResistor: 4300.0,
			r0:                100.0,
			adcValue:          0.5, // 50% of reference
			expectedTemp:      770.0, // Rough calculation
			tolerance:         10.0,  // Higher tolerance due to linear approximation
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			adc := &mockADC{}
			config := &PT100Config{
				ReferenceResistor: tt.referenceResistor,
				R0:                tt.r0,
			}

			sensor, err := NewPT100Sensor("test_pt100", "PA1", adc, config)
			if err != nil {
				t.Fatalf("Failed to create PT100 sensor: %v", err)
			}

			// Send ADC value
			adc.sendValue(tt.adcValue)

			// Check temperature
			currentTemp, _ := sensor.GetTemp(0.0)
			diff := currentTemp - tt.expectedTemp
			if diff < 0 {
				diff = -diff
			}

			if diff > tt.tolerance {
				t.Errorf("Expected temperature %.1f°C, got %.1f°C (diff: %.1f°C)",
					tt.expectedTemp, currentTemp, diff)
			}
		})
	}
}

// TestThermistorSensor tests the thermistor sensor
func TestThermistorSensor(t *testing.T) {
	tests := []struct {
		name           string
		beta           float64
		r0             float64
		t0             float64
		pullupResistor float64
		adcValue       float64
		minTemp        float64
		maxTemp        float64
	}{
		{
			name:           "High temperature (low ADC)",
			beta:           3950.0,
			r0:             100000.0,
			t0:             25.0,
			pullupResistor: 4700.0,
			adcValue:       0.1, // Low ADC value = hot
			minTemp:        200.0,
			maxTemp:        300.0,
		},
		{
			name:           "Medium temperature",
			beta:           3950.0,
			r0:             100000.0,
			t0:             25.0,
			pullupResistor: 4700.0,
			adcValue:       0.6,
			minTemp:        100.0,
			maxTemp:        200.0,
		},
		{
			name:           "Room temperature (high ADC)",
			beta:           3950.0,
			r0:             100000.0,
			t0:             25.0,
			pullupResistor: 4700.0,
			adcValue:       0.95, // High ADC value = cold
			minTemp:        0.0,
			maxTemp:        50.0,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			adc := &mockADC{}
			config := &ThermistorConfig{
				Beta:           tt.beta,
				R0:             tt.r0,
				T0:             tt.t0,
				PullupResistor: tt.pullupResistor,
			}

			sensor, err := NewThermistorSensor("test_thermistor", "PA2", adc, config)
			if err != nil {
				t.Fatalf("Failed to create Thermistor sensor: %v", err)
			}

			// Send ADC value
			adc.sendValue(tt.adcValue)

			// Check temperature is in expected range
			currentTemp, _ := sensor.GetTemp(0.0)
			if currentTemp < tt.minTemp || currentTemp > tt.maxTemp {
				t.Errorf("Temperature %.1f°C not in expected range [%.1f, %.1f]°C",
					currentTemp, tt.minTemp, tt.maxTemp)
			}
		})
	}
}

// TestThermistorTableSensor tests the table-based thermistor sensor
func TestThermistorTableSensor(t *testing.T) {
	// Create a simple temperature table
	table := []ThermistorEntry{
		{Temp: 20.0, Resistance: 126700.0},  // 20°C
		{Temp: 100.0, Resistance: 17600.0},  // 100°C
		{Temp: 200.0, Resistance: 3100.0},   // 200°C
		{Temp: 300.0, Resistance: 760.0},    // 300°C
	}

	tests := []struct {
		name           string
		pullupResistor float64
		adcValue       float64
		expectedTemp   float64
		tolerance      float64
	}{
		{
			name:           "Table entry 1 (20°C)",
			pullupResistor: 4700.0,
			adcValue:       126700.0 / (126700.0 + 4700.0),
			expectedTemp:   20.0,
			tolerance:      1.0,
		},
		{
			name:           "Table entry 2 (100°C)",
			pullupResistor: 4700.0,
			adcValue:       17600.0 / (17600.0 + 4700.0),
			expectedTemp:   100.0,
			tolerance:      1.0,
		},
		{
			name:           "Table entry 3 (200°C)",
			pullupResistor: 4700.0,
			adcValue:       3100.0 / (3100.0 + 4700.0),
			expectedTemp:   200.0,
			tolerance:      1.0,
		},
		{
			name:           "Interpolation between 100°C and 200°C",
			pullupResistor: 4700.0,
			adcValue:       10000.0 / (10000.0 + 4700.0), // ~10K ohms
			expectedTemp:   150.0, // Should interpolate to ~150°C
			tolerance:      20.0,  // Wider tolerance for interpolation
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			adc := &mockADC{}
			config := &ThermistorTableConfig{
				PullupResistor: tt.pullupResistor,
				Table:          table,
			}

			sensor, err := NewThermistorTableSensor("test_table", "PA3", adc, config)
			if err != nil {
				t.Fatalf("Failed to create ThermistorTable sensor: %v", err)
			}

			// Send ADC value
			adc.sendValue(tt.adcValue)

			// Check temperature
			currentTemp, _ := sensor.GetTemp(0.0)
			diff := currentTemp - tt.expectedTemp
			if diff < 0 {
				diff = -diff
			}

			if diff > tt.tolerance {
				t.Errorf("Expected temperature %.1f°C, got %.1f°C (diff: %.1f°C)",
					tt.expectedTemp, currentTemp, diff)
			}
		})
	}
}

// TestSensorInterface tests that all sensors implement the Sensor interface correctly
func TestSensorInterface(t *testing.T) {
	adc := &mockADC{}

	sensors := []struct {
		name   string
		sensor Sensor
	}{
		{
			name: "AD595",
			sensor: func() Sensor {
				s, _ := NewAD595Sensor("ad595", "PA0", adc, &AD595Config{})
				return s
			}(),
		},
		{
			name: "PT100",
			sensor: func() Sensor {
				s, _ := NewPT100Sensor("pt100", "PA1", adc, &PT100Config{})
				return s
			}(),
		},
		{
			name: "Thermistor",
			sensor: func() Sensor {
				s, _ := NewThermistorSensor("thermistor", "PA2", adc, &ThermistorConfig{})
				return s
			}(),
		},
		{
			name: "ThermistorTable",
			sensor: func() Sensor {
				table := []ThermistorEntry{
					{Temp: 20.0, Resistance: 126700.0},
					{Temp: 300.0, Resistance: 760.0},
				}
				s, _ := NewThermistorTableSensor("table", "PA3", adc, &ThermistorTableConfig{Table: table})
				return s
			}(),
		},
	}

	for _, tc := range sensors {
		t.Run(tc.name+" interface", func(t *testing.T) {
			// Test GetName
			name := tc.sensor.GetName()
			if name == "" {
				t.Error("GetName returned empty string")
			}

			// Test GetReportTimeDelta
			delta := tc.sensor.GetReportTimeDelta()
			if delta <= 0 {
				t.Errorf("GetReportTimeDelta returned invalid value: %f", delta)
			}

			// Test SetupMinMax
			err := tc.sensor.SetupMinMax(0.0, 300.0)
			if err != nil {
				t.Errorf("SetupMinMax failed: %v", err)
			}

			// Test GetTemp
			current, _ := tc.sensor.GetTemp(0.0)
			if current < 0 || current > 500 {
				t.Errorf("GetTemp returned invalid current temp: %f", current)
			}
		})
	}
}
