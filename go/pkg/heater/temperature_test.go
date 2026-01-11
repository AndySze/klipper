package heater

import (
	"math"
	"testing"
)

func TestDefaultThermistorParams(t *testing.T) {
	params := DefaultThermistorParams()

	if params.Beta <= 0 {
		t.Error("Beta should be positive")
	}
	if params.R0 <= 0 {
		t.Error("R0 should be positive")
	}
	if params.T0 <= 0 {
		t.Error("T0 should be positive")
	}
	if params.Pullup <= 0 {
		t.Error("Pullup should be positive")
	}
}

func TestNewTemperatureSensor(t *testing.T) {
	cfg := DefaultSensorConfig()
	sensor := NewTemperatureSensor(cfg)

	if sensor == nil {
		t.Fatal("NewTemperatureSensor returned nil")
	}

	if sensor.GetName() != cfg.Name {
		t.Errorf("Name = %s, want %s", sensor.GetName(), cfg.Name)
	}
}

func TestUpdateADC(t *testing.T) {
	cfg := DefaultSensorConfig()
	sensor := NewTemperatureSensor(cfg)

	// Room temperature (~25°C) for a 100K NTC thermistor
	// At 25°C, resistance = R0 = 100K
	// ADC = Pullup * ADCmax / (Pullup + R)
	// With 4.7K pullup and 100K thermistor:
	// ADC = 4700 * 4095 / (4700 + 100000) ≈ 184

	adc := sensor.TempToADC(25.0)
	err := sensor.UpdateADC(adc, 0.1)
	if err != nil {
		t.Fatalf("UpdateADC failed: %v", err)
	}

	temp := sensor.GetTemperature()
	if math.Abs(temp-25.0) > 1.0 {
		t.Errorf("Temperature at 25°C ADC = %.2f, want ~25.0", temp)
	}
}

func TestUpdateADCHighTemp(t *testing.T) {
	cfg := DefaultSensorConfig()
	sensor := NewTemperatureSensor(cfg)

	// At higher temperature, resistance decreases, ADC increases
	adc := sensor.TempToADC(200.0)
	err := sensor.UpdateADC(adc, 0.1)
	if err != nil {
		t.Fatalf("UpdateADC failed: %v", err)
	}

	temp := sensor.GetTemperature()
	if math.Abs(temp-200.0) > 2.0 {
		t.Errorf("Temperature at 200°C ADC = %.2f, want ~200.0", temp)
	}
}

func TestSensorFaultOpen(t *testing.T) {
	cfg := DefaultSensorConfig()
	cfg.MaxFaults = 2
	sensor := NewTemperatureSensor(cfg)

	// ADC at max indicates open circuit
	err := sensor.UpdateADC(cfg.ADCMax, 0.1)
	if err != nil {
		t.Error("First fault should not return error")
	}

	err = sensor.UpdateADC(cfg.ADCMax, 0.2)
	if err != ErrSensorOpen {
		t.Errorf("Expected ErrSensorOpen, got %v", err)
	}

	if !sensor.IsFaulted() {
		t.Error("Sensor should be faulted")
	}
}

func TestSensorFaultShort(t *testing.T) {
	cfg := DefaultSensorConfig()
	cfg.MaxFaults = 2
	sensor := NewTemperatureSensor(cfg)

	// ADC at 0 indicates short circuit
	err := sensor.UpdateADC(0, 0.1)
	if err != nil {
		t.Error("First fault should not return error")
	}

	err = sensor.UpdateADC(0, 0.2)
	if err != ErrSensorShort {
		t.Errorf("Expected ErrSensorShort, got %v", err)
	}
}

func TestSensorReset(t *testing.T) {
	cfg := DefaultSensorConfig()
	sensor := NewTemperatureSensor(cfg)

	adc := sensor.TempToADC(50.0)
	sensor.UpdateADC(adc, 0.1)

	if sensor.GetTemperature() == 0 {
		t.Error("Temperature should be non-zero before reset")
	}

	sensor.Reset()

	if sensor.GetTemperature() != 0 {
		t.Error("Temperature should be zero after reset")
	}
}

func TestGetStatus(t *testing.T) {
	cfg := DefaultSensorConfig()
	sensor := NewTemperatureSensor(cfg)

	adc := sensor.TempToADC(100.0)
	sensor.UpdateADC(adc, 0.5)

	status := sensor.GetStatus()

	if status.Name != cfg.Name {
		t.Errorf("Status.Name = %s, want %s", status.Name, cfg.Name)
	}

	if math.Abs(status.Temperature-100.0) > 2.0 {
		t.Errorf("Status.Temperature = %.2f, want ~100.0", status.Temperature)
	}

	if status.SampleTime != 0.5 {
		t.Errorf("Status.SampleTime = %f, want 0.5", status.SampleTime)
	}

	if status.IsFaulted {
		t.Error("Status should not show faulted")
	}
}

func TestAveraging(t *testing.T) {
	cfg := DefaultSensorConfig()
	cfg.AvgCount = 4
	sensor := NewTemperatureSensor(cfg)

	// Feed samples that should average to ~50°C
	temps := []float64{45, 50, 55, 50}
	for i, temp := range temps {
		adc := sensor.TempToADC(temp)
		sensor.UpdateADC(adc, float64(i)*0.1)
	}

	avgTemp := sensor.GetTemperature()
	expectedAvg := (45.0 + 50.0 + 55.0 + 50.0) / 4.0

	if math.Abs(avgTemp-expectedAvg) > 2.0 {
		t.Errorf("Average temperature = %.2f, want ~%.2f", avgTemp, expectedAvg)
	}
}

func TestTempToADCRoundtrip(t *testing.T) {
	cfg := DefaultSensorConfig()
	sensor := NewTemperatureSensor(cfg)

	testTemps := []float64{25, 50, 100, 150, 200, 250}

	for _, temp := range testTemps {
		adc := sensor.TempToADC(temp)
		sensor.UpdateADC(adc, 0.1)
		gotTemp := sensor.GetTemperature()

		if math.Abs(gotTemp-temp) > 1.0 {
			t.Errorf("Roundtrip for %.1f°C: got %.2f°C", temp, gotTemp)
		}
		sensor.Reset()
	}
}
