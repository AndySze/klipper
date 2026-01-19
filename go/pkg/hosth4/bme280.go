// BME280 - port of klippy/extras/bme280.py
//
// Support for i2c based temperature/humidity/pressure sensors
//
// Copyright (C) 2020 Eric Callahan <arksine.code@gmail.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"sync"
)

// BME280 registers.
const (
	bme280RegReset     = 0xE0
	bme280RegCtrlHum   = 0xF2
	bme280RegStatus    = 0xF3
	bme280RegCtrlMeas  = 0xF4
	bme280RegConfig    = 0xF5
	bme280RegPressureMSB = 0xF7
	bme280RegTempMSB   = 0xFA
	bme280RegHumMSB    = 0xFD
	bme280RegCal1      = 0x88
	bme280RegCal2      = 0xE1
	bme280ChipIDReg    = 0xD0
)

// BME280 chip IDs.
const (
	bmp280ChipID = 0x58
	bme280ChipID = 0x60
	bme680ChipID = 0x61
	bmp180ChipID = 0x55
	bmp388ChipID = 0x50
)

// Default I2C address.
const bme280ChipAddr = 0x76

// BME280 constants.
const (
	bme280ReportTime   = 0.8
	bme280ResetValue   = 0xB6
	bme280StatusMeasuring = 1 << 3
)

// BME chip types.
var bmeChipNames = map[byte]string{
	bmp280ChipID: "BMP280",
	bme280ChipID: "BME280",
	bme680ChipID: "BME680",
	bmp180ChipID: "BMP180",
	bmp388ChipID: "BMP388",
}

// BME280 represents a BME280/BMP280/BME680 environmental sensor.
type BME280 struct {
	rt           *runtime
	name         string
	chipType     string
	i2cAddr      int
	reportTime   float64
	temperature  float64
	humidity     float64
	pressure     float64
	lastReadTime float64
	calibData    bme280CalibData
	callback     func(readTime, temp, humidity, pressure float64)
	mu           sync.Mutex
}

// bme280CalibData holds calibration data for BME280.
type bme280CalibData struct {
	digT1 uint16
	digT2 int16
	digT3 int16
	digP1 uint16
	digP2 int16
	digP3 int16
	digP4 int16
	digP5 int16
	digP6 int16
	digP7 int16
	digP8 int16
	digP9 int16
	digH1 uint8
	digH2 int16
	digH3 uint8
	digH4 int16
	digH5 int16
	digH6 int8
}

// BME280Config holds configuration for BME280.
type BME280Config struct {
	Name       string
	I2CAddr    int
	ReportTime float64
}

// DefaultBME280Config returns default BME280 configuration.
func DefaultBME280Config() BME280Config {
	return BME280Config{
		I2CAddr:    bme280ChipAddr,
		ReportTime: bme280ReportTime,
	}
}

// newBME280 creates a new BME280 sensor.
func newBME280(rt *runtime, cfg BME280Config) (*BME280, error) {
	if cfg.Name == "" {
		return nil, fmt.Errorf("bme280: name is required")
	}
	if cfg.I2CAddr <= 0 {
		cfg.I2CAddr = bme280ChipAddr
	}
	if cfg.ReportTime <= 0 {
		cfg.ReportTime = bme280ReportTime
	}

	bme := &BME280{
		rt:         rt,
		name:       cfg.Name,
		chipType:   "BME280",
		i2cAddr:    cfg.I2CAddr,
		reportTime: cfg.ReportTime,
	}

	log.Printf("bme280: initialized '%s' addr=0x%02x report_time=%.2f",
		cfg.Name, cfg.I2CAddr, cfg.ReportTime)
	return bme, nil
}

// GetName returns the sensor name.
func (bme *BME280) GetName() string {
	return bme.name
}

// GetChipType returns the chip type.
func (bme *BME280) GetChipType() string {
	return bme.chipType
}

// SetChipType sets the chip type based on detected chip ID.
func (bme *BME280) SetChipType(chipID byte) {
	bme.mu.Lock()
	defer bme.mu.Unlock()

	if name, ok := bmeChipNames[chipID]; ok {
		bme.chipType = name
	}
}

// SetCallback sets the measurement callback.
func (bme *BME280) SetCallback(cb func(readTime, temp, humidity, pressure float64)) {
	bme.mu.Lock()
	defer bme.mu.Unlock()
	bme.callback = cb
}

// UpdateMeasurement updates the measurement values.
func (bme *BME280) UpdateMeasurement(readTime, temp, humidity, pressure float64) {
	bme.mu.Lock()
	defer bme.mu.Unlock()

	bme.temperature = temp
	bme.humidity = humidity
	bme.pressure = pressure
	bme.lastReadTime = readTime

	if bme.callback != nil {
		bme.callback(readTime, temp, humidity, pressure)
	}
}

// GetTemperature returns the last temperature reading.
func (bme *BME280) GetTemperature() float64 {
	bme.mu.Lock()
	defer bme.mu.Unlock()
	return bme.temperature
}

// GetHumidity returns the last humidity reading.
func (bme *BME280) GetHumidity() float64 {
	bme.mu.Lock()
	defer bme.mu.Unlock()
	return bme.humidity
}

// GetPressure returns the last pressure reading.
func (bme *BME280) GetPressure() float64 {
	bme.mu.Lock()
	defer bme.mu.Unlock()
	return bme.pressure
}

// CompensateTemperature compensates raw temperature data.
func (bme *BME280) CompensateTemperature(adcT int32) (float64, int32) {
	bme.mu.Lock()
	cal := bme.calibData
	bme.mu.Unlock()

	var1 := (float64(adcT)/16384.0 - float64(cal.digT1)/1024.0) * float64(cal.digT2)
	var2 := ((float64(adcT)/131072.0 - float64(cal.digT1)/8192.0) *
		(float64(adcT)/131072.0 - float64(cal.digT1)/8192.0)) * float64(cal.digT3)
	tFine := int32(var1 + var2)
	temp := (var1 + var2) / 5120.0
	return temp, tFine
}

// CompensatePressure compensates raw pressure data.
func (bme *BME280) CompensatePressure(adcP int32, tFine int32) float64 {
	bme.mu.Lock()
	cal := bme.calibData
	bme.mu.Unlock()

	var1 := float64(tFine)/2.0 - 64000.0
	var2 := var1 * var1 * float64(cal.digP6) / 32768.0
	var2 = var2 + var1*float64(cal.digP5)*2.0
	var2 = var2/4.0 + float64(cal.digP4)*65536.0
	var1 = (float64(cal.digP3)*var1*var1/524288.0 + float64(cal.digP2)*var1) / 524288.0
	var1 = (1.0 + var1/32768.0) * float64(cal.digP1)

	if var1 == 0 {
		return 0
	}

	pressure := 1048576.0 - float64(adcP)
	pressure = (pressure - var2/4096.0) * 6250.0 / var1
	var1 = float64(cal.digP9) * pressure * pressure / 2147483648.0
	var2 = pressure * float64(cal.digP8) / 32768.0
	pressure = pressure + (var1+var2+float64(cal.digP7))/16.0

	return pressure / 100.0 // Convert to hPa
}

// CompensateHumidity compensates raw humidity data.
func (bme *BME280) CompensateHumidity(adcH int32, tFine int32) float64 {
	bme.mu.Lock()
	cal := bme.calibData
	bme.mu.Unlock()

	h := float64(tFine) - 76800.0
	if h == 0 {
		return 0
	}

	h = (float64(adcH) - (float64(cal.digH4)*64.0 + float64(cal.digH5)/16384.0*h)) *
		(float64(cal.digH2) / 65536.0 * (1.0 + float64(cal.digH6)/67108864.0*h*
			(1.0+float64(cal.digH3)/67108864.0*h)))
	h = h * (1.0 - float64(cal.digH1)*h/524288.0)

	if h > 100.0 {
		h = 100.0
	} else if h < 0.0 {
		h = 0.0
	}

	return h
}

// GetStatus returns the sensor status.
func (bme *BME280) GetStatus() map[string]any {
	bme.mu.Lock()
	defer bme.mu.Unlock()

	return map[string]any{
		"temperature": bme.temperature,
		"humidity":    bme.humidity,
		"pressure":    bme.pressure,
	}
}

// GetMinReportTime returns the minimum report time.
func (bme *BME280) GetMinReportTime() float64 {
	return bme.reportTime
}
