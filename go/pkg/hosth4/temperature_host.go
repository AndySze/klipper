// Temperature Host - port of klippy/extras/temperature_host.py
//
// Support for Raspberry Pi / host temperature sensor
//
// Copyright (C) 2020 Al Crate <al3ph@users.noreply.github.com>
// Copyright (C) 2025 Go port
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"fmt"
	"log"
	"os"
	"strconv"
	"strings"
	"sync"
	"time"
)

const (
	hostReportTime      = 1.0
	defaultThermalPath  = "/sys/class/thermal/thermal_zone0/temp"
)

// TemperatureHost reads temperature from the host's thermal zone.
type TemperatureHost struct {
	rt          *runtime
	name        string
	path        string
	temp        float64
	minTemp     float64
	maxTemp     float64
	callback    func(readTime, temp float64)
	stopChan    chan struct{}
	running     bool
	mu          sync.RWMutex
}

// TemperatureHostConfig holds configuration for host temperature sensor.
type TemperatureHostConfig struct {
	Name       string // Sensor name
	SensorPath string // Path to thermal zone file (default: /sys/class/thermal/thermal_zone0/temp)
}

// DefaultTemperatureHostConfig returns default host temperature configuration.
func DefaultTemperatureHostConfig() TemperatureHostConfig {
	return TemperatureHostConfig{
		SensorPath: defaultThermalPath,
	}
}

// newTemperatureHost creates a new host temperature sensor.
func newTemperatureHost(rt *runtime, cfg TemperatureHostConfig) (*TemperatureHost, error) {
	path := cfg.SensorPath
	if path == "" {
		path = defaultThermalPath
	}

	// Check if file exists
	if _, err := os.Stat(path); os.IsNotExist(err) {
		return nil, fmt.Errorf("unable to open temperature file '%s'", path)
	}

	th := &TemperatureHost{
		rt:       rt,
		name:     cfg.Name,
		path:     path,
		stopChan: make(chan struct{}),
	}

	log.Printf("temperature_host: initialized '%s' with sensor path '%s'", cfg.Name, path)
	return th, nil
}

// Start begins periodic temperature sampling.
func (th *TemperatureHost) Start() {
	th.mu.Lock()
	if th.running {
		th.mu.Unlock()
		return
	}
	th.running = true
	th.mu.Unlock()

	go th.sampleLoop()
}

// Stop stops periodic temperature sampling.
func (th *TemperatureHost) Stop() {
	th.mu.Lock()
	if !th.running {
		th.mu.Unlock()
		return
	}
	th.running = false
	th.mu.Unlock()

	close(th.stopChan)
}

// sampleLoop periodically reads temperature from the host.
func (th *TemperatureHost) sampleLoop() {
	ticker := time.NewTicker(time.Duration(hostReportTime * float64(time.Second)))
	defer ticker.Stop()

	for {
		select {
		case <-th.stopChan:
			return
		case <-ticker.C:
			th.sampleTemperature()
		}
	}
}

// sampleTemperature reads the current temperature from the host.
func (th *TemperatureHost) sampleTemperature() {
	data, err := os.ReadFile(th.path)
	if err != nil {
		log.Printf("temperature_host: error reading data: %v", err)
		return
	}

	tempStr := strings.TrimSpace(string(data))
	tempMilliC, err := strconv.ParseFloat(tempStr, 64)
	if err != nil {
		log.Printf("temperature_host: error parsing temperature: %v", err)
		return
	}

	temp := tempMilliC / 1000.0

	th.mu.Lock()
	th.temp = temp
	minTemp := th.minTemp
	maxTemp := th.maxTemp
	cb := th.callback
	th.mu.Unlock()

	// Check temperature limits - just log warnings, shutdown is handled externally
	if minTemp > 0 && temp < minTemp {
		log.Printf("WARNING: HOST temperature %.1f below minimum temperature of %.1f", temp, minTemp)
	}
	if maxTemp > 0 && temp > maxTemp {
		log.Printf("WARNING: HOST temperature %.1f above maximum temperature of %.1f", temp, maxTemp)
	}

	// Call callback if registered
	if cb != nil {
		// Use current time as read time
		measuredTime := float64(time.Now().UnixNano()) / 1e9
		cb(measuredTime, temp)
	}
}

// SetupMinMax sets the temperature limits.
func (th *TemperatureHost) SetupMinMax(minTemp, maxTemp float64) {
	th.mu.Lock()
	defer th.mu.Unlock()
	th.minTemp = minTemp
	th.maxTemp = maxTemp
}

// SetupCallback registers a temperature callback.
func (th *TemperatureHost) SetupCallback(cb func(readTime, temp float64)) {
	th.mu.Lock()
	defer th.mu.Unlock()
	th.callback = cb
}

// GetReportTimeDelta returns the report time interval.
func (th *TemperatureHost) GetReportTimeDelta() float64 {
	return hostReportTime
}

// GetTemp returns the last temperature reading.
func (th *TemperatureHost) GetTemp(eventtime float64) (float64, float64) {
	th.mu.RLock()
	defer th.mu.RUnlock()
	return th.temp, 0
}

// GetStatus returns the sensor status.
func (th *TemperatureHost) GetStatus() map[string]any {
	th.mu.RLock()
	defer th.mu.RUnlock()

	return map[string]any{
		"temperature": round2(th.temp),
	}
}

// GetName returns the sensor name.
func (th *TemperatureHost) GetName() string {
	return th.name
}
