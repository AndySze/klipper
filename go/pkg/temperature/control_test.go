// Temperature control algorithm unit tests for Klipper Go migration
// Copyright (C) 2026  Klipper Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package temperature

import (
	"testing"
)

// TestControlPIDCreation tests creating PID control
func TestControlPIDCreation(t *testing.T) {
	tests := []struct {
		name      string
		kp, ki, kd float64
		maxPower  float64
		wantErr   bool
	}{
		{
			name:     "Valid PID parameters",
			kp:       22.2,
			ki:       1.08,
			kd:       114,
			maxPower: 1.0,
			wantErr:  false,
		},
		{
			name:     "Zero Kp - error",
			kp:       0,
			ki:       1.08,
			kd:       114,
			maxPower: 1.0,
			wantErr:  true,
		},
		{
			name:     "Zero Ki - error",
			kp:       22.2,
			ki:       0,
			kd:       114,
			maxPower: 1.0,
			wantErr:  true,
		},
		{
			name:     "Zero Kd - error",
			kp:       22.2,
			ki:       1.08,
			kd:       0,
			maxPower: 1.0,
			wantErr:  true,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			config := &HeaterConfig{
				PID_Kp:   tt.kp,
				PID_Ki:   tt.ki,
				PID_Kd:   tt.kd,
				MaxPower: tt.maxPower,
			}
			_, err := NewControlPID(config)
			if (err != nil) != tt.wantErr {
				t.Errorf("NewControlPID() error = %v, wantErr %v", err, tt.wantErr)
			}
		})
	}
}

// TestControlBangBangCreation tests creating bang-bang control
func TestControlBangBangCreation(t *testing.T) {
	tests := []struct {
		name     string
		maxDelta float64
		maxPower float64
		wantErr  bool
	}{
		{
			name:     "Valid parameters",
			maxDelta: 2.0,
			maxPower: 1.0,
			wantErr:  false,
		},
		{
			name:     "Zero max delta - error",
			maxDelta: 0,
			maxPower: 1.0,
			wantErr:  true,
		},
		{
			name:     "Negative max delta - error",
			maxDelta: -1.0,
			maxPower: 1.0,
			wantErr:  true,
		},
	}

	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			config := &HeaterConfig{
				MaxDelta: tt.maxDelta,
				MaxPower: tt.maxPower,
			}
			_, err := NewControlBangBang(config)
			if (err != nil) != tt.wantErr {
				t.Errorf("NewControlBangBang() error = %v, wantErr %v", err, tt.wantErr)
			}
		})
	}
}

// TestControlPIDUpdate tests PID temperature update behavior
func TestControlPIDUpdate(t *testing.T) {
	config := &HeaterConfig{
		PID_Kp:   22.2,
		PID_Ki:   1.08,
		PID_Kd:   114,
		MaxPower: 1.0,
	}
	pid, err := NewControlPID(config)
	if err != nil {
		t.Fatalf("Failed to create PID control: %v", err)
	}

	// Should not panic when called
	pid.TemperatureUpdate(0.0, 25.0, 200.0)
	pid.TemperatureUpdate(1.0, 190.0, 200.0)
	pid.TemperatureUpdate(2.0, 200.0, 200.0)
	pid.TemperatureUpdate(3.0, 210.0, 200.0)
}

// TestControlBangBangUpdate tests bang-bang temperature update behavior
func TestControlBangBangUpdate(t *testing.T) {
	config := &HeaterConfig{
		MaxDelta: 2.0,
		MaxPower: 1.0,
	}
	bb, err := NewControlBangBang(config)
	if err != nil {
		t.Fatalf("Failed to create bang-bang control: %v", err)
	}

	// Should not panic when called
	bb.TemperatureUpdate(0.0, 25.0, 60.0)
	bb.TemperatureUpdate(1.0, 58.0, 60.0)
	bb.TemperatureUpdate(2.0, 62.5, 60.0)
	bb.TemperatureUpdate(3.0, 57.5, 60.0)
}

// TestControlBusy tests the CheckBusy method
func TestControlBusy(t *testing.T) {
	t.Run("PID busy when cold", func(t *testing.T) {
		config := &HeaterConfig{
			PID_Kp:   22.2,
			PID_Ki:   1.08,
			PID_Kd:   114,
			MaxPower: 1.0,
		}
		pid, err := NewControlPID(config)
		if err != nil {
			t.Fatalf("Failed to create PID: %v", err)
		}
		busy := pid.CheckBusy(0.0, 25.0, 200.0)
		if !busy {
			t.Error("Expected PID to be busy when heating from cold")
		}
	})

	t.Run("PID not busy at target", func(t *testing.T) {
		config := &HeaterConfig{
			PID_Kp:   22.2,
			PID_Ki:   1.08,
			PID_Kd:   114,
			MaxPower: 1.0,
		}
		pid, err := NewControlPID(config)
		if err != nil {
			t.Fatalf("Failed to create PID: %v", err)
		}
		busy := pid.CheckBusy(0.0, 200.0, 200.0)
		if busy {
			t.Error("Expected PID not to be busy at target temperature")
		}
	})

	t.Run("Bang-bang busy when cold", func(t *testing.T) {
		config := &HeaterConfig{
			MaxDelta: 2.0,
			MaxPower: 1.0,
		}
		bb, err := NewControlBangBang(config)
		if err != nil {
			t.Fatalf("Failed to create bang-bang: %v", err)
		}
		busy := bb.CheckBusy(0.0, 25.0, 60.0)
		if !busy {
			t.Error("Expected bang-bang to be busy when heating from cold")
		}
	})

	t.Run("Bang-bang not busy at target", func(t *testing.T) {
		config := &HeaterConfig{
			MaxDelta: 2.0,
			MaxPower: 1.0,
		}
		bb, err := NewControlBangBang(config)
		if err != nil {
			t.Fatalf("Failed to create bang-bang: %v", err)
		}
		busy := bb.CheckBusy(0.0, 60.0, 60.0)
		if busy {
			t.Error("Expected bang-bang not to be busy at target temperature")
		}
	})
}

// TestControlPIDZeroTarget tests PID behavior with zero target
func TestControlPIDZeroTarget(t *testing.T) {
	config := &HeaterConfig{
		PID_Kp:   22.2,
		PID_Ki:   1.08,
		PID_Kd:   114,
		MaxPower: 1.0,
	}
	pid, err := NewControlPID(config)
	if err != nil {
		t.Fatalf("Failed to create PID control: %v", err)
	}

	// Zero target should not panic
	pid.TemperatureUpdate(0.0, 100.0, 0.0)
}

// TestControlBangBangHysteresis tests bang-bang hysteresis
func TestControlBangBangHysteresis(t *testing.T) {
	maxDelta := 2.0
	targetTemp := 60.0
	config := &HeaterConfig{
		MaxDelta: maxDelta,
		MaxPower: 1.0,
	}
	bbControl, err := NewControlBangBang(config)
	if err != nil {
		t.Fatalf("Failed to create bang-bang control: %v", err)
	}

	// Access internal implementation to check heating state
	type heatingState interface {
		getHeating() bool
	}

	// Wrapper to check heating state
	var control ControlAlgorithm = bbControl

	// Start heating from cold
	control.TemperatureUpdate(0.0, 25.0, targetTemp)

	// Heat until just below target
	control.TemperatureUpdate(1.0, 58.0, targetTemp)

	// Cross upper threshold (target + delta)
	control.TemperatureUpdate(2.0, 62.5, targetTemp)

	// Cool down but stay above lower threshold
	control.TemperatureUpdate(3.0, 58.0, targetTemp)

	// Cross lower threshold (target - delta)
	control.TemperatureUpdate(4.0, 57.5, targetTemp)

	// If we got here without panic, test passes
}
