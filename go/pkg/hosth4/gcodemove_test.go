package hosth4

import (
	"math"
	"testing"
)

func newTestGcodeMove() *gcodeMove {
	// Create a minimal toolhead for testing
	th := &toolhead{
		commandedPos: []float64{0, 0, 0, 0},
	}
	return newGCodeMove(th, 1.0)
}

func TestNewGCodeMove(t *testing.T) {
	gm := newTestGcodeMove()
	if gm == nil {
		t.Fatal("newGCodeMove returned nil")
	}

	// Check defaults
	if !gm.absoluteCoord {
		t.Error("expected absoluteCoord to be true")
	}
	if !gm.absoluteExtrude {
		t.Error("expected absoluteExtrude to be true")
	}
	if gm.speed != 25.0 {
		t.Errorf("expected speed 25.0, got %v", gm.speed)
	}
	if gm.speedFactor != 1.0/60.0 {
		t.Errorf("expected speedFactor 1/60, got %v", gm.speedFactor)
	}
	if gm.extrudeFactor != 1.0 {
		t.Errorf("expected extrudeFactor 1.0, got %v", gm.extrudeFactor)
	}
	if len(gm.homingPosition) != 4 {
		t.Errorf("expected homingPosition length 4, got %d", len(gm.homingPosition))
	}
}

func TestG90G91(t *testing.T) {
	gm := newTestGcodeMove()

	gm.cmdG91()
	if gm.absoluteCoord {
		t.Error("G91 should set relative coordinates")
	}

	gm.cmdG90()
	if !gm.absoluteCoord {
		t.Error("G90 should set absolute coordinates")
	}
}

func TestM82M83(t *testing.T) {
	gm := newTestGcodeMove()

	gm.cmdM83()
	if gm.absoluteExtrude {
		t.Error("M83 should set relative extrusion")
	}

	gm.cmdM82()
	if !gm.absoluteExtrude {
		t.Error("M82 should set absolute extrusion")
	}
}

func TestG20G21(t *testing.T) {
	gm := newTestGcodeMove()

	// G20 should return error
	err := gm.cmdG20()
	if err == nil {
		t.Error("G20 should return error (inches not supported)")
	}

	// G21 should not error (no-op)
	gm.cmdG21()
}

func TestM220(t *testing.T) {
	gm := newTestGcodeMove()

	// Set speed to 50%
	err := gm.cmdM220(map[string]string{"S": "50"})
	if err != nil {
		t.Fatalf("M220 S50 failed: %v", err)
	}

	expectedFactor := 50.0 / (60.0 * 100.0)
	if math.Abs(gm.speedFactor-expectedFactor) > 1e-9 {
		t.Errorf("expected speedFactor %v, got %v", expectedFactor, gm.speedFactor)
	}

	// Invalid S value
	err = gm.cmdM220(map[string]string{"S": "0"})
	if err == nil {
		t.Error("M220 S0 should fail")
	}

	err = gm.cmdM220(map[string]string{"S": "-100"})
	if err == nil {
		t.Error("M220 S-100 should fail")
	}
}

func TestM221(t *testing.T) {
	gm := newTestGcodeMove()

	// Set extrude factor to 95%
	err := gm.cmdM221(map[string]string{"S": "95"})
	if err != nil {
		t.Fatalf("M221 S95 failed: %v", err)
	}

	if math.Abs(gm.extrudeFactor-0.95) > 1e-9 {
		t.Errorf("expected extrudeFactor 0.95, got %v", gm.extrudeFactor)
	}

	// Invalid S value
	err = gm.cmdM221(map[string]string{"S": "0"})
	if err == nil {
		t.Error("M221 S0 should fail")
	}
}

func TestG92(t *testing.T) {
	gm := newTestGcodeMove()

	// Set position to X=10, Y=20
	gm.lastPosition = []float64{100, 200, 300, 400}
	err := gm.cmdG92(map[string]string{"X": "10", "Y": "20"})
	if err != nil {
		t.Fatalf("G92 failed: %v", err)
	}

	// base_position = last_position - v
	// So base_position[0] = 100 - 10 = 90
	if gm.basePosition[0] != 90 {
		t.Errorf("expected basePosition[0] = 90, got %v", gm.basePosition[0])
	}
	if gm.basePosition[1] != 180 {
		t.Errorf("expected basePosition[1] = 180, got %v", gm.basePosition[1])
	}
}

func TestGetGcodePosition(t *testing.T) {
	gm := newTestGcodeMove()

	gm.lastPosition = []float64{100, 200, 50, 10}
	gm.basePosition = []float64{10, 20, 0, 0}

	pos := gm.getGcodePosition()
	if pos[0] != 90 {
		t.Errorf("expected X=90, got %v", pos[0])
	}
	if pos[1] != 180 {
		t.Errorf("expected Y=180, got %v", pos[1])
	}
	if pos[2] != 50 {
		t.Errorf("expected Z=50, got %v", pos[2])
	}
	if pos[3] != 10 {
		t.Errorf("expected E=10, got %v", pos[3])
	}
}

func TestM114(t *testing.T) {
	gm := newTestGcodeMove()

	gm.lastPosition = []float64{10, 20, 30, 5}
	gm.basePosition = []float64{0, 0, 0, 0}

	result := gm.cmdM114()
	expected := "X:10.000 Y:20.000 Z:30.000 E:5.000"
	if result != expected {
		t.Errorf("expected %q, got %q", expected, result)
	}
}

func TestSaveRestoreState(t *testing.T) {
	gm := newTestGcodeMove()

	// Set up state
	gm.absoluteCoord = false
	gm.absoluteExtrude = false
	gm.speed = 50.0
	gm.speedFactor = 0.5
	gm.extrudeFactor = 0.9
	gm.lastPosition = []float64{10, 20, 30, 5}
	gm.basePosition = []float64{1, 2, 3, 4}
	gm.homingPosition = []float64{0.1, 0.2, 0.3, 0}

	// Save state
	gm.saveState("test")

	// Change state
	gm.absoluteCoord = true
	gm.absoluteExtrude = true
	gm.speed = 100.0
	gm.speedFactor = 1.0
	gm.extrudeFactor = 1.0
	gm.lastPosition = []float64{100, 200, 300, 50}
	gm.basePosition = []float64{10, 20, 30, 40}
	gm.homingPosition = []float64{1, 2, 3, 0}

	// Restore state (without move)
	err := gm.restoreState("test", false, 0)
	if err != nil {
		t.Fatalf("restoreState failed: %v", err)
	}

	if gm.absoluteCoord {
		t.Error("absoluteCoord should be false after restore")
	}
	if gm.absoluteExtrude {
		t.Error("absoluteExtrude should be false after restore")
	}
	if gm.speed != 50.0 {
		t.Errorf("expected speed 50.0, got %v", gm.speed)
	}
	if gm.speedFactor != 0.5 {
		t.Errorf("expected speedFactor 0.5, got %v", gm.speedFactor)
	}
	if gm.extrudeFactor != 0.9 {
		t.Errorf("expected extrudeFactor 0.9, got %v", gm.extrudeFactor)
	}

	// Restore non-existent state should fail
	err = gm.restoreState("nonexistent", false, 0)
	if err == nil {
		t.Error("restoreState with unknown name should fail")
	}
}

func TestSetGcodeOffset(t *testing.T) {
	gm := newTestGcodeMove()

	gm.lastPosition = []float64{100, 200, 50, 10}
	gm.basePosition = []float64{0, 0, 0, 0}
	gm.homingPosition = []float64{0, 0, 0, 0}

	// Set offset X=5, Y=10
	err := gm.cmdSetGcodeOffset(map[string]string{"X": "5", "Y": "10"})
	if err != nil {
		t.Fatalf("SET_GCODE_OFFSET failed: %v", err)
	}

	if gm.homingPosition[0] != 5 {
		t.Errorf("expected homingPosition[0] = 5, got %v", gm.homingPosition[0])
	}
	if gm.homingPosition[1] != 10 {
		t.Errorf("expected homingPosition[1] = 10, got %v", gm.homingPosition[1])
	}
	if gm.basePosition[0] != 5 {
		t.Errorf("expected basePosition[0] = 5, got %v", gm.basePosition[0])
	}
	if gm.basePosition[1] != 10 {
		t.Errorf("expected basePosition[1] = 10, got %v", gm.basePosition[1])
	}
}

func TestSetGcodeOffsetAdjust(t *testing.T) {
	gm := newTestGcodeMove()

	gm.homingPosition = []float64{5, 10, 2, 0}
	gm.basePosition = []float64{5, 10, 2, 0}
	gm.lastPosition = []float64{100, 200, 50, 10}

	// Adjust Z by +0.5
	err := gm.cmdSetGcodeOffset(map[string]string{"Z_ADJUST": "0.5"})
	if err != nil {
		t.Fatalf("SET_GCODE_OFFSET Z_ADJUST failed: %v", err)
	}

	// New Z offset = current homing_position[2] + 0.5 = 2 + 0.5 = 2.5
	if gm.homingPosition[2] != 2.5 {
		t.Errorf("expected homingPosition[2] = 2.5, got %v", gm.homingPosition[2])
	}
}

func TestGetStatus(t *testing.T) {
	gm := newTestGcodeMove()

	gm.absoluteCoord = true
	gm.absoluteExtrude = false
	gm.speed = 25.0
	gm.speedFactor = 1.0 / 60.0
	gm.extrudeFactor = 0.95
	gm.lastPosition = []float64{10, 20, 30, 5}
	gm.basePosition = []float64{0, 0, 0, 0}
	gm.homingPosition = []float64{0.1, 0.2, 0.3, 0}

	status := gm.GetStatus()

	if status["absolute_coordinates"] != true {
		t.Error("expected absolute_coordinates = true")
	}
	if status["absolute_extrude"] != false {
		t.Error("expected absolute_extrude = false")
	}
	if status["extrude_factor"] != 0.95 {
		t.Errorf("expected extrude_factor = 0.95, got %v", status["extrude_factor"])
	}

	// speed_factor = speedFactor * 60 = (1/60) * 60 = 1.0
	sf, ok := status["speed_factor"].(float64)
	if !ok || math.Abs(sf-1.0) > 1e-9 {
		t.Errorf("expected speed_factor = 1.0, got %v", status["speed_factor"])
	}

	// speed = speed / speedFactor = 25 / (1/60) = 1500
	sp, ok := status["speed"].(float64)
	if !ok || math.Abs(sp-1500.0) > 1e-9 {
		t.Errorf("expected speed = 1500, got %v", status["speed"])
	}

	// Check axis_map
	axisMap, ok := status["axis_map"].(map[string]int)
	if !ok {
		t.Error("axis_map should be map[string]int")
	} else {
		if axisMap["X"] != 0 || axisMap["Y"] != 1 || axisMap["Z"] != 2 || axisMap["E"] != 3 {
			t.Errorf("unexpected axis_map: %v", axisMap)
		}
	}

	// Check homing_origin
	homingOrigin, ok := status["homing_origin"].([]float64)
	if !ok {
		t.Error("homing_origin should be []float64")
	} else {
		if len(homingOrigin) != 4 {
			t.Errorf("expected homing_origin length 4, got %d", len(homingOrigin))
		}
		if homingOrigin[0] != 0.1 || homingOrigin[1] != 0.2 || homingOrigin[2] != 0.3 {
			t.Errorf("unexpected homing_origin: %v", homingOrigin)
		}
	}
}

func TestHandleHomeRailsEnd(t *testing.T) {
	gm := newTestGcodeMove()

	gm.homingPosition = []float64{1.0, 2.0, 3.0, 0}
	gm.basePosition = []float64{0, 0, 0, 0}
	gm.lastPosition = []float64{100, 200, 50, 10}
	gm.isPrinterReady = true

	// Simulate homing X and Z axes (0 and 2)
	gm.handleHomeRailsEnd([]int{0, 2})

	// basePosition should be updated to homingPosition for homed axes
	if gm.basePosition[0] != 1.0 {
		t.Errorf("expected basePosition[0] = 1.0, got %v", gm.basePosition[0])
	}
	if gm.basePosition[2] != 3.0 {
		t.Errorf("expected basePosition[2] = 3.0, got %v", gm.basePosition[2])
	}
	// Y axis should not change (not homed)
	if gm.basePosition[1] != 0 {
		t.Errorf("expected basePosition[1] = 0 (unchanged), got %v", gm.basePosition[1])
	}
}

func TestHandleActivateExtruder(t *testing.T) {
	gm := newTestGcodeMove()

	gm.extrudeFactor = 0.9
	gm.lastPosition = []float64{100, 200, 50, 15}
	gm.basePosition = []float64{0, 0, 0, 5}
	gm.isPrinterReady = true

	gm.handleActivateExtruder()

	// extrudeFactor should reset to 1.0
	if gm.extrudeFactor != 1.0 {
		t.Errorf("expected extrudeFactor = 1.0, got %v", gm.extrudeFactor)
	}

	// basePosition[3] should equal lastPosition[3]
	if gm.basePosition[3] != gm.lastPosition[3] {
		t.Errorf("expected basePosition[3] = %v, got %v", gm.lastPosition[3], gm.basePosition[3])
	}
}

func TestArcPlaneSelection(t *testing.T) {
	gm := newTestGcodeMove()

	gm.cmdG17()
	if gm.arcPlane != arcPlaneXY {
		t.Error("G17 should set XY plane")
	}

	gm.cmdG18()
	if gm.arcPlane != arcPlaneXZ {
		t.Error("G18 should set XZ plane")
	}

	gm.cmdG19()
	if gm.arcPlane != arcPlaneYZ {
		t.Error("G19 should set YZ plane")
	}
}

func TestHandleReadyShutdown(t *testing.T) {
	gm := newTestGcodeMove()

	if gm.isPrinterReady {
		t.Error("initially should not be ready")
	}

	gm.handleReady()
	if !gm.isPrinterReady {
		t.Error("should be ready after handleReady")
	}

	gm.handleShutdown()
	if gm.isPrinterReady {
		t.Error("should not be ready after handleShutdown")
	}
}
