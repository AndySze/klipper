package hosth4

import (
	"math"
	"strings"
	"testing"
)

func TestNewZMesh(t *testing.T) {
	params := zMeshParams{
		MinX:     0,
		MaxX:     200,
		MinY:     0,
		MaxY:     200,
		XCount:   3,
		YCount:   3,
		MeshXPPS: 2,
		MeshYPPS: 2,
		Algo:     "lagrange",
		Tension:  0.2,
	}

	mesh, err := newZMesh(params, "test")
	if err != nil {
		t.Fatalf("failed to create mesh: %v", err)
	}

	if mesh.GetProfileName() != "test" {
		t.Errorf("expected profile name 'test', got '%s'", mesh.GetProfileName())
	}

	gotParams := mesh.GetMeshParams()
	if gotParams.MinX != params.MinX || gotParams.MaxX != params.MaxX {
		t.Errorf("mesh params mismatch: got %+v", gotParams)
	}
}

func TestZMeshBuildMesh(t *testing.T) {
	params := zMeshParams{
		MinX:     0,
		MaxX:     200,
		MinY:     0,
		MaxY:     200,
		XCount:   3,
		YCount:   3,
		MeshXPPS: 2,
		MeshYPPS: 2,
		Algo:     "lagrange",
	}

	mesh, err := newZMesh(params, "test")
	if err != nil {
		t.Fatalf("failed to create mesh: %v", err)
	}

	// Simple flat bed - all zeros
	zMatrix := [][]float64{
		{0.0, 0.0, 0.0},
		{0.0, 0.0, 0.0},
		{0.0, 0.0, 0.0},
	}

	if err := mesh.BuildMesh(zMatrix); err != nil {
		t.Fatalf("failed to build mesh: %v", err)
	}

	// For a flat bed, Z should be 0 everywhere
	z := mesh.CalcZ(100, 100)
	if math.Abs(z) > 0.001 {
		t.Errorf("expected Z=0 at center, got %v", z)
	}
}

func TestZMeshCalcZ(t *testing.T) {
	params := zMeshParams{
		MinX:     0,
		MaxX:     100,
		MinY:     0,
		MaxY:     100,
		XCount:   3,
		YCount:   3,
		MeshXPPS: 2,
		MeshYPPS: 2,
		Algo:     "lagrange",
	}

	mesh, err := newZMesh(params, "test")
	if err != nil {
		t.Fatalf("failed to create mesh: %v", err)
	}

	// Tilted bed - higher at Y=100
	zMatrix := [][]float64{
		{0.0, 0.0, 0.0},   // Y=0
		{0.05, 0.05, 0.05}, // Y=50
		{0.1, 0.1, 0.1},   // Y=100
	}

	if err := mesh.BuildMesh(zMatrix); err != nil {
		t.Fatalf("failed to build mesh: %v", err)
	}

	// Check corners
	z00 := mesh.CalcZ(0, 0)
	z01 := mesh.CalcZ(0, 100)
	if z00 > z01 {
		t.Errorf("expected Z to increase with Y, got z(0,0)=%v z(0,100)=%v", z00, z01)
	}

	// Check middle
	z50 := mesh.CalcZ(50, 50)
	if z50 < z00 || z50 > z01 {
		t.Errorf("expected middle Z to be between corners, got %v", z50)
	}
}

func TestZMeshGetZRange(t *testing.T) {
	params := zMeshParams{
		MinX:     0,
		MaxX:     100,
		MinY:     0,
		MaxY:     100,
		XCount:   3,
		YCount:   3,
		MeshXPPS: 2,
		MeshYPPS: 2,
		Algo:     "lagrange",
	}

	mesh, err := newZMesh(params, "test")
	if err != nil {
		t.Fatalf("failed to create mesh: %v", err)
	}

	zMatrix := [][]float64{
		{-0.1, 0.0, 0.1},
		{0.0, 0.05, 0.0},
		{0.1, 0.0, -0.1},
	}

	if err := mesh.BuildMesh(zMatrix); err != nil {
		t.Fatalf("failed to build mesh: %v", err)
	}

	minZ, maxZ := mesh.GetZRange()
	if minZ > -0.09 {
		t.Errorf("expected minZ <= -0.09, got %v", minZ)
	}
	if maxZ < 0.09 {
		t.Errorf("expected maxZ >= 0.09, got %v", maxZ)
	}
}

func TestZMeshGetZAverage(t *testing.T) {
	params := zMeshParams{
		MinX:     0,
		MaxX:     100,
		MinY:     0,
		MaxY:     100,
		XCount:   3,
		YCount:   3,
		MeshXPPS: 2,
		MeshYPPS: 2,
		Algo:     "lagrange",
	}

	mesh, err := newZMesh(params, "test")
	if err != nil {
		t.Fatalf("failed to create mesh: %v", err)
	}

	// All probed points at 0.1, so average should be close to 0.1
	zMatrix := [][]float64{
		{0.1, 0.1, 0.1},
		{0.1, 0.1, 0.1},
		{0.1, 0.1, 0.1},
	}

	if err := mesh.BuildMesh(zMatrix); err != nil {
		t.Fatalf("failed to build mesh: %v", err)
	}

	avg := mesh.GetZAverage()
	if math.Abs(avg-0.1) > 0.01 {
		t.Errorf("expected average=0.1, got %v", avg)
	}
}

func TestZMeshSetMeshOffsets(t *testing.T) {
	// Use lagrange with pps=2 for proper interpolated mesh
	params := zMeshParams{
		MinX:     0,
		MaxX:     100,
		MinY:     0,
		MaxY:     100,
		XCount:   3,
		YCount:   3,
		MeshXPPS: 2,
		MeshYPPS: 2,
		Algo:     "lagrange",
	}

	mesh, err := newZMesh(params, "test")
	if err != nil {
		t.Fatalf("failed to create mesh: %v", err)
	}

	// Slope from corner to corner - makes offset effects visible
	zMatrix := [][]float64{
		{0.0, 0.1, 0.2},
		{0.1, 0.2, 0.3},
		{0.2, 0.3, 0.4},
	}

	if err := mesh.BuildMesh(zMatrix); err != nil {
		t.Fatalf("failed to build mesh: %v", err)
	}

	// Z at a point without offset
	zNoOffset := mesh.CalcZ(25, 25)

	// Apply offset - this shifts where the mesh is sampled
	x := 25.0
	y := 25.0
	mesh.SetMeshOffsets([2]*float64{&x, &y})

	// Same coordinate with offset should give different value
	zWithOffset := mesh.CalcZ(25, 25)

	// The values should differ since the lookup point is effectively shifted
	if math.Abs(zNoOffset-zWithOffset) < 0.01 {
		t.Errorf("expected offset to change Z value, got similar: noOffset=%v withOffset=%v", zNoOffset, zWithOffset)
	}
}

func TestBedMeshProfile(t *testing.T) {
	th := &toolhead{
		commandedPos: []float64{0, 0, 0, 0},
	}

	bm, err := newBedMesh(nil, th)
	if err != nil {
		t.Fatalf("failed to create bed mesh: %v", err)
	}

	// No mesh initially
	if bm.GetMesh() != nil {
		t.Error("expected no mesh initially")
	}

	// Try to save profile without mesh
	result, err := bm.cmdBedMeshProfile(map[string]string{"SAVE": "myprofile"})
	if err != nil {
		t.Fatalf("cmdBedMeshProfile failed: %v", err)
	}
	if !strings.Contains(result, "bed has not been probed") {
		t.Errorf("expected 'not probed' message, got: %s", result)
	}

	// Create and set a mesh
	params := zMeshParams{
		MinX:     0,
		MaxX:     100,
		MinY:     0,
		MaxY:     100,
		XCount:   3,
		YCount:   3,
		MeshXPPS: 2,
		MeshYPPS: 2,
		Algo:     "lagrange",
	}
	mesh, _ := newZMesh(params, "test")
	mesh.BuildMesh([][]float64{
		{0.1, 0.1, 0.1},
		{0.1, 0.1, 0.1},
		{0.1, 0.1, 0.1},
	})

	// Need to disable fade for this test
	bm.fadeEnd = math.MaxFloat64
	bm.SetMesh(mesh)

	// Save profile
	result, err = bm.cmdBedMeshProfile(map[string]string{"SAVE": "myprofile"})
	if err != nil {
		t.Fatalf("save profile failed: %v", err)
	}
	if !strings.Contains(result, "saved") {
		t.Errorf("expected 'saved' message, got: %s", result)
	}

	// Check profile exists
	profiles := bm.GetProfiles()
	if _, ok := profiles["myprofile"]; !ok {
		t.Error("profile 'myprofile' not found")
	}

	// Clear mesh
	bm.cmdBedMeshClear(nil)
	if bm.GetMesh() != nil {
		t.Error("expected mesh to be cleared")
	}

	// Load profile
	result, err = bm.cmdBedMeshProfile(map[string]string{"LOAD": "myprofile"})
	if err != nil {
		t.Fatalf("load profile failed: %v", err)
	}
	if bm.GetMesh() == nil {
		t.Error("expected mesh to be loaded")
	}

	// Remove profile
	result, err = bm.cmdBedMeshProfile(map[string]string{"REMOVE": "myprofile"})
	if err != nil {
		t.Fatalf("remove profile failed: %v", err)
	}
	if !strings.Contains(result, "removed") {
		t.Errorf("expected 'removed' message, got: %s", result)
	}

	profiles = bm.GetProfiles()
	if _, ok := profiles["myprofile"]; ok {
		t.Error("profile 'myprofile' should not exist after removal")
	}
}

func TestBedMeshOffset(t *testing.T) {
	th := &toolhead{
		commandedPos: []float64{0, 0, 0, 0},
	}

	bm, err := newBedMesh(nil, th)
	if err != nil {
		t.Fatalf("failed to create bed mesh: %v", err)
	}

	// Without mesh, should return message
	result, err := bm.cmdBedMeshOffset(map[string]string{"X": "10"})
	if err != nil {
		t.Fatalf("cmdBedMeshOffset failed: %v", err)
	}
	if !strings.Contains(result, "No mesh loaded") {
		t.Errorf("expected 'No mesh loaded' message, got: %s", result)
	}

	// Create and set a mesh
	params := zMeshParams{
		MinX:     0,
		MaxX:     100,
		MinY:     0,
		MaxY:     100,
		XCount:   3,
		YCount:   3,
		MeshXPPS: 2,
		MeshYPPS: 2,
		Algo:     "lagrange",
	}
	mesh, _ := newZMesh(params, "test")
	mesh.BuildMesh([][]float64{
		{0.1, 0.1, 0.1},
		{0.1, 0.1, 0.1},
		{0.1, 0.1, 0.1},
	})
	bm.fadeEnd = math.MaxFloat64
	bm.SetMesh(mesh)

	// Set offset
	result, err = bm.cmdBedMeshOffset(map[string]string{"X": "5", "Y": "10"})
	if err != nil {
		t.Fatalf("cmdBedMeshOffset failed: %v", err)
	}

	if bm.zMesh.meshOffsets[0] != 5 {
		t.Errorf("expected X offset 5, got %v", bm.zMesh.meshOffsets[0])
	}
	if bm.zMesh.meshOffsets[1] != 10 {
		t.Errorf("expected Y offset 10, got %v", bm.zMesh.meshOffsets[1])
	}

	// Set ZFADE offset
	result, err = bm.cmdBedMeshOffset(map[string]string{"ZFADE": "0.5"})
	if err != nil {
		t.Fatalf("cmdBedMeshOffset failed: %v", err)
	}
	if bm.toolOffset != 0.5 {
		t.Errorf("expected tool offset 0.5, got %v", bm.toolOffset)
	}
}

func TestBedMeshMap(t *testing.T) {
	th := &toolhead{
		commandedPos: []float64{0, 0, 0, 0},
	}

	bm, err := newBedMesh(nil, th)
	if err != nil {
		t.Fatalf("failed to create bed mesh: %v", err)
	}

	// Without mesh
	result, err := bm.cmdBedMeshMap(nil)
	if err != nil {
		t.Fatalf("cmdBedMeshMap failed: %v", err)
	}
	if !strings.Contains(result, "not been probed") {
		t.Errorf("expected 'not been probed' message, got: %s", result)
	}

	// Create and set a mesh
	params := zMeshParams{
		MinX:     0,
		MaxX:     100,
		MinY:     0,
		MaxY:     100,
		XCount:   3,
		YCount:   3,
		MeshXPPS: 2,
		MeshYPPS: 2,
		Algo:     "lagrange",
	}
	mesh, _ := newZMesh(params, "test")
	mesh.BuildMesh([][]float64{
		{0.1, 0.1, 0.1},
		{0.1, 0.1, 0.1},
		{0.1, 0.1, 0.1},
	})
	bm.fadeEnd = math.MaxFloat64
	bm.SetMesh(mesh)

	// With mesh - should return JSON
	result, err = bm.cmdBedMeshMap(nil)
	if err != nil {
		t.Fatalf("cmdBedMeshMap failed: %v", err)
	}
	if !strings.Contains(result, "mesh_map_output") {
		t.Errorf("expected 'mesh_map_output' prefix, got: %s", result)
	}
	if !strings.Contains(result, "mesh_min") {
		t.Errorf("expected 'mesh_min' in output, got: %s", result)
	}
}

func TestBedMeshOutput(t *testing.T) {
	th := &toolhead{
		commandedPos: []float64{0, 0, 0, 0},
	}

	bm, err := newBedMesh(nil, th)
	if err != nil {
		t.Fatalf("failed to create bed mesh: %v", err)
	}

	// Without mesh
	result, err := bm.cmdBedMeshOutput(nil)
	if err != nil {
		t.Fatalf("cmdBedMeshOutput failed: %v", err)
	}
	if !strings.Contains(result, "not been probed") {
		t.Errorf("expected 'not been probed' message, got: %s", result)
	}

	// Create and set a mesh
	params := zMeshParams{
		MinX:     0,
		MaxX:     100,
		MinY:     0,
		MaxY:     100,
		XCount:   3,
		YCount:   3,
		MeshXPPS: 2,
		MeshYPPS: 2,
		Algo:     "lagrange",
	}
	mesh, _ := newZMesh(params, "test")
	mesh.BuildMesh([][]float64{
		{0.1, 0.1, 0.1},
		{0.1, 0.1, 0.1},
		{0.1, 0.1, 0.1},
	})
	bm.fadeEnd = math.MaxFloat64
	bm.SetMesh(mesh)

	// With mesh
	result, err = bm.cmdBedMeshOutput(nil)
	if err != nil {
		t.Fatalf("cmdBedMeshOutput failed: %v", err)
	}
	if !strings.Contains(result, "Mesh Leveling") {
		t.Errorf("expected 'Mesh Leveling' in output, got: %s", result)
	}
	if !strings.Contains(result, "Mesh X,Y:") {
		t.Errorf("expected 'Mesh X,Y:' in output, got: %s", result)
	}
}

func TestBedMeshGetStatus(t *testing.T) {
	th := &toolhead{
		commandedPos: []float64{0, 0, 0, 0},
	}

	bm, err := newBedMesh(nil, th)
	if err != nil {
		t.Fatalf("failed to create bed mesh: %v", err)
	}

	status := bm.GetStatus()
	if status["profile_name"] != "" {
		t.Errorf("expected empty profile_name initially, got %v", status["profile_name"])
	}

	// Create and set a mesh
	params := zMeshParams{
		MinX:     0,
		MaxX:     100,
		MinY:     0,
		MaxY:     100,
		XCount:   3,
		YCount:   3,
		MeshXPPS: 2,
		MeshYPPS: 2,
		Algo:     "lagrange",
	}
	mesh, _ := newZMesh(params, "myprofile")
	mesh.BuildMesh([][]float64{
		{0.1, 0.1, 0.1},
		{0.1, 0.1, 0.1},
		{0.1, 0.1, 0.1},
	})
	bm.fadeEnd = math.MaxFloat64
	bm.SetMesh(mesh)

	status = bm.GetStatus()
	if status["profile_name"] != "myprofile" {
		t.Errorf("expected profile_name 'myprofile', got %v", status["profile_name"])
	}

	meshMin, ok := status["mesh_min"].([]float64)
	if !ok || len(meshMin) != 2 {
		t.Error("expected mesh_min to be []float64 with 2 elements")
	}
	if meshMin[0] != 0 || meshMin[1] != 0 {
		t.Errorf("expected mesh_min [0,0], got %v", meshMin)
	}

	meshMax, ok := status["mesh_max"].([]float64)
	if !ok || len(meshMax) != 2 {
		t.Error("expected mesh_max to be []float64 with 2 elements")
	}
	if meshMax[0] != 100 || meshMax[1] != 100 {
		t.Errorf("expected mesh_max [100,100], got %v", meshMax)
	}
}

func TestMoveSplitter(t *testing.T) {
	params := zMeshParams{
		MinX:     0,
		MaxX:     100,
		MinY:     0,
		MaxY:     100,
		XCount:   3,
		YCount:   3,
		MeshXPPS: 2,
		MeshYPPS: 2,
		Algo:     "lagrange",
	}

	mesh, _ := newZMesh(params, "test")
	mesh.BuildMesh([][]float64{
		{0.0, 0.0, 0.0},
		{0.0, 0.1, 0.0},
		{0.0, 0.0, 0.0},
	})

	splitter := newMoveSplitter(0.025, 5.0)
	splitter.Initialize(mesh, 0.0)

	prevPos := []float64{0, 0, 0, 0}
	nextPos := []float64{100, 100, 0, 0}
	splitter.BuildMove(prevPos, nextPos, 1.0)

	// Should generate multiple splits due to z-change
	splits := 0
	for !splitter.traverseComplete {
		split := splitter.Split()
		if split != nil {
			splits++
		}
	}

	if splits < 1 {
		t.Error("expected at least one split")
	}
}

func TestMicrostepsToMRES(t *testing.T) {
	tests := []struct {
		microsteps int
		mres       uint32
	}{
		{256, 0},
		{128, 1},
		{64, 2},
		{32, 3},
		{16, 4},
		{8, 5},
		{4, 6},
		{2, 7},
		{1, 8},
	}

	for _, tc := range tests {
		mres, err := MicrostepsToMRES(tc.microsteps)
		if err != nil {
			t.Errorf("MicrostepsToMRES(%d) failed: %v", tc.microsteps, err)
			continue
		}
		if mres != tc.mres {
			t.Errorf("MicrostepsToMRES(%d) = %d, want %d", tc.microsteps, mres, tc.mres)
		}

		ms := MRESToMicrosteps(tc.mres)
		if ms != tc.microsteps {
			t.Errorf("MRESToMicrosteps(%d) = %d, want %d", tc.mres, ms, tc.microsteps)
		}
	}

	// Invalid microsteps
	_, err := MicrostepsToMRES(15)
	if err == nil {
		t.Error("expected error for invalid microsteps")
	}
}
