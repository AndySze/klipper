package hosth4

import (
	"encoding/json"
	"fmt"
	"log"
	"math"
	"strings"
	"sync"
)

type moveTransform interface {
	Move(newPos []float64, speed float64) error
	GetPosition() []float64
}

type bedMesh struct {
	toolhead *toolhead

	lastPosition []float64

	zMesh *zMesh

	fadeStart       float64
	fadeEnd         float64
	fadeDist        float64
	fadeTarget      float64
	baseFadeTarget  *float64 // nil means auto (use mesh average)
	horizontalMoveZ float64

	toolOffset float64

	splitter *moveSplitter

	// Profile management
	profiles   map[string]*bedMeshProfile
	profilesMu sync.RWMutex

	// Status cache
	status   map[string]any
	statusMu sync.RWMutex
}

// bedMeshProfile stores a saved mesh profile.
type bedMeshProfile struct {
	Points     [][]float64
	MeshParams zMeshParams
}

func newBedMesh(cfg *configWrapper, th *toolhead) (*bedMesh, error) {
	if th == nil {
		return nil, fmt.Errorf("toolhead not initialized")
	}
	fadeStart := 1.0
	fadeEnd := 0.0
	var baseFadeTarget *float64
	horizontalMoveZ := 5.0
	splitDeltaZ := 0.025
	moveCheckDistance := 5.0
	if cfg != nil {
		if sec, ok := cfg.section("bed_mesh"); ok {
			if raw := sec["fade_start"]; raw != "" {
				if v, err := parseFloat(sec, "fade_start", nil); err == nil {
					fadeStart = v
				}
			}
			if raw := sec["fade_end"]; raw != "" {
				if v, err := parseFloat(sec, "fade_end", nil); err == nil {
					fadeEnd = v
				}
			}
			if raw := sec["fade_target"]; raw != "" {
				if v, err := parseFloat(sec, "fade_target", nil); err == nil {
					baseFadeTarget = &v
				}
			}
			if raw := sec["horizontal_move_z"]; raw != "" {
				if v, err := parseFloat(sec, "horizontal_move_z", nil); err == nil {
					horizontalMoveZ = v
				}
			}
			if raw := sec["split_delta_z"]; raw != "" {
				if v, err := parseFloat(sec, "split_delta_z", nil); err == nil {
					splitDeltaZ = v
				}
			}
			if raw := sec["move_check_distance"]; raw != "" {
				if v, err := parseFloat(sec, "move_check_distance", nil); err == nil {
					moveCheckDistance = v
				}
			}
		}
	}
	fadeDist := fadeEnd - fadeStart
	fadeTarget := 0.0
	if fadeDist <= 0.0 {
		// Match Klippy: disable fade if fade_end <= fade_start.
		fadeStart = math.MaxFloat64
		fadeEnd = math.MaxFloat64
		fadeDist = 0.0
		fadeTarget = 0.0
	}

	bm := &bedMesh{
		toolhead:        th,
		lastPosition:    append([]float64{}, th.commandedPos...),
		zMesh:           nil,
		fadeStart:       fadeStart,
		fadeEnd:         fadeEnd,
		fadeDist:        fadeDist,
		fadeTarget:      fadeTarget,
		baseFadeTarget:  baseFadeTarget,
		horizontalMoveZ: horizontalMoveZ,
		toolOffset:      0.0,
		splitter:        newMoveSplitter(splitDeltaZ, moveCheckDistance),
		profiles:        make(map[string]*bedMeshProfile),
		status:          make(map[string]any),
	}
	bm.updateStatus()
	return bm, nil
}

func (bm *bedMesh) SetMesh(mesh *zMesh) error {
	if mesh != nil && bm.fadeEnd != math.MaxFloat64 {
		// Calculate fade target
		if bm.baseFadeTarget == nil {
			bm.fadeTarget = mesh.GetZAverage()
		} else {
			bm.fadeTarget = *bm.baseFadeTarget
			minZ, maxZ := mesh.GetZRange()
			if bm.fadeTarget != 0.0 && (bm.fadeTarget < minZ || bm.fadeTarget > maxZ) {
				bm.zMesh = nil
				bm.fadeTarget = 0.0
				return fmt.Errorf("bed_mesh: fade_target lies outside of mesh z range min: %.4f, max: %.4f, fade_target: %.4f",
					minZ, maxZ, bm.fadeTarget)
			}
		}
		// Check fade distance vs mesh range
		minZ, maxZ := mesh.GetZRange()
		if bm.fadeDist <= math.Max(math.Abs(minZ), math.Abs(maxZ)) {
			bm.zMesh = nil
			bm.fadeTarget = 0.0
			return fmt.Errorf("bed_mesh: mesh extends outside of fade range, fade distance: %.2f mesh min: %.4f mesh max: %.4f",
				bm.fadeDist, minZ, maxZ)
		}
	} else {
		bm.fadeTarget = 0.0
	}

	bm.toolOffset = 0.0
	bm.zMesh = mesh
	if bm.splitter != nil {
		bm.splitter.Initialize(mesh, bm.fadeTarget)
	}
	bm.updateStatus()
	return nil
}

// GetMesh returns the current z mesh.
func (bm *bedMesh) GetMesh() *zMesh {
	return bm.zMesh
}

// updateStatus updates the cached status.
func (bm *bedMesh) updateStatus() {
	bm.statusMu.Lock()
	defer bm.statusMu.Unlock()

	bm.profilesMu.RLock()
	profiles := make(map[string]map[string]any, len(bm.profiles))
	for name := range bm.profiles {
		profiles[name] = map[string]any{}
	}
	bm.profilesMu.RUnlock()

	bm.status = map[string]any{
		"profile_name":   "",
		"mesh_min":       []float64{0.0, 0.0},
		"mesh_max":       []float64{0.0, 0.0},
		"probed_matrix":  [][]float64{{}},
		"mesh_matrix":    [][]float64{{}},
		"profiles":       profiles,
	}

	if bm.zMesh != nil {
		params := bm.zMesh.GetMeshParams()
		bm.status["profile_name"] = bm.zMesh.GetProfileName()
		bm.status["mesh_min"] = []float64{params.MinX, params.MinY}
		bm.status["mesh_max"] = []float64{params.MaxX, params.MaxY}
		bm.status["probed_matrix"] = bm.zMesh.GetProbedMatrix()
		bm.status["mesh_matrix"] = bm.zMesh.GetMeshMatrix()
	}
}

// GetStatus returns the current bed mesh status.
func (bm *bedMesh) GetStatus() map[string]any {
	bm.statusMu.RLock()
	defer bm.statusMu.RUnlock()
	// Return a copy
	result := make(map[string]any, len(bm.status))
	for k, v := range bm.status {
		result[k] = v
	}
	return result
}

// cmdBedMeshOutput handles the BED_MESH_OUTPUT command.
func (bm *bedMesh) cmdBedMeshOutput(args map[string]string) (string, error) {
	if bm.zMesh == nil {
		return "Bed has not been probed", nil
	}

	var result strings.Builder
	result.WriteString("Mesh Leveling Probed Z positions:\n")
	probed := bm.zMesh.GetProbedMatrix()
	for _, line := range probed {
		for _, z := range line {
			result.WriteString(fmt.Sprintf(" %f", z))
		}
		result.WriteString("\n")
	}

	mesh := bm.zMesh.GetMeshMatrix()
	result.WriteString(fmt.Sprintf("Mesh X,Y: %d,%d\n", bm.zMesh.meshXCount, bm.zMesh.meshYCount))
	result.WriteString(fmt.Sprintf("Search Height: %.0f\n", bm.horizontalMoveZ))
	result.WriteString(fmt.Sprintf("Mesh Offsets: X=%.4f, Y=%.4f\n", bm.zMesh.meshOffsets[0], bm.zMesh.meshOffsets[1]))
	result.WriteString(fmt.Sprintf("Mesh Average: %.2f\n", bm.zMesh.GetZAverage()))
	minZ, maxZ := bm.zMesh.GetZRange()
	result.WriteString(fmt.Sprintf("Mesh Range: min=%.4f max=%.4f\n", minZ, maxZ))
	result.WriteString(fmt.Sprintf("Interpolation Algorithm: %s\n", bm.zMesh.params.Algo))
	result.WriteString("Measured points:\n")
	for y := bm.zMesh.meshYCount - 1; y >= 0; y-- {
		for _, z := range mesh[y] {
			result.WriteString(fmt.Sprintf("  %f", z))
		}
		result.WriteString("\n")
	}

	return result.String(), nil
}

// cmdBedMeshMap handles the BED_MESH_MAP command.
func (bm *bedMesh) cmdBedMeshMap(args map[string]string) (string, error) {
	if bm.zMesh == nil {
		return "Bed has not been probed", nil
	}

	params := bm.zMesh.GetMeshParams()
	outdict := map[string]any{
		"mesh_min":    []float64{params.MinX, params.MinY},
		"mesh_max":    []float64{params.MaxX, params.MaxY},
		"z_positions": bm.zMesh.GetProbedMatrix(),
	}
	data, err := json.Marshal(outdict)
	if err != nil {
		return "", fmt.Errorf("bed_mesh: failed to serialize mesh map: %w", err)
	}
	return "mesh_map_output " + string(data), nil
}

// cmdBedMeshClear handles the BED_MESH_CLEAR command.
func (bm *bedMesh) cmdBedMeshClear(args map[string]string) (string, error) {
	if err := bm.SetMesh(nil); err != nil {
		return "", err
	}
	return "", nil
}

// cmdBedMeshOffset handles the BED_MESH_OFFSET command.
func (bm *bedMesh) cmdBedMeshOffset(args map[string]string) (string, error) {
	if bm.zMesh == nil {
		return "No mesh loaded to offset", nil
	}

	offsets := [2]*float64{nil, nil}
	if xStr, ok := args["X"]; ok && xStr != "" {
		x, err := bedMeshParseFloat(xStr)
		if err != nil {
			return "", fmt.Errorf("invalid X value: %s", xStr)
		}
		offsets[0] = &x
	}
	if yStr, ok := args["Y"]; ok && yStr != "" {
		y, err := bedMeshParseFloat(yStr)
		if err != nil {
			return "", fmt.Errorf("invalid Y value: %s", yStr)
		}
		offsets[1] = &y
	}
	bm.zMesh.SetMeshOffsets(offsets)

	if zfadeStr, ok := args["ZFADE"]; ok && zfadeStr != "" {
		zfade, err := bedMeshParseFloat(zfadeStr)
		if err != nil {
			return "", fmt.Errorf("invalid ZFADE value: %s", zfadeStr)
		}
		bm.toolOffset = zfade
	}

	return "", nil
}

// cmdBedMeshProfile handles the BED_MESH_PROFILE command.
func (bm *bedMesh) cmdBedMeshProfile(args map[string]string) (string, error) {
	if loadName, ok := args["LOAD"]; ok && loadName != "" {
		return bm.loadProfile(loadName)
	}
	if saveName, ok := args["SAVE"]; ok && saveName != "" {
		if saveName == "default" {
			return "Profile 'default' is reserved, please choose another profile name.", nil
		}
		return bm.saveProfile(saveName)
	}
	if removeName, ok := args["REMOVE"]; ok && removeName != "" {
		return bm.removeProfile(removeName)
	}
	return "Invalid syntax", nil
}

// saveProfile saves the current mesh to a profile.
func (bm *bedMesh) saveProfile(name string) (string, error) {
	if bm.zMesh == nil {
		return fmt.Sprintf("Unable to save to profile [%s], the bed has not been probed", name), nil
	}

	bm.profilesMu.Lock()
	bm.profiles[name] = &bedMeshProfile{
		Points:     bm.zMesh.GetProbedMatrix(),
		MeshParams: bm.zMesh.GetMeshParams(),
	}
	bm.profilesMu.Unlock()

	bm.updateStatus()
	log.Printf("bed_mesh: profile [%s] saved", name)
	return fmt.Sprintf("Bed Mesh state has been saved to profile [%s] for the current session.", name), nil
}

// loadProfile loads a mesh from a profile.
func (bm *bedMesh) loadProfile(name string) (string, error) {
	bm.profilesMu.RLock()
	profile, ok := bm.profiles[name]
	bm.profilesMu.RUnlock()

	if !ok {
		return "", fmt.Errorf("bed_mesh: Unknown profile [%s]", name)
	}

	mesh, err := newZMesh(profile.MeshParams, name)
	if err != nil {
		return "", err
	}
	if err := mesh.BuildMesh(profile.Points); err != nil {
		return "", err
	}
	if err := bm.SetMesh(mesh); err != nil {
		return "", err
	}
	log.Printf("bed_mesh: profile [%s] loaded", name)
	return fmt.Sprintf("Bed Mesh profile [%s] loaded", name), nil
}

// removeProfile removes a mesh profile.
func (bm *bedMesh) removeProfile(name string) (string, error) {
	bm.profilesMu.Lock()
	_, ok := bm.profiles[name]
	if ok {
		delete(bm.profiles, name)
	}
	bm.profilesMu.Unlock()

	if !ok {
		return fmt.Sprintf("No profile named [%s] to remove", name), nil
	}

	bm.updateStatus()
	log.Printf("bed_mesh: profile [%s] removed", name)
	return fmt.Sprintf("Profile [%s] removed from storage for this session.", name), nil
}

// GetProfiles returns all saved profiles.
func (bm *bedMesh) GetProfiles() map[string]*bedMeshProfile {
	bm.profilesMu.RLock()
	defer bm.profilesMu.RUnlock()
	result := make(map[string]*bedMeshProfile, len(bm.profiles))
	for k, v := range bm.profiles {
		result[k] = v
	}
	return result
}

func (bm *bedMesh) getZFactor(zPos float64) float64 {
	zPos = zPos + bm.toolOffset
	if zPos >= bm.fadeEnd {
		return 0.0
	}
	if zPos >= bm.fadeStart && bm.fadeDist > 0.0 {
		return (bm.fadeEnd - zPos) / bm.fadeDist
	}
	return 1.0
}

func (bm *bedMesh) GetPosition() []float64 {
	if bm.toolhead == nil {
		return nil
	}
	cur := bm.toolhead.commandedPos
	if len(cur) == 0 {
		return nil
	}

	// Match Klippy: return last non-transformed position, derived from the
	// toolhead's physical position and the current mesh adjustment.
	out := append([]float64{}, cur...)
	if bm.zMesh == nil {
		out[2] = out[2] - bm.fadeTarget
		bm.lastPosition = append([]float64{}, out...)
		return out
	}
	x, y, z := out[0], out[1], out[2]
	maxAdj := bm.zMesh.CalcZ(x, y)
	zAdj := maxAdj - bm.fadeTarget
	fadeZPos := z + bm.toolOffset
	factor := 1.0
	if math.Min(fadeZPos, (fadeZPos-zAdj)) >= bm.fadeEnd {
		factor = 0.0
	} else if math.Max(fadeZPos, (fadeZPos-zAdj)) >= bm.fadeStart && bm.fadeDist > 0.0 {
		factor = (bm.fadeEnd + bm.fadeTarget - fadeZPos) / (bm.fadeDist - zAdj)
		factor = constrain(factor, 0.0, 1.0)
	}
	finalZAdj := factor*zAdj + bm.fadeTarget
	out[2] = z - finalZAdj
	bm.lastPosition = append([]float64{}, out...)
	return out
}

func (bm *bedMesh) Move(newPos []float64, speed float64) error {
	if bm.toolhead == nil {
		return fmt.Errorf("toolhead not initialized")
	}
	if len(newPos) < 3 {
		return fmt.Errorf("bed_mesh requires xyz")
	}

	factor := bm.getZFactor(newPos[2])
	if bm.zMesh == nil || factor == 0.0 || bm.splitter == nil {
		adj := bm.fadeTarget
		phys := append([]float64{}, newPos...)
		phys[2] = phys[2] + adj
		if err := bm.toolhead.move(phys, speed); err != nil {
			return err
		}
		bm.lastPosition = append([]float64{}, newPos...)
		return nil
	}

	bm.splitter.BuildMove(bm.lastPosition, newPos, factor)
	for !bm.splitter.traverseComplete {
		splitMove := bm.splitter.Split()
		if splitMove == nil {
			return fmt.Errorf("bed_mesh: error splitting move")
		}
		if err := bm.toolhead.move(splitMove, speed); err != nil {
			return err
		}
	}
	bm.lastPosition = append([]float64{}, newPos...)
	return nil
}

type moveSplitter struct {
	splitDeltaZ       float64
	moveCheckDistance float64

	zMesh       *zMesh
	fadeOffset  float64
	zFactor     float64
	zOffset     float64
	prevPos     []float64
	nextPos     []float64
	currentPos  []float64
	axisMove    []bool
	moveLength  float64
	checkedDist float64

	traverseComplete bool
}

func newMoveSplitter(splitDeltaZ, moveCheckDistance float64) *moveSplitter {
	return &moveSplitter{
		splitDeltaZ:       splitDeltaZ,
		moveCheckDistance: moveCheckDistance,
	}
}

func (ms *moveSplitter) Initialize(mesh *zMesh, fadeOffset float64) {
	ms.zMesh = mesh
	ms.fadeOffset = fadeOffset
}

func (ms *moveSplitter) BuildMove(prevPos, nextPos []float64, factor float64) {
	ms.prevPos = append([]float64{}, prevPos...)
	ms.nextPos = append([]float64{}, nextPos...)
	ms.currentPos = append([]float64{}, prevPos...)
	ms.zFactor = factor
	ms.zOffset = ms.calcZOffset(prevPos)
	ms.traverseComplete = false
	ms.checkedDist = 0.0

	axesD := make([]float64, 0, 3)
	for i := 0; i < 3 && i < len(ms.nextPos) && i < len(ms.prevPos); i++ {
		axesD = append(axesD, ms.nextPos[i]-ms.prevPos[i])
	}
	sum := 0.0
	for i := 0; i < len(axesD); i++ {
		sum += axesD[i] * axesD[i]
	}
	ms.moveLength = math.Sqrt(sum)
	ms.axisMove = make([]bool, len(ms.nextPos))
	for i := 0; i < len(ms.axisMove) && i < len(ms.prevPos); i++ {
		ms.axisMove[i] = !isClose(ms.nextPos[i]-ms.prevPos[i], 0.0, 1e-10)
	}
}

func (ms *moveSplitter) calcZOffset(pos []float64) float64 {
	if ms.zMesh == nil || len(pos) < 2 {
		return 0.0
	}
	z := ms.zMesh.CalcZ(pos[0], pos[1])
	offset := ms.fadeOffset
	return ms.zFactor*(z-offset) + offset
}

func (ms *moveSplitter) setNextMove(distanceFromPrev float64) error {
	if ms.moveLength <= 0.0 {
		return nil
	}
	t := distanceFromPrev / ms.moveLength
	if t > 1.0 || t < 0.0 {
		return fmt.Errorf("bed_mesh: slice distance out of range")
	}
	for i := 0; i < len(ms.nextPos) && i < len(ms.prevPos); i++ {
		if ms.axisMove[i] {
			ms.currentPos[i] = lerp(t, ms.prevPos[i], ms.nextPos[i])
		}
	}
	return nil
}

func (ms *moveSplitter) Split() []float64 {
	if ms.traverseComplete {
		return nil
	}

	if ms.moveLength <= 0.0 {
		ms.currentPos = append([]float64{}, ms.nextPos...)
		ms.zOffset = ms.calcZOffset(ms.currentPos)
		ms.currentPos[2] += ms.zOffset
		ms.traverseComplete = true
		return ms.currentPos
	}

	if (len(ms.axisMove) > 0 && ms.axisMove[0]) || (len(ms.axisMove) > 1 && ms.axisMove[1]) {
		for ms.checkedDist+ms.moveCheckDistance < ms.moveLength {
			ms.checkedDist += ms.moveCheckDistance
			if err := ms.setNextMove(ms.checkedDist); err != nil {
				return nil
			}
			nextZ := ms.calcZOffset(ms.currentPos)
			if math.Abs(nextZ-ms.zOffset) >= ms.splitDeltaZ {
				ms.zOffset = nextZ
				out := append([]float64{}, ms.currentPos...)
				out[2] += ms.zOffset
				return out
			}
		}
	}

	ms.currentPos = append([]float64{}, ms.nextPos...)
	ms.zOffset = ms.calcZOffset(ms.currentPos)
	ms.currentPos[2] += ms.zOffset
	ms.traverseComplete = true
	return ms.currentPos
}

type zMeshParams struct {
	MinX float64
	MaxX float64
	MinY float64
	MaxY float64

	XCount   int
	YCount   int
	MeshXPPS int
	MeshYPPS int

	Algo    string
	Tension float64
}

type zMesh struct {
	params      zMeshParams
	profileName string

	meshOffsets [2]float64

	meshXMin float64
	meshXMax float64
	meshYMin float64
	meshYMax float64

	meshXCount int
	meshYCount int
	xMult      int
	yMult      int
	meshXDist  float64
	meshYDist  float64

	probedMatrix [][]float64
	meshMatrix   [][]float64
}

func newZMesh(params zMeshParams, profileName string) (*zMesh, error) {
	if params.XCount < 2 || params.YCount < 2 {
		return nil, fmt.Errorf("bed_mesh: invalid point counts")
	}
	meshXPPS := params.MeshXPPS
	meshYPPS := params.MeshYPPS
	if meshXPPS <= 0 {
		meshXPPS = 2
	}
	if meshYPPS <= 0 {
		meshYPPS = 2
	}
	xMult := meshXPPS + 1
	yMult := meshYPPS + 1
	meshXCount := (params.XCount-1)*meshXPPS + params.XCount
	meshYCount := (params.YCount-1)*meshYPPS + params.YCount
	if meshXCount < 2 || meshYCount < 2 {
		return nil, fmt.Errorf("bed_mesh: invalid mesh size")
	}
	meshXDist := (params.MaxX - params.MinX) / float64(meshXCount-1)
	meshYDist := (params.MaxY - params.MinY) / float64(meshYCount-1)
	if profileName == "" {
		profileName = fmt.Sprintf("adaptive-%p", &params)
	}
	return &zMesh{
		params:       params,
		profileName:  profileName,
		meshOffsets:  [2]float64{0.0, 0.0},
		meshXMin:     params.MinX,
		meshXMax:     params.MaxX,
		meshYMin:     params.MinY,
		meshYMax:     params.MaxY,
		meshXCount:   meshXCount,
		meshYCount:   meshYCount,
		xMult:        xMult,
		yMult:        yMult,
		meshXDist:    meshXDist,
		meshYDist:    meshYDist,
		probedMatrix: nil,
		meshMatrix:   nil,
	}, nil
}

// GetProfileName returns the profile name.
func (zm *zMesh) GetProfileName() string {
	return zm.profileName
}

// GetMeshParams returns the mesh parameters.
func (zm *zMesh) GetMeshParams() zMeshParams {
	return zm.params
}

// GetProbedMatrix returns the probed z-values (rounded to 6 decimal places).
func (zm *zMesh) GetProbedMatrix() [][]float64 {
	if zm.probedMatrix == nil {
		return [][]float64{{}}
	}
	result := make([][]float64, len(zm.probedMatrix))
	for i, row := range zm.probedMatrix {
		result[i] = make([]float64, len(row))
		for j, z := range row {
			result[i][j] = math.Round(z*1e6) / 1e6
		}
	}
	return result
}

// GetMeshMatrix returns the interpolated mesh matrix (rounded to 6 decimal places).
func (zm *zMesh) GetMeshMatrix() [][]float64 {
	if zm.meshMatrix == nil {
		return [][]float64{{}}
	}
	result := make([][]float64, len(zm.meshMatrix))
	for i, row := range zm.meshMatrix {
		result[i] = make([]float64, len(row))
		for j, z := range row {
			result[i][j] = math.Round(z*1e6) / 1e6
		}
	}
	return result
}

// GetZRange returns the min and max z values in the mesh.
func (zm *zMesh) GetZRange() (float64, float64) {
	if zm.meshMatrix == nil {
		return 0.0, 0.0
	}
	minZ := math.MaxFloat64
	maxZ := -math.MaxFloat64
	for _, row := range zm.meshMatrix {
		for _, z := range row {
			if z < minZ {
				minZ = z
			}
			if z > maxZ {
				maxZ = z
			}
		}
	}
	return minZ, maxZ
}

// GetZAverage returns the average z value in the mesh (rounded to 2 decimal places).
func (zm *zMesh) GetZAverage() float64 {
	if zm.meshMatrix == nil {
		return 0.0
	}
	sum := 0.0
	count := 0
	for _, row := range zm.meshMatrix {
		for _, z := range row {
			sum += z
			count++
		}
	}
	if count == 0 {
		return 0.0
	}
	return math.Round((sum/float64(count))*100) / 100
}

// SetMeshOffsets sets the X/Y mesh offsets.
func (zm *zMesh) SetMeshOffsets(offsets [2]*float64) {
	if offsets[0] != nil {
		zm.meshOffsets[0] = *offsets[0]
	}
	if offsets[1] != nil {
		zm.meshOffsets[1] = *offsets[1]
	}
}

func (zm *zMesh) BuildMesh(zMatrix [][]float64) error {
	zm.probedMatrix = zMatrix
	switch zm.params.Algo {
	case "", "lagrange":
		zm.sampleLagrange(zMatrix)
	case "direct":
		zm.sampleDirect(zMatrix)
	default:
		return fmt.Errorf("bed_mesh: unsupported algorithm %q", zm.params.Algo)
	}
	return nil
}

func (zm *zMesh) sampleDirect(zMatrix [][]float64) {
	zm.meshMatrix = zMatrix
}

func (zm *zMesh) sampleLagrange(zMatrix [][]float64) {
	xMult := zm.xMult
	yMult := zm.yMult
	mesh := make([][]float64, zm.meshYCount)
	for j := 0; j < zm.meshYCount; j++ {
		row := make([]float64, zm.meshXCount)
		for i := 0; i < zm.meshXCount; i++ {
			if (i%xMult) != 0 || (j%yMult) != 0 {
				row[i] = 0.0
				continue
			}
			row[i] = zMatrix[j/yMult][i/xMult]
		}
		mesh[j] = row
	}
	zm.meshMatrix = mesh

	xpts := make([]float64, 0, zm.params.XCount)
	ypts := make([]float64, 0, zm.params.YCount)
	for i := 0; i < zm.params.XCount; i++ {
		xpts = append(xpts, zm.getXCoordinate(i*zm.xMult))
	}
	for j := 0; j < zm.params.YCount; j++ {
		ypts = append(ypts, zm.getYCoordinate(j*zm.yMult))
	}

	// Interpolate X coordinates along probed rows.
	for y := 0; y < zm.meshYCount; y++ {
		if (y % yMult) != 0 {
			continue
		}
		for x := 0; x < zm.meshXCount; x++ {
			if (x % xMult) == 0 {
				continue
			}
			c := zm.getXCoordinate(x)
			zm.meshMatrix[y][x] = zm.calcLagrange(xpts, c, y, 0)
		}
	}
	// Interpolate Y coordinates for all columns.
	for x := 0; x < zm.meshXCount; x++ {
		for y := 0; y < zm.meshYCount; y++ {
			if (y % yMult) == 0 {
				continue
			}
			c := zm.getYCoordinate(y)
			zm.meshMatrix[y][x] = zm.calcLagrange(ypts, c, x, 1)
		}
	}
}

func (zm *zMesh) calcLagrange(lpts []float64, c float64, vec int, axis int) float64 {
	ptCnt := len(lpts)
	total := 0.0
	for i := 0; i < ptCnt; i++ {
		n := 1.0
		d := 1.0
		for j := 0; j < ptCnt; j++ {
			if j == i {
				continue
			}
			n *= (c - lpts[j])
			d *= (lpts[i] - lpts[j])
		}
		z := 0.0
		if axis == 0 {
			z = zm.meshMatrix[vec][i*zm.xMult]
		} else {
			z = zm.meshMatrix[i*zm.yMult][vec]
		}
		total += z * n / d
	}
	return total
}

func (zm *zMesh) CalcZ(x, y float64) float64 {
	if zm.meshMatrix == nil || zm.meshXCount < 2 || zm.meshYCount < 2 {
		return 0.0
	}
	tx, xidx := zm.getLinearIndex(x+zm.meshOffsets[0], 0)
	ty, yidx := zm.getLinearIndex(y+zm.meshOffsets[1], 1)
	tbl := zm.meshMatrix
	z0 := lerp(tx, tbl[yidx][xidx], tbl[yidx][xidx+1])
	z1 := lerp(tx, tbl[yidx+1][xidx], tbl[yidx+1][xidx+1])
	return lerp(ty, z0, z1)
}

func (zm *zMesh) getXCoordinate(index int) float64 {
	return zm.meshXMin + zm.meshXDist*float64(index)
}

func (zm *zMesh) getYCoordinate(index int) float64 {
	return zm.meshYMin + zm.meshYDist*float64(index)
}

func (zm *zMesh) getLinearIndex(coord float64, axis int) (float64, int) {
	var meshMin float64
	var meshCnt int
	var meshDist float64
	var coordFn func(int) float64
	if axis == 0 {
		meshMin, meshCnt, meshDist = zm.meshXMin, zm.meshXCount, zm.meshXDist
		coordFn = zm.getXCoordinate
	} else {
		meshMin, meshCnt, meshDist = zm.meshYMin, zm.meshYCount, zm.meshYDist
		coordFn = zm.getYCoordinate
	}
	idx := int(math.Floor((coord - meshMin) / meshDist))
	idx = int(constrain(float64(idx), 0.0, float64(meshCnt-2)))
	t := (coord - coordFn(idx)) / meshDist
	t = constrain(t, 0.0, 1.0)
	return t, idx
}

func isClose(a, b, absTol float64) bool {
	return math.Abs(a-b) <= absTol
}

func constrain(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}

func lerp(t, a, b float64) float64 {
	return a + t*(b-a)
}

// bedMeshParseFloat parses a string to float64.
func bedMeshParseFloat(s string) (float64, error) {
	var f float64
	_, err := fmt.Sscanf(s, "%f", &f)
	return f, err
}
