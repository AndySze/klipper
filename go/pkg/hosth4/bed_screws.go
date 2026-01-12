package hosth4

import (
	"fmt"
	"math"
	"strconv"
	"strings"
)

// screw represents a single bed screw position
type screw struct {
	coord [2]float64 // X, Y coordinates
	name  string
}

// bedScrewsState represents the current state of the bed screws adjustment tool
type bedScrewsState string

const (
	bedScrewsStateInactive bedScrewsState = ""
	bedScrewsStateAdjust   bedScrewsState = "adjust"
	bedScrewsStateFine     bedScrewsState = "fine"
)

// bedScrews implements the bed screws adjustment helper tool
type bedScrews struct {
	rt              *runtime
	screws          []screw               // Coarse adjustment positions
	fineAdjust      []screw               // Fine adjustment positions
	numberOfScrews  int                   // Total number of screws
	speed           float64               // Horizontal movement speed
	liftSpeed       float64               // Vertical movement speed
	horizontalMoveZ float64               // Z height for horizontal moves
	probeZ          float64               // Z height for probing

	state          bedScrewsState         // Current state (adjust/fine/inactive)
	currentScrew   int                    // Index of current screw
	acceptedScrews int                    // Number of accepted screws

	// Dynamic command state
	acceptActive   bool
	adjustedActive bool
	abortActive    bool
}

// newBedScrews creates a new bed screws adjustment helper
func newBedScrews(rt *runtime, cfg *configWrapper) (*bedScrews, error) {
	bs := &bedScrews{
		rt:              rt,
		speed:           50.0,
		liftSpeed:       5.0,
		horizontalMoveZ: 5.0,
		probeZ:          0.0,
		state:           bedScrewsStateInactive,
	}

	// Parse [bed_screws] section
	sec, ok := cfg.section("bed_screws")
	if !ok {
		return nil, fmt.Errorf("[bed_screws] section not found")
	}

	// Read speed parameters using parseFloat helper
	defSpeed := 50.0
	if speed, err := parseFloat(sec, "speed", &defSpeed); err == nil && speed > 0 {
		bs.speed = speed
	}
	defLiftSpeed := 5.0
	if liftSpeed, err := parseFloat(sec, "probe_speed", &defLiftSpeed); err == nil && liftSpeed > 0 {
		bs.liftSpeed = liftSpeed
	}
	defHZ := 5.0
	if hz, err := parseFloat(sec, "horizontal_move_z", &defHZ); err == nil {
		bs.horizontalMoveZ = hz
	}
	defPZ := 0.0
	if pz, err := parseFloat(sec, "probe_height", &defPZ); err == nil {
		bs.probeZ = pz
	}

	// Read screw positions
	for i := 1; i < 100; i++ {
		prefix := fmt.Sprintf("screw%d", i)
		coordStr, ok := sec[prefix]
		if !ok || strings.TrimSpace(coordStr) == "" {
			break
		}

		// Parse coordinate (format: "X,Y")
		parts := strings.Split(coordStr, ",")
		if len(parts) != 2 {
			return nil, fmt.Errorf("invalid %s coordinate: %s", prefix, coordStr)
		}
		x, err := strconv.ParseFloat(strings.TrimSpace(parts[0]), 64)
		if err != nil {
			return nil, fmt.Errorf("invalid %s X coordinate: %s", prefix, coordStr)
		}
		y, err := strconv.ParseFloat(strings.TrimSpace(parts[1]), 64)
		if err != nil {
			return nil, fmt.Errorf("invalid %s Y coordinate: %s", prefix, coordStr)
		}

		// Get screw name (optional)
		name := fmt.Sprintf("screw at %.3f,%.3f", x, y)
		if nameStr, ok := sec[prefix+"_name"]; ok && strings.TrimSpace(nameStr) != "" {
			name = strings.TrimSpace(nameStr)
		}

		bs.screws = append(bs.screws, screw{
			coord: [2]float64{x, y},
			name:  name,
		})

		// Check for fine_adjust position
		finePrefix := prefix + "_fine_adjust"
		if fineCoordStr, ok := sec[finePrefix]; ok && strings.TrimSpace(fineCoordStr) != "" {
			parts := strings.Split(fineCoordStr, ",")
			if len(parts) != 2 {
				return nil, fmt.Errorf("invalid %s coordinate: %s", finePrefix, fineCoordStr)
			}
			fx, err := strconv.ParseFloat(strings.TrimSpace(parts[0]), 64)
			if err != nil {
				return nil, fmt.Errorf("invalid %s X coordinate: %s", finePrefix, fineCoordStr)
			}
			fy, err := strconv.ParseFloat(strings.TrimSpace(parts[1]), 64)
			if err != nil {
				return nil, fmt.Errorf("invalid %s Y coordinate: %s", finePrefix, fineCoordStr)
			}
			bs.fineAdjust = append(bs.fineAdjust, screw{
				coord: [2]float64{fx, fy},
				name:  name,
			})
		}
	}

	if len(bs.screws) < 3 {
		return nil, fmt.Errorf("bed_screws: must have at least three screws")
	}

	bs.numberOfScrews = len(bs.screws)

	return bs, nil
}

// move moves the toolhead to the specified position
func (bs *bedScrews) move(x, y, z float64, speed float64) error {
	// Get toolhead
	th := bs.rt.toolhead
	if th == nil {
		return fmt.Errorf("toolhead not available")
	}

	// Get current position
	currentPos := th.commandedPos
	newPos := make([]float64, len(currentPos))
	copy(newPos, currentPos)

	// Update coordinates that are not NaN
	if !math.IsNaN(x) {
		newPos[0] = x
	}
	if !math.IsNaN(y) {
		newPos[1] = y
	}
	if !math.IsNaN(z) {
		newPos[2] = z
	}

	// Use toolhead.move()
	return th.move(newPos, speed)
}

// moveToScrew moves to the specified screw position
func (bs *bedScrews) moveToScrew(state bedScrewsState, screwIdx int) error {
	// Determine which screw list to use
	var screws []screw
	if state == bedScrewsStateAdjust {
		screws = bs.screws
	} else if state == bedScrewsStateFine {
		screws = bs.fineAdjust
	} else {
		return fmt.Errorf("invalid state: %s", state)
	}

	if screwIdx >= len(screws) {
		return fmt.Errorf("screw index %d out of range (max %d)", screwIdx, len(screws)-1)
	}

	screw := screws[screwIdx]

	// Move up, over, then down
	if err := bs.move(math.NaN(), math.NaN(), bs.horizontalMoveZ, bs.liftSpeed); err != nil {
		return err
	}
	if err := bs.move(screw.coord[0], screw.coord[1], bs.horizontalMoveZ, bs.speed); err != nil {
		return err
	}
	if err := bs.move(screw.coord[0], screw.coord[1], bs.probeZ, bs.liftSpeed); err != nil {
		return err
	}

	// Update state
	bs.state = state
	bs.currentScrew = screwIdx

	// Register dynamic commands
	bs.acceptActive = true
	bs.adjustedActive = true
	bs.abortActive = true

	// Respond with prompt
	bs.rt.tracef("Adjust %s. Then run ACCEPT, ADJUSTED, or ABORT\n", screw.name)
	bs.rt.tracef("Use ADJUSTED if a significant screw adjustment is made\n")

	return nil
}

// unregisterCommands unregisters the dynamic commands
func (bs *bedScrews) unregisterCommands() {
	bs.acceptActive = false
	bs.adjustedActive = false
	bs.abortActive = false
}

// reset resets the state machine
func (bs *bedScrews) reset() {
	bs.state = bedScrewsStateInactive
	bs.currentScrew = 0
	bs.acceptedScrews = 0
	bs.unregisterCommands()
}

// cmdBED_SCREWS_ADJUST handles the BED_SCREWS_ADJUST command
func (bs *bedScrews) cmdBED_SCREWS_ADJUST(args map[string]string) error {
	if bs.state != bedScrewsStateInactive {
		return fmt.Errorf("already in bed_screws helper; use ABORT to exit")
	}

	// Reset accepted screws counter
	bs.acceptedScrews = 0

	// Move to horizontal move Z
	if err := bs.move(math.NaN(), math.NaN(), bs.horizontalMoveZ, bs.speed); err != nil {
		return err
	}

	// Move to first screw
	return bs.moveToScrew(bedScrewsStateAdjust, 0)
}

// cmdACCEPT handles the ACCEPT command
func (bs *bedScrews) cmdACCEPT(args map[string]string) error {
	bs.unregisterCommands()
	bs.acceptedScrews++

	var screws []screw
	if bs.state == bedScrewsStateAdjust {
		screws = bs.screws
	} else if bs.state == bedScrewsStateFine {
		screws = bs.fineAdjust
	} else {
		return fmt.Errorf("invalid state: %s", bs.state)
	}

	// Check if we should continue with next screw
	if bs.currentScrew+1 < len(screws) && bs.acceptedScrews < bs.numberOfScrews {
		// Continue with next screw
		return bs.moveToScrew(bs.state, bs.currentScrew+1)
	}

	// Check if we need to retry coarse adjustments
	if bs.acceptedScrews < bs.numberOfScrews {
		// Retry from beginning
		return bs.moveToScrew(bedScrewsStateAdjust, 0)
	}

	// Check if we should switch to fine adjustment
	if bs.state == bedScrewsStateAdjust && len(bs.fineAdjust) > 0 {
		// Reset accepted screws for fine adjustment
		bs.acceptedScrews = 0
		// Perform fine screw adjustments
		return bs.moveToScrew(bedScrewsStateFine, 0)
	}

	// Done - reset and move up
	bs.reset()
	if err := bs.move(math.NaN(), math.NaN(), bs.horizontalMoveZ, bs.liftSpeed); err != nil {
		return err
	}

	bs.rt.tracef("Bed screws tool completed successfully\n")
	return nil
}

// cmdADJUSTED handles the ADJUSTED command
func (bs *bedScrews) cmdADJUSTED(args map[string]string) error {
	bs.unregisterCommands()
	bs.acceptedScrews = -1 // Force retry from beginning
	return bs.cmdACCEPT(args)
}

// cmdABORT handles the ABORT command
func (bs *bedScrews) cmdABORT(args map[string]string) error {
	bs.unregisterCommands()
	bs.reset()
	return nil
}

// isActive returns true if bed_screws adjustment is active
func (bs *bedScrews) isActive() bool {
	return bs.state != bedScrewsStateInactive
}

// getDynamicCommand returns a handler for a dynamic command if active
func (bs *bedScrews) getDynamicCommand(name string) (func(map[string]string) error, bool) {
	if name == "ACCEPT" && bs.acceptActive {
		return bs.cmdACCEPT, true
	}
	if name == "ADJUSTED" && bs.adjustedActive {
		return bs.cmdADJUSTED, true
	}
	if name == "ABORT" && bs.abortActive {
		return bs.cmdABORT, true
	}
	return nil, false
}
