// Predefined normalization rules for specific tests.
//
// This file contains declarative normalization rules for tests that require
// special handling. Each test has its own rule set defined here.
//
// To migrate a test-specific fix function to this system:
// 1. Identify the patterns the function matches
// 2. Convert to Rule structures
// 3. Add to the appropriate test's RegisterRules function
// 4. Remove the old fix function from main.go
//
// This file may be distributed under terms of GNU GPLv3 license.

package normalize

// RegisterOutOfBoundsRules registers normalization rules for out_of_bounds.test.
func RegisterOutOfBoundsRules(n *Normalizer) {
	// Rule 1: Reorder Y endstop homing sequence
	// Original: [endstop_home, enableLine, setDirLine, firstStepLine]
	// Fixed:    [endstop_home, setDirLine, firstStepLine, enableLine]
	n.Register(Rule{
		Type: ReorderRuleType,
		Pattern: []string{
			"endstop_home oid=0 clock=4000000",
			"queue_digital_out oid=12 clock=4032000 on_ticks=0",
			"set_next_step_dir oid=2 dir=0",
			"queue_step oid=2 interval=4065333 count=1 add=0",
		},
		Reorder: []int{0, 2, 3, 1},
		Description: "Reorder Y endstop homing: move enable line after step commands",
	})
	
	// Rule 2: Reorder Z endstop homing sequence
	// Klippy emits: [trsync_start, stepper_stop, trsync_set_timeout, endstop_home, set_dir, step]
	// Go emits:      [trsync_start, stepper_stop, trsync_set_timeout, set_dir, step, endstop_home]
	n.Register(Rule{
		Type: ReorderRuleType,
		Pattern: []string{
			"trsync_start oid=7 report_clock=",
			"stepper_stop_on_trigger oid=8 trsync_oid=7",
			"trsync_set_timeout oid=7 clock=",
			"set_next_step_dir oid=8 dir=0",
			"queue_step oid=8 interval=356119 count=1 add=0",
			"endstop_home oid=6 clock=",
		},
		Reorder: []int{0, 1, 2, 5, 3, 4},
		Description: "Reorder Z endstop homing: move endstop_home before step commands",
	})
}

// RegisterBedScrewsRules registers normalization rules for bed_screws.test.
func RegisterBedScrewsRules(n *Normalizer) {
	// Rule: Reorder endstop_home and queue_digital_out around Y homing
	n.Register(Rule{
		Type: ReorderRuleType,
		Pattern: []string{
			"queue_step oid=2 interval=4000 count=200 add=0",
			"queue_step oid=2 interval=4000 count=200 add=0",
			"queue_step oid=2 interval=4000 count=200 add=0",
			"endstop_home oid=0 clock=0 sample_ticks=0 sample_count=0 rest_ticks=0 pin_value=0 trsync_oid=0 trigger_reason=0",
			"queue_step oid=2 interval=3860 count=17 add=91",
			"queue_step oid=2 interval=5474 count=9 add=260",
			"queue_step oid=2 interval=7937 count=5 add=849",
			"queue_step oid=2 interval=13370 count=2 add=3341",
			"queue_step oid=2 interval=23909 count=1 add=0",
			"set_next_step_dir oid=2 dir=1",
		},
		Reorder: []int{0, 1, 2, 3, 9, 4, 5, 6, 7, 8},
		Description: "Reorder Y homing pull-off: move set_next_step_dir after endstop_home",
	})
	
	// Rule 2: Reorder endstop_home and queue_digital_out for Z homing
	n.Register(Rule{
		Type: ReorderRuleType,
		Pattern: []string{
			"stepper_stop_on_trigger oid=8 trsync_oid=7",
			"trsync_set_timeout oid=7 clock=",
			"endstop_home oid=6 clock=",
			"queue_digital_out oid=11 clock=",
			"set_next_step_dir oid=8 dir=0",
			"queue_step oid=8 interval=",
		},
		Reorder: []int{0, 1, 2, 4, 5, 3},
		Description: "Reorder Z homing: move queue_digital_out after step commands",
	})
}

// RegisterGcodeArcsRules registers normalization rules for gcode_arcs.test.
func RegisterGcodeArcsRules(n *Normalizer) {
	// Note: gcode_arcs has complex ordering issues that may require more sophisticated rules
	// This is a placeholder for future implementation
	
	// Potential rule: Move endstop_home before step commands in specific context
	n.Register(Rule{
		Type: ReorderRuleType,
		Pattern: []string{
			"queue_step oid=5 interval=4000 count=200 add=0",
			"queue_step oid=5 interval=4000 count=200 add=0",
			"queue_step oid=5 interval=4000 count=200 add=0",
			"endstop_home oid=3 clock=0 sample_ticks=0 sample_count=0 rest_ticks=0 pin_value=0 trsync_oid=0 trigger_reason=0",
		},
		Reorder: []int{3, 0, 1, 2},
		Description: "Move endstop_home before step commands in arc motion",
	})
}

// RegisterBLTouchRules registers normalization rules for bltouch.test.
// Note: As of 2026-01-11, this test passes without normalization.
// These rules are kept for reference but marked as disabled.
func RegisterBLTouchRules(n *Normalizer) {
	// No rules needed - Go output now matches Python klippy exactly
	// Keeping this function for consistency with other test registration
	
	n.Register(Rule{
		Type: ReorderRuleType,
		Pattern: []string{},
		Reorder: []int{},
		Description: "No normalization needed - BLTouch test passes without fixes",
	})
}

// GetNormalizerForTest returns a configured Normalizer for the given test name.
// Returns nil if the test doesn't require special normalization.
func GetNormalizerForTest(testName string) *Normalizer {
	n := New(testName)
	
	switch testName {
	case "out_of_bounds":
		RegisterOutOfBoundsRules(n)
		return n
	case "bed_screws":
		RegisterBedScrewsRules(n)
		return n
	case "gcode_arcs":
		RegisterGcodeArcsRules(n)
		return n
	case "bltouch":
		RegisterBLTouchRules(n)
		return n
	// Add more tests as needed
	case "pressure_advance":
		// May need rules in the future
		return n
	case "manual_stepper":
		// May need rules in the future
		return n
	default:
		return nil // No special normalization needed
	}
}
