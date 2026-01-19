// Example usage and tests for the normalize package.
//
// This file demonstrates how to use the normalize package and provides
// basic tests for rule application.
//
// This file may be distributed under terms of GNU GPLv3 license.

package normalize

import (
	"fmt"
	"testing"
)

// Example: Basic reorder rule
func ExampleNormalizer_Register_reorder() {
	normalizer := New("example")
	
	normalizer.Register(Rule{
		Type: ReorderRuleType,
		Pattern: []string{"A", "B", "C"},
		Reorder: []int{0, 2, 1}, // [A,B,C] -> [A,C,B]
		Description: "Swap B and C",
	})
	
	input := []string{"A", "B", "C", "D", "E"}
	output := normalizer.Apply(input)
	
	for _, line := range output {
		fmt.Println(line)
	}
	// Output:
	// A
	// C
	// B
	// D
	// E
}

// Test basic reorder rule
func TestBasicReorder(t *testing.T) {
	normalizer := New("test")
	
	normalizer.Register(Rule{
		Type: ReorderRuleType,
		Pattern: []string{"first", "second", "third"},
		Reorder: []int{0, 2, 1},
		Description: "Test reorder",
	})
	
	input := []string{
		"first line",
		"second line",
		"third line",
		"fourth line",
	}
	
	expected := []string{
		"first line",
		"third line",
		"second line",
		"fourth line",
	}
	
	result := normalizer.Apply(input)
	
	if len(result) != len(expected) {
		t.Fatalf("Expected %d lines, got %d", len(expected), len(result))
	}
	
	for i := range expected {
		if result[i] != expected[i] {
			t.Errorf("Line %d: expected %q, got %q", i, expected[i], result[i])
		}
	}
}

// Test prefix matching
func TestPrefixMatching(t *testing.T) {
	normalizer := New("test")
	
	normalizer.Register(Rule{
		Type: ReorderRuleType,
		Pattern: []string{"command_one", "command_two param"},
		Reorder: []int{1, 0},
		Description: "Test prefix matching",
	})
	
	input := []string{
		"command_one value=123",
		"command_two param=456",
		"other line",
	}
	
	result := normalizer.Apply(input)
	
	// Pattern should match even with extra text
	// The order should be swapped
	if len(result) != 3 {
		t.Fatalf("Expected 3 lines, got %d", len(result))
	}
}

// Test multiple rules
func TestMultipleRules(t *testing.T) {
	normalizer := New("test")
	
	// Rule 1: Swap first two lines
	normalizer.Register(Rule{
		Type: ReorderRuleType,
		Pattern: []string{"A", "B"},
		Reorder: []int{1, 0},
		Description: "Swap A and B",
	})
	
	// Rule 2: Swap next two lines
	normalizer.Register(Rule{
		Type: ReorderRuleType,
		Pattern: []string{"C", "D"},
		Reorder: []int{1, 0},
		Description: "Swap C and D",
	})
	
	input := []string{"A", "B", "C", "D"}
	result := normalizer.Apply(input)
	
	expected := []string{"B", "A", "D", "C"}
	
	for i := range expected {
		if result[i] != expected[i] {
			t.Errorf("Line %d: expected %q, got %q", i, expected[i], result[i])
		}
	}
}

// Test out_of_bounds rule
func TestOutOfBoundsRule(t *testing.T) {
	normalizer := New("out_of_bounds")
	RegisterOutOfBoundsRules(normalizer)
	
	input := []string{
		"endstop_home oid=0 clock=4000000",
		"queue_digital_out oid=12 clock=4032000 on_ticks=0",
		"set_next_step_dir oid=2 dir=0",
		"queue_step oid=2 interval=4065333 count=1 add=0",
	}
	
	result := normalizer.Apply(input)
	
	// Expected: endstop_home, set_next_step_dir, queue_step, queue_digital_out
	expected := []string{
		"endstop_home oid=0 clock=4000000",
		"set_next_step_dir oid=2 dir=0",
		"queue_step oid=2 interval=4065333 count=1 add=0",
		"queue_digital_out oid=12 clock=4032000 on_ticks=0",
	}
	
	for i := range expected {
		if result[i] != expected[i] {
			t.Errorf("Line %d: expected %q, got %q", i, expected[i], result[i])
		}
	}
}

// Test GetNormalizerForTest
func TestGetNormalizerForTest(t *testing.T) {
	tests := []struct {
		name       string
		shouldHaveRules bool
	}{
		{"out_of_bounds", true},
		{"bed_screws", true},
		{"gcode_arcs", true},
		{"bltouch", true},
		{"commands", false},
		{"unknown_test", false},
	}
	
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			n := GetNormalizerForTest(tt.name)
			
			if tt.shouldHaveRules {
				if n == nil {
					t.Errorf("Expected normalizer for %s", tt.name)
				}
			} else {
				if n != nil && len(n.rules) > 0 {
					t.Errorf("Expected no rules for %s", tt.name)
				}
			}
		})
	}
}
