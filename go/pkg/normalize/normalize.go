// Normalization utilities for Go host migration golden output comparison.
//
// This package provides a configuration-driven approach to normalizing
// MCU command ordering differences between Python klippy and Go host implementations.
// It replaces test-specific hardcoded fix functions with a more maintainable, extensible system.
//
// Design Goals:
// - Declarative normalization rules instead of imperative code
// - Easy to add new rules without modifying existing logic
// - Test-specific rule registration
// - Clear separation between normalization logic and test harness
//
// Usage:
//
//	// Create a normalizer for a specific test
//	normalizer := normalize.New("bltouch")
//
//	// Register rules for this test
//	normalizer.Register(normalize.ReorderRule{
//		Pattern: []string{
//			"endstop_home oid=0 clock=4000000",
//			"queue_digital_out oid=12 clock=4032000 on_ticks=0",
//			"set_next_step_dir oid=2 dir=0",
//			"queue_step oid=2 interval=4065333 count=1 add=0",
//		},
//		Reorder: []int{0, 2, 3, 1}, // Reorder lines: [0,1,2,3] -> [0,2,3,1]
//	})
//
//	// Apply normalization
//	normalizedLines := normalizer.Apply(originalLines)
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package normalize

import (
	"strings"
)

// RuleType defines the type of normalization rule.
type RuleType int

const (
	// ReorderRuleType reorders a sequence of lines according to an index map.
	ReorderRuleType RuleType = iota
	
	// RemoveRuleType removes a specific line or sequence.
	RemoveRuleType
	
	// InsertRuleType inserts lines at a specific position.
	InsertRuleType
	
	// ReplaceRuleType replaces a line pattern with new content.
	ReplaceRuleType
)

// Rule represents a single normalization rule.
type Rule struct {
	// Type of normalization to apply
	Type RuleType
	
	// Pattern to match (sequence of strings, can use prefixes)
	Pattern []string
	
	// For ReorderRuleType: New order indices (0-based)
	Reorder []int
	
	// For RemoveRuleType: Remove matched lines
	
	// For InsertRuleType: Lines to insert after match
	Insert []string
	
	// For ReplaceRuleType: Replacement lines
	Replace []string
	
	// Description of what this rule does (for documentation)
	Description string
}

// Normalizer manages normalization rules for a specific test.
type Normalizer struct {
	name  string
	rules []Rule
}

// New creates a new Normalizer for the given test name.
func New(testName string) *Normalizer {
	return &Normalizer{
		name:  testName,
		rules: make([]Rule, 0),
	}
}

// Register adds a normalization rule to this normalizer.
func (n *Normalizer) Register(rule Rule) {
	rule.Description = strings.TrimSpace(rule.Description)
	n.rules = append(n.rules, rule)
}

// Apply applies all registered rules to the given lines and returns normalized output.
func (n *Normalizer) Apply(lines []string) []string {
	result := make([]string, len(lines))
	copy(result, lines)
	
	for _, rule := range n.rules {
		switch rule.Type {
		case ReorderRuleType:
			result = n.applyReorder(result, rule)
		case RemoveRuleType:
			result = n.applyRemove(result, rule)
		case InsertRuleType:
			result = n.applyInsert(result, rule)
		case ReplaceRuleType:
			result = n.applyReplace(result, rule)
		}
	}
	
	return result
}

// applyReorder applies a reorder rule.
func (n *Normalizer) applyReorder(lines []string, rule Rule) []string {
	if len(rule.Pattern) == 0 || len(rule.Reorder) == 0 {
		return lines
	}
	
	out := make([]string, 0, len(lines))
	for i := 0; i < len(lines); i++ {
		// Check if this position matches the pattern
		if i+len(rule.Pattern) > len(lines) {
			out = append(out, lines[i:]...)
			break
		}
		
		matched := true
		for j, pattern := range rule.Pattern {
			if !strings.HasPrefix(lines[i+j], pattern) {
				matched = false
				break
			}
		}
		
		if matched {
			// Reorder the matched block
			for _, idx := range rule.Reorder {
				if idx >= 0 && idx < len(rule.Pattern) {
					out = append(out, lines[i+idx])
				}
			}
			i += len(rule.Pattern) - 1 // Skip the matched block
		} else {
			out = append(out, lines[i])
		}
	}
	
	return out
}

// applyRemove applies a remove rule.
func (n *Normalizer) applyRemove(lines []string, rule Rule) []string {
	if len(rule.Pattern) == 0 {
		return lines
	}
	
	out := make([]string, 0, len(lines))
	for i := 0; i < len(lines); i++ {
		// Check if this position matches the pattern
		if i+len(rule.Pattern) > len(lines) {
			out = append(out, lines[i:]...)
			break
		}
		
		matched := true
		for j, pattern := range rule.Pattern {
			if !strings.HasPrefix(lines[i+j], pattern) {
				matched = false
				break
			}
		}
		
		if matched {
			// Skip the matched block
			i += len(rule.Pattern) - 1
		} else {
			out = append(out, lines[i])
		}
	}
	
	return out
}

// applyInsert applies an insert rule.
func (n *Normalizer) applyInsert(lines []string, rule Rule) []string {
	if len(rule.Pattern) == 0 {
		return lines
	}
	
	out := make([]string, 0, len(lines)+len(rule.Insert))
	for i := 0; i < len(lines); i++ {
		out = append(out, lines[i])
		
		// Check if this position matches the pattern (after inserting current line)
		if i+len(rule.Pattern) > len(lines) {
			continue
		}
		
		matched := true
		for j, pattern := range rule.Pattern {
			if i+1+j >= len(lines) || !strings.HasPrefix(lines[i+1+j], pattern) {
				matched = false
				break
			}
		}
		
		if matched {
			// Insert the new lines after current position
			out = append(out, rule.Insert...)
		}
	}
	
	return out
}

// applyReplace applies a replace rule.
func (n *Normalizer) applyReplace(lines []string, rule Rule) []string {
	if len(rule.Pattern) == 0 {
		return lines
	}
	
	out := make([]string, 0, len(lines))
	for i := 0; i < len(lines); i++ {
		// Check if this position matches the pattern
		if i+len(rule.Pattern) > len(lines) {
			out = append(out, lines[i:]...)
			break
		}
		
		matched := true
		for j, pattern := range rule.Pattern {
			if !strings.HasPrefix(lines[i+j], pattern) {
				matched = false
				break
			}
		}
		
		if matched {
			// Replace with new content
			out = append(out, rule.Replace...)
			i += len(rule.Pattern) - 1 // Skip the matched block
		} else {
			out = append(out, lines[i])
		}
	}
	
	return out
}
