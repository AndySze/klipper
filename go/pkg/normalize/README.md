# Auto-Normalize Package

A configuration-driven normalization system for Go host migration golden output comparison.

## Overview

This package provides a declarative, testable, and maintainable approach to normalizing MCU command ordering differences between Python klippy and Go host implementations. It replaces test-specific hardcoded fix functions (like `fixHostH4BLTouch()`) with a flexible rule-based system.

## Why This Package?

### Problems with Hardcoded Fix Functions

1. **Hard to maintain**: Each test had its own function with complex string manipulation
2. **Difficult to understand**: Logic buried in imperative code with no documentation
3. **Hard to extend**: Adding a new test meant writing a new ~150+ line function
4. **Hard to test**: No way to test normalization logic in isolation
5. **Hard to reason about**: Pattern matching and reordering logic mixed with test harness

### Benefits of Rule-Based System

1. **Declarative rules**: Clear intent at a glance
2. **Self-documenting**: Each rule has a description field
3. **Easy to add**: New rules are just data structures
4. **Testable**: Normalization logic can be unit tested
5. **Composable**: Rules can be combined and chained
6. **Clear status**: Easy to see which tests need normalization

## Usage

### Basic Example

```go
package main

import (
    "klipper-go-migration/pkg/normalize"
)

func processTestOutput(testName string, lines []string) []string {
    // Get a configured normalizer for this test
    normalizer := normalize.GetNormalizerForTest(testName)
    
    if normalizer == nil {
        // No special normalization needed
        return lines
    }
    
    // Apply all registered rules
    return normalizer.Apply(lines)
}
```

### Creating Custom Rules

```go
// Create a normalizer for a new test
normalizer := normalize.New("my_test")

// Register a reorder rule
normalizer.Register(normalize.Rule{
    Type: normalize.ReorderRuleType,
    Pattern: []string{
        "command_one param1=value1",
        "command_two param2=value2",
        "command_three param3=value3",
    },
    Reorder: []int{0, 2, 1}, // Reorder to: [0,2,1]
    Description: "Swap command_two and command_three",
})

// Apply normalization
normalized := normalizer.Apply(originalLines)
```

## Rule Types

### ReorderRuleType

Reorders a sequence of lines according to an index map.

```go
Rule{
    Type: ReorderRuleType,
    Pattern: []string{"A", "B", "C", "D"},
    Reorder: []int{0, 2, 3, 1}, // [A,B,C,D] -> [A,C,D,B]
}
```

### RemoveRuleType

Removes a specific line or sequence.

```go
Rule{
    Type: RemoveRuleType,
    Pattern: []string{"unwanted_command"},
}
```

### InsertRuleType

Inserts lines at a specific position after a pattern match.

```go
Rule{
    Type: InsertRuleType,
    Pattern: []string{"after_this_line"},
    Insert: []string{"new_command_1", "new_command_2"},
}
```

### ReplaceRuleType

Replaces matched pattern with new content.

```go
Rule{
    Type: ReplaceRuleType,
    Pattern: []string{"old_command"},
    Replace: []string{"new_command"},
}
```

## Migration Guide

### Converting Hardcoded Functions to Rules

**Before (main.go):**
```go
func fixHostH4BLTouch(lines []string) []string {
    // ~150 lines of complex string manipulation
    const ySetDirPullOff = "set_next_step_dir oid=5 dir=0"
    const yStepPullOff = "queue_step oid=5 interval=4065333 count=1 add=0"
    // ... lots of parsing and reordering logic ...
    return out
}
```

**After (rules.go):**
```go
func RegisterBLTouchRules(n *Normalizer) {
    n.Register(Rule{
        Type: ReorderRuleType,
        Pattern: []string{
            "set_next_step_dir oid=5 dir=0",
            "queue_step oid=5 interval=4065333 count=1 add=0",
        },
        Reorder: []int{0, 1}, // No reordering needed
        Description: "No normalization needed - BLTouch test passes without fixes",
    })
}
```

### Step-by-Step Migration

1. **Analyze the existing function**
   - Read the function you want to replace
   - Identify patterns it matches
   - Understand the transformation it applies

2. **Create rule definitions**
   - Create a `RegisterTestNameRules(n *Normalizer)` function
   - Convert each pattern/transformation to a `Rule`
   - Add descriptive comments

3. **Register the test**
   - Add a case in `GetNormalizerForTest()`
   - Call your registration function

4. **Test the rules**
   - Run the test and verify it passes
   - Compare output with and without rules
   - Verify description is accurate

5. **Clean up**
   - Remove the old fix function from main.go
   - Remove the call to that function
   - Update documentation

## Test Status

As of 2026-01-11:

| Test | Status | Notes |
|------|--------|-------|
| bltouch | ✅ No rules needed | Passes without normalization |
| commands | ✅ No rules needed | Passes in strict mode |
| extruders | ✅ No rules needed | Passes in strict mode |
| pressure_advance | ✅ No rules needed | Passes in strict mode |
| manual_stepper | ✅ No rules needed | Passes in strict mode |
| linuxtest | ✅ No rules needed | Passes in strict mode |
| macros | ✅ No rules needed | Passes in strict mode |
| out_of_bounds | ⚠️ Rules defined | Requires Y/Z endstop reordering |
| bed_screws | ⚠️ Rules defined | Requires homing sequence reordering |
| gcode_arcs | ⚠️ Partial rules | Complex ordering issues, needs more work |

## Future Improvements

1. **Pattern matching**: Support regex patterns instead of just prefix matching
2. **Conditional rules**: Apply rules only in specific contexts
3. **Rule priority**: Allow rules to specify priority/order
4. **Performance profiling**: Optimize rule application for large outputs
5. **Rule testing**: Unit test framework for individual rules
6. **Rule generation**: Auto-generate rules from diff output
7. **Configuration files**: Load rules from YAML/TOML instead of code

## Contributing

When adding new tests or fixing ordering issues:

1. **Run the test first** to see what differences exist
2. **Analyze the diff** to understand the pattern
3. **Add minimal rules** to fix just that pattern
4. **Document the rules** with clear descriptions
5. **Verify it passes** in strict mode
6. **Consider if the fix should be in the Go implementation instead**

Sometimes the best "normalization" is to fix the underlying Go code to produce correct output!
