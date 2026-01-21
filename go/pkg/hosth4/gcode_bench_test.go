// Benchmark tests for G-code parsing performance
//
// Copyright (C) 2026 Go Migration Team
//
// This file may be distributed under the terms of the GNU GPLv3 license.

package hosth4

import (
	"testing"
)

// Sample G-code lines for benchmarking
var benchGCodeLines = []string{
	"G1 X100 Y200 Z0.2 F3000",            // Common move command
	"G1 X50.5 Y75.25 E1.234",             // Move with extrusion
	"G28 X Y",                             // Home command
	"M104 S200",                          // Set temperature
	"M109 S200",                          // Wait for temperature
	"SET_VELOCITY_LIMIT VELOCITY=300 ACCEL=3000",
	"; this is a comment",
	"G1 X10 ; inline comment",
	"G1 X10 (parenthesis comment) Y20",
	"SAVE_GCODE_STATE NAME=my_state",
}

func BenchmarkParseGCodeLine_G1(b *testing.B) {
	line := "G1 X100 Y200 Z0.2 F3000"
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, _ = parseGCodeLine(line)
	}
}

func BenchmarkParseGCodeLine_G1_WithExtrude(b *testing.B) {
	line := "G1 X50.5 Y75.25 E1.234"
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, _ = parseGCodeLine(line)
	}
}

func BenchmarkParseGCodeLine_ExtendedCommand(b *testing.B) {
	line := "SET_VELOCITY_LIMIT VELOCITY=300 ACCEL=3000 ACCEL_TO_DECEL=1500"
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, _ = parseGCodeLine(line)
	}
}

func BenchmarkParseGCodeLine_WithComment(b *testing.B) {
	line := "G1 X10 ; inline comment"
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, _ = parseGCodeLine(line)
	}
}

func BenchmarkParseGCodeLine_WithParenComment(b *testing.B) {
	line := "G1 X10 (parenthesis comment) Y20"
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, _ = parseGCodeLine(line)
	}
}

func BenchmarkParseGCodeLine_Mixed(b *testing.B) {
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		line := benchGCodeLines[i%len(benchGCodeLines)]
		_, _ = parseGCodeLine(line)
	}
}

func BenchmarkFloatArg(b *testing.B) {
	args := map[string]string{
		"X": "100.5",
		"Y": "200.75",
		"Z": "0.2",
		"F": "3000",
	}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, _ = floatArg(args, "X", 0)
		_, _ = floatArg(args, "Y", 0)
		_, _ = floatArg(args, "Z", 0)
		_, _ = floatArg(args, "F", 0)
	}
}

func BenchmarkFloatArg_Missing(b *testing.B) {
	args := map[string]string{
		"X": "100.5",
	}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, _ = floatArg(args, "Y", -1)
	}
}

// Memory allocation tests

func BenchmarkParseGCodeLine_Allocs(b *testing.B) {
	line := "G1 X100 Y200 Z0.2 F3000"
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		_, _ = parseGCodeLine(line)
	}
}

// Parallel benchmarks to test under concurrent load

func BenchmarkParseGCodeLine_Parallel(b *testing.B) {
	line := "G1 X100 Y200 Z0.2 F3000"
	b.RunParallel(func(pb *testing.PB) {
		for pb.Next() {
			_, _ = parseGCodeLine(line)
		}
	})
}

// Fast parser benchmarks

func BenchmarkParseGCodeLineFast_G1(b *testing.B) {
	line := "G1 X100 Y200 Z0.2 F3000"
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		cmd, _ := parseGCodeLineFast(line)
		if cmd != nil {
			ReleaseGCodeCommand(cmd)
		}
	}
}

func BenchmarkParseGCodeLineFast_ExtendedCommand(b *testing.B) {
	line := "SET_VELOCITY_LIMIT VELOCITY=300 ACCEL=3000 ACCEL_TO_DECEL=1500"
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		cmd, _ := parseGCodeLineFast(line)
		if cmd != nil {
			ReleaseGCodeCommand(cmd)
		}
	}
}

func BenchmarkParseGCodeLineFast_Allocs(b *testing.B) {
	line := "G1 X100 Y200 Z0.2 F3000"
	b.ReportAllocs()
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		cmd, _ := parseGCodeLineFast(line)
		if cmd != nil {
			ReleaseGCodeCommand(cmd)
		}
	}
}

func BenchmarkFastG1Parser(b *testing.B) {
	line := "G1 X100 Y200 Z0.2 F3000"
	var parser FastG1Parser
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		parser.Parse(line)
	}
}

func BenchmarkFastG1Parser_AllParams(b *testing.B) {
	line := "G1 X100.5 Y200.75 Z0.2 E1.234 F3000"
	var parser FastG1Parser
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		parser.Parse(line)
	}
}

func BenchmarkFastG1Parser_Parallel(b *testing.B) {
	line := "G1 X100 Y200 Z0.2 F3000"
	b.RunParallel(func(pb *testing.PB) {
		var parser FastG1Parser
		for pb.Next() {
			parser.Parse(line)
		}
	})
}

// Comparison: original vs fast vs specialized
func BenchmarkComparison_Original(b *testing.B) {
	line := "G1 X100 Y200 Z0.2 F3000"
	for i := 0; i < b.N; i++ {
		cmd, _ := parseGCodeLine(line)
		if cmd != nil {
			_, _ = floatArg(cmd.Args, "X", 0)
			_, _ = floatArg(cmd.Args, "Y", 0)
			_, _ = floatArg(cmd.Args, "Z", 0)
			_, _ = floatArg(cmd.Args, "F", 0)
		}
	}
}

func BenchmarkComparison_Fast(b *testing.B) {
	line := "G1 X100 Y200 Z0.2 F3000"
	for i := 0; i < b.N; i++ {
		cmd, _ := parseGCodeLineFast(line)
		if cmd != nil {
			_, _ = floatArgFast(cmd.Args, "X", 0)
			_, _ = floatArgFast(cmd.Args, "Y", 0)
			_, _ = floatArgFast(cmd.Args, "Z", 0)
			_, _ = floatArgFast(cmd.Args, "F", 0)
			ReleaseGCodeCommand(cmd)
		}
	}
}

func BenchmarkComparison_Specialized(b *testing.B) {
	line := "G1 X100 Y200 Z0.2 F3000"
	var parser FastG1Parser
	for i := 0; i < b.N; i++ {
		if parser.Parse(line) {
			_, _ = parser.X()
			_, _ = parser.Y()
			_, _ = parser.Z()
			_, _ = parser.F()
		}
	}
}
