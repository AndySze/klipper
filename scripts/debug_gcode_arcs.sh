#!/bin/bash
# Debug script for comparing Go and Python behavior on gcode_arcs.test
#
# This script:
# 1. Generates Python trace with timing information
# 2. Generates Go trace with timing information
# 3. Analyzes both traces to find divergence points
#
# Usage: ./scripts/debug_gcode_arcs.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"

cd "$PROJECT_ROOT"

echo "=== Klipper Go Migration: gcode_arcs Debug Script ==="
echo

# Check prerequisites
if [ ! -d "dict" ]; then
    echo "Error: dict/ directory not found."
    echo "Please download MCU data dictionaries first."
    exit 1
fi

# Step 1: Generate Python trace
echo "Step 1: Generating Python trace with motion timing..."
if [ ! -f "klippy/extras/motion_queuing.py.orig" ]; then
    cp klippy/extras/motion_queuing.py klippy/extras/motion_queuing.py.orig
fi

# Apply trace patch
patch -p1 < scripts/add_python_trace.patch || true

# Run Python with trace enabled
KLIPPY_TRACE_MOTION=1 python3 scripts/go_migration_golden.py \
    gen --dictdir dict --suite test/go_migration/suites/minimal.txt \
    --only gcode_arcs 2>&1 | tee python_gen.log

# Save trace
if [ -f "/tmp/klippy_motion_trace.txt" ]; then
    cp /tmp/klippy_motion_trace.txt trace_python.txt
    echo "  ✓ Python trace saved to trace_python.txt"
else
    echo "  ✗ Failed to generate Python trace"
fi

# Step 2: Generate Go trace
echo
echo "Step 2: Generating Go trace with motion timing..."
cd go

GOCACHE="../out/go-build-cache" \
GOPATH="../out/go-path" \
CGO_ENABLED=1 \
go run ./cmd/klipper-go-golden \
    -mode host-h4 \
    -only gcode_arcs \
    -dictdir ../dict \
    -trace > ../trace_go_raw.txt 2>&1

cd ..

# Extract relevant trace lines
grep "MQ " trace_go_raw.txt > trace_go.txt 2>/dev/null || true
echo "  ✓ Go trace saved to trace_go.txt"

# Step 3: Parse expected output to find the divergence region
echo
echo "Step 3: Analyzing divergence in gcode_arcs output..."

# Find line numbers of the diff in expected.txt
PYTHON_DIFF_LINE=$(grep -n "^queue_step oid=8 interval=8000 count=499" test/go_migration/golden/gcode_arcs/expected.txt | head -1 | cut -d: -f1)
GO_DIFF_LINE=$(grep -n "^queue_step oid=8 interval=8000 count=500" test/go_migration/golden/gcode_arcs/expected.txt | head -1 | cut -d: -f1)

if [ -n "$PYTHON_DIFF_LINE" ] && [ -n "$GO_DIFF_LINE" ]; then
    echo "  First divergence found around line $PYTHON_DIFF_LINE in expected output"
    echo "  This corresponds to the interval=8000 boundary issue"
fi

# Step 4: Analyze traces
echo
echo "Step 4: Analyzing trace patterns..."

if [ -f "trace_python.txt" ]; then
    echo "  Python trace events: $(wc -l < trace_python.txt)"

    # Find batch intervals
    python3 <<'EOF'
import re

with open('trace_python.txt', 'r') as f:
    lines = f.readlines()

advance_begin_lines = [l for l in lines if 'advance begin' in l]

print(f"\n  Python: First 10 advance_begin events:")
for i, line in enumerate(advance_begin_lines[:10]):
    # Extract timing
    m = re.search(r'want_flush=([\d.]+)', line)
    if m:
        print(f"    [{i}] want_flush={m.group(1)}")

# Calculate intervals
intervals = []
for i in range(1, len(advance_begin_lines)):
    m1 = re.search(r'want_flush=([\d.]+)', advance_begin_lines[i-1])
    m2 = re.search(r'want_flush=([\d.]+)', advance_begin_lines[i])
    if m1 and m2:
        interval = float(m2.group(1)) - float(m1.group(1))
        intervals.append(interval)

print(f"\n  Python: First 20 flush intervals:")
for i, interval in enumerate(intervals[:20]):
    print(f"    [{i}] {interval:.6f}s ({interval*1000:.3f}ms)")

# Check for ~0.25s pattern
batch_pattern = [i for i in intervals if abs(i - 0.25) < 0.01]
print(f"\n  Python: Intervals close to 0.25s: {len(batch_pattern)}/{len(intervals)}")
EOF
fi

if [ -f "trace_go.txt" ]; then
    echo "  Go trace events: $(wc -l < trace_go.txt)"

    # Use the analyzer
    python3 scripts/trace_analyzer.py trace_go.txt
fi

# Step 5: Summary
echo
echo "=== Debug Summary ==="
echo
echo "Generated files:"
echo "  - trace_python.txt  : Python motion timing trace"
echo "  - trace_go.txt      : Go motion timing trace"
echo "  - trace_go_raw.txt  : Complete Go output"
echo
echo "Next steps:"
echo "  1. Compare the flush intervals between Python and Go"
echo "  2. Look for the exact point where timing diverges"
echo "  3. Check if batch boundaries align at the same print times"
echo
echo "To manually inspect:"
echo "  grep 'advance begin' trace_python.txt | head -50"
echo "  grep 'advance begin' trace_go.txt | head -50"
echo
echo "To clean up and restore original files:"
echo "  mv klippy/extras/motion_queuing.py.orig klippy/extras/motion_queuing.py"
echo "  rm trace_*.txt python_gen.log trace_go_raw.txt"
