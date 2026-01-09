#!/bin/bash
# Simple script to generate Go trace for gcode_arcs

set -e

cd "$(dirname "$0")/.."

# Set absolute paths
export GOCACHE="$(pwd)/out/go-build-cache"
export GOPATH="$(pwd)/out/go-path"
export CGO_ENABLED=1

echo "Generating Go trace for gcode_arcs..."
echo

cd go
go run ./cmd/klipper-go-golden \
    -mode host-h4 \
    -only gcode_arcs \
    -dictdir ../dict \
    -trace

echo
echo "Checking for trace file..."
trace_file=$(find ../test/go_migration/golden/gcode_arcs -name "trace-*.log" 2>/dev/null | head -1)

if [ -n "$trace_file" ]; then
    echo "✓ Trace generated: $trace_file"
    echo
    echo "First 100 lines of trace:"
    head -100 "$trace_file"
    echo
    echo "Total lines: $(wc -l < "$trace_file")"
else
    echo "✗ No trace file found"
    exit 1
fi
