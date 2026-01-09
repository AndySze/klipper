#!/usr/bin/env python3
# Trace analyzer for comparing Go and Python flush timing
#
# This script analyzes trace output from Go host to understand
# the exact timing of flush events and step generation.

import sys
import re
from collections import defaultdict

def parse_go_trace(trace_file):
    """Parse Go trace output and extract flush events."""
    events = []

    with open(trace_file, 'r') as f:
        for line in f:
            line = line.strip()

            # Parse flush events
            if 'MQ advance begin' in line:
                m = re.search(r'want_flush=([\d.]+) want_sg=([\d.]+) need_flush=([\d.]+) need_sg=([\d.]+) last_flush=([\d.]+) last_sg=([\d.]+)', line)
                if m:
                    events.append({
                        'type': 'advance_begin',
                        'want_flush': float(m.group(1)),
                        'want_sg': float(m.group(2)),
                        'need_flush': float(m.group(3)),
                        'need_sg': float(m.group(4)),
                        'last_flush': float(m.group(5)),
                        'last_sg': float(m.group(6)),
                    })

            elif 'MQ advance end' in line:
                m = re.search(r'flush=([\d.]+) sg=([\d.]+) clear=([\d.]+)', line)
                if m:
                    events.append({
                        'type': 'advance_end',
                        'flush': float(m.group(1)),
                        'sg': float(m.group(2)),
                        'clear': float(m.group(3)),
                    })

            elif 'MQ note' in line:
                m = re.search(r'mqTime=([\d.]+) stepgen=(\w+) need_flush=([\d.]+) need_sg=([\d.]+) last_flush=([\d.]+) last_sg=([\d.]+)', line)
                if m:
                    events.append({
                        'type': 'note',
                        'mqTime': float(m.group(1)),
                        'is_stepgen': m.group(2) == 'true',
                        'need_flush': float(m.group(3)),
                        'need_sg': float(m.group(4)),
                        'last_flush': float(m.group(5)),
                        'last_sg': float(m.group(6)),
                    })

            elif 'MQ flushAll' in line:
                m = re.search(r'need_sg=([\d.]+) need_flush=([\d.]+) last_flush=([\d.]+) last_sg=([\d.]+)', line)
                if m:
                    events.append({
                        'type': 'flush_all',
                        'need_sg': float(m.group(1)),
                        'need_flush': float(m.group(2)),
                        'last_flush': float(m.group(3)),
                        'last_sg': float(m.group(4)),
                    })

            elif 'flushHandlerDebugOnce' in line:
                m = re.search(r'last_sg=([\d.]+) faux=([\d.]+) target=([\d.]+) batch=([\d.]+)', line)
                if m:
                    events.append({
                        'type': 'flush_handler',
                        'last_sg': float(m.group(1)),
                        'faux': float(m.group(2)),
                        'target': float(m.group(3)),
                        'batch': float(m.group(4)),
                    })

    return events

def analyze_batch_boundaries(events):
    """Analyze batch boundaries to find timing patterns."""
    print("\n=== Batch Boundary Analysis ===\n")

    advance_events = [e for e in events if e['type'] == 'advance_begin']
    note_events = [e for e in events if e['type'] == 'note']

    print(f"Total advance events: {len(advance_events)}")
    print(f"Total note events: {len(note_events)}")

    # Find batch boundaries (every ~0.25 seconds)
    if len(advance_events) > 1:
        print("\nFirst 10 advance events:")
        for i, e in enumerate(advance_events[:10]):
            print(f"  [{i}] want_flush={e['want_flush']:.6f} want_sg={e['want_sg']:.6f} "
                  f"last_flush={e['last_flush']:.6f} last_sg={e['last_sg']:.6f}")

        # Calculate intervals
        intervals = []
        for i in range(1, len(advance_events)):
            interval = advance_events[i]['want_flush'] - advance_events[i-1]['want_flush']
            intervals.append(interval)

        print(f"\nFlush intervals (first 10):")
        for i, interval in enumerate(intervals[:10]):
            print(f"  [{i}] {interval:.6f}s ({interval*1000:.3f}ms)")

        # Check for 0.25s pattern
        batch_intervals = [i for i in intervals if abs(i - 0.25) < 0.01]
        print(f"\nIntervals close to 0.25s: {len(batch_intervals)}/{len(intervals)}")

def analyze_eof_timing(events):
    """Analyze EOF timing to understand motor-off offset."""
    print("\n=== EOF Timing Analysis ===\n")

    # Find last flush events
    advance_ends = [e for e in events if e['type'] == 'advance_end']
    if advance_ends:
        last = advance_ends[-1]
        print(f"Last advance_end:")
        print(f"  flush time: {last['flush']:.9f}s")
        print(f"  step_gen time: {last['sg']:.9f}s")
        print(f"  clear time: {last['clear']:.9f}s")

        # Calculate expected motor-off time (flush time + some offset)
        # In Python: motor-off happens at last_flush_time + small_offset
        # The difference we see is ~0.022666s

def find_divergence_point(go_trace, expected_pattern=None):
    """Find where Go timing diverges from expected pattern."""
    print("\n=== Looking for Timing Divergence ===\n")

    events = parse_go_trace(go_trace)

    # Check for specific patterns in gcode_arcs
    # The issue is around interval=8000 boundaries

    # Look for step generation times that might indicate batch misalignment
    advance_events = [e for e in events if e['type'] == 'advance_begin']

    print("Checking for batch boundary alignment issues...")

    # In gcode_arcs, we expect regular batching at ~0.25s intervals
    # Look for irregular intervals
    if len(advance_events) > 2:
        irregular = []
        for i in range(2, min(len(advance_events), 50)):
            curr = advance_events[i]
            prev = advance_events[i-1]
            interval = curr['want_flush'] - prev['want_flush']

            # Check if interval is significantly different from 0.25s
            if abs(interval - 0.25) > 0.01:
                irregular.append({
                    'index': i,
                    'interval': interval,
                    'curr_flush': curr['want_flush'],
                    'prev_flush': prev['want_flush'],
                })

        if irregular:
            print(f"\nFound {len(irregular)} irregular intervals (first 5):")
            for ir in irregular[:5]:
                print(f"  [{ir['index']}] interval={ir['interval']:.6f}s "
                      f"prev={ir['prev_flush']:.6f} curr={ir['curr_flush']:.6f}")
        else:
            print("  No irregular intervals found in first 50 events")

    analyze_batch_boundaries(events)
    analyze_eof_timing(events)

def main():
    if len(sys.argv) < 2:
        print("Usage: trace_analyzer.py <trace_file>")
        print("\nThis script analyzes Go trace output to find timing divergences.")
        print("\nTo generate trace:")
        print("  cd go")
        print("  GOCACHE=../out/go-build-cache GOPATH=../out/go-path CGO_ENABLED=1 \\")
        print("    go run ./cmd/klipper-go-golden -mode host-h4 -only gcode_arcs \\")
        print("    -dictdir ../dict -trace > ../trace_go.txt 2>&1")
        sys.exit(1)

    trace_file = sys.argv[1]
    find_divergence_point(trace_file)

if __name__ == '__main__':
    main()
