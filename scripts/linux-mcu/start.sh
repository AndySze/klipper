#!/bin/sh
# Start script for Klipper Linux MCU in Docker
# Creates a TCP port that can be accessed from the host

set -e

INTERNAL_PTY=/tmp/klipper_pty
TCP_PORT="${MCU_PORT:-5555}"

# Start klipper MCU in background with internal PTY
/klipper/out/klipper.elf -I "$INTERNAL_PTY" &
MCU_PID=$!

# Wait for PTY to be created
echo "Waiting for MCU to start..."
for i in $(seq 1 50); do
    if [ -e "$INTERNAL_PTY" ]; then
        break
    fi
    sleep 0.1
done

if [ ! -e "$INTERNAL_PTY" ]; then
    echo "Error: MCU failed to create PTY at $INTERNAL_PTY"
    exit 1
fi

# Resolve the symlink to get actual PTY device
PTY_DEVICE=$(readlink -f "$INTERNAL_PTY")
echo "MCU PTY: $PTY_DEVICE"
echo "TCP Port: $TCP_PORT"

# Use socat to bridge PTY to TCP port
echo "Starting TCP bridge..."
socat TCP-LISTEN:$TCP_PORT,fork,reuseaddr FILE:"$PTY_DEVICE",raw,echo=0 &
SOCAT_PID=$!

echo "Linux MCU ready on TCP port $TCP_PORT"
echo "Connect with: socat - TCP:localhost:$TCP_PORT"
echo "Press Ctrl+C to stop"

# Handle shutdown
cleanup() {
    echo "Shutting down..."
    kill $SOCAT_PID 2>/dev/null || true
    kill $MCU_PID 2>/dev/null || true
    exit 0
}
trap cleanup INT TERM

# Wait for MCU process
wait $MCU_PID
