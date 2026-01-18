#!/bin/bash
# Run the Linux MCU Docker container
#
# Usage: ./run.sh [port]
#
# Arguments:
#   port - TCP port for the MCU connection (default: 5555)
#
# The container creates a TCP server that bridges to the MCU's pseudo-tty.
# Connect to it using socat or the Go host with TCP mode.

set -e

TCP_PORT="${1:-5555}"

echo "Starting Linux MCU simulator..."
echo "TCP port: $TCP_PORT"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run the container with the port exposed
docker run --rm -it \
    -p "$TCP_PORT:$TCP_PORT" \
    -e "MCU_PORT=$TCP_PORT" \
    --name klipper-linux-mcu \
    klipper-linux-mcu
