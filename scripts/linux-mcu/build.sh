#!/bin/bash
# Build the Linux MCU Docker image
#
# Usage: ./build.sh
#
# This builds a Docker image containing the Klipper Linux MCU simulator.
# The MCU communicates via a Unix socket that can be mounted from the host.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KLIPPER_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"

cd "$KLIPPER_DIR"

echo "Building Linux MCU Docker image..."
docker build -t klipper-linux-mcu -f scripts/linux-mcu/Dockerfile .

echo ""
echo "Build complete. Run with: ./scripts/linux-mcu/run.sh"
