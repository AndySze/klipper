# Dockerfile for building and running Klipper Linux MCU simulator
FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y \
    build-essential \
    python3 \
    libncurses-dev \
    socat \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /klipper

# Copy source code
COPY . .

# Build Linux MCU
RUN cp test/configs/linuxprocess.config .config \
    && make olddefconfig \
    && make -j$(nproc)

# Copy startup script
COPY scripts/linux-mcu/start.sh /start.sh
RUN chmod +x /start.sh

# Default socket path
ENV MCU_SOCKET=/tmp/klipper_mcu

# Run the MCU simulator with socket bridge
CMD ["/start.sh"]
