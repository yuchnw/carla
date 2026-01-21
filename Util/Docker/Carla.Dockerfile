FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libvulkan1 \
    libgl1 \
    libx11-6 \
    libxcomposite1 \
    libxrandr2 \
    libxi6 \
    libfreetype6 \
    libpng16-16 \
    python3 \
    python3-pip \
    xdg-user-dirs \
 && rm -rf /var/lib/apt/lists/*

# Copy CARLA packaged binary
COPY Dist/CARLA_Shipping_0.9.15.2-6-g7b74a9853-dirty /opt/carla
COPY ScanPatterns.yaml /opt/carla/

RUN useradd -m carlauser && chown -R carlauser /opt/carla
USER carlauser
