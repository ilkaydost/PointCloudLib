# Multi-stage build to reduce image size and build time
FROM ubuntu:20.04 AS builder

ENV DEBIAN_FRONTEND=noninteractive

# Install build tools and development packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget zip unzip tar curl pkg-config \
    python3 python3-pip python3-distutils python3-venv \
    libssl-dev ca-certificates ninja-build ccache \
    libpcl-dev \
    nlohmann-json3-dev \
    libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

# Download cpp-httplib (header-only library)
RUN mkdir -p /opt/cpp-httplib && \
    curl -L https://github.com/yhirose/cpp-httplib/archive/refs/tags/v0.11.4.tar.gz | \
    tar -xz -C /opt/cpp-httplib --strip-components=1

# Enable ccache for faster rebuilds
ENV PATH=/usr/lib/ccache:$PATH
RUN ccache -M 5G

WORKDIR /workspace

# Copy project sources
COPY . /workspace

# Configure and build with CMake using system libraries
RUN cmake -B build -S . \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_PREFIX_PATH=/opt/cpp-httplib \
    -DCMAKE_CXX_COMPILER_LAUNCHER=ccache \
    -DCMAKE_C_COMPILER_LAUNCHER=ccache \
    && cmake --build build -- -j$(nproc)

# Runtime stage - smaller image
FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# Install only runtime dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    libpcl-common1.10 \
    libpcl-io1.10 \
    libpcl-filters1.10 \
    libpcl-features1.10 \
    libpcl-segmentation1.10 \
    libboost-system1.71.0 \
    libboost-filesystem1.71.0 \
    ca-certificates \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

# Copy only the built binary from builder
COPY --from=builder /workspace/build/pointcloud_server /workspace/pointcloud_server

EXPOSE 5050

# Run the built server binary
CMD ["/workspace/pointcloud_server"]
