FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Install build tools and runtime deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential cmake git wget zip unzip tar curl pkg-config \
    python3 python3-pip python3-distutils python3-venv \
    libssl-dev ca-certificates ninja-build \
    && rm -rf /var/lib/apt/lists/*

# Set Python as default
RUN update-alternatives --install /usr/bin/python python /usr/bin/python3 1

# Install vcpkg
RUN git clone --depth=1 https://github.com/microsoft/vcpkg /opt/vcpkg \
    && /opt/vcpkg/bootstrap-vcpkg.sh -disableMetrics

ENV VCPKG_ROOT=/opt/vcpkg

WORKDIR /workspace

# Copy project sources
COPY . /workspace

# Install libraries from manifest (reads vcpkg.json) #
RUN $VCPKG_ROOT/vcpkg install --triplet x64-linux --clean-after-build

# Configure and build with CMake (uses CMakePresets.json)
RUN cmake --preset=default && cmake --build build -- -j$(nproc)

EXPOSE 5050

# Run the built server binary
CMD ["/workspace/build/pointcloud_server"]
