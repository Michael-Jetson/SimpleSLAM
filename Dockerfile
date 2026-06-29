# syntax=docker/dockerfile:1
###############################################################################
# SimpleSLAM — reproducible C++20 build & test environment
#
# The dependency set is taken 1:1 from scripts/install_deps.sh:
#   apt    : build-essential cmake git libeigen3-dev libspdlog-dev
#            libyaml-cpp-dev libfmt-dev
#   source : Catch2 v3.4.0, manif 0.0.5, nanoflann v1.6.1, robin-map v1.3.0,
#            eventpp (pinned commit), SPSCQueue (pinned commit)
#
# Stages:
#   deps-builder  compile the from-source libraries, install into /usr/local
#   build-env     lean toolchain + apt deps + the installed /usr/local libs
#   verify        OPTIONAL (--target verify): configure + build + run ctest
#   dev           DEFAULT lean image for bind-mount development / CI
#
# Quick start:
#   docker build -t simpleslam-dev .                  # lean dev/build image
#   docker build --target verify -t simpleslam-ci .   # build + run the tests
###############################################################################

ARG UBUNTU_VERSION=24.04

# Dependency pins — override with --build-arg for an upgrade/bisect.
ARG CATCH2_REF=v3.4.0
ARG MANIF_REF=0.0.5
ARG NANOFLANN_REF=v1.6.1
ARG ROBINMAP_REF=v1.3.0
# eventpp / SPSCQueue have no upstream release tags; pin to the exact commits
# the project already vendors under docker/deps/ for byte-stable reproducibility.
ARG EVENTPP_REF=1224dd6c9bd4577d686ac42334fc545997f5ece1
ARG SPSCQUEUE_REF=1053918dbd251fbff69b24ef27fa5d51c29ec2af


###############################################################################
# Stage 1 — build the from-source dependencies into /usr/local
###############################################################################
FROM ubuntu:${UBUNTU_VERSION} AS deps-builder
ENV DEBIAN_FRONTEND=noninteractive

# build tools + git for fetching sources. libeigen3-dev is required here because
# manif's CMake config runs find_package(Eigen3) at configure time.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        git \
        ca-certificates \
        libeigen3-dev \
    && rm -rf /var/lib/apt/lists/*

ARG CATCH2_REF
ARG MANIF_REF
ARG NANOFLANN_REF
ARG ROBINMAP_REF
ARG EVENTPP_REF
ARG SPSCQUEUE_REF

WORKDIR /tmp/deps

# Catch2 3.x — unit-test framework (static libs + CMake package config)
RUN git clone --branch ${CATCH2_REF} --depth 1 https://github.com/catchorg/Catch2.git catch2 \
    && cmake -S catch2 -B catch2/build -DCMAKE_CXX_STANDARD=20 -DBUILD_TESTING=OFF \
    && cmake --build catch2/build -j"$(nproc)" \
    && cmake --install catch2/build \
    && rm -rf catch2

# manif — SE3/SO3 Lie-group library (header-only + CMake config)
RUN git clone --branch ${MANIF_REF} --depth 1 https://github.com/artivis/manif.git manif \
    && cmake -S manif -B manif/build -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF \
    && cmake --build manif/build -j"$(nproc)" \
    && cmake --install manif/build \
    && rm -rf manif

# nanoflann — header-only KD-tree for nearest-neighbour search
RUN git clone --branch ${NANOFLANN_REF} --depth 1 https://github.com/jlblancoc/nanoflann.git nanoflann \
    && cmake -S nanoflann -B nanoflann/build -DNANOFLANN_BUILD_EXAMPLES=OFF -DNANOFLANN_BUILD_TESTS=OFF \
    && cmake --build nanoflann/build -j"$(nproc)" \
    && cmake --install nanoflann/build \
    && rm -rf nanoflann

# tsl::robin_map — header-only high-performance hash map (voxel hashing)
RUN git clone --branch ${ROBINMAP_REF} --depth 1 https://github.com/Tessil/robin-map.git robin-map \
    && cmake -S robin-map -B robin-map/build \
    && cmake --build robin-map/build -j"$(nproc)" \
    && cmake --install robin-map/build \
    && rm -rf robin-map

# eventpp — header-only event dispatcher (reserved for v1.5 Topic backend)
RUN git clone https://github.com/wqking/eventpp.git eventpp \
    && git -C eventpp checkout ${EVENTPP_REF} \
    && cmake -S eventpp -B eventpp/build -DEVENTPP_INSTALL=ON \
    && cmake --build eventpp/build -j"$(nproc)" \
    && cmake --install eventpp/build \
    && rm -rf eventpp

# SPSCQueue — single-header wait-free queue (reserved for v1.5 Topic backend)
RUN git clone https://github.com/rigtorp/SPSCQueue.git SPSCQueue \
    && git -C SPSCQueue checkout ${SPSCQUEUE_REF} \
    && cmake -S SPSCQueue -B SPSCQueue/build \
    && cmake --build SPSCQueue/build -j"$(nproc)" \
    && cmake --install SPSCQueue/build \
    && rm -rf SPSCQueue


###############################################################################
# Stage 2 — lean build environment: toolchain + apt deps + installed libs
###############################################################################
FROM ubuntu:${UBUNTU_VERSION} AS build-env
ENV DEBIAN_FRONTEND=noninteractive

# Toolchain + apt-provided dependencies (exact set from scripts/install_deps.sh).
# Ubuntu 24.04 LTS ships GCC 13 (full C++20), CMake 3.28, Eigen 3.4,
# spdlog 1.12, yaml-cpp 0.8, fmt 9 — all satisfy the project's find_package mins.
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        libeigen3-dev \
        libspdlog-dev \
        libyaml-cpp-dev \
        libfmt-dev \
    && rm -rf /var/lib/apt/lists/*

# Bring in the from-source libraries built in stage 1 (headers, CMake configs,
# Catch2 static libs). /usr/local is on the default CMake search path.
COPY --from=deps-builder /usr/local/ /usr/local/
RUN ldconfig

WORKDIR /workspace


###############################################################################
# Stage 3 — OPTIONAL in-image verification: configure, build, run ctest
#   docker build --target verify -t simpleslam-ci .
#
# NOTE: this compiles the FULL source tree. The in-process comm module
# (include/.../infra/topic.hpp, topic_hub.hpp + core/src/topic_hub.cpp) may be
# under concurrent edit and can break this build — that is expected and does
# not invalidate the toolchain. See docs/Docker.md.
###############################################################################
FROM build-env AS verify
COPY . /workspace
RUN cmake -B build -DBUILD_TESTING=ON \
    && cmake --build build -j"$(nproc)" \
    && ctest --test-dir build --output-on-failure


###############################################################################
# Stage 4 — DEFAULT lean dev/build image (no source baked in)
#   Intended for bind-mount development (see docker-compose.yml):
#     docker run --rm -it -v "$PWD":/workspace simpleslam-dev
###############################################################################
FROM build-env AS dev
LABEL org.opencontainers.image.title="SimpleSLAM dev/build environment" \
      org.opencontainers.image.description="Ubuntu 24.04 + C++20 toolchain and all SimpleSLAM dependencies" \
      org.opencontainers.image.source="https://github.com/Michael-Jetson/SimpleSLAM"
CMD ["bash"]
