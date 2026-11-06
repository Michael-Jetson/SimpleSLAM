# syntax=docker/dockerfile:1
###############################################################################
# SimpleSLAM 的构建与测试环境：Ubuntu 24.04（GCC 13、CMake 3.28）与全部依赖。
#
# 依赖与 scripts/install_deps.sh 一致：apt 装 Eigen、yaml-cpp、Catch2 与 python3；
# manif 只用头文件，从 GitHub 取固定的版本（它依赖的 tl::optional 随仓库一起分发）。
#
#   docker build -t simpleslam-dev .                    # 开发镜像（默认）
#   docker build --target verify -t simpleslam-ci .     # 构建并跑全部测试
#
# 构建时连不上 GitHub：先在宿主机把 manif 克隆到 docker/deps/manif，再加
#   --build-context manif=docker/deps/manif
# 用这份本地副本代替从 GitHub 取的那份，其余照常。
###############################################################################

ARG UBUNTU_VERSION=24.04
ARG MANIF_REF=0.0.5


###############################################################################
# manif 的头文件：只留 include/ 与 external/，与本地副本的目录结构相同
###############################################################################
FROM ubuntu:${UBUNTU_VERSION} AS manif-fetch
ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends git ca-certificates \
    && rm -rf /var/lib/apt/lists/*
ARG MANIF_REF
RUN git clone --branch ${MANIF_REF} --depth 1 https://github.com/artivis/manif.git /manif

FROM scratch AS manif
COPY --from=manif-fetch /manif/include /include
COPY --from=manif-fetch /manif/external /external


###############################################################################
# 构建环境：工具链、apt 的依赖、manif 的头文件
###############################################################################
FROM ubuntu:${UBUNTU_VERSION} AS build-env
ENV DEBIAN_FRONTEND=noninteractive
# python3 跑 Python 侧的测试与操作台，只用标准库
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        cmake \
        ninja-build \
        catch2 \
        libeigen3-dev \
        libyaml-cpp-dev \
        python3 \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*
COPY --from=manif /include/manif /usr/local/include/manif
COPY --from=manif /external/tl/tl /usr/local/include/tl
WORKDIR /workspace


###############################################################################
# 可选（--target verify）：在镜像里配置、构建、跑全部测试
###############################################################################
FROM build-env AS verify
COPY . /workspace
RUN cmake -B build -DBUILD_TESTING=ON \
    && cmake --build build -j"$(nproc)" \
    && ctest --test-dir build --output-on-failure


###############################################################################
# 默认：开发镜像，不带源码，挂载工作区使用（见 docker-compose.yml）
#   docker run --rm -it -v "$PWD":/workspace simpleslam-dev
###############################################################################
FROM build-env AS dev
LABEL org.opencontainers.image.title="SimpleSLAM dev/build environment" \
      org.opencontainers.image.description="Ubuntu 24.04 + C++20 toolchain and all SimpleSLAM dependencies" \
      org.opencontainers.image.source="https://github.com/Michael-Jetson/SimpleSLAM"
CMD ["bash"]
