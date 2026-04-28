#!/bin/bash
# SimpleSLAM v1.0 依赖安装脚本（Ubuntu 22.04/24.04 原生构建）
set -e

echo "=== 安装系统包 ==="
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake git \
    libeigen3-dev libspdlog-dev libyaml-cpp-dev libfmt-dev

echo "=== 安装 Catch2 3.x ==="
if ! cmake --find-package -DNAME=Catch2 -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST 2>/dev/null; then
    git clone --branch v3.4.0 --depth 1 https://github.com/catchorg/Catch2.git /tmp/catch2
    cmake -S /tmp/catch2 -B /tmp/catch2/build -DCMAKE_CXX_STANDARD=20 -DBUILD_TESTING=OFF
    cmake --build /tmp/catch2/build -j$(nproc)
    sudo cmake --install /tmp/catch2/build
    rm -rf /tmp/catch2
    echo "Catch2 3.x 安装完成"
else
    echo "Catch2 已安装，跳过"
fi

echo "=== 安装 manif ==="
if ! cmake --find-package -DNAME=manif -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST 2>/dev/null; then
    git clone --branch master --depth 1 https://github.com/artivis/manif.git /tmp/manif
    cmake -S /tmp/manif -B /tmp/manif/build -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
    cmake --build /tmp/manif/build -j$(nproc)
    sudo cmake --install /tmp/manif/build
    rm -rf /tmp/manif
    echo "manif 安装完成"
else
    echo "manif 已安装，跳过"
fi

echo "=== 安装 nanoflann ==="
if ! cmake --find-package -DNAME=nanoflann -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST 2>/dev/null; then
    git clone --branch v1.6.1 --depth 1 https://github.com/jlblancoc/nanoflann.git /tmp/nanoflann
    cmake -S /tmp/nanoflann -B /tmp/nanoflann/build -DNANOFLANN_BUILD_EXAMPLES=OFF -DNANOFLANN_BUILD_TESTS=OFF
    cmake --build /tmp/nanoflann/build -j$(nproc)
    sudo cmake --install /tmp/nanoflann/build
    rm -rf /tmp/nanoflann
    echo "nanoflann 安装完成"
else
    echo "nanoflann 已安装，跳过"
fi

echo "=== 安装 tsl::robin_map ==="
if ! cmake --find-package -DNAME=tsl-robin-map -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST 2>/dev/null; then
    git clone --branch v1.3.0 --depth 1 https://github.com/Tessil/robin-map.git /tmp/robin-map
    cmake -S /tmp/robin-map -B /tmp/robin-map/build
    cmake --build /tmp/robin-map/build -j$(nproc)
    sudo cmake --install /tmp/robin-map/build
    rm -rf /tmp/robin-map
    echo "tsl::robin_map 安装完成"
else
    echo "tsl::robin_map 已安装，跳过"
fi

echo "=== 安装 eventpp ==="
if ! cmake --find-package -DNAME=eventpp -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST 2>/dev/null; then
    git clone --depth 1 https://github.com/wqking/eventpp.git /tmp/eventpp
    cmake -S /tmp/eventpp -B /tmp/eventpp/build -DEVENTPP_INSTALL=ON
    cmake --build /tmp/eventpp/build -j$(nproc)
    sudo cmake --install /tmp/eventpp/build
    rm -rf /tmp/eventpp
    echo "eventpp 安装完成"
else
    echo "eventpp 已安装，跳过"
fi

echo "=== 安装 SPSCQueue ==="
if ! cmake --find-package -DNAME=SPSCQueue -DCOMPILER_ID=GNU -DLANGUAGE=CXX -DMODE=EXIST 2>/dev/null; then
    git clone --depth 1 https://github.com/rigtorp/SPSCQueue.git /tmp/SPSCQueue
    cmake -S /tmp/SPSCQueue -B /tmp/SPSCQueue/build
    cmake --build /tmp/SPSCQueue/build -j$(nproc)
    sudo cmake --install /tmp/SPSCQueue/build
    rm -rf /tmp/SPSCQueue
    echo "SPSCQueue 安装完成"
else
    echo "SPSCQueue 已安装，跳过"
fi

echo "=== 全部依赖安装完成 ==="
