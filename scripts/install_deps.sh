#!/bin/bash
# SimpleSLAM v0 依赖安装脚本（Ubuntu 22.04/24.04 原生构建）
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

echo "=== 全部依赖安装完成 ==="
