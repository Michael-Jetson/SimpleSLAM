// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 名称解析：绝对名 / 相对名 / 私有名，以及解析结果的合法性。

#include <SimpleSLAM/comm/types.hpp>

#include <catch2/catch_test_macros.hpp>

using simpleslam::comm::isValidResolvedName;
using simpleslam::comm::resolveName;

TEST_CASE("绝对名原样保留", "[comm][names]") {
    CHECK(resolveName("/lidar/points", "/slam/odometry") == "/lidar/points");
    CHECK(resolveName("/a//b/", "/slam/odometry") == "/a/b");
}

TEST_CASE("相对名解析到模块所在命名空间", "[comm][names]") {
    CHECK(resolveName("pose", "/slam/odometry") == "/slam/pose");
    CHECK(resolveName("pose", "/odometry") == "/pose");
}

TEST_CASE("私有名解析到模块自身", "[comm][names]") {
    CHECK(resolveName("~voxel_size", "/slam/odometry") == "/slam/odometry/voxel_size");
    CHECK(resolveName("~/voxel_size", "/slam/odometry") == "/slam/odometry/voxel_size");
    CHECK(resolveName("~", "/slam/odometry") == "/slam/odometry");
}

TEST_CASE("解析结果的合法性检查", "[comm][names]") {
    CHECK(isValidResolvedName("/slam/pose"));
    CHECK(isValidResolvedName("/a"));
    CHECK_FALSE(isValidResolvedName(""));
    CHECK_FALSE(isValidResolvedName("pose"));       // 非绝对名
    CHECK_FALSE(isValidResolvedName("/slam/"));     // 末尾斜杠
    CHECK_FALSE(isValidResolvedName("/slam//pose"));// 空段
    CHECK_FALSE(isValidResolvedName("/slam/po se"));// 非法字符
}
