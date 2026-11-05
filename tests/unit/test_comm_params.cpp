// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 参数：声明即 schema、声明不覆盖已配置的值、事务提交、校验不在锁内、旧快照不可变。

#include <SimpleSLAM/comm/parameters.hpp>

#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <string>

using namespace simpleslam::comm;

TEST_CASE("未声明的名字不接受写入", "[comm][params]") {
    ParameterStore store;
    const auto result = store.commit({{"/a/iterations", std::int64_t{5}}});
    CHECK(result.status == ParamCommit::kUnknownName);
    CHECK(result.revision == 0);
}

TEST_CASE("声明不推进版本，提交才推进", "[comm][params]") {
    ParameterStore store;
    store.declare<std::int64_t>("/a/iterations", 20);
    store.declare<double>("/a/voxel_size", 1.0);
    CHECK(store.snapshot()->revision == 0);

    const auto r1 = store.commit({{"/a/iterations", std::int64_t{30}}});
    REQUIRE(r1.ok());
    CHECK(r1.revision == 1);
    CHECK(store.snapshot()->get<std::int64_t>("/a/iterations") == 30);
}

TEST_CASE("类型不符整批拒绝", "[comm][params]") {
    ParameterStore store;
    store.declare<std::int64_t>("/a/iterations", 20);
    store.declare<double>("/a/voxel_size", 1.0);

    const auto result = store.commit(
        {{"/a/voxel_size", 2.0}, {"/a/iterations", std::string{"oops"}}});
    CHECK(result.status == ParamCommit::kRejectedType);
    CHECK(store.snapshot()->revision == 0);
    // 整批拒绝：合法的那一项也不得生效
    CHECK(store.snapshot()->get<double>("/a/voxel_size") == 1.0);
}

TEST_CASE("跨字段校验器可以整批否决", "[comm][params]") {
    ParameterStore store;
    store.declare<std::int64_t>("/a/min", 1);
    store.declare<std::int64_t>("/a/max", 10);
    store.setValidator([](const ParamSnapshot& s, std::string& detail) {
        const auto lo = s.get<std::int64_t>("/a/min").value_or(0);
        const auto hi = s.get<std::int64_t>("/a/max").value_or(0);
        if (lo > hi) {
            detail = "min 不得大于 max";
            return false;
        }
        return true;
    });

    const auto bad = store.commit({{"/a/min", std::int64_t{99}}});
    CHECK(bad.status == ParamCommit::kRejectedValue);
    CHECK(bad.detail == "min 不得大于 max");
    CHECK(store.snapshot()->revision == 0);
}

TEST_CASE("校验回调不在存储锁内，回调里读参数不会自锁", "[comm][params]") {
    ParameterStore store;
    store.declare<std::int64_t>("/a/iterations", 20);
    store.setValidator([&store](const ParamSnapshot& candidate, std::string&) {
        return store.snapshot()->revision <= candidate.revision;
    });
    CHECK(store.commit({{"/a/iterations", std::int64_t{30}}}).ok());
    CHECK(store.snapshot()->get<std::int64_t>("/a/iterations") == 30);
}

TEST_CASE("校验期间快照被换了，就基于新快照重来", "[comm][params]") {
    ParameterStore store;
    store.declare<std::int64_t>("/a/iterations", 20);
    store.declare<double>("/a/voxel_size", 1.0);
    int calls = 0;
    store.setValidator([&](const ParamSnapshot&, std::string&) {
        // 第一次校验时插进另一次写入。校验器不再挡它，这一次提交要基于它的结果重来
        if (++calls == 1) {
            store.preset("/a/voxel_size", ParamValue{2.0});
        }
        return true;
    });
    REQUIRE(store.commit({{"/a/iterations", std::int64_t{30}}}).ok());
    CHECK(calls == 2);
    CHECK(store.snapshot()->get<std::int64_t>("/a/iterations") == 30);
    CHECK(store.snapshot()->get<double>("/a/voxel_size") == 2.0);  // 中间那次写入没有被覆盖
}

TEST_CASE("已取到的快照不会被后续提交改动", "[comm][params]") {
    ParameterStore store;
    store.declare<std::int64_t>("/a/iterations", 20);
    const auto old_snapshot = store.snapshot();

    REQUIRE(store.commit({{"/a/iterations", std::int64_t{30}}}).ok());

    CHECK(old_snapshot->revision == 0);
    CHECK(old_snapshot->get<std::int64_t>("/a/iterations") == 20);
    CHECK(store.snapshot()->get<std::int64_t>("/a/iterations") == 30);
}

TEST_CASE("声明不覆盖配置已经给定的值", "[comm][params]") {
    // declare() 若用 insert_or_assign，模块声明默认值时就会把装配期预置的配置值
    // 改回默认值——配置与参数之间因此出现第二套真源。
    ParameterStore store;
    store.preset("/a/iterations", ParamValue{std::int64_t{50}});  // 配置先到

    store.declare<std::int64_t>("/a/iterations", 20);
    CHECK(store.snapshot()->get<std::int64_t>("/a/iterations") == 50);

    // 缺失的名字才按默认值建立
    store.declare<double>("/a/voxel_size", 0.5);
    CHECK(store.snapshot()->get<double>("/a/voxel_size") == 0.5);
    CHECK(store.snapshot()->revision == 0);  // 声明仍然不推进版本
}

TEST_CASE("声明类型与已有值不符是装配期错误", "[comm][params]") {
    ParameterStore store;
    store.preset("/a/iterations", ParamValue{std::string{"twenty"}});
    CHECK_THROWS_AS(store.declare<std::int64_t>("/a/iterations", 20), ParameterError);
}
