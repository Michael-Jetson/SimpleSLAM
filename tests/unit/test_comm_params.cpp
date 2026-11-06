// SPDX-License-Identifier: MIT
// Copyright (c) 2026 SimpleSLAM contributors

/// 参数：声明是常量、装配时定义；名字、类型、范围、写者、运行中改不改都在提交时核对；
/// 事务整批生效；旧快照不可变；收齐几个组件的声明时核对它们彼此一致；插件只在所属模块的
/// 回调里读写。

#include <SimpleSLAM/comm/parameters.hpp>

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

using namespace simpleslam::comm;

namespace {

constexpr Param<double> kGravity{
    .name = "gravity", .fallback = 9.81, .doc = "重力加速度", .min = 9.0, .max = 10.5};
constexpr Param<std::int64_t> kIterations{
    .name = "icp.iterations", .fallback = 20, .doc = "迭代次数", .min = 1, .runtime = true};
constexpr Param<bool> kDeskew{.name = "deskew", .fallback = true, .doc = "去畸变", .runtime = true};
constexpr Param<std::string> kMode{
    .name = "mode", .fallback = "fast", .doc = "运行模式", .runtime = true};

ParamDefinition definitionOf(const ParamUse& use, std::vector<std::string> writers = {}) {
    return ParamDefinition{.name = std::string(use.name),
                           .type = use.type,
                           .fallback = use.fallback(),
                           .doc = std::string(use.doc),
                           .min = use.min,
                           .max = use.max,
                           .runtime = use.runtime,
                           .writers = std::move(writers)};
}

/// 四个参数：迭代次数由 /odometry 写，其余只有外部能改。
ParameterStore& definedStore(ParameterStore& store) {
    store.define({definitionOf(reads(kGravity)), definitionOf(writes(kIterations), {"/odometry"}),
                  definitionOf(reads(kDeskew)), definitionOf(reads(kMode))});
    return store;
}

}  // namespace

TEST_CASE("声明是常量：类型、默认值、范围、运行中改不改都写在里面", "[comm][params]") {
    constexpr auto use = writes(kIterations);
    static_assert(use.type == ParamType::kInt);
    static_assert(use.writes && use.runtime);
    CHECK(std::get<std::int64_t>(use.fallback()) == 20);
    CHECK(std::get<std::string>(reads(kMode).fallback()) == "fast");
    static constexpr std::array kParams{reads(kGravity), writes(kIterations)};
    CHECK(kParams[0].name == "gravity");
}

TEST_CASE("定义时没给值的取默认值，给了的按声明核对；只能定义一次", "[comm][params]") {
    ParameterStore store;
    CHECK_FALSE(store.defined());
    store.define({definitionOf(reads(kGravity)), definitionOf(reads(kMode))},
                 {{"gravity", 9.80}});
    CHECK(store.defined());
    const auto snapshot = store.snapshot();
    CHECK(snapshot->revision == 0);  // 定义不推进版本，提交才推进
    CHECK(snapshot->get<double>("gravity") == 9.80);
    CHECK(snapshot->get<std::string>("mode") == "fast");
    CHECK_THROWS_AS(store.define({}), ParameterError);

    ParameterStore bad;
    CHECK_THROWS_AS(bad.define({definitionOf(reads(kGravity))}, {{"gravity", 11.0}}),
                    ParameterError);  // 越过上限
    ParameterStore unknown;
    CHECK_THROWS_AS(unknown.define({definitionOf(reads(kGravity))}, {{"gravty", 9.8}}),
                    ParameterError);  // 没有定义的名字
}

TEST_CASE("提交核对名字、类型与范围，不合格整批拒绝、版本不变", "[comm][params]") {
    ParameterStore store;
    definedStore(store);

    CHECK(store.commit({{"nope", 1.0}}, kExternalWriter).status == ParamCommit::kUnknownName);
    CHECK(store.commit({{"deskew", 1.0}}, kExternalWriter).status == ParamCommit::kRejectedType);
    const auto range = store.commit({{"gravity", 12.0}}, kExternalWriter);
    CHECK(range.status == ParamCommit::kRejectedValue);
    CHECK(range.detail.find("不能大于") != std::string::npos);
    // 整批拒绝：合法的那一项也不生效
    const auto batch = store.commit({{"deskew", false}, {"gravity", 12.0}}, kExternalWriter);
    CHECK_FALSE(batch.ok());
    CHECK(store.snapshot()->revision == 0);
    CHECK(store.snapshot()->get<bool>("deskew") == true);

    // 整数可以给实数参数
    const auto ok = store.commit({{"gravity", std::int64_t{10}}}, kExternalWriter);
    REQUIRE(ok.ok());
    CHECK(ok.revision == 1);
    CHECK(store.snapshot()->get<double>("gravity") == 10.0);
    CHECK(ok.previous->get<double>("gravity") == 9.81);
}

TEST_CASE("谁能写：声明了写的组件所属的模块；没有组件写的只有外部能改", "[comm][params]") {
    ParameterStore store;
    definedStore(store);
    CHECK(store.commit({{"icp.iterations", std::int64_t{30}}}, "/odometry").ok());
    CHECK(store.commit({{"icp.iterations", std::int64_t{40}}}, kExternalWriter).status
          == ParamCommit::kRejectedWriter);
    CHECK(store.commit({{"icp.iterations", std::int64_t{40}}}, "/backend").status
          == ParamCommit::kRejectedWriter);
    CHECK(store.commit({{"deskew", false}}, "/odometry").status == ParamCommit::kRejectedWriter);
    CHECK(store.commit({{"deskew", false}}, kExternalWriter).ok());
}

TEST_CASE("开始运行之后，声明为运行中不改的参数拒绝写入", "[comm][params]") {
    ParameterStore store;
    definedStore(store);
    CHECK(store.commit({{"gravity", 9.79}}, kExternalWriter).ok());  // 开始之前还能改
    store.freeze();
    CHECK(store.commit({{"gravity", 9.78}}, kExternalWriter).status
          == ParamCommit::kRejectedStatic);
    CHECK(store.commit({{"deskew", false}}, kExternalWriter).ok());  // 运行中可改的照常
}

TEST_CASE("下限写 kPositive 的参数，拒绝时说必须为正", "[comm][params]") {
    constexpr Param<double> kWeight{.name = "weight", .fallback = 1.0, .min = kPositive};
    ParameterStore store;
    store.define({definitionOf(reads(kWeight))});
    const auto zero = store.commit({{"weight", 0.0}}, kExternalWriter);
    CHECK(zero.status == ParamCommit::kRejectedValue);
    CHECK(zero.detail.find("必须为正") != std::string::npos);
}

TEST_CASE("已取到的快照不会被后续提交改动；空提交不推进版本", "[comm][params]") {
    ParameterStore store;
    definedStore(store);
    const auto old_snapshot = store.snapshot();
    REQUIRE(store.commit({{"gravity", 10.0}}, kExternalWriter).ok());
    CHECK(old_snapshot->revision == 0);
    CHECK(old_snapshot->get<double>("gravity") == 9.81);
    const auto empty = store.commit({}, kExternalWriter);
    CHECK(empty.ok());
    CHECK(empty.snapshot == nullptr);
    CHECK(store.snapshot()->revision == 1);
}

TEST_CASE("收齐声明：同名就是共享，类型、默认值、写者都要说得通", "[comm][params]") {
    const ParamComponent lio{.component = "odometry.plugin(lio)",
                             .module = "/odometry",
                             .uses = {reads(kGravity), writes(kIterations)}};
    const ParamComponent backend{.component = "graph.plugin(pgo)",
                                 .module = "/backend",
                                 .uses = {reads(kGravity), reads(kIterations)}};
    const auto resolved = resolveParams({lio, backend}, {{"gravity", 9.79}});
    REQUIRE(resolved.definitions.size() == 2);
    CHECK(std::get<double>(resolved.values.at("gravity")) == 9.79);
    CHECK(std::get<std::int64_t>(resolved.values.at("icp.iterations")) == 20);  // 默认值补齐
    for (const auto& definition : resolved.definitions) {
        if (definition.name == "icp.iterations") {
            CHECK(definition.writers == std::vector<std::string>{"/odometry"});
        }
    }

    // 没人声明的名字报"未知参数"，列出声明过的
    try {
        (void)resolveParams({lio}, {{"gravty", 9.8}});
        FAIL("应当报错");
    } catch (const ParameterError& e) {
        CHECK(std::string(e.what()).find("未知参数 gravty") != std::string::npos);
        CHECK(std::string(e.what()).find("gravity") != std::string::npos);
    }
    // 类型不一致
    constexpr Param<std::int64_t> kGravityInt{.name = "gravity", .fallback = 10};
    CHECK_THROWS_AS(resolveParams({lio, {.component = "x", .module = "/x",
                                         .uses = {reads(kGravityInt)}}},
                                  {}),
                    ParameterError);
    // 默认值不同：要在配置里写明
    constexpr Param<double> kGravityOther{.name = "gravity", .fallback = 9.80, .min = 9.0,
                                          .max = 10.5};
    const ParamComponent other{.component = "y", .module = "/y", .uses = {reads(kGravityOther)}};
    CHECK_THROWS_AS(resolveParams({lio, other}, {}), ParameterError);
    CHECK_NOTHROW(resolveParams({lio, other}, {{"gravity", 9.80}}));
    // 两个组件都写同一个参数
    const ParamComponent second_writer{.component = "z", .module = "/z",
                                       .uses = {writes(kIterations)}};
    CHECK_THROWS_AS(resolveParams({lio, second_writer}, {}), ParameterError);
    // 运行中不改的参数不能声明写
    const ParamComponent static_writer{.component = "w", .module = "/w",
                                       .uses = {writes(kGravity)}};
    CHECK_THROWS_AS(resolveParams({static_writer}, {}), ParameterError);
    // 值越界
    CHECK_THROWS_AS(resolveParams({lio}, {{"gravity", 20.0}}), ParameterError);
}

TEST_CASE("插件读写参数：只在作用域里读，写先记下", "[comm][params]") {
    // 没有作用域：构造函数里、自己开的线程里读，明确报错，不给默认值
    CHECK_THROWS_AS(readParam(kGravity), std::logic_error);

    auto snapshot = std::make_shared<ParamSnapshot>();
    snapshot->values = {{"gravity", 9.79}};
    std::vector<ParamChange> writes;
    {
        const ParamScope scope(snapshot, &writes);
        CHECK(readParam(kGravity) == 9.79);
        CHECK_THROWS_AS(readParam(kDeskew), std::logic_error);  // 没有定义
        writeParam(kIterations, std::int64_t{7});
        {
            const ParamScope inner(std::make_shared<ParamSnapshot>());  // 内层优先，不能写
            CHECK_THROWS_AS(readParam(kGravity), std::logic_error);
            CHECK_THROWS_AS(writeParam(kIterations, std::int64_t{8}), std::logic_error);
        }
        CHECK(readParam(kGravity) == 9.79);  // 出了内层，还原成外层
    }
    CHECK(ParamScope::current() == nullptr);
    REQUIRE(writes.size() == 1);
    CHECK(writes[0].name == "icp.iterations");
    CHECK(std::get<std::int64_t>(writes[0].value) == 7);
}
